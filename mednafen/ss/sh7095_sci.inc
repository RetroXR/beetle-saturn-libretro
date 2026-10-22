/* SH7095 (SH7604) serial communication interface -- the port a Saturn Link
 * Cable is wired to. See sci_link.h for what sits on the other end.
 *
 * Included into sh7095.inc, which is included into ss.c, so it shares the
 * CPU[] array and the static helpers around it.
 *
 * Modelled a byte at a time, not a bit at a time. The shift registers exist
 * and take a frame's worth of cycles to empty and fill, which is what a game
 * can see; the individual edges on TxD and SCK are not, and nothing reads
 * them. What that gives up is a peer changing the baud rate MID-byte, which
 * no protocol does on purpose.
 *
 * Timing rides on the CPU's own FRT/WDT deadline (FRT_WDT_NextTS), which every
 * run loop -- interpreter, cached interpreter and JIT -- already honours, so a
 * transfer completes on the cycle it is due however the CPU is being run. A
 * scheduler event of its own would have changed the savestate layout.
 */

#include "sci_link.h"

enum
{
 SH7095_SSR_TDRE = 0x80,
 SH7095_SSR_RDRF = 0x40,
 SH7095_SSR_ORER = 0x20,
 SH7095_SSR_FER  = 0x10,
 SH7095_SSR_PER  = 0x08,
 SH7095_SSR_TEND = 0x04,
 SH7095_SSR_MPB  = 0x02,
 SH7095_SSR_MPBT = 0x01,

 SH7095_SCR_TIE  = 0x80,
 SH7095_SCR_RIE  = 0x40,
 SH7095_SCR_TE   = 0x20,
 SH7095_SCR_RE   = 0x10,
 SH7095_SCR_TEIE = 0x04,
 SH7095_SCR_CKE1 = 0x02,

 SH7095_SMR_CA   = 0x80,
 SH7095_SMR_CHR  = 0x40,
 SH7095_SMR_PE   = 0x20,
 SH7095_SMR_STOP = 0x08
};

enum
{
 SH7095_SCI_TX_IDLE = 0,
 /* TSR is going out on this end's own clock; done at tx_end. */
 SH7095_SCI_TX_SHIFTING,
 /* Clocked mode, external clock: TSR is loaded and waits for the peer's SCK. */
 SH7095_SCI_TX_ARMED,
 /* Clocked mode, internal clock, receiving only: the clock runs with TxD idle,
  * which is how a clocked receiver asks the far end for its byte. */
 SH7095_SCI_TX_CLOCKING_RX
};

static bool sci_enabled;
static const ss_sci_driver *sci_drv;

static void SH7095_DMA_StartSG(SH7095* z);

static INLINE unsigned SH7095_SCI_Index(const SH7095* z)
{
 return (z == &CPU[0]) ? 0 : 1;
}

static INLINE bool SH7095_SCI_Sync(const SH7095* z)
{
 return (z->SCI.SMR & SH7095_SMR_CA) != 0;
}

static INLINE bool SH7095_SCI_ExternalClock(const SH7095* z)
{
 return (z->SCI.SCR & SH7095_SCR_CKE1) != 0;
}

/* One bit's time in CPU cycles. The SH7604 manual's BRR formulas are
 *   async  B = Pphi / (64 * 2^(2n-1) * (N+1))
 *   sync   B = Pphi / ( 8 * 2^(2n-1) * (N+1))
 * with n = SMR.CKS; the CPU's timestamp counts Pphi. */
static INLINE uint32_t SH7095_SCI_BitCycles(const SH7095* z)
{
 const unsigned n = z->SCI.SMR & 0x3;
 const uint32_t N1 = (uint32_t)z->SCI.BRR + 1;

 return ((SH7095_SCI_Sync(z) ? 4u : 32u) << (n << 1)) * N1;
}

static INLINE unsigned SH7095_SCI_FrameBits(const SH7095* z)
{
 const uint8_t smr = z->SCI.SMR;

 if(smr & SH7095_SMR_CA)
  return 8;

 return 1 + ((smr & SH7095_SMR_CHR) ? 7 : 8) + ((smr & SH7095_SMR_PE) ? 1 : 0) + ((smr & SH7095_SMR_STOP) ? 2 : 1);
}

static INLINE int32_t SH7095_SCI_FrameCycles(const SH7095* z)
{
 return (int32_t)(SH7095_SCI_BitCycles(z) * SH7095_SCI_FrameBits(z));
}

static void SH7095_SCI_Format(const SH7095* z, ss_sci_format* fmt)
{
 fmt->sync = SH7095_SCI_Sync(z);
 fmt->frame = z->SCI.SMR & 0x78;
 fmt->bit_master = SH7095_SCI_BitCycles(z) * (uint32_t)SS_ClockDiv();
}

static INLINE void SH7095_SCI_SetFlags(SH7095* z, uint8_t bits)
{
 z->SCI.SSR |= bits;
 z->SCI.SSRM |= bits;
}

/* ── the SCI as a DMA request source ────────────────────────────────────────
 *
 * DRCR0/1 let a DMA channel be paced by the SCI instead of running flat out:
 * 01 is RXI (a byte has arrived), 10 is TXI (TDR is free). Stock Mednafen
 * stored DRCR and ignored it, running every channel as an auto-request one --
 * and it masked every DMA address to the external bus, so a transfer aimed at
 * TDR (FFFFFE03) landed in external memory and never reached the port. Daytona
 * USA Circuit Edition hands its link packets to DMA channel 0 on TXI straight
 * after its handshake, which is why the handshake completed and then nothing
 * else ever crossed the cable. */
enum
{
 SH7095_DRCR_RXI = 1,
 SH7095_DRCR_TXI = 2
};

static bool sci_in_dma;

/* The SCI source channel `ch` is paced by, or 0 when it is not. Channels on
 * auto-request (CHCR.AR) and on the external DREQ keep Mednafen's behaviour. */
static INLINE unsigned SH7095_SCI_DMASource(const SH7095* z, unsigned ch)
{
 const unsigned drcr = z->DMACH[ch].DRCR & 3;

 if(!sci_enabled || (z->DMACH[ch].CHCR & 0x200))
  return 0;
 return (drcr == SH7095_DRCR_RXI || drcr == SH7095_DRCR_TXI) ? drcr : 0;
}

/* Whether the SCI is asking channel `ch` for a transfer right now; true for a
 * channel the SCI does not pace. */
static INLINE bool SH7095_SCI_DMARequest(const SH7095* z, unsigned ch)
{
 switch(SH7095_SCI_DMASource(z, ch))
 {
  case SH7095_DRCR_RXI:
	return (z->SCI.SCR & SH7095_SCR_RIE) && (z->SCI.SSR & SH7095_SSR_RDRF);
  case SH7095_DRCR_TXI:
	return (z->SCI.SCR & (SH7095_SCR_TIE | SH7095_SCR_TE)) == (SH7095_SCR_TIE | SH7095_SCR_TE) && (z->SCI.SSR & SH7095_SSR_TDRE);
 }
 return true;
}

/* Interrupt sources an enabled DMA channel has claimed: the DMAC answers them,
 * so the CPU is not interrupted as well. Bit 0 RXI, bit 1 TXI. */
static INLINE unsigned SH7095_SCI_DMAClaimed(const SH7095* z)
{
 unsigned claimed = 0;

 if((z->DMAOR & 0x07) != 0x01)
  return 0;
 for(unsigned ch = 0; ch < 2; ch++)
 {
  if((z->DMACH[ch].CHCR & 0x03) != 0x01)
   continue;
  switch(SH7095_SCI_DMASource(z, ch))
  {
   case SH7095_DRCR_RXI: claimed |= 1; break;
   case SH7095_DRCR_TXI: claimed |= 2; break;
  }
 }
 return claimed;
}

/* A request may have just appeared: let a waiting channel run. */
static void SH7095_SCI_DMAKick(SH7095* z)
{
 if(sci_in_dma || !SH7095_SCI_DMAClaimed(z) || z->timestamp == SS_EVENT_DISABLED_TS)
  return;

 SH7095_DMA_Update(z, SH7095_mem_timestamp);
 SH7095_DMA_RecalcRunning(z);
 SH7095_DMA_StartSG(z);
}

/* SS_SCI_ERRLOG=1: say so whenever a byte is lost -- an overrun, a framing
 * error, or a byte arriving at a receiver that is switched off. */
static int sci_errlog = -1;
static unsigned sci_errlog_n;

static void SH7095_SCI_LogLoss(const SH7095* z, const char* what, uint8_t value)
{
 if(sci_errlog < 0)
  sci_errlog = getenv("SS_SCI_ERRLOG") != NULL;
 if(!sci_errlog || sci_errlog_n >= 400)
  return;
 sci_errlog_n++;
 if(log_cb)
  log_cb(RETRO_LOG_WARN, "[scierr] %04X cpu%u %s value=%02X smr=%02X brr=%02X scr=%02X ssr=%02X pc=%08X ts=%d\n",
   (unsigned)((uintptr_t)CPU >> 4) & 0xFFFF, SH7095_SCI_Index(z), what, value, z->SCI.SMR, z->SCI.BRR, z->SCI.SCR, z->SCI.SSR, (unsigned)z->PC, (int)z->timestamp);
}

static void SH7095_SCI_RxByte(SH7095* z, uint8_t value, bool framing_error)
{
 SCITRACE("rx", 5, value);

 if(!(z->SCI.SCR & SH7095_SCR_RE))
 {
  SH7095_SCI_LogLoss(z, "receiver-off", value);
  return;
 }

 if(z->SCI.SSR & SH7095_SSR_RDRF)
 {
  SH7095_SCI_LogLoss(z, "overrun", value);
  SH7095_SCI_SetFlags(z, SH7095_SSR_ORER);	// RDR keeps the byte the game has not read
 }
 else
 {
  z->SCI.RDR = value;
  SH7095_SCI_SetFlags(z, SH7095_SSR_RDRF);
 }

 if(framing_error)
 {
  SH7095_SCI_LogLoss(z, "framing", value);
  SH7095_SCI_SetFlags(z, SH7095_SSR_FER);
 }
}

/* Put the byte now in the shift register on the wire, stamped for tx_end.
 *
 * Announced when it STARTS shifting, not when it finishes, because that is
 * the lookahead a real wire has: from the first bit the value is fixed and
 * the moment it lands is a frame-time away. Announcing at the end instead
 * left the stamp to the bus's commit horizon, which added a horizon of delay
 * to every byte -- and GunGriffon II waits only a couple of byte-times for
 * its partner's reply before calling it a communication error. What this
 * gives up is a TE cleared mid-byte, which real code does only to abandon a
 * transfer it no longer cares about. */
static void SH7095_SCI_Announce(SH7095* z, uint8_t value)
{
 ss_sci_format fmt;

 if(!sci_drv)
  return;

 SH7095_SCI_Format(z, &fmt);
 sci_drv->tx(SH7095_SCI_Index(z), value, &fmt, z->SCI.tx_end);
}

/* Start whatever the port can start at `now`: the next byte out of TDR, or a
 * receive-only clock. */
static void SH7095_SCI_TryStart(SH7095* z, sscpu_timestamp_t now)
{
 if(z->SCI.tx_state != SH7095_SCI_TX_IDLE)
  return;

 if(!(z->SCI.SCR & SH7095_SCR_TE))
 {
  if(SH7095_SCI_Sync(z) && !SH7095_SCI_ExternalClock(z) && (z->SCI.SCR & SH7095_SCR_RE) && !(z->SCI.SSR & SH7095_SSR_RDRF))
  {
   z->SCI.tx_state = SH7095_SCI_TX_CLOCKING_RX;
   z->SCI.tx_end = now + SH7095_SCI_FrameCycles(z);
   SH7095_SCI_Announce(z, 0xFF);	// the clock runs with TxD idle
  }
  return;
 }

 if(z->SCI.SSR & SH7095_SSR_TDRE)
  return;

 z->SCI.TSR = z->SCI.TDR;
 SH7095_SCI_SetFlags(z, SH7095_SSR_TDRE);
 z->SCI.SSR &= ~SH7095_SSR_TEND;

 if(SH7095_SCI_Sync(z) && SH7095_SCI_ExternalClock(z))
 {
  z->SCI.tx_state = SH7095_SCI_TX_ARMED;
  if(sci_drv)
   sci_drv->hold(SH7095_SCI_Index(z), z->SCI.TSR, true, now);
 }
 else
 {
  z->SCI.tx_state = SH7095_SCI_TX_SHIFTING;
  z->SCI.tx_end = now + SH7095_SCI_FrameCycles(z);
  SH7095_SCI_Announce(z, z->SCI.TSR);
 }
}

/* The frame in the shift registers has finished, at tx_end. */
static void SH7095_SCI_Complete(SH7095* z)
{
 const sscpu_timestamp_t at = z->SCI.tx_end;
 const unsigned idx = SH7095_SCI_Index(z);
 const bool sending = (z->SCI.tx_state == SH7095_SCI_TX_SHIFTING);

 // Already on the wire: SH7095_SCI_Announce sent it when it started.
 z->SCI.tx_state = SH7095_SCI_TX_IDLE;

 if(SH7095_SCI_Sync(z))
 {
  // This end drove SCK, so the same eight edges shifted the peer's TSR into
  // ours. With nothing on the far end the line idles high.
  uint8_t in = 0xFF;

  if(sci_drv && !sci_drv->sample(idx, &in, at))
   in = 0xFF;

  SH7095_SCI_RxByte(z, in, false);
 }

 if(sending && (z->SCI.SSR & SH7095_SSR_TDRE))
  z->SCI.SSR |= SH7095_SSR_TEND;

 SH7095_SCI_TryStart(z, at);
}

static void SH7095_SCI_Update(SH7095* z)
{
 if(!sci_enabled)
  return;

 while((z->SCI.tx_state == SH7095_SCI_TX_SHIFTING || z->SCI.tx_state == SH7095_SCI_TX_CLOCKING_RX) && z->timestamp >= z->SCI.tx_end)
 {
  SH7095_SCI_Complete(z);
  SH7095_RecalcPendingIntPEX(z);
  SH7095_SCI_DMAKick(z);
 }

 if(sci_drv && z == &CPU[0] && z->timestamp >= z->SCI.poll_ts)
 {
  int32_t next = sci_drv->poll(z->timestamp);

  if(next < 1)
   next = 1;
  z->SCI.poll_ts = z->timestamp + next;
 }
}

/* Cycles until the SCI next needs the CPU's attention, or `rt` if sooner. */
static INLINE int32_t SH7095_SCI_NextRT(const SH7095* z, int32_t rt)
{
 if(!sci_enabled)
  return rt;

 if(z->SCI.tx_state == SH7095_SCI_TX_SHIFTING || z->SCI.tx_state == SH7095_SCI_TX_CLOCKING_RX)
 {
  int32_t d = z->SCI.tx_end - z->timestamp;

  if(d < 1)
   d = 1;
  if(d < rt)
   rt = d;
 }

 if(sci_drv && z == &CPU[0])
 {
  int32_t d = z->SCI.poll_ts - z->timestamp;

  if(d < 1)
   d = 1;
  if(d < rt)
   rt = d;
 }

 return rt;
}

static INLINE void SH7095_SCI_Kick(SH7095* z)
{
 if(z->timestamp == SS_EVENT_DISABLED_TS)
  return;

 // Recalc_NET reads the FRT as of its last update, so bring it up to now.
 SH7095_FRT_WDT_Update(z);
 SH7095_FRT_WDT_Recalc_NET(z);
}

static void SH7095_SCI_WriteReg(SH7095* z, unsigned A, uint8_t V)
{
 SH7095_SCI_Update(z);

 switch(A)
 {
  case 0x00:
	z->SCI.SMR = V;
	break;

  case 0x01:
	z->SCI.BRR = V;
	break;

  case 0x02:
	{
	 const uint8_t old = z->SCI.SCR;

	 z->SCI.SCR = V;

	 if((old & SH7095_SCR_TE) && !(V & SH7095_SCR_TE))
	 {
	  // Clearing TE abandons whatever was in TSR and leaves the transmitter
	  // reporting empty, which is where it starts after a reset.
	  if(z->SCI.tx_state == SH7095_SCI_TX_ARMED && sci_drv)
	   sci_drv->hold(SH7095_SCI_Index(z), z->SCI.TSR, false, z->timestamp);
	  if(z->SCI.tx_state != SH7095_SCI_TX_CLOCKING_RX)
	   z->SCI.tx_state = SH7095_SCI_TX_IDLE;
	  z->SCI.SSR |= SH7095_SSR_TDRE | SH7095_SSR_TEND;
	 }

	 if(!(V & SH7095_SCR_RE) && z->SCI.tx_state == SH7095_SCI_TX_CLOCKING_RX)
	  z->SCI.tx_state = SH7095_SCI_TX_IDLE;
	}
	break;

  case 0x03:
	z->SCI.TDR = V;
	break;

  case 0x04:
	{
	 // TDRE, RDRF, ORER, FER and PER clear on writing 0 -- but only once the
	 // game has READ them as 1, which is what SSRM tracks, the same as FTCSR.
	 const uint8_t was = z->SCI.SSR;
	 uint8_t ssr = (was & (z->SCI.SSRM | V) & 0xF8) | (was & 0x06) | (V & SH7095_SSR_MPBT);

	 // With the transmitter off TDRE is held at 1: clearing it then does not
	 // queue TDR for when TE comes on. GunGriffon II clears the whole SSR with
	 // the port off and enables it later; reading that as "send TDR" put a
	 // stale 0xFF on the wire, which the other console took for its partner's
	 // reply, and the battle ended in a communication error.
	 if(!(z->SCI.SCR & SH7095_SCR_TE))
	  ssr |= SH7095_SSR_TDRE;

	 if((was & SH7095_SSR_TDRE) && !(ssr & SH7095_SSR_TDRE))
	  ssr &= ~SH7095_SSR_TEND;

	 z->SCI.SSR = ssr;
	}
	break;

  case 0x05:	// RDR is read-only
	break;
 }

 SH7095_SCI_TryStart(z, z->timestamp);
 SH7095_RecalcPendingIntPEX(z);
 SH7095_SCI_Kick(z);
 SH7095_SCI_DMAKick(z);
}

static uint8_t SH7095_SCI_ReadReg_(SH7095* z, unsigned A);

static uint8_t SH7095_SCI_ReadReg(SH7095* z, unsigned A)
{
 const uint8_t ret = SH7095_SCI_ReadReg_(z, A);

 SCITRACE("r8", A, ret);
 return ret;
}

static uint8_t SH7095_SCI_ReadReg_(SH7095* z, unsigned A)
{
 switch(A)
 {
  case 0x00: return z->SCI.SMR;
  case 0x01: return z->SCI.BRR;
  case 0x02: return z->SCI.SCR;
  case 0x03: return z->SCI.TDR;

  case 0x04:
	{
	 uint8_t ret;

	 if(sci_drv)
	  sci_drv->sync(z->timestamp);
	 SH7095_SCI_Update(z);
	 ret = z->SCI.SSR;
	 z->SCI.SSRM = 0x00;
	 return ret;
	}

  case 0x05:
	if(sci_drv)
	 sci_drv->sync(z->timestamp);
	return z->SCI.RDR;
 }

 return 0;
}

/* The DMAC reaching an SCI register. Writing TDR clears TDRE and reading RDR
 * clears RDRF, which is what makes each request good for exactly one byte. */
static void SH7095_SCI_DMAWrite(SH7095* z, unsigned A, uint8_t V)
{
 sci_in_dma = true;
 if(A == 0x03)
 {
  SH7095_SCI_Update(z);
  z->SCI.TDR = V;
  if(z->SCI.SCR & SH7095_SCR_TE)
  {
   z->SCI.SSR &= ~(SH7095_SSR_TDRE | SH7095_SSR_TEND);
   SH7095_SCI_TryStart(z, z->timestamp);
  }
  SH7095_RecalcPendingIntPEX(z);
  SH7095_SCI_Kick(z);
 }
 else
  SH7095_SCI_WriteReg(z, A, V);
 sci_in_dma = false;
}

static uint8_t SH7095_SCI_DMARead(SH7095* z, unsigned A)
{
 uint8_t ret;

 sci_in_dma = true;
 ret = SH7095_SCI_ReadReg(z, A);
 if(A == 0x05)
 {
  z->SCI.SSR &= ~SH7095_SSR_RDRF;
  SH7095_SCI_TryStart(z, z->timestamp);
  SH7095_RecalcPendingIntPEX(z);
 }
 sci_in_dma = false;
 return ret;
}

/* Pending SCI interrupt for GetPendingInt: sets *vecnum and returns true when
 * one is asserted. Receive error outranks receive, which outranks transmit
 * empty, which outranks transmit end -- the manual's order. A source a DMA
 * channel is answering does not interrupt the CPU as well. */
static INLINE bool SH7095_SCI_PendingInt(const SH7095* z, unsigned* vecnum)
{
 const uint8_t ssr = z->SCI.SSR;
 const uint8_t scr = z->SCI.SCR;
 unsigned claimed;

 if(!sci_enabled)
  return false;

 claimed = SH7095_SCI_DMAClaimed(z);

 if((scr & SH7095_SCR_RIE) && (ssr & (SH7095_SSR_ORER | SH7095_SSR_FER | SH7095_SSR_PER)))
  *vecnum = (z->VCRA >> 8) & 0x7F;
 else if((scr & SH7095_SCR_RIE) && (ssr & SH7095_SSR_RDRF) && !(claimed & 1))
  *vecnum = z->VCRA & 0x7F;
 else if((scr & SH7095_SCR_TIE) && (ssr & SH7095_SSR_TDRE) && !(claimed & 2))
  *vecnum = (z->VCRB >> 8) & 0x7F;
 else if((scr & SH7095_SCR_TEIE) && (ssr & SH7095_SSR_TEND))
  *vecnum = z->VCRB & 0x7F;
 else
  return false;

 return true;
}

/* ── the interface in sci_link.h ─────────────────────────────────────────── */

void SS_SCI_SetDriver(const ss_sci_driver *drv)
{
 sci_drv = drv;
 SS_SCI_KickPoll();
}

void SS_SCI_SetEnabled(bool enabled)
{
 sci_enabled = enabled;
}

bool SS_SCI_Enabled(void)
{
 return sci_enabled;
}

void SS_SCI_KickPoll(void)
{
 SH7095* z = &CPU[0];

 if(z->timestamp == SS_EVENT_DISABLED_TS)
  return;

 z->SCI.poll_ts = z->timestamp;
 SH7095_FRT_WDT_Update(z);
 SH7095_FRT_WDT_Recalc_NET(z);
}

uint32_t SS_SCI_FastestFrameMaster(unsigned cpu_mask)
{
 uint32_t best = 0;

 if(!sci_enabled)
  return 0;

 for(unsigned i = 0; i < 2; i++)
 {
  const SH7095* z = &CPU[i];
  uint32_t f;

  if(!((cpu_mask >> i) & 1) || z->timestamp == SS_EVENT_DISABLED_TS || !(z->SCI.SCR & (SH7095_SCR_TE | SH7095_SCR_RE)))
   continue;
  f = (uint32_t)SH7095_SCI_FrameCycles(z) * (uint32_t)SS_ClockDiv();
  if(!best || f < best)
   best = f;
 }

 return best;
}

int32_t SS_SCI_CPUTimestamp(unsigned cpu)
{
 return CPU[cpu & 1].timestamp;
}

static void SH7095_SCI_ExtClock(SH7095* z, uint8_t value, bool hold_seen)
{
 SH7095_SCI_RxByte(z, value, false);

 if(z->SCI.tx_state == SH7095_SCI_TX_ARMED && hold_seen)
 {
  z->SCI.tx_state = SH7095_SCI_TX_IDLE;
  if(z->SCI.SSR & SH7095_SSR_TDRE)
   z->SCI.SSR |= SH7095_SSR_TEND;
  SH7095_SCI_TryStart(z, z->timestamp);
 }

 SH7095_RecalcPendingIntPEX(z);
 SH7095_SCI_Kick(z);
 SH7095_SCI_DMAKick(z);
}

void SS_SCI_Receive(unsigned cpu, uint8_t value, const ss_sci_format *fmt, bool hold_seen)
{
 SH7095* z = &CPU[cpu & 1];
 bool framing_error = false;

 if(!sci_enabled || z->timestamp == SS_EVENT_DISABLED_TS)
  return;

 SH7095_SCI_Update(z);

 if(fmt)
 {
  // A receiver framing a different shape, or at a rate more than a few
  // percent away, samples garbage and says so.
  ss_sci_format mine;
  uint32_t a, b;

  SH7095_SCI_Format(z, &mine);
  a = mine.bit_master;
  b = fmt->bit_master;
  if(mine.sync != fmt->sync || mine.frame != fmt->frame || (a > b ? a - b : b - a) * 32 > (a > b ? a : b))
   framing_error = true;
 }

 if(SH7095_SCI_Sync(z) && !framing_error)
 {
  // Only the end whose clock is external is clocked by the far end's byte.
  if(SH7095_SCI_ExternalClock(z))
  {
   SH7095_SCI_ExtClock(z, value, hold_seen);
   return;
  }
 }

 SH7095_SCI_RxByte(z, value, framing_error);
 SH7095_RecalcPendingIntPEX(z);
 SH7095_SCI_DMAKick(z);
}
