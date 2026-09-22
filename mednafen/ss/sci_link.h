#ifndef __MDFN_SS_SCI_LINK_H
#define __MDFN_SS_SCI_LINK_H

/* The Saturn's rear Communication Connector, as the SH-2s see it.
 *
 * The connector carries the serial port (SCI) of BOTH SH-2s -- TxD, RxD and
 * SCK of the master and again of the slave -- plus the SCSP's MIDI pair. A
 * Link Cable joins two consoles through it. Stock Mednafen never emulated the
 * SCI at all: its six registers read as zero and every write was dropped,
 * which is also what a game sees here with the setting off.
 *
 * What lives in sh7095_sci.inc is the port itself: the transmit and receive
 * shift registers, the status flags, the bit timing from SMR/BRR and the four
 * interrupts. It knows nothing about cables. Whatever is on the other end of
 * the wire is a driver installed through SS_SCI_SetDriver; with none
 * installed a transmitted byte goes nowhere and a clocked receive reads the
 * idle-high line, 0xFF, which is a console with nothing in its socket.
 *
 * Every timestamp crossing this interface is the calling CPU's own
 * sscpu_timestamp_t, rebased each frame like the rest of the emulator's. The
 * driver converts to its own monotonic clock with SS_LinkClock. */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* How a byte left the port, so the far end can tell whether it framed the
 * same way. Two consoles on different baud rates or formats do not exchange
 * bytes, they exchange framing errors, and a game probing for a partner
 * depends on seeing that. */
typedef struct ss_sci_format
{
   bool     sync;          /* clocked synchronous (SMR.C/A) */
   uint8_t  frame;         /* SMR with the bits that shape a frame: CHR, PE, O/E, STOP */
   uint32_t bit_master;    /* one bit's time, in master-clock cycles */
} ss_sci_format;

typedef struct ss_sci_driver
{
   /* A byte has started leaving `cpu`'s TxD and finishes arriving at `ts`,
    * which is a frame-time from now -- the wire's own lookahead.
    *
    * In asynchronous mode this is the whole story: it lands in the peer's RxD.
    * In clocked mode this end drove SCK, so the same moment also clocked the
    * peer's transmit shift register out; `sample` asks the driver what it was. */
   void (*tx)(unsigned cpu, uint8_t value, const ss_sci_format *fmt, int32_t ts);

   /* Clocked mode, this end drove SCK: the byte the peer's TxD carried during
    * the transfer that ended at `ts`. False when the peer had nothing loaded,
    * in which case the line idled high. */
   bool (*sample)(unsigned cpu, uint8_t *value, int32_t ts);

   /* Clocked mode with an EXTERNAL clock: this end has loaded `value` into its
    * transmit shift register (armed) or taken it back (not armed), and waits
    * for the peer to clock it out. */
   void (*hold)(unsigned cpu, uint8_t value, bool armed, int32_t ts);

   /* A register read. Lets a byte already off the wire land at the cycle it
    * was due rather than at the next rendezvous. Must not block. */
   void (*sync)(int32_t ts);

   /* Master SH-2 only, at the deadline it asked for last time: meet the other
    * console. Returns cycles until it wants calling again (> 0). */
   int32_t (*poll)(int32_t ts);
} ss_sci_driver;

/* Installed by the frontend glue; NULL detaches. */
void SS_SCI_SetDriver(const ss_sci_driver *drv);

/* Off: the port is stock Mednafen's -- registers read zero, writes vanish. */
void SS_SCI_SetEnabled(bool enabled);
bool SS_SCI_Enabled(void);

/* Called by the driver, on the emulation thread, when a byte arrives.
 *
 * `hold_seen` matters only when the byte came from a peer that drove SCK
 * while `cpu` waits on an external clock: it says whether the byte this end
 * had armed was visible to that peer at the moment it clocked. If it was, the
 * transfer took it; if it was not, the peer read the idle line and this end
 * still holds its byte for the next clock. Deciding that from what the peer
 * could see, rather than from what this end did, is what keeps both consoles
 * agreeing about every byte however their threads interleave. */
void SS_SCI_Receive(unsigned cpu, uint8_t value, const ss_sci_format *fmt, bool hold_seen);

/* Wake the master SH-2's rendezvous deadline to `ts` (e.g. after attaching). */
void SS_SCI_KickPoll(void);

/* The monotonic master-clock count at the calling CPU timestamp `ts`: frames
 * already emulated plus this one's progress. Never goes backwards within a
 * session; a state load or reset is the driver's business to re-anchor. */
uint64_t SS_LinkClock(int32_t ts);
/* Master-clock cycles per CPU cycle this frame: 61 or 65, by resolution. */
int32_t SS_ClockDiv(void);
/* Master-clock cycles per second, for the bus's clock_rate. */
uint64_t SS_LinkClockRate(void);
/* The shortest frame, in master cycles, of any SH-2 whose port is enabled
 * (TE or RE) among the CPUs in `cpu_mask` (bit 0 master, bit 1 slave), or 0. A rendezvous horizon no longer than this
 * adds no delay: see SH7095_SCI_Announce. */
uint32_t SS_SCI_FastestFrameMaster(unsigned cpu_mask);
/* The current timestamp of `cpu`, for a driver called from outside it. */
int32_t SS_SCI_CPUTimestamp(unsigned cpu);

#ifdef __cplusplus
}
#endif

#endif
