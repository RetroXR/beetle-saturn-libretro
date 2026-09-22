/* A Saturn Link Cable carried by the frontend's link bus.
 *
 * Two Saturns emulated in one process cannot reach each other on their own: a
 * frontend that runs several cores at once loads each from its own copy of the
 * shared library, so every global -- including a coordinator a core might host
 * -- exists once per console. The frontend is the only thing they share, so it
 * hosts the bus and the core joins it through RETRO_ENVIRONMENT_GET_LINK_INTERFACE
 * (libretro/RetroArch#19454).
 *
 * The cable joins the two consoles' Communication Connectors, which carry the
 * serial port of each SH-2. What crosses here is bytes, per SH-2: the master's
 * TxD reaches the other console's master RxD and the slave's the slave's. A
 * byte is stamped with the moment it finishes arriving and held until this
 * console's clock gets there, so when it lands depends on emulated time alone.
 *
 * Clocked mode has an extra half. The end driving SCK shifts the other end's
 * transmit register in while shifting its own out, so it needs to know what the
 * other end had LOADED at that instant. Each end therefore announces its loaded
 * byte (a HOLD) as it loads it, and the clocking end takes the latest one stamped
 * at or before the transfer. That question has one answer on both consoles: the
 * bus never grants a core more than it asked for, so by the time the clocking
 * end reaches tick T the other has promised not to originate anything before T
 * and everything it stamped up to T is already on the bus. The far end applies
 * the same rule when the clock arrives, and the two never disagree about a byte.
 *
 * Attaching is safe whether or not anything is ever cabled: an unjoined port
 * bounds nobody and carries nothing, and the SCI behaves as a console with an
 * empty socket.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "link_sci.h"
#include "mednafen/ss/sci_link.h"

extern retro_log_printf_t log_cb;

/* Peers whose protocol ids differ are never joined, which keeps this cable out
 * of every other machine's socket. */
#define SL_PROTOCOL "saturn-sci-1"

/* A link cable has two ends. The bus would join a third console; refusing it
 * here is cheaper than deciding what a three-way crossover means. */
#define SL_MAX_PEERS 2

enum
{
   SL_BYTE = 1,   /* a byte off a TxD, stamped when it finishes arriving */
   SL_HOLD        /* clocked mode: what an external-clock end has loaded */
};

enum
{
   SL_FLAG_SYNC  = 1,
   SL_FLAG_ARMED = 2
};

/* Packed by hand: protocol_id exists so something else could speak this later,
 * and a shared struct layout would be an assumption nobody remembers making.
 *   0 type  1 sender's bus id  2 cpu  3 value  4 flags  5 frame  6-7 zero
 *   8-11 bit time in master cycles, little-endian */
#define SL_MSG_SIZE 12

/* Rendezvous intervals, as fractions of a second of emulated time.
 *
 * IDLE is for a port with nothing on the other end, where a rendezvous costs
 * next to nothing anyway. CABLED is the ceiling once a partner is there, and
 * it is a latency bound, not a cost one: a horizon, once published, cannot be
 * taken back, so the first byte a game sends after a quiet spell is stamped no
 * earlier than the horizon promised DURING the quiet spell. At two thousand a
 * second that was half a millisecond, and Daytona USA's hello arrived four
 * byte-times late and was answered into a handshake that never finished.
 * While either SH-2 has its port enabled the interval is one frame of the
 * fastest port instead (see sl_drv_poll), floored at BUSY_MAX so a port set
 * to a rate no game uses cannot make the two threads do nothing but meet. */
#define SL_IDLE_HZ     2000
#define SL_CABLED_HZ   20000
#define SL_BUSY_MAX_HZ 400000

#define SL_PENDING_MAX 512

/* Which SH-2s the cable actually wires, as a bit per CPU: the SLAVE only.
 *
 * The Communication Connector carries both SH-2s' serial lines, but crossing
 * the master's as well cannot be what the cable does. Every Saturn disc boots
 * through a Sega library routine that puts the MASTER SH-2's port in clocked
 * mode, sends a four-byte probe (80 11 3B 74) and listens for an answer; with
 * the master lines crossed, two cabled consoles answer each other's probe on
 * time and both boot into waiting for a development host that is not there.
 * Real consoles boot with the cable in, and every link game measured here
 * (Steeldom, GunGriffon II) talks on the slave SH-2's port, asynchronously. */
static unsigned sl_wired = 0x2u;
#define SL_WIRED sl_wired
#define SL_IS_WIRED(cpu) ((sl_wired >> ((cpu) & 1)) & 1u)

struct sl_event
{
   uint64_t tick;
   uint64_t seq;         /* arrival order, which settles a tie */
   uint8_t type;
   uint8_t cpu;
   uint8_t value;
   uint8_t flags;
   ss_sci_format fmt;
};

struct sl_hold
{
   bool armed;
   uint8_t value;
   uint64_t tick;
};

static const struct retro_link_interface *sl_link;
static retro_link_port_t *sl_handle;
static bool sl_attached;

static int sl_self_id;
static unsigned sl_peers;

static uint64_t sl_safe;           /* the furthest horizon published */
static uint64_t sl_last_stamp[2];  /* per SH-2: a wire carries one byte at a time */
static uint32_t sl_last_span[2];

/* What the other console has armed, as far as this one may know by now. */
static struct sl_hold sl_peer_hold[2];
/* What this console armed, and the tick the other end can first see it. */
static struct sl_hold sl_my_hold[2];

static struct sl_event sl_pending[SL_PENDING_MAX];
static unsigned sl_pending_count;
static uint64_t sl_next_seq;

static uint64_t sl_rate;

/* Frame edges (beetle_saturn_link_frame_edges; netplay pins it on).
 *
 * Group rollback stops every cabled console at every frame edge, and snapshots
 * the bus there, so nothing about a frame may reach past its edge: the port
 * never asks the bus for more than the frame (a peer stopped at the edge could
 * never grant it), and it meets the peer at both edges, so everything either
 * console stamped up to an edge is on the bus before the next frame runs. The
 * edges themselves line up because every frame spans the same number of link
 * ticks (SS_LinkFrameSpan). */
static bool sl_frame_edges;
static bool sl_in_frame;
static uint64_t sl_frame_end;

/* How far the bus has let this console run, or ~0 when nothing bounds it. A
 * byte may land only up to here. The CPU overshoots its rendezvous by an
 * instruction, or a block under the JIT, and a register read in that overshoot
 * would otherwise land anything due by the current cycle -- but a byte due past
 * the grant may not be on the bus yet, and whether it is depends on how far the
 * other emulation thread has got. Used with frame edges only. */
static uint64_t sl_granted = ~(uint64_t)0;

/* A load put the driver back verbatim; the reanchor that follows it must not
 * then throw that away. */
static bool sl_restored;

static void sl_log(enum retro_log_level level, const char *fmt, const char *arg)
{
   if (log_cb)
      log_cb(level, fmt, arg);
}

static uint64_t sl_now(int32_t ts)
{
   return SS_LinkClock(ts);
}

static uint64_t sl_stamp_floor(uint64_t tick)
{
   return tick < sl_safe ? sl_safe : tick;
}

static void sl_put32(uint8_t *p, uint32_t v)
{
   p[0] = (uint8_t)v;
   p[1] = (uint8_t)(v >> 8);
   p[2] = (uint8_t)(v >> 16);
   p[3] = (uint8_t)(v >> 24);
}

static uint32_t sl_get32(const uint8_t *p)
{
   return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void sl_send(uint64_t tick, uint8_t type, unsigned cpu, uint8_t value, uint8_t flags,
      const ss_sci_format *fmt)
{
   uint8_t msg[SL_MSG_SIZE];

   if (!sl_attached || sl_peers < 2)
      return;

   memset(msg, 0, sizeof(msg));
   msg[0] = type;
   msg[1] = (uint8_t)sl_self_id;
   msg[2] = (uint8_t)cpu;
   msg[3] = value;
   msg[4] = flags;
   if (fmt)
   {
      msg[5] = fmt->frame;
      sl_put32(msg + 8, fmt->bit_master);
   }
   sl_link->send(sl_handle, tick, RETRO_LINK_BROADCAST, msg, sizeof(msg));
}

static void sl_forget_peer(void)
{
   sl_pending_count = 0;
   memset(sl_peer_hold, 0, sizeof(sl_peer_hold));
}

/* Tell a freshly cabled peer what this end is holding. A HOLD sent before the
 * cable existed went nowhere. */
static void sl_reannounce(void)
{
   unsigned cpu;

   for (cpu = 0; cpu < 2; cpu++)
   {
      if (!sl_my_hold[cpu].armed)
         continue;
      sl_my_hold[cpu].tick = sl_safe;
      sl_send(sl_safe, SL_HOLD, cpu, sl_my_hold[cpu].value, SL_FLAG_SYNC | SL_FLAG_ARMED, NULL);
   }
}

static void sl_refresh_peers(void)
{
   unsigned was = sl_peers;
   unsigned count = 0;
   int id = sl_link->peers(sl_handle, &count);

   if (id < 0 || count > SL_MAX_PEERS)
   {
      if (count > SL_MAX_PEERS && was <= SL_MAX_PEERS)
         sl_log(RETRO_LOG_WARN, "[link] %s: more than two consoles on one Saturn cable\n", SL_PROTOCOL);
      sl_self_id = 0;
      sl_peers = 0;
   }
   else
   {
      sl_self_id = id;
      sl_peers = count;
   }

   if (sl_peers == was)
      return;

   /* Whatever either end knew came from a bus that no longer exists. */
   sl_forget_peer();

   if (sl_peers >= 2)
   {
      sl_log(RETRO_LOG_WARN, "[link] %s: cable connected\n", SL_PROTOCOL);
      sl_reannounce();
   }
   else if (was >= 2)
      sl_log(RETRO_LOG_WARN, "[link] %s: cable disconnected\n", SL_PROTOCOL);
}

static void sl_queue(uint64_t tick, const uint8_t *msg)
{
   struct sl_event *ev;

   if (sl_pending_count >= SL_PENDING_MAX)
      return;   /* sl_drain never takes more than there is room for */

   ev = &sl_pending[sl_pending_count++];
   ev->tick = tick;
   ev->seq = sl_next_seq++;
   ev->type = msg[0];
   ev->cpu = msg[2] & 1;
   ev->value = msg[3];
   ev->flags = msg[4];
   ev->fmt.sync = (msg[4] & SL_FLAG_SYNC) != 0;
   ev->fmt.frame = msg[5];
   ev->fmt.bit_master = sl_get32(msg + 8);
}

/* Take everything off the bus there is room for. A message left on the bus is
 * still there at the next drain; one taken and dropped is gone, and it is always
 * the byte a game is waiting for. */
static void sl_drain(void)
{
   uint8_t msg[SL_MSG_SIZE];
   uint64_t tick;
   unsigned from;
   size_t len;

   if (!sl_attached)
      return;

   while (sl_pending_count < SL_PENDING_MAX)
   {
      len = sizeof(msg);
      if (!sl_link->recv(sl_handle, &tick, &from, msg, &len))
         break;
      if (len == SL_MSG_SIZE && (msg[0] == SL_BYTE || msg[0] == SL_HOLD))
         sl_queue(tick, msg);
   }
}

static void sl_apply(const struct sl_event *ev)
{
   switch (ev->type)
   {
      case SL_HOLD:
         sl_peer_hold[ev->cpu].armed = (ev->flags & SL_FLAG_ARMED) != 0;
         sl_peer_hold[ev->cpu].value = ev->value;
         sl_peer_hold[ev->cpu].tick = ev->tick;
         break;

      case SL_BYTE:
      {
         /* The peer clocked this end, if it drove SCK: did it see our HOLD? */
         const struct sl_hold *mine = &sl_my_hold[ev->cpu];
         bool seen = mine->armed && mine->tick <= ev->tick;

         if (ev->fmt.sync && seen)
            sl_my_hold[ev->cpu].armed = false;
         SS_SCI_Receive(ev->cpu, ev->value, &ev->fmt, seen);
         break;
      }
   }
}

/* What may have landed by `now`: never past the grant, with frame edges. */
static uint64_t sl_visible(uint64_t now)
{
   if (sl_frame_edges && now > sl_granted)
      return sl_granted;
   return now;
}

/* Hand over everything due by `now`, oldest first, ties in arrival order. */
static void sl_release(uint64_t now)
{
   now = sl_visible(now);
   for (;;)
   {
      unsigned i, best = SL_PENDING_MAX;

      for (i = 0; i < sl_pending_count; i++)
      {
         if (sl_pending[i].tick > now)
            continue;
         if (best == SL_PENDING_MAX ||
             sl_pending[i].tick < sl_pending[best].tick ||
             (sl_pending[i].tick == sl_pending[best].tick && sl_pending[i].seq < sl_pending[best].seq))
            best = i;
      }
      if (best == SL_PENDING_MAX)
         return;

      {
         struct sl_event ev = sl_pending[best];
         sl_pending[best] = sl_pending[--sl_pending_count];
         sl_apply(&ev);
      }
   }
}

/* ── the driver the SCI sees ──────────────────────────────────────────────── */

static void sl_drv_tx(unsigned cpu, uint8_t value, const ss_sci_format *fmt, int32_t ts)
{
   uint64_t land;
   /* A frame's length from SMR's shape bits: start, 7 or 8 data, parity, 1 or 2
    * stop. Counting a parity frame as ten bits let bytes arrive faster than
    * they could have been sent. */
   uint32_t bits = fmt->sync ? 8u : 1u + ((fmt->frame & 0x40) ? 7u : 8u)
         + ((fmt->frame & 0x20) ? 1u : 0u) + ((fmt->frame & 0x08) ? 2u : 1u);
   uint32_t span = fmt->bit_master * bits;

   cpu &= 1;
   if (!sl_attached || sl_peers < 2 || !SL_IS_WIRED(cpu))
      return;

   land = sl_stamp_floor(sl_now(ts));
   /* Bytes leave a serial port a frame apart, and arrive that way, however far
    * the horizon pushed the one before. */
   if (sl_last_span[cpu] && land < sl_last_stamp[cpu] + sl_last_span[cpu])
      land = sl_last_stamp[cpu] + sl_last_span[cpu];
   sl_last_stamp[cpu] = land;
   sl_last_span[cpu] = span;

   sl_send(land, SL_BYTE, cpu, value, fmt->sync ? SL_FLAG_SYNC : 0, fmt);
}

static bool sl_drv_sample(unsigned cpu, uint8_t *value, int32_t ts)
{
   uint64_t now = sl_now(ts);
   struct sl_hold *h;

   cpu &= 1;
   if (!sl_attached || sl_peers < 2 || !SL_IS_WIRED(cpu))
      return false;

   sl_drain();
   sl_release(now);

   h = &sl_peer_hold[cpu];
   if (!h->armed || h->tick > sl_visible(now))
      return false;

   *value = h->value;
   h->armed = false;   /* clocked out; the peer arms the next one itself */
   return true;
}

static void sl_drv_hold(unsigned cpu, uint8_t value, bool armed, int32_t ts)
{
   uint64_t tick = sl_stamp_floor(sl_now(ts));

   cpu &= 1;
   if (!SL_IS_WIRED(cpu))
      return;
   sl_my_hold[cpu].armed = armed;
   sl_my_hold[cpu].value = value;
   sl_my_hold[cpu].tick = tick;

   sl_send(tick, SL_HOLD, cpu, value, SL_FLAG_SYNC | (armed ? SL_FLAG_ARMED : 0), NULL);
}

static void sl_drv_sync(int32_t ts)
{
   if (!sl_attached)
      return;
   sl_drain();
   sl_release(sl_now(ts));
}

/* Advance to `request`, and not a tick short of it.
 *
 * The bus hands back less than was asked for when a message or a cable change
 * arrives while this console waits. That only says something happened on
 * another thread at some moment, and a port that acted on it would take its
 * next step at a cycle chosen by thread timing, so two runs of the same inputs
 * -- a rollback and the lockstep run it has to reproduce -- would part. The
 * early return is taken only to drain the queue (which lands nothing: bytes
 * land by their tick). */
static uint64_t sl_advance_to(uint64_t now, uint64_t request)
{
   for (;;)
   {
      uint32_t wake = RETRO_LINK_WAKE_NONE;
      uint64_t grant = sl_link->advance(sl_handle, now, sl_safe, request, &wake);

      if (grant == RETRO_LINK_UNBOUNDED || grant >= request || (wake & RETRO_LINK_WAKE_DETACHED))
      {
         sl_granted = (grant == RETRO_LINK_UNBOUNDED || (wake & RETRO_LINK_WAKE_DETACHED))
               ? ~(uint64_t)0 : grant;
         return grant;
      }
      sl_drain();
   }
}

static int32_t sl_drv_poll(int32_t ts)
{
   uint64_t now, grant, grain;
   uint32_t wake = RETRO_LINK_WAKE_NONE;
   int32_t div = SS_ClockDiv();
   bool busy;

   if (!sl_attached)
      return 1 << 20;

   /* The horizon is the lookahead every byte is announced with, so it may not
    * be longer than the fastest frame either port is set up for: a byte
    * starting now lands one frame from now, and it may not be stamped any
    * later than that. With the ports off the consoles only have to agree
    * nobody is talking, and two thousand meetings a second is plenty. */
   {
      uint64_t fastest = SS_SCI_FastestFrameMaster(SL_WIRED);

      busy = sl_peers >= 2 && fastest != 0;
      uint64_t ceiling = sl_rate / (sl_peers >= 2 ? SL_CABLED_HZ : SL_IDLE_HZ);

      grain = busy ? fastest : ceiling;
      if (busy && grain < sl_rate / SL_BUSY_MAX_HZ)
         grain = sl_rate / SL_BUSY_MAX_HZ;
      if (grain > ceiling)
         grain = ceiling;
      if (grain < 1)
         grain = 1;
   }

   now = sl_now(ts);
   /* Never past the frame's edge, which a peer stopped there cannot grant. */
   if (sl_frame_edges && sl_in_frame && now + grain > sl_frame_end)
      grain = sl_frame_end > now ? sl_frame_end - now : 1;
   if (sl_safe < now + grain)
      sl_safe = now + grain;

   /* Published before reading: a peer parked on this console's horizon cannot
    * move until told it moved, and it may hold the very byte wanted next. */
   if (sl_frame_edges)
      grant = sl_advance_to(now, now + grain);
   else
      grant = sl_link->advance(sl_handle, now, sl_safe, now + grain, &wake);

   sl_refresh_peers();
   sl_drain();
   sl_release(now);

   /* Never run past the grant: that is what makes "everything the peer stamped
    * up to now is already on the bus" true, which sl_drv_sample relies on. */
   if (grant != RETRO_LINK_UNBOUNDED && grant > now && grant - now < grain)
      grain = grant - now;

   /* And wake for the next byte in the queue at the tick it is due, not at
    * the next rendezvous. A rendezvous interval is longer than a byte at the
    * rates games run, so releasing only there hands a receiver two bytes on
    * the same cycle -- and with nobody able to read RDR in between, the second
    * is an overrun. GunGriffon II's battle died of exactly that. */
   {
      unsigned i;
      for (i = 0; i < sl_pending_count; i++)
         if (sl_pending[i].tick > now && sl_pending[i].tick - now < grain)
            grain = sl_pending[i].tick - now;
   }

   {
      uint64_t cycles = (grain + (uint64_t)div - 1) / (uint64_t)div;
      if (cycles < 1)
         cycles = 1;
      if (cycles > (1u << 20))
         cycles = 1u << 20;
      return (int32_t)cycles;
   }
}

static const ss_sci_driver sl_driver = {
   sl_drv_tx,
   sl_drv_sample,
   sl_drv_hold,
   sl_drv_sync,
   sl_drv_poll,
};

/* ── attach and detach ────────────────────────────────────────────────────── */

void link_sci_attach(const struct retro_link_interface *link, unsigned port)
{
   if (sl_attached || !link)
      return;

   sl_rate = SS_LinkClockRate();
   if (!sl_rate)
      return;

   sl_handle = link->attach(port, SL_PROTOCOL, sl_rate);
   if (!sl_handle)
   {
      sl_log(RETRO_LOG_WARN, "[link] %s: the frontend refused the port\n", SL_PROTOCOL);
      return;
   }

   /* SS_LINK_WIRED=<mask> overrides which SH-2s the cable joins, for
    * finding out what a game uses; see SL_WIRED. */
   if (getenv("SS_LINK_WIRED"))
      sl_wired = (unsigned)atoi(getenv("SS_LINK_WIRED")) & 3u;

   sl_link = link;
   sl_attached = true;
   sl_self_id = 0;
   sl_peers = 0;
   sl_safe = 0;
   sl_pending_count = 0;
   sl_next_seq = 0;
   memset(sl_last_stamp, 0, sizeof(sl_last_stamp));
   memset(sl_last_span, 0, sizeof(sl_last_span));
   memset(sl_peer_hold, 0, sizeof(sl_peer_hold));
   memset(sl_my_hold, 0, sizeof(sl_my_hold));
   sl_in_frame = false;
   sl_granted = ~(uint64_t)0;

   SS_SCI_SetDriver(&sl_driver);
   sl_log(RETRO_LOG_WARN, "[link] %s: Communication Connector attached\n", SL_PROTOCOL);
}

void link_sci_detach(void)
{
   SS_SCI_SetDriver(NULL);

   if (sl_attached)
   {
      sl_link->detach(sl_handle);
      sl_attached = false;
      sl_handle = NULL;
   }
   sl_link = NULL;
   sl_pending_count = 0;
   sl_peers = 0;
   sl_in_frame = false;
   sl_granted = ~(uint64_t)0;
}

/* A reset or a state load moved the console's clock under the bus. Nothing in
 * flight survives either on real hardware, and SS_LinkClock never goes back, so
 * all that is left is to forget what was queued. */
void link_sci_reanchor(void)
{
   if (sl_restored)
   {
      sl_restored = false;
      return;
   }
   sl_pending_count = 0;
   memset(sl_peer_hold, 0, sizeof(sl_peer_hold));
   memset(sl_my_hold, 0, sizeof(sl_my_hold));
}

/* ── frame edges ──────────────────────────────────────────────────────────── */

/* A rendezvous at the edge itself: this console promises nothing before the
 * edge and waits for every peer to reach it, so all either stamped up to here
 * is on the bus, and drained, before the next frame runs. */
static void sl_edge(uint64_t edge)
{
   if (sl_safe < edge)
      sl_safe = edge;
   sl_advance_to(edge, edge);
   sl_refresh_peers();
   sl_drain();
}

void link_sci_set_frame_edges(bool on)
{
   if (on == sl_frame_edges)
      return;
   sl_frame_edges = on;
   sl_in_frame = false;
   sl_granted = ~(uint64_t)0;
   SS_SetLinkFrameEdges(on);
}

void link_sci_frame_begin(void)
{
   uint64_t edge;

   if (!sl_attached || !sl_frame_edges)
      return;
   edge = SS_LinkFrameBase();
   sl_edge(edge);
   sl_frame_end = edge + SS_LinkFrameSpan();
   sl_in_frame = true;
   sl_release(edge);
}

void link_sci_frame_end(void)
{
   if (!sl_attached || !sl_frame_edges)
      return;
   sl_in_frame = false;
   /* Emulate has already moved the clock on to the next frame's edge. */
   sl_edge(SS_LinkFrameBase());
}

/* ── the driver's state, in the savestate's LINK section ──────────────────── */

#define SL_STATE_MAGIC   0x4b4e4c53u /* "SLNK" */
#define SL_STATE_VERSION 1u

/* Packed by hand, little-endian, every byte written: padding left to chance
 * would make two identical states hash differently. */
struct sl_packer
{
   uint8_t *p;
   const uint8_t *q;
};

static void sl_w8(struct sl_packer *k, uint8_t v) { *k->p++ = v; }
static void sl_w32(struct sl_packer *k, uint32_t v) { sl_put32(k->p, v); k->p += 4; }
static void sl_w64(struct sl_packer *k, uint64_t v) { sl_w32(k, (uint32_t)v); sl_w32(k, (uint32_t)(v >> 32)); }
static uint8_t sl_r8(struct sl_packer *k) { return *k->q++; }
static uint32_t sl_r32(struct sl_packer *k) { uint32_t v = sl_get32(k->q); k->q += 4; return v; }
static uint64_t sl_r64(struct sl_packer *k) { uint64_t lo = sl_r32(k); return lo | ((uint64_t)sl_r32(k) << 32); }

/* The fixed part is 117 bytes and each queued event 26. */
typedef char sl_state_fits[(117 + SL_PENDING_MAX * 26 <= SS_LINK_STATE_BYTES) ? 1 : -1];

static void sl_w_hold(struct sl_packer *k, const struct sl_hold *h)
{
   sl_w8(k, h->armed ? 1 : 0);
   sl_w8(k, h->value);
   sl_w64(k, h->tick);
}

static void sl_r_hold(struct sl_packer *k, struct sl_hold *h)
{
   h->armed = sl_r8(k) != 0;
   h->value = sl_r8(k);
   h->tick = sl_r64(k);
}

void SS_LinkDriverStateSave(uint8_t *blob)
{
   struct sl_packer k;
   unsigned i;

   k.p = blob;
   sl_w32(&k, SL_STATE_MAGIC);
   sl_w32(&k, SL_STATE_VERSION);
   sl_w32(&k, (uint32_t)sl_self_id);
   sl_w32(&k, sl_peers);
   sl_w64(&k, sl_safe);
   for (i = 0; i < 2; i++)
   {
      sl_w64(&k, sl_last_stamp[i]);
      sl_w32(&k, sl_last_span[i]);
   }
   for (i = 0; i < 2; i++)
      sl_w_hold(&k, &sl_peer_hold[i]);
   for (i = 0; i < 2; i++)
      sl_w_hold(&k, &sl_my_hold[i]);
   sl_w32(&k, sl_pending_count);
   sl_w64(&k, sl_next_seq);
   sl_w8(&k, sl_in_frame ? 1 : 0);
   sl_w64(&k, sl_frame_end);
   sl_w64(&k, sl_granted);
   for (i = 0; i < sl_pending_count; i++)
   {
      const struct sl_event *ev = &sl_pending[i];
      sl_w64(&k, ev->tick);
      sl_w64(&k, ev->seq);
      sl_w8(&k, ev->type);
      sl_w8(&k, ev->cpu);
      sl_w8(&k, ev->value);
      sl_w8(&k, ev->flags);
      sl_w8(&k, ev->fmt.sync ? 1 : 0);
      sl_w8(&k, ev->fmt.frame);
      sl_w32(&k, ev->fmt.bit_master);
   }
}

bool SS_LinkDriverStateLoad(const uint8_t *blob)
{
   struct sl_packer k;
   unsigned i, count;

   k.q = blob;
   if (sl_r32(&k) != SL_STATE_MAGIC || sl_r32(&k) != SL_STATE_VERSION)
      return false;
   sl_self_id = (int)sl_r32(&k);
   sl_peers = sl_r32(&k);
   sl_safe = sl_r64(&k);
   for (i = 0; i < 2; i++)
   {
      sl_last_stamp[i] = sl_r64(&k);
      sl_last_span[i] = sl_r32(&k);
   }
   for (i = 0; i < 2; i++)
      sl_r_hold(&k, &sl_peer_hold[i]);
   for (i = 0; i < 2; i++)
      sl_r_hold(&k, &sl_my_hold[i]);
   count = sl_r32(&k);
   if (count > SL_PENDING_MAX)
      count = SL_PENDING_MAX;
   sl_next_seq = sl_r64(&k);
   sl_in_frame = sl_r8(&k) != 0;
   sl_frame_end = sl_r64(&k);
   sl_granted = sl_r64(&k);
   for (i = 0; i < count; i++)
   {
      struct sl_event *ev = &sl_pending[i];
      ev->tick = sl_r64(&k);
      ev->seq = sl_r64(&k);
      ev->type = sl_r8(&k);
      ev->cpu = sl_r8(&k);
      ev->value = sl_r8(&k);
      ev->flags = sl_r8(&k);
      ev->fmt.sync = sl_r8(&k) != 0;
      ev->fmt.frame = sl_r8(&k);
      ev->fmt.bit_master = sl_r32(&k);
   }
   sl_pending_count = count;
   sl_restored = true;
   return true;
}
