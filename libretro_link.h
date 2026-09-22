#ifndef LIBRETRO_LINK_H__
#define LIBRETRO_LINK_H__

/* The link interface proposed for libretro.h in
 * https://github.com/libretro/RetroArch/pull/19454, carried here verbatim
 * until the vendored libretro-common has it. libretro-common tracks upstream
 * and should stay updatable, so it is not patched; once libretro.h defines
 * RETRO_ENVIRONMENT_GET_LINK_INTERFACE this file steps aside. */

#include <libretro.h>

#ifndef RETRO_ENVIRONMENT_GET_LINK_INTERFACE

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Gives a core access to a link bus hosted by the frontend, so that core
 * instances running in the same process can emulate machines joined by a cable.
 *
 * The frontend arbitrates emulated time and moves payloads without interpreting
 * them. The wire protocol stays in the cores.
 *
 * Should be called in \c retro_init or \c retro_load_game. The returned function
 * pointers remain valid until \c retro_deinit, whether or not anything attaches.
 *
 * @param[out] data <tt>struct retro_link_interface *</tt>.
 * Set to the frontend's link interface. None of its members are \c NULL if this
 * call returns \c true.
 * @return \c true if the frontend hosts a link bus.
 * \c false otherwise, which a core must treat as nothing plugged in.
 * @see retro_link_interface
 */
#define RETRO_ENVIRONMENT_GET_LINK_INTERFACE (94 | RETRO_ENVIRONMENT_EXPERIMENTAL)

/**
 * Returned by \c retro_link_advance_t when nothing bounds this core: nothing is
 * attached to the port, or every peer has detached. The core then runs freely.
 */
#define RETRO_LINK_UNBOUNDED ((uint64_t)-1)

/** Peer id meaning "every other peer on this port's bus". */
#define RETRO_LINK_BROADCAST 0xFF

/**
 * Reasons reported through the \c wake_flags argument of
 * \c retro_link_advance_t. More than one may be set. A core should drain
 * \c retro_link_recv_t on #RETRO_LINK_WAKE_MESSAGE and refresh
 * \c retro_link_peers_t on #RETRO_LINK_WAKE_TOPOLOGY before asking to advance
 * again.
 */
#define RETRO_LINK_WAKE_NONE     0u
/** A message has arrived on this port since the last report. */
#define RETRO_LINK_WAKE_MESSAGE  (1u << 0)
/** This port's bus membership has changed since the last report. */
#define RETRO_LINK_WAKE_TOPOLOGY (1u << 1)
/** The port is not attached. The grant is #RETRO_LINK_UNBOUNDED. */
#define RETRO_LINK_WAKE_DETACHED (1u << 2)

/**
 * An attached port. Returned by \c retro_link_attach_t and passed to every other
 * call. Opaque to the core.
 */
typedef struct retro_link_port retro_link_port_t;

/**
 * Joins the bus on \c port.
 *
 * @param port The core's port number. A machine with more than one socket
 * distinguishes them here.
 * @param protocol_id Names the emulated wire protocol, e.g. "gba-sio-1". Peers
 * whose ids differ are never connected to each other.
 * @param clock_rate Ticks of this core's link timeline per second of emulated
 * time, e.g. 16777216 for a Game Boy Advance. Must not be zero.
 * @return A handle for every other call on this port, or \c NULL if the port
 * could not be attached.
 */
typedef retro_link_port_t *(RETRO_CALLCONV *retro_link_attach_t)(unsigned port,
      const char *protocol_id, uint64_t clock_rate);

/**
 * Leaves the bus. Peers observe this as a detach at the current tick.
 *
 * @param handle The handle returned by \c retro_link_attach_t.
 */
typedef void (RETRO_CALLCONV *retro_link_detach_t)(retro_link_port_t *handle);

/**
 * Current membership of this port's bus. Membership can change at any time, so
 * a core should query it per transfer.
 *
 * @param handle The handle returned by \c retro_link_attach_t.
 * @param[out] count If non-\c NULL, receives the number of participants,
 * including this one.
 * @return This core's index on the bus, or -1 when the port is not attached or
 * is not cabled to anything.
 */
typedef int (RETRO_CALLCONV *retro_link_peers_t)(retro_link_port_t *handle, unsigned *count);

/**
 * Queues \c buf for delivery to a peer, stamped at \c tick on this core's
 * timeline. The frontend copies the payload.
 *
 * @param handle The handle returned by \c retro_link_attach_t.
 * @param tick When the event happens, on this core's timeline.
 * @param to The peer's index on the bus, or #RETRO_LINK_BROADCAST.
 * @param buf The payload.
 * @param len The payload's length in bytes.
 * @return \c false if the port is not attached.
 */
typedef bool (RETRO_CALLCONV *retro_link_send_t)(retro_link_port_t *handle, uint64_t tick,
      unsigned to, const void *buf, size_t len);

/**
 * Pops the next queued message, oldest first. A message can be received as soon
 * as it is sent, before its tick is reached.
 *
 * @param handle The handle returned by \c retro_link_attach_t.
 * @param[out] tick When the event lands, on this core's timeline.
 * @param[out] from The sender's index on the bus.
 * @param[out] buf Receives the payload.
 * @param[in,out] len Buffer capacity on entry, bytes written on return.
 * @return \c false when the queue is empty.
 */
typedef bool (RETRO_CALLCONV *retro_link_recv_t)(retro_link_port_t *handle, uint64_t *tick,
      unsigned *from, void *buf, size_t *len);

/**
 * Publishes this core's position and commit horizon, then asks how far it may
 * advance. Blocks until \c request_tick can be granted, until every peer has
 * detached, or, when \c wake_flags is non-\c NULL, until the membership changes
 * or a message due at or before \c local_tick arrives.
 *
 * The grant must be a pure function of the participants' published ticks, with
 * no wall-clock input and no timeout.
 *
 * @param handle The handle returned by \c retro_link_attach_t.
 * @param local_tick Where the core is now.
 * @param safe_tick The core will not originate an event a peer must observe
 * before this tick. Must be at least \c local_tick and may never be retracted.
 * Peers cannot advance past \c local_tick unless \c safe_tick is ahead of it.
 * @param request_tick How far the core is asking to advance.
 * @param[out] wake_flags If non-\c NULL, receives the \c RETRO_LINK_WAKE_* reasons
 * observed on this call. Each arrival and each membership change is reported
 * once. Pass \c NULL to never be woken early.
 * @return The tick this core may advance to, or #RETRO_LINK_UNBOUNDED. It never
 * exceeds \c request_tick unless \c local_tick or a previous grant already stood
 * further on, and is never less than either. When woken early it is the greater
 * of those two.
 *
 * @note Thread-safe.
 */
typedef uint64_t (RETRO_CALLCONV *retro_link_advance_t)(retro_link_port_t *handle,
      uint64_t local_tick, uint64_t safe_tick, uint64_t request_tick,
      uint32_t *wake_flags);

/**
 * Result of \c RETRO_ENVIRONMENT_GET_LINK_INTERFACE.
 */
struct retro_link_interface
{
   retro_link_attach_t  attach;  /**< Join the bus on a port. */
   retro_link_detach_t  detach;  /**< Leave the bus. */
   retro_link_peers_t   peers;   /**< Who else is on this port's bus. */
   retro_link_send_t    send;    /**< Queue a payload for a peer. */
   retro_link_recv_t    recv;    /**< Take the next payload off the queue. */
   retro_link_advance_t advance; /**< Publish a position, ask how far to run. */
};


#ifdef __cplusplus
}
#endif

#endif /* RETRO_ENVIRONMENT_GET_LINK_INTERFACE */

/* The unflagged number, for a frontend that adopts it once upstream assigns
 * it; a core probes the experimental form first. */
#ifndef RETRO_ENVIRONMENT_GET_LINK_INTERFACE_FINAL
#define RETRO_ENVIRONMENT_GET_LINK_INTERFACE_FINAL 94
#endif

#endif
