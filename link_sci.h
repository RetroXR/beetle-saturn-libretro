#ifndef LINK_SCI_H__
#define LINK_SCI_H__

#include "libretro_link.h"

/* The Saturn Link Cable over the frontend's link bus. See link_sci.c. */
void link_sci_attach(const struct retro_link_interface *link, unsigned port);
void link_sci_detach(void);
/* After a reset or a state load: drop whatever was in flight. */
void link_sci_reanchor(void);

/* Frame edges (beetle_saturn_link_frame_edges): hold the port to each frame and
 * meet the peer at both edges, either side of Emulate(). See link_sci.c. */
void link_sci_set_frame_edges(bool on);
void link_sci_frame_begin(void);
void link_sci_frame_end(void);

#endif
