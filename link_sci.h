#ifndef LINK_SCI_H__
#define LINK_SCI_H__

#include "libretro_link.h"

/* The Saturn Link Cable over the frontend's link bus. See link_sci.c. */
void link_sci_attach(const struct retro_link_interface *link, unsigned port);
void link_sci_detach(void);
/* After a reset or a state load: drop whatever was in flight. */
void link_sci_reanchor(void);

#endif
