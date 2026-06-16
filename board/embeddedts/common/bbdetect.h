#ifndef BBDETECT_H
#define BBDETECT_H

#include <stddef.h>

/* This shouldn't ever need to be configured differently per-platform but can
 * be converted to a CFG_ option later if needed.
 */
#define BBID_MUX_BITS	3

struct bbdetect_pins {
	char *bit[BBID_MUX_BITS];
	char *in;
};

int bbdetect(const struct bbdetect_pins pins, unsigned long delay);

#endif // BBDETECT_H
