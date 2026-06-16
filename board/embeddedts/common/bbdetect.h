/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 Technologic Systems, Inc. dba embeddedTS
 */

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

/* Set the environment variables baseboard_id and baseboard_rev based on the
 * detection process.
 *
 * @note This uses the TS-SOCKET standard of expecting the baseboard to have a
 *       MUX IC with 3 inputs and an output to decode the 8 MUX bits as an ID
 *       and revision encoded in a single 8-bit value.
 *
 * @param pins A struct that contains GPIO pins laid out LSB first order that
 *             are the MUX input pins. Also defines the pin to read the data
 *             back from as the MUX pins are toggled.
 *
 * @param delay Number of microseconds to wait between switching MUX pin states
 *              before reading back the .in GPIO pin to read its state. Most
 *              platforms will want this to be 1000 us.
 *
 * @returns 0 on success, less than 0 on any error
 */
int bbdetect(const struct bbdetect_pins pins, unsigned long delay);

#endif // BBDETECT_H
