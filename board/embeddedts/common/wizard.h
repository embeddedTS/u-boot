/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __WIZARD_H__
#define __WIZARD_H__

#define WIZARD_I2C_ADDR 0x54
#define WIZARD_SERIAL 34
#define WIZARD_DEVICE_CONFIG 1024

struct ets_device_config {
	/* Set hdr to magic bytes "eTS0" when config is set */
	char hdr[4];

	/* board_config version 1. Make sure dance and zephyr agree */
	uint16_t version;

	/* serial/mac_address bytes */
	uint8_t serial[6];

	/* Example: 0x9370
	 * This is expected to be interpreted by the zephyr code to understand
	 * the specific platform nuances, supercap count, vin ranges, 5v only, etc
	 */
	uint32_t product_family;

	/* Example: TS-9370-DMN9I
	 * This will be interpreted by our internal production environment to know
	 * which blast or test to run on a board.
	 *
	 * Zephyr, u-boot, and Linux may print this out, but should avoid attempting
	 * to interpret this string. We don't want u-boot/linux/etc to have to be
	 * updated each time we add a new part number.  Eg, if batman wants a custom
	 * population, we may make a custom part: TS-9370-GOTHAM and we don't want
	 * customs to cause pointless version churn on zephyr, u-boot, or Linux.
	 */
	char product_string[100];

	/* Example: 0 = 1GB, 1 = 2GB
	 * other numbers would match other configs we keep later in the future.
	 * These are not globally unique. A TS-9370's u-boot can interpret "0" however
	 * is appropriate for that platform.  A TS-11070 may interpret "0" in a
	 * completely different way.
	 */
	uint32_t ram_timing;

	/* "A", or "P1"
	 * Zephyr, u-boot, linux, or users may make decisions for rev workarounds or
	 * detecting board revision changes based on this.
	 */
	char pcb_rev[8];

	/* The BOM rev sticker typically has something like:
	 * "A01", pcb_rev="A", bom_rev="01"
	 * For prototypes, this follows:
	 * "P1A", pcb_rev="P1", bom_rev="A"
	 * Most things shouldn't intepret and try to do anything with this.  This would
	 * be given as a "in case of emergency".
	 * Eg, we release REV A01, and swap out to REV A02 with a new accelerometer.  It
	 * passes our initial tests, looks identical, but we find a subtle problem we can't
	 * detect.  This would let us then detect and do some kind of workaround.  This is
	 * also just useful for tracking purposes, eg if we have boards crashing with a
	 * particular BOM rev.
	 */
	char bom_rev[8];

	/* CRC16-CCITT (KERMIT) of previous packed data */
	uint16_t crc;
} __attribute__((packed));

int wizard_write(u16 addr, u16 value);
int wizard_read(u16 addr, u16 *value);
int wizard_read_mac(uint8_t *mac_buffer);
struct ets_device_config *wizard_read_config(void);

#endif // __WIZARD_H__
