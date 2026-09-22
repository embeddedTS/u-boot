// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2024 Technologic Systems, Inc. (dba embeddedTS)
 *
 * This provides an abstraction to the wizard microcontroller
 * on embeddedTS platforms. This device uses 16-bit address/data
 */

#include <dm/uclass.h>
#include <i2c.h>

#include "wizard.h"

/*
 * u-boot crc16_ccitt() uses the CRC-16/CCITT-FALSE variant, zephyr crc16_ccitt()
 * uses CRC-16/CCITT-TRUE variant:
 */
static uint16_t crc16_ccitt_true(uint16_t crc, const uint8_t *buf, size_t len)
{
	uint8_t e, f;

	for (; len > 0; len--) {
		e = crc ^ *buf++;
		f = e ^ (e << 4);
		crc = (crc >> 8) ^ ((uint16_t)f << 8) ^ ((uint16_t)f << 3) ^ ((uint16_t)f >> 4);
	}

	return crc;
}

/*
 * wizard_get_i2c_chip - Locate the Wizard
 *
 * On early prototypes the wizard is on bus 0, on new designs
 * it is on bus 3. When the early prototypes are dropped this
 * can be simplified to just use bus 3, or locate the compatible node
 * on the device tree.
 *
 * This has to work using the provisional/default device tree; before
 * the correct device tree for this board model has been selected.
 */
static struct udevice *wizard_get_i2c_chip(void)
{
	struct udevice *chip;
	struct udevice *bus;
	u16 value;

	if (uclass_get_device_by_seq(UCLASS_I2C, 3, &bus))
		return NULL;

	if (i2c_get_chip(bus, WIZARD_I2C_ADDR, 2, &chip))
		return NULL;

	if (i2c_set_chip_offset_len(chip, 2))
		return NULL;

	if (dm_i2c_read(chip, 0, (uint8_t *)&value, sizeof(value)))
		return NULL;

	return chip;
}

int wizard_write(u16 addr, u16 value)
{
	struct udevice *chip;

	chip = wizard_get_i2c_chip();
	if (!chip)
		return -ENODEV;

	return dm_i2c_write(chip, cpu_to_be16(addr), (uint8_t *)&value, 2);
}

int wizard_read(u16 addr, u16 *value)
{
	struct udevice *chip;
	int ret;

	chip = wizard_get_i2c_chip();
	if (!chip) {
		printf("Error: No I2C chip found\n");
		return -ENODEV;
	}

	ret = dm_i2c_read(chip, cpu_to_be16(addr), (uint8_t *)value, 2);
	if (ret) {
		printf("Error: dm_i2c_read failed, ret=%d\n", ret);
		return ret;
	}

	return 0;
}

int wizard_read_mac(uint8_t *mac_buffer)
{
	int ret;

	struct ets_device_config *devcfg = wizard_read_config();
	if (devcfg) {
		memcpy(mac_buffer, devcfg->serial, sizeof(devcfg->serial));
		ret = 0;
	} else {
		printf("Error: failed to read mac\n");
		ret = -EINVAL;
	}
	return ret;
}

struct ets_device_config *wizard_read_config(void)
{
	/* device config is large and doesn't change, only read it once */
	static struct ets_device_config devcfg;
	static bool devcfg_valid;

	struct udevice *chip;
	u16 reg_addr = WIZARD_DEVICE_CONFIG;
	int err;

	if (devcfg_valid) {
		return &devcfg;
	}

	chip = wizard_get_i2c_chip();
	if (!chip) {
		printf("Error: No I2C chip found\n");
		return NULL;
	}

	err = dm_i2c_read(chip, cpu_to_be16(reg_addr), (uint8_t *)&devcfg, sizeof(devcfg));
	if (err) {
		printf("Error: dm_i2c_read failed, err=%d\n", err);
		return NULL;
	}

	/* Device Config should have "eTS0" in the hdr and validate with crc16 */
	if (strncmp(devcfg.hdr, "eTS0", 4) != 0 ||
			crc16_ccitt_true(0, (const uint8_t *)&devcfg, sizeof(devcfg) - 2) != devcfg.crc) {
		printf("Error: ets_device_config from wizard contains corrupt data\n");
		return NULL;
	}

	devcfg_valid = true;
	return &devcfg;
}
