/*
 * Copyright (C) 2016 Technologic Systems
 *
 * Author: Kris Bahnsen <kris@embeddedarm.com>
 *   Based on work by Mark Featherston <mark@embeddedarm.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>

#include <asm/gpio.h>
#include <asm/arch/iomux-mx28.h>
#include <cli.h>
#include <command.h>
#include <i2c.h>

#define JP_MUX_CONFIG (MXS_PAD_3V3 | MXS_PAD_8MA | MXS_PAD_PULLUP)

iomux_cfg_t const jp_pads[] = {
	MX28_PAD_LCD_D08__GPIO_1_8 | JP_MUX_CONFIG,
	MX28_PAD_LCD_D09__GPIO_1_9 | JP_MUX_CONFIG,
};

static int do_readjp(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int jp;

	mxs_iomux_setup_multiple_pads(jp_pads, ARRAY_SIZE(jp_pads));

	gpio_direction_output(MX28_PAD_LCD_D08__GPIO_1_8, 1);
	gpio_direction_input(MX28_PAD_LCD_D09__GPIO_1_9);
	jp = gpio_get_value(MX28_PAD_LCD_D09__GPIO_1_9);

	if(jp) setenv("jp1", "on");
	else setenv("jp1", "off");

	return 0;
}

U_BOOT_CMD(readjp, 1, 1,	do_readjp,
	"Read Option Jumpers",
	""
);
