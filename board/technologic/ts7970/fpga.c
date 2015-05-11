/*
 * (C) Copyright 2010
 * Stefano Babic, DENX Software Engineering, sbabic@denx.de
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/gpio.h>
#include <fpga.h>
#include <lattice.h>

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_FPGA)

static void ts7970_jtag_init(void)
{
	return;
}

static void ts7970_fpga_jtag_set_tdi(int value)
{
	gpio_set_value(CONFIG_FPGA_TDI, value);
}

static void ts7970_fpga_jtag_set_tms(int value)
{
	gpio_set_value(CONFIG_FPGA_TMS, value);
}

static void ts7970_fpga_jtag_set_tck(int value)
{
	gpio_set_value(CONFIG_FPGA_TCK, value);
}

static int ts7970_fpga_jtag_get_tdo(void)
{
	return gpio_get_value(CONFIG_FPGA_TDO);
}

lattice_board_specific_func ts7970_fpga_fns = {
	ts7970_jtag_init,
	ts7970_fpga_jtag_set_tdi,
	ts7970_fpga_jtag_set_tms,
	ts7970_fpga_jtag_set_tck,
	ts7970_fpga_jtag_get_tdo
};

Lattice_desc ts7970_fpga = {
	Lattice_XP2,
	lattice_jtag_mode,
	356519,
	(void *) &ts7970_fpga_fns,
	NULL,
	0,
	"machxo_2_cb132"
};

int ts7970_fpga_init(void)
{
	fpga_init();
	fpga_add(fpga_lattice, &ts7970_fpga);

	return 0;
}

#endif
