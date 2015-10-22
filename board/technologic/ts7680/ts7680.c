/*
 * Technologic TS-7680 Single-board Computer
 *
 * (C) Copyright 2015 Technologic Systems
 * Based on work by:
 * Stuart Longland of VRT Systems <stuartl@vrt.com.au>
 *
 * Based on mx28evk.c:
 * Freescale MX28EVK board
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux-mx28.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <linux/mii.h>
#include <miiphy.h>
#include <netdev.h>
#include <errno.h>
#include <spi.h>
#include <fpga.h>
#include <lattice.h>
#include <i2c.h>

#define TS7680_EN_SDPWR     MX28_PAD_PWM3__GPIO_3_28
#define TS7680_SDBOOT_JP    MX28_PAD_LCD_D12__GPIO_1_12
#define TS7680_POWER_FAIL	MX28_PAD_SSP0_DETECT__GPIO_2_9

DECLARE_GLOBAL_DATA_PTR;
int random_mac = 0;

void mx28_adjust_mac(int dev_id, unsigned char *mac)
{
	mac[0] = 0x00;
	mac[1] = 0xD0;
	mac[2] = 0x69;
}

static void enable_fpga_clk(void) 
{
	// Clear PWM clk gate
	writel(0x20000000, 0x80040088); // HW_CLKCTRL_XTAL_SET

	// Take PWM out of reset
	writel(0x80000000, 0x80064008); // HW_PWM_CTRL_CLR
	writel(0x40000000, 0x80064008); // HW_PWM_CTRL_CLR

	// Set PWM active and period
	writel(0x10000, 0x80064050); // HW_PWM_ACTIVE2
	writel(0xb0001, 0x80064060); // HW_PWM_PERIOD2

	// Enable PWM output
	writel(0x4, 0x80064004); // HW_PWM_CTRL_SET
	
}


#if defined(CONFIG_FPGA)

static void ts7680_jtag_init(void)
{
	gpio_direction_output(CONFIG_FPGA_TDI, 1);
	gpio_direction_output(CONFIG_FPGA_TCK, 1);
	gpio_direction_output(CONFIG_FPGA_TMS, 1);
	gpio_direction_input(CONFIG_FPGA_TDO);
	return;
}

static void ts7680_fpga_jtag_set_tdi(int value)
{
	gpio_set_value(CONFIG_FPGA_TDI, value);
}

static void ts7680_fpga_jtag_set_tms(int value)
{
	gpio_set_value(CONFIG_FPGA_TMS, value);
}

static void ts7680_fpga_jtag_set_tck(int value)
{
	gpio_set_value(CONFIG_FPGA_TCK, value);
}

static int ts7680_fpga_jtag_get_tdo(void)
{
	return gpio_get_value(CONFIG_FPGA_TDO);
}

lattice_board_specific_func ts7680_fpga_fns = {
	ts7680_jtag_init,
	ts7680_fpga_jtag_set_tdi,
	ts7680_fpga_jtag_set_tms,
	ts7680_fpga_jtag_set_tck,
	ts7680_fpga_jtag_get_tdo
};

Lattice_desc ts7680_fpga = {
	Lattice_XP2,
	lattice_jtag_mode,
	589012,
	(void *) &ts7680_fpga_fns,
	NULL,
	0,
	"machxo_2_cb132"
};

int ts7680_fpga_init(void)
{
	fpga_init();
	fpga_add(fpga_lattice, &ts7680_fpga);

	return 0;
}

#endif // CONFIG_FPGA

int board_early_init_f(void)
{
	/* IO0 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK0, 480000);
	/* IO1 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK1, 480000);

	/* SSP clocks at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK0, 96000, 0);
	/* SSP clocks at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK1, 96000, 0);
	/* SSP2 clock at 160MHz */
	mxs_set_sspclk(MXC_SSPCLK2, 160000, 0);

	return 0;
}

int dram_init(void)
{
	return mxs_dram_init();
}

int misc_init_r(void)
{
	int sdboot = 0;

	setenv("model", "7680");

	gpio_direction_input(TS7680_SDBOOT_JP);
	sdboot = gpio_get_value(TS7680_SDBOOT_JP);

	if(sdboot) setenv("jpsdboot", "off");
	else setenv("jpsdboot", "on");

#if defined(CONFIG_FPGA)
	ts7680_fpga_init();
#endif
	enable_fpga_clk();

	return 0;
}

int board_init(void)
{
	/* Adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	return 0;
}

static int ts7680_mmc_cd(int id) {
	return 1;
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	mxs_iomux_setup_pad(TS7680_EN_SDPWR);

	gpio_direction_output(TS7680_EN_SDPWR, 1); // EN_SD_POWER#
	udelay(1000);
	gpio_direction_output(TS7680_EN_SDPWR, 0);

	/* SD card */
	ret = mxsmmc_initialize(bis, 0, NULL, ts7680_mmc_cd);
	if(ret != 0) {
		printf("SD controller initialized with %d\n", ret);
	}

	/* eMMC */
	ret = mxsmmc_initialize(bis, 1, NULL, ts7680_mmc_cd);
	if(ret != 0) {
		printf("eMMC controller initialized with %d\n", ret);
	}

	return 0;
}

#ifdef	CONFIG_CMD_NET

int board_eth_init(bd_t *bis)
{
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;
	struct eth_device *dev;
	int ret;
	uchar enetaddr[6];
	uint8_t val = 0x2;

	/* Take switch out of reset */
	i2c_write(0x28, 0x2b, 2, &val, 1);

	ret = cpu_eth_init(bis);
	if (ret)
		return ret;

	/* MX28EVK uses ENET_CLK PAD to drive FEC clock */
	writel(CLKCTRL_ENET_TIME_SEL_RMII_CLK | CLKCTRL_ENET_CLK_OUT_EN,
	       &clkctrl_regs->hw_clkctrl_enet);

	ret = fecmxc_initialize_multi(bis, 0, 0, MXS_ENET0_BASE);
	if (ret) {
		puts("FEC MXS: Unable to init FEC0\n");
		return ret;
	}

	dev = eth_get_dev_by_name("FEC0");
	if (!dev) {
		puts("FEC MXS: Unable to get FEC0 device entry\n");
		return -EINVAL;
	}

	eth_parse_enetaddr(getenv("ethaddr"), enetaddr);
        if (!enetaddr[3] && !enetaddr[4] && !enetaddr[5]) {
                printf("No MAC address set in fuses.  Using random mac address.\n");
                eth_random_addr(enetaddr);
                random_mac = 1;
                if (eth_setenv_enetaddr("ethaddr", enetaddr)) {
                        printf("Failed to set ethernet address\n");
                }
        }

	return ret;
}

#endif
