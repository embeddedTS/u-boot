/*
 * Technologic TS-7400-V2 Single-board Computer
 *
 * (C) Copyright 2018-2022 Technologic Systems, Inc. dba embeddedTS
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

#define TS7400V2_EN_SDPWR	MX28_PAD_PWM3__GPIO_3_28
#define TS7400V2_SDBOOT_JP	MX28_PAD_LCD_D00__GPIO_1_0
#define TS7400V2_ENET_RSTn	MX28_PAD_SSP0_DETECT__GPIO_2_9

DECLARE_GLOBAL_DATA_PTR;
int random_mac = 0;

void mx28_adjust_mac(int dev_id, unsigned char *mac)
{
	mac[0] = 0x00;
	mac[1] = 0xD0;
	mac[2] = 0x69;
}


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

	setenv("model", "7400v2");

	gpio_direction_input(TS7400V2_SDBOOT_JP);
	sdboot = gpio_get_value(TS7400V2_SDBOOT_JP);

	if(sdboot) setenv("jpsdboot", "on");
	else setenv("jpsdboot", "off");

	hw_watchdog_init();

	return 0;
}

int board_init(void)
{
	/* Adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	return 0;
}

static int ts7400v2_mmc_cd(int id) {
	return 1;
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	mxs_iomux_setup_pad(TS7400V2_EN_SDPWR);

	gpio_direction_output(TS7400V2_EN_SDPWR, 1); // EN_SD_POWER#
	udelay(1000);
	gpio_direction_output(TS7400V2_EN_SDPWR, 0);

	/* SD card */
	ret = mxsmmc_initialize(bis, 0, NULL, ts7400v2_mmc_cd);
	if(ret != 0) {
		printf("SD controller initialized with %d\n", ret);
	}

	/* eMMC */
	ret = mxsmmc_initialize(bis, 1, NULL, ts7400v2_mmc_cd);
	if(ret != 0) {
		printf("eMMC controller initialized with %d\n", ret);
	}

	return 0;
}

#ifdef	CONFIG_CMD_NET

#define MXS_OCOTP_MAX_TIMEOUT   1000000

int board_eth_init(bd_t *bis)
{
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;
	struct eth_device *dev;
	struct mxs_ocotp_regs *ocotp_regs =
	  (struct mxs_ocotp_regs *)MXS_OCOTP_BASE;
	uint32_t data;

	int ret;
	uchar enetaddr[6];
	uint8_t val = 0x2;

	/* Take switch out of reset */

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
		printf("No MAC addr. set in fuses.  Using random MAC addr.\n");

		writel(OCOTP_CTRL_RD_BANK_OPEN, &ocotp_regs->hw_ocotp_ctrl_set);

		if (mxs_wait_mask_clr(&ocotp_regs->hw_ocotp_ctrl_reg,
		  OCOTP_CTRL_BUSY, MXS_OCOTP_MAX_TIMEOUT)) {
			printf("MXS FEC: Can't get MAC from OCOTP\n");
			return;
		}

		data = readl(&ocotp_regs->hw_ocotp_ops2);

		enetaddr[3] = 0x4f;
		enetaddr[4] = (data >> 8) & 0xff;
		enetaddr[5] = data & 0xff;
		mx28_adjust_mac(0, enetaddr);

		if (eth_setenv_enetaddr("ethaddr", enetaddr)) {
			printf("Failed to set ethernet address\n");
		}
	}

	return ret;
}

#endif
