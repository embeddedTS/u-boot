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

DECLARE_GLOBAL_DATA_PTR;
int random_mac = 0;

void mx28_adjust_mac(int dev_id, unsigned char *mac)
{
	mac[0] = 0x00;
	mac[1] = 0xD0;
	mac[2] = 0x69;
}

/*
 * Functions
 */
int board_early_init_f(void)
{
	/* IO0 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK0, 480000);
	/* IO1 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK1, 480000);

	/* SSP0 clock at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK0, 96000, 0);
	/* SSP2 clock at 160MHz */
	mxs_set_sspclk(MXC_SSPCLK2, 160000, 0);

#ifdef	CONFIG_CMD_USB
	mxs_iomux_setup_pad(MX28_PAD_SSP2_SS1__USB1_OVERCURRENT);
	mxs_iomux_setup_pad(MX28_PAD_AUART2_RX__GPIO_3_8 |
			MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_NOPULL);
	gpio_direction_output(MX28_PAD_AUART2_RX__GPIO_3_8, 1);
#endif

	mxs_iomux_setup_pad(MX28_PAD_SSP0_DETECT__GPIO_2_9);

	return 0;
}

int dram_init(void)
{
	return mxs_dram_init();
}

int misc_init_r(void)
{
	struct mxs_spl_data *data = (struct mxs_spl_data *)
	  ((CONFIG_SYS_TEXT_BASE - sizeof(struct mxs_spl_data)) & ~0xf);

	setenv_hex("bootmode", mxs_boot_modes[data->boot_mode_idx].boot_pads);
	setenv("model", "7695");
}


int board_init(void)
{
	/* Adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	return 0;
}

#ifdef	CONFIG_CMD_MMC
static int ts7400_mmc_cd(int id) {
	return 1;
}
int board_mmc_init(bd_t *bis)
{
	/* Configure MMC0 Power Enable */
	int ret = mxsmmc_initialize(bis, 0, NULL, ts7400_mmc_cd);
	//if (ret)
	return ret;
	//return mxsmmc_initialize(bis, 1, NULL, ts7400_mmc_cd);
}
#endif

#ifdef	CONFIG_CMD_NET

int board_eth_init(bd_t *bis)
{
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;
	struct eth_device *dev;
	int ret;
	uchar enetaddr[6];

	ret = cpu_eth_init(bis);
	if (ret)
		return ret;

	/* MX28EVK uses ENET_CLK PAD to drive FEC clock */
	writel(CLKCTRL_ENET_TIME_SEL_RMII_CLK | CLKCTRL_ENET_CLK_OUT_EN,
	       &clkctrl_regs->hw_clkctrl_enet);

	/* Reset FEC PHYs */
	gpio_direction_output(MX28_PAD_SSP0_DETECT__GPIO_2_9, 0);
	udelay(15000);
	gpio_set_value(MX28_PAD_SSP0_DETECT__GPIO_2_9, 1);

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
