/*
 * Technologic Trafficnet BAT-433 (TS-7553) Single-board Computer
 *
 * (C) Copyright 2015-2022 Technologic Systems, Inc. dba embeddedTS
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
#include <i2c.h>

#define TRAFFICNET_EN_SDPWR		MX28_PAD_PWM3__GPIO_3_28
#define TRAFFICNET_SDBOOT_JP		MX28_PAD_LCD_D12__GPIO_1_12
#define TRAFFICNET_UBOOT_JP		MX28_PAD_LCD_D11__GPIO_1_11
#define TRAFFICNET_POWER_FAIL		MX28_PAD_SSP0_DETECT__GPIO_2_9

#define TRAFFICNET_OFFBDSPI_SELN	MX28_PAD_GPMI_RDN__GPIO_0_24
#define TRAFFICNET_EN_BOOT_FLASH	MX28_PAD_GPMI_D04__GPIO_0_4

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

	setenv("model", "7554");

	gpio_direction_input(TRAFFICNET_SDBOOT_JP);
	sdboot = gpio_get_value(TRAFFICNET_SDBOOT_JP);

	if(sdboot) setenv("jpsdboot", "off");
	else setenv("jpsdboot", "on");

	gpio_direction_input(TRAFFICNET_UBOOT_JP);
	sdboot = gpio_get_value(TRAFFICNET_UBOOT_JP);

	if(sdboot) setenv("jpuboot", "off");
	else setenv("jpuboot", "on");

	return 0;
}

int board_init(void)
{
	/* Adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	return 0;
}

static int TRAFFICNET_mmc_cd(int id) {
	return 1;
}

int board_mmc_init(bd_t *bis)
{
	int ret;
	mxs_iomux_setup_pad(TRAFFICNET_EN_SDPWR);

	gpio_direction_output(TRAFFICNET_EN_SDPWR, 1); // EN_SD_POWER#
	udelay(1000);
	gpio_direction_output(TRAFFICNET_EN_SDPWR, 0);

	/* SD card */
	ret = mxsmmc_initialize(bis, 0, NULL, TRAFFICNET_mmc_cd);
	if(ret != 0) {
		printf("SD controller initialized with %d\n", ret);
	}

	/* eMMC */
	ret = mxsmmc_initialize(bis, 1, NULL, TRAFFICNET_mmc_cd);
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

		writel(OCOTP_CTRL_RD_BANK_OPEN, &ocotp_regs->hw_ocotp_ctrl_set);

		if (mxs_wait_mask_clr(&ocotp_regs->hw_ocotp_ctrl_reg,
		  OCOTP_CTRL_BUSY, MXS_OCOTP_MAX_TIMEOUT)) {
			printf("MXS FEC: Can't get MAC from OCOTP\n");
			return -EINVAL;
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

static int set_mx28_spi(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned int mode;

	if(argc != 2) {
		printf("Usage:\n%s %s", cmdtp->name, cmdtp->help);
		return 1;
	}

	mode = simple_strtoul(argv[1], NULL, 16);
	switch(mode) {
	  case 0:
		gpio_direction_output(TRAFFICNET_EN_BOOT_FLASH, 1);
		gpio_direction_input(TRAFFICNET_OFFBDSPI_SELN);
		break;
	  case 1:
		gpio_direction_output(TRAFFICNET_EN_BOOT_FLASH, 1);
		gpio_direction_output(TRAFFICNET_OFFBDSPI_SELN, 1);
		break;
	  case 2:
		gpio_direction_output(TRAFFICNET_EN_BOOT_FLASH, 1);
		gpio_direction_output(TRAFFICNET_OFFBDSPI_SELN, 0);
		break;
	  case 3:
		gpio_direction_output(TRAFFICNET_EN_BOOT_FLASH, 0);
		gpio_direction_input(TRAFFICNET_OFFBDSPI_SELN);
		break;
	  default:
		printf("Argument must be 0, 1, 2, or 3\n");
		return 1;
		break;
	}

	return 0;
}

U_BOOT_CMD(mx28_prod, 2, 0, set_mx28_spi, 
	"Production command to set boot SPI settings",
	"[mode]\n"
	"  Where mode is:\n"
	"    0 - En. SPI CS#, 9468 switch selected\n"
	"    1 - En. SPI CS#, force on-board SPI\n"
	"    2 - En. SPI CS#, force off-board SPI\n"
	"    3 - Dis. SPI CS# output (En. use of UART 2 & 3)\n");

static int wait_for_supercaps(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned int pct;
	unsigned char buf[16] = {0};
	unsigned int check;
	unsigned int verbose = 0;

	if (argc < 2) {
		printf("Usage:\n%s %s", cmdtp->name, cmdtp->help);
		return 1;
	}

	i2c_read(0x78, 0x0, 1, &buf[0], 16);
	check = ((buf[15] >> 3) & 0x1);
	if(check) {
		printf("NO CHRG jumper is set, not waiting for SuperCaps to"
		  " charge\n");
		return 0;
	}
	if(argc == 3) verbose = simple_strtoul(argv[2], NULL, 10);
	pct = simple_strtoul(argv[1], NULL, 10);
	if(pct == 0) {
		printf("Not waiting for SuperCaps to charge\n");
		return 0;
	} else {
		printf("Waiting until SuperCaps are charged to %d%%\n", pct);
	}
	if(pct > 100) pct = 100;

	while(1) {
		i2c_read(0x78, 0x0, 1, &buf[0], 4);
		check = ((buf[2]<<8|buf[3])*1000/409*2);
		if(check >= 2500) {
			check = ((check - 2500)/23);
			if(check > 100) check = 100;
			if(verbose) printf("%d%%\n", check);
			if(check >= pct) return 0;
		} else {
			if(verbose) printf("0%%\n");
		}
		if(ctrlc()) return 1;
		udelay(1000000);
	}
}

U_BOOT_CMD(wait_chrg, 3, 0, wait_for_supercaps,
	"Wait until SuperCaps have a specific charge percentage",
	"[percentage] [verbose]\n"
	"  Percentage can be 0 to 100. 0 means no delay\n"
	"  Verbose can be a 1 to output percentage every second; 0 or not\n"
	"    passed to disable this output and wait silently\n");
