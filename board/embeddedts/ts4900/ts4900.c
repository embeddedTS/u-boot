// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016-2026 Technologic Systems, Inc. dba embeddedTS
 *
 * Author: Mark Featherston <mark@embeddedTS.com>
 */
#include <image.h>
#include <init.h>
#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/mach-imx/spi.h>
#include <asm/sections.h>
#include <env.h>
#include <i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <asm/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/spi.h>
#include <asm/mach-imx/video.h>
#include <mmc.h>
#include <fsl_esdhc_imx.h>
#include <miiphy.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <input.h>
#include <usb.h>
#include <usb/ehci-ci.h>

#include "ice40.h"
#include "strap_decode.h"
#include "../common/parse_gpio_straps.h"
#include "../common/rtc_workaround.h"

DECLARE_GLOBAL_DATA_PTR;

#define GPIO_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL (PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | 	\
	PAD_CTL_HYS | PAD_CTL_ODE | PAD_CTL_SRE_FAST)

//#define DISP0_PWR_EN	IMX_GPIO_NR(1, 21)
#define TS4900_SPI_CS		IMX_GPIO_NR(3, 19)
#define TS4900_EN_SDPWR		IMX_GPIO_NR(2, 28)
#define TS4900_ENRTC		IMX_GPIO_NR(3, 23)
#define TS4900_SCL		IMX_GPIO_NR(3, 21)
#define TS4900_SDA		IMX_GPIO_NR(3, 28)
#define TS4900_PHY_RST		IMX_GPIO_NR(4, 20)
#define TS4900_RGMII_RXC	IMX_GPIO_NR(6, 30)
#define TS4900_RGMII_RD0	IMX_GPIO_NR(6, 25)
#define TS4900_RGMII_RD1	IMX_GPIO_NR(6, 27)
#define TS4900_RGMII_RD2	IMX_GPIO_NR(6, 28)
#define TS4900_RGMII_RD3	IMX_GPIO_NR(6, 29)
#define TS4900_RGMII_RX_CTL	IMX_GPIO_NR(6, 24)
#define TS4900_REVSTRAP		IMX_GPIO_NR(2, 11)
#define TS4900_REVSTRAPD	IMX_GPIO_NR(6, 5)
#define TS4900_REVSTRAPE	IMX_GPIO_NR(1, 29)
#if 0
#define TS4900_EN_5V		IMX_GPIO_NR(2, 22)
#define TS4900_OFFBD_RST	IMX_GPIO_NR(2, 21)
#define TS4900_SDBOOT		IMX_GPIO_NR(2, 26)
#define TS4900_SCL		IMX_GPIO_NR(3, 21)
#define TS4900_SDA		IMX_GPIO_NR(3, 28)
#define TS4900_REVSTRAP		IMX_GPIO_NR(2, 11)
#define TS4900_REVSTRAPD	IMX_GPIO_NR(6, 5)
#define TS4900_REVSTRAPE	IMX_GPIO_NR(1, 29)
#define TS4900_SPI_CS		IMX_GPIO_NR(3, 19)
#define TS4900_PHY_RST		IMX_GPIO_NR(4, 20)
#define TS4900_RGMII_RXC	IMX_GPIO_NR(6, 30)
#define TS4900_RGMII_RD0	IMX_GPIO_NR(6, 25)
#define TS4900_RGMII_RD1	IMX_GPIO_NR(6, 27)
#define TS4900_RGMII_RD2	IMX_GPIO_NR(6, 28)
#define TS4900_RGMII_RD3	IMX_GPIO_NR(6, 29)
#define TS4900_RGMII_RX_CTL	IMX_GPIO_NR(6, 24)
#define TS4900_OTG_ID		IMX_GPIO_NR(1, 1)
#define TS4900_WIFI_EN		IMX_GPIO_NR(1, 26)
#define TS4900_BT_EN		IMX_GPIO_NR(1, 27)
#define TS4900_SD1_D0		IMX_GPIO_NR(1, 16)
#define TS4900_SD1_D1		IMX_GPIO_NR(1, 17)
#define TS4900_SD1_D2		IMX_GPIO_NR(1, 19)
#define TS4900_SD1_D3		IMX_GPIO_NR(1, 21)
#define TS4900_SD1_CMD		IMX_GPIO_NR(1, 18)
#define TS4900_SD1_CLK		IMX_GPIO_NR(1, 20)
#endif

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

static iomux_v3_cfg_t const enet_pads1[] = {
	/* pin 35 - 1 (PHY_AD2) on reset */
	IOMUX_PADS(PAD_RGMII_RXC__GPIO6_IO30            | MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* pin 32 - 1 - (MODE0) all */
	IOMUX_PADS(PAD_RGMII_RD0__GPIO6_IO25            | MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* pin 31 - 1 - (MODE1) all */
	IOMUX_PADS(PAD_RGMII_RD1__GPIO6_IO27            | MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* pin 28 - 1 - (MODE2) all */
	IOMUX_PADS(PAD_RGMII_RD2__GPIO6_IO28            | MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* pin 27 - 1 - (MODE3) all */
	IOMUX_PADS(PAD_RGMII_RD3__GPIO6_IO29            | MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
	IOMUX_PADS(PAD_RGMII_RX_CTL__GPIO6_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL)),

	// PHY RESET
	IOMUX_PADS(PAD_DI0_PIN4__GPIO4_IO20             | MUX_PAD_CTRL(NO_PAD_CTRL)),
};


#if 0

static iomux_v3_cfg_t const usdhc2_pads[] = {
	IOMUX_PADS(PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D4__SD2_DATA4	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D5__SD2_DATA5	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D6__SD2_DATA6	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D7__SD2_DATA7	| MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D2__GPIO2_IO02	| MUX_PAD_CTRL(NO_PAD_CTRL)), /* CD */
};

static iomux_v3_cfg_t const usdhc3_pads[] = {
	IOMUX_PADS(PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT4__SD3_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT5__SD3_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT6__SD3_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT7__SD3_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_NANDF_D0__GPIO2_IO00    | MUX_PAD_CTRL(NO_PAD_CTRL)), /* CD */
};

static iomux_v3_cfg_t const usdhc4_pads[] = {
	IOMUX_PADS(PAD_SD4_CLK__SD4_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_CMD__SD4_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT0__SD4_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT1__SD4_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT2__SD4_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT3__SD4_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT4__SD4_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT5__SD4_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT6__SD4_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
	IOMUX_PADS(PAD_SD4_DAT7__SD4_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL)),
};

static iomux_v3_cfg_t const ecspi1_pads[] = {
	IOMUX_PADS(PAD_KEY_COL0__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_COL1__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW0__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_KEY_ROW1__GPIO4_IO09 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const rgb_pads[] = {
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN15__IPU1_DI0_PIN15 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DI0_PIN4__IPU1_DI0_PIN04 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT0__IPU1_DISP0_DATA00 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT1__IPU1_DISP0_DATA01 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT2__IPU1_DISP0_DATA02 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT3__IPU1_DISP0_DATA03 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT4__IPU1_DISP0_DATA04 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT5__IPU1_DISP0_DATA05 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT6__IPU1_DISP0_DATA06 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT7__IPU1_DISP0_DATA07 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT8__IPU1_DISP0_DATA08 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT9__IPU1_DISP0_DATA09 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT10__IPU1_DISP0_DATA10 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT11__IPU1_DISP0_DATA11 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT12__IPU1_DISP0_DATA12 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT13__IPU1_DISP0_DATA13 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT14__IPU1_DISP0_DATA14 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT15__IPU1_DISP0_DATA15 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT16__IPU1_DISP0_DATA16 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT17__IPU1_DISP0_DATA17 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT18__IPU1_DISP0_DATA18 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT19__IPU1_DISP0_DATA19 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT20__IPU1_DISP0_DATA20 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT21__IPU1_DISP0_DATA21 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT22__IPU1_DISP0_DATA22 | MUX_PAD_CTRL(NO_PAD_CTRL)),
	IOMUX_PADS(PAD_DISP0_DAT23__IPU1_DISP0_DATA23 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const bl_pads[] = {
	IOMUX_PADS(PAD_SD1_DAT3__GPIO1_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static void enable_backlight(void)
{
	SETUP_IOMUX_PADS(bl_pads);
	gpio_request(DISP0_PWR_EN, "Display Power Enable");
	gpio_direction_output(DISP0_PWR_EN, 1);
}

static void enable_rgb(struct display_info_t const *dev)
{
	SETUP_IOMUX_PADS(rgb_pads);
	enable_backlight();
}

static void enable_lvds(struct display_info_t const *dev)
{
	enable_backlight();
}

iomux_v3_cfg_t const di0_pads[] = {
	IOMUX_PADS(PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK),	/* DISP0_CLK */
	IOMUX_PADS(PAD_DI0_PIN2__IPU1_DI0_PIN02),		/* DISP0_HSYNC */
	IOMUX_PADS(PAD_DI0_PIN3__IPU1_DI0_PIN03),		/* DISP0_VSYNC */
};
#endif

#ifdef CONFIG_FSL_ESDHC_IMX
struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC2_BASE_ADDR},
	{USDHC3_BASE_ADDR},
};

int board_mmc_get_env_dev(int devno)
{
	return devno - 1;
}

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	printf("KRIS: called getcd\n");
	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR: // microSD
	case USDHC3_BASE_ADDR: // eMMC
		ret = 1;
		break;
	default:
		ret = -1;
		break;
	}

	return ret;
}

void board_mmc_power_init(void)
{
	printf("KRIS: mmc power init\n");
	gpio_request(TS4900_EN_SDPWR, "sd-vmmc-en");
	gpio_direction_output(TS4900_EN_SDPWR, 1);
	/* XXX: TODO: Verify if this is needed. A reset may have the power below
	 * 0.5 V for longer than 1 ms anyway. If not, a scope should reveal
	 * roughly how long it takes for the rail to collapse and re-establish.
	 */
	udelay(15000);
	gpio_direction_output(TS4900_EN_SDPWR, 0);
}

int board_mmc_init(struct bd_info *bis)
{
	/* XXX: TODO: Put the iomux here later maybe for sd power control */
	/* XXX: NOTE: It _should_ be possible down the road to be able to
	 * forceably unbind the MMC controller from linux which _should_ cause
	 * it to correctly disable the regulator. This could be used for
	 * forcing a power cycle! Investigate this later once we get further
	 * along with LTS support.
	 *
	 * This actually needs to be a note to deal with LATER as the 4900
	 * has microSD power control as its own thing but eMMC is powered by
	 * 3.3 V
	 */

	return 0;
	#if 0
	struct src *psrc = (struct src *)SRC_BASE_ADDR;
	unsigned reg = readl(&psrc->sbmr1) >> 11;
	/*
	 * Upon reading BOOT_CFG register the following map is done:
	 * Bit 11 and 12 of BOOT_CFG register can determine the current
	 * mmc port
	 * 0x1                  SD1
	 * 0x2                  SD2
	 * 0x3                  SD4
	 */

	switch (reg & 0x3) {
	case 0x1:
		SETUP_IOMUX_PADS(usdhc2_pads);
		usdhc_cfg[0].esdhc_base = USDHC2_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	case 0x2:
		SETUP_IOMUX_PADS(usdhc3_pads);
		usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	case 0x3:
		SETUP_IOMUX_PADS(usdhc4_pads);
		usdhc_cfg[0].esdhc_base = USDHC4_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC4_CLK);
		gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
		break;
	}

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}
#endif

#if defined(CONFIG_VIDEO_IPUV3)
#if 0
static void disable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;

	int reg = readl(&iomux->gpr[2]);

	reg &= ~(IOMUXC_GPR2_LVDS_CH0_MODE_MASK |
		 IOMUXC_GPR2_LVDS_CH1_MODE_MASK);

	writel(reg, &iomux->gpr[2]);
}

static void do_enable_hdmi(struct display_info_t const *dev)
{
	disable_lvds(dev);
	imx_enable_hdmi_phy();
}

struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Hannstar-XGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15384,
		.left_margin    = 160,
		.right_margin   = 24,
		.upper_margin   = 29,
		.lower_margin   = 3,
		.hsync_len      = 136,
		.vsync_len      = 6,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_hdmi,
	.enable	= do_enable_hdmi,
	.mode	= {
		.name           = "HDMI",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15384,
		.left_margin    = 160,
		.right_margin   = 24,
		.upper_margin   = 29,
		.lower_margin   = 3,
		.hsync_len      = 136,
		.vsync_len      = 6,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= {
		.name           = "SEIKO-WVGA",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 29850,
		.left_margin    = 89,
		.right_margin   = 164,
		.upper_margin   = 23,
		.lower_margin   = 10,
		.hsync_len      = 10,
		.vsync_len      = 10,
		.sync           = 0,
		.vmode          = FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;

	/* Setup HSYNC, VSYNC, DISP_CLK for debugging purposes */
	SETUP_IOMUX_PADS(di0_pads);

	enable_ipu_clock();
	imx_setup_hdmi();

	/* Turn on LDB0, LDB1, IPU,IPU DI0 clocks */
	reg = readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK | MXC_CCM_CCGR3_LDB_DI1_MASK;
	writel(reg, &mxc_ccm->CCGR3);

	/* set LDB0, LDB1 clk select to 011/011 */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK
		 | MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_MASK);
	reg |= (3 << MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET)
	      | (3 << MXC_CCM_CS2CDR_LDB_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->cs2cdr);

	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV | MXC_CCM_CSCMR2_LDB_DI1_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI1_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);

	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     | IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     | IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     | IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     | IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     | IOMUXC_GPR2_LVDS_CH0_MODE_DISABLED
	     | IOMUXC_GPR2_LVDS_CH1_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = (reg & ~(IOMUXC_GPR3_LVDS1_MUX_CTL_MASK
			| IOMUXC_GPR3_HDMI_MUX_CTL_MASK))
	    | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0
	       << IOMUXC_GPR3_LVDS1_MUX_CTL_OFFSET);
	writel(reg, &iomux->gpr[3]);
}
#endif
#endif /* CONFIG_VIDEO_IPUV3 */
struct display_info_t const displays[] = {0};
size_t display_count = 0;

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

#ifdef CONFIG_USB_EHCI_MX6
static void setup_usb(void)
{
	/*
	 * set daisy chain for otg_pin_id on 6q.
	 * for 6dl, this bit is reserved
	 */
	imx_iomux_set_gpr_register(1, 13, 1, 0);
}
#endif

#if 0
static int setup_fec(void)
{
        struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
        struct anatop_regs *anatop = (struct anatop_regs *)ANATOP_BASE_ADDR;
        int reg, ret;

        /* Use 125MHz anatop loopback REF_CLK1 for ENET1 */
        clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK, 0);

        ret = enable_fec_anatop_clock(0, ENET_125MHZ);
        if (ret)
                return ret;

        imx_iomux_v3_setup_multiple_pads(phy_control_pads,
                                         ARRAY_SIZE(phy_control_pads));

        /* Enable the ENET power, active low */
        gpio_request(IMX_GPIO_NR(2, 6), "enet_rst");
        gpio_direction_output(IMX_GPIO_NR(2, 6) , 0);

        /* Reset AR8031 PHY */
        gpio_request(IMX_GPIO_NR(2, 7), "phy_rst");
        gpio_direction_output(IMX_GPIO_NR(2, 7) , 0);
        mdelay(10);
        gpio_set_value(IMX_GPIO_NR(2, 7), 1);

        reg = readl(&anatop->pll_enet);
        reg |= BM_ANADIG_PLL_ENET_REF_25M_ENABLE;
        writel(reg, &anatop->pll_enet);

        return 0;
}

int board_eth_init(struct bd_info *bis)
{
        imx_iomux_v3_setup_multiple_pads(fec1_pads, ARRAY_SIZE(fec1_pads));
        setup_fec();

        return cpu_eth_init(bis);
}
#endif

/* Must be called early in boot, either late_init() or misc_init_r(), before
 * calls to eth_init() are ultimately made. We rely on the devicetree to set
 * the real final ethernet MAC/MDIO/MII IOMUX settings, but, need to control
 * these pins as GPIO to force a proper bootstrapping when un-resetting the
 * PHYs
 */
static void early_phy_strap_reset(void)
{
	SETUP_IOMUX_PADS(enet_pads1);

        // Assert reset
	gpio_request(TS4900_PHY_RST, "phy");
        gpio_direction_output(TS4900_PHY_RST, 1);

        gpio_request(TS4900_RGMII_RXC, "phy");
        gpio_request(TS4900_RGMII_RD0, "phy");
        gpio_request(TS4900_RGMII_RD1, "phy");
        gpio_request(TS4900_RGMII_RD2, "phy");
        gpio_request(TS4900_RGMII_RD3, "phy");
        gpio_request(TS4900_RGMII_RX_CTL, "phy");

        gpio_direction_output(TS4900_RGMII_RXC, 1);
        gpio_direction_output(TS4900_RGMII_RD0, 1);
        gpio_direction_output(TS4900_RGMII_RD1, 1);
        gpio_direction_output(TS4900_RGMII_RD2, 1);
        gpio_direction_output(TS4900_RGMII_RD3, 1);
        gpio_direction_output(TS4900_RGMII_RX_CTL, 1);

        /* Need delay at least 10ms according to KSZ9031 spec */
        udelay(10000);

        // De-assert reset
        gpio_direction_output(TS4900_PHY_RST, 0);

        /* Need 100us delay to exit from reset. */
	/* XXX: datasheet doesn't spec deassert wait time */
        udelay(1000 * 100);
}

int board_init(void)
{
	/* XXX: What is the point of this? */
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#if defined(CONFIG_VIDEO_IPUV3)
	//setup_display();
#endif
#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif

	return 0;
}

#if 0
int power_init_board(void)
{
	struct udevice *dev;
	unsigned int reg;
	int ret;

	ret = pmic_get("pfuze100@8", &dev);
	if (ret == -ENODEV)
		return 0;

	if (ret != 0)
		return ret;

	ret = pfuze_mode_init(dev, APS_PFM);
	if (ret < 0)
		return ret;

	/* Increase VGEN3 from 2.5 to 2.8V */
	reg = pmic_reg_read(dev, PFUZE100_VGEN3VOL);
	reg &= ~LDO_VOL_MASK;
	reg |= LDOB_2_80V;
	pmic_reg_write(dev, PFUZE100_VGEN3VOL, reg);

	/* Increase VGEN5 from 2.8 to 3V */
	reg = pmic_reg_read(dev, PFUZE100_VGEN5VOL);
	reg &= ~LDO_VOL_MASK;
	reg |= LDOB_3_00V;
	pmic_reg_write(dev, PFUZE100_VGEN5VOL, reg);

	return 0;
}
#endif

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? (TS4900_SPI_CS) : -1;
}
#endif

#ifdef CONFIG_CMD_BMODE
/* XXX: We probably don't want this in production, but, setting it up for use
 * could be beneficial for customers testing new bootloaders! e.g. set it to
 * USB mode and force serial mode that way/
 */
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"sd2",	 MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{"sd3",	 MAKE_CFGVAL(0x40, 0x30, 0x00, 0x00)},
	/* 8 bit bus width */
	{"emmc", MAKE_CFGVAL(0x60, 0x58, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "ts4900");

	if (is_mx6dqp())
		env_set("board_rev", "MX6QP");
	else if (is_mx6dq())
		env_set("board_rev", "MX6Q");
	else if (is_mx6sdl())
		env_set("board_rev", "MX6DL");
#endif

	early_phy_strap_reset();

	return 0;
}

#ifdef CONFIG_XPL_BUILD
#include <asm/arch/mx6-ddr.h>
#include <spl.h>
#include <linux/libfdt.h>

#ifdef CONFIG_SPL_OS_BOOT
/* XXX: used for falcon boot */
int spl_start_uboot(void)
{
	return 0;
}
#endif

static struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		/* XXX: the IOMUX_PADS() macro causes an issue here?? */
		.i2c_mode  = MX6Q_PAD_EIM_D21__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_EIM_D21__GPIO3_IO21 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = TS4900_SCL
	},
	.sda = {
		.i2c_mode = MX6Q_PAD_EIM_D28__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6Q_PAD_EIM_D28__GPIO3_IO28 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = TS4900_SDA
	},
};

void fpga_program(void)
{
	/* Set up SPI IOMUX for booting from SPI flash */
	SETUP_IOMUX_PADS(ecspi1_pads);

	/* Get ready to program FPGA */
	// Enable ECSPI2 clock
	setbits_le32(CCM_CCGR1, MXC_CCM_CCGR1_ECSPI2S_MASK);

	SETUP_IOMUX_PADS(fpga_pads);

	/* Program FPGA */
	do_ice40_load();
	/* XXX: TODO: NOTE! Need to be mindful of what to do if FPGA programming
	 * fails. Should we try again? Reboot? Assume really crap RAM values?
	 */
}

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0x00C03F3F, &ccm->CCGR0);
	writel(0x0030FC03, &ccm->CCGR1);
	writel(0x0FFFC000, &ccm->CCGR2);
	writel(0x3FF00000, &ccm->CCGR3);
	writel(0x00FFF300, &ccm->CCGR4);
	writel(0x0F0000C3, &ccm->CCGR5);
	writel(0x000003FF, &ccm->CCGR6);
}

/* XXX: Future note before finalizing this file:
 * https://github.com/u-boot/u-boot/commit/3b30eece271cfc4096c2d20048c89e8bed0bbbfd
 * The order of this table is based on the TS-4900 from the original 2014 U-Boot
 * which used a handful of .cfg files and included them together to get generic
 * imx6q setup and then RAM layout specific setup. This ultimately differed
 * in ordering than the mx6sabersd dcd tables, and it would appear from the
 * above commit that the order did change for SPL to resolve some potential bugs.
 * Bear this in mind and consider re-testing RAM calibration.
 */
static int ts4900_1000mhz_4x256mx16_dcd_table[] = {
	0x020e05a8, 0x00000030,
	0x020e05b0, 0x00000030,
	0x020e0524, 0x00000030,
	0x020e051c, 0x00000030,
	0x020e0518, 0x00000030,
	0x020e050c, 0x00000030,
	0x020e05b8, 0x00000030,
	0x020e05c0, 0x00000030,
	0x020e0784, 0x00000030,
	0x020e0788, 0x00000030,
	0x020e0794, 0x00000030,
	0x020e079c, 0x00000030,
	0x020e07a0, 0x00000030,
	0x020e07a4, 0x00000030,
	0x020e07a8, 0x00000030,
	0x020e0748, 0x00000030,
	0x020e074c, 0x00000030,
	0x020e078c, 0x00000030,
	0x020e05ac, 0x00020030,
	0x020e05b4, 0x00020030,
	0x020e0528, 0x00020030,
	0x020e0520, 0x00020030,
	0x020e0514, 0x00020030,
	0x020e0510, 0x00020030,
	0x020e05bc, 0x00020030,
	0x020e05c4, 0x00020030,
	0x020e056c, 0x00020030,
	0x020e0578, 0x00020030,
	0x020e0588, 0x00020030,
	0x020e0594, 0x00020030,
	0x020e057c, 0x00020030,
	0x020e0590, 0x00003000,
	0x020e0598, 0x00003000,
	0x020e059c, 0x00003030,
	0x020e05a0, 0x00003030,
	0x020e0750, 0x00020000,
	0x020e0774, 0x00020000,
	0x020e0758, 0x00000000,
	0x020e058c, 0x00000000,
	0x020e0798, 0x000c0000,
	0x021b081c, 0x33333333,
	0x021b0820, 0x33333333,
	0x021b0824, 0x33333333,
	0x021b0828, 0x33333333,
	0x021b481c, 0x33333333,
	0x021b4820, 0x33333333,
	0x021b4824, 0x33333333,
	0x021b4828, 0x33333333,
	0x021b0018, 0x00081740,
	0x021b001c, 0x00008000,
	0x021b0004, 0x00020036,
	0x021b000c, 0x898e7974,
	0x021b0010, 0xdb538f64,
	0x021b0014, 0x01ff00db,
	0x021b002c, 0x000026d2,
	0x021b0030, 0x008e1023,
	0x021b0008, 0x09444040,
	0x021b0004, 0x00025576,
	0x021b0040, 0x00000047,
	0x021b0000, 0x841a0000,
	0x021b001c, 0x04088032,
	0x021b001c, 0x00008033,
	0x021b001c, 0x00428031,
	0x021b001c, 0x19308030,
	0x021b001c, 0x04008040,
	0x021b0800, 0xa1390003,
	0x021b4800, 0xa1390003,
	0x021b0020, 0x00007800,
	0x021b0818, 0x00022227,
	0x021b4818, 0x00022227,
	0x021b083c, 0x03280338,
	0x021b0840, 0x0328031c,
	0x021b483c, 0x0330033c,
	0x021b4840, 0x032c0274,
	0x021b0848, 0x46343c3e,
	0x021b4848, 0x3e3c3648,
	0x021b0850, 0x3a3c443e,
	0x021b4850, 0x4a324a3e,
	0x021b080c, 0x001c001f,
	0x021b0810, 0x0029001c,
	0x021b480c, 0x0018002c,
	0x021b4810, 0x000f002a,
	0x021b08b8, 0x00000800,
	0x021b48b8, 0x00000800,
	0x021b001c, 0x00000000,
	0x021b0404, 0x00011006,
};

static void ddr_init(int *table, int size)
{
	int i;

	for (i = 0; i < size / 2 ; i++)
		writel(table[2 * i + 1], table[2 * i]);
}

static void spl_dram_init(enum ram_configs config)
{
	switch (config) {
	case s_1g_800mhz:
	case s_2g_800mhz:
	case s_1g_1000mhz:
	default:
		printf("KRIS: UNSUPPORTED MEMORY TYPE!\n");
		while(1);
		break;
	case q_2g_1000mhz:
		ddr_init(ts4900_1000mhz_4x256mx16_dcd_table,
			 ARRAY_SIZE(ts4900_1000mhz_4x256mx16_dcd_table));
		break;
	}
}

void board_init_f(ulong dummy)
{
	s32 straps;

	/* ASAP, disable the RTC power, and drive I2C pins low.
	 * While the RTC is normally disabled out of reset, ensure its driven
	 * low, also driving the I2C lines low to help fully bleed off any power.
	 * Note that this only needs to happen on rev A+
	 */
	SETUP_IOMUX_PADS(i2c1_pads_gpio);
	rtc_drain(rtc_gpio, ARRAY_SIZE(rtc_gpio));

	/* Initialize SPL */
	spl_early_init();

	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();
	gpr_init();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	fpga_program();

	/* Re-enable RTC power */
	/* TODO: Tune the delay time */
	rtc_enable(rtc_gpio[0], 140000);

	/* Set up I2C1 pinmux as peripheral.
	 * NOTE! The pad settings disable internal pull, that means these pins
	 * will slowly rise with RTC VDD which is safe.
	 */
	gpio_free(TS4900_SDA);
	gpio_free(TS4900_SCL);
	SETUP_IOMUX_PADS(i2c1_pads_i2c);
	udelay(1);

	/* Get CPU strapping */
	SETUP_IOMUX_PADS(cpu_strap_pads);
	straps = parse_gpio_straps(cpu_strap_gpio, ARRAY_SIZE(cpu_strap_gpio));
	printf("KRIS: cpu straps %d (0x%x)\n", straps, (u32)straps);


	/* At this point, we should be able to talk to the FPGA */
	setup_i2c(0, 100000, 0x28, &i2c_pad_info0);
	printf("KRIS: bus num %d\n", i2c_set_bus_num(0));
	printf("KRIS: FPGA probe %d\n", i2c_probe(0x28));
	printf("KRIS: FPGA strap 0x%x\n", i2c_reg_read(0x28, 51));



	/* Find board info here! */
	spl_dram_init(ts4900_ram_strap_decode(straps, i2c_reg_read(0x28, 51)));

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}
#endif

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	if (is_mx6dq()) {
		if (!strcmp(name, "imx6q-sabresd"))
			return 0;
	} else if (is_mx6dqp()) {
		if (!strcmp(name, "imx6qp-sabresd"))
			return 0;
	} else if (is_mx6dl()) {
		if (!strcmp(name, "imx6dl-sabresd"))
			return 0;
	}

	return -1;
}
#endif


