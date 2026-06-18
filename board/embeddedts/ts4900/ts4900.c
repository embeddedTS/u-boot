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
#include <micrel.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <input.h>
#include <usb.h>
#include <usb/ehci-ci.h>

#include "strap_decode.h"
#include "../common/parse_gpio_straps.h"

DECLARE_GLOBAL_DATA_PTR;

/* TODO: XXX:
 * When running in U-Boot proper, we need to use dm GPIO handling rather than
 * the legacy stuff.
 */
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
#define TS4900_SPI_CS		IMX_GPIO_NR(3, 19)
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


/* XXX: TODO: Implement GPU support? */


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

int board_phy_config(struct phy_device *phydev)
{
	int ret;

	/* Set up GMII Clock Pad Skew */
	ksz9031_phy_extended_write(phydev, 0x2, 0x8, 0x4000, 0x3EF);
	/* Set AN FLP Burst Transmit LO -- 16 ms */
	ksz9031_phy_extended_write(phydev, 0x0, 0x3, 0x4000, 0x1A80);
	/* Set AN FLP Burst Transmit HI -- 16 ms */
	ksz9031_phy_extended_write(phydev, 0x0, 0x4, 0x4000, 0x0006);

	/* Only advertize 100 Mbit or lower to reduce link time. */
	ret = phy_set_supported(phydev, SPEED_100);
	if (ret)
		return ret;

	if (phydev->drv->config) {
		ret = phydev->drv->config(phydev); 
		if (ret)
			return ret;
	}

        return 0;
}

/* Must be called early in boot, either late_init() or misc_init_r(), before
 * calls to eth_init() are ultimately made. We rely on the devicetree to set
 * the real final ethernet MAC/MDIO/MII IOMUX settings, but, need to control
 * these pins as GPIO to force a proper bootstrapping when un-resetting the
 * PHYs
 */
static void early_phy_strap_reset(void)
{
	SETUP_IOMUX_PADS(enet_pads1);

	/* Assert reset */
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

	/* Datasheet calls out 10 ms of stable supply voltage to de-assertion
	 * of reset. Lets just wait that 10 ms since its not clear if that is
	 * the reset assertion time. No specific reset assertion time is called
	 * out.
	 */
	udelay(10000);

	/* De-assert reset */
	gpio_direction_output(TS4900_PHY_RST, 0);

	/* Datasheet calls out 6 ns from de-assertion of reset to strap pin
	 * output time.
	 */
	udelay(1);
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

/* Issues the offboard reset for 1 ms
 * If running in the "legacy" boot mode, read the sdboot jumper and export
 * that to the environment.
 */
static int offbd_reset(void)
{
	struct gpio_desc offbdrst, vusb;
#ifdef CFG_ETS_LEGACY_BOOT
	struct gpio_desc jpsdboot;
#endif

	if (gpio_request_by_line_name(NULL, "OFF_BD_RESET#", &offbdrst,
			(GPIOD_IS_OUT | GPIOD_ACTIVE_LOW | GPIOD_IS_OUT_ACTIVE)) < 0)
		return -1;

	if (gpio_request_by_line_name(NULL, "EN_USB_5V#", &vusb,
			(GPIOD_IS_OUT)) < 0)
		return -1;

#ifdef CFG_ETS_LEGACY_BOOT
	/* NOTE: mxc GPIO driver does not support set_flags function call,
	 * so no flags like pull can be set here without erroring.
	 */
	if (gpio_request_by_line_name(NULL, "BUS_DIR", &jpsdboot,
			(GPIOD_IS_IN)) < 0)
		return -1;
#endif

	mdelay(1);

#ifdef CFG_ETS_LEGACY_BOOT
	/* Following legacy behavior, only check for zero or non-zero */
	if (dm_gpio_get_value(&jpsdboot))
		env_set("jpsdboot", "off");
	else
		env_set("jpsdboot", "on");

	/* BUG!!
	 * dm_gpio_free() dereferences the first arg, which we don't have a
	 * struct udevice due to how we obtained the GPIO, so, this could be
	 * a problem, but is the "right thing" to do.
	 */
	dm_gpio_free(NULL, &jpsdboot);
#endif

	/* Deassert reset */
	if (dm_gpio_set_value(&offbdrst, 0) < 0)
		return -1;

	/* NOTE!
	 * Legacy TS-4900 code waited 100 ms after unreset to turn on USB 5 V.
	 * Based on the datasheets of the parts used in our baseboards, this
	 * is an overly conservative timeout. Most hubs are ready to start
	 * attaching devices withing 10 us of reset being released. Our LTS
	 * platform code follows this 10 us. If there are any issues with USB
	 * on baseboards with hubs, this is the place to start.
	 */
	udelay(10);

	if (dm_gpio_set_value(&vusb, 1) < 0)
		return -1;

	/* We intentionally don't free OFFBD_RST and EN_5V since we want these
	 * states to be retained through the rest of U-Boot.
	 */
	return 0;
};


int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	/* Issue off board reset, optionally read sdboot jumper, and enable
	 * USB 5 V rail.
	 */
	if (offbd_reset() < 0)
		printf("ERROR RESETTING OFF BOARD PERIPHERALS!\n");

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


