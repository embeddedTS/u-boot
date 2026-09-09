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
#include <led.h>
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
#include "../common/bbdetect.h"
#include "../common/parse_gpio_straps.h"

DECLARE_GLOBAL_DATA_PTR;

/* TODO: XXX:
 * When running in U-Boot proper, we need to use dm GPIO handling rather than
 * the legacy stuff.
 */
#if 0
#define TS4900_OTG_ID		IMX_GPIO_NR(1, 1)
#define TS4900_WIFI_EN		IMX_GPIO_NR(1, 26)
#define TS4900_BT_EN		IMX_GPIO_NR(1, 27)
#endif

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();
	return 0;
}

/* We need to control some of the MII pins as GPIO prior to PHY unreset in
 * order to configure copper straps.
 */
static iomux_v3_cfg_t const enet_pads[] = {
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

/* Baseboard ID handling */
static const struct bbdetect_pins bbpins = {
	.bit = {
		"RED_LED#",	// bit 0
		"GREEN_LED#",	// bit 1
		"BUS_DIR",	// bit 3
	},
	.in = "DIO_15",
};

/* Option strap handling */
static const char *strap_pins[] = {
	"REV_STRAP",
	"REV_STRAP_D",
	"REV_STRAP_E",
};

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

static const char *names[] = {
	/* We do lump reset in here, for ease of iteration, but its
	 * starting value of 1 matches the value we want to set everything
	 * else to so it works out nicely.
	 */
	"ENET_PHY_RST",
	"RGMII_RXC",
	"RGMII_RD0",
	"RGMII_RD1",
	"RGMII_RD2",
	"RGMII_RD3",
	"RGMII_RX_CTL",
};

/* Must be called early in boot, either late_init() or misc_init_r(), before
 * calls to eth_init() are ultimately made. We rely on the devicetree to set
 * the real final ethernet MAC/MDIO/MII IOMUX settings, but, need to control
 * these pins as GPIO to force a proper bootstrapping when un-resetting the
 * PHYs
 */
/* This could also be done in SPL, this would make this dance a little more
 * simple, however, with it not being necessary for boot and other PHY config
 * happening at this stage, its a bit more clean to leave it here.
 */
static int early_phy_strap_reset(void)
{
	struct gpio_desc descs[ARRAY_SIZE(names)];
	int i;

	SETUP_IOMUX_PADS(enet_pads);

	/* Get all of our pins in the required states, this includes setting
	 * specific pins to a high state while PHY reset is asserted.
	 */
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		if (gpio_request_by_line_name(NULL, names[i], &descs[i],
				(GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE)) < 0)
			return -1;
	}
	/* Datasheet calls out 10 ms of stable supply voltage to de-assertion
	 * of reset. Lets just wait that 10 ms since its not clear if that is
	 * the reset assertion time. No specific reset assertion time is called
	 * out.
	 */
	udelay(10000);

	/* De-assert reset */
	dm_gpio_set_value(&descs[0], 0);

	/* Datasheet calls out 6 ns from de-assertion of reset to strap pin
	 * output time.
	 */
	udelay(1);

	/* It is safe to free all of the pins at this point */
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		dm_gpio_free(NULL, &descs[i]);
	}

	return 0;
}

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

	/* NOTE: mxc GPIO driver does not support set_flags function call,
	 * so no flags like pull can be set here without erroring.
	 */
	if (gpio_request_by_line_name(NULL, "BUS_DIR", &jpsdboot,
			(GPIOD_IS_IN)) < 0)
		return -1;
#endif

	if (gpio_request_by_line_name(NULL, "OFF_BD_RESET#", &offbdrst,
			(GPIOD_IS_OUT | GPIOD_ACTIVE_LOW | GPIOD_IS_OUT_ACTIVE)) < 0)
		return -1;

	if (gpio_request_by_line_name(NULL, "EN_USB_5V#", &vusb,
			(GPIOD_IS_OUT)) < 0)
		return -1;

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

int board_init(void)
{
	/* XXX: What is the point of this? */
	/* address of boot parameters */
	gd->bd->bi_boot_params = CFG_SYS_SDRAM_BASE + 0x100;

#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif

	return 0;
}

int board_late_init(void)
{
	s32 straps;
	uint8_t val;
	struct udevice *dev;

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	/* Issue off board reset, optionally read sdboot jumper, and enable
	 * USB 5 V rail.
	 */
	if (offbd_reset() < 0)
		printf("\nERROR RESETTING OFF BOARD PERIPHERALS!\n");

	if (early_phy_strap_reset() < 0)
		printf("\nERROR STRAPPING ENET PHY!\n");

	/* Parse baseboard ID and rev
	 * Since this uses the LED pins, this relies on the u-boot.dtsi deleting
	 * the default-state property in order to not have a GPIO conflict.
	 * Because of that, we need to then set the LED state explicitly after
	 * we get the baseboard ID variables.
	 *
	 * Since by this point the devicetree has been loaded and parsed, the
	 * IOMUX for the pins used for this are already set up.
	 *
	 * The bbdetect function automatically exports the _id and _rev variables
	 * to the environment.
	 */
	if (bbdetect(bbpins, 1000) < 0)
		printf("\nERROR READING BASEBOARD ID\n");
	if (!led_get_by_label("red:status", &dev))
		led_set_state(dev, LEDST_ON);
	if (!led_get_by_label("green:power", &dev))
		led_set_state(dev, LEDST_OFF);

	/* Export build options, CPU info, etc., to environment.
	 * Note that these values are NOT normalized like the need to be for
	 * DRAM configuration in SPL. This specifically is for backwards
	 * compatibility with existing tshwctl paradigms.
	 */
	env_set("board_name", "ts4900");
	if (is_mx6dq())
		env_set("cpu", "q");
	if (is_mx6sdl())
		env_set("cpu", "dl");

	straps = parse_gpio_straps(strap_pins, ARRAY_SIZE(strap_pins));
	if (straps < 0)
		printf("\nERROR READING CPU STRAP IO\n");
	env_set_hex("pcb_revision", straps);

	if (i2c_get_chip_for_busnum(0, 0x28, 1, &dev) < 0)
		printf("\nERROR SETTING UP FPGA I2C READ\n");

	if (dm_i2c_read(dev, 51, &val, 1)) {
		printf("\nERROR READING FPGA\n");
		env_set_hex("bom_options", -1);
	} else {
		env_set_hex("bom_options", val);
	}

	return 0;
}
