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
#include <watchdog.h>

#include <asm/arch/mx6-ddr.h>
#include <spl.h>
#include <linux/libfdt.h>

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

#define SPI_PAD_CLK_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_130ohm | PAD_CTL_SRE_SLOW)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_260ohm | PAD_CTL_SRE_SLOW)

#define I2C_PAD_CTRL (PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | 	\
	PAD_CTL_HYS | PAD_CTL_ODE | PAD_CTL_SRE_FAST)

//#define DISP0_PWR_EN	IMX_GPIO_NR(1, 21)
#define TS4900_SPI_CS		IMX_GPIO_NR(3, 19)
#define TS4900_ENRTC		IMX_GPIO_NR(3, 23)
#define TS4900_SCL		IMX_GPIO_NR(3, 21)
#define TS4900_SDA		IMX_GPIO_NR(3, 28)
#define TS4900_REVSTRAP		IMX_GPIO_NR(2, 11)
#define TS4900_REVSTRAPD	IMX_GPIO_NR(6, 5)
#define TS4900_REVSTRAPE	IMX_GPIO_NR(1, 29)
#define TS4900_RED_LEDn		IMX_GPIO_NR(1, 2)
#define TS4900_GREEN_LEDn	IMX_GPIO_NR(2, 24)
#if 0
#define TS4900_EN_5V		IMX_GPIO_NR(2, 22)
#define TS4900_OFFBD_RST	IMX_GPIO_NR(2, 21)
#define TS4900_SDBOOT		IMX_GPIO_NR(2, 26)
#define TS4900_SCL		IMX_GPIO_NR(3, 21)
#define TS4900_SDA		IMX_GPIO_NR(3, 28)
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

static iomux_v3_cfg_t const uart1_pads[] = {
	IOMUX_PADS(PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
	IOMUX_PADS(PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL)),
};

/* XXX: This is only used in SPL to set up SPI NOR flash. U-Boot proper uses
 * devicetree to set up these pins as needed.
 */
static iomux_v3_cfg_t const ecspi1_pads[] = {
	IOMUX_PADS(PAD_EIM_D19__GPIO3_IO19  | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CLK_CTRL)),
};

static iomux_v3_cfg_t const i2c1_pads_gpio[] = {
	IOMUX_PADS(PAD_EIM_D23__GPIO3_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL)), // EN_RTC
	IOMUX_PADS(PAD_EIM_D21__GPIO3_IO21 | MUX_PAD_CTRL(I2C_PAD_CTRL)), // SCL
	IOMUX_PADS(PAD_EIM_D28__GPIO3_IO28 | MUX_PAD_CTRL(I2C_PAD_CTRL)), // SDA
};

static struct rtc_gpio rtc_gpio[] = {
	{ TS4900_ENRTC,	1 },
	{ TS4900_SDA,	0 },
	{ TS4900_SCL,	0 },
};

static iomux_v3_cfg_t const i2c1_pads_i2c[] = {
	IOMUX_PADS(PAD_EIM_D21__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL)), // SCL
	IOMUX_PADS(PAD_EIM_D28__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL)), // SDA
};

static iomux_v3_cfg_t const fpga_pads[] = {
	/* FPGA_DONE */
	IOMUX_PADS(PAD_CSI0_DATA_EN__GPIO5_IO20    | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	/* FPGA_RESET */
	IOMUX_PADS(PAD_CSI0_VSYNC__GPIO5_IO21      | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	/* FPGA_SPI_CS# */
	IOMUX_PADS(PAD_CSI0_DAT16__GPIO6_IO02      | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT10__ECSPI2_MISO     | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT9__ECSPI2_MOSI      | MUX_PAD_CTRL(SPI_PAD_CTRL)),
	IOMUX_PADS(PAD_CSI0_DAT8__ECSPI2_SCLK      | MUX_PAD_CTRL(SPI_PAD_CLK_CTRL)),
	/* OFFBD_CS# */
	IOMUX_PADS(PAD_CSI0_DAT11__GPIO5_IO29      | MUX_PAD_CTRL(SPI_PAD_CTRL)),
};

static iomux_v3_cfg_t const fpga_clk[] = {
	/* FPGA_CLK */
	IOMUX_PADS(PAD_GPIO_3__XTALOSC_REF_CLK_24M | MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const led_pads[] = {
	/* RED_LED# */
	IOMUX_PADS(PAD_GPIO_2__GPIO1_IO02	| MUX_PAD_CTRL(NO_PAD_CTRL)),
	/* GREEN_LED# */
	IOMUX_PADS(PAD_EIM_CS1__GPIO2_IO24	| MUX_PAD_CTRL(NO_PAD_CTRL)),
};

static iomux_v3_cfg_t const cpu_strap_pads[] = {
	IOMUX_PADS(PAD_SD4_DAT3__GPIO2_IO11	| MUX_PAD_CTRL(GPIO_PAD_CTRL)), // A/C strap
	IOMUX_PADS(PAD_CSI0_DAT19__GPIO6_IO05	| MUX_PAD_CTRL(GPIO_PAD_CTRL)), // D strap
	IOMUX_PADS(PAD_ENET_TXD1__GPIO1_IO29	| MUX_PAD_CTRL(GPIO_PAD_CTRL)), // E strap
};

static unsigned cpu_strap_gpio[] = {
	TS4900_REVSTRAPE,
	TS4900_REVSTRAPD,
	TS4900_REVSTRAP,
};

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

static void setup_iomux_uart(void)
{
	SETUP_IOMUX_PADS(uart1_pads);
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

#ifdef CONFIG_MXC_SPI
int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? (TS4900_SPI_CS) : -1;
}
#endif

#ifdef CONFIG_SPL_OS_BOOT
/* XXX: used for falcon boot */
int spl_start_uboot(void)
{
	return 0;
}
#endif


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

	/* NOTE:
	 * In older U-Boot releases for this platform, the 24 MHz clock was set
	 * on the IOMUX alongside the FPGA programming pins. However, here it
	 * was observed that the FPGA does not get a clean reset if that is the
	 * case. The reset logic for the TS-4900 FPGA is based on the PLL.
	 * Enabling the clock after programming lets the FPGA come out of its
	 * internal reset and lets the fabric execute. This is how we can
	 * guarantee that the FPGA gets at least one posedge on the clock before
	 * the PLL is locked.
	 */
	SETUP_IOMUX_PADS(fpga_clk);
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
#if 1 // Set to 0 to enable 400 MHz RAM clk
	0x020e0798, 0x000C0000,
	0x020e0758, 0x00000000,
	0x020e0588, 0x00000030,
	0x020e0594, 0x00000030,
	0x020e056c, 0x00000030,
	0x020e0578, 0x00000030,
	0x020e074c, 0x00000030,
	0x020e057c, 0x00000030,
	0x020e058c, 0x00000000,
	0x020e059c, 0x00003030,
	0x020e05a0, 0x00003030,
	0x020e078c, 0x00000030,
	0x020e0750, 0x00020000,
	0x020e05a8, 0x00000030,
	0x020e05b0, 0x00000030,
	0x020e0524, 0x00000030,
	0x020e051c, 0x00000030,
	0x020e0518, 0x00000030,
	0x020e050c, 0x00000030,
	0x020e05b8, 0x00000030,
	0x020e05c0, 0x00000030,
	0x020e0774, 0x00020000,
	0x020e0784, 0x00000030,
	0x020e0788, 0x00000030,
	0x020e0794, 0x00000030,
	0x020e079c, 0x00000030,
	0x020e07a0, 0x00000030,
	0x020e07a4, 0x00000030,
	0x020e07a8, 0x00000030,
	0x020e0748, 0x00000030,
	0x020e05ac, 0x00000030,
	0x020e05b4, 0x00000030,
	0x020e0528, 0x00000030,
	0x020e0520, 0x00000030,
	0x020e0514, 0x00000030,
	0x020e0510, 0x00000030,
	0x020e05bc, 0x00000030,
	0x020e05c4, 0x00000030,
	0x021b0800, 0xa1390003,
	0x021b080c, 0x001a0018,
	0x021b0810, 0x0023001d,
	0x021b480c, 0x00140026,
	0x021b4810, 0x000e001b,
	0x021b083c, 0x432b0338,
	0x021b0840, 0x03210317,
	0x021b483c, 0x432e033e,
	0x021b4840, 0x03250264,
	0x021b0848, 0x40353a3c,
	0x021b4848, 0x39363242,
	0x021b0850, 0x3b3b3f3a,
	0x021b4850, 0x4534463e,
	0x021b081c, 0x33333333,
	0x021b0820, 0x33333333,
	0x021b0824, 0x33333333,
	0x021b0828, 0x33333333,
	0x021b481c, 0x33333333,
	0x021b4820, 0x33333333,
	0x021b4824, 0x33333333,
	0x021b4828, 0x33333333,
	0x021b08c0, 0x24921492,
	0x021b48c0, 0x24921492,
	0x021b08b8, 0x00000800,
	0x021b48b8, 0x00000800,
	0x021b0004, 0x00020036,
	0x021b0008, 0x24444040,
	0x021b000c, 0x8A8F7955,
	0x021b0010, 0xFF320F64,
	0x021b0014, 0x01FF00DB,
	0x021b0018, 0x00011740,
	0x021b001c, 0x00008000,
	0x021b002c, 0x000026d2,
	0x021b0030, 0x008F1023,
	0x021b0040, 0x00000047,
	0x021b0000, 0x841A0000,
	0x021b001c, 0x04088032,
	0x021b001c, 0x00008033,
	0x021b001c, 0x00048031,
	0x021b001c, 0x19408030,
	0x021b001c, 0x04008040,
	0x021b0020, 0x00007800,
	0x021b0818, 0x00011117,
	0x021b4818, 0x00011117,
	0x021b0004, 0x00025576,
	0x021b0404, 0x00011006,
	0x021b001c, 0x00000000,
#else
	0x020c4018, 0x00060324,
	0x020e0798, 0x000C0000,
	0x020e0758, 0x00000000,
	0x020e0588, 0x00000030,
	0x020e0594, 0x00000030,
	0x020e056c, 0x00000030,
	0x020e0578, 0x00000030,
	0x020e074c, 0x00000030,
	0x020e057c, 0x00000030,
	0x020e058c, 0x00000000,
	0x020e059c, 0x00003030,
	0x020e05a0, 0x00003030,
	0x020e078c, 0x00000030,
	0x020e0750, 0x00020000,
	0x020e05a8, 0x00000030,
	0x020e05b0, 0x00000030,
	0x020e0524, 0x00000030,
	0x020e051c, 0x00000030,
	0x020e0518, 0x00000030,
	0x020e050c, 0x00000030,
	0x020e05b8, 0x00000030,
	0x020e05c0, 0x00000030,
	0x020e0774, 0x00020000,
	0x020e0784, 0x00000030,
	0x020e0788, 0x00000030,
	0x020e0794, 0x00000030,
	0x020e079c, 0x00000030,
	0x020e07a0, 0x00000030,
	0x020e07a4, 0x00000030,
	0x020e07a8, 0x00000030,
	0x020e0748, 0x00000030,
	0x020e05ac, 0x00000030,
	0x020e05b4, 0x00000030,
	0x020e0528, 0x00000030,
	0x020e0520, 0x00000030,
	0x020e0514, 0x00000030,
	0x020e0510, 0x00000030,
	0x020e05bc, 0x00000030,
	0x020e05c4, 0x00000030,
	0x021b0800, 0xa1390003,
	0x021b080c, 0x00170016,
	0x021b0810, 0x001d001a,
	0x021b480c, 0x00130020,
	0x021b4810, 0x000f0019,
	0x021b083c, 0x4243024f,
	0x021b0840, 0x023e0234,
	0x021b483c, 0x42430250,
	0x021b4840, 0x0240021e,
	0x021b0848, 0x40373b3d,
	0x021b4848, 0x3a393640,
	0x021b0850, 0x39393b3b,
	0x021b4850, 0x4131403b,
	0x021b081c, 0x33333333,
	0x021b0820, 0x33333333,
	0x021b0824, 0x33333333,
	0x021b0828, 0x33333333,
	0x021b481c, 0x33333333,
	0x021b4820, 0x33333333,
	0x021b4824, 0x33333333,
	0x021b4828, 0x33333333,
	0x021b08c0, 0x24921492,
	0x021b48c0, 0x24921492,
	0x021b08b8, 0x00000800,
	0x021b48b8, 0x00000800,
	0x021b0004, 0x0001002D,
	0x021b0008, 0x1B333030,
	0x021b000c, 0x676B52F3,
	0x021b0010, 0xB66D0B63,
	0x021b0014, 0x01FF00DA,
	0x021b0018, 0x00011740,
	0x021b001c, 0x00008000,
	0x021b002c, 0x000026d2,
	0x021b0030, 0x006B1023,
	0x021b0040, 0x00000047,
	0x021b0000, 0x841A0000,
	0x021b001c, 0x04008032,
	0x021b001c, 0x00008033,
	0x021b001c, 0x00048031,
	0x021b001c, 0x15208030,
	0x021b001c, 0x04008040,
	0x021b0020, 0x00007800,
	0x021b0818, 0x00011117,
	0x021b4818, 0x00011117,
	0x021b0004, 0x0001556D,
	0x021b0404, 0x00011006,
	0x021b001c, 0x00000000,
#endif
};

static int ts4900_800mhz_2x512mx16_dcd_table[] = {
	0x020e0774, 0x000C0000,
	0x020e0754, 0x00000000,
	0x020e04ac, 0x00000030,
	0x020e04b0, 0x00000030,
	0x020e0464, 0x00000030,
	0x020e0490, 0x00000030,
	0x020e074c, 0x00000030,
	0x020e0494, 0x00000030,
	0x020e04a0, 0x00000000,
	0x020e04b4, 0x00003030,
	0x020e04b8, 0x00003030,
	0x020e076c, 0x00000030,
	0x020e0750, 0x00020000,
	0x020e04bc, 0x00000030,
	0x020e04c0, 0x00000030,
	0x020e04c4, 0x00000030,
	0x020e04c8, 0x00000030,
	0x020e04cc, 0x00000030,
	0x020e04d0, 0x00000030,
	0x020e04d4, 0x00000030,
	0x020e04d8, 0x00000030,
	0x020e0760, 0x00020000,
	0x020e0764, 0x00000030,
	0x020e0770, 0x00000030,
	0x020e0778, 0x00000030,
	0x020e077c, 0x00000030,
	0x020e0780, 0x00000030,
	0x020e0784, 0x00000030,
	0x020e078c, 0x00000030,
	0x020e0748, 0x00000030,
	0x020e0470, 0x00000030,
	0x020e0474, 0x00000030,
	0x020e0478, 0x00000030,
	0x020e047c, 0x00000030,
	0x020e0480, 0x00000030,
	0x020e0484, 0x00000030,
	0x020e0488, 0x00000030,
	0x020e048c, 0x00000030,
	0x021b0800, 0xa1390003,
	0x021b080c, 0x00520054,
	0x021b0810, 0x0048004d,
	0x021b480c, 0x001F001F,
	0x021b4810, 0x001F001F,
	0x021b083c, 0x4250025c,
	0x021b0840, 0x0240023c,
	0x021b483c, 0x4201020C,
	0x021b4840, 0x01660172,
	0x021b0848, 0x464a4a4a,
	0x021b4848, 0x4A4F5049,
	0x021b0850, 0x33332e2f,
	0x021b4850, 0x3238372B,
	0x021b081c, 0x33333333,
	0x021b0820, 0x33333333,
	0x021b0824, 0x33333333,
	0x021b0828, 0x33333333,
	0x021b481c, 0x33333333,
	0x021b4820, 0x33333333,
	0x021b4824, 0x33333333,
	0x021b4828, 0x33333333,
	0x021b08b8, 0x00000800,
	0x021b48b8, 0x00000800,
	0x021b0004, 0x0002002D,
	0x021b0008, 0x1B333030,
	0x021b000c, 0x8B8F52F3,
	0x021b0010, 0xB66D0B63,
	0x021b0014, 0x01FF00DB,
	0x021b0018, 0x00011740,
	0x021b001c, 0x00008000,
	0x021b002c, 0x000026d2,
	0x021b0030, 0x008F1023,
	0x021b0040, 0x00000047,
	0x021b0000, 0x85190000,
	0x021b001c, 0x04008032,
	0x021b001c, 0x00008033,
	0x021b001c, 0x00048031,
	0x021b001c, 0x15208030,
	0x021b001c, 0x04008040,
	0x021b0020, 0x00007800,
	0x021b0818, 0x00011117,
	0x021b4818, 0x00011117,
	0x021b0004, 0x0002556D,
	0x021b0404, 0x00011006,
	0x021b001c, 0x00000000,
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
	case s_1g_1000mhz:
	default:
		printf("KRIS: UNSUPPORTED MEMORY TYPE!\n");
		while(1);
		break;
	case q_2g_1000mhz:
		ddr_init(ts4900_1000mhz_4x256mx16_dcd_table,
			 ARRAY_SIZE(ts4900_1000mhz_4x256mx16_dcd_table));
		break;
	case s_2g_800mhz:
		ddr_init(ts4900_800mhz_2x512mx16_dcd_table,
			 ARRAY_SIZE(ts4900_800mhz_2x512mx16_dcd_table));
		break;
	}
}

void board_init_f(ulong dummy)
{
	s32 straps;

	/* Turn on the WDT as soon as possible, its possible it was enabled in
	 * hardware already. Note that there are no other explicit feeds that
	 * take place until U-Boot is started. The default timeout needs to be
	 * long enough to ensure that U-Boot can start or additional feeds need
	 * to be added here.
	 *
	 * If enabled in hardware by blowing bit 21 of bank 0 word 6 of OTP,
	 * the WDT is enabled with a 90 s timeout.
	 */
#if defined(CONFIG_IMX_WATCHDOG)
	hw_watchdog_init();
#endif

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

	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* Turn on LEDs to let user know that U-Boot has begun initialization */
	SETUP_IOMUX_PADS(led_pads);
	gpio_request(TS4900_RED_LEDn, "led");
	gpio_request(TS4900_GREEN_LEDn, "led");
	gpio_direction_output(TS4900_RED_LEDn, 0);
	gpio_direction_output(TS4900_GREEN_LEDn, 0);

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
	/* XXX: TODO: Use a blob list to pass straps forward to U-Boot proper */
	SETUP_IOMUX_PADS(cpu_strap_pads);
	straps = parse_gpio_straps(cpu_strap_gpio, ARRAY_SIZE(cpu_strap_gpio));


	/* At this point, we should be able to talk to the FPGA */
	/* TODO: NOTE: setup_i2c() is generating a force_idle_bus message */
	/* BUG: XXX: There is still some issue on a short hardware reset pulse
	 * where the I2C RTC bus doesn't correctly start back up:
	 * iCE40 FPGA reloaded successfully
	 * force_idle_bus: sda=0 scl=0 sda.gp=0x5c scl.gp=0x55
	 * wait_for_sr_state: Arbitration lost sr=32 cr=88 state=202
	 * wait_for_sr_state: failed sr=22 cr=88 state=2000
	 * i2c_imx_stop:trigger stop failed
	 * wait_for_sr_state: failed sr=22 cr=88 state=2000
	 * i2c_imx_stop:trigger stop failed
	 */

	setup_i2c(0, 100000, 0x28, &i2c_pad_info0);
	i2c_set_bus_num(0);
	spl_dram_init(ts4900_ram_strap_decode(straps, i2c_reg_read(0x28, 51)));

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}

#if 0 
/* XXX: Unsure if we need this long term, keeping it here as reference */
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
#endif
