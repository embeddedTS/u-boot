/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Author: Mark Featherston <mark@embeddedarm.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/arch/mx6-pins.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/video.h>
#include <asm/imx-common/sata.h>
#include <mmc.h>
#include <malloc.h>
#include <spi.h>
#include <net.h>
#include <image.h>
#include <fsl_esdhc.h>
#include <command.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/crm_regs.h>
#include <i2c.h>
#include <micrel.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#include <fpga.h>
#include <lattice.h>

#define TS7970_HUB_RESETN	IMX_GPIO_NR(2, 11)
#define TS7970_EN_RTC		IMX_GPIO_NR(3, 23)
#define TS7970_SCL			IMX_GPIO_NR(3, 21)
#define TS7970_SDA			IMX_GPIO_NR(3, 28)
#define TS7970_EN_5V		IMX_GPIO_NR(2, 22)
#define TS7970_SEL_DC		IMX_GPIO_NR(5, 17)
#define TS7970_SDBOOT		IMX_GPIO_NR(2, 26)
#define TS7970_SPI_CS		IMX_GPIO_NR(3, 19)
#define TS7970_PHY_RST		IMX_GPIO_NR(4, 20)
#define TS7970_RGMII_RXC	IMX_GPIO_NR(6, 30)
#define TS7970_RGMII_RD0	IMX_GPIO_NR(6, 25)
#define TS7970_RGMII_RD1	IMX_GPIO_NR(6, 27)
#define TS7970_RGMII_RD2	IMX_GPIO_NR(6, 28)
#define TS7970_RGMII_RD3	IMX_GPIO_NR(6, 29)
#define TS7970_RGMII_RX_CTL	IMX_GPIO_NR(6, 24)
#define TS7970_EN_SDPWR		IMX_GPIO_NR(2, 28)
#define TS7970_PUSHSW       IMX_GPIO_NR(1, 04)
#define TS7970_SCL			IMX_GPIO_NR(3, 21)
#define TS7970_SDA			IMX_GPIO_NR(3, 28)

DECLARE_GLOBAL_DATA_PTR;
int random_mac = 0;

#define UART_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL (PAD_CTL_PUS_100K_UP |                  \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |   \
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

iomux_v3_cfg_t const ecspi1_pads[] = {
	MX6_PAD_EIM_D19__GPIO3_IO19 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

iomux_v3_cfg_t const misc_pads[] = {
	MX6_PAD_SD4_DAT3__GPIO2_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL), 		 // USB_HUB_RESET#
	MX6_PAD_EIM_A16__GPIO2_IO22 | MUX_PAD_CTRL(NO_PAD_CTRL), 		 // EN_USB_5V
	MX6_PAD_EIM_RW__GPIO2_IO26 | MUX_PAD_CTRL(NO_PAD_CTRL), 		 // JP_SD_BOOT#
	MX6_PAD_DISP0_DAT23__GPIO5_IO17 | MUX_PAD_CTRL(NO_PAD_CTRL), 	 // SEL_DC_USB#
	MX6_PAD_EIM_D23__GPIO3_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL), 		 // EN_RTC_PWR#
	MX6_PAD_GPIO_4__GPIO1_IO04 | MUX_PAD_CTRL(NO_PAD_CTRL), 		 // FPGA_IRQ_1#
	MX6_PAD_GPIO_3__XTALOSC_REF_CLK_24M | MUX_PAD_CTRL(NO_PAD_CTRL), // FPGA CLK
};

/* SD card */
iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__SD2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_EIM_EB0__GPIO2_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL), // EN_SD_POWER
};

/* MMC */
iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD   | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const uart1_gpio_pads[] = {
	MX6_PAD_SD3_DAT7__GPIO6_IO17 | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__GPIO6_IO18 | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads1[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* pin 35 - 1 (PHY_AD2) on reset */
	MX6_PAD_RGMII_RXC__GPIO6_IO30		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 32 - 1 - (MODE0) all */
	MX6_PAD_RGMII_RD0__GPIO6_IO25		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 31 - 1 - (MODE1) all */
	MX6_PAD_RGMII_RD1__GPIO6_IO27		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 28 - 1 - (MODE2) all */
	MX6_PAD_RGMII_RD2__GPIO6_IO28		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 27 - 1 - (MODE3) all */
	MX6_PAD_RGMII_RD3__GPIO6_IO29		| MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
	MX6_PAD_RGMII_RX_CTL__GPIO6_IO24	| MUX_PAD_CTRL(NO_PAD_CTRL),
	
	// PHY RESET
	MX6_PAD_DI0_PIN4__GPIO4_IO20		| MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads2[] = {
	MX6_PAD_RGMII_RXC__RGMII_RXC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL	| MUX_PAD_CTRL(ENET_PAD_CTRL),
};

iomux_v3_cfg_t const i2c_pads[] = {
	MX6_PAD_EIM_D21__GPIO3_IO21	| MUX_PAD_CTRL(I2C_PAD_CTRL),
	MX6_PAD_EIM_D28__GPIO3_IO28	| MUX_PAD_CTRL(I2C_PAD_CTRL),
};

#if defined(CONFIG_FPGA)

iomux_v3_cfg_t const fpga_jtag_pads[] = {
	MX6_PAD_DISP0_DAT14__GPIO5_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL), 	// JTAG_FPGA_TMS
	MX6_PAD_DISP0_DAT17__GPIO5_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL), 	// JTAG_FPGA_TCK
	MX6_PAD_DISP0_DAT18__GPIO5_IO12 | MUX_PAD_CTRL(NO_PAD_CTRL),	// JTAG_FPGA_TDO
	MX6_PAD_DISP0_DAT22__GPIO5_IO16 | MUX_PAD_CTRL(NO_PAD_CTRL), 	// JTAG_FPGA_TDI
};

static void ts7970_jtag_init(void)
{
	imx_iomux_v3_setup_multiple_pads(fpga_jtag_pads,
					 ARRAY_SIZE(fpga_jtag_pads));
	gpio_direction_output(CONFIG_FPGA_TDI, 1);
	gpio_direction_output(CONFIG_FPGA_TCK, 1);
	gpio_direction_output(CONFIG_FPGA_TMS, 1);
	gpio_direction_input(CONFIG_FPGA_TDO);
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
	589012,
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

struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode  = MX6_PAD_EIM_D21__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_D21__GPIO3_IO21 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = TS7970_SCL
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D28__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_D28__GPIO3_IO28 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = TS7970_SDA
	}
};

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? (TS7970_SPI_CS) : -1;
}

void setup_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
					 ARRAY_SIZE(ecspi1_pads));

	// Enable clock
	setbits_le32(CCM_CCGR1, MXC_CCM_CCGR1_ECSPI2S_MASK);
}

int dram_init(void)
{	
	gd->ram_size = (phys_size_t)CONFIG_DDR_MB * 1024 * 1024;
	return 0;
}

static void setup_iomux_enet(void)
{
	// Assert reset
	gpio_direction_output(TS7970_PHY_RST, 1);
	imx_iomux_v3_setup_multiple_pads(enet_pads1, ARRAY_SIZE(enet_pads1));

	gpio_direction_output(TS7970_RGMII_RXC, 1);
	gpio_direction_output(TS7970_RGMII_RD0, 1);
	gpio_direction_output(TS7970_RGMII_RD1, 1);
	gpio_direction_output(TS7970_RGMII_RD2, 1);
	gpio_direction_output(TS7970_RGMII_RD3, 1);
	imx_iomux_v3_setup_multiple_pads(enet_pads1, ARRAY_SIZE(enet_pads1));
	gpio_direction_output(TS7970_RGMII_RX_CTL, 1);

	/* Need delay at least 10ms according to KSZ9031 spec */
	udelay(1000 * 100);

	// De-assert reset
	gpio_direction_output(TS7970_PHY_RST, 0);

	/* Need 100us delay to exit from reset. */
	udelay(1000 * 100);

	imx_iomux_v3_setup_multiple_pads(enet_pads2, ARRAY_SIZE(enet_pads2));
}

struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC2_BASE_ADDR},
	{USDHC3_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC2_BASE_ADDR: // SD
		ret = 1;
		break;
	case USDHC3_BASE_ADDR: // MMC
		ret = 1;
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
	s32 status = 0;
	int i;

	imx_iomux_v3_setup_multiple_pads(usdhc2_pads, 
		ARRAY_SIZE(usdhc2_pads));
	imx_iomux_v3_setup_multiple_pads(usdhc3_pads, 
		ARRAY_SIZE(usdhc3_pads));

	gpio_direction_output(TS7970_EN_SDPWR, 1); // EN_SD_POWER#
	udelay(1000);
	gpio_direction_output(TS7970_EN_SDPWR, 0);

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SD
	 * mmc1                    eMMC
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0: // SD
			usdhc_cfg[0].max_bus_width = 4;
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

			break;
		case 1: // MMC
			usdhc_cfg[1].max_bus_width = 4;
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
			       "(%d) then supported by the board (%d)\n",
			       i + 1, CONFIG_SYS_FSL_USDHC_NUM);
			return status;
		}

		status |= fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
	}

	return status;
}

int board_phy_config(struct phy_device *phydev)
{
	ksz9031_phy_extended_write(phy, 0x2, 0x8, 0x8000, 0x303);

	if (phydev->drv->config)
		phydev->drv->config(phydev); 

	return 0;
}

void fixup_i2c(void)
{
	int i;

	// EN RTC fet
	gpio_direction_output(TS7970_EN_RTC, 0);

	imx_iomux_v3_setup_multiple_pads(i2c_pads, ARRAY_SIZE(i2c_pads));
	gpio_direction_input(TS7970_SCL);
	gpio_direction_input(TS7970_SDA);

	// On rare occasions the RTC misbehaves and drives the pins
	// on i2c.  Turning off the fet prevents this condition
	udelay(1000*2); // 2ms to turn on fet
	for (i = 0; i < 5; i++)
	{
		if (gpio_get_value(TS7970_SCL) == 1 &&
			gpio_get_value(TS7970_SDA) == 1)
			break;
		puts("Attempting to reset RTC\n");
		// Enable RTC FET
		gpio_direction_output(TS7970_EN_RTC, 1);
		udelay(1000*200); // at least 140ms to discharge
		gpio_direction_output(TS7970_EN_RTC, 0);
		udelay(1000*2); // 2ms to turn on
		if(i == 4) puts ("Not able to force bus idle.  Giving up.\n");
	}
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	uchar enetaddr[6];
	int ret;

	setup_iomux_enet();
	bus = fec_get_miibus(base, -1);
	if (!bus)
		return 0;
	/* scan phy 4,5,6,7 */
	phydev = phy_find_by_mask(bus, (0xf << 4), PHY_INTERFACE_MODE_RGMII);

	if (!phydev) {
		free(bus);
		return 0;
	}
	printf("using phy at %d\n", phydev->addr);
	ret  = fec_probe(bis, -1, base, bus, phydev);
	if (ret) {
		printf("FEC MXC: %s:failed\n", __func__);
		free(phydev);
		free(bus);
	}

	// This should only happen in production
#ifdef CONFIG_RANDOM_MACADDR
	if (!eth_getenv_enetaddr("ethaddr", enetaddr)) {
		printf("No MAC address set in fuses.  Using random mac address.\n");
		eth_random_addr(enetaddr);
		random_mac = 1;
		if (eth_setenv_enetaddr("ethaddr", enetaddr)) {
			printf("Failed to set ethernet address\n");
		}
	}
#endif
	
	return ret;
}

int board_early_init_f(void)
{
	char *rcause = get_reset_cause(0);

	/* The TS-7970 should reset from a poke in the fpga to cause a true 
	 * POR reset.  Unfortunately the i2c bus needed to do that can't
	 * be accessed this early on, so this will just silence the uart
	 * on boots that are actually resets until it boots enough to reset */
	if(strstr(rcause, "WDOG")) {
		imx_iomux_v3_setup_multiple_pads(uart1_gpio_pads, ARRAY_SIZE(uart1_gpio_pads));
	} else {
		imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	}

	return 0;
}

void reset_misc(void)
{
	uint8_t val = 0x2;
	/* Poke reset register in the fpga */
	i2c_write(0x28, 30, 2, &val, 1);
}

int misc_init_r(void)
{
	uint8_t val = 0x2;
	int sdboot = 0;
	struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	char *rcause = get_reset_cause(1);

	/* If there is ever a watchdog reset, cause a full POR */
	if(strstr(rcause, "WDOG")) {
		i2c_write(0x28, 30, 2, &val, 1);
	}

	imx_iomux_v3_setup_multiple_pads(misc_pads, ARRAY_SIZE(misc_pads));

	// Turn off USB hub until hub is reset
	// Set DC_SEL_USB to use usb on the standard header
	gpio_direction_input(TS7970_SDBOOT);
	gpio_direction_output(TS7970_EN_5V, 0);
	gpio_direction_output(TS7970_SEL_DC, 0); // 0=USB A conn, 1=HD1
	gpio_direction_output(TS7970_HUB_RESETN, 0); // hub only needs 1us
	udelay(1);
	gpio_set_value(TS7970_HUB_RESETN, 1);

	// Magic number for usb power on reset
	// bad things happen with most devices if the hub
	// gets a reset and power doesn't.  This *might* not be
	// enough for some devices
	udelay(10000); 
	gpio_set_value(TS7970_EN_5V, 1);
	sdboot = gpio_get_value(TS7970_SDBOOT);

	if(sdboot) setenv("jpsdboot", "off");
	else setenv("jpsdboot", "on");

	setenv("imx_type", CONFIG_IMX_TYPE);

	#ifdef CONFIG_MX6Q
	setenv("cpu", "q");
	#else
	setenv("cpu", "dl");
	#endif

	setenv("model", "7970");

	i2c_read(0x28, 31, 2, &val, 1);
	if(val & 0x4) {
		setenv("pushsw", "off");
	} else {
		setenv("pushsw", "on");
	}

	/* PCIE does not get properly disabled from a watchdog reset.  This prevents 
	 * a hang in the kernel if pcie was enabled in a previous boot. */
	setbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_TEST_POWERDOWN);
	clrbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_REF_SSP_EN);

	return 0;
}

int board_init(void)
{
	fixup_i2c();
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_spi();

	#ifdef CONFIG_CMD_SATA
	setup_sata();
	#endif

	#ifdef CONFIG_FPGA
	ts7970_fpga_init();
	#endif

	return 0;
}

int board_late_init(void)
{
	return 0;
}

int checkboard(void)
{
	puts("Board: TS-7970\n");
	return 0;
}
