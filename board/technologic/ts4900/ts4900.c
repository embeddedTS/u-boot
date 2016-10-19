/*
 * Copyright (C) 2016 Technologic Systems
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
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/boot_mode.h>
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

#define TS4900_EN_5V		IMX_GPIO_NR(2, 22)
#define TS4900_OFFBD_RST	IMX_GPIO_NR(2, 21)
#define TS4900_SDBOOT		IMX_GPIO_NR(2, 26)
#define TS4900_SCL			IMX_GPIO_NR(3, 21)
#define TS4900_SDA			IMX_GPIO_NR(3, 28)
#define TS4900_ENRTC		IMX_GPIO_NR(3, 23)
#define TS4900_REVSTRAP		IMX_GPIO_NR(2, 11)
#define TS4900_REVSTRAPD	IMX_GPIO_NR(6, 5)
#define TS4900_SPI_CS		IMX_GPIO_NR(3, 19)
#define TS4900_PHY_RST		IMX_GPIO_NR(4, 20)
#define TS4900_RGMII_RXC	IMX_GPIO_NR(6, 30)
#define TS4900_RGMII_RD0	IMX_GPIO_NR(6, 25)
#define TS4900_RGMII_RD1	IMX_GPIO_NR(6, 27)
#define TS4900_RGMII_RD2	IMX_GPIO_NR(6, 28)
#define TS4900_RGMII_RD3	IMX_GPIO_NR(6, 29)
#define TS4900_RGMII_RX_CTL	IMX_GPIO_NR(6, 24)
#define TS4900_EN_SDPWR		IMX_GPIO_NR(2, 28)

DECLARE_GLOBAL_DATA_PTR;
int random_mac = 0;

#define UART_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL (PAD_CTL_SPEED_MED | PAD_CTL_DSE_80ohm)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL (PAD_CTL_PUS_100K_UP |                  \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |   \
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

iomux_v3_cfg_t const ecspi1_pads[] = {
	MX6_PAD_EIM_D19__GPIO3_IO19   | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

iomux_v3_cfg_t const misc_pads[] = {
	MX6_PAD_EIM_A17__GPIO2_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL), // off_bd_reset
	MX6_PAD_GPIO_2__GPIO1_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL), // Red LED
	MX6_PAD_EIM_CS1__GPIO2_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL), // Green LED
	MX6_PAD_EIM_RW__GPIO2_IO26 | MUX_PAD_CTRL(NO_PAD_CTRL), // MODE2
	MX6_PAD_EIM_OE__GPIO2_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL), // BD_ID_DATA
	MX6_PAD_SD4_DAT3__GPIO2_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL), // Rev A/C strap
	MX6_PAD_CSI0_DAT19__GPIO6_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL), // Rev D strap
	MX6_PAD_EIM_D23__GPIO3_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL), // EN_RTC
	MX6_PAD_EIM_A16__GPIO2_IO22 | MUX_PAD_CTRL(NO_PAD_CTRL), // EN_USB_5V
};


#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	{"emmc", MAKE_CFGVAL(0x60, 0x30, 0x00, 0x00)},
	{NULL,   0},
};
#endif

char board_rev(void)
{
	static int rev = -1;

	if(rev == -1) {
		uint8_t dat;
		// Read REV strapping pin
		gpio_direction_input(TS4900_REVSTRAP);
		gpio_direction_input(TS4900_REVSTRAPD);
		dat = gpio_get_value(TS4900_REVSTRAP);
		if(dat) {
			rev = 'A';
		} else {
			dat = gpio_get_value(TS4900_REVSTRAPD);
			if(dat) {
				rev = 'C';
			} else {
				rev = 'D';
			}
		}
	}

	return rev;
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? (TS4900_SPI_CS) : -1;
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

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
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

static void setup_iomux_enet(void)
{
	// Assert reset
	gpio_direction_output(TS4900_PHY_RST, 1);
	imx_iomux_v3_setup_multiple_pads(enet_pads1, ARRAY_SIZE(enet_pads1));

	gpio_direction_output(TS4900_RGMII_RXC, 1);
	gpio_direction_output(TS4900_RGMII_RD0, 1);
	gpio_direction_output(TS4900_RGMII_RD1, 1);
	gpio_direction_output(TS4900_RGMII_RD2, 1);
	gpio_direction_output(TS4900_RGMII_RD3, 1);
	imx_iomux_v3_setup_multiple_pads(enet_pads1, ARRAY_SIZE(enet_pads1));
	gpio_direction_output(TS4900_RGMII_RX_CTL, 1);

	/* Need delay at least 10ms according to KSZ9031 spec */
	udelay(1000 * 100);

	// De-assert reset
	gpio_direction_output(TS4900_PHY_RST, 0);

	/* Need 100us delay to exit from reset. */
	udelay(1000 * 100);

	imx_iomux_v3_setup_multiple_pads(enet_pads2, ARRAY_SIZE(enet_pads2));
}

iomux_v3_cfg_t const fpga_pads[] = {
	MX6_PAD_CSI0_DATA_EN__GPIO5_IO20 | MUX_PAD_CTRL(SPI_PAD_CTRL), // FPGA_DONE
	MX6_PAD_CSI0_VSYNC__GPIO5_IO21 | MUX_PAD_CTRL(SPI_PAD_CTRL),// FPGA_RESET
	MX6_PAD_CSI0_DAT16__GPIO6_IO02 | MUX_PAD_CTRL(SPI_PAD_CTRL), // FPGA_SPI_CS#
	MX6_PAD_CSI0_DAT10__ECSPI2_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT9__ECSPI2_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT8__ECSPI2_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__GPIO5_IO29 | MUX_PAD_CTRL(SPI_PAD_CTRL), // offboard CS#
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
	MX6_PAD_SD3_RST__GPIO7_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FSL_ESDHC
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

	gpio_direction_output(TS4900_EN_SDPWR, 1); // EN_SD_POWER#
	udelay(1000);
	gpio_direction_output(TS4900_EN_SDPWR, 0);
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
#endif

int board_phy_config(struct phy_device *phydev)
{
	ksz9031_phy_extended_write(phydev, 0x2, 0x8, 0x4000, 0x3EF);
	ksz9031_phy_extended_write(phydev, 0x0, 0x3, 0x4000, 0x1A80);
	ksz9031_phy_extended_write(phydev, 0x0, 0x4, 0x4000, 0x0006);

	if (phydev->drv->config)
		phydev->drv->config(phydev); 

	return 0;
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
	char enet1addr[18];
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

#ifdef CONFIG_RANDOM_MACADDR
	if (!eth_getenv_enetaddr("ethaddr", enetaddr)) {
		printf("No MAC address set in fuses.  Using random mac address.\n");
		eth_random_addr(enetaddr);
		random_mac = 1;
		if (eth_setenv_enetaddr("ethaddr", enetaddr)) {
			printf("Failed to set ethernet address\n");
		}
	} else {
		/* Each board is allocated two sequential mac addresses.
		 * This is used for USB ethernets or the I210 on a carrier board */
		if (enetaddr[5] == 0xff) {
			if (enetaddr[4] == 0xff) {
				enetaddr[3]++;
			}
			enetaddr[4]++;
		}
		enetaddr[5]++;

		snprintf(enet1addr, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
				 enetaddr[0],
				 enetaddr[1],
				 enetaddr[2],
				 enetaddr[3],
				 enetaddr[4],
				 enetaddr[5]);
		setenv("eth1addr", enet1addr);
	}

#endif
	
	return ret;
}

int board_early_init_f(void)
{
	setup_iomux_uart();
	return 0;
}

int misc_init_r(void)
{
	int sdboot;
	char rev[2] = {0, 0};
	struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	
	imx_iomux_v3_setup_multiple_pads(misc_pads, ARRAY_SIZE(misc_pads));

	// Set OFF_BD_RESET low and check if the SD boot jumper is on
	gpio_direction_output(TS4900_OFFBD_RST, 0);
	gpio_direction_input(TS4900_SDBOOT);
	gpio_direction_output(TS4900_EN_5V, 0); // EN_USB_5V
	udelay(1000);
	sdboot = gpio_get_value(TS4900_SDBOOT);
	// OFF_BD_RESET should be left high to diable reset of offboard peripherals
	gpio_direction_output(TS4900_OFFBD_RST, 1);

	// Pulse EN_USB_5V - allow time for usb hubs coming out 
	// of reset from off_bd_reset
	udelay(1000 * 100); // 100ms
	gpio_direction_output(TS4900_EN_5V, 1);

	if(sdboot) setenv("jpsdboot", "off");
	else setenv("jpsdboot", "on");

	setenv("imx_type", CONFIG_IMX_TYPE);

	#ifdef CONFIG_MX6Q
	setenv("cpu", "q");
	#else
	setenv("cpu", "dl");
	#endif

	setenv("model", "4900");
	setenv("rcause", get_reset_cause(1));
	rev[0] = board_rev();
	setenv("rev", rev);

	/* PCIE does not get properly disabled from a watchdog reset.  This prevents 
	 * a hang in the kernel if pcie was enabled in a previous boot. */
	setbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_TEST_POWERDOWN);
	clrbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_REF_SSP_EN);

	return 0;
}

void setup_fpga(void)
{
	imx_iomux_v3_setup_multiple_pads(fpga_pads, ARRAY_SIZE(fpga_pads));	
}

struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode  = MX6_PAD_EIM_D21__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_D21__GPIO3_IO21 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = TS4900_SCL
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D28__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_D28__GPIO3_IO28 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = TS4900_SDA
	}
};

int board_init(void)
{
	int i;
	char rev;
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	setup_spi();
	setup_fpga();

	rev = board_rev();

	/* The Intersil RTC does not behave correctly on every boot.  When it
	 * fails it locks up the I2C bus by driving it to ~1.5V.  It should only
	 * be open drain, but the theory is that memory is corrupted on startup
	 * on the RTC itself causing it to have a seemingly random behavior.
	 * The >= REVB boards include a FET to toggle on/off power to the RTC.  If
	 * we catch the force_idle_bus failing, then then off the fet, drive the pins 
	 * low, wait, turn it back on, and this seems to fix the RTC issues. */
	if(rev != 'A') {
		// EN RTC fet
		gpio_direction_output(TS4900_ENRTC, 0);
		udelay(1000*2); // 2ms to turn on

		/* 5 is an arbitrary magic number.  2 should be enough, but 5 is 
		 * including overkill and doesn't take very long if it were to fail up to 5 */
		for (i = 0; i < 5; i++)
		{
			if (force_idle_bus(&i2c_pad_info0) == 0){
				if(i != 0)
					printf("Recovered I2C\n");
				break;
			}
			puts("Attempting to reset I2C\n");

			// Drive I2C pins low
			imx_iomux_v3_setup_pad(i2c_pad_info0.sda.gpio_mode);
			imx_iomux_v3_setup_pad(i2c_pad_info0.scl.gpio_mode);
			gpio_direction_output(i2c_pad_info0.sda.gp, 0);
			gpio_direction_output(i2c_pad_info0.scl.gp, 0);

			// Enable RTC FET
			gpio_direction_output(TS4900_ENRTC, 1);
			udelay(1000*140); // 140ms to discharge
			gpio_direction_output(TS4900_ENRTC, 0);
			udelay(1000*2); // 2ms to turn on
			imx_iomux_v3_setup_pad(i2c_pad_info0.sda.i2c_mode);
			imx_iomux_v3_setup_pad(i2c_pad_info0.scl.i2c_mode);

			if(i == 4) puts ("Not able to force bus idle.  Giving up.\n");
		}
	}
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);

	#ifdef CONFIG_CMD_SATA
	setup_sata();
	#endif

	#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
	#endif

	return 0;
}

int board_late_init(void)
{
	return 0;
}

int checkboard(void)
{
	puts("Board: TS-4900\n");
	printf("Revision: %c\n", board_rev());

	return 0;
}
