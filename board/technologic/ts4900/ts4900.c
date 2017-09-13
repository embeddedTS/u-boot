/*
 * Copyright (C) 2016 Technologic Systems
 *
 * Author: Mark Featherston <mark@embeddedarm.com.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/mach-imx/sata.h>
#include <asm/mach-imx/spi.h>
#include <fpga.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <lattice.h>
#include <malloc.h>
#include <micrel.h>
#include <miiphy.h>
#include <mmc.h>
#include <netdev.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL (PAD_CTL_SPEED_MED | PAD_CTL_DSE_80ohm)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm     | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define WEAK_PULLUP		(PAD_CTL_PUS_100K_UP |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |	\
	PAD_CTL_SRE_SLOW)

#define WEAK_PULLDOWN	(PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_HYS | PAD_CTL_SRE_SLOW)

#define EN_5V			IMX_GPIO_NR(2, 22)
#define OFFBD_RST		IMX_GPIO_NR(2, 21)
#define SDBOOT			IMX_GPIO_NR(2, 26)
#define ICE_DONE		IMX_GPIO_NR(5, 20)
#define ICE_RST			IMX_GPIO_NR(5, 21)
#define ICE_CS			IMX_GPIO_NR(6, 2)
#define ENRTC			IMX_GPIO_NR(3, 23)
#define REVSTRAPA		IMX_GPIO_NR(2, 11)
#define REVSTRAPD		IMX_GPIO_NR(6, 5)
#define SPI_CS			IMX_GPIO_NR(3, 19)
#define PHY_RST			IMX_GPIO_NR(4, 20)
#define RGMII_RXC		IMX_GPIO_NR(6, 30)
#define RGMII_RD0		IMX_GPIO_NR(6, 25)
#define RGMII_RD1		IMX_GPIO_NR(6, 27)
#define RGMII_RD2		IMX_GPIO_NR(6, 28)
#define RGMII_RD3		IMX_GPIO_NR(6, 29)
#define RGMII_RX_CTL	IMX_GPIO_NR(6, 24)
#define EN_SDPWR		IMX_GPIO_NR(2, 28)
#define BB_S0			IMX_GPIO_NR(1, 2)
#define BB_S1			IMX_GPIO_NR(2, 24)
#define BB_S2			IMX_GPIO_NR(2, 26)
#define BB_IN			IMX_GPIO_NR(2, 25)

iomux_v3_cfg_t const misc_pads[] = {
	MX6_PAD_EIM_A17__GPIO2_IO21 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_GPIO_2__GPIO1_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_EIM_CS1__GPIO2_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_EIM_RW__GPIO2_IO26 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_EIM_OE__GPIO2_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SD4_DAT3__GPIO2_IO11 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_CSI0_DAT19__GPIO6_IO05 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_EIM_D23__GPIO3_IO23 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_EIM_A16__GPIO2_IO22 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_SD3_DAT7__UART1_TX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_SD3_DAT6__UART1_RX_DATA | MUX_PAD_CTRL(UART_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads1[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TXC__RGMII_TXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD0__RGMII_TD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD1__RGMII_TD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD2__RGMII_TD2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TD3__RGMII_TD3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK | MUX_PAD_CTRL(ENET_PAD_CTRL),
	/* pin 35 - 1 (PHY_AD2) on reset */
	MX6_PAD_RGMII_RXC__GPIO6_IO30 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 32 - 1 - (MODE0) all */
	MX6_PAD_RGMII_RD0__GPIO6_IO25 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 31 - 1 - (MODE1) all */
	MX6_PAD_RGMII_RD1__GPIO6_IO27 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 28 - 1 - (MODE2) all */
	MX6_PAD_RGMII_RD2__GPIO6_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 27 - 1 - (MODE3) all */
	MX6_PAD_RGMII_RD3__GPIO6_IO29 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* pin 33 - 1 - (CLK125_EN) 125Mhz clockout enabled */
	MX6_PAD_RGMII_RX_CTL__GPIO6_IO24 | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* PHY RESET */
	MX6_PAD_DI0_PIN4__GPIO4_IO20 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

iomux_v3_cfg_t const enet_pads2[] = {
	MX6_PAD_RGMII_RXC__RGMII_RXC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD0__RGMII_RD0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD1__RGMII_RD1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD2__RGMII_RD2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RD3__RGMII_RD3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

/* SD card */
iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__SD2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__SD2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__SD2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__SD2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__SD2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__SD2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* EN_SD_POWER*/
	MX6_PAD_EIM_EB0__GPIO2_IO28 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

/* MMC */
iomux_v3_cfg_t const usdhc3_pads[] = {
	MX6_PAD_SD3_CLK__SD3_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_CMD__SD3_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT0__SD3_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT1__SD3_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT2__SD3_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_DAT3__SD3_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD3_RST__GPIO7_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#if defined(CONFIG_FPGA)

iomux_v3_cfg_t const fpga_pads[] = {
	MX6_PAD_CSI0_DATA_EN__GPIO5_IO20 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_VSYNC__GPIO5_IO21 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT16__GPIO6_IO02 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT10__ECSPI2_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT9__ECSPI2_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT8__ECSPI2_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__GPIO5_IO29 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_GPIO_3__XTALOSC_REF_CLK_24M | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void ts4900_fpga_spi_init(void)
{
	imx_iomux_v3_setup_multiple_pads(fpga_pads,
					 ARRAY_SIZE(fpga_pads));
	gpio_direction_output(ICE_CS, 1);
	gpio_direction_output(ICE_RST, 1);
	gpio_direction_input(ICE_DONE);

	return;
}

static int ts4900_spi_get_done(void)
{
	return gpio_get_value(ICE_DONE);
}

static void ts4900_spi_set_rst(int val)
{
	gpio_set_value(ICE_RST, val);
}

static void ts4900_spi_set_cs(int val)
{
	gpio_set_value(ICE_CS, val);
}

lattice_board_specific_func ts4900_fpga_fns = {
	.spi_init = ts4900_fpga_spi_init,
	.spi_get_done = ts4900_spi_get_done,
	.spi_set_cs = ts4900_spi_set_cs,
	.spi_set_rst = ts4900_spi_set_rst
};

Lattice_desc ts4900_fpga = {
	ICE40,
	lattice_spi_mode,
	118642,
	(void *)&ts4900_fpga_fns,
	NULL,
	ICE40_SPI_COOKIE(1, 1),
	"ice40hx8k"
};

int setup_fpga(void)
{
	/* Keep FPGA in reset on startup until reprogrammed */
	ts4900_spi_set_rst(1);
	fpga_init();

	fpga_add(fpga_lattice, &ts4900_fpga);

	return 0;
}
#endif

#ifdef CONFIG_CMD_BMODE
/* SD is on a powered off by default.  Only eMMC can be used with bmode */
static const struct boot_mode board_boot_modes[] = {
	{"emmc", MAKE_CFGVAL(0x60, 0x30, 0x00, 0x00)},
	{NULL,   0},
};
#endif

char board_rev(void)
{
	static int rev = -1;

	if (rev == -1) {
		uint8_t dat;
		gpio_direction_input(REVSTRAPA);
		gpio_direction_input(REVSTRAPD);
		dat = gpio_get_value(REVSTRAPA);
		if (dat) {
			rev = 'A';
		} else {
			dat = gpio_get_value(REVSTRAPD);
			if (dat)
				rev = 'C';
			else
				rev = 'D';
		}
	}

	return rev;
}

#ifdef CONFIG_MXC_SPI

iomux_v3_cfg_t const ecspi1_pads[] = {
	MX6_PAD_EIM_D19__GPIO3_IO19   | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D17__ECSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D18__ECSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EIM_D16__ECSPI1_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	return (bus == 0 && cs == 0) ? (SPI_CS) : -1;
}

void setup_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads,
					 ARRAY_SIZE(ecspi1_pads));

	/* Enable SPI clk */
	setbits_le32(CCM_CCGR1, MXC_CCM_CCGR1_ECSPI2S_MASK);
}

#endif

int dram_init(void)
{
	gd->ram_size = (phys_size_t)CONFIG_DDR_MB * 1024 * 1024;
	return 0;
}

static void setup_iomux_enet(void)
{
	gpio_direction_output(PHY_RST, 1);
	imx_iomux_v3_setup_multiple_pads(enet_pads1, ARRAY_SIZE(enet_pads1));

	gpio_direction_output(RGMII_RXC, 1);
	gpio_direction_output(RGMII_RD0, 1);
	gpio_direction_output(RGMII_RD1, 1);
	gpio_direction_output(RGMII_RD2, 1);
	gpio_direction_output(RGMII_RD3, 1);
	imx_iomux_v3_setup_multiple_pads(enet_pads1, ARRAY_SIZE(enet_pads1));
	gpio_direction_output(RGMII_RX_CTL, 1);

	/* Need delay at least 10ms according to KSZ9031 spec */
	udelay(1000 * 100);
	gpio_direction_output(PHY_RST, 0);

	/* Need 100us delay to exit from reset. */
	udelay(100);

	imx_iomux_v3_setup_multiple_pads(enet_pads2, ARRAY_SIZE(enet_pads2));
}

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
	case USDHC2_BASE_ADDR: /* SD */
		ret = 1;
		break;
	case USDHC3_BASE_ADDR: /* MMC */
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

	/* Maybe not as needed with recent cards, but previously we have
	 * observed some SD cards getting stuck with a power reset as the only
	 * way to recover. */
	gpio_direction_output(EN_SDPWR, 1);
	udelay(1000);
	gpio_direction_output(EN_SDPWR, 0);
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    SD
	 * mmc1                    eMMC
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			usdhc_cfg[0].max_bus_width = 4;
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

			break;
		case 1:
			usdhc_cfg[1].max_bus_width = 4;
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers\n"
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

	if (!eth_env_get_enetaddr("ethaddr", enetaddr)) {
#ifdef CONFIG_RANDOM_MACADDR
		uint32_t uniq1, uniq2;
		int i;
		printf("No MAC address set in fuses. Using random MAC.\n");

		/* Similar to eth_random_addr, but use seed unique to cpu */
		fuse_read(0, 1, &uniq1);
		fuse_read(0, 2, &uniq2);

		srand(uniq1 ^ uniq2);
		for (i = 0; i < 6; i++)
			enetaddr[i] = rand();
		enetaddr[0] &= 0xfe; /* clear multicast bit */
		enetaddr[0] |= 0x02; /* set local assignment bit (IEEE802) */
		if (eth_env_set_enetaddr("ethaddr", enetaddr))
			printf("Failed to set ethernet address\n");
#endif
	}

	/* Each board is allocated two sequential mac addresses.
	 * This is commonly used for USB ethernets or the I210
	 * on a carrier board */
	if (enetaddr[5] == 0xff) {
		if (enetaddr[4] == 0xff)
			enetaddr[3]++;
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
	env_set("eth1addr", enet1addr);

	return ret;
}

int board_early_init_f(void)
{
	setup_iomux_uart();
	return 0;
}

/* Gets ID from hard wired 3-state 8-input multiplexer
 * https://wiki.embeddedarm.com/wiki/TS-4900#Baseboard_ID
 */
static int bbdetect(void)
{
	int i, id = 0;
	gpio_direction_output(BB_S0, 1);
	gpio_direction_output(BB_S1, 1);
	gpio_direction_output(BB_S2, 1);
	gpio_direction_input(BB_IN);

	for (i = 0; i < 8; i++) {
		if (i & 1)
			gpio_set_value(BB_S0, 1);
		else
			gpio_set_value(BB_S0, 0);

		if (i & 2)
			gpio_set_value(BB_S1, 1);
		else
			gpio_set_value(BB_S1, 0);

		if (i & 4)
			gpio_set_value(BB_S2, 1);
		else
			gpio_set_value(BB_S2, 0);

		udelay(1000);

		id = (id >> 1);
		if (gpio_get_value(BB_IN))
			id |= 0x80;
	}

	env_set_hex("baseboardid", id & ~0xc0);
	env_set_hex("baseboardrev", ((id & 0xc0) >> 6));
	return 0;
}

int misc_init_r(void)
{
	int sdboot;
	char rev[2] = {0, 0};

	imx_iomux_v3_setup_multiple_pads(misc_pads, ARRAY_SIZE(misc_pads));

	/* off_bd_reset is used to pulse a reset for common off board
	 * peripherals. USB 5V is given an extra 100ms which covers most USB
	 * hubs to come out of reset.  SD boot is also read now because it can
	 * only be read while off_bd_rst is asserted */
	gpio_direction_output(OFFBD_RST, 0);
	gpio_direction_input(SDBOOT);
	gpio_direction_output(EN_5V, 0);
	mdelay(1);
	sdboot = gpio_get_value(SDBOOT);
	gpio_direction_output(OFFBD_RST, 1);
	mdelay(100);
	gpio_direction_output(EN_5V, 1);

	if (sdboot)
		env_set("jpsdboot", "off");
	else
		env_set("jpsdboot", "on");

	env_set("model", "4900");
	env_set_hex("reset_cause", get_imx_reset_cause());
	rev[0] = board_rev();
	env_set("rev", rev);

	/* Determine the baseboard so we know which device tree to load */
	bbdetect();

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

	return 0;
}

struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6_PAD_EIM_D21__I2C1_SCL |
			    MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_D21__GPIO3_IO21 |
			     MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 21)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D28__I2C1_SDA |
			    MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_D28__GPIO3_IO28 |
			     MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 28)
	}
};

int board_init(void)
{
	struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif

#ifdef CONFIG_FPGA
	setup_fpga();
#endif

#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

	/* RTC Requires startup sequence with all IO driven low */
	imx_iomux_v3_setup_pad(i2c_pad_info0.sda.gpio_mode);
	imx_iomux_v3_setup_pad(i2c_pad_info0.scl.gpio_mode);
	gpio_direction_output(i2c_pad_info0.sda.gp, 0);
	gpio_direction_output(i2c_pad_info0.scl.gp, 0);
	gpio_direction_output(ENRTC, 1);
	/* Takes 140ms to completely drop voltage */
	mdelay(140);
	gpio_direction_output(ENRTC, 0);
	mdelay(2);

#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
#endif

	/* Pick GPIO1 for USB OTG id */
	clrsetbits_le32(&iomuxc_regs->gpr[1],
			IOMUXC_GPR1_OTG_ID_MASK,
			IOMUXC_GPR1_OTG_ID_GPIO1);

	return 0;
}

int checkboard(void)
{
	puts("Board: TS-4900\n");
	printf("Revision: %c\n", board_rev());

	return 0;
}
