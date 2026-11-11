// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2017-2022 Technologic Systems, Inc. dba embeddedTS
 */

#include <init.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <miiphy.h>
#include <phy.h>
#include <linux/delay.h>
#include <linux/sizes.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP | \
			 PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | \
			 PAD_CTL_SRE_FAST | PAD_CTL_HYS)
#define GPIO_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP | \
			 PAD_CTL_DSE_48ohm | PAD_CTL_SRE_FAST)
#define ENET_PAD_CTRL	(PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
			 PAD_CTL_SPEED_HIGH | PAD_CTL_DSE_48ohm | \
			 PAD_CTL_SRE_FAST)
#define ENET_CLK_PAD_CTRL (PAD_CTL_SPEED_MED | PAD_CTL_DSE_120ohm | \
			      PAD_CTL_SRE_FAST)

#define EN_ETH_PHY_PWR	IMX_GPIO_NR(2, 10)
#define ETH_PHY_RSTN	IMX_GPIO_NR(1, 4)
#define PHY_DUPLEX	IMX_GPIO_NR(2, 0)
#define PHY_ADDR2	IMX_GPIO_NR(2, 1)
#define PHY_CONFIG2	IMX_GPIO_NR(2, 2)
#define PHY_ISOLATE	IMX_GPIO_NR(2, 7)
#define EN_SD_PWR	IMX_GPIO_NR(3, 12)
#define EN_EMMC_PWR	IMX_GPIO_NR(5, 2)
#define USDHC1_VSELECT	IMX_GPIO_NR(1, 5)

static const iomux_v3_cfg_t uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static const iomux_v3_cfg_t storage_power_pads[] = {
	MX6_PAD_LCD_DATA07__GPIO3_IO12 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_GPIO1_IO05__USDHC1_VSELECT | MUX_PAD_CTRL(GPIO_PAD_CTRL),
	MX6_PAD_SNVS_TAMPER2__GPIO5_IO02 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static const iomux_v3_cfg_t fec_strap_pads[] = {
	MX6_PAD_ENET1_RX_DATA0__GPIO2_IO00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA1__GPIO2_IO01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_EN__GPIO2_IO02 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_ER__GPIO2_IO07 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_EN__GPIO2_IO10 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO1_IO04__GPIO1_IO04 | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static const iomux_v3_cfg_t fec_pads[] = {
	MX6_PAD_GPIO1_IO06__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA0__ENET1_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA1__ENET1_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_EN__ENET1_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_CLK__ENET1_REF_CLK1 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA0__ENET1_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA1__ENET1_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_ER__ENET1_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_EN__ENET1_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

int dram_init(void)
{
	gd->ram_size = SZ_512M;

	return 0;
}

int board_early_init_f(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(storage_power_pads,
					 ARRAY_SIZE(storage_power_pads));

	gpio_request(EN_EMMC_PWR, "EN_EMMC_PWR");
	gpio_request(EN_SD_PWR, "EN_SD_PWR");
	gpio_request(USDHC1_VSELECT, "USDHC1_VSELECT");

	/* The soldered eMMC is powered through this GPIO. */
	gpio_direction_output(EN_EMMC_PWR, 1);

	/* Enable the SD socket and force its signalling to 3.3 V. */
	gpio_direction_output(EN_SD_PWR, 1);
	gpio_direction_output(USDHC1_VSELECT, 0);

	return 0;
}

static void setup_fec(void)
{
	struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
			IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);
	enable_fec_anatop_clock(0, ENET_50MHZ);
	enable_enet_clk(1);
}

static void ts7553v2_reset_phy(void)
{
	imx_iomux_v3_setup_multiple_pads(fec_strap_pads,
					 ARRAY_SIZE(fec_strap_pads));
	gpio_request(PHY_DUPLEX, "PHY_DUPLEX");
	gpio_request(PHY_ADDR2, "PHY_ADDR2");
	gpio_request(PHY_CONFIG2, "PHY_CONFIG2");
	gpio_request(PHY_ISOLATE, "PHY_ISOLATE");
	gpio_request(ETH_PHY_RSTN, "ETH_PHY_RSTN");
	gpio_request(EN_ETH_PHY_PWR, "EN_ETH_PHY_PWR");

	/* PHY address, RMII mode and duplex are latched as reset is released. */
	gpio_direction_output(PHY_DUPLEX, 0);
	gpio_direction_output(PHY_ADDR2, 0);
	gpio_direction_output(PHY_CONFIG2, 0);
	gpio_direction_output(PHY_ISOLATE, 0);
	gpio_direction_output(ETH_PHY_RSTN, 0);
	gpio_direction_output(EN_ETH_PHY_PWR, 0);
	mdelay(5);
	gpio_direction_output(EN_ETH_PHY_PWR, 1);
	mdelay(10);
	gpio_direction_output(ETH_PHY_RSTN, 1);
	mdelay(1);

	imx_iomux_v3_setup_multiple_pads(fec_pads, ARRAY_SIZE(fec_pads));
}

int board_init(void)
{
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	ts7553v2_reset_phy();
	setup_fec();

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	/* Configure the KSZ8051 PHY at strap address 2 for RMII and REF_CLK. */
	phydev->bus->write(phydev->bus, 2, MDIO_DEVAD_NONE, 0x0, 0x8000);
	phydev->bus->write(phydev->bus, 2, MDIO_DEVAD_NONE, 0x16, 0x0202);
	phydev->bus->write(phydev->bus, 2, MDIO_DEVAD_NONE, 0x1f, 0x8180);

	return 0;
}

u32 get_board_rev(void)
{
	return get_cpu_rev();
}

int checkboard(void)
{
	puts("Board: embeddedTS TS-7553-V2\n");

	return 0;
}
