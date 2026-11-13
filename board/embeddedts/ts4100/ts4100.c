// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016-2022 Technologic Systems, Inc. dba embeddedTS
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
#include <dm.h>
#include <i2c.h>
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
#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP | \
			 PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define EN_ETH_PHY_PWR	IMX_GPIO_NR(1, 10)
#define PHY1_DUPLEX	IMX_GPIO_NR(2, 0)
#define PHY2_DUPLEX	IMX_GPIO_NR(2, 8)
#define PHY1_PHYADDR2	IMX_GPIO_NR(2, 1)
#define PHY2_PHYADDR2	IMX_GPIO_NR(2, 9)
#define PHY1_CONFIG_2	IMX_GPIO_NR(2, 2)
#define PHY2_CONFIG_2	IMX_GPIO_NR(2, 10)
#define PHY1_ISOLATE	IMX_GPIO_NR(2, 7)
#define PHY2_ISOLATE	IMX_GPIO_NR(2, 15)
#define EN_EMMC_PWR	IMX_GPIO_NR(5, 2)
#define TS4100_FPGA_GPIO_I2C	2
#define ETH_PHY_RESET		24

static const iomux_v3_cfg_t uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static const iomux_v3_cfg_t emmc_power_pads[] = {
	MX6_PAD_SNVS_TAMPER2__GPIO5_IO02 | MUX_PAD_CTRL(GPIO_PAD_CTRL),
};

static const iomux_v3_cfg_t fpga_i2c_pads[] = {
	MX6_PAD_LCD_DATA01__I2C3_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
	MX6_PAD_LCD_DATA00__I2C3_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
};

static const iomux_v3_cfg_t fec_strap_pads[] = {
	MX6_PAD_GPIO1_IO06__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA0__GPIO2_IO00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_DATA0__GPIO2_IO08 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA1__GPIO2_IO01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_DATA1__GPIO2_IO09 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_EN__GPIO2_IO02 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_EN__GPIO2_IO10 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_ER__GPIO2_IO07 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_ER__GPIO2_IO15 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_JTAG_MOD__GPIO1_IO10 | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static const iomux_v3_cfg_t fec_pads[] = {
	MX6_PAD_GPIO1_IO06__ENET1_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_EN__ENET1_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_ER__ENET1_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA0__ENET1_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA1__ENET1_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_EN__ENET1_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA0__ENET1_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA1__ENET1_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_CLK__ENET1_REF_CLK1 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	MX6_PAD_ENET2_RX_EN__ENET2_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_ER__ENET2_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_DATA0__ENET2_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_RX_DATA1__ENET2_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_EN__ENET2_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA0__ENET2_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_DATA1__ENET2_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_CLK__ENET2_REF_CLK2 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
};

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

int board_early_init_f(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	imx_iomux_v3_setup_multiple_pads(emmc_power_pads,
					 ARRAY_SIZE(emmc_power_pads));
	gpio_request(EN_EMMC_PWR, "EN_EMMC_PWR");
	gpio_direction_output(EN_EMMC_PWR, 1);

	return 0;
}

static void setup_fec(void)
{
	struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
			IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC2_MASK,
			IOMUX_GPR1_FEC2_CLOCK_MUX1_SEL_MASK);
	enable_fec_anatop_clock(0, ENET_50MHZ);
	enable_fec_anatop_clock(1, ENET_50MHZ);
	enable_enet_clk(1);
}

static int ts4100_fpga_gpio_output(unsigned int gpio, int value)
{
	struct udevice *chip;
	u8 reg;
	int ret;

	ret = i2c_get_chip_for_busnum(TS4100_FPGA_GPIO_I2C, 0x28, 2, &chip);
	if (ret)
		return ret;

	ret = dm_i2c_read(chip, gpio, &reg, sizeof(reg));
	if (ret)
		return ret;

	if (value)
		reg |= BIT(1);
	else
		reg &= ~BIT(1);

	ret = dm_i2c_write(chip, gpio, &reg, sizeof(reg));
	if (ret || (reg & BIT(0)))
		return ret;

	reg |= BIT(0);
	return dm_i2c_write(chip, gpio, &reg, sizeof(reg));
}

static void setup_phys(void)
{
	int ret;

	/* The MXC I2C driver does not configure pinmux from the device tree. */
	imx_iomux_v3_setup_multiple_pads(fpga_i2c_pads,
					 ARRAY_SIZE(fpga_i2c_pads));
	imx_iomux_v3_setup_multiple_pads(fec_strap_pads,
					 ARRAY_SIZE(fec_strap_pads));
	gpio_request(EN_ETH_PHY_PWR, "EN_ETH_PHY_PWR");
	gpio_request(PHY1_DUPLEX, "PHY1_DUPLEX");
	gpio_request(PHY2_DUPLEX, "PHY2_DUPLEX");
	gpio_request(PHY1_PHYADDR2, "PHY1_PHYADDR2");
	gpio_request(PHY2_PHYADDR2, "PHY2_PHYADDR2");
	gpio_request(PHY1_CONFIG_2, "PHY1_CONFIG_2");
	gpio_request(PHY2_CONFIG_2, "PHY2_CONFIG_2");
	gpio_request(PHY1_ISOLATE, "PHY1_ISOLATE");
	gpio_request(PHY2_ISOLATE, "PHY2_ISOLATE");

	gpio_direction_output(EN_ETH_PHY_PWR, 0);
	gpio_direction_output(PHY1_DUPLEX, 0);
	gpio_direction_output(PHY2_DUPLEX, 0);
	gpio_direction_output(PHY1_PHYADDR2, 0);
	gpio_direction_output(PHY2_PHYADDR2, 0);
	gpio_direction_output(PHY1_CONFIG_2, 0);
	gpio_direction_output(PHY2_CONFIG_2, 0);
	gpio_direction_output(PHY1_ISOLATE, 0);
	gpio_direction_output(PHY2_ISOLATE, 0);
	/* ETH_PHY_RESET# is active low: hold the PHY in reset while powering it. */
	ret = ts4100_fpga_gpio_output(ETH_PHY_RESET, 0);
	if (ret)
		printf("Warning: could not assert Ethernet PHY reset (%d)\n", ret);
	mdelay(5);
	gpio_direction_output(EN_ETH_PHY_PWR, 1);
	mdelay(15);
	ret = ts4100_fpga_gpio_output(ETH_PHY_RESET, 1);
	if (ret)
		printf("Warning: could not release Ethernet PHY reset (%d)\n", ret);
	imx_iomux_v3_setup_multiple_pads(fec_pads, ARRAY_SIZE(fec_pads));
}

int board_init(void)
{
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	return 0;
}

int board_late_init(void)
{
	setup_phys();
	setup_fec();

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	phydev->bus->write(phydev->bus, 1, MDIO_DEVAD_NONE, 0x0, 0x8000);
	phydev->bus->write(phydev->bus, 1, MDIO_DEVAD_NONE, 0x16, 0x0202);
	phydev->bus->write(phydev->bus, 2, MDIO_DEVAD_NONE, 0x16, 0x0202);
	phydev->bus->write(phydev->bus, 1, MDIO_DEVAD_NONE, 0x1f, 0x8180);
	phydev->bus->write(phydev->bus, 2, MDIO_DEVAD_NONE, 0x1f, 0x8180);

	return 0;
}

u32 get_board_rev(void)
{
	return get_cpu_rev();
}

int checkboard(void)
{
	puts("Board: embeddedTS TS-4100\n");

	return 0;
}
