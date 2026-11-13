// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 embeddedTS
 */

#include <init.h>
#include <i2c.h>
#include <hang.h>
#include <spl.h>
#include <stdio.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/mx6ul_pins.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/sections.h>
#include <fsl_esdhc_imx.h>

#define UART_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP | \
			 PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | \
			 PAD_CTL_SRE_FAST | PAD_CTL_HYS)
#define I2C_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP | \
			 PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define FPGA_I2C_BUS	2
#define FPGA_I2C_ADDR	0x28
#define FPGA_STRAP_REG	308
#define MMDC_BASE	0x021b0000

static const iomux_v3_cfg_t uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static struct i2c_pads_info fpga_i2c_pads = {
	.scl = {
		.i2c_mode = MX6_PAD_LCD_DATA01__I2C3_SCL |
			    MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_LCD_DATA01__GPIO3_IO06 |
			     MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 6),
	},
	.sda = {
		.i2c_mode = MX6_PAD_LCD_DATA00__I2C3_SDA |
			    MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_LCD_DATA00__GPIO3_IO05 |
			     MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 5),
	},
};

static struct fsl_esdhc_cfg usdhc_cfg = {
	.esdhc_base = USDHC2_BASE_ADDR,
	.max_bus_width = 4,
};

#define USDHC_PAD_CTRL	(PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_22K_UP | \
			 PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm | \
			 PAD_CTL_SRE_FAST | PAD_CTL_HYS)

static const iomux_v3_cfg_t usdhc2_pads[] = {
	MX6_PAD_NAND_RE_B__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_WE_B__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA00__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA01__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA02__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA03__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

struct reg_value {
	u32 reg;
	u32 value;
};

static const struct reg_value iomux_regs[] = {
	{ 0x020e04b4, 0x000c0000 }, { 0x020e04ac, 0x00000000 },
	{ 0x020e027c, 0x00000028 }, { 0x020e0250, 0x00000028 },
	{ 0x020e024c, 0x00000028 }, { 0x020e0490, 0x00000028 },
	{ 0x020e0288, 0x00000028 }, { 0x020e0270, 0x00000000 },
	{ 0x020e0260, 0x00000028 }, { 0x020e0264, 0x00000028 },
	{ 0x020e04a0, 0x00000028 }, { 0x020e0494, 0x00020000 },
	{ 0x020e0280, 0x00000028 }, { 0x020e0284, 0x00000028 },
	{ 0x020e04b0, 0x00020000 }, { 0x020e0498, 0x00000028 },
	{ 0x020e04a4, 0x00000028 }, { 0x020e0244, 0x00000028 },
	{ 0x020e0248, 0x00000028 },
};

static const struct reg_value ddr_common[] = {
	{ 0x081c, 0x33333333 }, { 0x0820, 0x33333333 },
	{ 0x082c, 0xf3333333 }, { 0x0830, 0xf3333333 },
	{ 0x08c0, 0x00921012 }, { 0x08b8, 0x00000800 },
	{ 0x0004, 0x0002002d }, { 0x0008, 0x1b333030 },
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0xffffffff, &ccm->CCGR0);
	writel(0xffffffff, &ccm->CCGR1);
	writel(0xffffffff, &ccm->CCGR2);
	writel(0xffffffff, &ccm->CCGR3);
	writel(0xffffffff, &ccm->CCGR4);
	writel(0xffffffff, &ccm->CCGR5);
	writel(0xffffffff, &ccm->CCGR6);
	writel(0xffffffff, &ccm->CCGR7);
}

static int read_ram_option(u8 *option)
{
	int ret;

	ret = setup_i2c(FPGA_I2C_BUS, 100000, FPGA_I2C_ADDR,
			&fpga_i2c_pads);
	if (ret)
		return ret;

	/* SPL registers only I2C3, so it is adapter 0 in the legacy list. */
	ret = i2c_set_bus_num(0);
	if (ret)
		return ret;

	/* FPGA register 308 contains the resistor strap value. */
	return i2c_read(FPGA_I2C_ADDR, FPGA_STRAP_REG, 2, option, 1);
}

static void write_mmdc_regs(const struct reg_value *regs, uint count)
{
	uint i;

	for (i = 0; i < count; i++)
		writel(regs[i].value, (void *)(MMDC_BASE + regs[i].reg));
}

static void write_iomux_regs(const struct reg_value *regs, uint count)
{
	uint i;

	for (i = 0; i < count; i++)
		writel(regs[i].value, (void *)regs[i].reg);
}

static void init_ddr(bool ram_1g)
{
	/* Keep the established board DCD values, selected by the RAM strap. */
	struct reg_value calibration[] = {
		{ 0x080c, ram_1g ? 0x00120007 : 0x00010000 },
		{ 0x083c, ram_1g ? 0x014a014a : 0x01410140 },
		{ 0x0848, ram_1g ? 0x4040484b : 0x40404c50 },
		{ 0x0850, ram_1g ? 0x40404a3e : 0x4040544b },
	};
	const struct reg_value ddr_start[] = {
		{ 0x001c, 0x00008000 }, { 0x0800, 0xa1390003 },
	};
	const struct reg_value ddr_end[] = {
		{ 0x001c, 0x02008032 }, { 0x001c, 0x00008033 },
		{ 0x001c, 0x00048031 }, { 0x001c, 0x15208030 },
		{ 0x001c, 0x04008040 }, { 0x0020, 0x00007800 },
		{ 0x0818, 0x00000227 }, { 0x0004, 0x0002556d },
		{ 0x0404, 0x00011006 }, { 0x001c, 0x00000000 },
	};

	write_iomux_regs(iomux_regs, ARRAY_SIZE(iomux_regs));
	write_mmdc_regs(ddr_start, ARRAY_SIZE(ddr_start));
	write_mmdc_regs(calibration, ARRAY_SIZE(calibration));
	write_mmdc_regs(ddr_common, ARRAY_SIZE(ddr_common));
	writel(ram_1g ? 0x8b8f52f3 : 0x676b52f3,
	       (void *)(MMDC_BASE + 0x000c));
	writel(0xb66d0b63, (void *)(MMDC_BASE + 0x0010));
	writel(0x01ff00db, (void *)(MMDC_BASE + 0x0014));
	writel(0x00211740, (void *)(MMDC_BASE + 0x0018));
	writel(0x00008000, (void *)(MMDC_BASE + 0x001c));
	writel(0x000026d2, (void *)(MMDC_BASE + 0x002c));
	writel(ram_1g ? 0x008f1023 : 0x006b1023,
	       (void *)(MMDC_BASE + 0x0030));
	writel(ram_1g ? 0x0000005f : 0x0000004f,
	       (void *)(MMDC_BASE + 0x0040));
	writel(ram_1g ? 0x85180000 : 0x84180000,
	       (void *)(MMDC_BASE + 0x0000));
	if (!ram_1g)
		writel(0x00400a38, (void *)(MMDC_BASE + 0x0890));
	write_mmdc_regs(ddr_end, ARRAY_SIZE(ddr_end));
}

void board_init_f(ulong dummy)
{
	u8 option;
	bool ram_1g;
	int ret;

	ccgr_init();
	arch_cpu_init();
	timer_init();
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	preloader_console_init();

	ret = read_ram_option(&option);
	if (ret) {
		printf("TS-4100: failed to read RAM strap (%d)\n", ret);
		hang();
	}

	switch (option & 0x0f) {
	/* 512 MiB options: TS-4100-SMN1I and TS-4100-SMW5I. */
	case 0x05:
	case 0x0f:
		ram_1g = false;
		break;
	/* 1 GiB option: TS-4100-SRW9I. */
	case 0x09:
		ram_1g = true;
		break;
	default:
		printf("TS-4100: unsupported RAM strap 0x%02x\n", option);
		hang();
	}

	printf("TS-4100: initializing %s RAM (strap 0x%02x)\n",
	       ram_1g ? "1 GiB" : "512 MiB", option);
	init_ddr(ram_1g);

	memset(__bss_start, 0, __bss_end - __bss_start);
	board_init_r(NULL, 0);
}

int board_mmc_init(struct bd_info *bis)
{
	imx_iomux_v3_setup_multiple_pads(usdhc2_pads,
					 ARRAY_SIZE(usdhc2_pads));
	usdhc_cfg.sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

	return fsl_esdhc_initialize(bis, &usdhc_cfg);
}
