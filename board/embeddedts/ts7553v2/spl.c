// SPDX-License-Identifier: GPL-2.0+
/*
 * TS-7553-V2 SPL.
 *
 * Keep the current DDR3 profile equivalent to the board's original ROM DCD.
 * Additional RAM variants can select a separate profile here once their
 * hardware identification straps are defined.
 */

#include <init.h>
#include <spl.h>
#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/mx6ul_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/sections.h>
#include <fsl_esdhc_imx.h>
#include <linux/sizes.h>

#define EN_EMMC_PWR IMX_GPIO_NR(5, 2)
#define UART_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP | \
			   PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | \
			   PAD_CTL_SRE_FAST | PAD_CTL_HYS)
#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_47K_UP | \
			    PAD_CTL_SPEED_HIGH | PAD_CTL_DSE_80ohm | \
			    PAD_CTL_SRE_FAST | PAD_CTL_HYS)

struct regval {
	uintptr_t addr;
	u32 value;
};

static const struct regval ddr_profile_512m[] = {
	{ 0x020c4068, 0xffffffff }, { 0x020c406c, 0xffffffff },
	{ 0x020c4070, 0xffffffff }, { 0x020c4074, 0xffffffff },
	{ 0x020c4078, 0xffffffff }, { 0x020c407c, 0xffffffff },
	{ 0x020c4080, 0xffffffff }, { 0x020c4084, 0xffffffff },
	{ 0x020e04b4, 0x000c0000 }, { 0x020e04ac, 0x00000000 },
	{ 0x020e027c, 0x00000010 }, { 0x020e0250, 0x00000010 },
	{ 0x020e024c, 0x00000010 }, { 0x020e0490, 0x00000010 },
	{ 0x020e0288, 0x00000010 }, { 0x020e0270, 0x00000000 },
	{ 0x020e0260, 0x00000010 }, { 0x020e0264, 0x00000010 },
	{ 0x020e04a0, 0x00000010 }, { 0x020e0494, 0x00020000 },
	{ 0x020e0280, 0x00000010 }, { 0x020e0284, 0x00000010 },
	{ 0x020e04b0, 0x00020000 }, { 0x020e0498, 0x00000010 },
	{ 0x020e04a4, 0x00000010 }, { 0x020e0244, 0x00000010 },
	{ 0x020e0248, 0x00000010 },
	{ 0x021b001c, 0x00008000 }, { 0x021b0800, 0xa1390003 },
	{ 0x021b080c, 0x00040000 }, { 0x021b083c, 0x01400140 },
	{ 0x021b0848, 0x40404a4e }, { 0x021b0850, 0x40405046 },
	{ 0x021b081c, 0x33333333 }, { 0x021b0820, 0x33333333 },
	{ 0x021b082c, 0xf3333333 }, { 0x021b0830, 0xf3333333 },
	{ 0x021b08c0, 0x00921012 }, { 0x021b08b8, 0x00000800 },
	{ 0x021b0004, 0x0002002d }, { 0x021b0008, 0x1b333030 },
	{ 0x021b000c, 0x676b52f3 }, { 0x021b0010, 0xb66d0b63 },
	{ 0x021b0014, 0x01ff00db }, { 0x021b0018, 0x00201740 },
	{ 0x021b001c, 0x00008000 }, { 0x021b002c, 0x000026d2 },
	{ 0x021b0030, 0x006b1023 }, { 0x021b0040, 0x0000005f },
	{ 0x021b0000, 0x85180000 }, { 0x021b001c, 0x02008032 },
	{ 0x021b001c, 0x00008033 }, { 0x021b001c, 0x00048031 },
	{ 0x021b001c, 0x15208030 }, { 0x021b001c, 0x04008040 },
	{ 0x021b0020, 0x00000800 }, { 0x021b0818, 0x00000227 },
	{ 0x021b0004, 0x0002552d }, { 0x021b0404, 0x00011006 },
	{ 0x021b001c, 0x00000000 },
};

static const iomux_v3_cfg_t uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static const iomux_v3_cfg_t emmc_power_pad[] = {
	MX6_PAD_SNVS_TAMPER2__GPIO5_IO02 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static const iomux_v3_cfg_t usdhc2_pads[] = {
	MX6_PAD_NAND_RE_B__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_WE_B__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA00__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA01__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA02__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA03__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static struct fsl_esdhc_cfg usdhc2_cfg = {
	.esdhc_base = USDHC2_BASE_ADDR,
	.max_bus_width = 4,
};

static void ddr_init(void)
{
	unsigned int i;

	/* Current production memory is the 512 MiB DDR3 profile. */
	for (i = 0; i < ARRAY_SIZE(ddr_profile_512m); i++)
		writel(ddr_profile_512m[i].value,
		       (void __iomem *)ddr_profile_512m[i].addr);
}

void board_init_f(ulong dummy)
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
	arch_cpu_init();
	timer_init();
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
	preloader_console_init();
	ddr_init();
	memset(__bss_start, 0, __bss_end - __bss_start);
	board_init_r(NULL, 0);
}

int board_mmc_getcd(struct mmc *mmc)
{
	return 1;
}

int board_mmc_init(struct bd_info *bis)
{
	imx_iomux_v3_setup_multiple_pads(emmc_power_pad,
					 ARRAY_SIZE(emmc_power_pad));
	gpio_direction_output(EN_EMMC_PWR, 1);
	imx_iomux_v3_setup_multiple_pads(usdhc2_pads,
					 ARRAY_SIZE(usdhc2_pads));
	usdhc2_cfg.sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

	return fsl_esdhc_initialize(bis, &usdhc2_cfg);
}

