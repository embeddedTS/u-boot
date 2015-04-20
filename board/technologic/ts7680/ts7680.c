/*
 * Technologic TS-7680 Single-board Computer
 *
 * (C) Copyright 2015 Technologic Systems
 * Based on work by:
 * Stuart Longland of VRT Systems <stuartl@vrt.com.au>
 *
 * Based on mx28evk.c:
 * Freescale MX28EVK board
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux-mx28.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <linux/mii.h>
#include <miiphy.h>
#include <netdev.h>
#include <errno.h>
#include <spi.h>

DECLARE_GLOBAL_DATA_PTR;
int random_mac = 0;

void mx28_adjust_mac(int dev_id, unsigned char *mac)
{
	mac[0] = 0x00;
	mac[1] = 0xD0;
	mac[2] = 0x69;
}

/*
 * Functions
 */
int board_early_init_f(void)
{
	/* IO0 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK0, 480000);
	/* IO1 clock at 480MHz */
	mxs_set_ioclk(MXC_IOCLK1, 480000);

	/* SSP0 clock at 96MHz */
	mxs_set_sspclk(MXC_SSPCLK0, 96000, 0);
	/* SSP2 clock at 160MHz */
	mxs_set_sspclk(MXC_SSPCLK2, 160000, 0);

#ifdef	CONFIG_CMD_USB
	mxs_iomux_setup_pad(MX28_PAD_SSP2_SS1__USB1_OVERCURRENT);
	mxs_iomux_setup_pad(MX28_PAD_AUART2_RX__GPIO_3_8 |
			MXS_PAD_4MA | MXS_PAD_3V3 | MXS_PAD_NOPULL);
	gpio_direction_output(MX28_PAD_AUART2_RX__GPIO_3_8, 1);
#endif

	mxs_iomux_setup_pad(MX28_PAD_SSP0_DETECT__GPIO_2_9);

	return 0;
}

int dram_init(void)
{
	return mxs_dram_init();
}

int misc_init_r(void)
{
	struct mxs_spl_data *data = (struct mxs_spl_data *)
	  ((CONFIG_SYS_TEXT_BASE - sizeof(struct mxs_spl_data)) & ~0xf);

	setenv_hex("bootmode", mxs_boot_modes[data->boot_mode_idx].boot_pads);
}


int board_init(void)
{
	/* Adress of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	return 0;
}

#ifdef	CONFIG_CMD_MMC
static int ts7400_mmc_cd(int id) {
	return 1;
}
int board_mmc_init(bd_t *bis)
{
	/* Configure MMC0 Power Enable */
	int ret = mxsmmc_initialize(bis, 0, NULL, ts7400_mmc_cd);
	//if (ret)
	return ret;
	//return mxsmmc_initialize(bis, 1, NULL, ts7400_mmc_cd);
}
#endif

#ifdef	CONFIG_CMD_NET

int board_eth_init(bd_t *bis)
{
	struct mxs_clkctrl_regs *clkctrl_regs =
		(struct mxs_clkctrl_regs *)MXS_CLKCTRL_BASE;
	struct eth_device *dev;
	int ret;
	uchar enetaddr[6];

	ret = cpu_eth_init(bis);
	if (ret)
		return ret;

	/* MX28EVK uses ENET_CLK PAD to drive FEC clock */
	writel(CLKCTRL_ENET_TIME_SEL_RMII_CLK | CLKCTRL_ENET_CLK_OUT_EN,
	       &clkctrl_regs->hw_clkctrl_enet);

	/* Reset FEC PHYs */
	gpio_direction_output(MX28_PAD_SSP0_DETECT__GPIO_2_9, 0);
	udelay(15000);
	gpio_set_value(MX28_PAD_SSP0_DETECT__GPIO_2_9, 1);

	ret = fecmxc_initialize_multi(bis, 0, 0, MXS_ENET0_BASE);
	if (ret) {
		puts("FEC MXS: Unable to init FEC0\n");
		return ret;
	}

	dev = eth_get_dev_by_name("FEC0");
	if (!dev) {
		puts("FEC MXS: Unable to get FEC0 device entry\n");
		return -EINVAL;
	}

	eth_parse_enetaddr(getenv("ethaddr"), enetaddr);
        if (!enetaddr[3] && !enetaddr[4] && !enetaddr[5]) {
                printf("No MAC address set in fuses.  Using random mac address.\n");
                eth_random_addr(enetaddr);
                random_mac = 1;
                if (eth_setenv_enetaddr("ethaddr", enetaddr)) {
                        printf("Failed to set ethernet address\n");
                }
        }

	return ret;
}

#endif

static int do_ice40_load(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
        unsigned int len, data;
        struct spi_slave *slave;
        int ret_val, i;
        char zeros[100];

        memset(zeros, 0, 100);

        // Parse image from mkimage
        data = simple_strtoul(argv[1], NULL, 16);
        len = simple_strtoul(argv[2], NULL, 16);

        slave = spi_setup_slave(2, 0, 25000000, SPI_MODE_0);
        if(spi_claim_bus(slave)){
                printf("Failed to claim the SPI bus\n");
                return 1;
        }

	gpio_direction_output(MX28_PAD_LCD_D23__GPIO_1_23, 0); //fabric reset
        // We need to pulse reset while the clock is high, so set it to a gpio first
        mxs_iomux_setup_pad(MX28_PAD_SSP2_SCK__GPIO_2_16 | MXS_PAD_3V3 | MXS_PAD_8MA);
	gpio_direction_input(MX28_PAD_LCD_D16__GPIO_1_16); //fpga_done
	gpio_direction_output(MX28_PAD_LCD_D17__GPIO_1_17, 0); //reset low
	gpio_direction_output(MX28_PAD_SSP2_SCK__GPIO_2_16, 1); //spi_clk high
	gpio_direction_output(MX28_PAD_LCD_D18__GPIO_1_18, 0); // spi_cs# low
	udelay(1);
	gpio_set_value(MX28_PAD_LCD_D17__GPIO_1_17, 1); //reset high
	udelay(801);
	mxs_iomux_setup_pad(MX28_PAD_SSP2_SCK__SSP2_SCK | MXS_PAD_3V3 | MXS_PAD_4MA | MXS_PAD_PULLUP);

	// Send 8 clocks
	ret_val = spi_xfer(slave, 8, zeros, NULL, 0);

	// Finally send data
	ret_val = spi_xfer(slave, len * 8, (void *)data, NULL, 0);

        // FPGA requires additional spi clocks after bitstream
        ret_val = spi_xfer(slave, 100 * 8, zeros, NULL, 0);

        for(i = 0; i <= 3000; i++)
        {
                if(gpio_get_value(MX28_PAD_LCD_D16__GPIO_1_16)) //fpga_done
                        break;
                if(i == 3000){
                        printf("FPGA_DONE never asserted\n");
                        ret_val = 1;
                }
                udelay(1000);
        }

	/* Issue reset to FPGA fabric */
	gpio_direction_output(MX28_PAD_LCD_D23__GPIO_1_23, 1);
	udelay(100); //25 clocks, plenty of time
	gpio_direction_output(MX28_PAD_LCD_D23__GPIO_1_23, 0);

        spi_release_bus(slave);

	/* Reset FEC PHYs */
	gpio_direction_output(MX28_PAD_SSP0_DETECT__GPIO_2_9, 0);
	udelay(15000);
	gpio_set_value(MX28_PAD_SSP0_DETECT__GPIO_2_9, 1);

        return ret_val;
}

U_BOOT_CMD(ice40, 3, 0, do_ice40_load,
        "ICE40 programming support",
        "[image address] [filesize]\n"
        "    Image must be in mkimage legacy format\n"
);
