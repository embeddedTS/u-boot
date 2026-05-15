/*
 * Copyright (C) 2016-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * Author: Mark Featherston <mark@embeddedTS.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/gpio.h>
#include <command.h>
#include <linux/delay.h>
#include <spi.h>

static int do_ice40_load(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	ulong len, i;
	u8 *data;
	struct spi_slave *slave;
	int ret_val;

	gpio_request(CONFIG_ICE40_FPGA_DONE, "ice40");
	gpio_request(CONFIG_ICE40_FPGA_RESET, "ice40");
	gpio_request(CONFIG_ICE40_CS, "ice40");

	// Parse image from mkimage
	data = (u8 *)simple_strtoul(argv[1], NULL, 16);
	len = simple_strtoul(argv[2], NULL, 16);

	printf("Setting up bus\n");
	slave = spi_setup_slave(1, CONFIG_ICE40_BUS, 25000000, SPI_MODE_3);
	if(spi_claim_bus(slave)){
		printf("Failed to claim the SPI bus\n");
		return 1;
	}

	printf("Bus set up\n");

	gpio_direction_input(CONFIG_ICE40_FPGA_DONE); // fpga_done
	gpio_direction_output(CONFIG_ICE40_FPGA_RESET, 0); // reset low
	gpio_direction_output(CONFIG_ICE40_CS, 0); // spi cs# low
	udelay(1); // at least 200ns
	gpio_set_value(CONFIG_ICE40_FPGA_RESET, 1); // reset high
	mdelay(2);

	printf("XFER\n");
	i = len * 8;
	do {
		if (i > 0x8000)
			len = 0x8000;
		else
			len = i;
		i -= len;

		ret_val = spi_xfer(slave, len * 8, data, NULL, 0);
		data += len;

		printf("loop, rem %ld\n", i);
	} while (i > 0 && ret_val == 0);
	printf("XFER COMPLETE\n");

	// FPGA requires additional spi clocks after bitstream
	char zeroes[100];
	memset(zeroes, 0, 100);
	ret_val = spi_xfer(slave, 100 * 8, zeroes, NULL, 0);

	gpio_set_value(CONFIG_ICE40_CS, 1); // spi cs# high

	for(i = 0; i <= 3000; i++)
	{
		if(gpio_get_value(CONFIG_ICE40_FPGA_DONE)){
			printf("ICE40 FPGA reloaded successfully\n");
			break;
		}
		if(i == 3000){ 
			printf("FPGA_DONE never asserted\n");
			ret_val = 1;
		}
		udelay(1000);
	}

	spi_release_bus(slave);

	return ret_val;
}

U_BOOT_CMD(ice40, 3, 0, do_ice40_load,
	"ICE40 programming support",
	" [image address] [filesize]\n"
);
