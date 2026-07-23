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
#include <spi_flash.h>
#include <stdlib.h>

#define ICE40_BLK_SZ	0x1000

int do_ice40_load(void)
{
	ulong i;
	ulong len = CFG_ICE40_LEN;
	u32 data = CFG_ICE40_START;
	struct spi_slave *fpga_dev;
	struct spi_flash *nor_dev;
	int ret_val;
	u8 buf[ICE40_BLK_SZ];

	gpio_request(CFG_ICE40_FPGA_DONE, "ice40");
	gpio_request(CFG_ICE40_FPGA_RESET, "ice40");
	gpio_request(CFG_ICE40_CS, "ice40");

	fpga_dev = spi_setup_slave(1, CFG_ICE40_BUS, 25000000, SPI_MODE_3);
	ret_val = spi_claim_bus(fpga_dev);
	if (ret_val)
		goto out;

	nor_dev = spi_flash_probe(CONFIG_SF_DEFAULT_BUS, CONFIG_SF_DEFAULT_CS,
				  CONFIG_SF_DEFAULT_SPEED, CONFIG_SF_DEFAULT_MODE);
	if (!nor_dev) {
		ret_val = -ENODEV;
		printf("Failed to probe bus\n");
		goto spi_out;
	}

	gpio_direction_input(CFG_ICE40_FPGA_DONE); // fpga_done
	gpio_direction_output(CFG_ICE40_FPGA_RESET, 0); // reset low
	gpio_direction_output(CFG_ICE40_CS, 0); // spi cs# low
	udelay(1); // at least 200ns
	gpio_set_value(CFG_ICE40_FPGA_RESET, 1); // reset high
	mdelay(2);

	i = len;
	do {
		/* Calculate remaining length to read/write */
		if (i > ICE40_BLK_SZ)
			len = ICE40_BLK_SZ;
		else
			len = i;
		i -= len;

		/* Get data from SPI flash */
		ret_val = spi_flash_read(nor_dev, data, ICE40_BLK_SZ, buf);
		if (ret_val < 0) {
			printf("SPI flash read failed\n");
			goto spi_out;
		} 

		ret_val = spi_xfer(fpga_dev, len * 8, buf, NULL, 0);
		if (ret_val) {
			printf("SPI xfer to FPGA failed @ byte %x\n", data);
			goto spi_out;
		}
		data += len;

	} while (i > 0 && ret_val == 0);

	// FPGA requires additional spi clocks after bitstream
	memset(buf, 0, 100);
	ret_val = spi_xfer(fpga_dev, 100 * 8, buf, NULL, 0);
	if (ret_val) {
		printf("SPI xfer to FPGA failed @ clocks\n");
		goto spi_out;
	}

	gpio_set_value(CFG_ICE40_CS, 1); // spi cs# high

	for(i = 0; i <= 3000; i++)
	{
		if(gpio_get_value(CFG_ICE40_FPGA_DONE)){
			printf("iCE40 FPGA reloaded successfully\n");
			break;
		}
		if(i == 3000){ 
			printf("FPGA_DONE never asserted\n");
			ret_val = 1;
		}
		udelay(1000);
	}

spi_out:
	spi_release_bus(fpga_dev);

out:
	return ret_val;
}
