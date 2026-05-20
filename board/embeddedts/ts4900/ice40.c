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
	//static u8 fpga_buf[FPGA_CHUNK_SIZE] __aligned(ARCH_DMA_MINALIGN);
	u8 buf[ICE40_BLK_SZ];
	//u8 *buf = NULL;

	printf("KRIS: malloc\n");
	//buf = malloc(ICE40_BLK_SZ);
	printf("KRIS: malloced %p\n", buf);

	gpio_request(CFG_ICE40_FPGA_DONE, "ice40");
	gpio_request(CFG_ICE40_FPGA_RESET, "ice40");
	gpio_request(CFG_ICE40_CS, "ice40");

	printf("Setting up bus\n");
	fpga_dev = spi_setup_slave(1, CFG_ICE40_BUS, 25000000, SPI_MODE_3);
	if (spi_claim_bus(fpga_dev)) {
		printf("Failed to claim the SPI bus\n");
		return 1;
	}

	printf("KRIS: setting up flash\n");
	nor_dev = spi_flash_probe(CONFIG_SF_DEFAULT_BUS, CONFIG_SF_DEFAULT_CS,
				  CONFIG_SF_DEFAULT_SPEED, CONFIG_SF_DEFAULT_MODE);
	if (!nor_dev) {
		printf("Failed to probe bus\n");
		return -ENODEV;
	}

	printf("Bus set up\n");

	gpio_direction_input(CFG_ICE40_FPGA_DONE); // fpga_done
	gpio_direction_output(CFG_ICE40_FPGA_RESET, 0); // reset low
	gpio_direction_output(CFG_ICE40_CS, 0); // spi cs# low
	udelay(1); // at least 200ns
	gpio_set_value(CFG_ICE40_FPGA_RESET, 1); // reset high
	mdelay(2);

	printf("XFER\n");
	i = len;
	do {
		/* Calculate remaining length to read/write */
		if (i > ICE40_BLK_SZ)
			len = ICE40_BLK_SZ;
		else
			len = i;
		i -= len;

		/* Get data from SPI flash */
		if (spi_flash_read(nor_dev, data, ICE40_BLK_SZ, buf)) {
			printf("SPI read failed\n");
			break;
		} 

		ret_val = spi_xfer(fpga_dev, len * 8, buf, NULL, 0);
		data += len;

		printf("loop, rem %ld\n", i);
	} while (i > 0 && ret_val == 0);
	printf("XFER COMPLETE\n");

	// FPGA requires additional spi clocks after bitstream
	memset(buf, 0, 100);
	ret_val = spi_xfer(fpga_dev, 100 * 8, buf, NULL, 0);

	gpio_set_value(CFG_ICE40_CS, 1); // spi cs# high

	for(i = 0; i <= 3000; i++)
	{
		if(gpio_get_value(CFG_ICE40_FPGA_DONE)){
			printf("ICE40 FPGA reloaded successfully\n");
			break;
		}
		if(i == 3000){ 
			printf("FPGA_DONE never asserted\n");
			ret_val = 1;
		}
		udelay(1000);
	}

	spi_release_bus(fpga_dev);

	return ret_val;
}
