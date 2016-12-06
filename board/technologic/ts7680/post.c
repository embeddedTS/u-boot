/*
 * Copyright (C) 2016 Technologic Systems
 *
 * Author: Kris Bahnsen <kris@embeddedarm.com>
 *   Based on work by Mark Featherston <mark@embeddedarm.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>

#include <asm/gpio.h>
#include <asm/arch/iomux-mx28.h>
#include <cli.h>
#include <command.h>
#include <i2c.h>
#include <status_led.h>

#include <miiphy.h>

#include "post.h"

int do_mmcops(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
int do_mem_mtest(cmd_tbl_t *cmdtp, int flag, int argc,
			char * const argv[]);

#define POST_MUX_CONFIG (MXS_PAD_3V3 | MXS_PAD_8MA | MXS_PAD_PULLUP)

iomux_cfg_t const posttest_pads[] = {
	MX28_PAD_AUART0_CTS__GPIO_3_2 | POST_MUX_CONFIG
};

int ts7680_loopback_test(void)
{
	int ret = 0;
	uint8_t val;

	printf("Testing LS inputs\n");

	ret |= i2c_read(0x28, 0xE, 2, &val, 1);
	if((val & 0xF) != 0xF)
		ret = 1;

	val = 3;
	ret |= i2c_write(0x28, 0xF, 2, &val, 1);
	ret |= i2c_write(0x28, 0x10, 2, &val, 1);
	ret |= i2c_write(0x28, 0x11, 2, &val, 1);

	ret |= i2c_read(0x28, 0xE, 2, &val, 1);
	if((val & 0xF) != 0x8)
		ret = 1;

	if (ret == 0) printf("GPIO test passed\n");
	else printf("GPIO test failed\n");
	return ret;
}

int ts7682_loopback_test(void)
{
	int ret = 0;
	uint8_t val;

	printf("Testing LS inputs\n");

	ret |= i2c_read(0x28, 0xE, 2, &val, 1);
	if((val & 0x3F) != 0x3F)
		ret = 1;

	val = 3;
	ret |= i2c_write(0x28, 0x0, 2, &val, 1);
	ret |= i2c_write(0x28, 0x1, 2, &val, 1);
	ret |= i2c_write(0x28, 0x2, 2, &val, 1);
	ret |= i2c_write(0x28, 0x3, 2, &val, 1);
	ret |= i2c_write(0x28, 0x4, 2, &val, 1);
	ret |= i2c_write(0x28, 0x5, 2, &val, 1);

	ret |= i2c_read(0x28, 0xE, 2, &val, 1);
	if((val & 0x3F) != 0x0)
		ret = 1;

	if (ret == 0) printf("GPIO test passed\n");
	else printf("GPIO test failed\n");
	return ret;
}

int fpga_test(void)
{
	int ret = 0;
	uint8_t val, model;
	printf("FPGA test starting...\n");

	/* FPGA should be loaded before running this test. */
	ret |= i2c_read(0x28, 0x7D, 2, &model, 1);
	ret |= i2c_read(0x28, 0x7F, 2, &val, 1);
	if((val != 0xA) && (model == 0x82 && val != 0x0)) {
		printf("Wrong rev detected, got 0x%X\n", val);
		ret = 1;
	}

	if (ret == 0) printf("FPGA test passed\n");
	else printf("FPGA test failed\n");
	return 0;
}

int switch_test(void)
{
	int ret = 0;
	struct phy_device *phydev;
	int phyid;

	printf("Ethernet test starting...\n");

	phydev = mdio_phydev_for_ethname("FEC0");
	phyid = phy_read(phydev, MDIO_DEVAD_NONE, MII_PHYSID2);
	phyid &= 0xFFF0;

	if(phyid != 512)
		ret = 1;

	if (ret == 0) printf("Ethernet test passed\n");
	else printf("Ethernet test failed\n");
	return ret;
}

int emmc_test(void)
{
	int ret = 0, i;
	printf("eMMC test starting...\n");
	uint32_t *loadaddr = (uint32_t *)0x12000000;

	char *query_argv[3] = { "mmc", "dev", "1" };
	char *write_argv[5] = { "mmc", "write", "0x12000000", "0x0", "0x800" };
	char *read_argv[5] = { "mmc", "read", "0x12000000", "0x0", "0x800" };

	/* This tests simple enumeration */
	ret |= do_mmcops(0, 0, 3, query_argv);

	memset(loadaddr, 0xAAAAAAAA, 1024*1024*4);
	ret |= do_mmcops(0, 0, 5, write_argv);
	memset(loadaddr, 0x00000000, 1024*1024*4);
	ret |= do_mmcops(0, 0, 5, read_argv);

	for (i = 0; i < (1024*1024)/4; i++)
	{
		if (loadaddr[i] != 0xAAAAAAAA)
		{
			printf("\tWrong value at %d\n", i);
			printf("got 0x%X, expected 0xAAAAAAAA\n", loadaddr[i]);
			ret = 1;
		}
	}

	memset(loadaddr, 0x55555555, 1024*1024*4);
	ret |= do_mmcops(0, 0, 5, write_argv);
	memset(loadaddr, 0x00000000, 1024*1024*4);
	ret |= do_mmcops(0, 0, 5, read_argv);

	for (i = 0; i < (1024*1024)/4; i++)
	{
		if (loadaddr[i] != 0x55555555)
		{
			printf("\tWrong value at %d\n", i);
			printf("got 0x%X, expected 0x55555555\n", loadaddr[i]);
			ret = 1;
		}
	}

	if (ret == 0) printf("eMMC test passed\n");
	else printf("eMMC test failed\n");
	return ret;
}

int wifi_test(void)
{
	int ret = 0;
	uint8_t val;
	printf("WIFI test starting...\n");

	/* Wifi is very complex and implementing a real test in u-boot is 
	 * probably not wise but the bluetooth on the same chip shows sign of 
	 * life by going low after the chip is enabled. */
	gpio_direction_input(MX28_PAD_AUART0_CTS__GPIO_3_2);
	if(gpio_get_value(MX28_PAD_AUART0_CTS__GPIO_3_2) != 1)
		ret = 1;

	val = 3;
	ret |= i2c_write(0x28, 0x2D, 2, &val, 1);
	mdelay(500);

	if(gpio_get_value(MX28_PAD_AUART0_CTS__GPIO_3_2) != 0)
		ret = 1;

	if (ret == 0) printf("WIFI test passed\n");
	else printf("WIFI test failed\n");
	return ret;
}

int mem_test(void)
{
	int ret = 0;
	int argc = 5;
	/* Arguments are the memory addresses, pattern (not relevant in alt 
	 *  test) and 20 which is the number of iterations (in hex) which takes 
	 * about 1 second */
	char *argv[5] = { "mtest", "0x40000000", "0x40010000", "1", "20" };

	printf("RAM test starting...\n");

	ret |= do_mem_mtest(0, 0, argc, argv);

	if (ret == 0) printf("RAM test passed\n");
	else printf("RAM test failed\n");
	return ret;
}

void post_pass(void)
{
	printf("POST test passed\n");
	red_led_off();
}

void post_fail(void)
{
	printf("POST test failed\n");
	green_led_off();
}

static int do_post_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = 0;
	int emmc_present = 0;
	int wifi_present = 0;
	uint8_t val;

	mxs_iomux_setup_multiple_pads(posttest_pads, ARRAY_SIZE(posttest_pads));


	/* Make sure the FPGA is sane before running these tests */
	ret |= fpga_test();
	ret |= i2c_read(0x28, 0x7E, 2, &val, 1);

	wifi_present = (val & (1 << 2));

	ret |= switch_test();

	ret |= emmc_test();

	if(wifi_present){
		ret |= wifi_test();
	}

	ret |= i2c_read(0x28, 0x7D, 2, &val, 1);
	if(val == 0x82) ret |= ts7682_loopback_test();
	else ret |= ts7680_loopback_test();

	ret |= mem_test();

	if(ret) {
		post_fail();
		return 1;
	} else {
		post_pass();
		return 0;
	}
}

U_BOOT_CMD(post, 1, 1,	do_post_test,
	"Runs a POST test",
	""
);
