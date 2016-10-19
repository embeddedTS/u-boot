/*
 * Copyright (C) 2016 Technologic Systems
 *
 * Author: Mark Featherston <mark@embeddedarm.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>

#include <asm/arch/mx6-pins.h>
#include <asm/gpio.h>
#include <asm/imx-common/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <cli.h>
#include <command.h>
#include <i2c.h>
#include <status_led.h>

#include <miiphy.h>
#include <micrel.h>

#include "post.h"

#define LOOP_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

iomux_v3_cfg_t const posttest_pads[] = {
	MX6_PAD_EIM_A17__GPIO2_IO21 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_09
	MX6_PAD_DISP0_DAT8__GPIO4_IO29 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_19
	MX6_PAD_DISP0_DAT9__GPIO4_IO30 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_21
	MX6_PAD_DISP0_DAT10__GPIO4_IO31 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_23
	MX6_PAD_DISP0_DAT11__GPIO5_IO05 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_25
	MX6_PAD_DISP0_DAT12__GPIO5_IO06 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_27
	MX6_PAD_DISP0_DAT13__GPIO5_IO07 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_31
	MX6_PAD_DISP0_DAT14__GPIO5_IO08 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_33
	MX6_PAD_DISP0_DAT15__GPIO5_IO09 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_35
	MX6_PAD_DISP0_DAT16__GPIO5_IO10 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_37
	MX6_PAD_DISP0_DAT17__GPIO5_IO11 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_39
	MX6_PAD_DISP0_DAT18__GPIO5_IO12 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_41
	MX6_PAD_DISP0_DAT19__GPIO5_IO13 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_43
	MX6_PAD_DISP0_DAT20__GPIO5_IO14 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_45
	MX6_PAD_DI0_DISP_CLK__GPIO4_IO16 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_49
	MX6_PAD_DI0_PIN2__GPIO4_IO18 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_51
	MX6_PAD_DI0_PIN3__GPIO4_IO19 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_53
	MX6_PAD_DI0_PIN15__GPIO4_IO17 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_55
	MX6_PAD_SD4_DAT1__GPIO2_IO09 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_57
	MX6_PAD_EIM_D31__GPIO3_IO31 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_65
	MX6_PAD_KEY_ROW4__GPIO4_IO15 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_69
	MX6_PAD_EIM_WAIT__GPIO5_IO00 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_97
	MX6_PAD_KEY_COL4__GPIO4_IO14 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_71
	MX6_PAD_EIM_A21__GPIO2_IO17 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_77
	MX6_PAD_EIM_BCLK__GPIO6_IO31 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_79
	MX6_PAD_EIM_A18__GPIO2_IO20 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_81
	MX6_PAD_EIM_A20__GPIO2_IO18 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_85
	MX6_PAD_EIM_A23__GPIO6_IO06 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_91
	MX6_PAD_EIM_A24__GPIO5_IO04 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_93
	MX6_PAD_EIM_A16__GPIO2_IO22 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_04
	MX6_PAD_EIM_EB0__GPIO2_IO28 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_22
	MX6_PAD_DISP0_DAT0__GPIO4_IO21 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_24
	MX6_PAD_DISP0_DAT1__GPIO4_IO22 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_26
	MX6_PAD_DISP0_DAT2__GPIO4_IO23 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_28
	MX6_PAD_DISP0_DAT3__GPIO4_IO24 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_30
	MX6_PAD_DISP0_DAT4__GPIO4_IO25 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_32
	MX6_PAD_DISP0_DAT5__GPIO4_IO26 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_34
	MX6_PAD_DISP0_DAT6__GPIO4_IO27 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_38
	MX6_PAD_DISP0_DAT7__GPIO4_IO28 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_40
	MX6_PAD_DISP0_DAT21__GPIO5_IO15 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_42
	MX6_PAD_DISP0_DAT22__GPIO5_IO16 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_44
	MX6_PAD_DISP0_DAT23__GPIO5_IO17 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_46
	MX6_PAD_EIM_A19__GPIO2_IO19 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_48
	MX6_PAD_EIM_DA15__GPIO3_IO15 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_64
	MX6_PAD_EIM_DA14__GPIO3_IO14 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_66
	MX6_PAD_EIM_DA13__GPIO3_IO13 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_68
	MX6_PAD_EIM_DA12__GPIO3_IO12 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_70
	MX6_PAD_EIM_DA11__GPIO3_IO11 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_72
	MX6_PAD_EIM_DA4__GPIO3_IO04 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_86
	MX6_PAD_EIM_EB1__GPIO2_IO29 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_99
	MX6_PAD_EIM_DA10__GPIO3_IO10 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_74
	MX6_PAD_EIM_DA5__GPIO3_IO05 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_84
	MX6_PAD_EIM_DA9__GPIO3_IO09 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_76
	MX6_PAD_EIM_DA6__GPIO3_IO06 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_82
	MX6_PAD_EIM_DA8__GPIO3_IO08 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_78
	MX6_PAD_EIM_DA7__GPIO3_IO07 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_80
	MX6_PAD_EIM_DA3__GPIO3_IO03 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_88
	MX6_PAD_EIM_DA2__GPIO3_IO02 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_90
	MX6_PAD_EIM_DA1__GPIO3_IO01 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_92
	MX6_PAD_EIM_CS0__GPIO2_IO23 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_100
	MX6_PAD_EIM_DA0__GPIO3_IO00 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_94
	MX6_PAD_EIM_LBA__GPIO2_IO27 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN1_96
	MX6_PAD_CSI0_DAT4__GPIO5_IO22 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_36
	MX6_PAD_CSI0_DAT6__GPIO5_IO24 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_38
	MX6_PAD_CSI0_DAT5__GPIO5_IO23 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_40
	MX6_PAD_CSI0_DAT7__GPIO5_IO25 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_42
	MX6_PAD_CSI0_DAT12__GPIO5_IO30 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_52
	MX6_PAD_CSI0_DAT17__GPIO6_IO03 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_56
	MX6_PAD_SD4_DAT7__GPIO2_IO15 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_58
	MX6_PAD_SD3_RST__GPIO7_IO08 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_60
	MX6_PAD_GPIO_16__GPIO7_IO11 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_62
	MX6_PAD_GPIO_17__GPIO7_IO12 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_64
	MX6_PAD_CSI0_DAT13__GPIO5_IO31 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_66
	MX6_PAD_GPIO_19__GPIO4_IO05 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_68
	MX6_PAD_CSI0_MCLK__GPIO5_IO19 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_70
	MX6_PAD_CSI0_PIXCLK__GPIO5_IO18 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_72
	MX6_PAD_KEY_COL2__GPIO4_IO10 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_97
	MX6_PAD_KEY_ROW2__GPIO4_IO11 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_99
	MX6_PAD_GPIO_7__GPIO1_IO07 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_78
	MX6_PAD_GPIO_8__GPIO1_IO08 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_80
	MX6_PAD_EIM_D24__GPIO3_IO24 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_82
	MX6_PAD_EIM_D25__GPIO3_IO25 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // CN2_84
	/* Wifi Signals */
	MX6_PAD_SD1_DAT0__GPIO1_IO16 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // SD1_D0
	MX6_PAD_SD1_DAT1__GPIO1_IO17 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // SD1_D1
	MX6_PAD_SD1_DAT2__GPIO1_IO19 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // SD1_D2
	MX6_PAD_SD1_DAT3__GPIO1_IO21 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // SD1_D3
	MX6_PAD_SD1_CMD__GPIO1_IO18 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // SD1_CMD
	MX6_PAD_SD1_CLK__GPIO1_IO20 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // SD1_CLK
	MX6_PAD_SD4_DAT6__GPIO2_IO14 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // UART2_CTS
	MX6_PAD_SD4_DAT5__GPIO2_IO13 | MUX_PAD_CTRL(LOOP_PAD_CTRL), // UART2_RTS
};

int test_gpio_once(int gpio1, int gpio2, const char *gpio1_name, const char *gpio2_name)
{
	int ret = 0;

	ret |= gpio_direction_output(gpio1, 0);
	ret |= gpio_direction_input(gpio2);

	if(gpio_get_value(gpio2) != 0) {
		printf("\t%s failed to go low, or %s cannot read its value\n", gpio1_name, gpio2_name);
		ret |= 1;
	}

	ret |= gpio_direction_output(gpio1, 1);
	ret |= gpio_direction_input(gpio2);
	if(gpio_get_value(gpio2) != 1) {
		printf("\t%s failed to go high, or %s cannot read its value\n", gpio1_name, gpio2_name);
		ret |= 1;
	}

	gpio_direction_input(gpio1);
	gpio_direction_input(gpio2);

	return ret;
}

int test_gpio_once_fpga(int gpio1, int gpio2, const char *gpio1_name, const char *gpio2_name)
{
	int ret = 0;
	uint8_t val;

	// Set crossbar as a GPIO and set OE
	val = 0xf9;
	ret |= i2c_write(0x28, gpio1, 2, &val, 1);
	/* Set as an input */
	val = 0xf8;
	ret |= i2c_write(0x28, gpio2, 2, &val, 1);
	ret |= i2c_read(0x28, gpio2, 2, &val, 1);

	if((val & 0x4) != 0x0) {
		printf("\t%s failed to go low, or %s cannot read its value\n", gpio1_name, gpio2_name);
		ret |= 1;
	}

	val = 0x3;
	i2c_write(0x28, gpio1, 2, &val, 1);
	ret |= i2c_read(0x28, gpio2, 2, &val, 1);
	if((val & 0x4) != 0x4) {
		printf("\t%s failed to go high, or %s cannot read its value\n", gpio1_name, gpio2_name);
		ret |= 1;
	}

	/* Set FPGA IO to input */
	val = 0x0;
	i2c_write(0x28, gpio1, 2, &val, 1);
	i2c_write(0x28, gpio2, 2, &val, 1);

	return ret;
}

int test_gpio_2(int gpio1, int gpio2, const char *gpio1_name, const char *gpio2_name)
{
	int ret = 0;

	ret |= test_gpio_once(gpio1, gpio2, gpio1_name, gpio2_name);
	ret |= test_gpio_once(gpio2, gpio1, gpio2_name, gpio1_name);

	return ret;
}

/* In out loopback some connections are FPGA/CPU pins */
int test_gpio_2_fpga(int fpga_gpio, int cpu_gpio, const char *fpga_name, const char *cpu_name)
{
	int ret = 0;
	uint8_t val;

	// Set crossbar as a GPIO and set OE
	val = 0xf9;
	ret |= i2c_write(0x28, fpga_gpio, 2, &val, 1);
	ret |= gpio_direction_input(cpu_gpio);

	if(gpio_get_value(cpu_gpio) != 0) {
		printf("\t%s failed to go low, or %s cannot read its value\n", fpga_name, cpu_name);
		ret |= 1;
	}

	val = 0x3;
	i2c_write(0x28, fpga_gpio, 2, &val, 1);
	ret |= gpio_direction_input(cpu_gpio); 
	if(gpio_get_value(cpu_gpio) != 1) {
		printf("\t%s failed to go high, or %s cannot read its value\n", fpga_name, cpu_name);
		ret |= 1;
	}

	/* Set FPGA IO to input */
	val = 0x0;
	i2c_write(0x28, fpga_gpio, 2, &val, 1);

	ret |= gpio_direction_output(cpu_gpio, 0);
	i2c_read(0x28, fpga_gpio, 2, &val, 1);
	if((val & 0x4) == 0x4) {
		printf("\t%s failed to go low, or %s cannot read its value\n", cpu_name, fpga_name);
		ret |= 1;
	}

	ret |= gpio_direction_output(cpu_gpio, 1);
	i2c_read(0x28, fpga_gpio, 2, &val, 1);
	if((val & 0x4) == 0x0) {
		printf("\t%s failed to go high, or %s cannot read its value\n", cpu_name, fpga_name);
		ret |= 1;
	}

	gpio_direction_input(cpu_gpio);

	return ret;
}

int test_gpio_3(int gpio1, 
	int gpio2,
	int gpio3,
	const char *gpio1_name,
	const char *gpio2_name,
	const char *gpio3_name)
{
	int ret = 0;
	gpio_direction_input(gpio1);
	gpio_direction_input(gpio2);
	gpio_direction_input(gpio3);

	ret |= test_gpio_2(gpio1, gpio2, gpio1_name, gpio2_name);
	ret |= test_gpio_2(gpio2, gpio3, gpio2_name, gpio3_name);
	ret |= test_gpio_2(gpio1, gpio3, gpio1_name, gpio3_name);

	return 0;
}

int ts9550_loopback_test(void)
{
	int ret = 0;

	printf("GPIO loopback tests starting...\n");

	ret |= test_gpio_2(IMX_GPIO_NR(2, 21), IMX_GPIO_NR(4, 29), "CN1_09", "CN1_19");
	ret |= test_gpio_2(IMX_GPIO_NR(4, 30), IMX_GPIO_NR(4, 31), "CN1_21", "CN1_23");
	ret |= test_gpio_2(IMX_GPIO_NR(5, 5), IMX_GPIO_NR(5, 6), "CN1_25", "CN1_27");
	ret |= test_gpio_2(IMX_GPIO_NR(5, 7), IMX_GPIO_NR(5, 8), "CN1_31", "CN1_33");
	ret |= test_gpio_2(IMX_GPIO_NR(5, 9), IMX_GPIO_NR(5, 10), "CN1_35", "CN1_37");
	ret |= test_gpio_2(IMX_GPIO_NR(5, 11), IMX_GPIO_NR(5, 12), "CN1_39", "CN1_41");
	ret |= test_gpio_2(IMX_GPIO_NR(5, 13), IMX_GPIO_NR(5, 14), "CN1_43", "CN1_45");
	ret |= test_gpio_2(IMX_GPIO_NR(4, 16), IMX_GPIO_NR(4, 18), "CN1_49", "CN1_51");
	ret |= test_gpio_3(IMX_GPIO_NR(4, 19), IMX_GPIO_NR(4, 17), IMX_GPIO_NR(2, 9), "CN1_53", "CN1_55", "CN1_57");
	ret |= test_gpio_2_fpga(0, IMX_GPIO_NR(3, 31), "CN1_63", "CN1_65");
	ret |= test_gpio_2_fpga(1, IMX_GPIO_NR(4, 15), "CN1_67", "CN1_69");
	ret |= test_gpio_2_fpga(1, IMX_GPIO_NR(5, 0), "CN1_67", "CN1_97");

	/* skipping CN1_73, is input only */

	//ret |= test_gpio_2_fpga(0, IMX_GPIO_NR(4, 14), "CN1_73", "CN1_71");
	ret |= test_gpio_2(IMX_GPIO_NR(2, 17), IMX_GPIO_NR(6, 31), "CN1_77", "CN1_79");
	ret |= test_gpio_2(IMX_GPIO_NR(2, 20), IMX_GPIO_NR(2, 18), "CN1_81", "CN1_85");
	ret |= test_gpio_2(IMX_GPIO_NR(6, 6), IMX_GPIO_NR(5, 4), "CN1_91", "CN1_93");
	/* Skipping CN1_22, cannot go low by design */
	ret |= test_gpio_2(IMX_GPIO_NR(2, 22), IMX_GPIO_NR(4, 21), "CN1_04", "CN1_24");
	ret |= test_gpio_3(IMX_GPIO_NR(4, 22), IMX_GPIO_NR(4, 23), IMX_GPIO_NR(4, 24), "CN1_26", "CN1_28", "CN1_30");
	ret |= test_gpio_2(IMX_GPIO_NR(4, 25), IMX_GPIO_NR(4, 26), "CN1_32", "CN1_34");
	ret |= test_gpio_2(IMX_GPIO_NR(4, 27), IMX_GPIO_NR(4, 28), "CN1_38", "CN1_40");
	ret |= test_gpio_2(IMX_GPIO_NR(5, 15), IMX_GPIO_NR(5, 16), "CN1_42", "CN1_44");
	ret |= test_gpio_2(IMX_GPIO_NR(5, 17), IMX_GPIO_NR(2, 19), "CN1_46", "CN1_48");
	ret |= test_gpio_2(IMX_GPIO_NR(3, 15), IMX_GPIO_NR(3, 14), "CN1_64", "CN1_66");
	ret |= test_gpio_2(IMX_GPIO_NR(3, 13), IMX_GPIO_NR(3, 12), "CN1_68", "CN1_70");
	ret |= test_gpio_3(IMX_GPIO_NR(3, 11), IMX_GPIO_NR(3, 4), IMX_GPIO_NR(2, 29), "CN1_72", "CN1_86", "CN1_99");
	ret |= test_gpio_2(IMX_GPIO_NR(3, 10), IMX_GPIO_NR(3, 5), "CN1_74", "CN1_84");
	ret |= test_gpio_2(IMX_GPIO_NR(3, 9), IMX_GPIO_NR(3, 6), "CN1_76", "CN1_82");
	ret |= test_gpio_2(IMX_GPIO_NR(3, 8), IMX_GPIO_NR(3, 7), "CN1_78", "CN1_80");
	ret |= test_gpio_2(IMX_GPIO_NR(3, 3), IMX_GPIO_NR(3, 2), "CN1_88", "CN1_90");
	ret |= test_gpio_2(IMX_GPIO_NR(3, 1), IMX_GPIO_NR(2, 23), "CN1_92", "CN1_100");
	ret |= test_gpio_2(IMX_GPIO_NR(3, 0), IMX_GPIO_NR(2, 27), "CN1_94", "CN1_96");
	ret |= test_gpio_2(IMX_GPIO_NR(5, 22), IMX_GPIO_NR(5, 24), "CN2_36", "CN2_38");
	ret |= test_gpio_2(IMX_GPIO_NR(5, 23), IMX_GPIO_NR(5, 25), "CN2_40", "CN2_42");
	ret |= test_gpio_3(IMX_GPIO_NR(5, 30), IMX_GPIO_NR(1, 0), IMX_GPIO_NR(6, 3), "CN2_52", "CN2_54", "CN2_56");
	ret |= test_gpio_2(IMX_GPIO_NR(2, 15), IMX_GPIO_NR(7, 8), "CN2_58", "CN2_60");
	ret |= test_gpio_2(IMX_GPIO_NR(7, 11), IMX_GPIO_NR(7, 12), "CN2_62", "CN2_64");
	ret |= test_gpio_2(IMX_GPIO_NR(5, 31), IMX_GPIO_NR(4, 5), "CN2_66", "CN2_68");
	ret |= test_gpio_2(IMX_GPIO_NR(5, 19), IMX_GPIO_NR(5, 18), "CN2_70", "CN2_72");
	ret |= test_gpio_2(IMX_GPIO_NR(4, 10), IMX_GPIO_NR(4, 11), "CN2_97", "CN2_99");

	ret |= test_gpio_once_fpga(5, 6, "CN2_78", "CN2_80");
	ret |= test_gpio_2(IMX_GPIO_NR(3, 24), IMX_GPIO_NR(3, 25), "CN2_82", "CN2_84");
	ret |= test_gpio_once_fpga(7, 8, "CN2_86", "CN2_88");
	ret |= test_gpio_2(IMX_GPIO_NR(3, 24), IMX_GPIO_NR(3, 25), "CN2_90", "CN2_92");
	ret |= test_gpio_once_fpga(9, 10, "CN2_94", "CN2_96");
	ret |= test_gpio_once_fpga(11, 12, "CN2_98", "CN2_100");

	if (ret == 0) printf("GPIO test passed\n");
	else printf("GPIO test failed\n");
	return ret;
}

int fpga_test(void)
{
	int ret = 0;
	uint8_t val;
	printf("FPGA test starting...\n");

	/* FPGA should be loaded before running this test.  
	 * Tested with: /boot/ts4900-fpga.bin bf93c03ef914cf008287c8cd60781cc8 
	 * Expects addr d51 to include rev 4 */
	ret |= i2c_read(0x28, 51, 2, &val, 1);
	if(((val & 0xf0) >> 4) != 0x4) {
		printf("Wrong rev detected, expected 4 and got 0x%X\n", val);
		ret = 1;
	}

	if (ret == 0) printf("FPGA test passed\n");
	else printf("FPGA test failed\n");
	return 0;
}

int micrel_phy_test(void)
{
	int ret = 0;
	struct phy_device *phydev;
	int phyid1;
	int phyid2;

	printf("Ethernet test starting...\n");

	phydev = mdio_phydev_for_ethname("FEC");
	phyid1 = phy_read(phydev, MDIO_DEVAD_NONE, MII_PHYSID1);
	phyid2 = phy_read(phydev, MDIO_DEVAD_NONE, MII_PHYSID2);

	if(phyid1 != 0x22 || phyid2 != 0x1622)
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
	int ret = 0, i;
	uint8_t val;
	printf("WIFI test starting...\n");

	/* Wifi is very complex and implementing a real test in u-boot is probably 
	 * not wise but the bluetooth on the same chip shows sign of life by going low
	 * after the chip is enabled. */
	gpio_direction_input(IMX_GPIO_NR(2, 13)); // RTS
	if(gpio_get_value(IMX_GPIO_NR(2, 13)) != 1)
		ret = 1;

	val = 3;
	ret |= i2c_write(0x28, 13, 2, &val, 1);
	mdelay(500);
	if(gpio_get_value(IMX_GPIO_NR(2, 13)) != 0)
		ret = 1;

	if (ret == 0) printf("WIFI test passed\n");
	else printf("WIFI test failed\n");
	return ret;
}

int mem_test(void)
{
	int ret = 0;
	int argc = 5;
	/* Arguments are the memory addresses, pattern (not relevant in alt test) and
	 * 20 which is the number of iterations (in hex) which takes about 1 second
	 * on the i.MX6 quad */
	char *argv[5] = { "mtest", "0x10000000", "0x10010000", "1", "20" };

	printf("RAM test starting...\n");

	ret |= do_mem_mtest(0, 0, argc, argv);

	if (ret == 0) printf("RAM test passed\n");
	else printf("RAM test failed\n");
	return ret;
}

/* On the TS-9550 the test results blink LEDs
 * and run loopback tests not present on other boards */
int is_9550(void)
{
	if(strcmp("15", getenv("baseboardid")) == 0){
		return 1;
	} else {
		return 0;
	}
}

void post_pass(void)
{
	printf("POST test passed\n");
	red_led_off();
	while(is_9550()) {
		mdelay(500);
		green_led_off();
		mdelay(500);
		green_led_on();
	}
}

void post_fail(void)
{
	printf("POST test failed\n");
	green_led_off();
	while(is_9550()) {
		mdelay(500);
		red_led_off();
		mdelay(500);
		red_led_on();
	}
}

static int do_post_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = 0;
	int emmc_present = 0;
	int wifi_present = 0;

	imx_iomux_v3_setup_multiple_pads(posttest_pads, ARRAY_SIZE(posttest_pads));

	uint8_t val;
	// Make sure the "ice40" command has been run with a valid
	// FPGA before running this test
	ret |= fpga_test();
	ret |= i2c_read(0x28, 51, 2, &val, 1);

	emmc_present = !(val & 0x8);
	wifi_present = !(val & 0x4);

	ret |= micrel_phy_test();

	if(emmc_present){
		ret |= emmc_test();
		ret |= wifi_test();
	}

	// If environment variable for TS-9550 baseboard, 
	// Run these tests:
	if(is_9550()) {
		// Make sure REV is at least C for loopback tests
		ret |= ts9550_loopback_test();
	}

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