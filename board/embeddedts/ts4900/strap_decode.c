#include <asm/mach-imx/sys_proto.h>
#include <asm/types.h>
#include <stddef.h>
#include <stdio.h>

#include "strap_decode.h"

/* As of 20260521 there are 4 main CPU+RAM configurations:
 * s-1g-800:  i.MX6S, 800 MHz, 1 GB of RAM as 2x256x16
 * s-2g-800:  i.MX6S, 800 MHz, 2 GB of RAM as 4x256x16
 * s-1g-1000: i.MX6S, 1 GHz, 1 GB of RAM as 2x256x16
 * q-2g-1000: i.MX6Q, 1 GHz, 2 GB of RAM as 4x256x16
 *
 * Decoding existing straps depends on the CPU as well as PCB revision to
 * arrive at an exact configuration.
 *
 * | CPU | PCB | 1_29 | 6_5 | 2_11 | 51:1 | 51:0 | RAM CONF  |
 * -----------------------------------------------------------
 * |  S  |  A  |  Z   |  Z  |  Z   |  0   |  0   | s-1g-800  |
 * |  S  |  C  |  Z   |  Z  |  0   |  0   |  Z   | s-1g-800  |
 * |  S  |  D  |  Z   |  0  |  0   |  0   |  Z   | s-1g-800  |
 * |  S  |  E  |  0   |  0  |  X   |  0   |  Z   | s-1g-800  |
 *
 * |  S  | D04 |  Z   |  0  |  0   |  0   |  0   | s-2g-800  |
 * |  S  |  E  |  0   |  0  |  X   |  0   |  0   | s-2g-800  |
 *
 * |  S  |  A  |  Z   |  Z  |  Z   |  Z   |  0   | s-1g-1000 |
 * |  S  |  C  |  Z   |  Z  |  0   |  Z   |  0   | s-1g-1000 |
 * |  S  |  D  |  Z   |  0  |  0   |  Z   |  Z   | s-1g-1000 |
 * |  S  |  E  |  0   |  0  |  X   |  Z   |  Z   | s-1g-1000 |
 *
 * |  Q  |  A  |  Z   |  Z  |  Z   |  0   |  0   | q-2g-1000 |
 * |  Q  |  C  |  Z   |  Z  |  0   |  0   |  0   | q-2g-1000 |
 * |  Q  |  D  |  Z   |  0  |  0   |  Z   |  Z   | q-2g-1000 |
 * |  Q  |  E  |  0   |  0  |  X   |  Z   |  Z   | q-2g-1000 |
 *
 * Where:
 * X_Y is GPIO bank (X) + pin (Y) notation
 * X:Y is FPGA register (X) and bit (Y) notation
 * Z is pin is floating
 * X is don't care
 * 0 is pin is grounded
 * 1 is pin is tied high
 *
 * Rev E has a strap that is don't care because there are different BOM Revs
 * that have this pin strapped differently. The good news is that 6_5 and 1_29
 * are copper straps that are always correct in all BOM revs and can be used
 * without 2_11 to definitively ID a rev E board.
 */

/* Normalized PCB rev mapping */
#define REV_E		0x00
#define REV_D		0x04
#define REV_C		0x06
#define REV_A		0x07

/* Normalized FPGA RAM straps */
/* NOTE! Bit 7 is set if CPU is quad core! */
#define S_8_1G		0x01
#define S_8_2G		0x00
#define S_10_1G		0x03
#define Q_10_2G		0x83
#define MEM_ERR		0xFF

/* Early PCB Rev FPGA RAM straps */
/* NOTE! Bit 7 is set if CPU is quad core! */
#define REV_A_S_8_1G	0x00
#define REV_C_S_8_1G	0x01
#define REV_AC_S_10_1G	0x02
#define REV_AC_Q_10_2G	0x80

static inline u8 normalize_pcb_rev(u8 cpu_strap)
{
	/* Normalize the CPU straps to assume if not Rev A, that the Rev A strap
	 * is set to 0. This accounts for the potential of Rev E to have this
	 * strap floating on some early BOMs.
	 */
	if (!((cpu_strap & 0x07) == 0x07))
		cpu_strap &= ~(0x01);

	return cpu_strap;
}

char ts4900_pcb_rev_char(u8 cpu_strap)
{
	cpu_strap = normalize_pcb_rev(cpu_strap);

	switch (cpu_strap & 0x07) {
	case REV_E:
		return 'E';
	case REV_D:
		return 'D';
	case REV_C:
		return 'C';
	case REV_A:
		return 'A';
	default:
		return '?';
	}
}

static u8 normalize_fpga_strap(u8 cpu_strap, u8 fpga_strap)
{
	u8 rev = normalize_pcb_rev(cpu_strap);
	u8 mem = fpga_strap & 0x03;

	/* There are overlapping FPGA strap values between S and Q, therefore,
	 * if we're on an i.MX6Q, set this in our strap mapping.
	 */
	if (is_mx6dq())
		mem |= 0x80;

	/* We only need to normalize mapping of Rev A and C. Rev B does not
	 * exist, and Rev D+ should all use normalized meanings of the strap
	 * values.
	 */
	switch (rev) {
	case REV_E:
	case REV_D:
		break;
	case REV_C:
		switch (mem) {
		case REV_C_S_8_1G:
			mem = S_8_1G;
			break;
		case REV_AC_S_10_1G:
			mem = S_10_1G;
			break;
		case REV_AC_Q_10_2G:
			mem = Q_10_2G;
			break;
		default:
			printf("Error! Invalid straps read from FPGA!\n");
			mem = MEM_ERR;
			break;
		}
		break;
	case REV_A:
		switch (mem) {
		case REV_A_S_8_1G:
			mem = S_8_1G;
			break;
		case REV_AC_S_10_1G:
			mem = S_10_1G;
			break;
		case REV_AC_Q_10_2G:
			mem = Q_10_2G;
			break;
		default:
			printf("Error! Invalid straps read from FPGA!\n");
			mem = MEM_ERR;
			break;
		}
		break;
	default:
		printf("Unknown PCB Rev! Assuming normalized strapping\n");
		break;
	}

	return mem;
}

enum ram_configs ts4900_ram_strap_decode(u8 cpu, u8 fpga)
{
	enum ram_configs config;

	printf("KRIS: PCB Rev. %c\n", ts4900_pcb_rev_char(cpu));

	switch (normalize_fpga_strap(cpu, fpga)) {
	case S_8_1G:
		config = s_1g_800mhz;
		printf("s-1g-800\n");
		break;
	case S_10_1G:
		config = s_1g_1000mhz;
		printf("s-1g-1000\n");
		break;
	case S_8_2G:
		config = s_2g_800mhz;
		printf("s-2g-800\n");
		break;
	case Q_10_2G:
		config = q_2g_1000mhz;
		printf("q-2g-1000\n");
		break;
	default:
		config = ram_config_cnt;
		printf("ERROR\n");
		break;
	}

	return config;
}
