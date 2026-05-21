#ifndef STRAP_DECODE_H
#define STRAP_DECODE_H

#include <asm/types.h>

enum ram_configs {
	s_1g_800mhz = 0,
	s_2g_800mhz,
	s_1g_1000mhz,
	q_2g_1000mhz,
	ram_config_cnt,
};

char ts4900_pcb_rev_char(u8 cpu_strap);

enum ram_configs ts4900_ram_strap_decode(u8 cpu, u8 fpga);

#endif // STRAP_DECODD_H
