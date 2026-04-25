// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016-2026 Technologic Systems, Inc. dba embeddedTS
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CFG_MXC_UART_BASE		UART1_BASE

/* iCE40 FPGA programmed from SPL */
#define CFG_ICE40_FPGA_DONE		IMX_GPIO_NR(5, 20)
#define CFG_ICE40_FPGA_RESET		IMX_GPIO_NR(5, 21)
#define CFG_ICE40_CS			IMX_GPIO_NR(6, 2)
/* XXX: TODO: Clean this up later with binman to not be hardcoded */
#define CFG_ICE40_START			0x200000
#define CFG_ICE40_LEN			0x1cf72

/* Physical Memory Map */
#define CFG_SYS_SDRAM_BASE		MMDC0_ARB_BASE_ADDR
#define CFG_SYS_INIT_RAM_ADDR		IRAM_BASE_ADDR
#define CFG_SYS_INIT_RAM_SIZE		IRAM_SIZE

#endif // __CONFIG_H
