/*
 * Copyright (C) 2016-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * Configuration settings for the embeddedTS TS-4900
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"

#define CFG_MXC_UART_BASE		UART1_BASE

/* iCE40 FPGA programmed from SPL */
#define CFG_ICE40_BUS			0
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

#if 0
Verify SATA functions
#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif
#endif

#endif // __CONFIG_H
