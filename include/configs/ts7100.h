/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2020-2023 Technologic Systems dba embeddedTS
 */
#ifndef __TS7100_CONFIG_H
#define __TS7100_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/mach-imx/gpio.h>
#include "ts_imx6ul_env.h"

#define CFG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CFG_MXC_UART
#define CFG_MXC_UART_BASE		UART1_BASE

#ifdef CONFIG_FSL_USDHC
#define CFG_SYS_FSL_ESDHC_ADDR	USDHC1_BASE_ADDR
#define CFG_SYS_FSL_USDHC_NUM	1
#endif

#define FDT_ADDR_R			0x83000000
#define RAMDISK_ADDR_R			0x90000000

#define CFG_EXTRA_ENV_SETTINGS \
	"board=ts7100\0" \
	"autoload=no\0" \
	"loadaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"kernel_addr_r=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0" \
	"fdt_addr_r=" __stringify(FDT_ADDR_R) "\0" \
	"ramdisk_addr_r=" __stringify(RAMDISK_ADDR_R) "\0" \
	"scriptaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"pxefile_addr_r=" __stringify(CONFIG_LOADADDR) "\0" \
	"console=ttymxc0,115200\0" \
	"silo_chrg_pct=0\0" \
	TS_IMX6UL_NFSBOOT_ENV

#define CFG_SYS_MEMTEST_START		0x80000000
#define CFG_SYS_MEMTEST_END		(CFG_SYS_MEMTEST_START + 0x20000000)

#define CFG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CFG_SYS_HZ			1000

#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define CFG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CFG_SYS_INIT_RAM_ADDR		IRAM_BASE_ADDR
#define CFG_SYS_INIT_RAM_SIZE		IRAM_SIZE

#define CFG_SYS_INIT_SP_OFFSET \
	(CFG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CFG_SYS_INIT_SP_ADDR \
	(CFG_SYS_INIT_RAM_ADDR + CFG_SYS_INIT_SP_OFFSET)

#define CFG_SYS_MMC_ENV_DEV		0
#define CFG_SYS_I2C_MXC
#define CFG_SYS_I2C_MXC_I2C1
#define CFG_SYS_I2C_SPEED		100000

#ifdef CONFIG_CMD_USB
#define CFG_EHCI_HCD_INIT_AFTER_RESET
#define CFG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CFG_MXC_USB_FLAGS		0
#define CFG_USB_MAX_CONTROLLER_COUNT	2
#endif

#endif /* __TS7100_CONFIG_H */
