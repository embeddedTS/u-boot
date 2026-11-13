/* SPDX-License-Identifier: GPL-2.0+ */
/* Copyright (C) 2020-2022 embeddedTS */
#ifndef __TS7250V3_CONFIG_H
#define __TS7250V3_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/mach-imx/gpio.h>

#define CFG_SYS_MALLOC_LEN		(16 * SZ_1M)
#define CFG_MXC_UART
#define CFG_MXC_UART_BASE		UART1_BASE

#ifdef CONFIG_FSL_USDHC
#define CFG_SYS_FSL_ESDHC_ADDR		USDHC1_BASE_ADDR
#define CFG_SYS_FSL_USDHC_NUM		2
#endif

#define CFG_IMX_THERMAL
#define CFG_SYS_MEMTEST_START		0x80000000
#define CFG_SYS_MEMTEST_END		(CFG_SYS_MEMTEST_START + 0x08000000)
#define CFG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CFG_SYS_HZ			1000
#define CFG_SYS_SDRAM_BASE		MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM			CFG_SYS_SDRAM_BASE
#define CFG_SYS_INIT_RAM_ADDR		IRAM_BASE_ADDR
#define CFG_SYS_INIT_RAM_SIZE		IRAM_SIZE
#define CFG_SYS_INIT_SP_OFFSET \
	(CFG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CFG_SYS_INIT_SP_ADDR \
	(CFG_SYS_INIT_RAM_ADDR + CFG_SYS_INIT_SP_OFFSET)

#define CFG_SYS_MMC_ENV_DEV		0
#define CFG_SYS_MMC_ENV_PART		1

#define CFG_EXTRA_ENV_SETTINGS \
	"board=ts7250v3\0" \
	"autoload=no\0" \
	"netretry=once\0" \
	"loadaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdtaddr=0x83000000\0" \
	"kernel_addr_r=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdt_addr_r=0x83000000\0" \
	"ramdisk_addr_r=0x90000000\0" \
	"scriptaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"pxefile_addr_r=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdtfile=imx6ul-ts7250v3.dtb\0" \
	"console=ttymxc0,115200\0" \
	"boot_targets=usb0 mmc1 mmc0 dhcp pxe\0" \
	"clearenv=mmc dev 0 1; mmc erase 800 2000; mmc erase C00 2000;\0"

#endif /* __TS7250V3_CONFIG_H */
