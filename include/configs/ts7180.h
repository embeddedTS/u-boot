/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2020-2022 Technologic Systems, Inc. dba embeddedTS
 */
#ifndef __TS7180_CONFIG_H
#define __TS7180_CONFIG_H


#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/mach-imx/gpio.h>
#include "ts_imx6ul_env.h"

/* SPL options in include/configs */

/* Size of malloc() pool */
#define CFG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CFG_MXC_UART
#define CFG_MXC_UART_BASE		UART1_BASE

/* MMC Configs */
#ifdef CONFIG_FSL_USDHC
#define CFG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR
#define CFG_SYS_FSL_USDHC_NUM	2
#endif

#define CFG_IMX_THERMAL

#define FDT_ADDR_R			0x83000000
#define RAMDISK_ADDR_R			0x90000000

#define CFG_EXTRA_ENV_SETTINGS \
	"board=ts7180\0" \
	"autoload=no\0" \
	"netretry=once\0" \
	"clearenv=env default -f -a; env save;\0" \
	"loadaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdtaddr=" __stringify(FDT_ADDR_R) "\0" \
	"kernel_addr_r=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0" \
	"fdt_addr_r=" __stringify(FDT_ADDR_R) "\0" \
	"ramdisk_addr_r=" __stringify(RAMDISK_ADDR_R) "\0" \
	"scriptaddr=" __stringify(CONFIG_LOADADDR) "\0" \
	"pxefile_addr_r=" __stringify(CONFIG_LOADADDR) "\0" \
	"fdtfile=imx6ul-ts7180.dtb\0" \
	"console=ttymxc0,115200\0" \
	"ethact=ethernet@2188000\0" \
	"silo_chrg_pct=0\0" \
	TS_IMX6UL_NFSBOOT_ENV

/* Miscellaneous configurable options */
#define CFG_SYS_MEMTEST_START	0x80000000
#define CFG_SYS_MEMTEST_END		(CFG_SYS_MEMTEST_START + 0x8000000)

#define CFG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CFG_SYS_HZ			1000

/* Physical Memory Map */
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CFG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CFG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CFG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CFG_SYS_INIT_SP_OFFSET \
	(CFG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CFG_SYS_INIT_SP_ADDR \
	(CFG_SYS_INIT_RAM_ADDR + CFG_SYS_INIT_SP_OFFSET)

/* environment organization */
#define CFG_SYS_MMC_ENV_DEV		1

/* I2C Configs */
#define CFG_SYS_I2C_MXC
#define CFG_SYS_I2C_MXC_I2C1
#define CFG_SYS_I2C_MXC_I2C3
#define CFG_SYS_I2C_SPEED		100000

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CFG_EHCI_HCD_INIT_AFTER_RESET
#define CFG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CFG_MXC_USB_FLAGS   0
#define CFG_USB_MAX_CONTROLLER_COUNT 2
#endif

#endif
