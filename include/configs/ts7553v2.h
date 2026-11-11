/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2017-2022 Technologic Systems, Inc. dba embeddedTS
 */
#ifndef __TS7553V2_CONFIG_H
#define __TS7553V2_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"

#define CFG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define CFG_MXC_UART
#define CFG_MXC_UART_BASE		UART1_BASE

#define CFG_SYS_MMC_ENV_DEV		1

#define CFG_EXTRA_ENV_SETTINGS \
	"board=ts7553v2\0" \
	"boot_targets=mmc1 mmc0 usb0 pxe dhcp\0"

#define CFG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CFG_SYS_HZ			1000

#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define PHYS_SDRAM_SIZE			SZ_512M
#define CFG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CFG_SYS_INIT_RAM_ADDR		IRAM_BASE_ADDR
#define CFG_SYS_INIT_RAM_SIZE		IRAM_SIZE
#define CFG_SYS_INIT_SP_OFFSET \
	(CFG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CFG_SYS_INIT_SP_ADDR \
	(CFG_SYS_INIT_RAM_ADDR + CFG_SYS_INIT_SP_OFFSET)

#define CFG_SYS_I2C_MXC
#define CFG_SYS_I2C_MXC_I2C1
#define CFG_SYS_I2C_SPEED		100000

#ifdef CONFIG_CMD_USB
#define CFG_EHCI_HCD_INIT_AFTER_RESET
#define CFG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CFG_MXC_USB_FLAGS		0
#define CFG_USB_MAX_CONTROLLER_COUNT	2
#endif

#endif /* __TS7553V2_CONFIG_H */
