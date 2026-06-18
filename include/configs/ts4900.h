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

/* Undefine this to not support legacy boot flow, e.g. sdboot, emmcboot,
 * sataboot, etc., commands a part of the environment, and instead rely on
 * the booted distribution supporting only standard boot.
 */
#define CFG_ETS_LEGACY_BOOT		1

#define CFG_MXC_UART_BASE		UART1_BASE
#define CONSOLE_DEV			"ttymxc0"

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

#ifdef CFG_ETS_LEGACY_BOOT
#define CFG_EXTRA_ENV_SETTINGS \
		CFG_EXTRA_ENV_SETTINGS_COMMON \
		CFG_EXTRA_ENV_SETTINGS_LEGACY
#else
#define CFG_EXTRA_ENV_SETTINGS \
		CFG_EXTRA_ENV_SETTINGS_COMMON
#endif

/* To support legacy images as well as modern ones relying on standard boot,
 * we define a handful of common env settings to be included in either env
 * case.
 */
#define CFG_EXTRA_ENV_SETTINGS_COMMON \
	"initrd_addr=0x10800000\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_addr_r=0x18000000\0" \
	"fdt_high=0xffffffff\0" \

#define CFG_EXTRA_ENV_SETTINGS_LEGACY \
	"uimage=/boot/uImage\0" \
	"cmdline_append=console=ttymxc0,115200 ro init=/sbin/init\0" \
	"findfdt=" \
		"if test $rev = 'E'; then " \
			"if load ${bootdev} ${bootpart} ${fdt_addr_r} /boot/imx6${cpu}-ts4900-reve-${baseboard_id}.dtb; then " \
				"echo Baseboard $baseboard_id detected;" \
			"elif load ${bootdev} ${bootpart} ${fdt_addr_r} /boot/imx6${cpu}-ts4900-reve.dtb; then " \
				"echo Booting default device tree;" \
			"elif load ${bootdev} ${bootpart} ${fdt_addr_r} /boot/imx6${cpu}-ts4900-${baseboard_id}.dtb; then " \
				"echo Baseboard $baseboard_id detected;" \
			"elif load ${bootdev} ${bootpart} ${fdt_addr_r} /boot/imx6${cpu}-ts4900.dtb; then " \
				"echo Booting default device tree;" \
			"fi;" \
		"else " \
			"if load ${bootdev} ${bootpart} ${fdt_addr_r} /boot/imx6${cpu}-ts4900-${baseboard_id}.dtb; then " \
				"echo Baseboard $baseboard_id detected;" \
			"elif load ${bootdev} ${bootpart} ${fdt_addr_r} /boot/imx6${cpu}-ts4900.dtb; then " \
				"echo Booting default device tree;" \
			"fi;" \
		"fi;\0" \
	"bootlinux=if load ${bootdev} ${bootpart} ${loadaddr} /boot/boot.ub; " \
			"then echo Booting from custom /boot/boot.ub; " \
			"source ${loadaddr}; " \
		"fi; " \
		"run findfdt; " \
		"load ${bootdev} ${bootpart} ${loadaddr} ${uimage}; " \
		"setenv bootargs root=${rootdev} rootwait rw ${cmdline_append}; " \
		"bootm ${loadaddr} - ${fdt_addr_r};\0" \
	"sdboot=echo Booting from the SD card ...; " \
		"env set bootdev mmc; " \
		"env set bootpart 0:1; " \
		"env set rootdev '/dev/mmcblk1p1'; " \
		"run bootlinux;\0" \
	"emmcboot=echo Booting from the eMMC ...; " \
		"env set bootdev mmc; " \
		"env set bootpart 1:1; " \
		"env set rootdev '/dev/mmcblk2p1'; " \
		"run bootlinux;\0" \
	"usbboot=echo Booting from USB ...; " \
		"env set bootdev usb; " \
		"env set bootpart 0:1; " \
		"env set rootdev '/dev/sda1'; " \
		"usb start; " \
		"run bootlinux;\0" \
	"usbprod=usb start; " \
		"if usb storage; " \
			"then echo Checking USB storage for updates; " \
			"if load usb 0:1 ${loadaddr} /tsinit.ub; " \
				"then led green on;" \
				"source ${loadaddr}; " \
				"led red off; " \
				"exit; " \
			"fi; " \
		"fi; \0"

#if 0
XXX: TODO: This needs to be dealt with at some point, probably when moving the env
#ifdef CFG_ETS_LEGACY_BOOT
#define CONFIG_BOOTCOMMAND \
	"run usbprod; " \
	"if test ${jpsdboot} = 'on' ; " \
		"then run sdboot; " \
		"else run emmcboot; " \
	"fi;"
#endif
#endif

#endif // __CONFIG_H
