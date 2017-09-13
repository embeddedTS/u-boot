/*
 * Copyright (C) 20167Technologic Systems
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __TS4900_CONFIG_H
#define __TS4900_CONFIG_H

#include <linux/sizes.h>
#include <config_distro_defaults.h>
#include "mx6_common.h"

#define CONFIG_MACH_TYPE		4843
#define CONFIG_SYS_GENERIC_BOARD
#undef CONFIG_CMD_IMLS

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		SZ_32M

#define CONFIG_MISC_INIT_R
#define CONFIG_BOARD_EARLY_INIT_F

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE
#define CONFIG_BAUDRATE			115200

/* I2C Configs */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_SPEED		100000

#define CONFIG_MAX_FPGA_DEVICES		1

/* MMC Configs */
#define CONFIG_SYS_FSL_ESDHC_ADDR	0
#define CONFIG_SYS_FSL_USDHC_NUM	2

#ifdef CONFIG_CMD_SATA
#define CONFIG_DWC_AHSATA
#define CONFIG_SYS_SATA_MAX_DEVICE	1
#define CONFIG_DWC_AHSATA_PORT_ID	0
#define CONFIG_DWC_AHSATA_BASE_ADDR	SATA_ARB_BASE_ADDR
#define CONFIG_LBA48
#define CONFIG_LIBATA
#endif

/*
 * PCI express
 */
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#endif

#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		7
#define CONFIG_HAS_ETH1
#define CONFIG_NFS_TIMEOUT		300UL

/* USB Configs */ 
#define CONFIG_USBD_HS
#define CONFIG_USB_ETHER
#define CONFIG_USB_FUNCTION_MASS_STORAGE
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET	
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0

/* Create build specific env vars */
#ifdef CONFIG_MX6Q
#define ENV_CPU_TYPE 			"cpu=q\0"
#else
#define ENV_CPU_TYPE 			"cpu=dl\0"
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
	"initrd_high=0xffffffff\0" \
	"fdtaddr=0x18000000\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_addr=0x10800000\0 " \
	ENV_CPU_TYPE \
	"model=4900\0" \
	"autoload=no\0" \
	"nfsroot=192.168.0.36:/mnt/storage/imx6/\0" \
	"cmdline_append=console=ttymxc0,115200 ro init=/sbin/init\0" \
	"clearenv=if sf probe; then " \
		"sf erase 0x100000 0x2000 && " \
		"echo restored environment to factory default ; fi\0" \
	"sdboot=echo Booting from the SD card ...; " \
		"if load mmc 0:1 ${loadaddr} /boot/boot.ub; " \
			"then echo Booting from custom /boot/boot.ub; " \
			"source ${loadaddr}; " \
		"fi; " \
		"if load mmc 0:1 ${fdtaddr} /boot/imx6${cpu}-ts4900-${baseboardid}.dtb; " \
			"then echo $baseboardid detected; " \
		"else " \
			"echo Booting default device tree; " \
			"load mmc 0:1 ${fdtaddr} /boot/imx6${cpu}-ts4900.dtb; " \
		"fi; " \
		"load mmc 0:1 ${loadaddr} /boot/ts4900-fpga.bin; " \
		"fpga load 0 ${loadaddr} ${filesize}; " \
		"load mmc 0:1 ${loadaddr} /boot/uImage; " \
		"setenv bootargs root=/dev/mmcblk1p1 rootwait rw ${cmdline_append}; " \
		"bootm ${loadaddr} - ${fdtaddr}; \0" \
	"emmcboot=echo Booting from the eMMC ...; " \
		"if load mmc 1:1 ${loadaddr} /boot/boot.ub; " \
			"then echo Booting from custom /boot/boot.ub; " \
			"source ${loadaddr}; " \
		"fi; " \
		"if load mmc 1:1 ${fdtaddr} /boot/imx6${cpu}-ts4900-${baseboardid}.dtb; " \
			"then echo $baseboardid detected; " \
		"else " \
			"echo Booting default device tree; " \
			"load mmc 1:1 ${fdtaddr} /boot/imx6${cpu}-ts4900.dtb; " \
		"fi; " \
		"load mmc 1:1 ${loadaddr} /boot/ts4900-fpga.bin; " \
		"fpga load 0 ${loadaddr} ${filesize}; " \
		"load mmc 1:1 ${loadaddr} /boot/uImage; " \
		"setenv bootargs root=/dev/mmcblk2p1 rootwait rw ${cmdline_append}; " \
		"bootm ${loadaddr} - ${fdtaddr}; \0" \
	"sataboot=echo Booting from SATA ...; " \
		"sata init; " \
		"if load sata 0:1 ${loadaddr} /boot/boot.ub; " \
			"then echo Booting from custom /boot/boot.ub; " \
			"source ${loadaddr}; " \
		"fi; " \
		"if load sata 0:1 ${fdtaddr} /boot/imx6${cpu}-ts4900-${baseboardid}.dtb; " \
			"then echo $baseboardid detected; " \
		"else " \
			"echo Booting default device tree; " \
			"load sata 0:1 ${fdtaddr} /boot/imx6${cpu}-ts4900.dtb; " \
		"fi; " \
		"load sata 0:1 ${loadaddr} /boot/ts4900-fpga.bin; " \
		"fpga load 0 ${loadaddr} ${filesize}; " \
		"load sata 0:1 ${loadaddr} /boot/uImage; " \
		"setenv bootargs root=/dev/sda1 rootwait rw ${cmdline_append}; " \
		"bootm ${loadaddr} - ${fdtaddr}; \0" \
	"usbprod=usb start; " \
		"if usb storage; " \
			"then echo Checking USB storage for updates; " \
			"if load usb 0:1 ${loadaddr} /tsinit.ub; " \
				"then led green on;" \
				"source ${loadaddr}; " \
				"led red off; " \
				"exit; " \
			"fi; " \
		"fi; \0" \
	"nfsboot=echo Booting from NFS ...; " \
		"dhcp ; " \
		"nfs ${fdtaddr} ${nfsroot}/boot/imx6${cpu}-ts4900-${baseboardid}.dtb; " \
		"if fdt addr ${fdtaddr}; " \
			"then echo $baseboardid detected; " \
		"else " \
			"echo Booting default device tree; " \
			"nfs ${fdtaddr} ${nfsroot}/boot/imx6${cpu}-ts4900.dtb; " \
		"fi; " \
		"nfs ${loadaddr} ${nfsroot}/boot/ts4900-fpga.bin; " \
		"fpga load 0 ${loadaddr} ${filesize}; " \
		"nfs ${loadaddr} ${nfsroot}/boot/uImage; " \
		"setenv bootargs root=/dev/nfs ip=dhcp nfsroot=${nfsroot} " \
			"rootwait rw init=/sbin/init ${cmdline_append}; " \
		"bootm ${loadaddr} - ${fdtaddr}; \0"

#define CONFIG_BOOTCOMMAND \
	"run usbprod; " \
	"if test ${jpsdboot} = 'on' ; " \
		"then run sdboot; " \
		"else run emmcboot; " \
	"fi;"

#define CONFIG_PREBOOT			""
#define CONFIG_BOOTCOUNT_LIMIT
#define CONFIG_BOOTCOUNT_ENV
#define CONFIG_IMX_WATCHDOG
#define CONFIG_HW_WATCHDOG
#define CONFIG_IMX_WATCHDOG
#define CONFIG_WATCHDOG_TIMEOUT_MSECS	30000 /* 30s */

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		0x10010000
#define CONFIG_SYS_MEMTEST_SCRATCH	0x10800000
#define CONFIG_SYS_ALT_MEMTEST

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

#define CONFIG_ENV_SIZE			(8 * 1024)
#define CONFIG_ENV_OFFSET		0x100000
#define CONFIG_ENV_SECT_SIZE		(8 * 1024)

#define CONFIG_GPIO_LED_INVERTED_TABLE	{2, 56}

#ifdef CONFIG_CMD_SF
#define CONFIG_MXC_SPI
#define CONFIG_SF_DEFAULT_BUS		0
#define CONFIG_SF_DEFAULT_CS		0
#define CONFIG_SF_DEFAULT_SPEED		15000000
#define CONFIG_SF_DEFAULT_MODE		(SPI_MODE_0)
#endif

#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED

#endif /* __TS4900_CONFIG_H */
