/*
 * (C) Copyright 2011 Freescale Semiconductor, Inc.
 *
 * Trafficnet config
 * Based on m28evk.h
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __CONFIGS_NGMINI_H__
#define __CONFIGS_NGMINI_H__

/* System configurations */
#define CONFIG_MX28				/* i.MX28 SoC */

/* U-Boot Commands */
#define CONFIG_SYS_NO_FLASH
#include <config_cmd_default.h>
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DOS_PARTITION

#define CONFIG_BOARD_SPECIFIC_LED
#define CONFIG_STATUS_LED
#define CONFIG_CMD_LED

#define STATUS_LED_BOOT			STATUS_LED_GREEN

#define CONFIG_RED_LED                  MX28_PAD_GPMI_D07__GPIO_0_7
#define CONFIG_GREEN_LED                MX28_PAD_GPMI_D05__GPIO_0_5
#define STATUS_LED_RED			0
#define STATUS_LED_GREEN		1

#define STATUS_LED_BIT                  STATUS_LED_RED
#define STATUS_LED_STATE                STATUS_LED_ON
#define STATUS_LED_PERIOD               (CONFIG_SYS_HZ / 2)

#define STATUS_LED_BIT1                 STATUS_LED_GREEN
#define STATUS_LED_STATE1               STATUS_LED_OFF
#define STATUS_LED_PERIOD1              (CONFIG_SYS_HZ / 2)

#define CONFIG_CMD_CACHE
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_GPIO
#define CONFIG_CMD_MII
#define CONFIG_CMD_MMC
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_EXT4_WRITE
#define CONFIG_CMD_FS_GENERIC
#define CONFIG_CMD_NET
#define CONFIG_CMD_NFS
#define CONFIG_CMD_PING
#define CONFIG_CMD_SAVEENV
#define CONFIG_CMD_SETEXPR
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_SF
#define CONFIG_CMD_SPI
#define CONFIG_CMD_USB
#define CONFIG_CMD_TIME
#define CONFIG_LIB_RAND
#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ
#define CONFIG_SUPPORT_RAW_INITRD

/* Memory configuration */
#define CONFIG_NR_DRAM_BANKS		1		/* 1 bank of DRAM */
#define PHYS_SDRAM_1			0x40000000	/* Base address */
#define PHYS_SDRAM_1_SIZE		0x40000000	/* Max 1 GB RAM */
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1

/* Environment */
#define CONFIG_SYS_NO_FLASH
#define CONFIG_ENV_SIZE			(8 * 1024)
#define CONFIG_ENV_OVERWRITE

/* Environemnt is in SPI flash */
#define CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_OFFSET		0x100000	
#define CONFIG_ENV_SECT_SIZE		(4 * 1024)
#define CONFIG_ENV_SPI_CS		0
#define CONFIG_ENV_SPI_BUS		2
#define CONFIG_ENV_SPI_MAX_HZ		24000000
#define CONFIG_ENV_SPI_MODE		SPI_MODE_0

/* FEC Ethernet on SoC */
#ifdef	CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_NET_MULTI
#define CONFIG_MX28_FEC_MAC_IN_OCOTP
#endif

/* NFS */
#ifdef CONFIG_CMD_NFS
#define CONFIG_NFS_TIMEOUT 100UL
#endif

/* USB */
#ifdef	CONFIG_CMD_USB
#define CONFIG_EHCI_MXS_PORT1
#define CONFIG_USB_MAX_CONTROLLER_COUNT	1
#define	CONFIG_USB_STORAGE
#define	CONFIG_USB_HOST_ETHER
#define	CONFIG_USB_ETHER_ASIX
#define	CONFIG_USB_ETHER_SMSC95XX
#endif

/* I2C */
#define CONFIG_CMD_I2C
#define CONFIG_I2C_MXS
#define CONFIG_SYS_I2C_SPEED            100000 

/* SPI */
#ifdef CONFIG_CMD_SPI
#define CONFIG_DEFAULT_SPI_BUS		2
#define CONFIG_DEFAULT_SPI_MODE		SPI_MODE_0

/* SPI Flash */
#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SPI_FLASH_ISSI
#define CONFIG_SF_DEFAULT_BUS		2
#define CONFIG_SF_DEFAULT_CS		0
/* this may vary and depends on the installed chip */
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#define CONFIG_SF_DEFAULT_SPEED		24000000
#endif

#endif

#define CONFIG_LOADADDR		0x42000000
#define CONFIG_SYS_LOAD_ADDR	CONFIG_LOADADDR
#define CONFIG_MISC_INIT_R

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT              "U-Boot > "
#define CONFIG_AUTO_COMPLETE

#define CONFIG_BOOTDELAY	           1
#define CONFIG_AUTOBOOT_KEYED          1
#define CONFIG_AUTOBOOT_PROMPT         "Press Ctrl+C to abort autoboot in %d second(s)\n", bootdelay
#define CTRL(c) ((c)&0x1F)     
#define CONFIG_AUTOBOOT_STOP_STR       (char []){CTRL('C'), 0}

#define CONFIG_PREBOOT \
	"if test \"${jpuboot}\" = \"on\"; then " \
		" setenv bootdelay -1; " \
		" echo UBoot jumper installed, checking usb and stopping boot.; " \
		" run usbprod; " \
	" else " \
		" setenv bootdelay 0; " \
	"fi" 

/* Extra Environment */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"autoload=no\0" \
	"nfsroot=/nfsroot/\0" \
	"nfsip=192.168.0.1\0" \
	"fdtaddr=0x41000000\0" \
	"cmdline_append=rw rootwait console=ttyAMA0,115200 loglevel=3\0" \
	"boot_fdt=yes\0" \
	"ip_dyn=yes\0" \
	"clearenv=if sf probe; then " \
		"sf erase 0x100000 0x2000 && " \
		"echo restored environment to factory default ; fi\0" \
	"update-uboot=if test \"${spi}\" = \"onboard\"; " \
			"then mx28_prod 1; " \
			"echo Overriding SPI sel., writing to onboard SPI; " \
		"else if test \"${spi}\" = \"offboard\"; " \
			"then mx28_prod 2; " \
			"echo Overriding SPI sel., writing to offboard SPI; " \
		"fi; " \
		"fi; " \
		"echo Updating u-boot from /boot/u-boot.sb; " \
		"if test ${jpsdboot} = 'on' ; " \
			"then if load usb 0:2 ${loadaddr} /boot/u-boot.sb; " \
				"then sf probe; " \
				"sf erase 0 80000; " \
				"sf write ${loadaddr} 0 ${filesize}; " \
			"fi;" \
		"else " \
			"if load mmc 1:2 ${loadaddr} /boot/u-boot.sb; " \
				"then sf probe; " \
				"sf erase 0 80000; " \
				"sf write ${loadaddr} 0 ${filesize}; " \
			"fi; " \
		"fi;" \
		"mx28_prod 0;\0 " \
	"emmcboot=echo Booting from the onboard eMMC  ...; " \
		"if load mmc 1:2 ${loadaddr} /boot/boot.ub; " \
			"then echo Booting from custom /boot/boot.ub; " \
			"source ${loadaddr}; " \
		"fi; " \
		"load mmc 1:2 ${loadaddr} /boot/uImage; " \
		"load mmc 1:2 ${fdtaddr} /boot/imx28-tsngmini.dtb; " \
		"setenv bootargs root=/dev/mmcblk2p2 ${cmdline_append}; " \
		"mx28_prod 3;" \
		"bootm ${loadaddr} - ${fdtaddr}; \0"\
	"sdboot=echo Booting from the SD Card ...; " \
		"if load mmc 0:2 ${loadaddr} /boot/boot.ub; " \
			"then echo Booting from custom /boot/boot.ub; " \
			"source ${loadaddr}; " \
		"fi; " \
		"load mmc 0:2 ${loadaddr} /boot/uImage; " \
		"load mmc 0:2 ${fdtaddr} /boot/imx28-tsngmini.dtb; " \
		"setenv bootargs root=/dev/mmcblk0p2 ${cmdline_append}; " \
		"mx28_prod 3;" \
		"bootm ${loadaddr} - ${fdtaddr}; \0"\
	"usbboot=echo Booting from USB ...; " \
		"usb start;" \
		"if load usb 0:2 ${loadaddr} /boot/boot.ub; " \
			"then echo Booting from custom /boot/boot.ub; " \
			"source ${loadaddr}; " \
		"fi; " \
		"load usb 0:2 ${loadaddr} /boot/uImage; " \
		"load usb 0:2 ${fdtaddr} /boot/imx28-tsngmini.dtb; " \
		"setenv bootargs root=/dev/sda2 ${cmdline_append}; " \
		"mx28_prod 3;" \
		"bootm ${loadaddr} - ${fdtaddr}; \0"\
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
		"dhcp; " \
		"env set serverip ${nfsip}; " \
		"nfs ${loadaddr} ${nfsroot}/boot/uImage; " \
		"setenv bootargs root=/dev/nfs ip=dhcp " \
		  "nfsroot=${serverip}:${nfsroot},vers=2,nolock ${cmdline_append}; " \
		"nfs ${fdtaddr} ${nfsroot}/boot/imx28-tsngmini.dtb; " \
		"mx28_prod 3;" \
		"bootm ${loadaddr} - ${fdtaddr};\0"\

#define CONFIG_BOOTCOMMAND \
	"if test ${jpsdboot} = 'on' ; " \
		"then run usbboot; " \
		"else run emmcboot; " \
	"fi;"
	
/* The rest of the configuration is shared */
#include <configs/mxs.h>

#endif /* __CONFIGS_NGMINI_H__ */
