/*
 * (C) Copyright 2011 Freescale Semiconductor, Inc.
 *
 * TS-7680 config
 * Based on m28evk.h
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __CONFIGS_TS7680_H__
#define __CONFIGS_TS7680_H__

/* System configurations */
#define CONFIG_MX28				/* i.MX28 SoC */
#define CONFIG_MACH_TYPE	MACH_TYPE_MX28EVK

/* U-Boot Commands */
#define CONFIG_SYS_NO_FLASH
#include <config_cmd_default.h>
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DOS_PARTITION

#define CONFIG_BOARD_SPECIFIC_LED
#define CONFIG_STATUS_LED
#define CONFIG_CMD_LED

#define STATUS_LED_BOOT			STATUS_LED_GREEN

#define STATUS_LED_RED			0
#define STATUS_LED_GREEN		1
#define STATUS_LED_YELLOW		2
#define STATUS_LED_BLUE			3

#define STATUS_LED_BIT                  STATUS_LED_RED
#define STATUS_LED_STATE                STATUS_LED_ON
#define STATUS_LED_PERIOD               (CONFIG_SYS_HZ / 2)

#define STATUS_LED_BIT1                 STATUS_LED_GREEN
#define STATUS_LED_STATE1               STATUS_LED_ON
#define STATUS_LED_PERIOD1              (CONFIG_SYS_HZ / 2)

#define STATUS_LED_BIT2                 STATUS_LED_YELLOW
#define STATUS_LED_STATE2               STATUS_LED_ON
#define STATUS_LED_PERIOD2              (CONFIG_SYS_HZ / 2)

#define STATUS_LED_BIT3                 STATUS_LED_BLUE
#define STATUS_LED_STATE3               STATUS_LED_ON
#define STATUS_LED_PERIOD3              (CONFIG_SYS_HZ / 2)


#define CONFIG_CMD_CACHE
#define CONFIG_CMD_DATE
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_GPIO
#define CONFIG_CMD_MII
#define CONFIG_CMD_MMC
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_CMD_EXT4
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
#define CONFIG_LIB_RAND

/* Memory configuration */
#define CONFIG_NR_DRAM_BANKS		1		/* 1 bank of DRAM */
#define PHYS_SDRAM_1			0x40000000	/* Base address */
#define PHYS_SDRAM_1_SIZE		0x40000000	/* Max 1 GB RAM */
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1

/* Environment */
#define CONFIG_ENV_SIZE			(8 * 1024)
#define CONFIG_ENV_OVERWRITE

/* Environment is in MMC */
#define CONFIG_SYS_MMC_ENV_DEV		0
#if defined(CONFIG_CMD_MMC) && defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(256 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0
#endif

/* Environemnt is in SPI flash */
#if defined(CONFIG_CMD_SF) && defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_SYS_REDUNDAND_ENVIRONMENT
#define CONFIG_ENV_OFFSET		0x40000		/* 256K */
#define CONFIG_ENV_OFFSET_REDUND	(CONFIG_ENV_OFFSET + CONFIG_ENV_SIZE)
#define CONFIG_ENV_SECT_SIZE		0x1000
#define CONFIG_ENV_SPI_CS		0
#define CONFIG_ENV_SPI_BUS		2
#define CONFIG_ENV_SPI_MAX_HZ		24000000
#define CONFIG_ENV_SPI_MODE		SPI_MODE_0
#endif

/* FEC Ethernet on SoC */
#ifdef	CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_NET_MULTI
#define CONFIG_MX28_FEC_MAC_IN_OCOTP
#endif

/* RTC */
#ifdef	CONFIG_CMD_DATE
#define	CONFIG_RTC_MXS
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
#ifdef CONFIG_CMD_I2C
#define CONFIG_I2C_MXS
#define CONFIG_SYS_I2C_SPEED            100000
#endif

/* SPI */
#ifdef CONFIG_CMD_SPI
#define CONFIG_DEFAULT_SPI_BUS		2
#define CONFIG_DEFAULT_SPI_MODE		SPI_MODE_0

/* SPI Flash */
#ifdef CONFIG_CMD_SF
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SF_DEFAULT_BUS		2
#define CONFIG_SF_DEFAULT_CS		0
/* this may vary and depends on the installed chip */
#define CONFIG_SF_DEFAULT_MODE		SPI_MODE_0
#define CONFIG_SF_DEFAULT_SPEED		24000000
#endif

#endif

/* Framebuffer support */
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_LOGO
#define CONFIG_SPLASH_SCREEN
#define CONFIG_CMD_BMP
#define CONFIG_BMP_16BPP
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_VIDEO_BMP_GZIP
#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE	(512 << 10)
#endif

/* Boot Linux */
#define CONFIG_BOOTDELAY	1
#define CONFIG_AUTOBOOT_KEYED   1
#define CONFIG_AUTOBOOT_PROMPT  "Press Ctrl+c to abort autoboot in %d second\n", bootdelay
#define CTRL(c) ((c)&0x1F)     
#define CONFIG_AUTOBOOT_STOP_STR  (char []){CTRL('C'), 0}

#define CONFIG_BOOTFILE		"uImage"
#define CONFIG_LOADADDR		0x42000000
#define CONFIG_SYS_LOAD_ADDR	CONFIG_LOADADDR
#define CONFIG_MISC_INIT_R

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT              "U-Boot > "
#define CONFIG_AUTO_COMPLETE

/* Extra Environment */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"autoload=no\0" \
	"uimage=/boot/uImage\0" \
	"script=/boot/boot.ub\0" \
	"usb_script=/tsinit.ub\0" \
	"nfsroot=/nfsroot/\0" \
	"nfsip=192.168.0.1\0" \
	"fdtaddr=0x41000000\0" \
	"bootpart=0:2\0" \
	"bootname=SD card\0" \
	"cmdline_append=rw rootwait console=null\0" \
	"boot_fdt=yes\0" \
	"ip_dyn=yes\0" \
	"emmcboot=" \
		"setenv bootpart 2:0;" \
		"setenv bootname eMMC;" \
		"setenv bootargs root=/dev/mmcblk2p2 ${cmdline_append};" \
		"run sdboot; \0" \
	"sdboot=echo Booting from the ${bootname} ...; " \
		"if load mmc ${bootpart} ${loadaddr} ${script}; " \
			"then echo Booting from custom ${script}; " \
			"source ${loadaddr}; " \
		"fi; " \
		"load mmc ${bootpart} ${loadaddr} ${uimage}; " \
		"if load mmc ${bootpart} ${fdtaddr} /boot/imx28-ts7680.dtb; then " \
			"echo Using device tree; " \
			"bootm ${loadaddr} - ${fdtaddr}; "\
		"else " \
			"echo Booting without device tree ; " \
			"bootm ${loadaddr};" \
		"fi; \0" \
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
		"env set serverip ${nfsip}; " \
		"nfs ${loadaddr} ${nfsroot}${uimage}; " \
		"setenv bootargs root=/dev/nfs ip=dhcp " \
		  "nfsroot=${serverip}:${nfsroot},vers=2,nolock ${cmdline_append}; " \
		"if nfs ${fdtaddr} ${nfsroot}/boot/imx28-ts7680.dtb; then " \
			"echo Using device tree; " \
			"bootm ${loadaddr} - ${fdtaddr}; "\
		"else " \
			"echo Booting without device tree ; " \
			"bootm ${loadaddr};" \
		"fi; \0" \


#define CONFIG_BOOTCOMMAND \
	"setenv bootargs root=/dev/mmcblk0p2 ${cmdline_append};" \
	"run usbprod; "\
	"run sdboot;"\
	

/* The rest of the configuration is shared */
#include <configs/mxs.h>

#endif /* __CONFIGS_TS7680_H__ */
