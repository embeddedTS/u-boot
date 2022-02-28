/*
 * Copyright (C) 2019 Technologic Systems
 *
 * TS-7670 config
 * Based on m28evk.h
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __CONFIGS_TS7670_H__
#define __CONFIGS_TS7670_H__

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

#define CONFIG_RED_LED			MX28_PAD_GPMI_CE1N__GPIO_0_17
#define CONFIG_GREEN_LED		MX28_PAD_GPMI_RESETN__GPIO_0_28
#define CONFIG_YEL_LED			MX28_PAD_LCD_RS__GPIO_1_26
#define CONFIG_BLUE_LED			MX28_PAD_LCD_RD_E__GPIO_1_24
#define STATUS_LED_RED			0
#define STATUS_LED_GREEN		1
#define STATUS_LED_YELLOW		2
#define STATUS_LED_BLUE			3

#define STATUS_LED_BIT                  STATUS_LED_RED
#define STATUS_LED_STATE                STATUS_LED_ON
#define STATUS_LED_PERIOD               (CONFIG_SYS_HZ / 2)

#define STATUS_LED_BIT1                 STATUS_LED_GREEN
#define STATUS_LED_STATE1               STATUS_LED_OFF
#define STATUS_LED_PERIOD1              (CONFIG_SYS_HZ / 2)

#define STATUS_LED_BIT2                 STATUS_LED_YELLOW
#define STATUS_LED_STATE2               STATUS_LED_OFF
#define STATUS_LED_PERIOD2              (CONFIG_SYS_HZ / 2)

#define STATUS_LED_BIT3                 STATUS_LED_BLUE
#define STATUS_LED_STATE3               STATUS_LED_OFF
#define STATUS_LED_PERIOD3              (CONFIG_SYS_HZ / 2)

#define CONFIG_CMD_CACHE
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_GPIO
#define CONFIG_CMD_MII
#define CONFIG_CMD_MMC
#define CONFIG_CMD_FAT
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EXT3
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_EXT4_WRITE
#define CONFIG_CMD_FS_GENERIC
#define CONFIG_CMD_NET
#define CONFIG_CMD_NFS
#define CONFIG_CMD_PING
#define CONFIG_CMD_SAVEENV
#define CONFIG_CMD_SETEXPR
#define CONFIG_CMD_BOOTZ
#define CONFIG_CMD_USB
#define CONFIG_CMD_TIME
#define CONFIG_LIB_RAND
#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ
#define CONFIG_SUPPORT_RAW_INITRD

/* Memory configuration */
#define CONFIG_SYS_ALT_MEMTEST
#define CONFIG_CMD_MEMTEST

#define CONFIG_NR_DRAM_BANKS		1		/* 1 bank of DRAM */
#define PHYS_SDRAM_1			0x40000000	/* Base address */
#define PHYS_SDRAM_1_SIZE		0x40000000	/* Max 1 GB RAM */
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1

/* Env is at the 1MB boundary in emmc boot partition 0 */
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_SYS_MMC_ENV_DEV		1 /* mmcblk0 */
#define CONFIG_SYS_MMC_ENV_PART		1 /* boot0 */

#define CONFIG_ENV_OFFSET		0x400000 /* 4MiB */
#define CONFIG_ENV_SIZE			(128 * 1024) /* 128k */
#define CONFIG_ENV_OFFSET_REDUND	0x600000 /* 6MiB */
#define CONFIG_ENV_OVERWRITE

#define CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG

/* FEC Ethernet on SoC */
#ifdef	CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_NET_MULTI
#define CONFIG_MX28_FEC_MAC_IN_OCOTP
#endif

/* USB */
#ifdef	CONFIG_CMD_USB
#define CONFIG_EHCI_MXS_PORT1
#define CONFIG_USB_MAX_CONTROLLER_COUNT	1
#define	CONFIG_USB_STORAGE
#endif

/* I2C */
#define CONFIG_CMD_I2C
#define CONFIG_I2C_MXS
#define CONFIG_SYS_I2C_SPEED		100000 

#define CONFIG_LOADADDR			0x42000000
#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_MISC_INIT_R

/* Watchdog support */
#define CONFIG_TS7400V2_WATCHDOG
#define CONFIG_HW_WATCHDOG
#define CONFIG_SPL_WATCHDOG_SUPPORT


/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT		"U-Boot > "
#define CONFIG_AUTO_COMPLETE

#define CONFIG_BOOTDELAY		1
#define CONFIG_AUTOBOOT_KEYED		1
#define CONFIG_AUTOBOOT_PROMPT         "Press Ctrl+C to abort autoboot in %d second(s)\n", bootdelay
#define CTRL(c) ((c)&0x1F)     
#define CONFIG_AUTOBOOT_STOP_STR       (char []){CTRL('C'), 0}

/*#define CONFIG_PREBOOT \*/

/* Extra Environment */
#define CONFIG_EXTRA_ENV_SETTINGS \
	"autoload=no\0" \
	"nfsroot=/nfsroot/\0" \
	"nfsip=192.168.0.1\0" \
	"fdtaddr=0x41000000\0" \
	"cmdline_append=rw rootwait console=ttyAMA0,115200 loglevel=3\0" \
	"boot_fdt=yes\0" \
	"ip_dyn=yes\0" \
        "clearenv=mmc dev 1 1; mmc erase 2000 400; mmc erase 3000 400;\0" \
	"emmcboot=echo Booting from the onboard eMMC  ...; " \
		"if load mmc 1:2 ${loadaddr} /boot/boot.ub; " \
			"then echo Booting from custom /boot/boot.ub; " \
			"source ${loadaddr}; " \
		"fi; " \
		"load mmc 1:2 ${loadaddr} /boot/uImage; " \
		"load mmc 1:2 ${fdtaddr} /boot/imx28-ts${model}.dtb; " \
		"setenv bootargs root=/dev/mmcblk1p2 ${cmdline_append}; " \
		"bootm ${loadaddr} - ${fdtaddr}; \0"\
	"sdboot=echo Booting from the SD Card ...; " \
		"if load mmc 0:2 ${loadaddr} /boot/boot.ub; " \
			"then echo Booting from custom /boot/boot.ub; " \
			"source ${loadaddr}; " \
		"fi; " \
		"load mmc 0:2 ${loadaddr} /boot/uImage; " \
		"load mmc 0:2 ${fdtaddr} /boot/imx28-ts${model}.dtb; " \
		"setenv bootargs root=/dev/mmcblk0p2 ${cmdline_append}; " \
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
		"nfs ${fdtaddr} ${nfsroot}/boot/imx28-ts${model}.dtb; " \
		"bootm ${loadaddr} - ${fdtaddr};\0"\

#define CONFIG_BOOTCOMMAND \
	"run usbprod; " \
	"if test ${jpsdboot} = 'on' ; " \
		"then run sdboot; " \
		"else run emmcboot; " \
	"fi;"
	
/* The rest of the configuration is shared */
#include <configs/mxs.h>

#endif /* __CONFIGS_TS7670_H__ */
