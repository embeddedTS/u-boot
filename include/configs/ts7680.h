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
#define CONFIG_CMD_NAND
#define CONFIG_CMD_SF
#define CONFIG_CMD_SPI
#define CONFIG_CMD_USB
#define CONFIG_LIB_RAND
/*#define CONFIG_CMD_NAND_TRIMFFS*/

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

/* Environment is in NAND */
#if defined(CONFIG_CMD_NAND) && defined(CONFIG_ENV_IS_IN_NAND)
#define CONFIG_ENV_SIZE_REDUND		CONFIG_ENV_SIZE
#define CONFIG_ENV_SECT_SIZE		(128 * 1024)
#define CONFIG_ENV_RANGE		(512 * 1024)
#define CONFIG_ENV_OFFSET		0x300000
#define CONFIG_ENV_OFFSET_REDUND	\
		(CONFIG_ENV_OFFSET + CONFIG_ENV_RANGE)
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

/* UBI and NAND partitioning */
#ifdef CONFIG_CMD_NAND
#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS
#define CONFIG_CMD_MTDPARTS
#define CONFIG_RBTREE
#define CONFIG_LZO
#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS
#define MTDIDS_DEFAULT			"nand0=gpmi-nand"
#define MTDPARTS_DEFAULT			\
	"mtdparts=gpmi-nand:"			\
		"3m(bootloader)ro,"		\
		"512k(environment),"		\
		"512k(redundant-environment),"	\
		"4m(kernel),"			\
		"1m(fdt),"			\
		"8m(ramdisk),"			\
		"3m(reserved),"			\
		"20m(tsconfig),"		\
		"-(filesystem)"
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
#define CONFIG_SF_DEFAULT_BUS		2
#define CONFIG_SF_DEFAULT_CS		0
/* this may vary and depends on the installed chip */
#define CONFIG_SPI_FLASH_SST
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
	"fpga_bitstream=/boot/ts7680-fpga.bin\0" \
	"fdtaddr=0x41000000\0" \
	"bootpart=0:2\0" \
	"bootname=SD card\0" \
	"cmdline_append=rw rootwait console=null\0" \
	"update_nand_full_file=/boot/u-boot.nand\0" \
	"update_nand_firmware_file=/boot/u-boot.nand\0"	\
	"update_nand_firmware_maxsz=0x100000\0"	\
	"update_nand_stride=0x40\0"	/* MX28 datasheet ch. 12.12 */ \
	"update_nand_count=0x4\0"	/* MX28 datasheet ch. 12.12 */ \
	"update_nand_get_fcb_size="	/* Get size of FCB blocks */ \
		"nand device 0 ; " \
		"nand info ; " \
		"setexpr fcb_sz ${update_nand_stride} * ${update_nand_count};" \
		"setexpr update_nand_fcb ${fcb_sz} * ${nand_writesize}\0" \
	"update_nand_firmware_full=" /* Update FCB, DBBT and FW */ \
		"if load mmc 0:2 ${loadaddr} ${update_nand_full_file}; then " \
		"run update_nand_get_fcb_size ; " \
		"nand scrub -y 0x0 ${filesize} ; " \
		"nand write.raw ${loadaddr} 0x0 ${fcb_sz} ; " \
		"setexpr update_off ${loadaddr} + ${update_nand_fcb} ; " \
		"setexpr update_sz ${filesize} - ${update_nand_fcb} ; " \
		"nand write ${update_off} ${update_nand_fcb} ${update_sz} ; " \
		"fi\0" \
	"update_nand_firmware="		/* Update only firmware */ \
		"if load mmc 0:2 ${loadaddr} ${update_nand_firmware_file}; then " \
		"run update_nand_get_fcb_size ; " \
		"setexpr fcb_sz ${update_nand_fcb} * 2 ; " /* FCB + DBBT */ \
		"setexpr fw_sz ${update_nand_firmware_maxsz} * 2 ; " \
		"setexpr fw_off ${fcb_sz} + ${update_nand_firmware_maxsz};" \
		"nand erase ${fcb_sz} ${fw_sz} ; " \
		"nand write ${loadaddr} ${fcb_sz} ${filesize} ; " \
		"nand write ${loadaddr} ${fw_off} ${filesize} ; " \
		"fi\0" \
	"update_nand_kernel="		/* Update kernel */ \
		"if load mmc 0:2 ${loadaddr} ${uimage}; then " \
		"mtdparts default; " \
		"nand erase.part kernel; " \
		"nand write ${loadaddr} kernel ${filesize}; " \
		"fi\0" \
	"update_nand_fdt="		/* Update fdt */ \
		"if load mmc 0:2 ${loadaddr} /boot/imx28-ts7680.dtb; then " \
		"mtdparts default; " \
		"nand erase.part fdt; " \
		"nand write ${loadaddr} fdt ${filesize}" \
		"fi\0" \
	"nandboot=echo Booting from NAND...;" /* Boot from NAND */ \
		"mtdparts default; " \
		"ubi part tsconfig ;" \
		"ubifsmount ubi0:tsconfig ;"\
		"if ubifsload ${loadaddr} /boot/boot.ub; " \
			"then source ${loadaddr}; " \
		"fi; " \
		"ubifsload ${loadaddr} ${fpga_bitstream} ; " \
		"ice40 ${loadaddr} ${filesize}; " \
		"nand read ${loadaddr} kernel 0x00400000; " \
		"if test ${boot_fdt} = yes; then " \
			"nand read ${fdtaddr} fdt 0x00080000; " \
			"bootm ${loadaddr} - ${fdtaddr}; " \
		"else " \
			"if test ${boot_fdt} = no; then " \
				"bootm; " \
			"else " \
				"echo \"ERROR: Set boot_fdt to yes or no.\"; " \
			"fi; " \
		"fi\0" \
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
		"load mmc ${bootpart} ${loadaddr} ${fpga_bitstream} ; " \
		"ice40 ${loadaddr} ${filesize}; " \
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
		"load mmc ${bootpart} ${loadaddr} ${fpga_bitstream} ; " \
		"ice40 ${loadaddr} ${filesize}; " \
		"dhcp ; " \
		"env set serverip ${nfsip}; " \
		"nfs ${loadaddr} ${nfsroot}${fpga_bitstream}; " \
		"ice40 ${loadaddr} ${filesize}; " \
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
	"if test ${bootmode} = 4; "\
		"then run nandboot; "\
 		"else run sdboot; "\
	"fi ;"
	

/* The rest of the configuration is shared */
#include <configs/mxs.h>

#endif /* __CONFIGS_TS7680_H__ */
