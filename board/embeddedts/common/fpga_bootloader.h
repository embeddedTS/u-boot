/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef FPGA_BOOTLOADER_H
#define FPGA_BOOTLOADER_H

/* These updater regs are only available while in the bootloader before we have booted
 * to the application load
 */
#define UPDATER_BASE				(FPGA_BASE + 0x100)

/* Status Register (0x0) */
#define UPDATER_STATUS				(UPDATER_BASE + 0x0)
#define UPDATER_STATUS_ERASE_SUCCESS		BIT(4)
#define UPDATER_STATUS_WRITE_SUCCESS		BIT(3)
#define UPDATER_STATUS_READ_SUCCESS		BIT(2)
#define UPDATER_STATUS_BUSY_MASK		GENMASK(1, 0)
#define UPDATER_STATUS_BUSY_IDLE		FIELD_PREP(UPDATER_STATUS_BUSY_SHIFT, 0)
#define UPDATER_STATUS_BUSY_ERASE		FIELD_PREP(UPDATER_STATUS_BUSY_SHIFT, 1)
#define UPDATER_STATUS_BUSY_WRITE		FIELD_PREP(UPDATER_STATUS_BUSY_SHIFT, 2)
#define UPDATER_STATUS_BUSY_READ		FIELD_PREP(UPDATER_STATUS_BUSY_SHIFT, 3)
#define UPDATER_STATUS_SECTOR1_PROTECTED	BIT(5)
#define UPDATER_STATUS_SECTOR2_PROTECTED	BIT(6)
#define UPDATER_STATUS_SECTOR3_PROTECTED	BIT(7)
#define UPDATER_STATUS_SECTOR4_PROTECTED	BIT(8)
#define UPDATER_STATUS_SECTOR5_PROTECTED	BIT(9)

/* Control Register (0x4) */
#define UPDATER_CTRL				(UPDATER_BASE + 0x4)
#define UPDATER_CTRL_START_READ			BIT(31)
#define UPDATER_CTRL_SECTOR1_WP			BIT(23)
#define UPDATER_CTRL_SECTOR2_WP			BIT(24)
#define UPDATER_CTRL_SECTOR3_WP			BIT(25)
#define UPDATER_CTRL_SECTOR4_WP			BIT(26)
#define UPDATER_CTRL_SECTOR5_WP			BIT(27)
#define UPDATER_CTRL_SECTOR_ERASE_MASK		GENMASK(22, 20)
#define UPDATER_CTRL_SECTOR1_ERASE		FIELD_PREP(UPDATER_CTRL_SECTOR_ERASE_MASK, 1)
#define UPDATER_CTRL_SECTOR2_ERASE		FIELD_PREP(UPDATER_CTRL_SECTOR_ERASE_MASK, 2)
#define UPDATER_CTRL_SECTOR3_ERASE		FIELD_PREP(UPDATER_CTRL_SECTOR_ERASE_MASK, 3)
#define UPDATER_CTRL_SECTOR4_ERASE		FIELD_PREP(UPDATER_CTRL_SECTOR_ERASE_MASK, 4)
#define UPDATER_CTRL_SECTOR5_ERASE		FIELD_PREP(UPDATER_CTRL_SECTOR_ERASE_MASK, 5)
#define UPDATER_CTRL_PAGE_ERASE_MASK		GENMASK(19, 0)

/* Flash Address Register (0x8) */
#define UPDATER_ADDR				(UPDATER_BASE + 0x8)

/* Flash Data Register (0xC) */
#define UPDATER_FLASHDATA			(UPDATER_BASE + 0xC)

/* Reconfiguration Register (0x10) */
#define UPDATER_RECONFIG			(UPDATER_BASE + 0x10)
#define UPDATER_RECONFIG_EN_ERROR_LEDS		BIT(2)
#define UPDATER_RECONFIG_IMAGE_SELECT		BIT(1)
#define UPDATER_RECONFIG_RECONFIG_START		BIT(0)

/*
 * Sectors IDs and addresses
 * 1: 0x00000 - 0x03FFF UFM
 * 2: 0x04000 - 0x07FFF UFM
 * 3: 0x08000 - 0x1CFFF CFM (Image 2)
 * 4: 0x1C800 - 0x2AFFF CFM (Image 2)
 * 5: 0x2B000 - 0x4DFFF CFM (Image 1)
 */
#define CFM0_BASE		0x2B000
#define CFM1_BASE		0x08000
#define CFM_SIZE		0x23000
#define WORD_ADDRESS(val)	((val) >> 2)

/* Model/info core, this is present in both the bootloader/application */
#define FPGA_BASE		0x28000000
#define FPGA_MODEL		(FPGA_BASE + 0x0)
#define FPGA_TAG_VERSION	(FPGA_BASE + 0x4)
#define FPGA_HASH		(FPGA_BASE + 0x8)
#define FPGA_SCRATCH0		(FPGA_BASE + 0x10)
#define FPGA_SCRATCH1		(FPGA_BASE + 0x14)

int flash_wait_until_idle(u32 timeout_ms, u32 *reg);
int flash_write(u32 flash_addr, u32 data_addr, u32 len);
int flash_read(u32 flash_addr, u32 data_addr, u32 len);
int flash_sector_erase(uint8_t sector);
int flash_update_app(u32 addr, u32 len);
int flash_read_app(u32 addr, u32 len);
int flash_update_bootloader(u32 addr, u32 len);
int flash_read_bootloader(u32 addr, u32 len);
int fpga_update_from_flash(void);
int fpga_reconfig(void);
void print_fpga_version(void);

#endif // FPGA_BOOTLOADER_H
