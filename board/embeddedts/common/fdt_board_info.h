/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 Technologic Systems, Inc. dba embeddedTS
 */

#ifndef FDT_BOARD_INFO_H
#define FDT_BOARD_INFO_H

/* Modify the live FDT to include the "bom-options" property with the specified
 * value.
 *
 * @param fdt Pointer to the FTD blob
 *
 * @param val Value to set to the property
 *
 * @returns 0 on success, negative error value otherwise.
 */
int ets_fdt_set_bom_options(void *fdt, u32 val);

/* Modify the live FDT to include the "io-options" property with the specified
 * value.
 *
 * @param fdt Pointer to the FTD blob
 *
 * @param val Value to set to the property
 *
 * @returns 0 on success, negative error value otherwise.
 */
int ets_fdt_set_io_options(void *fdt, u32 val);

/* Modify the live FDT to include the "cpu-options" property with the specified
 * value.
 *
 * @param fdt Pointer to the FTD blob
 *
 * @param val Value to set to the property
 *
 * @returns 0 on success, negative error value otherwise.
 */
int ets_fdt_set_cpu_options(void *fdt, u32 val);

/* Modify the live FDT to include the "io-model" property with the specified
 * value.
 *
 * @param fdt Pointer to the FTD blob
 *
 * @param val Value to set to the property
 *
 * @returns 0 on success, negative error value otherwise.
 */
int ets_fdt_set_io_model(void *fdt, u32 val);

/* Modify the live FDT to include the "pcb-revision" property with the specified
 * value.
 *
 * @param fdt Pointer to the FTD blob
 *
 * @param val Value to set to the property
 *
 * @returns 0 on success, negative error value otherwise.
 */
int ets_fdt_set_pcb_revision(void *fdt, const char *string);

/* Modify the live FDT to include the "bom-revision" property with the specified
 * value.
 *
 * @param fdt Pointer to the FTD blob
 *
 * @param val Value to set to the property
 *
 * @returns 0 on success, negative error value otherwise.
 */
int ets_fdt_set_bom_revision(void *fdt, const char *string);

/* Modify the live FDT to include the "ram-timing" property with the specified
 * value.
 *
 * @param fdt Pointer to the FTD blob
 *
 * @param val Value to set to the property
 *
 * @returns 0 on success, negative error value otherwise.
 */
int ets_fdt_set_ram_timing(void *fdt, u32 val);

/* Modify the live FDT to include the "baseboard-id" property with the specified
 * value.
 *
 * @param fdt Pointer to the FTD blob
 *
 * @param val Value to set to the property
 *
 * @returns 0 on success, negative error value otherwise.
 */
int ets_fdt_set_bbid(void *fdt, u32 val);

/* Modify the live FDT to include the "baseboard-revision" property with the specified
 * value.
 *
 * @param fdt Pointer to the FTD blob
 *
 * @param val Value to set to the property
 *
 * @returns 0 on success, negative error value otherwise.
 */
int ets_fdt_set_bbrev(void *fdt, u32 val);

#endif // FDT_BOARD_INFO_H
