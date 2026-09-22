// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Technologic Systems, Inc. dba embeddedTS
 */

#ifndef FDT_BOARD_INFO_H
#define FDT_BOARD_INFO_H

int ets_fdt_set_bom_options(void *fdt, u32 val);

int ets_fdt_set_io_options(void *fdt, u32 val);

int ets_fdt_set_cpu_options(void *fdt, u32 val);

int ets_fdt_set_io_model(void *fdt, u32 val);

int ets_fdt_set_pcb_revision(void *fdt, const char *string);

int ets_fdt_set_bom_revision(void *fdt, const char *string);

int ets_fdt_set_ram_timing(void *fdt, u32 val);

int ets_fdt_set_bbid(void *fdt, u32 val);

int ets_fdt_set_bbrev(void *fdt, u32 val);

#endif // FDT_BOARD_INFO_H
