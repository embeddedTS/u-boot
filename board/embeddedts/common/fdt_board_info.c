// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Technologic Systems, Inc. dba embeddedTS
 */

#include <fdt_support.h>
#include <stdio.h>
#include "fdt_board_info.h"

#define VENDOR "technologic"


static int set_fdt_u32(void *fdt, u32 val, const char *name)
{
	int root_node = fdt_path_offset(fdt, "/");
	int ret;

	if (root_node < 0) {
		printf("Failed to find / node");
		return root_node;
	}

	ret = fdt_setprop_u32(fdt, root_node, name, val);
	if (ret < 0)
		printf("Failed to set property %s to %d\n", name, val);

	return ret;
}

static int set_fdt_string(void *fdt, const char *string, const char *name)
{
	int root_node = fdt_path_offset(fdt, "/");
	int ret;

	if (root_node < 0) {
		printf("Failed to find / node");
		return root_node;
	}

	ret = fdt_setprop_string(fdt, root_node, name, string);
	if (ret < 0)
		printf("Failed to set property %s to %s\n", name, string);

	return ret;
}

int ets_fdt_set_bom_options(void *fdt, u32 val)
{
	return set_fdt_u32(fdt, val, VENDOR",bom-options");
}

int ets_fdt_set_io_options(void *fdt, u32 val)
{
	return set_fdt_u32(fdt, val, VENDOR",io-options");
}

int ets_fdt_set_cpu_options(void *fdt, u32 val)
{
	return set_fdt_u32(fdt, val, VENDOR",cpu-options");
}

int ets_fdt_set_io_model(void *fdt, u32 val)
{
	return set_fdt_u32(fdt, val, VENDOR",io-model");
}

int ets_fdt_set_pcb_revision(void *fdt, const char *string)
{
	return set_fdt_string(fdt, string, VENDOR",pcb-revision");
}

int ets_fdt_set_bom_revision(void *fdt, const char *string)
{
	return set_fdt_string(fdt, string, VENDOR",bom-revision");
}

int ets_fdt_set_ram_timing(void *fdt, u32 val)
{
	return set_fdt_u32(fdt, val, VENDOR",ram-timing");
}

int ets_fdt_set_bbid(void *fdt, u32 val)
{
	return set_fdt_u32(fdt, val, VENDOR",baseboard-id");
}

int ets_fdt_set_bbrev(void *fdt, u32 val)
{
	return set_fdt_u32(fdt, val, VENDOR",baseboard-revision");
}
