/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __TS4900_FPGA_RECOVERY_H__
#define __TS4900_FPGA_RECOVERY_H__

#include <linux/errno.h>

#if IS_ENABLED(CONFIG_TS4900_SDP_FPGA_RECOVERY)
int ts4900_sdp_fpga_recovery(void);
#else
static inline int ts4900_sdp_fpga_recovery(void)
{
	return -ENOSYS;
}
#endif

#endif
