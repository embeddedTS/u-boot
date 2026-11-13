/* SPDX-License-Identifier: GPL-2.0+ */

#ifndef __TS_IMX6UL_ENV_H
#define __TS_IMX6UL_ENV_H

#define TS_IMX6UL_NFSBOOT_ENV \
	"nfsboot=setenv autoload no; dhcp; " \
	"wget ${fdt_addr_r} ${nfsroot}/boot/${fdtfile}; " \
	"wget ${kernel_addr_r} ${nfsroot}/boot/zImage; " \
	"setenv bootargs root=/dev/nfs rw ip=dhcp " \
		"nfsroot=${nfsroot}${nfsroot_options} console=${console}; " \
	"fdt addr ${fdt_addr_r}; " \
	"bootz ${kernel_addr_r} - ${fdt_addr_r}\0"

#endif /* __TS_IMX6UL_ENV_H */
