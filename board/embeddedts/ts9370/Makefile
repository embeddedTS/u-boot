#
# Copyright 2023-2024 embeddedTS
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += ts9370.o
obj-y += ../../../board/freescale/common/

ifdef CONFIG_XPL_BUILD
obj-y += spl.o
obj-y += lpddr4x-16gb.o
obj-y += lpddr4x-8gb.o
else
obj-y += ../common/ts-macs.o
endif

obj-y += ../common/fpga_bootloader.o ../common/wizard.o
