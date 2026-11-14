# SPDX-License-Identifier: GPL-2.0+
#
# Copyright (C) 2026 Technologic Systems, Inc. dba embeddedTS

ifdef CONFIG_XPL_BUILD
obj-y := spl.o ice40.o strap_decode.o wdt_reset.o ../common/parse_gpio_straps.o ../common/rtc_workaround.o
else
obj-y  := ts4900.o strap_decode.o ../common/parse_gpio_straps.o ../common/bbdetect.o ../common/fdt_board_info.o ../common/bbextension.o
endif
