# SPDX-License-Identifier: GPL-2.0+
# Copyright (C) 2020-2022 embeddedTS

obj-y := ts7250v3.o led.o

ifdef CONFIG_XPL_BUILD
obj-y += spl.o
endif

ifndef CONFIG_SPL_BUILD
obj-y += mfg.o super.o
endif
