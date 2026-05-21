// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Technologic Systems, Inc. dba embeddedTS
 */

#ifndef RTC_WORKAROUND_H
#define RTC_WORKAROUND_H

#include <stddef.h>

struct rtc_gpio {
	unsigned gpio;
	int disable;
};

int rtc_drain(struct rtc_gpio *rtc_gpio, size_t cnt);

int rtc_enable(struct rtc_gpio rtc_gpio, unsigned long pre_delay, unsigned long post_delay);

#endif // RTC_WORKAROUND_H
