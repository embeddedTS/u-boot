#ifndef RTC_WORKAROUND_H
#define RTC_WORKAROUND_H

#include <stddef.h>

struct rtc_gpio {
	unsigned gpio;
	int disable;
};

int rtc_drain(struct rtc_gpio *rtc_gpio, size_t cnt);

int rtc_enable(struct rtc_gpio rtc_gpio, unsigned long delay);

#endif // RTC_WORKAROUND_H
