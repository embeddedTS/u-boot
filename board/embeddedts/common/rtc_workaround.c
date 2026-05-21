// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Technologic Systems, Inc. dba embeddedTS
 */

#include <asm/gpio.h>
#include <linux/delay.h>
#include <stddef.h>

#include "rtc_workaround.h"

int rtc_drain(struct rtc_gpio *rtc_gpio, size_t cnt)
{
	size_t i;

	for (i = 0; i < cnt; i++) {
		if (gpio_request(rtc_gpio[i].gpio, "rtc") < 0)
			return -1;
		if (gpio_direction_output(rtc_gpio[i].gpio, rtc_gpio[i].disable) < 0)
			return -1;
		/* While it is not a problem at the moment, we don't want to
		 * free the GPIO at this point since, eventually, if this goes
		 * the way of libgpiod, it could mean that the GPIO may go back
		 * to some default state which we don't want.
		 */
	}

	return 0;
}

int rtc_enable(struct rtc_gpio rtc_gpio, unsigned long pre_delay, unsigned long post_delay)
{
	/* On platforms with a separate RTC enable and a separate 3.3 V rail,
	 * it has been observed that if this rail has any power in it, it
	 * can take some time to collapse fully. On a TS-4900, this was
	 * observed to be about 915 ms in total. This delay can be used to
	 * adjust any additional time needed.
	 *
	 * There does not appear to be any datasheet requirement for VDD to
	 * be drained for an amount of time before bringing the rail back up.
	 */
	if (pre_delay > 0)
		udelay(pre_delay);

	/* We assume the GPIO passed here was already requested */
	if (gpio_direction_output(rtc_gpio.gpio, !rtc_gpio.disable) < 0)
		return -1;

	/* Wait some time for the I2C lines to fully power on once the 3.3 V
	 * rail was enabled. On a TS-4900, this was observed to be about
	 * 550 us.
	 *
	 * Note that AN1549 hints that the RTC may not respond for 90 ms, or as
	 * much as 3 s, on I2C after power has fully been enabled.
	 */
	if (post_delay > 0)
		udelay(post_delay);

	return 0;
}
