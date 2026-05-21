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

int rtc_enable(struct rtc_gpio rtc_gpio, unsigned long delay)
{
	if (delay > 0)
		udelay(delay);

	/* We assume the GPIO passed here was already requested */
	if (gpio_direction_output(rtc_gpio.gpio, !rtc_gpio.disable) < 0)
		return -1;

	return 0;
}
