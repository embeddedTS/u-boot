/* Parses up to 31 CPU strapping pins */

#include <asm/gpio.h>
#include <linux/delay.h>
#include <stddef.h>

#if !CONFIG_IS_ENABLED(DM_GPIO)
s32 parse_gpio_straps(unsigned gpio[], size_t cnt)
{
	u32 ret = 0;
	size_t i;
	int rc;

	if (cnt > 31)
		return -EOVERFLOW;

	for (i = 0; i < cnt; i++) {
		if (gpio_request(gpio[i], "strap") < 0)
			return -1;
		if (gpio_direction_input(gpio[i]) < 0)
			return -1;
		udelay(1); // Let input settle
		rc = gpio_get_value(gpio[i]);
		if (rc < 0)
			return -1;
		ret <<= 1;
		ret |= !!rc;
		gpio_free(gpio[i]);
	}

	return ret;
}

#else

s32 parse_gpio_straps(const char *pins[], size_t cnt)
{
	u32 ret = 0;
	size_t i;
	struct gpio_desc desc;
	int rc;

	for (i = 0; i < cnt; i++) {
		if (gpio_request_by_line_name(NULL, pins[i], &desc,
				(GPIOD_IS_IN)) < 0)
			return -1;

		udelay(1); // Let input settle

		rc = dm_gpio_get_value(&desc);
		if (rc < 0)
			return -1;
		ret <<= 1;
		ret |= !!rc;
		dm_gpio_free(NULL, &desc);
	}

	return ret;
}

#endif // !CONFIG_IS_ENABLED(DM_GPIO)
