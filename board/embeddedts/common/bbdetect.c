/* Generic bbdetect mechanism using embeddedTS MUX on baseboard with 3 bits of
 * output selection, and then output.
 */

#include <asm/gpio.h>
#include <env.h>
#include <linux/delay.h>
#include <stddef.h>

#include "bbdetect.h"

#if !CONFIG_IS_ENABLED(DM_GPIO)
#error "The bbdetect functions require DM_GPIO!"
#endif

int bbdetect(const struct bbdetect_pins pins, unsigned long delay)
{
	struct gpio_desc desc_bit[BBID_MUX_BITS];
	struct gpio_desc desc_in;
	int i, bit;
	u8 bbid = 0;
	u8 id, rev;
	int rc;

	/* Request pins, setting outputs low */
	for (i = 0; i < BBID_MUX_BITS; i++) {
		if (gpio_request_by_line_name(NULL, pins.bit[i], &desc_bit[i],
				(GPIOD_IS_OUT)) < 0)
			return -1;
	}
	if (gpio_request_by_line_name(NULL, pins.in, &desc_in,
			(GPIOD_IS_IN)) < 0)
		return -1;

	/* Shift in data */
	for (i = 0; i < 8; i++) {
		for (bit = 0; bit < BBID_MUX_BITS; bit++) {
			if (dm_gpio_set_value(&desc_bit[bit], !!(i & (BIT(bit)))) < 0)
				return -1;
		}

		udelay(delay);

		rc = dm_gpio_get_value(&desc_in);
		if (rc < 0)
			return -1;

		if (rc > 0)
			bbid |= BIT(i);
	}

	/* Parse out strap segments from whole bbid */
	id = (bbid & 0x3f);
	rev = (bbid & 0xc0) >> 6;

	env_set_hex("baseboard_id", id);
	env_set_hex("baseboard_rev", rev);

	/* Release pins */
	dm_gpio_free(NULL, &desc_in);
	for (i = 0; i < BBID_MUX_BITS; i++)
		dm_gpio_free(NULL, &desc_bit[i]);

	return 0;
}
