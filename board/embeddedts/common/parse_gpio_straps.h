#ifndef PARSE_GPIO_STRAPS_H
#define PARSE_GPIO_STRAPS_H

/* This can be used in either SPL or U-Boot proper, and some environments may
 * not have the full DM_GPIO available. In that case, those use the traditional
 * flat numbered GPIOs. The DM_GPIO variant uses GPIO labels.
 *
 * Both return up to 31 GPIO strap values, or -1 on any error.
 */
#if !CONFIG_IS_ENABLED(DM_GPIO)
s32 parse_gpio_straps(unsigned gpio[], size_t cnt);
#else
s32 parse_gpio_straps(const char *pins[], size_t cnt);
#endif

#endif // PARSE_GPIO_STRAPS_H
