/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 Technologic Systems, Inc. dba embeddedTS
 */

#ifndef PARSE_GPIO_STRAPS_H
#define PARSE_GPIO_STRAPS_H

/* This can be used in either SPL or U-Boot proper, and some environments may
 * not have the full DM_GPIO available. In that case, those use the traditional
 * flat numbered GPIOs. The DM_GPIO variant uses GPIO labels.
 *
 * Both return up to 31 GPIO strap values, or -1 on any error.
 */
#if !CONFIG_IS_ENABLED(DM_GPIO)
/* When DM_GPIO is unavailable, parse the array of arbitrary GPIO pins and
 * return their value.
 *
 * @param gpio Array of GPIO numbers, LSB first. That is, gpio[0] is BIT(0) in
 *             the return. Up to 31 total GPIOs can be used for this.
 *
 * @param cnt Count of the number of GPIOs to evaluate.
 *
 * @returns Negative on error, otherwise, a 31-bit field that was the value or
 *          the GPIO straps as they were evaluated. BIT(0) in the return value
 *          is the value of GPIO number gpio[0] that was passed.
 */
s32 parse_gpio_straps(unsigned int gpio[], size_t cnt);

#else
/* When DM_GPIO is available, parse the array of named GPIO pins and return
 * their value.
 *
 * @param pins Array of GPIO names, LSB first. That is, gpio[0] is BIT(0) in
 *             the return. Up to 31 total GPIOs can be used for this.
 *
 * @param cnt Count of the number of GPIOs to evaluate.
 *
 * @returns Negative on error, otherwise, a 31-bit field that was the value or
 *          the GPIO straps as they were evaluated. BIT(0) in the return value
 *          is the value of the GPIO named pins[0] that was passed.
 */
s32 parse_gpio_straps(const char * const pins[], size_t cnt);
#endif

#endif // PARSE_GPIO_STRAPS_H
