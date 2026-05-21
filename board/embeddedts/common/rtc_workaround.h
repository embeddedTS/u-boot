/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 Technologic Systems, Inc. dba embeddedTS
 */

#ifndef RTC_WORKAROUND_H
#define RTC_WORKAROUND_H

#include <stddef.h>

struct rtc_gpio {
	unsigned int gpio;
	int disable;
};

/* Drain the RTCs I2C lines by disabling system power to the RTC and driving the
 * I2C lines low as GPIO pins to help reset the RTC.
 *
 * This function is specific to the ISL12022 RTC which can have glitchy startup
 * issues. This process fully ensures any glitches that may have appeared on the
 * I2C lines cause a full internal reset of the RTC on system power and the I2C
 * interface.
 *
 * @param rtc_gpio Pointer to the rtc_gpio struct. This is non DM_GPIO numbers.
 *                 The first GPIO in this list must be the GPIO used to enable
 *                 the RTC. The disable member of the struct is the state to set
 *                 the corresponding GPIO to in order to disable it.
 *
 * @param cnt Count of members of the rtc_gpio structure. This will usually be 3,
 *            but it is passed here for completeness.
 *
 * @returns 0 on success, -1 on any error.
 */
int rtc_drain(struct rtc_gpio *rtc_gpio, size_t cnt);

/* Re-enable the RTC.
 *
 * This function should be run some time after rtc_drain() to re-enable the RTC.
 *
 * @note Before calling this function, the I2C lines should be re-set to inputs.
 *       The amount of time
 *
 * @param rtc_gpio The single struct member for the RTC power enable line. It
 *                 will be set to the !disabled value after the pre_delay time
 *                 and the function will wait an additional post_delay time.
 *
 * @param pre_delay Delay in us to wait before re-enabling the RTC power. The
 *                  pre-delay may not always be needed so long as enough time
 *                  passes in real time before calling this function to ensure
 *                  the RTC system power is disabled and the I2C lines have been
 *                  held low long enough to bleed out any residual power.
 *
 * @param post_delay Delay in us to wait after re-enabling the RTC power. See
 *                   comments in the function for more information on this value.
 *
 * @returns 0 on success, -1 on any error.
 */
int rtc_enable(struct rtc_gpio rtc_gpio, unsigned long pre_delay, unsigned long post_delay);

#endif // RTC_WORKAROUND_H
