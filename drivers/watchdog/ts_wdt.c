/*
 * i2c watchdog.
 * Copyright (C) 2015 Mark Featherston <mark@embeddedarm.com>
 * Technologic Systems
 *
 * Adapted for use in U-Boot by Stuart Longland <stuartl@vrt.com.au>
 * VRT Systems
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <config.h>
#include <i2c.h>
#include <watchdog.h>

#define TS_DEFAULT_TIMEOUT 30
#define TS_WATCHDOG_DISABLE 65535

#ifndef CONFIG_WATCHDOG_TIMEOUT_MSECS
#define CONFIG_WATCHDOG_TIMEOUT_MSECS (TS_DEFAULT_TIMEOUT * 1000)
#endif

#ifndef CONFIG_WATCHDOG_TSWDOG_BUS
#define CONFIG_WATCHDOG_TSWDOG_BUS (~0)
#endif

#ifndef CONFIG_WATCHDOG_TSWDOG_ADDR
#define CONFIG_WATCHDOG_TSWDOG_ADDR (~0)
#endif

/* The WDT expects 3 values:
 * 0 (always)
 * and two bytes for the feed length in deciseconds
 * 1 <MSB>
 * 2 <LSB>
 * there are also 3 special values if they are specified
 * in the LSB with a 0 MSB:
 * 0 - 200ms
 * 1 - 2s
 * 2 - 4s
 * 3 - 10s
 * 4 - disable watchdog
 */

static int ts_wdt_write(u16 deciseconds)
{
	u8 out[2];

	out[0] = (deciseconds & 0xff00) >> 8;
	out[1] = deciseconds & 0xff;

	debug("select I2C bus %d\n", CONFIG_WATCHDOG_TSWDOG_BUS);
	if (i2c_set_bus_num(CONFIG_WATCHDOG_TSWDOG_BUS))
		return -1;

	debug("set timeout %d 0.1sec\n", deciseconds);
	if (i2c_write(CONFIG_WATCHDOG_TSWDOG_ADDR, 0, 1, out, 2))
		return -1;

	debug("done\n");
	return 0;
}

#ifdef CONFIG_TECHNOLOGIC_WATCHDOG
void hw_watchdog_reset(void)
{
	/* Disable watchdog */
	debug("Resetting watchdog\n");
	ts_wdt_write(CONFIG_WATCHDOG_TIMEOUT_MSECS/100);
}

void hw_watchdog_init(void)
{
	/* Timeout is in .1 secs */
	debug("Initialising watchdog\n");
	ts_wdt_write(CONFIG_WATCHDOG_TIMEOUT_MSECS/100);
}
#endif
