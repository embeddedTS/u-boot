/*
 * watchdog.c - driver for TS-7400-V2 Watchdog
 *
 * Licensed under the GPL-2 or later.
 */

#include <common.h>
#include <i2c.h>
#include <watchdog.h>
#include <status_led.h>

#ifndef CONFIG_SPL_BUILD
static int refcnt = 0;
static int wdtinit = 1;
static ulong lastfeed;

void reset_misc(void)
{
	/* timeout 0 == about 0.3 s,
	 * the smallest amount we can arm for */
	uint8_t val[2] = {0x0, 0x0};

	i2c_write(0x78, 0, 1, &val[0], 2);

	while(1);
}

void hw_watchdog_reset(void)
{
	uint8_t val[2] = {0x0, 0x2};

	/* I2C calls sleep which calls the wdt feeder.  Return immediately
	 * if it is already being called. */
	if(refcnt || !wdtinit) return;

	refcnt = 1;
	/* Our watchdog is over I2C, and U-Boot by default feeds the watchdog 
	 * a little too aggressively.  This will rate limit this to no more 
	 * than 1 feed per second */
	if(get_timer(lastfeed) > 1000){
		i2c_write(0x78, 0, 1, &val[0], 2);
		lastfeed = get_timer(0);
	}
	refcnt = 0;
}

void hw_watchdog_init(void)
{
	/* The TS-7400-V2 is compatible with the "old style" TS WDT feed, we 
	 * use that for simplicity sake.
	 * Using a value of 2 results in about 10 s timeout.
	 *
	 * mxs-i2c.c in this U-Boot prevents writing with an address length of
	 * 0. This means we set chip address properly, reg address to 0, with
	 * alen of 1, and then two bytes of data. This results in a 3 byte write
	 * to the uC which is a proper feed.
	 */
	uint8_t val[2] = {0x0, 0x2};
	i2c_set_bus_num(0);

	i2c_write(0x78, 0, 1, &val[0], 2);
	wdtinit = 1;
	lastfeed = get_timer(0);
}

#else
void hw_watchdog_reset(void){};
void hw_watchdog_init(void){};
#endif
