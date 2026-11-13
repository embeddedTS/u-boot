// SPDX-License-Identifier: GPL-2.0+
/*
 * embeddedTS supervisory microcontroller watchdog
 *
 * The microcontroller accepts a three-byte I2C command: 0 followed by the
 * timeout in deciseconds.  A timeout of 0 requests its minimum 400 ms period.
 */

#include <dm.h>
#include <i2c.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <time.h>
#include <wdt.h>

struct ts_micro_wdt_priv {
	u16 timeout_ds;
	ulong lastfeed;
	int refcnt;
};

static int ts_micro_wdt_feed(struct udevice *dev, u16 timeout_ds,
			     bool rate_limit)
{
	struct ts_micro_wdt_priv *priv = dev_get_priv(dev);
	struct i2c_msg msg;
	u8 data[] = { 0, timeout_ds >> 8, timeout_ds & 0xff };
	ulong now;
	int ret;

	/* I2C transfers can sleep, which may invoke the watchdog feeder again. */
	if (priv->refcnt)
		return 0;

	now = get_timer(0);
	if (rate_limit && get_timer(priv->lastfeed) < 1000)
		return 0;

	msg.addr = dev_read_addr(dev);
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;
	priv->refcnt++;
	ret = dm_i2c_xfer(dev, &msg, 1);
	priv->refcnt--;
	if (!ret)
		priv->lastfeed = now;

	return ret;
}

static int ts_micro_wdt_start(struct udevice *dev, u64 timeout_ms, ulong flags)
{
	struct ts_micro_wdt_priv *priv = dev_get_priv(dev);
	u64 timeout_ds = DIV_ROUND_UP(timeout_ms, 100);

	if (!timeout_ds || timeout_ds > 0xffff)
		return -EINVAL;

	priv->timeout_ds = timeout_ds;

	return ts_micro_wdt_feed(dev, priv->timeout_ds, false);
}

static int ts_micro_wdt_reset(struct udevice *dev)
{
	struct ts_micro_wdt_priv *priv = dev_get_priv(dev);

	return ts_micro_wdt_feed(dev, priv->timeout_ds, true);
}

static int ts_micro_wdt_stop(struct udevice *dev)
{
	return ts_micro_wdt_feed(dev, 3, false);
}

static int ts_micro_wdt_expire_now(struct udevice *dev, ulong flags)
{
	return ts_micro_wdt_feed(dev, 0, false);
}

static const struct wdt_ops ts_micro_wdt_ops = {
	.start = ts_micro_wdt_start,
	.stop = ts_micro_wdt_stop,
	.reset = ts_micro_wdt_reset,
	.expire_now = ts_micro_wdt_expire_now,
};

static const struct udevice_id ts_micro_wdt_ids[] = {
	{ .compatible = "ts-wdt" },
	{ }
};

U_BOOT_DRIVER(ts_micro_wdt) = {
	.name = "ts_micro_wdt",
	.id = UCLASS_WDT,
	.of_match = ts_micro_wdt_ids,
	.priv_auto = sizeof(struct ts_micro_wdt_priv),
	.ops = &ts_micro_wdt_ops,
};
