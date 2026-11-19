// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Technologic Systems, Inc. dba embeddedTS
 */

/* Generic baseboard extension scan relying on the env vars baseboard_id and
 * baseboard_rev to load in devicetree overlays for specific baseboards.
 */

/* TODO:
 * The current extension framework only allows for a single scan handler to be
 * registered. Rather, it only calls the first one registered. So if there are
 * any individual boards that have their own results they might want to overlay,
 * for example the TS-4300 TS-8551 mipi2dp dtbo, we currently only provide some
 * scan helper functions that can be used instead. Another option may be custom
 * use of plat data in the drvdata structure as a callback to add additional
 * scan logic to submit all under the same scan handler.
 */

#if CONFIG_IS_ENABLED(SUPPORT_EXTENSION_SCAN) && \
		      !CONFIG_IS_ENABLED(XPL_BUILD)

#include <env.h>
#include <extension_board.h>

struct known_ids {
	u8		id;
	const char	*name;
};

static struct known_ids known_ids[] = {
	{ 0x02, "TS-TPC-8390" },
	{ 0x06, "TS-8160" },
	{ 0x07, "TS-8100" },
	{ 0x08, "TS-8820" },
	{ 0x09, "TS-8150" },
	{ 0x0a, "TS-TPC-8900" },
	{ 0x13, "TS-8550" },
	{ 0x14, "TS-TPC-8950" },
	{ 0x16, "TS-8551" },
};

const char *ets_bb_extension_name_get(u8 id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(known_ids); i++) {
		if (id == known_ids[i].id)
			return known_ids[i].name;
	}

	return "Custom";
}

bool ets_bb_extension_id_is_known(u8 id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(known_ids); i++) {
		if (known_ids[i].id == id)
			return true;
	}

	return false;
}

/* Populates an extension struct based on baseboard ID and rev and returns that */
/* baseboard should already be allocated */
/* Returns 1 on baseboard found, 0 on no baseboard found, or negative error */
int ets_bb_extension_board_scan(struct extension *baseboard, struct udevice *dev)
{
	u8 baseboard_id;
	u8 baseboard_rev;
	const char *plat = dev_get_plat(dev);

	if (!baseboard || !dev)
		return -EINVAL;

	/* Get the baseboard info from the env. Assume an ID of 0 means no baseboard */
	baseboard_id = env_get_hex("baseboard_id", 0);
	if (!baseboard_id)
		return 0;

	baseboard_rev = env_get_hex("baseboard_rev", 0);

	if (ets_bb_extension_id_is_known(baseboard_id))
		snprintf(baseboard->owner, sizeof(baseboard->owner), "embeddedTS");
	snprintf(baseboard->name, sizeof(baseboard->name), "%s",
		 ets_bb_extension_name_get(baseboard_id));
	snprintf(baseboard->version, sizeof(baseboard->version), "%d",
		 baseboard_rev);

	if (plat)
		snprintf(baseboard->overlay, sizeof(baseboard->overlay),
			 "%s-%02x.dtbo", plat, baseboard_id);
	else
		snprintf(baseboard->overlay, sizeof(baseboard->overlay),
			 "%02x.dtbo", baseboard_id);

	/* With baseboard_id in the environment, that means there is a valid
	 * baseboard, so we return that as a count for other functions to use.
	 */
	return 1;
}

static int _ets_bb_extension_board_scan(struct udevice *dev,
					struct alist *extension_list)
{
	int extensions = 0;
	struct extension baseboard = {0};

	extensions = ets_bb_extension_board_scan(&baseboard, dev);
	if (extensions > 0)
		if (!alist_add(extension_list, baseboard))
			return -ENOMEM;

	return extensions;
}

/* Since there is a need for some kind of prefix when looking up an extension,
 * we don't directly register the DRVINFO ourselves. The actual board files should
 * implement the DRVINFO with plat set to the necessary prefix if there is one.
 */
U_BOOT_EXTENSION(ets_bb_extension, _ets_bb_extension_board_scan);

#endif // SUPPORT_EXTENSION_SCAN
