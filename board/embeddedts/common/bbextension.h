/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 Technologic Systems, Inc. dba embeddedTS
 */

#ifndef BBEXTENSION_H
#define BBEXTENSION_H

/* From an internal list of known baseboard IDs, return a matching name
 *
 * @param id 8-bit baseboard ID
 *
 * @returns const char pointer to board name (or "Custom" in the case of an
 *          unmatched ID
 */
const char *ets_bb_extension_name_get(u8 id);

/* Return if a board ID is known
 *
 * @param id 8-bit baseboard ID
 *
 * @returns true if the ID is in our known table and would not return as "Custom",
 *          false otherwise.
 */
bool ets_bb_extension_id_is_known(u8 id);

/* Stub to do the bulk of the scanning work of baseboard ID. Intended to be
 * called from any board specific extension device registration driver callback.
 *
 * This only needs to be used if the default ets_bb_extension driver is not
 * configured in a board file. In this case, the board file would have registered
 * its own extension scan device, as well as driver. This function can be used
 * as part of that to layer any additional custom extensions on top of that. An
 * example of this is the TS-4300 on a TS-8551 that has populated the mipi2dp
 * module. The board extension device can call this to set up the first extension,
 * and then calls its own code to add the correct dtbo for the mipi2dp support.
 *
 * @note This expected the env variable "baseboard_id" to be set, "baseboard_rev"
 *       should be set as well for completeness.
 *
 * @note This function only builds the struct extension *baseboard contents, it
 *       does not add the extension to the extension list.
 *
 * @param baseboard Pointer to a cleared struct extension variable to have its
 *                  contents populated with the details of the baseboard, if any.
 *
 * @param dev Pointer to the registered struct udevice set up by U_BOOT_DRVINFO.
 *            This is needed as the plat member should contain the prefix string
 *            used to identify the architecture and model prefix of the devicetree
 *            file.
 *
 * @returns The number of baseboards found, normally 1, 0 in the case of the
 *          environment variable baseboard_id being 0. Returns negative error
 *          on error.
 */
int ets_bb_extension_board_scan(struct extension *baseboard, struct udevice *dev);

#endif // BBEXTENSION_H
