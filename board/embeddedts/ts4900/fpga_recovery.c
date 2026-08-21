// SPDX-License-Identifier: GPL-2.0+
/*
 * TS-4900 pre-DRAM FPGA recovery over USB SDP.
 *
 * This is intentionally board-local. It embeds a private SDP HID gadget so
 * that the standard U-Boot SDP gadget remains a memory downloader for all
 * other platforms. The only intercepted destination is SDP_FPGA_TOKEN.
 */
#include <dm.h>
#include <g_dnl.h>
#include <linux/kernel.h>
#include <spi_flash.h>
#include <time.h>
#include <usb.h>

#define SDP_FPGA_TOKEN		0x0091f000

static struct spi_flash *recovery_flash;
static u32 recovery_remaining;
static bool recovery_complete;
static int recovery_error;

static void *ts4900_sdp_memcpy(void *dest, const void *src, size_t size)
{
	ulong address = (ulong)dest;
	u32 offset;

	if (address < SDP_FPGA_TOKEN ||
	    address >= SDP_FPGA_TOKEN + CFG_ICE40_LEN)
		return memcpy(dest, src, size);

	offset = address - SDP_FPGA_TOKEN;
	if (size > CFG_ICE40_LEN - offset)
		size = CFG_ICE40_LEN - offset;
	recovery_error = spi_flash_write(recovery_flash, CFG_ICE40_START + offset,
					 size, src);
	if (recovery_error)
		return dest;

	recovery_remaining -= size;
	if (!recovery_remaining)
		recovery_complete = true;
	return dest;
}

/*
 * Keep the SDP implementation private to this board. Rename public symbols
 * and suppress its normal gadget registration before including the source.
 */
#define sdp_init ts4900_sdp_init
#define spl_sdp_handle ts4900_spl_sdp_handle
#define sdp_handle ts4900_sdp_handle
#define sdp_add ts4900_sdp_add
#define memcpy ts4900_sdp_memcpy
#undef DECLARE_GADGET_BIND_CALLBACK
#define DECLARE_GADGET_BIND_CALLBACK(...)
#include "../../../drivers/usb/gadget/f_sdp.c"
#undef DECLARE_GADGET_BIND_CALLBACK
#undef memcpy
#undef sdp_add
#undef sdp_handle
#undef spl_sdp_handle
#undef sdp_init

#define DECLARE_GADGET_BIND_CALLBACK(usb_fname, callback_ptr) \
	ll_entry_declare(struct g_dnl_bind_callback, \
		__usb_function_name_##usb_fname, g_dnl_bind_callbacks) = { \
			.usb_function_name = #usb_fname, \
			.fptr = callback_ptr \
		}
DECLARE_GADGET_BIND_CALLBACK(usb_dnl_ts4900_fpga, ts4900_sdp_add);
#undef DECLARE_GADGET_BIND_CALLBACK

static int ts4900_sdp_wait_configured(struct udevice *udc, ulong timeout_ms)
{
	ulong start = get_timer(0);

	while (!sdp_func->configuration_done) {
		if (timeout_ms && get_timer(start) >= timeout_ms)
			return -ETIMEDOUT;
		schedule();
		dm_usb_gadget_handle_interrupts(udc);
	}

	return 0;
}

static int ts4900_sdp_receive(struct udevice *udc, ulong timeout_ms)
{
	ulong start = get_timer(0);

	while (1) {
		if (recovery_error)
			return recovery_error;
		if (recovery_complete && sdp_func->state == SDP_STATE_IDLE)
			return 0;
		if (timeout_ms && get_timer(start) >= timeout_ms)
			return -ETIMEDOUT;

		schedule();
		dm_usb_gadget_handle_interrupts(udc);
		sdp_handle_in_ep(NULL, NULL);
		if (sdp_func->ep_int_enable)
			sdp_handle_out_ep();
	}
}

int ts4900_sdp_fpga_recovery(void)
{
	struct udevice *udc;
	u32 erase_len;
	int ret;

	recovery_flash = spi_flash_probe(CONFIG_SF_DEFAULT_BUS,
					 CONFIG_SF_DEFAULT_CS,
					 CONFIG_SF_DEFAULT_SPEED,
					 CONFIG_SF_DEFAULT_MODE);
	if (!recovery_flash)
		return -ENODEV;

	erase_len = DIV_ROUND_UP(CFG_ICE40_LEN,
				 recovery_flash->mtd.erasesize) *
		    recovery_flash->mtd.erasesize;
	ret = spi_flash_erase(recovery_flash, CFG_ICE40_START, erase_len);
	if (ret)
		return ret;

	recovery_remaining = CFG_ICE40_LEN;
	recovery_complete = false;
	recovery_error = 0;
	ret = udc_device_get_by_index(CONFIG_SPL_SDP_USB_DEV, &udc);
	if (ret)
		return ret;

	board_usb_init(CONFIG_SPL_SDP_USB_DEV, USB_INIT_DEVICE);
	g_dnl_clear_detach();
	ret = g_dnl_register("usb_dnl_ts4900_fpga");
	if (ret)
		goto out_put;

	ret = ts4900_sdp_wait_configured(udc,
					 CONFIG_TS4900_SDP_FPGA_TIMEOUT);
	if (!ret)
		ret = ts4900_sdp_receive(udc, CONFIG_TS4900_SDP_FPGA_TIMEOUT);
	g_dnl_unregister();
out_put:
	udc_device_put(udc);
	return ret;
}
