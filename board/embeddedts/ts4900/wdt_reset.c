#include <asm/arch/imx-regs.h>
#include <asm/io.h>
#include <watchdog.h>
#include <fsl_wdog.h>

/* This function behaves the same as imx_watchdog_expire_now(), but provides a
 * non-weak reset_cpu() call.
 */
void reset_cpu(void)
{
	struct watchdog_regs *wdog = (struct watchdog_regs *)WDOG1_BASE_ADDR;

	u16 wcr = WCR_WDE;

	/* Asserts both internal and external WDOG resets */

	/* Write 3 times to ensure it works, due to IMX6Q errata ERR004346 */
	writew(wcr, &wdog->wcr);
	writew(wcr, &wdog->wcr);
	writew(wcr, &wdog->wcr);

	while (1) {
		 /*
		  * spin before reset
		  */
	}
}
