#include <common.h>
#include <asm/io.h>
#include <asm/arch/gpio.h>
#include <asm/gpio.h>
#include <asm/arch/iomux-mx28.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <status_led.h>

static unsigned int saved_state[4] = {STATUS_LED_OFF,
	STATUS_LED_OFF, STATUS_LED_OFF, STATUS_LED_OFF};

iomux_cfg_t const led_pads[] = {
	MX28_PAD_GPMI_CE1N__GPIO_0_17, // Red
	MX28_PAD_GPMI_RESETN__GPIO_0_28, // Green
	MX28_PAD_LCD_RS__GPIO_1_26,
	MX28_PAD_LCD_RD_E__GPIO_1_24,	
};

void coloured_LED_init(void)
{
	mxs_iomux_setup_multiple_pads(led_pads, ARRAY_SIZE(led_pads));
}

void red_led_on(void)
{
	gpio_direction_output(led_pads[0], 0);
	saved_state[STATUS_LED_RED] = STATUS_LED_ON;
}

void red_led_off(void)
{
	gpio_direction_output(led_pads[0], 1);
	saved_state[STATUS_LED_RED] = STATUS_LED_OFF;
}

void green_led_on(void)
{
	gpio_direction_output(led_pads[1], 0);
	saved_state[STATUS_LED_GREEN] = STATUS_LED_ON;
}

void yellow_led_off(void)
{
	gpio_direction_output(led_pads[2], 1);
	saved_state[STATUS_LED_YELLOW] = STATUS_LED_OFF;
}

void yellow_led_on(void)
{
	gpio_direction_output(led_pads[2], 0);
	saved_state[STATUS_LED_YELLOW] = STATUS_LED_ON;
}

void blue_led_off(void)
{
	gpio_direction_output(led_pads[3], 0);
	saved_state[STATUS_LED_BLUE] = STATUS_LED_OFF;
}

void blue_led_on(void)
{
	gpio_direction_output(led_pads[3], 1);
	saved_state[STATUS_LED_BLUE] = STATUS_LED_ON;
}

void green_led_off(void)
{
	gpio_direction_output(led_pads[1], 1);
	saved_state[STATUS_LED_GREEN] = STATUS_LED_OFF;
}

void __led_init(led_id_t mask, int state)
{
	__led_set(mask, state);
}

void __led_toggle(led_id_t mask)
{
	if (STATUS_LED_RED == mask) {
		if (STATUS_LED_ON == saved_state[STATUS_LED_RED])
			red_led_off();
		else
			red_led_on();

	} else if (STATUS_LED_GREEN == mask) {
		if (STATUS_LED_ON == saved_state[STATUS_LED_GREEN])
			green_led_off();
		else
			green_led_on();

	}
}

void __led_set(led_id_t mask, int state)
{
	switch(mask) {
	  case STATUS_LED_RED:
		if(STATUS_LED_ON == state) {
			red_led_on();
		} else {
			red_led_off();
		}
		break;
	  case STATUS_LED_GREEN:
		if(STATUS_LED_ON == state) {
			green_led_on();
		} else {
			green_led_off();
		}
		break;
	  case STATUS_LED_YELLOW:
		if(STATUS_LED_ON == state) {
			yellow_led_on();
		} else {
			yellow_led_off();
		}
		break;
	  case STATUS_LED_BLUE:
		if(STATUS_LED_ON == state) {
			blue_led_on();
		} else {
			blue_led_off();
		}
		break;
	}
}
