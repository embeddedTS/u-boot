#include <common.h>
#include <command.h>
#include <asm/gpio.h>

static int do_bbdetect(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int i, id = 0;
	gpio_direction_output(CONFIG_BB_S0, 1);
	gpio_direction_output(CONFIG_BB_S1, 1);
	gpio_direction_output(CONFIG_BB_S2, 1);
	gpio_direction_input(CONFIG_BB_IN);

	for(i = 0; i < 8; i++) {
		if(i & 1)gpio_set_value(CONFIG_BB_S0, 1);
		else gpio_set_value(CONFIG_BB_S0, 0);

		if(i & 2)gpio_set_value(CONFIG_BB_S1, 1);
		else gpio_set_value(CONFIG_BB_S1, 0);
		
		if(i & 4)gpio_set_value(CONFIG_BB_S2, 1);
		else gpio_set_value(CONFIG_BB_S2, 0);

		udelay(CONFIG_BB_USDLY);

		id = (id >> 1);
		if(gpio_get_value(CONFIG_BB_IN)) id |= 0x80;
	}

	switch (id & ~0xc0) {
		case 0:
			setenv("baseboard", "8200");
			break;
		case 2:
			setenv("baseboard", "8390");
			break;
		case 4:
			setenv("baseboard", "8500");
			break;
		case 5:
			setenv("baseboard", "8400");
			break;
		case 6:
			setenv("baseboard", "8160");
			break;
		case 7:
			setenv("baseboard", "8100");
			break;
		case 8:
			setenv("baseboard", "8820");
			break;
		case 9:
			setenv("baseboard", "8150");
			break;
		case 10:
			setenv("baseboard", "8900");
			break;
		case 11:
			setenv("baseboard", "8290");
			break;
		case 13:
			setenv("baseboard", "8700");
			break;
		case 14:
			setenv("baseboard", "8280");
			break;
		case 15:
			setenv("baseboard", "8380");
			break;
		case 16:
			setenv("baseboard", "an20");
			break;
		case 17:
			setenv("baseboard", "8920");
			break;
		default:
			setenv("baseboard", "unknown");
			break;
	}

	setenv_hex("baseboardid", id & ~0xc0);
	setenv_hex("baseboardrev", ((id & 0xc0) >> 6));
	return 0;
}

U_BOOT_CMD(
	bbdetect,	1,		1,	do_bbdetect,
	"Reads the value from a hard wired 8-input mux",
	""
);
