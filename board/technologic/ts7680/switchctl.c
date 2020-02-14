/*
 * Copyright (C) 2018 Technologic Systems
 *
 * Author: Mark Featherston <mark@embeddedarm.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>

#include <asm/gpio.h>
#include <asm/arch/iomux-mx28.h>
#include <cli.h>
#include <command.h>
#include <i2c.h>
#include <status_led.h>

#include <miiphy.h>

int cpu_port = 0;

struct vtu {
	int v, vid;
	unsigned char tags[7];
	unsigned char forceVID[7];
};

#define GLOBAL_REGS_1			0x1f
#define GLOBAL_REGS_2			0x17
#define VTU_OPS_REGISTER		0x05 /* offset in GLOBAL_REGS_1 */
#define VTU_OP_FLUSHALL			(1 << 12)
#define VTU_OP_LOAD			(3 << 12)
#define VTU_OP_GET_NEXT			(4 << 12)
#define VTU_OP_GET_CLR_VIOLATION	(7 << 12)
#define VTU_VID_REG			0x06 /* offset in GLOBAL_REGS_1 */

volatile unsigned int *mxgpioregs;

int switch_mdio_init(void)
{
	mxgpioregs = (unsigned int *)0x80018000;

	/* Set up MDIO/MDC as GPIO */
	mxgpioregs[0x184/4] = 0xf;

	return 0;
}

static int inline MDIO_RD(void) {return ((mxgpioregs[0x940/4] >> 1) & 0x1);}
static int inline MDC_RD(void) {return ((mxgpioregs[0x940/4] >> 0) & 0x1);}
static void inline MDIO_OUT(void) {mxgpioregs[0xb44/4] = 0x2;}
static void inline MDIO_IN(void) {mxgpioregs[0xb48/4] = 0x2;}
static void inline MDC_OUT(void) {mxgpioregs[0xb44/4] = 0x1;}
static void inline MDC_IN(void) {mxgpioregs[0xb48/4] = 0x1;}
static void inline MDIO_HI(void) {mxgpioregs[0x744/4] = 0x2;}
static void inline MDIO_LO(void) {mxgpioregs[0x748/4] = 0x2;}
static void inline MDC_HI(void) {mxgpioregs[0x744/4] = 0x1;}
static void inline MDC_LO(void) {mxgpioregs[0x748/4] = 0x1;}

int switch_mdio_write(unsigned long phy, unsigned long reg, unsigned short data) {
	int x;
	unsigned int b;

	MDIO_OUT();
	MDC_OUT();

	MDC_LO();
	MDIO_HI();

	/* preamble, toggle clock high then low */
	for(x=0; x < 32; x++) {
		MDC_HI();
		MDC_LO();
	}
	/* preamble ends with MDIO high and MDC low */

	/* start bit followed by write bit */
	for(x=0; x < 2; x++) {
		MDIO_LO();
		MDC_HI();
		MDC_LO();
		MDIO_HI();
		MDC_HI();
		MDC_LO();
	}
	/* ends with MDIO high and MDC low */

	/* send the PHY address, starting with MSB */
	b = 0x10;
	for(x=0; x < 5; x++) {
		if (phy & b)
		  {MDIO_HI();}
		else
		  {MDIO_LO();}

		MDC_HI();
		MDC_LO();

		b >>= 1;
	}
	/* ends with MDC low, MDIO indeterminate */

	/* send the register address, starting with MSB */
	b = 0x10;
	for(x=0; x < 5; x++) {
		if (reg & b)
		  {MDIO_HI();}
		else
		  {MDIO_LO();}

		MDC_HI();
		MDC_LO();

		b >>= 1;
	}

	/* ends with MDC low, MDIO indeterminate */

	/* clock a one, then clock a zero */
	MDIO_HI();
	MDC_HI();
	MDC_LO();

	MDIO_LO();
	MDC_HI();
	MDC_LO();

	/* send the data, starting with MSB */
	b = 0x8000;
	for(x=0; x < 16; x++) {
		if (data & b)
		 { MDIO_HI();}
		else
		  {MDIO_LO();}

		MDC_HI();
		MDC_LO();

		b >>= 1;
	}

	MDIO_IN();
	MDC_IN();

	return 0;
}

int switch_mdio_read(unsigned long phy, unsigned long reg,
  volatile unsigned short *data) {
	int x, d;
	unsigned int a,b;

	d = 0;

	MDC_OUT();
	MDIO_OUT();

	MDC_LO();
	MDIO_HI();

	/* preamble, toggle clock high then low */
	for(x=0; x < 32; x++) {
		MDC_HI();
		MDC_LO();
	}
	/* preamble ends with MDIO high and MDC low */

	/* start bit */
	MDIO_LO(); MDC_HI(); MDC_LO();
	MDIO_HI(); MDC_HI(); MDC_LO();
	MDC_HI();  MDC_LO();
	MDIO_LO(); MDC_HI(); MDC_LO();

	/* send the PHY address, starting with MSB */
	b = 0x10;
	for(x=0; x < 5; x++) {
		if (phy & b)
		  {MDIO_HI();}
		else
		  {MDIO_LO();}

		MDC_HI();
		MDC_LO();

		b >>= 1;
		}
		/* ends with MDC low, MDIO indeterminate */

	/* send the register address, starting with MSB */
	b = 0x10;
	for(x=0; x < 5; x++) {
		if (reg & b)
		  {MDIO_HI();}
		else
		  {MDIO_LO();}

		MDC_HI();
		MDC_LO();

		b >>= 1;
	}

	MDIO_IN();
	/* ends with MDC low, MDIO indeterminate */

	/* another clock */
	MDC_HI();
	MDC_LO();

	/* read the data, starting with MSB */
	b = 0x8000;
	for(x=0; x < 16; x++) {
		MDC_HI();
		a = MDIO_RD();

		if (a & 1)
		d |= b;

		MDC_LO();

		b >>= 1;
	}

	MDIO_IN();
	MDC_IN();

	*data = d;
	return 0;
}

void switch_errata_3_1(unsigned int n)
{
	volatile unsigned short smicmd, x;

	/* Write 0x0003 to PHY register 13 */
	do {
		switch_mdio_read(0x17, 0x18, &x);
	} while (x & (1 << 15));
	smicmd = 0x960D | n;
	switch_mdio_write(0x17, 0x19, 0x0003); /* smi data */
	switch_mdio_write(0x17, 0x18, smicmd);

	/* Write 0x0000 to PHY register 14 */
	do {
		switch_mdio_read(0x17, 0x18, &x);
	} while (x & (1 << 15));
	smicmd = 0x960E | n;
	switch_mdio_write(0x17, 0x19, 0); /* smi data */
	switch_mdio_write(0x17, 0x18, smicmd);

	/* Write 0x4003 to PHY register 13 */
	do {
		switch_mdio_read(0x17, 0x18, &x);
	} while (x & (1 << 15));
	smicmd = 0x960D | n;
	switch_mdio_write(0x17, 0x19, 0x4003); /* smi data */
	switch_mdio_write(0x17, 0x18, smicmd);

	/* Write 0x0000 to PHY register 14 */
	do {
		switch_mdio_read(0x17, 0x18, &x);
	} while (x & (1 << 15));
	smicmd = 0x960E | n;
	switch_mdio_write(0x17, 0x19, 0); /* smi data */
	switch_mdio_write(0x17, 0x18, smicmd);
}

void switch_enphy(unsigned long phy) {
	volatile unsigned short x;
	do {
		switch_mdio_read(0x17, 0x18, &x);
	} while (x & (1 << 15));
	switch_mdio_write(0x17, 0x19, 0xb300);
	switch_mdio_write(0x17, 0x18, phy);
}

void switch_disphy(unsigned long phy) {
	volatile unsigned short x;
	do {
		switch_mdio_read(0x17, 0x18, &x);
	} while (x & (1 << 15));
	switch_mdio_write(0x17, 0x19, 0x0900);
	switch_mdio_write(0x17, 0x18, phy);
}

void switch_enflood(unsigned long port) {
	volatile unsigned short data;
	switch_mdio_read(port, 0x04, &data);
	switch_mdio_write(port, 0x04, data | 0xf);
}

void switch_enbcastflood(unsigned long port) {
	switch_mdio_write(port, 0x04, 0x7f);
}

void switch_forcelink(unsigned long port) {
	volatile unsigned short data;
	switch_mdio_read(port, 0x01, &data);
	switch_mdio_write(port, 0x01, data | 0x30);
}

static int do_switchctl(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	volatile unsigned short swmod;
	int ret = 0;

	switch_mdio_init();
	switch_mdio_read(0x18, 0x03, &swmod);

	// this chip has 2 PHYs and 2 RMII ports
	// Apply Marvell 88E60x0 Errata 3.1 for PHY 0 and 1.
	switch_errata_3_1(0);
	switch_errata_3_1(1<<5);

	// enable PHYs
	switch_enphy(0x9600); // Enable port 0
	if (argc > 1) {
		switch_disphy(0x9620);
		printf("Disabled switch port 1\n");
	} else {
		switch_enphy(0x9620);
	}

	// Force link up on P5, the CPU port
	switch_forcelink(0x1d);

	switch_enflood(0x18);
	switch_enflood(0x19);
	switch_enflood(0x1d);

	return ret;
}

U_BOOT_CMD(switchctl, 2, 1,	do_switchctl,
	"Enables the switch to forward packets to both ports",
	"[-d]\n"
	"  -d   Disable port 1 after init\n"
);

