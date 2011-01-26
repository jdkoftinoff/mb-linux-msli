/*
 *	include/linux/broadcom_leds.h
 *
 *	LED control for Broadcom BCM54xx Gigabit Ethernet
 *	transceivers.
 *
 *	Copyright (c) 2011  Scott Wagner and Maciej W. Rozycki
 *
 *	Inspired by code written by Amy Fong.
 *
 *	This LED control code is known to work for the BCM5481 and
 *	believed to work for the BCM5482, and untested on everything else.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 */

#ifndef BROADCOM_LEDS_H_
#define BROADCOM_LEDS_H_

enum BC_PHY_LEDVAL {
	BC_PHY_LED_OFF = 0,
	BC_PHY_LED_ON = 1,
	BC_PHY_LED_LINKSPD1 = 0x10,
	BC_PHY_LED_LINKSPD2 = 0x11,
	BC_PHY_LED_XMTLED = 0x12,
	BC_PHY_LED_ACTIVITY = 0x13,
	BC_PHY_LED_FDXLED = 0x14,
	BC_PHY_LED_SLAVE = 0x15,
	BC_PHY_LED_INTR = 0x16,
	BC_PHY_LED_QUALITY = 0x17,
	BC_PHY_LED_RCVLED = 0x18,
	BC_PHY_LED_MULTICOLOR1 = 0x1A,
	BC_PHY_LED_OPENSHORT = 0x1B,
	BC_PHY_LED_SRC_OFF = 0x1E,
	BC_PHY_LED_SRC_ON = 0x1F,
	BC_PHY_LED_DEFAULT = -1,
};

enum BC_PHY_LEDSEL {
	BC_PHY_LED1 = 1,
	BC_PHY_LED2 = 2,
	BC_PHY_LED3 = 3,
	BC_PHY_LED4 = 4,
};
void bc_phy_led_set(int phyno, enum BC_PHY_LEDSEL whichLed, enum BC_PHY_LEDVAL val);

#endif /* BROADCOM_LEDS_H_ */
