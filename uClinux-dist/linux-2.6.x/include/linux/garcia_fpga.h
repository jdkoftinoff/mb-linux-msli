/*
 * garcia_fpga.h
 *
 *  Created on: Jan 25, 2011
 *      Author: swagner
 */

#ifndef GARCIA_FPGA_H_
#define GARCIA_FPGA_H_

enum {
	BICOLOR_OFF   = 0,
	BICOLOR_RED   = 1,
	BICOLOR_GREEN = 2,
	BICOLOR_AMBER = 3,
};

enum {
	POWER_LED  = 0,
	STATUS_LED = 1,
};

int garcia_led_set(int led, int value);

#endif /* GARCIA_FPGA_H_ */
