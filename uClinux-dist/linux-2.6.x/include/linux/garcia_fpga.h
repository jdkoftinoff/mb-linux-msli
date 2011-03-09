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

#ifndef BIT
#define BIT(x) (1 << (x))
#endif
/* N.B. GARCIA_FPGA_GPIO_SLOT_MUTE_n and GARCIA_FPGA_GPIO_SLOT_RESET_n are active low */
#define GARCIA_FPGA_GPIO_BOX_ID_MASK (BIT(0) | BIT(1) | BIT(2))
#define GARCIA_FPGA_GPIO_PUSHBUTTON BIT(3)
#define GARCIA_FPGA_GPIO_PWRFAIL BIT(4)
#define GARCIA_FPGA_GPIO_JUMPER_1 BIT(5)
#define GARCIA_FPGA_GPIO_JUMPER_2 BIT(6)
#define GARCIA_FPGA_GPIO_DEBUG_0 BIT(7)
#define GARCIA_FPGA_GPIO_DEBUG_1 BIT(8)
#define GARCIA_FPGA_GPIO_DEBUG_2 BIT(9)
#define GARCIA_FPGA_GPIO_DEBUG_3 BIT(10)

#define GARCIA_FPGA_POWER_LED_A BIT(0)
#define GARCIA_FPGA_POWER_LED_B BIT(1)
#define GARCIA_FPGA_STATUS_LED_A BIT(2)
#define GARCIA_FPGA_STATUS_LED_B BIT(3)
#define GARCIA_FPGA_SLOT_BUF_NOE BIT(4)
#define GARCIA_FPGA_GENERAL_DIR BIT(10)

int garcia_led_set(int led, int value);
typedef void (*gpio_irq_callback)(uint32_t gpioval, void *data);
int set_gpio_irq_callback(uint32_t falling_edge_mask, uint32_t rising_edge_mask,
		gpio_irq_callback callback, void *data);

#endif /* GARCIA_FPGA_H_ */
