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
#define GARCIA_FPGA_LX100_ID BIT(14)
#define GARCIA_FPGA_LX150_ID BIT(15)

#define GARCIA_FPGA_POWER_LED_A BIT(0)
#define GARCIA_FPGA_POWER_LED_B BIT(1)
#define GARCIA_FPGA_STATUS_LED_A BIT(2)
#define GARCIA_FPGA_STATUS_LED_B BIT(3)
#define GARCIA_FPGA_SLOT_BUF_NOE BIT(4)
#define GARCIA_FPGA_PACKETIZER_01_ENA BIT(5)
#define GARCIA_FPGA_PACKETIZER_23_ENA BIT(6)
#define GARCIA_FPGA_POWER_LED_FLASH BIT(8)
#define GARCIA_FPGA_STATUS_LED_FLASH BIT(9)
#define GARCIA_FPGA_GENERAL_DIR BIT(10)

uint32_t garcia_gpio_clear(uint32_t clearmask);
uint32_t garcia_gpio_set(uint32_t setmask);
uint32_t garcia_gpio_toggle(uint32_t xormask);
int garcia_led_set(int led, int value);
typedef void (*gpio_irq_callback)(uint32_t gpioval, void *data);
int set_gpio_irq_callback(uint32_t falling_edge_mask, uint32_t rising_edge_mask,
		gpio_irq_callback callback, void *data);
void garcia_control_set_master(int is_master);

/* ioctl commands of Garcia control character device */

#define GARCIA_IOC_MAGIC 'g'
#define GARCIA_IOC_READ_STATUS    _IOR(GARCIA_IOC_MAGIC, 0, __u32)
#define GARCIA_IOC_READ_STATUS_NB _IOR(GARCIA_IOC_MAGIC, 1, __u32)
#define GARCIA_IOC_WRITE_STATUS   _IOW(GARCIA_IOC_MAGIC, 2, __u32)

#define GARCIA_MUTE_CONTROLLER_MASK 0x00F00000 /* Mask of mute control depacketizer stream assignment */
#define GARCIA_MUTE_CONTROLLER_SHIFT   20   /* Bit shift of mute control depacketizer stream assignment */
#define GARCIA_STATUS_MASK_SLOT_POPULATED 0x2000 /* If set, GARCIA_STATUS_SLOT_POPULATED value will be used */
#define GARCIA_STATUS_SLOT_POPULATED 0x1000 /* Slot is not empty - it proxies for an I/O card */
#define GARCIA_STATUS_ENA_UNMUTE    0x800   /* If set, and GARCIA_STATUS_MUTE_FORCE is clear, force unmute */
#define GARCIA_STATUS_MUTE_FORCE    0x400   /* If set, slot is forced to be muted */
#define GARCIA_STATUS_MUTE_ON       (GARCIA_STATUS_MUTE_FORCE & ~GARCIA_STATUS_ENA_UNMUTE)
#define GARCIA_STATUS_MUTE_OFF      (~GARCIA_STATUS_MUTE_FORCE & GARCIA_STATUS_ENA_UNMUTE)
#define GARCIA_STATUS_MUTE_AUTO     (GARCIA_STATUS_MUTE_FORCE | GARCIA_STATUS_ENA_UNMUTE)
#define GARCIA_STATUS_SERDES_SYNC	0x200	/* SERDES/buffers are synced (RO) */
#define GARCIA_STATUS_LRCLK_ACTIVE	0x100	/* LRCLK is present (RO) */
#define GARCIA_STATUS_LRCLK_MASTER	 0x80	/* This slot is providing the master LRCLK for the AVB subsystem (RO) */
#define GARCIA_STATUS_BUFFER_COLL	 0x40	/* Buffer collision (RO) */
#define GARCIA_STATUS_MASTER_MODE	 0x20	/* Driver is master (Hub48 emulator) (RO) */
#define GARCIA_STATUS_INT_ENA		 0x10	/* Interrupt enabled */
#define GARCIA_STATUS_RESET_SIG		  0x8	/* Reset signal is asserted (s:in, m:out) */
#define GARCIA_STATUS_MUTE_SIG		  0x4	/* Mute signal is asserted (s:in, m:out) */
#define GARCIA_STATUS_STROBE		  0x2	/* Strobe signal is asserted (s:in, m:out) */
#define GARCIA_STATUS_SSI_DDIR		  0x1	/* Data direction of SSI (R/W) */

#endif /* GARCIA_FPGA_H_ */
