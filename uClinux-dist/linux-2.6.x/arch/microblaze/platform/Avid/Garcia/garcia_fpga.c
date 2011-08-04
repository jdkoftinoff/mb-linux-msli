/*
 *  garcia_fpga.c - Avid Garcia FPGA base driver
 *
 *  Copyright (C) 2011 Lab X Technologies, LLC
 *  Written by Scott Wagner <scott.wagner@labxtechnologies.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/xilinx_devices.h>
#include <asm/io.h>
#include <linux/vmalloc.h>
#include <linux/timer.h>
#include "xbasic_types.h"
#include "xio.h"
#include "garcia_fpga_priv.h"

#ifdef CONFIG_OF
// For open firmware.
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif


#define garcia_fpga_ReadReg(BaseAddress, RegOffset) \
	(XIo_In32(((BaseAddress) + (RegOffset))))
#define garcia_fpga_WriteReg(BaseAddress, RegOffset, Data) \
	XIo_Out32(((BaseAddress) + (RegOffset)), (Data))

#define garcia_fpga_read_gpio() \
	garcia_fpga_ReadReg(fpga_gpio.gpioaddr, GARCIA_FPGA_GPIO_REGISTER)
#define garcia_fpga_write_gpio(val) \
	garcia_fpga_WriteReg(fpga_gpio.gpioaddr, GARCIA_FPGA_GPIO_REGISTER, val)

#define DRIVER_NAME "garcia_fpga"
#define DRIVER_VERSION "1.0"
#define PUSHBUTTON_RESET_TIME 5*HZ

#define SLOT_RESET_PULSEWIDTH 3 /* mS */
#define N_IRQRESP_VECTORS 8

struct garcia_fpga_gpio_struct {
	uint32_t fifo_irq;
	uint32_t gpioaddr;
	uint32_t shadow_value;
	uint32_t last_gpio_irq;
	struct timer_list reset_timer;
	struct {
		uint32_t falling_mask;
		uint32_t rising_mask;
		gpio_irq_callback callback;
		void *data;
	} irqresponse[N_IRQRESP_VECTORS];
	struct class *gpioclass;
};

static struct garcia_fpga_gpio_struct fpga_gpio;

uint32_t garcia_gpio_clear(uint32_t clearmask) {
	uint32_t val = fpga_gpio.shadow_value & ~clearmask;
	fpga_gpio.shadow_value = val;
	val = ((val & GARCIA_GPIO_INPUTS_MASK) |
			(garcia_fpga_read_gpio() & ~GARCIA_GPIO_INPUTS_MASK));
	garcia_fpga_write_gpio(val);
	return val;
}
EXPORT_SYMBOL(garcia_gpio_clear);

uint32_t garcia_gpio_set(uint32_t setmask) {
	uint32_t val = fpga_gpio.shadow_value | setmask;
	fpga_gpio.shadow_value = val;
	val = ((val & GARCIA_GPIO_INPUTS_MASK) |
			(garcia_fpga_read_gpio() & ~GARCIA_GPIO_INPUTS_MASK));
	garcia_fpga_write_gpio(val);
	return val;
}
EXPORT_SYMBOL(garcia_gpio_set);

uint32_t garcia_gpio_toggle(uint32_t xormask) {
	uint32_t val = fpga_gpio.shadow_value ^ xormask;
	fpga_gpio.shadow_value = val;
	val = ((val & GARCIA_GPIO_INPUTS_MASK) |
			(garcia_fpga_read_gpio() & ~GARCIA_GPIO_INPUTS_MASK));
	garcia_fpga_write_gpio(val);
	return val;
}
EXPORT_SYMBOL(garcia_gpio_toggle);

int garcia_led_set(int led, int value)
{
	uint32_t val;
	int rc = -1;
	val = fpga_gpio.shadow_value;
	if (led == POWER_LED) {
		switch(value) {
		case BICOLOR_OFF:
			val = (val & ~(GARCIA_FPGA_POWER_LED_A | GARCIA_FPGA_POWER_LED_B));
			break;
		case BICOLOR_RED:
			val = (val & ~(GARCIA_FPGA_POWER_LED_B)) | GARCIA_FPGA_POWER_LED_A;
			break;
		case BICOLOR_GREEN:
			val = (val & ~(GARCIA_FPGA_POWER_LED_A)) | GARCIA_FPGA_POWER_LED_B;
			break;
		case BICOLOR_AMBER:
			val = (val | (GARCIA_FPGA_POWER_LED_A | GARCIA_FPGA_POWER_LED_B));
			break;
		}
		rc = (val & (GARCIA_FPGA_POWER_LED_A | GARCIA_FPGA_POWER_LED_B));
		fpga_gpio.shadow_value = val;
	} else if (led == STATUS_LED) {
		switch(value) {
		case BICOLOR_OFF:
			val = (val & ~(GARCIA_FPGA_STATUS_LED_A | GARCIA_FPGA_STATUS_LED_B));
			break;
		case BICOLOR_RED:
			val = (val & ~(GARCIA_FPGA_STATUS_LED_B)) | GARCIA_FPGA_STATUS_LED_A;
			break;
		case BICOLOR_GREEN:
			val = (val & ~(GARCIA_FPGA_STATUS_LED_A)) | GARCIA_FPGA_STATUS_LED_B;
			break;
		case BICOLOR_AMBER:
			val = (val | (GARCIA_FPGA_STATUS_LED_A | GARCIA_FPGA_STATUS_LED_B));
			break;
		}
		rc = (val & (GARCIA_FPGA_STATUS_LED_A | GARCIA_FPGA_STATUS_LED_B)) >> 2;
		fpga_gpio.shadow_value = val;
	}
	if (rc >= 0) {
		val &= GARCIA_GPIO_INPUTS_MASK;
		val |= (garcia_fpga_read_gpio() & ~GARCIA_GPIO_INPUTS_MASK);
		garcia_fpga_write_gpio(val);
	}
	return rc;
}
EXPORT_SYMBOL(garcia_led_set);

int set_gpio_irq_callback(uint32_t falling_edge_mask, uint32_t rising_edge_mask,
		gpio_irq_callback callback, void *data)
{
	int i;
	for (i = 0; i < N_IRQRESP_VECTORS; i++) {
		if (callback == fpga_gpio.irqresponse[i].callback &&
				data == fpga_gpio.irqresponse[i].data) { /* Found existing callback */
			/* Release the callback if masks are 0 */
			if (falling_edge_mask == 0 && rising_edge_mask == 0) {
				fpga_gpio.irqresponse[i].callback = NULL;
				fpga_gpio.irqresponse[i].data = NULL;
			}
			fpga_gpio.irqresponse[i].falling_mask = falling_edge_mask;
			fpga_gpio.irqresponse[i].rising_mask = rising_edge_mask;
			break;
		} else if (fpga_gpio.irqresponse[i].callback == NULL) { /* Assign new callback */
			fpga_gpio.irqresponse[i].callback = callback;
			fpga_gpio.irqresponse[i].data = data;
			fpga_gpio.irqresponse[i].falling_mask = falling_edge_mask;
			fpga_gpio.irqresponse[i].rising_mask = rising_edge_mask;
			break;
		}
	}
	return ((i == N_IRQRESP_VECTORS) ? -ENFILE : 0);
}
EXPORT_SYMBOL(set_gpio_irq_callback);

static irqreturn_t garcia_fpga_irq(int irq, void *data)
{
	struct garcia_fpga_gpio_struct *gp = data;
	int i;
	u32 value = garcia_fpga_ReadReg(gp->gpioaddr, GARCIA_FPGA_GPIO_REGISTER)
				& GARCIA_FPGA_GPIO_MASK;
	u32 changed = gp->last_gpio_irq ^ value;
	if (changed != 0) {
		for (i = 0; i < N_IRQRESP_VECTORS; i++) {
			if ((gp->irqresponse[i].falling_mask & changed) != 0 &&
					(gp->irqresponse[i].falling_mask & value) == 0) {
				(*(gp->irqresponse[i].callback))(value, gp->irqresponse[i].data);
			}
			if ((gp->irqresponse[i].rising_mask & changed) != 0 &&
					(gp->irqresponse[i].rising_mask & value) != 0) {
				(*(gp->irqresponse[i].callback))(value, gp->irqresponse[i].data);
			}
		}
		gp->last_gpio_irq = value;
	}
	if ((value & GARCIA_FPGA_GPIO_PUSHBUTTON) == 0) { // Button is pushed
		mod_timer(&gp->reset_timer, jiffies + PUSHBUTTON_RESET_TIME);
	} else {
		del_timer_sync(&gp->reset_timer);
	}
	/* Reset the interrupt */
	garcia_fpga_WriteReg(gp->gpioaddr, GARCIA_FPGA_GPIO_IPISR,
			garcia_fpga_ReadReg(gp->gpioaddr, GARCIA_FPGA_GPIO_IPISR));
	return IRQ_HANDLED;
}

static ssize_t garcia_r_spimaster(struct class *c,char *buf)
{
	int count = 0;
	int val;
	if ((fpga_gpio.shadow_value & GARCIA_FPGA_SLOT_BUF_NOE) != 0) {
		val = -1;
	} else if ((fpga_gpio.shadow_value & GARCIA_FPGA_GENERAL_DIR) == 0) {
		val = 0;
	} else {
		val = 1;
	}
	count = snprintf(buf, PAGE_SIZE, "%d\n", val);
	return count;
}

static ssize_t garcia_w_spimaster(struct class *c, const char * buf, size_t count)
{
	unsigned long int val;

	if (strict_strtoul(buf, 0, &val) == 0) {
		fpga_gpio.shadow_value &= ~GARCIA_FPGA_GENERAL_DIR;
		if (val != 0) {
			fpga_gpio.shadow_value |= GARCIA_FPGA_GENERAL_DIR;
			garcia_control_set_master(1);
		} else {
			garcia_control_set_master(0);
		}
		fpga_gpio.shadow_value &= ~GARCIA_FPGA_SLOT_BUF_NOE;
		garcia_fpga_write_gpio((garcia_fpga_read_gpio() & ~GARCIA_GPIO_INPUTS_MASK) |
					(fpga_gpio.shadow_value & GARCIA_GPIO_INPUTS_MASK));
	}
	return count;
}

static ssize_t garcia_r_packetizer_ena(struct class *c,char *buf)
{
	int count = 0;
	int val = 0;
	if ((fpga_gpio.shadow_value & GARCIA_FPGA_PACKETIZER_01_ENA) != 0) {
		val |= 1;
	}
	if ((fpga_gpio.shadow_value & GARCIA_FPGA_PACKETIZER_23_ENA) != 0) {
		val |= 2;
	}
	count = snprintf(buf, PAGE_SIZE, "%d\n", val);
	return count;
}

static ssize_t garcia_w_packetizer_ena(struct class *c, const char * buf, size_t count)
{
	unsigned long int val;

	if (strict_strtoul(buf, 0, &val) == 0) {
		fpga_gpio.shadow_value &= ~(GARCIA_FPGA_PACKETIZER_01_ENA | GARCIA_FPGA_PACKETIZER_23_ENA);
		if ((val & 1) != 0) {
			fpga_gpio.shadow_value |= GARCIA_FPGA_PACKETIZER_01_ENA;
		}
		if ((val & 2) != 0) {
			fpga_gpio.shadow_value |= GARCIA_FPGA_PACKETIZER_23_ENA;
		}
		garcia_fpga_write_gpio((garcia_fpga_read_gpio() & ~GARCIA_GPIO_INPUTS_MASK) |
					(fpga_gpio.shadow_value & GARCIA_GPIO_INPUTS_MASK));
	}
	return count;
}

static ssize_t garcia_r_inputs(struct class *c, char *buf)
{
	return (snprintf(buf, PAGE_SIZE, "%lx\n",
			garcia_fpga_read_gpio() & GARCIA_GPIO_INPUTS_MASK));
}

static ssize_t garcia_r_selector(struct class *c, char *buf)
{
	return (snprintf(buf, PAGE_SIZE, "%ld\n",
			garcia_fpga_read_gpio() & GARCIA_FPGA_GPIO_BOX_ID_MASK));
}

static ssize_t garcia_r_pushbutton(struct class *c, char *buf)
{
	return (snprintf(buf, PAGE_SIZE, "%d\n",
			((garcia_fpga_read_gpio() & GARCIA_FPGA_GPIO_PUSHBUTTON) == 0)));
}

static ssize_t garcia_r_jumper1(struct class *c, char *buf)
{
	return (snprintf(buf, PAGE_SIZE, "%d\n",
			((garcia_fpga_read_gpio() & GARCIA_FPGA_GPIO_JUMPER_1) == 0)));
}

static ssize_t garcia_r_jumper2(struct class *c, char *buf)
{
	return (snprintf(buf, PAGE_SIZE, "%d\n",
			((garcia_fpga_read_gpio() & GARCIA_FPGA_GPIO_JUMPER_2) == 0)));
}

static ssize_t garcia_r_gpioraw(struct class *c, char *buf)
{
	int count = 0;
	u32 val = garcia_fpga_read_gpio();
	count = snprintf(buf, PAGE_SIZE, "%x\n", val);
	return count;
}

static struct class_attribute garcia_fpga_class_attrs[] = {
	__ATTR(spimaster, S_IRUGO | S_IWUGO, garcia_r_spimaster, garcia_w_spimaster),
	__ATTR(packetizer_ena, S_IRUGO | S_IWUGO, garcia_r_packetizer_ena, garcia_w_packetizer_ena),
	__ATTR(inputs, S_IRUGO, garcia_r_inputs, NULL),
	__ATTR(selector, S_IRUGO, garcia_r_selector, NULL),
	__ATTR(pushbutton, S_IRUGO, garcia_r_pushbutton, NULL),
	__ATTR(jumper1, S_IRUGO, garcia_r_jumper1, NULL),
	__ATTR(jumper2, S_IRUGO, garcia_r_jumper2, NULL),
	__ATTR(gpioraw, S_IRUGO, garcia_r_gpioraw, NULL),
	__ATTR_NULL,
};

static struct class garcia_fpga_class = {
	.name =		DRIVER_NAME,
	.owner =	THIS_MODULE,
	.class_attrs =	garcia_fpga_class_attrs,
};

static void reset_timer_function(unsigned long data)
{
	(void)data;
	printk("Restarting!\n");
	machine_restart(NULL);
}

static int garcia_led_default[2] = {2, 3};
module_param_array(garcia_led_default, int, NULL, 0);
MODULE_PARM_DESC(garcia_led_default, "Garcia LEDs default state");


static int __devinit garcia_fpga_probe(struct device *dev)
{
	int rc;
	struct resource *p_mem;
	struct resource *p_irq;
	struct platform_device *pdev = to_platform_device(dev);

	printk(KERN_INFO "Platform Device Probing \'%s\'\n", DRIVER_NAME);

	/* param check */
	if (!pdev) {
		dev_err(dev, "labx_ethernet: Internal error. Probe called with NULL param.\n");
		return -ENODEV;
	}

	/* Get iospace and an irq for the device */
	p_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!p_mem) {
		dev_err(dev, "labx_ethernet: IO resource MEM not found.\n");
		return -ENODEV;
	}
	p_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	fpga_gpio.gpioaddr = (u32) ioremap_nocache(p_mem->start, p_mem->end - p_mem->start + 1);
	fpga_gpio.fifo_irq = p_irq->start;
	/* Register for GPIO Interrupt */
	rc = request_irq(fpga_gpio.fifo_irq, garcia_fpga_irq, 0, DRIVER_NAME, &fpga_gpio);
	if (rc != 0) {
		dev_err(dev, "irq request failure: %d\n", fpga_gpio.fifo_irq);
	} else {
		garcia_fpga_WriteReg(fpga_gpio.gpioaddr, GARCIA_FPGA_GPIO_GIER, GARCIA_FPGA_GPIO_GIE);
		garcia_fpga_WriteReg(fpga_gpio.gpioaddr, GARCIA_FPGA_GPIO_IPIER, GARCIA_FPGA_GPIO_IPIE);
	}

	fpga_gpio.last_gpio_irq = garcia_fpga_read_gpio() & GARCIA_FPGA_GPIO_MASK;
	fpga_gpio.shadow_value = (garcia_fpga_read_gpio() & ~GARCIA_GPIO_INPUTS_MASK) |
				GARCIA_FPGA_SLOT_BUF_NOE;
	garcia_fpga_write_gpio(fpga_gpio.shadow_value);
	garcia_led_set(POWER_LED, garcia_led_default[POWER_LED]);
	garcia_led_set(STATUS_LED, garcia_led_default[STATUS_LED]);
	init_timer(&fpga_gpio.reset_timer);
	fpga_gpio.reset_timer.function = reset_timer_function;
	fpga_gpio.reset_timer.data = 0;
	rc = class_register(&garcia_fpga_class);

	return rc;
}

static int __devinit garcia_fpga_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
	int rc;
	struct resource r_mem;
	struct resource r_irq;

	printk(KERN_INFO "Device Tree Probing \'%s\'\n",ofdev->node->name);

	/* Get iospace for the device */
	rc = of_address_to_resource(ofdev->node, 0, &r_mem);
	if(rc) {
		dev_warn(&ofdev->dev, "invalid address\n");
		return rc;
	}
	/* Get IRQ for the device */
	rc = of_irq_to_resource(ofdev->node, 0, &r_irq);
	if(rc == NO_IRQ || rc == ENODEV) {
		dev_warn(&ofdev->dev, "no IRQ found\n");
		r_irq.start = 19;
		rc = 0;
	}

	fpga_gpio.gpioaddr = (u32) ioremap_nocache(r_mem.start, r_mem.end - r_mem.start + 1);
	fpga_gpio.fifo_irq = r_irq.start;
	/* Register for GPIO Interrupt */
	rc = request_irq(fpga_gpio.fifo_irq, garcia_fpga_irq, 0, DRIVER_NAME, &fpga_gpio);
	if (rc != 0) {
		dev_err(&ofdev->dev, "irq request failure: %d\n", fpga_gpio.fifo_irq);
	} else {
		garcia_fpga_WriteReg(fpga_gpio.gpioaddr, GARCIA_FPGA_GPIO_GIER, GARCIA_FPGA_GPIO_GIE);
		garcia_fpga_WriteReg(fpga_gpio.gpioaddr, GARCIA_FPGA_GPIO_IPIER, GARCIA_FPGA_GPIO_IPIE);
	}
	fpga_gpio.last_gpio_irq = garcia_fpga_read_gpio() & GARCIA_FPGA_GPIO_MASK;
	fpga_gpio.shadow_value = (garcia_fpga_read_gpio() & ~GARCIA_GPIO_INPUTS_MASK) |
			GARCIA_FPGA_SLOT_BUF_NOE;
	garcia_fpga_write_gpio(fpga_gpio.shadow_value);
	garcia_led_set(POWER_LED, garcia_led_default[POWER_LED]);
	garcia_led_set(STATUS_LED, garcia_led_default[STATUS_LED]);
	init_timer(&fpga_gpio.reset_timer);
	fpga_gpio.reset_timer.function = reset_timer_function;
	fpga_gpio.reset_timer.data = 0;
	rc |= class_register(&garcia_fpga_class);

	return rc;
}

static int garcia_fpga_remove(struct device *dev)
{
	memset(&fpga_gpio, 0, sizeof(fpga_gpio));
	return 0;		/* success */
}

static int __devexit garcia_fpga_of_remove(struct of_device *dev)
{
	return garcia_fpga_remove(&dev->dev);
}

static struct device_driver garcia_fpga_driver = {
	.name = DRIVER_NAME,
	.bus = &platform_bus_type,

	.probe = garcia_fpga_probe,
	.remove = garcia_fpga_remove,
};

static struct of_device_id garcia_fpga_of_match[] = {
	{ .compatible = "xlnx,xps-gpio-2.00.a", },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, garcia_fpga_of_match);

static struct of_platform_driver garcia_fpga_of_driver = {
	.name		= DRIVER_NAME,
	.match_table	= garcia_fpga_of_match,
	.probe		= garcia_fpga_of_probe,
	.remove		= __devexit_p(garcia_fpga_of_remove),
};

static int __init garcia_fpga_init(void)
{
	int status;
	status = driver_register(&garcia_fpga_driver);
#ifdef CONFIG_OF
	status |= of_register_platform_driver(&garcia_fpga_of_driver);
#endif
	return status;
}

static void __exit garcia_fpga_exit(void)
{
	driver_unregister(&garcia_fpga_driver);
#ifdef CONFIG_OF
	of_unregister_platform_driver(&garcia_fpga_of_driver);
#endif
}

MODULE_AUTHOR("Scott Wagner <scott.wagner@labxtechnologies.com>");
MODULE_DESCRIPTION("Base Avid Garcia FPGA driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(garcia_fpga_init);
module_exit(garcia_fpga_exit);
