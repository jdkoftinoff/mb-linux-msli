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
#include "xbasic_types.h"
#include "xio.h"
#include <linux/garcia_fpga.h>
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
#define DRIVER_VERSION "0.4"

struct garcia_fpga_gpio_struct {
	uint32_t fifo_irq;
	uint32_t gpioaddr;
	uint32_t shadow_value;
};

static struct garcia_fpga_gpio_struct fpga_gpio;

int garcia_led_set(int led, int value)
{
	uint32_t val;
	int rc = -1;
	val = (fpga_gpio.shadow_value & ~GARCIA_GPIO_RW_MASK);
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
		val |= (garcia_fpga_read_gpio() & GARCIA_GPIO_RW_MASK);
		garcia_fpga_write_gpio(val);
	}
	return rc;
}
EXPORT_SYMBOL(garcia_led_set);

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
	fpga_gpio.shadow_value = garcia_fpga_read_gpio() & GARCIA_GPIO_RW_MASK;
	garcia_fpga_write_gpio(fpga_gpio.shadow_value);

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
		r_irq.start = 0;
		rc = 0;
	}

	fpga_gpio.gpioaddr = (u32) ioremap_nocache(r_mem.start, r_mem.end - r_mem.start + 1);
	fpga_gpio.fifo_irq = r_irq.start;
	fpga_gpio.shadow_value = garcia_fpga_read_gpio() & GARCIA_GPIO_RW_MASK;
	garcia_fpga_write_gpio(fpga_gpio.shadow_value);

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
