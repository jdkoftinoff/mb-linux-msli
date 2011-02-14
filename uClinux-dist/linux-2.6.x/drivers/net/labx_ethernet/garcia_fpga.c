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
	struct class *gpioclass;
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
	count = sprintf(buf, "%d\n", val);
	return count;
}

static ssize_t garcia_w_spimaster(struct class *c, const char * buf, size_t count)
{
	unsigned long int val;
	unsigned long int cur;

	if (strict_strtoul(buf, 0, &val) == 0) {
		fpga_gpio.shadow_value &= ~GARCIA_FPGA_GENERAL_DIR;
		if (val != 0) {
			fpga_gpio.shadow_value |= GARCIA_FPGA_GENERAL_DIR;
		}
		fpga_gpio.shadow_value &= ~GARCIA_FPGA_SLOT_BUF_NOE;
		cur = (garcia_fpga_read_gpio() & GARCIA_GPIO_RW_MASK) |
				(fpga_gpio.shadow_value & ~GARCIA_GPIO_RW_MASK);
		garcia_fpga_write_gpio(cur);
	}
	return count;
}

static ssize_t garcia_r_ssidir(struct class *c, char *buf)
{
	int count = 0;
	u32 val;
	val = ((fpga_gpio.shadow_value & GARCIA_FPGA_SLOT_SSI_DDIR_1) ? BIT(0) : 0) |
			((fpga_gpio.shadow_value & GARCIA_FPGA_SLOT_SSI_DDIR_2) ? BIT(1) : 0) |
			((fpga_gpio.shadow_value & GARCIA_FPGA_SLOT_SSI_DDIR_3) ? BIT(2) : 0) |
			((fpga_gpio.shadow_value & GARCIA_FPGA_SLOT_SSI_DDIR_4) ? BIT(3) : 0) |
			((fpga_gpio.shadow_value & GARCIA_FPGA_SLOT_SSI_DDIR_5) ? BIT(4) : 0);
	count = sprintf(buf, "%u\n", val);
	return count;
}

static ssize_t garcia_w_ssidir(struct class *c, const char * buf, size_t count)
{
	unsigned long int val;
	unsigned long int cur;

	if (strict_strtoul(buf, 0, &val) == 0) {
		fpga_gpio.shadow_value &= ~(GARCIA_FPGA_SLOT_SSI_DDIR_1 |
				GARCIA_FPGA_SLOT_SSI_DDIR_2 | GARCIA_FPGA_SLOT_SSI_DDIR_3 |
				GARCIA_FPGA_SLOT_SSI_DDIR_4 | GARCIA_FPGA_SLOT_SSI_DDIR_5);
		if (val & BIT(0)) fpga_gpio.shadow_value |= GARCIA_FPGA_SLOT_SSI_DDIR_1;
		if (val & BIT(1)) fpga_gpio.shadow_value |= GARCIA_FPGA_SLOT_SSI_DDIR_2;
		if (val & BIT(2)) fpga_gpio.shadow_value |= GARCIA_FPGA_SLOT_SSI_DDIR_3;
		if (val & BIT(3)) fpga_gpio.shadow_value |= GARCIA_FPGA_SLOT_SSI_DDIR_4;
		if (val & BIT(4)) fpga_gpio.shadow_value |= GARCIA_FPGA_SLOT_SSI_DDIR_5;
		cur = (garcia_fpga_read_gpio() & GARCIA_GPIO_RW_MASK) |
				(fpga_gpio.shadow_value & ~GARCIA_GPIO_RW_MASK);
		garcia_fpga_write_gpio(cur);
	}
	return count;
}

static ssize_t garcia_r_mute(struct class *c, char *buf)
{
	int count = 0;
	u32 val = garcia_fpga_read_gpio();
	val = ((val & GARCIA_FPGA_GPIO_SLOT_MUTE_1) ? BIT(0) : 0) |
			((val & GARCIA_FPGA_GPIO_SLOT_MUTE_2) ? BIT(1) : 0) |
			((val & GARCIA_FPGA_GPIO_SLOT_MUTE_3) ? BIT(2) : 0) |
			((val & GARCIA_FPGA_GPIO_SLOT_MUTE_4) ? BIT(3) : 0) |
			((val & GARCIA_FPGA_GPIO_SLOT_MUTE_5) ? BIT(4) : 0);
	count = sprintf(buf, "%u\n", val);
	return count;
}

static ssize_t garcia_w_mute(struct class *c, const char * buf, size_t count)
{
	unsigned long int val;
	unsigned long int cur;

	if (strict_strtoul(buf, 0, &val) == 0) {
		cur = (fpga_gpio.shadow_value & ~GARCIA_GPIO_RW_MASK);
		cur |= (garcia_fpga_read_gpio() & (GARCIA_FPGA_GPIO_SLOT_RESET_1 |
				GARCIA_FPGA_GPIO_SLOT_RESET_2 | GARCIA_FPGA_GPIO_SLOT_RESET_3 |
				GARCIA_FPGA_GPIO_SLOT_RESET_4 | GARCIA_FPGA_GPIO_SLOT_RESET_5));
		if (val & BIT(0)) cur |= GARCIA_FPGA_GPIO_SLOT_MUTE_1;
		if (val & BIT(1)) cur |= GARCIA_FPGA_GPIO_SLOT_MUTE_2;
		if (val & BIT(2)) cur |= GARCIA_FPGA_GPIO_SLOT_MUTE_3;
		if (val & BIT(3)) cur |= GARCIA_FPGA_GPIO_SLOT_MUTE_4;
		if (val & BIT(4)) cur |= GARCIA_FPGA_GPIO_SLOT_MUTE_5;
		garcia_fpga_write_gpio(cur);
	}
	return count;
}

static ssize_t garcia_r_reset(struct class *c, char *buf)
{
	int count = 0;
	u32 val = garcia_fpga_read_gpio();
	val = ((val & GARCIA_FPGA_GPIO_SLOT_RESET_1) ? BIT(0) : 0) |
			((val & GARCIA_FPGA_GPIO_SLOT_RESET_2) ? BIT(1) : 0) |
			((val & GARCIA_FPGA_GPIO_SLOT_RESET_3) ? BIT(2) : 0) |
			((val & GARCIA_FPGA_GPIO_SLOT_RESET_4) ? BIT(3) : 0) |
			((val & GARCIA_FPGA_GPIO_SLOT_RESET_5) ? BIT(4) : 0);
	count = sprintf(buf, "%u\n", val);
	return count;
}

static ssize_t garcia_w_reset(struct class *c, const char * buf, size_t count)
{
	unsigned long int val;
	unsigned long int cur;

	if (strict_strtoul(buf, 0, &val) == 0) {
		cur = (fpga_gpio.shadow_value & ~GARCIA_GPIO_RW_MASK);
		cur |= (garcia_fpga_read_gpio() & (GARCIA_FPGA_GPIO_SLOT_MUTE_1 |
				GARCIA_FPGA_GPIO_SLOT_MUTE_2 | GARCIA_FPGA_GPIO_SLOT_MUTE_3 |
				GARCIA_FPGA_GPIO_SLOT_MUTE_4 | GARCIA_FPGA_GPIO_SLOT_MUTE_5));
		if (val & BIT(0)) cur |= GARCIA_FPGA_GPIO_SLOT_RESET_1;
		if (val & BIT(1)) cur |= GARCIA_FPGA_GPIO_SLOT_RESET_2;
		if (val & BIT(2)) cur |= GARCIA_FPGA_GPIO_SLOT_RESET_3;
		if (val & BIT(3)) cur |= GARCIA_FPGA_GPIO_SLOT_RESET_4;
		if (val & BIT(4)) cur |= GARCIA_FPGA_GPIO_SLOT_RESET_5;
		garcia_fpga_write_gpio(cur);
	}
	return count;
}

static ssize_t garcia_r_inputs(struct class *c, char *buf)
{
	int count = 0;
	u32 val = garcia_fpga_read_gpio() & ~GARCIA_GPIO_RW_MASK;
	count = sprintf(buf, "%x\n", val);
	return count;
}

static struct class_attribute garcia_fpga_class_attrs[] = {
	__ATTR(spimaster, S_IRUGO | S_IWUGO, garcia_r_spimaster, garcia_w_spimaster),
	__ATTR(ssidir, S_IRUGO | S_IWUGO, garcia_r_ssidir, garcia_w_ssidir),
	__ATTR(mute, S_IRUGO | S_IWUGO, garcia_r_mute, garcia_w_mute),
	__ATTR(reset, S_IRUGO | S_IWUGO, garcia_r_reset, garcia_w_reset),
	__ATTR(inputs, S_IRUGO, garcia_r_inputs, NULL),
	__ATTR_NULL,
};

static struct class garcia_fpga_class = {
	.name =		DRIVER_NAME,
	.owner =	THIS_MODULE,
	.class_attrs =	garcia_fpga_class_attrs,
};

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
		r_irq.start = 0;
		rc = 0;
	}

	fpga_gpio.gpioaddr = (u32) ioremap_nocache(r_mem.start, r_mem.end - r_mem.start + 1);
	fpga_gpio.fifo_irq = r_irq.start;
	fpga_gpio.shadow_value = (garcia_fpga_read_gpio() & GARCIA_GPIO_RW_MASK) | GARCIA_FPGA_SLOT_BUF_NOE;
	garcia_fpga_write_gpio(fpga_gpio.shadow_value);
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
