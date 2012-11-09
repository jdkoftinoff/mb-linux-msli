/*
 * Xilinx gpio driver
 *
 * Copyright 2008 Xilinx, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel_stat.h>
#include <linux/xilinx_devices.h>

/* Register Offset Definitions */
#define XGPIO_DATA_OFFSET   (0x0) /* Port 1 Data register  */
#define XGPIO_TRI_OFFSET    (0x4) /* Port 1 I/O direction register  */
#define XGPIO2_DATA_OFFSET  (0x8) /* Port 2 Data Register */
#define XGPIO2_TRI_OFFSET   (0xC) /* Port 2 I/O direction register  */

#define XGPIO_GIE_OFFSET    (0x11C)      /* Global Interrupt Enable Register */
#define XGPIO_GIE_ENABLE    (0x80000000)
#define XGPIO_GIE_DISABLE   (0x00000000)
#define XGPIO_IPIER_OFFSET  (0x128)      /* IP Interrupt Enable Register */
#define XGPIO_IPIER_ENABLE1 (0x00000001)
#define XGPIO_IPIER_ENABLE2 (0x00000002)
#define XGPIO_IPISR_OFFSET  (0x120)      /* IP Interrupt Status Register */
#define XGPIO_IPISR_CH1     (0x00000001)
#define XGPIO_IPISR_CH2     (0x00000002)

struct xgpio_instance {
	struct of_mm_gpio_chip mmchip;
	u32 gpio_state;		/* GPIO state shadow register */
	u32 gpio_dir;		/* GPIO direction shadow register */
	u8  gpio_width; /* GPIO width */
	u32 gpio2_state; /* GPIO2 state shadow register */
	u32 gpio2_dir; /* GPIO2 direction shadow register */
	u8  gpio2_width; /* GPIO2 width */
	u8  isdual;   /* if GPIO is dual port */
	spinlock_t gpio_lock;	/* Lock used for synchronization */
	u32 gpio_ldreg;			/* Last data register value */
	u32 gpio_ldreg2;		/* Last data register value 2 */
	u32 gpio_baseaddr;		/* Base address for this instance */
	u32 gpio_managed_irqs;		/* Number of managed IRQs */
	u32 gpio_managed_irq_start;	/* Starting IRQ number for managed IRQs */
};

static irqreturn_t xgpio_isr(int irq, void *dev_id)
{
	struct xgpio_instance *chip = (struct xgpio_instance *)dev_id;
	struct irqaction *action;
	u32 isr=0;
	u32 dreg=0;
	u32 xbits=0;
	u32 mIrq = NO_IRQ;
	u32 irq_idx;
	int port_width = (chip->isdual) ? (chip->gpio_managed_irqs/2) : chip->gpio_managed_irqs;
	int port1_start = chip->gpio_managed_irq_start + ((chip->isdual) ? (chip->gpio_managed_irqs/2) : 0);
	int port2_start = chip->gpio_managed_irq_start;

	isr = in_be32((void *)(chip->gpio_baseaddr + XGPIO_IPISR_OFFSET));
	out_be32((void *)(chip->gpio_baseaddr + XGPIO_IPISR_OFFSET),isr);
	//printk("isr %08X\n", isr);
	if (isr & XGPIO_IPISR_CH1) {
		dreg = in_be32((void *)(chip->gpio_baseaddr + XGPIO_DATA_OFFSET));
		xbits = chip->gpio_ldreg^dreg;
		//printk("CH1: %08X, %08X, %08X\n", dreg, chip->gpio_ldreg, xbits);
		chip->gpio_ldreg = dreg;	

		for (irq_idx=0; irq_idx<port_width; irq_idx++) {
			if (xbits & (1<<irq_idx)) {
				mIrq = port1_start + irq_idx;

				/* TODO: Register properly so we don't have to call the action directly */
				action = irq_desc[mIrq].action;
				if (action) {
					//printk("CH1: action %u\n", mIrq);
					action->handler(mIrq, action->dev_id);
				}
				kstat_incr_irqs_this_cpu(mIrq, irq_desc);

				//generic_handle_irq(mIrq);
			}
		}
	}

	if (isr & XGPIO_IPISR_CH2) {
		dreg = in_be32((void *)(chip->gpio_baseaddr + XGPIO2_DATA_OFFSET));
		xbits = chip->gpio_ldreg2^dreg;
		//printk("CH2: %08X, %08X, %08X\n", dreg, chip->gpio_ldreg2, xbits);
		chip->gpio_ldreg2 = dreg;	

		for (irq_idx=0; irq_idx<port_width; irq_idx++) {
			if (xbits & (1<<irq_idx)) {
				mIrq = port2_start + irq_idx;

				/* TODO: Register properly so we don't have to call the action directly */
				action = irq_desc[mIrq].action;
				if (action) {
					//printk("CH2: action %u\n", mIrq);
					action->handler(mIrq, action->dev_id);
				}
				kstat_incr_irqs_this_cpu(mIrq, irq_desc);

				//generic_handle_irq(mIrq);
			}
		}
	}
	return IRQ_HANDLED;
}

static int xgpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct xgpio_instance *chip = container_of(
		to_of_mm_gpio_chip(gc), struct xgpio_instance, mmchip);
	//printk("%s: gc: %p base: %08X offset: %d\n",__func__, gc, chip->gpio_baseaddr, offset);

	//printk("%s: Returning:  %d\n",__func__,chip->gpio_managed_irq_start+offset); 
	return chip->gpio_managed_irq_start + offset;
}

/**
 * xgpio_get - Read the specified signal of the GPIO device.
 * @gc:     Pointer to gpio_chip device structure.
 * @gpio:   GPIO signal number.
 *
 * This function reads the specified signal of the GPIO device. It returns 0 if
 * the signal clear, 1 if signal is set or negative value on error.
 */
static int xgpio_get(struct gpio_chip *gc, unsigned int gpio)
{
  struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
  struct xgpio_instance *chip =
      container_of(mm_gc, struct xgpio_instance, mmchip);
   
  if ((chip->isdual) && (gpio >= chip->gpio_width))
  {
    gpio -= chip->gpio_width;
    if(((chip->gpio2_dir) >> gpio) & 1) {
      return (in_be32(mm_gc->regs + XGPIO2_DATA_OFFSET) >> gpio) & 1;
    } else {
      return ((chip->gpio2_state >> gpio) & 1);
    }
  } else {
    if(((chip->gpio_dir) >> gpio) & 1) {
      return (in_be32(mm_gc->regs + XGPIO_DATA_OFFSET) >> gpio) & 1;
    } else {
      return ((chip->gpio_state >> gpio) & 1);
    }
  }
}

/**
 * xgpio_set - Write the specified signal of the GPIO device.
 * @gc:     Pointer to gpio_chip device structure.
 * @gpio:   GPIO signal number.
 * @val:    Value to be written to specified signal.
 *
 * This function writes the specified value in to the specified signal of the
 * GPIO device.
 */
static void xgpio_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
	unsigned long flags;
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
	struct xgpio_instance *chip =
	    container_of(mm_gc, struct xgpio_instance, mmchip);
	spin_lock_irqsave(&chip->gpio_lock, flags);

	/* Write to GPIO signal and set its direction to output */
	if ((chip->isdual) && (gpio >= chip->gpio_width))
	{
  	gpio -= chip->gpio_width;
	  if (val)
	  	chip->gpio2_state |= 1 << gpio;
	  else
	  	chip->gpio2_state &= ~(1 << gpio);
	  out_be32(mm_gc->regs + XGPIO2_DATA_OFFSET, chip->gpio2_state);
  }
  else
  {
	  if (val)
	  	chip->gpio_state |= 1 << gpio;
	  else
	  	chip->gpio_state &= ~(1 << gpio);
	  out_be32(mm_gc->regs + XGPIO_DATA_OFFSET, chip->gpio_state);
  }
	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

/**
 * xgpio_dir_in - Set the direction of the specified GPIO signal as input.
 * @gc:     Pointer to gpio_chip device structure.
 * @gpio:   GPIO signal number.
 *
 * This function sets the direction of specified GPIO signal as input.
 * It returns 0 if direction of GPIO signals is set as input otherwise it
 * returns negative error value.
 */
static int xgpio_dir_in(struct gpio_chip *gc, unsigned int gpio)
{
	unsigned long flags;
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
	struct xgpio_instance *chip =
	    container_of(mm_gc, struct xgpio_instance, mmchip);

	spin_lock_irqsave(&chip->gpio_lock, flags);

	/* Set the GPIO bit in shadow register and set direction as input */
	if ((chip->isdual) && (gpio >= chip->gpio_width))
	{
  	gpio -= chip->gpio_width;
    chip->gpio2_dir |= (1 << gpio);
	  out_be32(mm_gc->regs + XGPIO2_TRI_OFFSET, chip->gpio2_dir);	
  }
  else
  {
  	chip->gpio_dir |= (1 << gpio);
  	out_be32(mm_gc->regs + XGPIO_TRI_OFFSET, chip->gpio_dir);
  }
	spin_unlock_irqrestore(&chip->gpio_lock, flags);

	return 0;
}

/**
 * xgpio_dir_out - Set the direction of the specified GPIO signal as output.
 * @gc:     Pointer to gpio_chip device structure.
 * @gpio:   GPIO signal number.
 * @val:    Value to be written to specified signal.
 *
 * This function sets the direction of specified GPIO signal as output. If all
 * GPIO signals of GPIO chip is configured as input then it returns
 * error otherwise it returns 0.
 */
static int xgpio_dir_out(struct gpio_chip *gc, unsigned int gpio, int val)
{
	unsigned long flags;
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
	struct xgpio_instance *chip =
	    container_of(mm_gc, struct xgpio_instance, mmchip);
	    
	spin_lock_irqsave(&chip->gpio_lock, flags);

	/* Write state of GPIO signal */
	if ((chip->isdual) && (gpio >= chip->gpio_width))
	{
	  gpio -= chip->gpio_width;
	  if (val)
	  	chip->gpio2_state |= 1 << gpio;
	  else
	  	chip->gpio2_state &= ~(1 << gpio);
	  out_be32(mm_gc->regs + XGPIO2_DATA_OFFSET, chip->gpio2_state);
	  /* Clear the GPIO bit in shadow register and set direction as output */
	  chip->gpio2_dir &= (~(1 << gpio));
	  out_be32(mm_gc->regs + XGPIO2_TRI_OFFSET, chip->gpio2_dir);
  }
	else
	{  
	  if (val)
	  	chip->gpio_state |= 1 << gpio;
	  else
	  	chip->gpio_state &= ~(1 << gpio);
	  out_be32(mm_gc->regs + XGPIO_DATA_OFFSET, chip->gpio_state);
	  /* Clear the GPIO bit in shadow register and set direction as output */
	  chip->gpio_dir &= (~(1 << gpio));
	  out_be32(mm_gc->regs + XGPIO_TRI_OFFSET, chip->gpio_dir);
  }
	spin_unlock_irqrestore(&chip->gpio_lock, flags);

	return 0;
}

/**
 * xgpio_save_regs - Set initial values of GPIO pins
 * @mm_gc: pointer to memory mapped GPIO chip structure
 */
static void xgpio_save_regs(struct of_mm_gpio_chip *mm_gc)
{
	struct xgpio_instance *chip =
	    container_of(mm_gc, struct xgpio_instance, mmchip);

	out_be32(mm_gc->regs + XGPIO_DATA_OFFSET, chip->gpio_state);
	out_be32(mm_gc->regs + XGPIO_TRI_OFFSET, chip->gpio_dir);
	out_be32(mm_gc->regs + XGPIO2_DATA_OFFSET, chip->gpio2_state);
	out_be32(mm_gc->regs + XGPIO2_TRI_OFFSET, chip->gpio2_dir);
}

/**
 * xgpio_of_probe - Probe method for the GPIO device.
 * @np: pointer to device tree node
 *
 * This function probes the GPIO device in the device tree. It initializes the
 * driver data structure. It returns 0, if the driver is bound to the GPIO
 * device, or a negative value if there is an error.
 */
static int __devinit xgpio_of_probe(struct device_node *np)
{
	struct xgpio_instance *chip;
	struct of_gpio_chip *ofchip;
	struct irq_desc *desc;
	int status = 0;
	int i;
	const u32 *tree_info;
        u32 isrN = 0;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	ofchip = &chip->mmchip.of_gc;

	/* Check if the GPIO is dual ports */
	tree_info = of_get_property(np, "xlnx,is-dual", NULL);
	if (tree_info)
		chip->isdual = *tree_info;

	/* Update GPIO state shadow register with default value */
	tree_info = of_get_property(np, "xlnx,dout-default", NULL);
	if (tree_info)
		chip->gpio_state = *tree_info;
	
	if (chip->isdual)
	{	
		/* Update GPIO2 state shadow register with default value */
		tree_info = of_get_property(np, "xlnx,dout-default-2", NULL);
		if (tree_info)
			chip->gpio2_state = *tree_info;
	}
	/* Update GPIO direction shadow register with default value */
	chip->gpio_dir = 0xFFFFFFFF; /* By default, all pins are inputs */
	tree_info = of_get_property(np, "xlnx,tri-default", NULL);
	if (tree_info)
		chip->gpio_dir = *tree_info;
		
	if (chip->isdual)
	{
		/* Update GPIO2 direction shadow register with default value */
		chip->gpio2_dir = 0xFFFFFFFF; /* By default, all pins are inputs */
		tree_info = of_get_property(np, "xlnx,tri-default-2", NULL);
		if (tree_info)
			chip->gpio2_dir = *tree_info;
	}

	/* Check device node and parent device node for device width */
	ofchip->gc.ngpio = (chip->isdual)? 64:32; /* By default assume full GPIO controller */
	tree_info = of_get_property(np, "xlnx,gpio-width", NULL);
	if (!tree_info)
		tree_info = of_get_property(np->parent,
					    "xlnx,gpio-width", NULL);
	if (tree_info)
	{
		ofchip->gc.ngpio = *tree_info;
		chip->gpio_width = *tree_info;
	}
	/* if it is dual port, add the width of the second port */	
	if (chip->isdual)
	{
		tree_info = of_get_property(np, "xlnx,gpio2-width", NULL);
		if (tree_info)    
		{             
			ofchip->gc.ngpio += *tree_info;
			chip->gpio2_width = *tree_info;
		}    	
	}

	spin_lock_init(&chip->gpio_lock);

	ofchip->gpio_cells = 2;
	ofchip->gc.direction_input = xgpio_dir_in;
	ofchip->gc.direction_output = xgpio_dir_out;
	ofchip->gc.get = xgpio_get;
	ofchip->gc.set = xgpio_set;
	ofchip->gc.to_irq = xgpio_to_irq;

	chip->mmchip.save_regs = xgpio_save_regs;

	/* Get our base address. */
	tree_info = of_get_property(np, "reg", NULL);
	if(tree_info) {
		chip->gpio_baseaddr = *tree_info;
	}

	/* Call the OF gpio helper to setup and register the GPIO device */
	status = of_mm_gpiochip_add(np, &chip->mmchip);
	if (status) {
		kfree(chip);
		pr_err("%s: error in probe function with status %d\n",
		       np->full_name, status);
		return status;
	}
	pr_info("XGpio: %s: registered\n", np->full_name);

	/* Configure IRQs */
	tree_info = of_get_property(np, "interrupts", NULL);
	if (tree_info)
	{
		/* Setup IRQs managed by this GPIO 1. Currently, GPIO 1 manages 1 GPIO line */

		/* Take over interrupt numbers right after the maximum
		   interrupt number that the microblaze supports (32
		   interrupts from our interrupt space). TODO: register
		   the interrupts for real, or we have a chance of a
		   conflict with other subsystems which also map interrupts
		   in the interrupt controller's interrupt space after
		   the microblaze ones. */
		chip->gpio_managed_irq_start = 32;

                /* The number of IRQs our GPIO controller uses. TODO:
		   assumes a single GPIO controller in the system. */
		chip->gpio_managed_irqs = chip->gpio_width;
		if(chip->isdual) {
			chip->gpio_managed_irqs += chip->gpio2_width;
		}

		/* Set the chip member of the IRQ descriptors that
		   this driver handles to the dummy descriptor. */
		for(i = 0; i < chip->gpio_managed_irqs; i++) {
			desc = irq_to_desc(chip->gpio_managed_irq_start + i);
			/* This driver uses secondary interrupts, which
			   may not have had space allocated for them. */
			if(!desc)
				return -ENOSYS;
			desc->chip = &dummy_irq_chip;
		}

		printk("%s: XGpio: %s: setup %u IRQs, starting at number %u\n",
		       __func__, np->full_name, chip->gpio_managed_irqs,
                       chip->gpio_managed_irq_start);
		chip->gpio_ldreg = in_be32((void *)(chip->gpio_baseaddr + XGPIO_DATA_OFFSET));
		if (chip->isdual) {
			chip->gpio_ldreg2 = in_be32((void *)(chip->gpio_baseaddr + XGPIO2_DATA_OFFSET));
		}

		isrN = *tree_info;
		status = request_irq(isrN, xgpio_isr, 0, "xgpio", chip);
		if (status) {
			printk("%s: Unable to claim irq %d; error %d\n",
			       __func__, *tree_info, status);
			kfree(chip);
			return status;
		}

		/* Enable GPIO Interrupt(s) */
		if (chip->isdual) {
			out_be32((void *)(chip->gpio_baseaddr + XGPIO_IPIER_OFFSET),XGPIO_IPIER_ENABLE1|XGPIO_IPIER_ENABLE2);
		} else {
			out_be32((void *)(chip->gpio_baseaddr + XGPIO_IPIER_OFFSET),XGPIO_IPIER_ENABLE1);
		}

		/* Enable interrupts on GPIO */
		out_be32((void *)(chip->gpio_baseaddr + XGPIO_GIE_OFFSET),XGPIO_GIE_ENABLE);
	}

	printk("%s: Configured GPIO at Base address 0x%x IRQ %u\n", __func__, chip->gpio_baseaddr, isrN);

	return 0;
}

static struct of_device_id xgpio_of_match[] __devinitdata = {
	{ .compatible = "xlnx,xps-gpio-1.00.a", 
	  .compatible = "xlnx,xps-gpio-2.00.a",},
	{ /* end of list */ },
};

static int __init xgpio_init(void)
{
	struct device_node *np;

	for_each_matching_node(np, xgpio_of_match)
		xgpio_of_probe(np);

	return 0;
}

/* Make sure we get initialized before anyone else tries to use us */
subsys_initcall(xgpio_init);
/* No exit call at the moment as we cannot unregister of GPIO chips */

MODULE_AUTHOR("Xilinx, Inc.");
MODULE_DESCRIPTION("Xilinx GPIO driver");
MODULE_LICENSE("GPL");
