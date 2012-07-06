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

/* Register Offset Definitions */
#define XGPIO_DATA_OFFSET   (0x0)	/* Port 1 Data register  */
#define XGPIO_TRI_OFFSET    (0x4)	/* Port 1 I/O direction register  */
#define XGPIO2_DATA_OFFSET  (0x8) /* Port 2 Data Register */
#define XGPIO2_TRI_OFFSET   (0xC)	/* Port 2 I/O direction register  */

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
};

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
	int status = 0;
	const u32 *tree_info;
	u8 isdual = 0;

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

	chip->mmchip.save_regs = xgpio_save_regs;

	/* Call the OF gpio helper to setup and register the GPIO device */
	status = of_mm_gpiochip_add(np, &chip->mmchip);
	if (status) {
		kfree(chip);
		pr_err("%s: error in probe function with status %d\n",
		       np->full_name, status);
		return status;
	}
	pr_info("XGpio: %s: registered\n", np->full_name);
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
