/*
 *  linux/drivers/net/labx_avb/labrinth_legacy_bridge.c
 *
 *  Legacy packet bridge configuration driver for Labrinth
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *
 *  Copyright (C) 2011 Lab X Technologies LLC, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "labrinth_legacy_bridge.h"
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <xio.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF


/* Driver name and the revision range of hardware expected.
 * This driver will work with revision 1.1 only.
 */
#define DRIVER_NAME "labrinth_legacy_bridge"

/* Maximum number of legacy_bridges and instance count */
#define MAX_INSTANCES 4
static uint32_t instanceCount;

/* Number of milliseconds we will wait before bailing out of a synced write */
#define SYNCED_WRITE_TIMEOUT_MSECS  (100)

#if 0
#define DBG(f, x...) printk(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

/*
 * Character device hook functions
 */

static int legacy_bridge_open(struct inode *inode, struct file *filp) {
  struct legacy_bridge *bridge = (struct legacy_bridge*)filp->private_data;
  unsigned long flags;
  int returnValue = 0;

  /* Lock the mutex and ensure there is only one owner */
  preempt_disable();
  spin_lock_irqsave(&bridge->mutex, flags);
  if(bridge->opened) {
    returnValue = -1;
  } else {
    bridge->opened = true;
  }

  spin_unlock_irqrestore(&bridge->mutex, flags);
  preempt_enable();
  
  return(returnValue);
}

static int legacy_bridge_release(struct inode *inode, struct file *filp) {
  struct legacy_bridge *bridge = (struct legacy_bridge*)filp->private_data;
  unsigned long flags;

  preempt_disable();
  spin_lock_irqsave(&bridge->mutex, flags);
  bridge->opened = false;

  spin_unlock_irqrestore(&bridge->mutex, flags);
  preempt_enable();
  return(0);
}

/* I/O control operations for the driver */
static int legacy_bridge_ioctl(struct inode *inode, 
                               struct file *filp,
                               unsigned int command, 
                               unsigned long arg) {
  // Switch on the request
  int returnValue = 0;
  struct legacy_bridge *bridge = (struct legacy_bridge*)filp->private_data;

  switch(command) {
  case IOC_CONFIG_MAC_FILTER:
    break;
    
  default:
    returnValue = -EINVAL;
  }

  /* Return an error code appropriate to the command */
  return(returnValue);
}

/* Character device file operations structure */
static struct file_operations legacy_bridge_fops = {
  .open	   = legacy_bridge_open,
  .release = legacy_bridge_release,
  .ioctl   = legacy_bridge_ioctl,
  .owner   = THIS_MODULE,
};

/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 * This is exported to allow polymorphic drivers to invoke it.
 * @param name - Base name of the instance
 * @param pdev - Platform device structure
 * @param addressRange - Resource describing the hardware's I/O range
 */
static int legacy_bridge_probe(const char *name, 
                               struct platform_device *pdev,
                               struct resource *addressRange) {
  struct legacy_bridge *bridge;
  int returnValue;

  /* Create and populate a device structure */
  bridge = (struct legacy_bridge*) kmalloc(sizeof(struct legacy_bridge), GFP_KERNEL);
  if(!bridge) return(-ENOMEM);

  /* Request and map the device's I/O memory region into uncacheable space */
  bridge->physicalAddress = addressRange->start;
  bridge->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
  snprintf(bridge->name, NAME_MAX_SIZE, "%s%d", name, instanceCount++);
  bridge->name[NAME_MAX_SIZE - 1] = '\0';
  if(request_mem_region(bridge->physicalAddress, bridge->addressRangeSize,
                        bridge->name) == NULL) {
    returnValue = -ENOMEM;
    goto free;
  }

  bridge->virtualAddress = 
    (void*) ioremap_nocache(bridge->physicalAddress, bridge->addressRangeSize);
  if(!bridge->virtualAddress) {
    returnValue = -ENOMEM;
    goto release;
  }

  /* Announce the device */
  printk(KERN_INFO "%s: Found Lab X AVB Legacy Bridge at 0x%08X, ",
         bridge->name, 
         (uint32_t)bridge->physicalAddress);

  /* Initialize other resources */
  spin_lock_init(&bridge->mutex);
  bridge->opened = false;

  /* Register as a misc device */
  bridge->miscdev.minor = MISC_DYNAMIC_MINOR;
  bridge->miscdev.name = bridge->name;
  bridge->miscdev.fops = &legacy_bridge_fops;
  returnValue = misc_register(&bridge->miscdev);
  if(returnValue) {
    printk(KERN_WARNING DRIVER_NAME ": Unable to register misc device.\n");
    goto unmap;
  }

  /* Provide navigation between the device structures */
  platform_set_drvdata(pdev, bridge);
  bridge->pdev = pdev;

  /* Return success */
  return(0);

 unmap:
  iounmap(bridge->virtualAddress);
 release:
  release_mem_region(bridge->physicalAddress, bridge->addressRangeSize);
 free:
  kfree(bridge);
  return(returnValue);
}

#ifdef CONFIG_OF

static int legacy_bridge_platform_remove(struct platform_device *pdev);

/* Probe for registered devices */
static int __devinit legacy_bridge_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
  struct resource r_mem_struct  = {};
  struct resource *addressRange = &r_mem_struct;
  struct platform_device *pdev  = to_platform_device(&ofdev->dev);
  const char *name = dev_name(&ofdev->dev);
  int rc = 0;

  /* Obtain the resources for this instance */
  rc = of_address_to_resource(ofdev->node, 0, addressRange);
  if(rc) {
    dev_warn(&ofdev->dev, "Invalid address\n");
    return(rc);
  }

  /* Dispatch to the generic function */
  return(0);
  /*  return(legacy_bridge_probe(name, pdev, addressRange)); */
}

static int __devexit legacy_bridge_of_remove(struct of_device *dev)
{
  struct platform_device *pdev = to_platform_device(&dev->dev);
  legacy_bridge_platform_remove(pdev);
  return(0);
}

static struct of_device_id legacy_bridge_of_match[] = {
  { .compatible = "xlnx,labrinth-legacy-bridge-1.00.a", },
  { /* end of list */ },
};


static struct of_platform_driver of_legacy_bridge_driver = {
  .name	       = DRIVER_NAME,
  .match_table = legacy_bridge_of_match,
  .probe       = legacy_bridge_of_probe,
  .remove      = __devexit_p(legacy_bridge_of_remove),
};

#endif /* CONFIG_OF */

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int legacy_bridge_platform_probe(struct platform_device *pdev) {
  struct resource *addressRange;

  /* Obtain the resources for this instance */
  addressRange = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!addressRange) {
    printk(KERN_ERR "%s: IO resource address not found.\n", pdev->name);
    return(-ENXIO);
  }

  /* Dispatch to the generic function */
  return(legacy_bridge_probe(pdev->name, pdev, addressRange));
}

/* Remove a previously-probed device */
static int legacy_bridge_remove(struct legacy_bridge *bridge) {
  iounmap(bridge->virtualAddress);
  release_mem_region(bridge->physicalAddress, bridge->addressRangeSize);
  kfree(bridge);
  return(0);
}

static int legacy_bridge_platform_remove(struct platform_device *pdev) {
  struct legacy_bridge *bridge;

  /* Get a handle to the legacy_bridge and begin shutting it down */
  bridge = platform_get_drvdata(pdev);
  if(!bridge) return(-1);
  return(legacy_bridge_remove(bridge));
}

/* Platform device driver structure */
static struct platform_driver legacy_bridge_driver = {
  .probe  = legacy_bridge_platform_probe,
  .remove = legacy_bridge_platform_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __init legacy_bridge_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": Biamp Labrinth Legacy Bridge driver\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&of_legacy_bridge_driver);
#endif
 
  /* Register as a platform device driver */
  instanceCount = 0;
  if((returnValue = platform_driver_register(&legacy_bridge_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }

  return(0);
}

static void __exit legacy_bridge_driver_exit(void)
{
  /* Unregister as a platform device driver */
  platform_driver_unregister(&legacy_bridge_driver);
}

module_init(legacy_bridge_driver_init);
module_exit(legacy_bridge_driver_exit);

MODULE_AUTHOR("Eldridge M. Mount IV <eldridge.mount@labxtechnologies.com>");
MODULE_DESCRIPTION("Biamp Labrinth Legacy Bridge driver");
MODULE_LICENSE("GPL");
