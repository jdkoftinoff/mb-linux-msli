/*
 *  linux/drivers/spi/biamp_spi_mailbox.c
 *
 *  Biamp SPI mailbox peripheral driver
 *
 *  Written by Albert M. Hajjar (albert.hajjar@labxtechnologies.com)
 *
 *  Copyright (C) 2010 Lab X Technologies LLC, All Rights Reserved.
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

#include "biamp_spi_mailbox.h"
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <xio.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF


/* Driver name and the revision range of hardware expected.
 * This driver will work with revision 1.1 only.
 */
#define DRIVER_NAME "biamp_spi_mailbox"

/* Major device number for the driver */
#define DRIVER_MAJOR 255

/* Maximum number of packetizers and instance count */
#define MAX_INSTANCES 1
static uint32_t instanceCount;

#if 0
#define DBG(f, x...) printk(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

/* Disables the passed instance */
static void disable_mailbox(struct spi_mailbox *mailbox) {
  uint32_t controlRegister;

  DBG("Disabling the mailbox\n");
  controlRegister = XIo_In32(REGISTER_ADDRESS(mailbox, CONTROL_REG));
  controlRegister &= ~MAILBOX_ENABLE;
  XIo_Out32(REGISTER_ADDRESS(mailbox, CONTROL_REG), controlRegister);
}

/* Enables the passed instance */
static void enable_mailbox(struct spi_mailbox *mailbox) {
  uint32_t controlRegister;

  DBG("Enabling the mailbox\n");
  controlRegister = XIo_In32(REGISTER_ADDRESS(mailbox, CONTROL_REG));
  controlRegister |= MAILBOX_ENABLE;
  XIo_Out32(REGISTER_ADDRESS(mailbox, CONTROL_REG), controlRegister);
}

/* Resets the state of the passed instance */
static void reset_mailbox(struct spi_mailbox *mailbox) {
  /* Disable the instance, and wipe its registers */
  disable_mailbox(mailbox);
}

/*
 * Character device hook functions
 */

static int spi_mailbox_open(struct inode *inode, struct file *filp)
{
  struct spi_mailbox *mailbox;
  unsigned long flags;
  int returnValue = 0;

  mailbox = container_of(inode->i_cdev, struct spi_mailbox, cdev);
  filp->private_data = mailbox;

  /* Lock the mutex and ensure there is only one owner */
  preempt_disable();
  spin_lock_irqsave(&mailbox->mutex, flags);
  if(mailbox->opened) {
    returnValue = -1;
  } else {
    mailbox->opened = true;
  }

  spin_unlock_irqrestore(&mailbox->mutex, flags);
  preempt_enable();
  
  return(returnValue);
}

static int spi_mailbox_release(struct inode *inode, struct file *filp)
{
  struct spi_mailbox *mailbox = (struct spi_mailbox*)filp->private_data;
  unsigned long flags;

  preempt_disable();
  spin_lock_irqsave(&mailbox->mutex, flags);
  mailbox->opened = false;
  spin_unlock_irqrestore(&mailbox->mutex, flags);
  preempt_enable();
  return(0);
}

/* Character device file operations structure */
static struct file_operations spi_mailbox_fops = {
  .open	   = spi_mailbox_open,
  .release = spi_mailbox_release,
  .owner   = THIS_MODULE,
};

/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 */
static int spi_mailbox_probe(const char *name, 
                                  struct platform_device *pdev,
                                  struct resource *addressRange) {
				  
  struct spi_mailbox *mailbox;
  int returnValue;

  /* Create and populate a device structure */
  mailbox = (struct spi_mailbox*) kmalloc(sizeof(struct spi_mailbox), GFP_KERNEL);
  if(!mailbox) return(-ENOMEM);

  /* Request and map the device's I/O memory region into uncacheable space */
  mailbox->physicalAddress = addressRange->start;
  mailbox->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
  snprintf(mailbox->name, NAME_MAX_SIZE, "%s", name);
  mailbox->name[NAME_MAX_SIZE - 1] = '\0';
  if(request_mem_region(mailbox->physicalAddress, mailbox->addressRangeSize,
                        mailbox->name) == NULL) {
    returnValue = -ENOMEM;
    goto free;
  }

  mailbox->virtualAddress = 
    (void*) ioremap_nocache(mailbox->physicalAddress, mailbox->addressRangeSize);
  if(!mailbox->virtualAddress) {
    returnValue = -ENOMEM;
    goto release;
  }

  /* Announce the device */
  printk(KERN_INFO "%s: Found Biamp mailbox at 0x%08X, ",
         mailbox->name, (uint32_t)mailbox->physicalAddress);

  /* Initialize other resources */
  spin_lock_init(&mailbox->mutex);
  mailbox->opened = false;

  /* Provide navigation between the device structures */
  platform_set_drvdata(pdev, mailbox);
  mailbox->pdev = pdev;

  /* Reset the state of the mailbox */
  reset_mailbox(mailbox);

  /* Add as a character device to make the instance available for use */
  cdev_init(&mailbox->cdev, &spi_mailbox_fops);
  mailbox->cdev.owner = THIS_MODULE;
  kobject_set_name(&mailbox->cdev.kobj, "%s%d", pdev->name, pdev->id);
  mailbox->instanceNumber = instanceCount++;
  cdev_add(&mailbox->cdev, MKDEV(DRIVER_MAJOR, mailbox->instanceNumber), 1);

  /* Return success */
  return(0);

 unmap:
  iounmap(mailbox->virtualAddress);
 release:
  release_mem_region(mailbox->physicalAddress, mailbox->addressRangeSize);
 free:
  kfree(mailbox);
  return(returnValue);
}

#ifdef CONFIG_OF
static int spi_mailbox_platform_remove(struct platform_device *pdev);

/* Probe for registered devices */
static int __devinit spi_mailbox_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
  struct resource r_mem_struct = {};
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
  return(spi_mailbox_probe(name, pdev, addressRange));
}

static int __devexit spi_mailbox_of_remove(struct of_device *dev)
{
  struct platform_device *pdev = to_platform_device(&dev->dev);
  spi_mailbox_platform_remove(pdev);
  return(0);
}

static struct of_device_id mailbox_of_match[] = {
  { .compatible = "xlnx,biamp-spi-mailbox-1.00.a", },
  { /* end of list */ },
};


static struct of_platform_driver of_spi_mailbox_driver = {
  .name		= DRIVER_NAME,
  .match_table  = mailbox_of_match,
  .probe   	= spi_mailbox_of_probe,
  .remove       = __devexit_p(spi_mailbox_of_remove),
};
#endif

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int spi_mailbox_platform_probe(struct platform_device *pdev) {
  struct resource *addressRange;

  /* Obtain the resources for this instance */
  addressRange = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!addressRange) {
    printk(KERN_ERR "%s: IO resource address not found.\n", pdev->name);
    return(-ENXIO);
  }

  /* Dispatch to the generic function */
  return(spi_mailbox_probe(pdev->name, pdev, addressRange));
}

static int spi_mailbox_platform_remove(struct platform_device *pdev) {
  struct spi_mailbox *mailbox;

  /* Get a handle to the mailbox and begin shutting it down */
  mailbox = platform_get_drvdata(pdev);
  if(!mailbox) return(-1);
  cdev_del(&mailbox->cdev);
  reset_mailbox(mailbox);
  iounmap(mailbox->virtualAddress);
  release_mem_region(mailbox->physicalAddress, mailbox->addressRangeSize);
  kfree(mailbox);
  return(0);
}

/* Platform device driver structure */
static struct platform_driver spi_mailbox_driver = {
  .probe  = spi_mailbox_platform_probe,
  .remove = spi_mailbox_platform_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __init spi_mailbox_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": SPI Mailbox driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Lab X Technologies, LLC\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&of_spi_mailbox_driver);
#endif
 
  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&spi_mailbox_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }

  /* Allocate a range of major / minor device numbers for use */
  instanceCount = 0;
  if((returnValue = register_chrdev_region(MKDEV(DRIVER_MAJOR, 0),MAX_INSTANCES, DRIVER_NAME)) < 0) { 
    printk(KERN_INFO DRIVER_NAME "Failed to allocate character device range\n");
  }
  return(0);
}

static void __exit spi_mailbox_driver_exit(void)
{
  unregister_chrdev_region(MKDEV(DRIVER_MAJOR, 0),MAX_INSTANCES);

  /* Unregister as a platform device driver */
  platform_driver_unregister(&spi_mailbox_driver);
}

module_init(spi_mailbox_driver_init);
module_exit(spi_mailbox_driver_exit);

MODULE_AUTHOR("Albert M. Hajjar <albert.hajjar@labxtechnologies.com>");
MODULE_DESCRIPTION("Biamp SPI Mailbox driver");
MODULE_LICENSE("GPL");
