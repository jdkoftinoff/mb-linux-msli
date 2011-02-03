/*
 *  linux/drivers/net/labx_avb/labx_redundancy_switch.c
 *
 *  Lab X Technologies AVB redundancy switch driver
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

#include "labx_redundancy_switch.h"
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
#define DRIVER_NAME "labx_redundancy_switch"
#define DRIVER_VERSION_MIN  0x10
#define DRIVER_VERSION_MAX  0x10

/* Major device number for the driver */
#define DRIVER_MAJOR 248

/* Maximum number of redundancy_switchs and instance count */
#define MAX_INSTANCES 64
static uint32_t instanceCount;

/* Number of milliseconds we will wait before bailing out of a synced write */
#define SYNCED_WRITE_TIMEOUT_MSECS  (100)

#if 0
#define DBG(f, x...) printk(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

/* Disables the passed instance */
static void disable_redundancy_switch(struct redundancy_switch *redundancy_switch) {
  uint32_t controlRegister;

  DBG("Disabling the redundancy switch\n");

  controlRegister = XIo_In32(REGISTER_ADDRESS(redundancy_switch, CONTROL_REG));
  controlRegister &= ~SWITCH_ENABLE;
  XIo_Out32(REGISTER_ADDRESS(redundancy_switch, CONTROL_REG), controlRegister);
}

/* Enables the passed instance */
static void enable_redundancy_switch(struct redundancy_switch *redundancy_switch) {
  uint32_t controlRegister;

  DBG("Enabling the redundancy switch\n");

  controlRegister = XIo_In32(REGISTER_ADDRESS(redundancy_switch, CONTROL_REG));
  controlRegister |= SWITCH_ENABLE;
  XIo_Out32(REGISTER_ADDRESS(redundancy_switch, CONTROL_REG), controlRegister);
}

/* Collects the stream status from the hardware */
static void get_stream_status(struct redundancy_switch *redundancy_switch,
                              uint32_t *statusWords) {
  statusWords[0] = XIo_In32(REGISTER_ADDRESS(redundancy_switch, STREAM_STATUS_0_REG));
  statusWords[1] = XIo_In32(REGISTER_ADDRESS(redundancy_switch, STREAM_STATUS_1_REG));
  statusWords[2] = XIo_In32(REGISTER_ADDRESS(redundancy_switch, STREAM_STATUS_2_REG));
  statusWords[3] = XIo_In32(REGISTER_ADDRESS(redundancy_switch, STREAM_STATUS_3_REG));
}

/* TODO: Implement the stream "bump" command */
#if 0
/* Waits for a bump command to complete, using either polling or
 * an interrupt-driven waitqueue.
 */
static int32_t await_bump_command(struct redundancy_switch *redundancy_switch) {
  int32_t returnValue = 0;

  /* Determine whether to use an interrupt or polling */
  if(redundancy_switch->irq != NO_IRQ_SUPPLIED) {
    int32_t waitResult;

    /* Place ourselves onto a wait queue if the synced write is flagged as
     * pending by the hardware, as this indicates that the microengine is active,
     * and we need to wait for it to finish a pass through all the microcode 
     * before the hardware will commit the write to its destination (a register
     * or microcode RAM.)  If the engine is inactive or the write already 
     * committed, we will not actually enter the wait queue.
     */
    waitResult =
      wait_event_interruptible_timeout(redundancy_switch->syncedWriteQueue,
				       ((XIo_In32(REGISTER_ADDRESS(redundancy_switch, SYNC_REG)) & 
					 SYNC_PENDING) == 0),
				       msecs_to_jiffies(SYNCED_WRITE_TIMEOUT_MSECS));

    /* If the wait returns zero, then the timeout elapsed; if negative, a signal
     * interrupted the wait.
     */
    if(waitResult == 0) returnValue = -ETIMEDOUT;
    else if(waitResult < 0) returnValue = -EAGAIN;
  } else {
    /* No interrupt was supplied during the device probe, simply poll for the bit. */
    /* TODO: Need to introduce timeout semantics to this mode as well! */
    while(XIo_In32(REGISTER_ADDRESS(redundancy_switch, SYNC_REG)) & SYNC_PENDING);
  }

  /* Return success or "timed out" */
  return(returnValue);
}

#endif


/* Interrupt service routine for the instance */
static irqreturn_t labx_redundancy_switch_interrupt(int irq, void *dev_id) {
  struct redundancy_switch *redundancy_switch = (struct redundancy_switch*) dev_id;
  uint32_t maskedFlags;
  uint32_t irqMask;

  /* Read the interrupt flags and immediately clear them */
  maskedFlags = XIo_In32(REGISTER_ADDRESS(redundancy_switch, IRQ_FLAGS_REG));
  irqMask = XIo_In32(REGISTER_ADDRESS(redundancy_switch, IRQ_MASK_REG));
  maskedFlags &= irqMask;
  XIo_Out32(REGISTER_ADDRESS(redundancy_switch, IRQ_FLAGS_REG), maskedFlags);

  /* Detect the bump acknowledge IRQ */
  if((maskedFlags & BUMP_ACK_IRQ) != 0) {
    /* Wake up all threads waiting for a synchronization event */
    wake_up_interruptible(&(redundancy_switch->bumpWaitQueue));
  }

  return(IRQ_HANDLED);
}

/*
 * Character device hook functions
 */

static int redundancy_switch_open(struct inode *inode, struct file *filp)
{
  struct redundancy_switch *redundancy_switch;
  unsigned long flags;
  int returnValue = 0;

  redundancy_switch = container_of(inode->i_cdev, struct redundancy_switch, cdev);
  filp->private_data = redundancy_switch;

  /* Lock the mutex and ensure there is only one owner */
  preempt_disable();
  spin_lock_irqsave(&redundancy_switch->mutex, flags);
  if(redundancy_switch->opened) {
    returnValue = -1;
  } else {
    redundancy_switch->opened = true;
  }

  /* Invoke the open() operation on the derived driver, if there is one */
  if((redundancy_switch->derivedFops != NULL) && 
     (redundancy_switch->derivedFops->open != NULL)) {
    redundancy_switch->derivedFops->open(inode, filp);
  }

  spin_unlock_irqrestore(&redundancy_switch->mutex, flags);
  preempt_enable();
  
  return(returnValue);
}

static int redundancy_switch_release(struct inode *inode, struct file *filp)
{
  struct redundancy_switch *redundancy_switch = (struct redundancy_switch*)filp->private_data;
  unsigned long flags;

  preempt_disable();
  spin_lock_irqsave(&redundancy_switch->mutex, flags);
  redundancy_switch->opened = false;

  /* Invoke the release() operation on the derived driver, if there is one */
  if((redundancy_switch->derivedFops != NULL) && 
     (redundancy_switch->derivedFops->release != NULL)) {
    redundancy_switch->derivedFops->release(inode, filp);
  }

  spin_unlock_irqrestore(&redundancy_switch->mutex, flags);
  preempt_enable();
  return(0);
}

/* I/O control operations for the driver */
static int redundancy_switch_ioctl(struct inode *inode, 
                                   struct file *filp,
                                   unsigned int command, 
                                   unsigned long arg) {
  // Switch on the request
  int returnValue = 0;
  struct redundancy_switch *redundancy_switch = (struct redundancy_switch*)filp->private_data;

  switch(command) {
  case IOC_DISABLE_RED_SWITCH:
    disable_redundancy_switch(redundancy_switch);
    break;
    
  case IOC_ENABLE_RED_SWITCH:
    enable_redundancy_switch(redundancy_switch);
    break;

  case IOC_GET_RED_SWITCH_STATUS:
    {
      uint32_t statusWords[STREAM_STATUS_WORDS];

      /* Get the stream status, then copy into the userspace pointer */
      get_stream_status(redundancy_switch, statusWords);
      if(copy_to_user((void __user*)arg, statusWords, 
                      (STREAM_STATUS_WORDS * sizeof(uint32_t))) != 0) {
        return(-EFAULT);
      }
    }
    break;
    
  default:
    /* We don't recognize this command; give our derived driver's ioctl()
     * a crack at it, if one exists.
     */
    if((redundancy_switch->derivedFops != NULL) && 
       (redundancy_switch->derivedFops->ioctl != NULL)) {
      returnValue = redundancy_switch->derivedFops->ioctl(inode, filp, command, arg);
    } else returnValue = -EINVAL;
  }

  /* Return an error code appropriate to the command */
  return(returnValue);
}

/* Character device file operations structure */
static struct file_operations redundancy_switch_fops = {
  .open	   = redundancy_switch_open,
  .release = redundancy_switch_release,
  .ioctl   = redundancy_switch_ioctl,
  .owner   = THIS_MODULE,
};

/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 * This is exported to allow polymorphic drivers to invoke it.
 * @param name - Name of the instance
 * @param pdev - Platform device structure
 * @param addressRange - Resource describing the hardware's I/O range
 * @param irq          - Resource describing the hardware's IRQ
 * @param derivedFops  - File operations to delegate to, NULL if unused
 * @param derivedData  - Pointer for derived driver to make use of
 * @param newInstance  - Pointer to the new driver instance, NULL if unused
 */
int redundancy_switch_probe(const char *name, 
                            struct platform_device *pdev,
                            struct resource *addressRange,
                            struct resource *irq,
                            struct file_operations *derivedFops,
                            void *derivedData,
                            struct redundancy_switch **newInstance) {
  struct redundancy_switch *redundancy_switch;
  uint32_t versionWord;
  uint32_t versionMajor;
  uint32_t versionMinor;
  uint32_t versionCompare;
  int returnValue;

  /* Create and populate a device structure */
  redundancy_switch = (struct redundancy_switch*) kmalloc(sizeof(struct redundancy_switch), GFP_KERNEL);
  if(!redundancy_switch) return(-ENOMEM);

  /* Request and map the device's I/O memory region into uncacheable space */
  redundancy_switch->physicalAddress = addressRange->start;
  redundancy_switch->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
  snprintf(redundancy_switch->name, NAME_MAX_SIZE, "%s", name);
  redundancy_switch->name[NAME_MAX_SIZE - 1] = '\0';
  if(request_mem_region(redundancy_switch->physicalAddress, redundancy_switch->addressRangeSize,
                        redundancy_switch->name) == NULL) {
    returnValue = -ENOMEM;
    goto free;
  }

  redundancy_switch->virtualAddress = 
    (void*) ioremap_nocache(redundancy_switch->physicalAddress, redundancy_switch->addressRangeSize);
  if(!redundancy_switch->virtualAddress) {
    returnValue = -ENOMEM;
    goto release;
  }

  /* Ensure that the engine and its interrupts are disabled */
  disable_redundancy_switch(redundancy_switch);
  XIo_Out32(REGISTER_ADDRESS(redundancy_switch, IRQ_MASK_REG), NO_IRQS);

  /* Retain the IRQ and register our handler, if an IRQ resource was supplied. */
  if(irq != NULL) {
    redundancy_switch->irq = irq->start;
    returnValue = request_irq(redundancy_switch->irq, &labx_redundancy_switch_interrupt, IRQF_DISABLED, 
                              redundancy_switch->name, redundancy_switch);
    if (returnValue) {
      printk(KERN_ERR "%s : Could not allocate Lab X Audio Redundancy_Switch interrupt (%d).\n",
             redundancy_switch->name, redundancy_switch->irq);
      goto unmap;
    }
  } else redundancy_switch->irq = NO_IRQ_SUPPLIED;

  /* Inspect and check the version to ensure it lies within the range of hardware
   * we support.
   */
  versionWord = XIo_In32(REGISTER_ADDRESS(redundancy_switch, REVISION_REG));
  versionMajor = ((versionWord >> REVISION_FIELD_BITS) & REVISION_FIELD_MASK);
  versionMinor = (versionWord & REVISION_FIELD_MASK);
  versionCompare = ((versionMajor << REVISION_FIELD_BITS) | versionMinor);
  if((versionCompare < DRIVER_VERSION_MIN) | 
     (versionCompare > DRIVER_VERSION_MAX)) {
    printk(KERN_INFO "%s: Found incompatible hardware version %d.%d at 0x%08X\n",
           redundancy_switch->name, versionMajor, versionMinor, (uint32_t)redundancy_switch->physicalAddress);
    returnValue = -ENXIO;
    goto unmap;
  }
  redundancy_switch->versionMajor = versionMajor;
  redundancy_switch->versionMinor = versionMinor;

  /* Announce the device */
  printk(KERN_INFO "%s: Found Lab X AVB Redundancy Switch %d.%d at 0x%08X, ",
         redundancy_switch->name, versionMajor, versionMinor, 
         (uint32_t)redundancy_switch->physicalAddress);
  if(redundancy_switch->irq == NO_IRQ_SUPPLIED) {
    printk("polled commands\n");
  } else {
    printk("IRQ %d\n", redundancy_switch->irq);
  }

  /* Initialize other resources */
  spin_lock_init(&redundancy_switch->mutex);
  redundancy_switch->opened = false;

  /* Provide navigation between the device structures */
  platform_set_drvdata(pdev, redundancy_switch);
  redundancy_switch->pdev = pdev;

  /* Reset the state of the redundancy_switch */
  disable_redundancy_switch(redundancy_switch);

  /* Add as a character device to make the instance available for use */
  cdev_init(&redundancy_switch->cdev, &redundancy_switch_fops);
  redundancy_switch->cdev.owner = THIS_MODULE;
  redundancy_switch->instanceNumber = instanceCount++;
  kobject_set_name(&redundancy_switch->cdev.kobj, "%s.%d", redundancy_switch->name, redundancy_switch->instanceNumber);
  cdev_add(&redundancy_switch->cdev, MKDEV(DRIVER_MAJOR, redundancy_switch->instanceNumber), 1);

  /* Initialize the waitqueue used for bump commands */
  init_waitqueue_head(&(redundancy_switch->bumpWaitQueue));

  /* Now that the device is configured, enable interrupts if they are to be used */
  if(redundancy_switch->irq != NO_IRQ_SUPPLIED) {
    XIo_Out32(REGISTER_ADDRESS(redundancy_switch, IRQ_FLAGS_REG), ALL_IRQS);
  }

  /* Retain any derived file operations & data to dispatch to */
  redundancy_switch->derivedFops = derivedFops;
  redundancy_switch->derivedData = derivedData;

  /* Return success, setting the return pointer if valid */
  if(newInstance != NULL) *newInstance = redundancy_switch;
  return(0);

 unmap:
  iounmap(redundancy_switch->virtualAddress);
 release:
  release_mem_region(redundancy_switch->physicalAddress, redundancy_switch->addressRangeSize);
 free:
  kfree(redundancy_switch);
  return(returnValue);
}

#ifdef CONFIG_OF

static int redundancy_switch_platform_remove(struct platform_device *pdev);

/* Probe for registered devices */
static int __devinit redundancy_switch_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
  struct resource r_mem_struct = {};
  struct resource r_irq_struct = {};
  struct resource *addressRange = &r_mem_struct;
  struct resource *irq          = &r_irq_struct;
  struct platform_device *pdev  = to_platform_device(&ofdev->dev);
  const char *name = dev_name(&ofdev->dev);
  int rc = 0;

  /* Obtain the resources for this instance */
  rc = of_address_to_resource(ofdev->node, 0, addressRange);
  if(rc) {
    dev_warn(&ofdev->dev, "Invalid address\n");
    return(rc);
  }

  rc = of_irq_to_resource(ofdev->node, 0, irq);
  if(rc == NO_IRQ) {
    /* No IRQ was defined; null the resource pointer to indicate polled mode */
    irq = NULL;
    return(rc);
  }

  /* Dispatch to the generic function */
  return(redundancy_switch_probe(name, pdev, addressRange, irq, NULL, NULL, NULL));
}

static int __devexit redundancy_switch_of_remove(struct of_device *dev)
{
  struct platform_device *pdev = to_platform_device(&dev->dev);
  redundancy_switch_platform_remove(pdev);
  return(0);
}


/* Directly compatible with "pure" Lab X Audio Redundancy_Switch peripherals.
 *
 * An advanced use of the Lab X AVB Redundancy_Switch is to incorporate
 * it into a custom peripheral, incorporating audio interface & other
 * additional functionality into the core.  In this case, a custom
 * driver directly matches the instance, and registers character device
 * hooks with this driver.
 */
static struct of_device_id redundancy_switch_of_match[] = {
  { .compatible = "xlnx,labx-redundancy-switch-1.00.a", },
  { /* end of list */ },
};


static struct of_platform_driver of_redundancy_switch_driver = {
  .name	       = DRIVER_NAME,
  .match_table = redundancy_switch_of_match,
  .probe       = redundancy_switch_of_probe,
  .remove      = __devexit_p(redundancy_switch_of_remove),
};

#endif /* CONFIG_OF */

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int redundancy_switch_platform_probe(struct platform_device *pdev) {
  struct resource *addressRange;
  struct resource *irq;

  /* Obtain the resources for this instance */
  addressRange = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!addressRange) {
    printk(KERN_ERR "%s: IO resource address not found.\n", pdev->name);
    return(-ENXIO);
  }

  /* Attempt to obtain the IRQ; if none is specified, the resource pointer is
   * NULL, and polling will be used.
   */
  irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

  /* Dispatch to the generic function */
  return(redundancy_switch_probe(pdev->name, pdev, addressRange, irq, NULL, NULL, NULL));
}

/* This is exported to allow polymorphic drivers to invoke it. */
int redundancy_switch_remove(struct redundancy_switch *redundancy_switch) {
  cdev_del(&redundancy_switch->cdev);
  disable_redundancy_switch(redundancy_switch);
  iounmap(redundancy_switch->virtualAddress);
  release_mem_region(redundancy_switch->physicalAddress, redundancy_switch->addressRangeSize);
  kfree(redundancy_switch);
  return(0);
}

static int redundancy_switch_platform_remove(struct platform_device *pdev) {
  struct redundancy_switch *redundancy_switch;

  /* Get a handle to the redundancy_switch and begin shutting it down */
  redundancy_switch = platform_get_drvdata(pdev);
  if(!redundancy_switch) return(-1);
  return(redundancy_switch_remove(redundancy_switch));
}

/* Platform device driver structure */
static struct platform_driver redundancy_switch_driver = {
  .probe  = redundancy_switch_platform_probe,
  .remove = redundancy_switch_platform_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __init redundancy_switch_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": AVB Audio Redundancy_Switch driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Lab X Technologies, LLC\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&of_redundancy_switch_driver);
#endif
 
  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&redundancy_switch_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }

  /* Allocate a range of major / minor device numbers for use */
  instanceCount = 0;
  if((returnValue = register_chrdev_region(MKDEV(DRIVER_MAJOR, 0), MAX_INSTANCES, DRIVER_NAME)) < 0) { 
    printk(KERN_INFO DRIVER_NAME "Failed to allocate character device range\n");
  }
  return(0);
}

static void __exit redundancy_switch_driver_exit(void)
{
  unregister_chrdev_region(MKDEV(DRIVER_MAJOR, 0), MAX_INSTANCES);

  /* Unregister as a platform device driver */
  platform_driver_unregister(&redundancy_switch_driver);
}

module_init(redundancy_switch_driver_init);
module_exit(redundancy_switch_driver_exit);

MODULE_AUTHOR("Eldridge M. Mount IV <eldridge.mount@labxtechnologies.com>");
MODULE_DESCRIPTION("Lab X Technologies AVB Redundancy Switch driver");
MODULE_LICENSE("GPL");
