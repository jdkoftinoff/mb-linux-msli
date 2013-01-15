/*
 *  linux/arch/microblaze/platform/MeyerSound/CAL_ICS/cal_sample_pointers.c
 *
 *  Lab X Technologies AVB CAL_ICS data streams routing driver
 *
 *  Written by Yi Cao (yi.cao@labxtechnologies.com)
 *
 *  Copyright (C) 2011 Meyer Sound Laboratories Inc, All Rights Reserved.
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

#include "cal_sample_pointers.h"
#include <linux/dma-mapping.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <xio.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF

/* Driver name */
#define DRIVER_NAME "cal_sample_pointers"

/* Major device number for the driver */
#define DRIVER_MAJOR 251

/* Maximum number of cal_sample_pointers and instance count */
#define MAX_INSTANCES 64
static uint32_t instanceCount;
#define NAME_MAX_SIZE    (256)
#define NO_IRQ_SUPPLIED   (-1)
#define NO_IRQS 0x0
#define ALL_IRQS 0x1

#if 0
#define DBG(f, x...) printk(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

/* reset the passed instance */
static void reset_sample_pointers(struct sample_pointers *sample_pointers) {

  int reg;
  DBG("reseting the routing matrix\n");
  /* set the routing path of all the output to AVB1 */
  for (reg = 0; reg < 0x14; reg++) 
    XIo_Out32(REGISTER_ADDRESS(sample_pointers, reg), 0);
  /* set all the offset to be 1 */
  for (reg = 0x14; reg < 0x26; reg++) 
    XIo_Out32(REGISTER_ADDRESS(sample_pointers, reg), 1);
  /* set the soft mute reg to 0, i.e., no mute */
  XIo_Out32(REGISTER_ADDRESS(sample_pointers, MUTE_CTRL_REG), 0);
  /* set the routing from the 3 ADC channels to the 3 Scarf channels */  
  XIo_Out32(REGISTER_ADDRESS(sample_pointers, SCARF1_CHAN1_SEL_REG), ADC2_1_CHAN_ADDR);
  XIo_Out32(REGISTER_ADDRESS(sample_pointers, SCARF1_CHAN2_SEL_REG), ADC2_1_CHAN_ADDR);
  XIo_Out32(REGISTER_ADDRESS(sample_pointers, SCARF2_CHAN1_SEL_REG), ADC2_1_CHAN_ADDR);
  XIo_Out32(REGISTER_ADDRESS(sample_pointers, SCARF2_CHAN2_SEL_REG), ADC2_1_CHAN_ADDR);
  XIo_Out32(REGISTER_ADDRESS(sample_pointers, SCARF3_CHAN1_SEL_REG), ADC2_1_CHAN_ADDR);
  XIo_Out32(REGISTER_ADDRESS(sample_pointers, SCARF3_CHAN2_SEL_REG), ADC2_1_CHAN_ADDR);
}

/* load soft mute coefficients */
static void load_soft_mute_coefficient(struct sample_pointers *sample_pointers) {

  int reg;
  DBG("loading the soft mute coefficient table\n");

  for (reg = 0; reg < 256; reg++) 
    XIo_Out32(REGISTER_ADDRESS(sample_pointers, MUTE_COEFF_REG_BASE_ADDRESS+reg), MUTE_COEFF_TABLE[0][reg]);
}



/* Interrupt service routine for the instance */
static irqreturn_t cal_sample_pointers_interrupt(int irq, void *dev_id) {
  struct sample_pointers *sample_pointers = (struct sample_pointers*) dev_id;
  uint32_t maskedFlags;
  uint32_t irqMask;

  /* Read the interrupt flags and immediately clear them */
  maskedFlags = XIo_In32(REGISTER_ADDRESS(sample_pointers, HOST_IRQ_FLAGS_REG));
  irqMask = XIo_In32(REGISTER_ADDRESS(sample_pointers, HOST_IRQ_MASK_REG));
  maskedFlags &= irqMask;
  XIo_Out32(REGISTER_ADDRESS(sample_pointers, HOST_IRQ_FLAGS_REG), maskedFlags);

  /* To add the service routine of IRQ */
  

  return(IRQ_HANDLED);
}

/*
 * Character device hook functions
 */

static int sample_pointers_open(struct inode *inode, struct file *filp)
{
  struct sample_pointers *sample_pointers;
  unsigned long flags;
  int returnValue = 0;

  sample_pointers = container_of(inode->i_cdev, struct sample_pointers, cdev);
  filp->private_data = sample_pointers;

  /* Lock the mutex and ensure there is only one owner */
  preempt_disable();
  spin_lock_irqsave(&sample_pointers->mutex, flags);
  sample_pointers->opened++;

  /* Invoke the open() operation on the derived driver, if there is one */
  if((sample_pointers->derivedFops != NULL) && 
     (sample_pointers->derivedFops->open != NULL)) {
    sample_pointers->derivedFops->open(inode, filp);
  }

  spin_unlock_irqrestore(&sample_pointers->mutex, flags);
  preempt_enable();
  
  return(returnValue);
}

static int sample_pointers_release(struct inode *inode, struct file *filp)
{
  struct sample_pointers *sample_pointers = (struct sample_pointers*)filp->private_data;
  unsigned long flags;

  preempt_disable();
  spin_lock_irqsave(&sample_pointers->mutex, flags);
  sample_pointers->opened--;

  /* Invoke the release() operation on the derived driver, if there is one */
  if((sample_pointers->derivedFops != NULL) && 
     (sample_pointers->derivedFops->release != NULL)) {
    sample_pointers->derivedFops->release(inode, filp);
  }

  spin_unlock_irqrestore(&sample_pointers->mutex, flags);
  preempt_enable();
  return(0);
}

/* I/O control operations for the driver */
static int sample_pointers_ioctl(struct inode *inode, 
                                   struct file *filp,
                                   unsigned int command, 
                                   unsigned long arg) {
  int returnValue = 0;
  uint32_t Value = 0xFF;
  uint32_t Reg = 0xFF;
  struct sample_pointers *sample_pointers = (struct sample_pointers*)filp->private_data;

  /* The least significant byte of arg is the Value (sample memory address, or sample offset value)
     The second to the least significant byte of arg is the register address of output channel select
  */
  switch(command) {
    
  case IOC_RESET_ROUTING_MATRIX:
    reset_sample_pointers(sample_pointers);
    break;
      
  case IOC_READ_ROUTING_MATRIX:
    {
      uint32_t userVal = 0;
      if(copy_from_user(&userVal, (void __user*)arg, 
                        sizeof(uint32_t)) != 0) {
        return(-EFAULT);
      }
      Reg = userVal;

      /* Get the stream status, then copy into the userspace pointer */
      Value = XIo_In32(REGISTER_ADDRESS(sample_pointers, Reg));
      if(copy_to_user((void __user*)arg, &Value, 
                      sizeof(uint32_t)) != 0) {
        return(-EFAULT);
      }
    }
    break;
    
  case IOC_SET_ROUTING_MATRIX: 
    {
      uint32_t userVal = 0;
      if(copy_from_user(&userVal, (void __user*)arg, 
                        sizeof(uint32_t)) != 0) {
        return(-EFAULT);
      }
      Value = userVal & 0xFF;
      Reg = (userVal >> 8) & 0xFF;
      XIo_Out32(REGISTER_ADDRESS(sample_pointers, Reg), Value);
    }
    break;
  
  case IOC_UPDATE_SOFT_MUTE_COEFF_TABLE:
    {
      uint32_t userVal = 0;
      if(copy_from_user(&userVal, (void __user*)arg, 
                        sizeof(uint32_t)) != 0) {
        return(-EFAULT);
      }
      Value = userVal;
      if ( (Value < 0) || (Value >= COEFF_TABLE_MAX_NUM) ) {
        Value = 0;
      }
      for (Reg = 0; Reg < 256; Reg++) {
        XIo_Out32(REGISTER_ADDRESS(sample_pointers, MUTE_COEFF_REG_BASE_ADDRESS+Reg), MUTE_COEFF_TABLE[Value][Reg]);
      }
    }
    break;
   
  case IOC_CAL_SET_AVB_MUTE:
    {
      uint32_t userVal = 0;
      uint32_t channel = 0;
      if(copy_from_user(&userVal, (void __user*)arg, 
                        sizeof(uint32_t)) != 0) {
        return(-EFAULT);
      }
      channel = userVal & 0x3;
      Value = XIo_In32(REGISTER_ADDRESS(sample_pointers, AVB_STREAM_CHAN_REG));
      Value &= ~(0xF << (channel*4));
      Value |= ((userVal >> 8) & 0xF) << (channel*4);
      XIo_Out32(REGISTER_ADDRESS(sample_pointers, AVB_STREAM_CHAN_REG), Value);
    }
    break;
  
  default:
      if((sample_pointers->derivedFops != NULL) && 
         (sample_pointers->derivedFops->ioctl != NULL)) {
        returnValue = sample_pointers->derivedFops->ioctl(inode, filp, command, arg);
      } else returnValue = -EINVAL;  
  
  }
  /* Return an error code appropriate to the command */
  return(returnValue);
}

/* Character device file operations structure */
static struct file_operations sample_pointers_fops = {
  .open	   = sample_pointers_open,
  .release = sample_pointers_release,
  .ioctl   = sample_pointers_ioctl,
  .owner   = THIS_MODULE,
};


/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 * This is exported to allow polymorphic drivers to invoke it.
 * @param name - Name of the instance
 * @param pdev - Platform device structure
 * @param addressRange - Resource describing the hardware's I/O 
 * @param irq          - Resource describing the hardware's interrupt
 */
int cal_sample_pointers_probe(const char *name, 
                              struct platform_device *pdev,
                              struct resource *addressRange,
                              struct resource *irq,
                              struct file_operations *derivedFops,
                              void *derivedData,
                              struct sample_pointers **newInstance) {

  
  struct sample_pointers *sample_pointers;
  int returnValue;

  /* Create and populate a device structure */
  sample_pointers = (struct sample_pointers*) kmalloc(sizeof(struct sample_pointers), GFP_KERNEL);
  if(!sample_pointers) return(-ENOMEM);

  /* Request and map the device's I/O memory region into uncacheable space */
  sample_pointers->physicalAddress = addressRange->start;
  sample_pointers->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
  snprintf(sample_pointers->name, NAME_MAX_SIZE, "%s", name);
  sample_pointers->name[NAME_MAX_SIZE - 1] = '\0';
  if(request_mem_region(sample_pointers->physicalAddress, sample_pointers->addressRangeSize,
                        sample_pointers->name) == NULL) {
    returnValue = -ENOMEM;
    goto free;
  }

  sample_pointers->virtualAddress = 
    (void*) ioremap_nocache(sample_pointers->physicalAddress, sample_pointers->addressRangeSize);
  if(!sample_pointers->virtualAddress) {
    returnValue = -ENOMEM;
    goto release;
  }

  /* Ensure that the engine and its interrupts are disabled */
  XIo_Out32(REGISTER_ADDRESS(sample_pointers, HOST_IRQ_MASK_REG), NO_IRQS);

  /* Retain the IRQ and register our handler, if an IRQ resource was supplied. */
  if(irq != NULL) {
    sample_pointers->irq = irq->start;
    returnValue = request_irq(sample_pointers->irq, &cal_sample_pointers_interrupt, IRQF_DISABLED, 
                              sample_pointers->name, sample_pointers);
    if (returnValue) {
      printk(KERN_ERR "%s : Could not allocate Lab X Audio sample_pointers interrupt (%d).\n",
             sample_pointers->name, sample_pointers->irq);
      goto unmap;
    }
  } else sample_pointers->irq = NO_IRQ_SUPPLIED;

  /* Inspect and check the version to ensure it lies within the range of hardware
   * we support.
   */

  
  if(sample_pointers->irq == NO_IRQ_SUPPLIED) {
    printk("polled commands\n");
  } else {
    printk("IRQ %d\n", sample_pointers->irq);
  }

  /* Initialize other resources */
  spin_lock_init(&sample_pointers->mutex);
  sample_pointers->opened = 0;

  /* Provide navigation between the device structures */
  platform_set_drvdata(pdev, sample_pointers);
  sample_pointers->pdev = pdev;

  /* Reset the state of the sample_pointers */
  reset_sample_pointers(sample_pointers);
  /* load the soft mute coefficient table */
  load_soft_mute_coefficient(sample_pointers);

  /* Add as a character device to make the instance available for use */
  cdev_init(&sample_pointers->cdev, &sample_pointers_fops);
  sample_pointers->cdev.owner = THIS_MODULE;
  sample_pointers->instanceNumber = instanceCount++;
  kobject_set_name(&sample_pointers->cdev.kobj, "%s.%d", sample_pointers->name, sample_pointers->instanceNumber);
  cdev_add(&sample_pointers->cdev, MKDEV(DRIVER_MAJOR, sample_pointers->instanceNumber), 1);


  /* Now that the device is configured, enable interrupts if they are to be used */
  if(sample_pointers->irq != NO_IRQ_SUPPLIED) {
    XIo_Out32(REGISTER_ADDRESS(sample_pointers, HOST_IRQ_FLAGS_REG), ALL_IRQS);
  }

  /* Retain any derived file operations & data to dispatch to */
  sample_pointers->derivedFops = derivedFops;
  sample_pointers->derivedData = derivedData;

  /* Return success, setting the return pointer if valid */
  if(newInstance != NULL) *newInstance = sample_pointers;
  return(0);

 unmap:
  iounmap(sample_pointers->virtualAddress);
 release:
  release_mem_region(sample_pointers->physicalAddress, sample_pointers->addressRangeSize);
 free:
  kfree(sample_pointers);
  return(returnValue);
      
}

#ifdef CONFIG_OF
static int cal_sample_pointers_platform_remove(struct platform_device *pdev);

/* Probe for registered devices */
static int __devinit cal_sample_pointers_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
  struct resource r_mem_struct = {};
  struct resource r_irq_struct = {};
  struct resource *addressRange = &r_mem_struct;
  struct resource *irq          = &r_irq_struct;
  struct platform_device *pdev  = to_platform_device(&ofdev->dev);
  const char *name = ofdev->node->name;
  int rc = 0;

  /* Obtain the resources for this instance */
  rc = of_address_to_resource(ofdev->node, 0, addressRange);
  if(rc) {
    printk(KERN_ERR "%s : No address found in device tree\n", name);
    return(rc);
  }

  rc = of_irq_to_resource(ofdev->node, 0, irq);
  if(rc == NO_IRQ) {
    printk(KERN_ERR "%s : No IRQ found in device tree\n", name);
    return(rc);
  }

  /* Dispatch to the generic function */
  return(cal_sample_pointers_probe(name, pdev, addressRange, irq, NULL, NULL, NULL));
}

static int __devexit cal_sample_pointers_of_remove(struct of_device *dev)
{
  struct platform_device *pdev = to_platform_device(&dev->dev);
  cal_sample_pointers_platform_remove(pdev);
  return(0);
}

/* Define the devices from the tree we are compatible with */
static struct of_device_id cal_sample_pointers_of_match[] = {
  { .compatible = "xlnx,cal-sample-pointers-1.00.a", },
  { /* end of list */ },
};


static struct of_platform_driver of_cal_sample_pointers_driver = {
  .name        = DRIVER_NAME,
  .match_table = cal_sample_pointers_of_match,
  .probe       = cal_sample_pointers_of_probe,
  .remove      = __devexit_p(cal_sample_pointers_of_remove),
};
#endif

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int cal_sample_pointers_platform_probe(struct platform_device *pdev) {
  struct resource *addressRange;
  struct resource *irq;

  /* Obtain the resources for this instance */
  addressRange = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!addressRange) {
    printk(KERN_ERR "%s: IO resource address not found.\n", pdev->name);
    return(-ENXIO);
  }
  irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

  /* Dispatch to the generic function */
  return(cal_sample_pointers_probe(pdev->name, pdev, addressRange, irq, NULL, NULL, NULL));
}

/* Remove a previously-probed device */
static int cal_sample_pointers_remove(struct sample_pointers *sample_pointers) {
  reset_sample_pointers(sample_pointers);
  return(0);
}

static int cal_sample_pointers_platform_remove(struct platform_device *pdev) {
  struct sample_pointers *pointers;

  /* Get a handle to the legacy_bridge and begin shutting it down */
  pointers = platform_get_drvdata(pdev);
  if(!pointers) return(-1);
  return(cal_sample_pointers_remove(pointers));
}

/* Platform device driver structure */
static struct platform_driver cal_sample_pointers_driver = {
  .probe  = cal_sample_pointers_platform_probe,
  .remove = cal_sample_pointers_platform_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __devinit cal_sample_pointers_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": CAL sample pointers driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Meyer Sound Laboratories Inc\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&of_cal_sample_pointers_driver);
#endif
 
  instanceCount = 0;
  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&cal_sample_pointers_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }
  if((returnValue = register_chrdev_region(MKDEV(DRIVER_MAJOR, 0),MAX_INSTANCES, DRIVER_NAME)) < 0) { 
    printk(KERN_INFO DRIVER_NAME "Failed to allocate character device range\n");
  }

  return(0);
}

static void __devexit cal_sample_pointers_driver_exit(void)
{
  unregister_chrdev_region(MKDEV(DRIVER_MAJOR, 0),MAX_INSTANCES);
  /* Unregister as a platform device driver */
  platform_driver_unregister(&cal_sample_pointers_driver);
}

module_init(cal_sample_pointers_driver_init);
module_exit(cal_sample_pointers_driver_exit);

MODULE_AUTHOR("Yi Cao <yi.cao@labxtechnologies.com>");
MODULE_DESCRIPTION("CAL sample pointers driver");
MODULE_LICENSE("GPL");
