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
#include <linux/biamp_spi_mailbox_defs.h>
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
  DBG("Mailbox disabled\n");
}

/* Enables the passed instance */
static void enable_mailbox(struct spi_mailbox *mailbox) {
  uint32_t controlRegister;

  DBG("Enabling the mailbox\n");
  controlRegister = XIo_In32(REGISTER_ADDRESS(mailbox, CONTROL_REG));
  controlRegister |= MAILBOX_ENABLE;
  printk("MBOX : Writing 0x%08X to 0x%08X\n", controlRegister, REGISTER_ADDRESS(mailbox, CONTROL_REG));
  XIo_Out32(REGISTER_ADDRESS(mailbox, CONTROL_REG), controlRegister);
}

/* Resets the state of the passed instance */
static void reset_mailbox(struct spi_mailbox *mailbox) {
  /* Disable the instance, and wipe its registers */
  disable_mailbox(mailbox);
}

/* Waits for a synchronized write to commit, using either polling or
 * an interrupt-driven waitqueue.
 */
static int32_t await_message_ready(struct spi_mailbox *mailbox) {
  int32_t returnValue = 0;

  /* Determine whether to use an interrupt or polling */
  int32_t waitResult;

  /* Place ourselves onto a wait queue if a message is flagged as available
   * by the hardware, as this indicates that the mailbox is active, and we 
   * need to wait for it to finish reading the message.  If the mailbox in 
   * not active or the message has alreadly been read, we will not actually 
   * enter the wait queue.
   */
  waitResult =
    wait_event_interruptible(mailbox->messageReadQueue, (mailbox->messageReadyFlag == MESSAGE_READY));

  /* Reset message ready flag after exiting the wait queue */ 
  mailbox->messageReadyFlag = MESSAGE_NOT_READY;

  /* If negative, a signal interrupted the wait. */
  if(waitResult < 0) returnValue = -EINTR;
			    
  /* Return success */
  return(returnValue);
}

/* Interrupt service routine for the instance */
static irqreturn_t biamp_spi_mailbox_interrupt(int irq, void *dev_id) {
  struct spi_mailbox *mailbox = (struct spi_mailbox*) dev_id;
  uint32_t maskedFlags;
  uint32_t irqMask;

  /* Read the interrupt flags and immediately clear them */
  maskedFlags = XIo_In32(REGISTER_ADDRESS(mailbox, IRQ_FLAGS_REG));
  irqMask = XIo_In32(REGISTER_ADDRESS(mailbox, IRQ_MASK_REG));

  printk("MBOX : IRQ, 0x%08X\n", maskedFlags);
  
  maskedFlags &= irqMask;
  XIo_Out32(REGISTER_ADDRESS(mailbox, IRQ_FLAGS_REG), maskedFlags);

  /* Detect the slave-to-host message wait_received IRQ */
  if((maskedFlags & IRQ_S2H_MSG_RX) != 0) {
    /* Set message ready flag before waking up thread */
    mailbox->messageReadyFlag = MESSAGE_READY;
    /* Wake up all threads waiting for a synchronization event */
    wake_up_interruptible(&(mailbox->messageReadQueue));
  }

  return(IRQ_HANDLED);
}

/*
 * Character device hook functions
 */

static int spi_mailbox_open(struct inode *inode, struct file *filp)
{
  struct spi_mailbox *mailbox;
  unsigned long flags;
  int returnValue = 0;

  /* Navigate to the driver object */
  mailbox = container_of(inode->i_cdev, struct spi_mailbox, cdev);
  filp->private_data = mailbox;

  /* Clear the message ready flag for the first time */
  mailbox->messageReadyFlag = MESSAGE_NOT_READY;

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

/* Retrieve message length and message contents
 */
static uint32_t read_mailbox_message(struct spi_mailbox *mailbox, uint8_t* mailboxMessage) {
  uint32_t messageLength;
  uint32_t controlReg;
  int i;

  DBG("Read mailbox message \n");
  messageLength  = XIo_In32(REGISTER_ADDRESS(mailbox, HOST_MSG_LEN_REG));
  for(i=0; i < ((messageLength + 3)/4); i++) { 
    ((uint32_t*)mailboxMessage)[i] = XIo_In32(MSG_RAM_BASE(mailbox)+(i*4));
  }
  /* Toggle bit in control register to acknowledge message has been picked up */
  printk("MBOX : Hitting consumed bit\n");
  controlReg = XIo_In32(REGISTER_ADDRESS(mailbox, CONTROL_REG));
  XIo_Out32(REGISTER_ADDRESS(mailbox, CONTROL_REG), (controlReg | HOST_MESSAGE_CONSUMED));
  return(messageLength);
}

/* Write response out to a received message
 */
static void send_message_response(struct spi_mailbox *mailbox, 
                                  MessageData *data) {
  int i;
  DBG("Writing response message \n");
  printk("Sending response\n");
  for(i=0; i < ((data->length + 3)/4); i++) {
    XIo_Out32(MSG_RAM_BASE(mailbox)+(i*4), ((uint32_t*)data->messageContent)[i]);
  }
  XIo_Out32(REGISTER_ADDRESS(mailbox, HOST_MSG_LEN_REG), data->length);
}

/* Set IRQ flags
 */
static void set_spi_irq_flags(struct spi_mailbox *mailbox, uint8_t flagValue) {
	
  DBG("Setting mailbox flags \n");
  XIo_Out32(REGISTER_ADDRESS(mailbox, SPI_IRQ_FLAGS_SET_REG), flagValue);
}

/* Read IRQ flags
 */
static uint8_t read_spi_irq_flags(struct spi_mailbox *mailbox) {
  uint8_t irqFlags;
	
  DBG("Clearing mailbox flags \n");
  irqFlags = XIo_In32(REGISTER_ADDRESS(mailbox, SPI_IRQ_FLAGS_SET_REG));
  return(irqFlags);
}

/* Clear IRQ flags
 */
static void clear_spi_irq_flags(struct spi_mailbox *mailbox, uint8_t flagValue) {
	
  DBG("Clearing mailbox flags \n");
  XIo_Out32(REGISTER_ADDRESS(mailbox, SPI_IRQ_FLAGS_CLEAR_REG), flagValue);
}

/* Set IRQ mask 
 */
static void set_spi_irq_mask(struct spi_mailbox *mailbox, uint8_t maskValue) {
	
  DBG("Setting mailbox mask \n");
  XIo_Out32(REGISTER_ADDRESS(mailbox, SPI_IRQ_MASK_REG), maskValue);
}

/* Read IRQ mask
 */
static uint8_t read_spi_irq_mask(struct spi_mailbox *mailbox) {
  uint8_t irqMask;
	
  DBG("Reading mailbox mask \n");
  irqMask = XIo_In32(REGISTER_ADDRESS(mailbox, SPI_IRQ_MASK_REG));
  return(irqMask);
}
/* Buffer for storing configuration words */
static uint8_t messageBuffer[MAX_MESSAGE_DATA];

/* I/O control operations for the driver */
static int spi_mailbox_ioctl(struct inode *inode, struct file *flip,
                             unsigned int command, unsigned long arg)
{
  // Switch on the request
  int returnValue = 0;
  struct spi_mailbox *mailbox = (struct spi_mailbox*)flip->private_data;

  switch(command) {
  case IOC_START_MBOX:
    enable_mailbox(mailbox);
    break;

  case IOC_STOP_MBOX:
    disable_mailbox(mailbox);
    break;

  case IOC_READ_MBOX:
    {  
      MessageData userMessage;
      MessageData localMessage;
    
      /* Copy into our local descriptor, then obtain buffer pointer from userland */
      if(copy_from_user(&userMessage, (void __user*)arg, sizeof(userMessage)) != 0) {
        return(-EFAULT);
      }

      returnValue = await_message_ready(mailbox);
     
      if(returnValue < 0) {
        return(returnValue);
      }

      localMessage.length = read_mailbox_message(mailbox, messageBuffer);
     
      if(copy_to_user((void __user*)userMessage.messageContent, messageBuffer,
                      (min(userMessage.length, localMessage.length))) != 0) {
        return(-EFAULT);
      }

      if(copy_to_user((void __user*)arg, &localMessage.length, sizeof(localMessage.length)) != 0 ) {
        return(-EFAULT);
      }
    }  
    break;
  
  case IOC_WRITE_RESP:
    {  
      MessageData userMessage;

      /* Copy into our local descriptor, then obtain buffer pointer from userland */
      if(copy_from_user(&userMessage, (void __user*)arg, sizeof(userMessage)) != 0) {
        return(-EFAULT);
      }
      if(userMessage.length > MAX_MESSAGE_DATA) {
        return(-EINVAL);
      }
      if(copy_from_user(messageBuffer, (void __user*)userMessage.messageContent, 
                        userMessage.length) != 0) {
        return(-EFAULT);
      }
      userMessage.messageContent = messageBuffer;
      send_message_response(mailbox, &userMessage);
    }  
    break;
  
  case IOC_SET_SPI_IRQ_FLAGS:
    {
      set_spi_irq_flags(mailbox, ((uint8_t)arg));
    }
    break;

  case IOC_CLEAR_SPI_IRQ_FLAGS:
    {
      clear_spi_irq_flags(mailbox, ((uint8_t)arg));
    }

  case IOC_READ_SPI_IRQ_FLAGS:
    {
      uint32_t returnValue = read_spi_irq_flags(mailbox);
     
      if(copy_to_user((void __user*)arg, &returnValue, sizeof(returnValue)) != 0 ) {
        return(-EFAULT);
      }
    }

  case IOC_SET_SPI_IRQ_MASK:
    {
      set_spi_irq_mask(mailbox, ((uint8_t)arg));
    }

  case IOC_READ_SPI_IRQ_MASK:
    {
      uint32_t returnValue = read_spi_irq_mask(mailbox);
    
      if(copy_to_user((void __user*)arg, &returnValue, sizeof(returnValue)) != 0 ) {
        return(-EFAULT);
      }
    }

  default:
    /* We don't recognize this command.
     */
    returnValue = -EINVAL;
  }

  /* Return an error code appropriate to the command */
  return(returnValue);
}
	  

/* Character device file operations structure */
static struct file_operations spi_mailbox_fops = {
  .open	   = spi_mailbox_open,
  .release = spi_mailbox_release,
  .ioctl   = spi_mailbox_ioctl,
  .owner   = THIS_MODULE,
};

/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 * @param name - Name of the instance
 * @param pdev - Platform device structure
 * @param addressRange - Resource describing the hardware's I/O range
 * @param irq          - Resource describing the hardware's IRQ
 */
static int spi_mailbox_probe(const char *name, 
                             struct platform_device *pdev,
                             struct resource *addressRange,
                             struct resource *irq) {
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

  /* Ensure that the mailbox and its interrupts are disabled */
  disable_mailbox(mailbox);
  XIo_Out32(REGISTER_ADDRESS(mailbox, IRQ_MASK_REG), NO_IRQS);

  /* Retain the IRQ and register our handler, if an IRQ resource was supplied. */
  if(irq != NULL) {
    mailbox->irq = irq->start;
    returnValue = request_irq(mailbox->irq, &biamp_spi_mailbox_interrupt, IRQF_DISABLED,
                              mailbox->name, mailbox);
    if (returnValue) {
      printk(KERN_ERR "%s : Could not allocate Biamp SPI Mailbox interrupt (%d).\n",
             mailbox->name, mailbox->irq);
      goto unmap;
    }
  } else mailbox->irq = NO_IRQ_SUPPLIED;
  
  /* Announce the device */
  printk(KERN_INFO "%s: Found Biamp mailbox at 0x%08X, ",
         mailbox->name, (uint32_t)mailbox->physicalAddress);
  if(mailbox->irq == NO_IRQ_SUPPLIED) {
    printk("polled interlocks\n");
  } else {
    printk("IRQ %d\n", mailbox->irq);
  }

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
  kobject_set_name(&mailbox->cdev.kobj, "%s%d", mailbox->name, mailbox->instanceNumber);
  mailbox->instanceNumber = instanceCount++;
  cdev_add(&mailbox->cdev, MKDEV(DRIVER_MAJOR, mailbox->instanceNumber), 1);

  /* Initialize the waitqueue used for synchronized writes */
  init_waitqueue_head(&(mailbox->messageReadQueue));

  /* Now that the device is configured, enable interrupts if they are to be used */
  if(mailbox->irq != NO_IRQ_SUPPLIED) {
    XIo_Out32(REGISTER_ADDRESS(mailbox, IRQ_FLAGS_REG), ALL_IRQS);
    XIo_Out32(REGISTER_ADDRESS(mailbox, IRQ_MASK_REG), IRQ_S2H_MSG_RX);
  }

  DBG("Mailbox initialized\n");

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
  return(spi_mailbox_probe(name, pdev, addressRange, irq));
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
  return(spi_mailbox_probe(pdev->name, pdev, addressRange, irq));
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
