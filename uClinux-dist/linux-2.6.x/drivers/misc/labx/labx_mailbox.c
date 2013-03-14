/*
 *  linux/drivers/misc/labx/labx_mailbox.c
 *
 *  LABX mailbox peripheral driver
 *
 *  Written by Albert M. Hajjar (albert.hajjar@labxtechnologies.com)
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

#include "labx_mailbox.h"
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/labx_mailbox_defs.h>
#include <linux/kthread.h>
#include <xio.h>


#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF


/* Driver name and the revision range of hardware expected.
 * This driver will work with revision 1.1 only.
 */
#define DRIVER_NAME "labx_mailbox"

struct labx_mailbox* labx_mailboxes[MAX_MAILBOX_DEVICES] = {};
static uint32_t instanceCount;

#if 0
#define DBG(f, x...) printk(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

/* Disables the passed instance */
void disable_mailbox(struct labx_mailbox *mailbox) {
  uint32_t controlRegister;

  DBG("Disabling the mailbox\n");
  controlRegister = XIo_In32(REGISTER_ADDRESS(mailbox, SUPRV_CONTROL_REG));
  controlRegister &= ~(MAILBOX_ENABLE | MAILBOX_API_ENABLE);
  XIo_Out32(REGISTER_ADDRESS(mailbox, SUPRV_CONTROL_REG), controlRegister);
  DBG("Mailbox disabled\n");
}

/* Enables the passed instance */
void enable_mailbox(struct labx_mailbox *mailbox) {
  uint32_t controlRegister;

  DBG("Enabling the mailbox\n");
  controlRegister = XIo_In32(REGISTER_ADDRESS(mailbox, SUPRV_CONTROL_REG));
  controlRegister |= (MAILBOX_ENABLE | MAILBOX_API_ENABLE);
  XIo_Out32(REGISTER_ADDRESS(mailbox, SUPRV_CONTROL_REG), controlRegister);
  DBG("Mailbox enabled\n")
}

/* Resets the state of the passed instance */
static void reset_mailbox(struct labx_mailbox *mailbox) {
  /* Disable the instance, and wipe its registers */
  disable_mailbox(mailbox);
}

/* Interrupt service routine for the instance */
static irqreturn_t mailbox_interrupt(int irq, void *dev_id) {
  struct labx_mailbox *mailbox = (struct labx_mailbox*) dev_id;
  uint32_t maskedFlags;
  uint32_t irqMask;
  irqreturn_t returnValue = IRQ_NONE;

  /* Read the interrupt flags and immediately clear them */
  maskedFlags = XIo_In32(REGISTER_ADDRESS(mailbox, SUPRV_IRQ_FLAGS_REG));
  irqMask = XIo_In32(REGISTER_ADDRESS(mailbox, SUPRV_IRQ_MASK_REG));
  
  maskedFlags &= irqMask;
  XIo_Out32(REGISTER_ADDRESS(mailbox, SUPRV_IRQ_FLAGS_REG), maskedFlags);

  /* Detect the host-to-supervisor message wait_received IRQ */
  if((maskedFlags & SUPRV_IRQ_0) != 0) {
    /* Set message ready flag before waking up thread */
    mailbox->messageReadyFlag = MESSAGE_READY;
    
    /* Wake up all threads waiting for a synchronization event */
    wake_up_interruptible(&(mailbox->messageReadQueue));
    returnValue = IRQ_HANDLED;
  }

  /* Return whether this was an IRQ we handled or not */
  return(returnValue);
}

static int netlink_thread(void *data)
{
  struct labx_mailbox *mailbox = (struct labx_mailbox*)data;

  __set_current_state(TASK_RUNNING);

  do {
    set_current_state(TASK_INTERRUPTIBLE);
      
    wait_event_interruptible(mailbox->messageReadQueue,
                             ((mailbox->messageReadyFlag == MESSAGE_READY) || (kthread_should_stop())));

    if (kthread_should_stop()) break;

    __set_current_state(TASK_RUNNING);

    /* Reset message ready flag */ 
    mailbox->messageReadyFlag = MESSAGE_NOT_READY;
    
    mailbox_event_send_request(mailbox);
  } while (!kthread_should_stop());

  return 0;
}

/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 * @param name - Name of the instance
 * @param pdev - Platform device structure
 * @param addressRange - Resource describing the hardware's I/O range
 * @param irq          - Resource describing the hardware's IRQ
 */
static int mailbox_probe(const char *name, 
                             struct platform_device *pdev,
                             struct resource *addressRange,
                             struct resource *irq) {
  struct labx_mailbox *mailbox;
  int returnValue;
  int i;

  /* Create and populate a device structure */
  mailbox = (struct labx_mailbox*) kmalloc(sizeof(struct labx_mailbox), GFP_KERNEL);
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
  XIo_Out32(REGISTER_ADDRESS(mailbox, SUPRV_IRQ_MASK_REG), NO_IRQS);

  /* Clear the message ready flag for the first time */
  mailbox->messageReadyFlag = MESSAGE_NOT_READY;
  
  /* Retain the IRQ and register our handler, if an IRQ resource was supplied. */
  if(irq != NULL) {
    mailbox->irq = irq->start;
    returnValue = request_irq(mailbox->irq, &mailbox_interrupt, IRQF_DISABLED,
                              mailbox->name, mailbox);
    if (returnValue) {
      printk(KERN_ERR "%s : Could not allocate Mailbox interrupt (%d).\n",
             mailbox->name, mailbox->irq);
      goto unmap;
    }
  } else mailbox->irq = NO_IRQ_SUPPLIED;
  
  /* Announce the device */
  printk(KERN_INFO "%s: Found mailbox at 0x%08X, ",
         mailbox->name, (uint32_t)mailbox->physicalAddress);
  if(mailbox->irq == NO_IRQ_SUPPLIED) {
    printk("polled interlocks\n");
  } else {
    printk("IRQ %d\n", mailbox->irq);
  }

  /* Initialize other resources */
  spin_lock_init(&mailbox->mutex);
  mailbox->opened = true;

  /* Provide navigation between the device structures */
  platform_set_drvdata(pdev, mailbox);
  mailbox->pdev = pdev;

  /* Reset the state of the mailbox */
  reset_mailbox(mailbox);

  /* Initialize the waitqueue used for synchronized writes */
  init_waitqueue_head(&(mailbox->messageReadQueue));

  /* Initialize the netlink state and start the thread */
  mailbox->netlinkSequence = 0;
  mailbox->netlinkTask = kthread_run(netlink_thread, (void*)mailbox, "%s:netlink", mailbox->name);
  if (IS_ERR(mailbox->netlinkTask)) {
    printk(KERN_ERR "Mailbox netlink task creation failed.\n");
    returnValue = -EIO;
    goto free;
  }
  
  /* Now that the device is configured, enable interrupts if they are to be used */
  if(mailbox->irq != NO_IRQ_SUPPLIED) {
    XIo_Out32(REGISTER_ADDRESS(mailbox, SUPRV_IRQ_MASK_REG), ALL_IRQS);
    XIo_Out32(REGISTER_ADDRESS(mailbox, SUPRV_IRQ_FLAGS_REG), ALL_IRQS);
  }

  // Add the mailbox instance to the list of current devices
  for(i=0;i<MAX_MAILBOX_DEVICES;i++) {
    if(NULL == labx_mailboxes[i]) {
      labx_mailboxes[i] = mailbox;
      printk("Adding mailbox: %s\n", labx_mailboxes[i]->name);
      break;
    }
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
static int mailbox_platform_remove(struct platform_device *pdev);

/* Probe for registered devices */
static int __devinit mailbox_of_probe(struct of_device *ofdev, const struct of_device_id *match)
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
  return(mailbox_probe(name, pdev, addressRange, irq));
}

static int __devexit mailbox_of_remove(struct of_device *dev)
{
  struct platform_device *pdev = to_platform_device(&dev->dev);
  mailbox_platform_remove(pdev);
  return(0);
}

static struct of_device_id mailbox_of_match[] = {
  { .compatible = "xlnx,labx-mailbox-1.00.a", },
  { /* end of list */ },
};


static struct of_platform_driver of_mailbox_driver = {
  .name		= DRIVER_NAME,
  .match_table  = mailbox_of_match,
  .probe   	= mailbox_of_probe,
  .remove       = __devexit_p(mailbox_of_remove),
};
#endif

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int mailbox_platform_probe(struct platform_device *pdev) {
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
  return(mailbox_probe(pdev->name, pdev, addressRange, irq));
}

static int mailbox_platform_remove(struct platform_device *pdev) {
  struct labx_mailbox *mailbox;

  /* Get a handle to the mailbox and begin shutting it down */
  mailbox = platform_get_drvdata(pdev);
  if(!mailbox) return(-1);
  
  /* Release the IRQ */
  if (mailbox->irq != NO_IRQ_SUPPLIED) {
    free_irq(mailbox->irq, mailbox);
  }
  
  kthread_stop(mailbox->netlinkTask);
  reset_mailbox(mailbox);
  iounmap(mailbox->virtualAddress);
  release_mem_region(mailbox->physicalAddress, mailbox->addressRangeSize);
  kfree(mailbox);
  return(0);
}

/* Platform device driver structure */
static struct platform_driver mailbox_driver = {
  .probe  = mailbox_platform_probe,
  .remove = mailbox_platform_remove,
  .driver = {
  .name   = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __init mailbox_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": Mailbox Driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright (c) Lab X Technologies, LLC\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&of_mailbox_driver);
#endif
 
  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&mailbox_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }

  /* Reset instance count for platform device */
  instanceCount = 0;
  
  /* Initialize the Netlink layer for the driver */
  register_mailbox_netlink();
  
  return(0);
}

static void __exit mailbox_driver_exit(void)
{
  /* Unregister Generic Netlink family */
  unregister_mailbox_netlink();

  /* Unregister as a platform device driver */
  platform_driver_unregister(&mailbox_driver);
}

module_init(mailbox_driver_init);
module_exit(mailbox_driver_exit);

MODULE_AUTHOR("Albert M. Hajjar <albert.hajjar@labxtechnologies.com>");
MODULE_DESCRIPTION("Lab X Technologies Mailbox driver");
MODULE_LICENSE("GPL");
