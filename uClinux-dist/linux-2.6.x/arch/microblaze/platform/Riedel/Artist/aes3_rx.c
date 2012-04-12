/*
 *  linux/arch/microblaze/platform/Riedel/Artist/aes3_rx.c
 *
 *  Lab X Technologies AVB Riedel Artist AES receiver driver
 *
 *  Written by Yi Cao (yi.cao@labxtechnologies.com)
 *
 *  Copyright (C) 2012 Riedel Communications, All Rights Reserved.
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

#include "aes3_rx.h"
#include <linux/dma-mapping.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kdev_t.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/labx_aes3_rx_defs.h>
#include <net/genetlink.h>
#include <net/netlink.h>
#include <xio.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF

/* Driver name */
#define DRIVER_NAME "aes3_rx"

/* Major device number for the driver */
#define DRIVER_MAJOR 251

/* Maximum number of aes3_rx and instance count */
#define MAX_INSTANCES 64
static uint32_t instanceCount;

#define NO_IRQ_SUPPLIED   (-1)
#define NO_IRQS 0x0

/* Number of milliseconds to wait before permitting consecutive events from
 * being propagated up to userspace
 */
#define EVENT_THROTTLE_MSECS (100)

#if 0
#define DBG(f, x...) printk(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif



/* Interrupt service routine for the instance */
static irqreturn_t aes3_rx_interrupt(int irq, void *dev_id) {
  struct aes3_rx *rx = (struct aes3_rx*) dev_id;
  uint32_t maskedFlags;
  irqreturn_t returnValue = IRQ_NONE;

  /* write into the status reg will clear the interrupt */
  maskedFlags = XIo_In32(REGISTER_ADDRESS(rx, AES_STREAM_STATUS_REG));
  XIo_Out32(REGISTER_ADDRESS(rx, AES_STREAM_STATUS_REG), maskedFlags);

  /* To add the service routine of IRQ */
  
  /* Wake up the Netlink thread to consume status data */
  rx->statusReady = AES_NEW_STATUS_READY;
  wake_up_interruptible(&(rx->statusFifoQueue));
  returnValue = IRQ_HANDLED;

  return(IRQ_HANDLED);
}


/* Generic Netlink family definition */
static struct genl_family events_genl_family = {
  .id      = GENL_ID_GENERATE,
  .hdrsize = 0,
  .name    = LABX_AES_EVENTS_FAMILY_NAME,
  .version = LABX_AES_EVENTS_FAMILY_VERSION,
  .maxattr = LABX_AES_EVENTS_A_MAX,
};

/* Multicast groups */
static struct genl_multicast_group labx_aes_mcast = {
  .name = LABX_AES_EVENTS_STATUS_GROUP,
};

/* Method to conditionally transmit a Netlink packet containing one or more
 * packets of information from the status FIFO
 */
static int tx_netlink_status(struct aes3_rx *rx) {
  struct sk_buff *skb;
  void *msgHead;
  int returnValue = 0;
  uint32_t aesStatus;

  /* We will send a packet, allocate a socket buffer */
  skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
  if(skb == NULL) return(-ENOMEM);

  /* Create the message headers */
  msgHead = genlmsg_put(skb, 
                        0, 
                        rx->netlinkSequence++, 
                        &events_genl_family, 
                        0, 
                        LABX_AES_EVENTS_C_STATUS_PACKETS);
  if(msgHead == NULL) {
    returnValue = -ENOMEM;
    goto tx_failure;
  }

  /* Write the AES device's ID, properly translated for userspace, to identify the
   * message source.  The full ID is required since this driver can be used encapsulated 
   * within any number of more complex devices.
   */
  returnValue = nla_put_u32(skb, LABX_AES_EVENTS_A_AES_DEVICE, new_encode_dev(rx->deviceNode));
  if(returnValue != 0) goto tx_failure;

  /* Read the AES status register and send it out
   */
  aesStatus = XIo_In32(REGISTER_ADDRESS(rx, AES_STREAM_STATUS_REG));
  returnValue = nla_put_u32(skb, 
                            LABX_AES_EVENTS_A_AES_STATUS, 
                            aesStatus);
  if(returnValue != 0) goto tx_failure;


  /* Finalize the message and multicast it */
  genlmsg_end(skb, msgHead);
  returnValue = genlmsg_multicast(skb, 0, labx_aes_mcast.id, GFP_ATOMIC);

  switch(returnValue) {
  case 0:
  case -ESRCH:
    // Success or no process was listening, simply break
    break;

  default:
    // This is an actual error, print the return code
    printk(KERN_INFO DRIVER_NAME ": Failure delivering multicast Netlink message: %d\n",
           returnValue);
    goto tx_failure;
  }


 tx_failure:
  return(returnValue);
}

/* Kernel thread used to produce Netlink packets based upon the status FIFO */
static int netlink_thread(void *data) {
  struct aes3_rx *rx = (struct aes3_rx*) data;

  /* Use the "__" version to avoid using a memory barrier, no need since
   * we're going to the running state
   */
  __set_current_state(TASK_RUNNING);

  do {
    set_current_state(TASK_INTERRUPTIBLE);

    /* Go to sleep only if the status FIFO is empty */
    wait_event_interruptible(rx->statusFifoQueue, (rx->statusReady == AES_NEW_STATUS_READY) || kthread_should_stop());

    if (kthread_should_stop()) break;

    __set_current_state(TASK_RUNNING);

    /* Set the status flag as "idle"; the status ready IRQ is triggered only
     * when the FIFO goes from an empty to not-empty state, so there's no race
     * condition here.
     */
    rx->statusReady = AES_STATUS_IDLE;

    /* Transmit a Netlink
     * packet if there are any complete status packets to be sent.
     */
    tx_netlink_status(rx);

    /* Before returning to waiting, optionally sleep a little bit.  
     * There should be no need to disable
     * interrupts to prevent a race condition since the only part of the ISR
     * which modifies the IRQ mask is the servicing of the IRQs which are at
     * present disabled.
     *
     * Inserting a delay here, in conjunction with the disabling of the event
     * IRQ flags by the ISR, effectively limits the rate at which events can
     * be generated and sent up to user space.  Using an ISR / waitqueue is,
     * however, more responsive to occasional events than straight-up polling.
     */
    msleep(EVENT_THROTTLE_MSECS);
  } while(!kthread_should_stop());

  return(0);
}

/*
 * Character device hook functions
 */

static int aes3_rx_open(struct inode *inode, struct file *filp)
{
  struct aes3_rx *rx;
  unsigned long flags;
  int returnValue = 0;

  rx = container_of(inode->i_cdev, struct aes3_rx, cdev);
  filp->private_data = rx;

  /* Lock the mutex and ensure there is only one owner */
  preempt_disable();
  spin_lock_irqsave(&rx->mutex, flags);
  if(rx->opened) {
    returnValue = -1;
  } else {
    rx->opened = true;
  }

  /* Invoke the open() operation on the derived driver, if there is one */
  if((rx->derivedFops != NULL) && 
     (rx->derivedFops->open != NULL)) {
    rx->derivedFops->open(inode, filp);
  }

  spin_unlock_irqrestore(&rx->mutex, flags);
  preempt_enable();
  
  return(returnValue);
}

static int aes3_rx_release(struct inode *inode, struct file *filp)
{
  struct aes3_rx *rx = (struct aes3_rx*)filp->private_data;
  unsigned long flags;

  preempt_disable();
  spin_lock_irqsave(&rx->mutex, flags);
  rx->opened = false;

  /* Invoke the release() operation on the derived driver, if there is one */
  if((rx->derivedFops != NULL) && 
     (rx->derivedFops->release != NULL)) {
    rx->derivedFops->release(inode, filp);
  }

  spin_unlock_irqrestore(&rx->mutex, flags);
  preempt_enable();
  return(0);
}


/* I/O control operations for the driver */
static int aes3_rx_ioctl(struct inode *inode, 
                                   struct file *filp,
                                   unsigned int command, 
                                   unsigned long arg) {
 int returnValue = 0;
 uint32_t Value;
 struct aes3_rx *rx = (struct aes3_rx*)filp->private_data;

 switch(command) {
     
 case IOC_READ_STREAM_STATUS:
   {
     /* Get the stream status, then copy into the userspace pointer */
     Value = XIo_In32(REGISTER_ADDRESS(rx, AES_STREAM_STATUS_REG));
     if(copy_to_user((void __user*)arg, &Value, 
                     sizeof(uint32_t)) != 0) {
       return(-EFAULT);
     }
   }
   break;
   
 case IOC_CONFIG_AES: 
  {
   if(copy_from_user(&Value, (void __user*)arg, sizeof(Value)) != 0) {
        return(-EFAULT);
   }
   XIo_Out32(REGISTER_ADDRESS(rx, AES_CONTROL_REG), Value);
  }
   break;
   
 case IOC_READ_AES_MASK:
   {
     /* Get the stream mask, then copy into the userspace pointer */
     Value = XIo_In32(REGISTER_ADDRESS(rx, AES_STREAM_MASK_REG));
     if(copy_to_user((void __user*)arg, &Value, 
                     sizeof(uint32_t)) != 0) {
       return(-EFAULT);
     }
   }
   break;
   
 case IOC_SET_AES_MASK: 
  {
   if(copy_from_user(&Value, (void __user*)arg, sizeof(Value)) != 0) {
        return(-EFAULT);
   }
   XIo_Out32(REGISTER_ADDRESS(rx, AES_STREAM_MASK_REG), Value);
  }
   break;
 
 default:
     if((rx->derivedFops != NULL) && 
        (rx->derivedFops->ioctl != NULL)) {
       returnValue = rx->derivedFops->ioctl(inode, filp, command, arg);
     } else returnValue = -EINVAL;  
 
 }
 /* Return an error code appropriate to the command */
 return(returnValue);
}

/* Character device file operations structure */
static struct file_operations aes3_rx_fops = {
  .open	   = aes3_rx_open,
  .release = aes3_rx_release,
  .ioctl   = aes3_rx_ioctl,
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
int aes3_rx_probe(const char *name, 
                              struct platform_device *pdev,
                              struct resource *addressRange,
                              struct resource *irq,
                              struct file_operations *derivedFops,
                              void *derivedData,
                              struct aes3_rx **newInstance) {
  struct aes3_rx *rx;
  int returnValue;

  /* Create and populate a device structure */
  rx = (struct aes3_rx*) kmalloc(sizeof(struct aes3_rx), GFP_KERNEL);
  if(!rx) return(-ENOMEM);

  /* Request and map the device's I/O memory region into uncacheable space */
  rx->physicalAddress = addressRange->start;
  rx->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
  snprintf(rx->name, NAME_MAX_SIZE, "%s", name);
  rx->name[NAME_MAX_SIZE - 1] = '\0';
  if(request_mem_region(rx->physicalAddress, rx->addressRangeSize,
                        rx->name) == NULL) {
    returnValue = -ENOMEM;
    goto free;
  }

  rx->virtualAddress = 
    (void*) ioremap_nocache(rx->physicalAddress, rx->addressRangeSize);
  if(!rx->virtualAddress) {
    returnValue = -ENOMEM;
    goto release;
  }
  
  /* Initialize the waitqueue used for status FIFO events */
  init_waitqueue_head(&(rx->statusFifoQueue));


  /* Ensure that the engine and its interrupts are disabled */
  XIo_Out32(REGISTER_ADDRESS(rx, AES_STREAM_STATUS_REG), NO_IRQS);
  
  /*Initialize the Stream mask, enable all the 8 streams*/
  XIo_Out32(REGISTER_ADDRESS(rx, AES_STREAM_MASK_REG), 0xff);
  
  /* Run the thread assigned to converting status FIFO words into Netlink packets
   * after initializing its state to wait for the ISR
   */
  rx->statusReady = AES_STATUS_IDLE;
  rx->netlinkSequence = 0;
  rx->netlinkTask = kthread_run(netlink_thread, (void*) rx, "%s:netlink", rx->name);
  if (IS_ERR(rx->netlinkTask)) {
    printk(KERN_ERR "%s: AES Netlink task creation failed\n", rx->name);
    return(-EIO);
  }

  /* Retain the IRQ and register our handler, if an IRQ resource was supplied. */
  if(irq != NULL) {
    rx->irq = irq->start;
    returnValue = request_irq(rx->irq, &aes3_rx_interrupt, IRQF_DISABLED, 
                              rx->name, rx);
    if (returnValue) {
      printk(KERN_ERR "%s : Could not allocate Lab X Riedel Artist AES3 RX interrupt (%d).\n",
             rx->name, rx->irq);
      goto unmap;
    }
  } else rx->irq = NO_IRQ_SUPPLIED;

  /* Inspect and check the version to ensure it lies within the range of hardware
   * we support.
   */

  
  if(rx->irq == NO_IRQ_SUPPLIED) {
    printk("polled commands\n");
  } else {
    printk("IRQ %d\n", rx->irq);
  }

  /* Initialize other resources */
  spin_lock_init(&rx->mutex);
  rx->opened = false;

  /* Provide navigation between the device structures */
  platform_set_drvdata(pdev, rx);
  rx->pdev = pdev;

  /* Add as a character device to make the instance available for use */
  cdev_init(&rx->cdev, &aes3_rx_fops);
  rx->cdev.owner = THIS_MODULE;
  rx->instanceNumber = instanceCount++;
  kobject_set_name(&rx->cdev.kobj, "%s.%d", rx->name, rx->instanceNumber);
  cdev_add(&rx->cdev, MKDEV(DRIVER_MAJOR, rx->instanceNumber), 1);

  /*Initialize deviceNode, to be used in the netlink messages */
  rx->deviceNode = MKDEV(DRIVER_MAJOR, rx->instanceNumber);

  /* Retain any derived file operations & data to dispatch to */
  rx->derivedFops = derivedFops;
  rx->derivedData = derivedData;

  /* Return success, setting the return pointer if valid */
  if(newInstance != NULL) *newInstance = rx;
  return(0);

 unmap:
  iounmap(rx->virtualAddress);
 release:
  release_mem_region(rx->physicalAddress, rx->addressRangeSize);
 free:
  kfree(rx);
  return(returnValue);
      
}

#ifdef CONFIG_OF
static int aes3_rx_platform_remove(struct platform_device *pdev);

/* Probe for registered devices */
static int __devinit aes3_rx_of_probe(struct of_device *ofdev, const struct of_device_id *match)
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
  return(aes3_rx_probe(name, pdev, addressRange, irq, NULL, NULL, NULL));
}

static int __devexit aes3_rx_of_remove(struct of_device *dev)
{
  struct platform_device *pdev = to_platform_device(&dev->dev);
  aes3_rx_platform_remove(pdev);
  return(0);
}

/* Define the devices from the tree we are compatible with */
static struct of_device_id aes3_rx_of_match[] = {
  { .compatible = "xlnx,labx-aes3-rx-1.01.a", },
  { .compatible = "xlnx,labx-aes3-rx-1.02.a", },
  { /* end of list */ },
};


static struct of_platform_driver of_aes3_rx_driver = {
  .name        = DRIVER_NAME,
  .match_table = aes3_rx_of_match,
  .probe       = aes3_rx_of_probe,
  .remove      = __devexit_p(aes3_rx_of_remove),
};
#endif

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int aes3_rx_platform_probe(struct platform_device *pdev) {
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
  return(aes3_rx_probe(pdev->name, pdev, addressRange, irq, NULL, NULL, NULL));
}

/* Remove a previously-probed device */
static int aes3_rx_remove(struct aes3_rx *rx) {
  kthread_stop(rx->netlinkTask);
  cdev_del(&rx->cdev);
  iounmap(rx->virtualAddress);
  release_mem_region(rx->physicalAddress, rx->addressRangeSize);
  kfree(rx);
  return(0);
}

static int aes3_rx_platform_remove(struct platform_device *pdev) {
  struct aes3_rx *rx;

  /* Get a handle to the aes3_rx and begin shutting it down */
  rx = platform_get_drvdata(pdev);
  if(!rx) return(-1);
  return(aes3_rx_remove(rx));
}

/* Platform device driver structure */
static struct platform_driver aes3_rx_driver = {
  .probe  = aes3_rx_platform_probe,
  .remove = aes3_rx_platform_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __devinit aes3_rx_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": AES3 Receiver driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Riedel Communications\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&of_aes3_rx_driver);
#endif
 
  instanceCount = 0;
  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&aes3_rx_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    goto register_failure;
  }
  if((returnValue = register_chrdev_region(MKDEV(DRIVER_MAJOR, 0),MAX_INSTANCES, DRIVER_NAME)) < 0) { 
    printk(KERN_INFO DRIVER_NAME "Failed to allocate character device range\n");
    goto register_failure;
  }
  
  /* Register the Generic Netlink family for use */
  returnValue = genl_register_family(&events_genl_family);
  if(returnValue != 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register Generic Netlink family\n");
    goto register_failure;
  }

  /* Register multicast groups */
  returnValue = genl_register_mc_group(&events_genl_family, &labx_aes_mcast);
  if(returnValue != 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register Generic Netlink multicast group\n");
    genl_unregister_family(&events_genl_family);
    goto register_failure;
  }

register_failure:
  return(returnValue);
}

static void __devexit aes3_rx_driver_exit(void)
{
  unregister_chrdev_region(MKDEV(DRIVER_MAJOR, 0),MAX_INSTANCES);
  /* Unregister as a platform device driver */
  platform_driver_unregister(&aes3_rx_driver);
  
  genl_unregister_family(&events_genl_family);
}

module_init(aes3_rx_driver_init);
module_exit(aes3_rx_driver_exit);

MODULE_AUTHOR("Yi Cao <yi.cao@labxtechnologies.com>");
MODULE_DESCRIPTION("AES3 receiver driver");
MODULE_LICENSE("GPL");
