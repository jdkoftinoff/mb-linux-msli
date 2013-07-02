/*
 *  linux/drivers/net/labx_avb/labx_mtd_bridge.c
 *
 *  Lab X Technologies MTD bridge Flash driver
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *
 *  Copyright (C) 2009 Lab X Technologies LLC, All Rights Reserved.
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

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <xio.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF


#if 0
#define DBG(f, x...) printk(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

#define DRIVER_NAME "labx_mtd_bridge"

/* Driver structure to maintain state for each device instance */
#define NAME_MAX_SIZE    (256)
#define NO_IRQ_SUPPLIED   (-1)
struct mtd_bridge {
  /* Pointer back to the platform device */
  struct platform_device *pdev;

  /* Name for use in identification */
  char name[NAME_MAX_SIZE];

  /* Physical and virtual base address */
  uintptr_t      physicalAddress;
  uintptr_t      addressRangeSize;
  void __iomem  *virtualAddress;

  /* Interrupt request number */
  int32_t irq;

  /* Wait queue for putting threads to sleep */
  wait_queue_head_t queue;

  /* Mutex for the device instance */
  spinlock_t mutex;
};

/* Interrupt service routine for the instance */
static irqreturn_t labx_mtd_bridge_interrupt(int irq, void *dev_id) {
  struct mtd_bridge *bridge = (struct mtd_bridge*) dev_id;
  uint32_t maskedFlags;
  uint32_t irqMask;

#if 0
  /* Read the interrupt flags and immediately clear them */
  maskedFlags = XIo_In32(REGISTER_ADDRESS(bridge, IRQ_FLAGS_REG));
  irqMask = XIo_In32(REGISTER_ADDRESS(bridge, IRQ_MASK_REG));
  maskedFlags &= irqMask;
  XIo_Out32(REGISTER_ADDRESS(bridge, IRQ_FLAGS_REG), maskedFlags);

  /* Detect the synchronized write IRQ */
  if((maskedFlags & SYNC_IRQ) != 0) {
    /* Wake up all threads waiting for a synchronization event */
    wake_up_interruptible(&(bridge->queue));
  }

  /* Detect the engine late IRQ */
  if((maskedFlags & ENGINE_LATE_IRQ) != 0) {  
    /* TODO - We need to create an AVB event mechanism using kernel events, so
     *        that they can be propagated up to the user API!!!
     */
    printk("%s: Engine not idle at beginning of shaping interval!\n", 
           bridge->name);

    /* Disable the interrupt source to prevent ongoing interrupts; this indicates
     * a systemic problem.  Either the bandwidth credit has been misconfigured, or
     * something at the system level is taking strict priority over this credit-based
     * shaping stage, which is not permitted.
     */
    irqMask &= ~ENGINE_LATE_IRQ;
    XIo_Out32(REGISTER_ADDRESS(bridge, IRQ_MASK_REG), irqMask);
  }

  /* Detect the shaper overrun IRQ; the comments for the "engine late" IRQ
   * are equally applicable to this as well.
   */
  if((maskedFlags & OVERRUN_IRQ) != 0) {  
    printk("%s: Packet overrun at output stage!\n", bridge->name);
    irqMask &= ~OVERRUN_IRQ;
    XIo_Out32(REGISTER_ADDRESS(bridge, IRQ_MASK_REG), irqMask);
  }

#endif

  return(IRQ_HANDLED);
}

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
int mtd_bridge_probe(const char *name, 
                     struct platform_device *pdev,
                     struct resource *addressRange,
                     struct resource *irq) {
  struct mtd_bridge *bridge;
  int returnValue;

  /* Create and populate a device structure */
  bridge = (struct mtd_bridge*) kmalloc(sizeof(struct mtd_bridge), GFP_KERNEL);
  if(!bridge) return(-ENOMEM);

  /* Request and map the device's I/O memory region into uncacheable space */
  bridge->physicalAddress = addressRange->start;
  bridge->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
  snprintf(bridge->name, NAME_MAX_SIZE, "%s", name);
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

  /* Ensure that the interrupts are disabled */
  printk("<<DISABLE IRQ>>\n");
  // XIo_Out32(REGISTER_ADDRESS(bridge, IRQ_MASK_REG), NO_IRQS);

  /* Retain the IRQ and register our handler, if an IRQ resource was supplied. */
  printk("<<NO IRQ YET>>\n");
#if 0
  if(irq != NULL) {
    bridge->irq = irq->start;
    returnValue = request_irq(bridge->irq, &labx_mtd_bridge_interrupt, IRQF_DISABLED, 
                              bridge->name, bridge);
    if (returnValue) {
      printk(KERN_ERR "%s : Could not allocate Lab X Audio Bridge interrupt (%d).\n",
             bridge->name, bridge->irq);
      goto unmap;
    }
  } else bridge->irq = NO_IRQ_SUPPLIED;
#endif

  /* Announce the device */
  printk(KERN_INFO "%s: Found Lab X MTD Bridge at 0x%08X, IRQ %d\n", 
         bridge->name, (uint32_t) bridge->physicalAddress, bridge->irq);

  /* Initialize other resources */
  spin_lock_init(&bridge->mutex);

  /* Provide navigation between the device structures */
  platform_set_drvdata(pdev, bridge);
  bridge->pdev = pdev;

  /* Initialize the waitqueue used for synchronized writes */
  init_waitqueue_head(&(bridge->queue));

  /* Now that the device is configured, enable interrupts if they are to be used */
  if(bridge->irq != NO_IRQ_SUPPLIED) {
    //    XIo_Out32(REGISTER_ADDRESS(bridge, IRQ_FLAGS_REG), ALL_IRQS);
    //    XIo_Out32(REGISTER_ADDRESS(bridge, IRQ_MASK_REG), SYNC_IRQ);
  }

  return 0;

 unmap:
  iounmap(bridge->virtualAddress);
 release:
  release_mem_region(bridge->physicalAddress, bridge->addressRangeSize);
 free:
  kfree(bridge);
  return(returnValue);
}

#ifdef CONFIG_OF
static int mtd_bridge_platform_remove(struct platform_device *pdev);

/* Probe for registered devices */
static int __devinit mtd_bridge_of_probe(struct of_device *ofdev, const struct of_device_id *match)
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
  return(mtd_bridge_probe(name, pdev, addressRange, irq));
}

static int __devexit mtd_bridge_of_remove(struct of_device *dev)
{
  struct platform_device *pdev = to_platform_device(&dev->dev);
  mtd_bridge_platform_remove(pdev);
  return(0);
}


/* Directly compatible with Lab X MTD Bridge peripherals. */
static struct of_device_id bridge_of_match[] = {
  { .compatible = "xlnx,lawo-mtd-bridge-1.00.a", },
  { /* end of list */ },
};


static struct of_platform_driver of_mtd_bridge_driver = {
  .name		     = DRIVER_NAME,
  .match_table = bridge_of_match,
  .probe   	   = mtd_bridge_of_probe,
  .remove      = __devexit_p(mtd_bridge_of_remove),
};
#endif

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int mtd_bridge_platform_probe(struct platform_device *pdev) {
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
  return(mtd_bridge_probe(pdev->name, pdev, addressRange, irq));
}

/* This is exported to allow polymorphic drivers to invoke it. */
int mtd_bridge_remove(struct mtd_bridge *bridge) {
  iounmap(bridge->virtualAddress);
  release_mem_region(bridge->physicalAddress, bridge->addressRangeSize);
  kfree(bridge);
  return 0;
}

static int mtd_bridge_platform_remove(struct platform_device *pdev) {
  struct mtd_bridge *bridge;

  /* Get a handle to the bridge and begin shutting it down */
  bridge = platform_get_drvdata(pdev);
  if(!bridge) return(-1);
  return(mtd_bridge_remove(bridge));
}

/* Platform device driver structure */
static struct platform_driver mtd_bridge_driver = {
  .probe  = mtd_bridge_platform_probe,
  .remove = mtd_bridge_platform_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __init mtd_bridge_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": MTD Bridge driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Lab X Technologies, LLC\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&of_mtd_bridge_driver);
#endif
 
  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&mtd_bridge_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }
  return 0;
}

static void __exit mtd_bridge_driver_exit(void)
{
  /* Unregister as a platform device driver */
  platform_driver_unregister(&mtd_bridge_driver);
}

module_init(mtd_bridge_driver_init);
module_exit(mtd_bridge_driver_exit);

MODULE_AUTHOR("Eldridge M. Mount IV <eldridge.mount@labxtechnologies.com>");
MODULE_DESCRIPTION("Lab X Technologies MTD bridge driver");
MODULE_LICENSE("GPL");
