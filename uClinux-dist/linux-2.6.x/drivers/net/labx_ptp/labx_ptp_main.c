/*
 *  linux/drivers/net/labx_ptp_main.c
 *
 *  Lab X Technologies Precision Time Protocol (PTP) driver
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

#include "labx_ptp.h"
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <xio.h>

/* Enable this to get some extra link debug messages */
// #define DEBUG_LINK
#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF

/* Driver name and the revision of hardware expected. */
#define DRIVER_NAME "labx_ptp"
#define HARDWARE_VERSION_MAJOR  1
#define HARDWARE_VERSION_MINOR  1

/* Major device number for the driver */
#define DRIVER_MAJOR 253

/* Maximum number of packetizers and instance count */
#define MAX_INSTANCES 64
static uint32_t instanceCount;

/* Default MAC address to use prior to an ioctl() setting it from userspace.
 * This is a Xilinx address since Lab X doesn't have a MAC address range
 * allocated... yet. ;)
 */
static u8 DEFAULT_SOURCE_MAC[MAC_ADDRESS_BYTES] = {
  0x00, 0x0A, 0x35, 0x00, 0x22, 0xFF
};

/* Default PTP properties; where applicable, defaults from IEEE P802.1AS are used */
#define DEFAULT_DOMAIN_NUM             (0x00)
#define DEFAULT_UTC_OFFSET             (-5)
#define DEFAULT_GRANDMASTER_PRIORITY1  (248)
#define DEFAULT_GRANDMASTER_PRIORITY2  (248)
#define DEFAULT_CLOCK_CLASS            (248)
#define DEFAULT_CLOCK_ACCURACY         (0xFE)
#define DEFAULT_SCALED_LOG_VARIANCE    (0x4100)
#define DEFAULT_TIME_SOURCE            (PTP_SOURCE_OTHER)
#define DEFAULT_DELAY_MECHANISM        (PTP_DELAY_MECHANISM_E2E)

#if 0
#define DBG(f, x...) pr_debug(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

/* Interrupt service routine for the instance */
static irqreturn_t labx_ptp_interrupt(int irq, void *dev_id)
{
  struct ptp_device *ptp = dev_id;
  uint32_t maskedFlags;
  uint32_t txCompletedFlags;
  uint32_t flags;

  /* Read the interrupt flags and immediately clear them */
  maskedFlags = XIo_In32(REGISTER_ADDRESS(ptp, PTP_IRQ_FLAGS_REG));
  maskedFlags &= XIo_In32(REGISTER_ADDRESS(ptp, PTP_IRQ_MASK_REG));
  XIo_Out32(REGISTER_ADDRESS(ptp, PTP_IRQ_FLAGS_REG), maskedFlags);

  /* Detect the timer IRQ */
  if((maskedFlags & PTP_TIMER_IRQ) != 0) {
    /* Kick off the timer tasklet */
    tasklet_schedule(&ptp->timerTasklet);
  }

  /* Detect the Rx IRQ */
  if((maskedFlags & PTP_RX_IRQ) != 0) {
    /* Kick off the Rx tasklet */
    tasklet_schedule(&ptp->rxTasklet);
  }
  
  /* Detect the Tx IRQ from any enabled buffer bits */
  txCompletedFlags = (maskedFlags & PTP_TX_IRQ_MASK);
  if(txCompletedFlags != PTP_TX_BUFFER_NONE) {
    /* Add the new pending Tx buffer IRQ flags to the mask in the device
     * structure for the tasklet to whittle away at.  Lock the mutex so we
     * avoid a race condition with the Tx tasklet.
     */
    preempt_disable();
    spin_lock_irqsave(&ptp->mutex, flags);
    ptp->pendingTxFlags |= txCompletedFlags;
    spin_unlock_irqrestore(&ptp->mutex, flags);
    preempt_enable();

    /* Now kick off the Tx tasklet */
    tasklet_schedule(&ptp->txTasklet);
  }
  
  return(IRQ_HANDLED);
}

/*
 * Character device hook functions
 */

static int ptp_device_open(struct inode *inode, struct file *filp)
{
  struct ptp_device *ptp;
  uint32_t flags;
  int returnValue = 0;

  ptp = container_of(inode->i_cdev, struct ptp_device, cdev);
  filp->private_data = ptp;

  /* Lock the mutex and ensure there is only one client */
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  if(ptp->opened) {
    returnValue = -1;
  } else {
    ptp->opened = true;
  }

  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();

  return(returnValue);
}

static int ptp_device_release(struct inode *inode, struct file *filp)
{
  struct ptp_device *ptp = (struct ptp_device*)filp->private_data;
  uint32_t flags;

  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  ptp->opened = false;
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();
  return(0);
}

static int ptp_device_event(struct notifier_block *nb, unsigned long event, void *ptr)
{
  struct net_device *dev = ptr;
  struct ptp_device *ptp = container_of(nb, struct ptp_device, notifier);
  int on;
 
  if (event != NETDEV_CHANGE) return NOTIFY_DONE; /* Only interrested in carrier changes */

  on = netif_carrier_ok(dev);

#ifdef DEBUG_LINK
  printk(KERN_DEBUG "%s: ptp_device_event NETDEV_CHANGE, carrier %i\n",
    dev->name, on);
#endif

  /* TODO: If there are multiple interfaces we should discriminate based on which interface changed state */

  if (on)
  {
    /* Enable Rx/Tx when the link comes up */
    XIo_Out32(REGISTER_ADDRESS(ptp, PTP_TX_REG), XIo_In32(REGISTER_ADDRESS(ptp, PTP_TX_REG)) | PTP_TX_ENABLE);
    XIo_Out32(REGISTER_ADDRESS(ptp, PTP_RX_REG), XIo_In32(REGISTER_ADDRESS(ptp, PTP_RX_REG)) | PTP_RX_ENABLE);

    ptp->portEnabled = TRUE;
  }
  else
  {
    /* Disable Rx/Tx when the link goes down */
    XIo_Out32(REGISTER_ADDRESS(ptp, PTP_TX_REG), XIo_In32(REGISTER_ADDRESS(ptp, PTP_TX_REG)) & ~PTP_TX_ENABLE);
    XIo_Out32(REGISTER_ADDRESS(ptp, PTP_RX_REG), XIo_In32(REGISTER_ADDRESS(ptp, PTP_RX_REG)) & ~PTP_RX_ENABLE);

    ptp->portEnabled = FALSE;
  }

  return NOTIFY_DONE;
}

/* Stops the PTP service */
static void ptp_stop_service(struct ptp_device *ptp) {
  /* Stopping the service is as simple as disabling all the interrupts */
  /* Configure the timer and enable its interrupt.  Enabling the mask
   * bit for the timer IRQ also enables the timer itself.
   */
  XIo_Out32(REGISTER_ADDRESS(ptp, PTP_IRQ_MASK_REG), PTP_NO_IRQS);

  /* Also stop the RTC */
  disable_rtc(ptp);
}

/* Starts the PTP service */
static void ptp_start_service(struct ptp_device *ptp) {
  uint32_t irqMask;

  /* Initialize state machines */
  init_state_machines(ptp);

  /* Clear the pending transmit interrupt flags */
  ptp->pendingTxFlags = PTP_TX_BUFFER_NONE;

  /* Enable all of the interrupt sources, packet transmission of the messages we
   * need to capture Tx timestamps or send followups for, and packet reception.
   */
  irqMask = (PTP_TIMER_IRQ | PTP_RX_IRQ | 
             PTP_TX_IRQ(PTP_TX_SYNC_BUFFER) |
             PTP_TX_IRQ(PTP_TX_DELAY_REQ_BUFFER) |
             PTP_TX_IRQ(PTP_TX_PDELAY_REQ_BUFFER) |
             PTP_TX_IRQ(PTP_TX_PDELAY_RESP_BUFFER));
  XIo_Out32(REGISTER_ADDRESS(ptp, PTP_IRQ_FLAGS_REG), irqMask);
  XIo_Out32(REGISTER_ADDRESS(ptp, PTP_IRQ_MASK_REG), irqMask);
  XIo_Out32(REGISTER_ADDRESS(ptp, PTP_TX_REG), PTP_TX_ENABLE);
  XIo_Out32(REGISTER_ADDRESS(ptp, PTP_RX_REG), PTP_RX_ENABLE);
}

/* I/O control operations for the driver */
static int ptp_device_ioctl(struct inode *inode, struct file *filp,
                            unsigned int command, unsigned long arg)
{
  // Switch on the request
  struct ptp_device *ptp = (struct ptp_device*) filp->private_data;
  uint32_t flags;

  switch(command) {
  case IOC_PTP_STOP_SERVICE:
    ptp_stop_service(ptp);
    break;

  case IOC_PTP_START_SERVICE:
    ptp_start_service(ptp);
    break;

  case IOC_PTP_GET_PROPERTIES:
    {
      uint32_t copyResult;

      /* Copy the properties into the userspace argument */
      copyResult = copy_to_user((void __user*)arg, &ptp->properties, sizeof(PtpProperties));
      if(copyResult != 0) return(-EFAULT);
    }
    break;

  case IOC_PTP_SET_PROPERTIES:
    {
      uint32_t copyResult;

      /* Copy the userspace argument into the device, then reload the packet
       * templates to propagate the new configuration information.
       */
      preempt_disable();
      spin_lock_irqsave(&ptp->mutex, flags);
      copyResult = copy_from_user(&ptp->properties, (void __user*)arg, sizeof(PtpProperties));
      if(copyResult == 0) init_tx_templates(ptp);
      spin_unlock_irqrestore(&ptp->mutex, flags);
      preempt_enable();
      if(copyResult != 0) return(-EFAULT);
    }
    break;

  case IOC_PTP_GET_TIME:
    {
      PtpTime capturedTime;

      get_rtc_time(ptp, &capturedTime);
      if(copy_to_user((void __user*)arg, &capturedTime, sizeof(capturedTime)) != 0) {
        return(-EFAULT);
      }
    }
    break;

  case IOC_PTP_SET_TIME:
    {
      PtpTime setTime;

      if(copy_from_user(&setTime, (void __user*)arg, sizeof(setTime)) != 0) {
        return(-EFAULT);
      }
      set_rtc_time(ptp, &setTime);
    }
    break;

  case IOC_PTP_GET_RTC_COEF:
    {
      PtpCoefficients c;

      c.P = ptp->coefficients.P;
      c.I = ptp->coefficients.I;
      c.D = ptp->coefficients.D;

      if(copy_to_user((void __user*)arg, &c, sizeof(c)) != 0) {
        return(-EFAULT);
      }
    }
    break;

  case IOC_PTP_SET_RTC_COEF:
    {
      PtpCoefficients c;

      if(copy_from_user(&c, (void __user*)arg, sizeof(c)) != 0) {
        return(-EFAULT);
      }

      ptp->coefficients.P = c.P;
      ptp->coefficients.I = c.I;
      ptp->coefficients.D = c.D;
    }
    break;

  default:
    return(-EINVAL);
  }

  return(0);
}

/* Character device file operations structure */
static struct file_operations ptp_device_fops = {
  .open	   = ptp_device_open,
  .release = ptp_device_release,
  .ioctl   = ptp_device_ioctl,
  .owner   = THIS_MODULE,
};

/* Common, factored-out function which provides the "meat" of the probe
 * functionality regardless of how it is invoked.
 */
static int ptp_probe(const char *name,
                     struct platform_device *pdev,
                     struct resource *addressRange,
                     struct resource *irq,
                     PtpPlatformData *platformData)
{
  struct ptp_device *ptp;
  uint32_t versionWord;
  uint32_t versionMajor;
  uint32_t versionMinor;
  int returnValue;
  int byteIndex;
  PtpClockQuality *quality;

  /* Create and populate a device structure */
  ptp = (struct ptp_device*) kmalloc(sizeof(struct ptp_device), GFP_KERNEL);
  if(!ptp) return(-ENOMEM);

  /* Request and map the device's I/O memory region into uncacheable space */
  ptp->physicalAddress = addressRange->start;
  ptp->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
  snprintf(ptp->name, NAME_MAX_SIZE, "%s%d", name, pdev->id);
  ptp->name[NAME_MAX_SIZE - 1] = '\0';
  if(request_mem_region(ptp->physicalAddress, ptp->addressRangeSize,
                        ptp->name) == NULL) {
    returnValue = -ENOMEM;
    goto free;
  }

  ptp->virtualAddress = 
    (void*) ioremap_nocache(ptp->physicalAddress, ptp->addressRangeSize);
  if(!ptp->virtualAddress) {
    returnValue = -ENOMEM;
    goto release;
  }

  /* Inspect and check the version */
  versionWord = XIo_In32(REGISTER_ADDRESS(ptp, PTP_REVISION_REG));
  versionMajor = ((versionWord >> REVISION_FIELD_BITS) & REVISION_FIELD_MASK);
  versionMinor = (versionWord & REVISION_FIELD_MASK);
  if((versionMajor != HARDWARE_VERSION_MAJOR) | 
     (versionMinor != HARDWARE_VERSION_MINOR)) {
    printk(KERN_INFO "%s: Found incompatible hardware version %d.%d at 0x%08X\n",
           ptp->name, versionMajor, versionMinor, (uint32_t)ptp->physicalAddress);
    returnValue = -ENXIO;
    goto unmap;
  }

  /* Ensure that the interrupts are disabled */
  XIo_Out32(REGISTER_ADDRESS(ptp, PTP_IRQ_MASK_REG), PTP_NO_IRQS);

  /* Retain the IRQ and register our handler */
  ptp->irq = irq->start;
  returnValue = request_irq(ptp->irq, &labx_ptp_interrupt, IRQF_DISABLED, "Lab X PTP", ptp);
  if (returnValue) {
    printk(KERN_ERR "%s: : Could not allocate Lab X PTP interrupt (%d).\n",
           ptp->name, ptp->irq);
    goto unmap;
  }
  ptp->pendingTxFlags = PTP_TX_BUFFER_NONE;

  /* Announce the device */
  printk(KERN_INFO "%s: Found Lab X PTP hardware %d.%d at 0x%08X, IRQ %d\n", 
         ptp->name,
         HARDWARE_VERSION_MAJOR,
         HARDWARE_VERSION_MINOR,
         (uint32_t)ptp->physicalAddress,
         ptp->irq);

  /* Initialize other resources */
  spin_lock_init(&ptp->mutex);
  ptp->opened = false;

  /* Provide navigation from the platform device structure */
  platform_set_drvdata(pdev, ptp);

  /* Add as a character device to make the instance available for use */
  cdev_init(&ptp->cdev, &ptp_device_fops);
  ptp->cdev.owner = THIS_MODULE;
  kobject_set_name(&ptp->cdev.kobj, "%s%d", pdev->name, pdev->id);
  ptp->instanceNumber = instanceCount++;
  cdev_add(&ptp->cdev, MKDEV(DRIVER_MAJOR, ptp->instanceNumber), 1);

  /* Configure the prescaler and divider used to generate a 10 msec event timer.
   * The register values are terminal counts, so are one less than the count value.
   */
  XIo_Out32(REGISTER_ADDRESS(ptp, PTP_TIMER_REG), 
            (((platformData->timerPrescaler - 1) & PTP_PRESCALER_MASK) |
             (((platformData->timerDivider - 1) & PTP_DIVIDER_MASK) << PTP_DIVIDER_SHIFT)));

  /* Assign the MAC transmit and receive latency
   * TODO: The MAC latency should be specified as a platform resource!
   */

  /* Configure defaults and initialize the transmit templates */
  quality = &ptp->properties.grandmasterClockQuality;
  for(byteIndex = 0; byteIndex < MAC_ADDRESS_BYTES; byteIndex++) {
    ptp->properties.sourceMacAddress[byteIndex] = DEFAULT_SOURCE_MAC[byteIndex];
  }
  ptp->properties.domainNumber         = DEFAULT_DOMAIN_NUM;
  ptp->properties.currentUtcOffset     = DEFAULT_UTC_OFFSET;
  ptp->properties.grandmasterPriority1 = DEFAULT_GRANDMASTER_PRIORITY1;
  quality->clockClass                  = DEFAULT_CLOCK_CLASS;
  quality->clockAccuracy               = DEFAULT_CLOCK_ACCURACY;
  quality->offsetScaledLogVariance     = DEFAULT_SCALED_LOG_VARIANCE;
  ptp->properties.grandmasterPriority2 = DEFAULT_GRANDMASTER_PRIORITY2;
  ptp->properties.timeSource           = DEFAULT_TIME_SOURCE;

  ptp->properties.grandmasterIdentity[0] = DEFAULT_SOURCE_MAC[0];
  ptp->properties.grandmasterIdentity[1] = DEFAULT_SOURCE_MAC[1];
  ptp->properties.grandmasterIdentity[2] = DEFAULT_SOURCE_MAC[2];
  ptp->properties.grandmasterIdentity[3] = 0xFF;
  ptp->properties.grandmasterIdentity[4] = 0xFE;
  ptp->properties.grandmasterIdentity[5] = DEFAULT_SOURCE_MAC[3];
  ptp->properties.grandmasterIdentity[6] = DEFAULT_SOURCE_MAC[4];
  ptp->properties.grandmasterIdentity[7] = DEFAULT_SOURCE_MAC[5];


  ptp->properties.delayMechanism       = DEFAULT_DELAY_MECHANISM;
  init_tx_templates(ptp);

  /* Configure the instance's RTC control loop based on data provided by
   * the platform when it defines the device.
   */
  ptp->nominalIncrement.mantissa = platformData->nominalIncrement.mantissa;
  ptp->nominalIncrement.fraction = platformData->nominalIncrement.fraction;
  ptp->coefficients.P = platformData->coefficients.P;
  ptp->coefficients.I = platformData->coefficients.I;
  ptp->coefficients.D = platformData->coefficients.D;

  /* Register for network device events */
  ptp->notifier.notifier_call = ptp_device_event;

  if (register_netdevice_notifier(&ptp->notifier) != 0)
  {
    /* TODO: anything to do if we can't register for events? */
  }

  /* Return success */
  return(0);

 unmap:
  iounmap(ptp->virtualAddress);
 release:
  release_mem_region(ptp->physicalAddress, ptp->addressRangeSize);
 free:
  kfree(ptp);
  return(returnValue);
}

#ifdef CONFIG_OF
static int ptp_remove(struct platform_device *pdev);

static u32 get_u32(struct of_device *ofdev, const char *s) {
	u32 *p = (u32 *)of_get_property(ofdev->node, s, NULL);
	if(p) {
		return *p;
	} else {
		dev_warn(&ofdev->dev, "Parameter %s not found, defaulting to 0.\n", s);
		return 0;
	}
}
static int __devinit ptp_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
  struct resource r_mem_struct;
  struct resource *addressRange = &r_mem_struct;
  struct resource r_irq_struct;
  struct resource *irq = &r_irq_struct;
  struct platform_device *pdev = to_platform_device(&ofdev->dev);
  PtpPlatformData platformData;
  int rc = 0;

  printk(KERN_INFO "Device Tree Probing \'%s\'\n", ofdev->node->name);

  /* Obtain the resources for this instance */
  rc = of_address_to_resource(ofdev->node,0,addressRange);
  if (rc) {
	  dev_warn(&ofdev->dev,"invalid address\n");
	  return rc;
  }

  /* Get IRQ for the device */
  rc = of_irq_to_resource(ofdev->node, 0, irq);
  if(rc == NO_IRQ) {
    dev_warn(&ofdev->dev, "no IRQ found.\n");
    return rc;
  }

  /* Consult the device tree for other required parameters */
  platformData.timerPrescaler            = get_u32(ofdev,"xlnx,timer-prescaler");
  platformData.timerDivider              = get_u32(ofdev,"xlnx,timer-divider");
  platformData.nominalIncrement.mantissa = get_u32(ofdev,"xlnx,nominal-increment-mantissa");
  platformData.nominalIncrement.fraction = get_u32(ofdev,"xlnx,nominal-increment-fraction");
  platformData.coefficients.P            = get_u32(ofdev,"xlnx,rtc-p-coefficient");
  platformData.coefficients.I            = get_u32(ofdev,"xlnx,rtc-i-coefficient");
  platformData.coefficients.D            = get_u32(ofdev,"xlnx,rtc-d-coefficient");

  /* Dispatch to the common probe function */
  return(ptp_probe(ofdev->node->name, pdev, addressRange, irq, &platformData));
}

static int __devexit ptp_of_remove(struct of_device *dev)
{
	struct platform_device *pdev = to_platform_device(&dev->dev);
	ptp_remove(pdev);
	return(0);
}

static struct of_device_id ptp_of_match[] = {
	{ .compatible = "xlnx,labx-ptp-1.00.a", },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, ptp_of_match);

static struct of_platform_driver of_ptp_driver = {
	.name		= DRIVER_NAME,
	.match_table	= ptp_of_match,
	.probe		= ptp_of_probe,
	.remove		= __devexit_p(ptp_of_remove),
};
#endif // CONFIG_OF


/*
 * Platform device hook functions
 */

static int ptp_probe_platform(struct platform_device *pdev)
{
  struct resource *addressRange;
  struct resource *irq;
  PtpPlatformData *platformData;

  /* Obtain the resources for this instance */
  addressRange = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
  if (!addressRange | !irq) return -ENXIO;

  /* Get a pointer to platform data provided specific to the board */
  platformData = (PtpPlatformData*) pdev->dev.platform_data;

  /* Dispatch to the common probe function */
  return(ptp_probe(pdev->name, pdev, addressRange, irq, platformData));
}

static int ptp_remove(struct platform_device *pdev)
{
  struct ptp_device *ptp;

  /* Get a handle to the ptp and begin shutting it down */

  /* TODO: release IRQ and disable interrupts (in reverse order...)! */
  ptp = platform_get_drvdata(pdev);
  if(!ptp) return(-1);

  unregister_netdevice_notifier(&ptp->notifier);

  iounmap(ptp->virtualAddress);
  release_mem_region(ptp->physicalAddress, ptp->addressRangeSize);
  kfree(ptp);
  return(0);
}

/* Platform device driver structure */
static struct platform_driver ptp_driver = {
  .probe  = ptp_probe_platform,
  .remove = ptp_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Types and methods to maintain a linked list of callbacks.  The methods are
 * exported to allow other "in the know" kernel modules to invoke them.
 *
 * NOTE - This is a legacy implementation, left over from the Xilinx LLTEMAC
 *        driver.  This was necessary since the Xilinx AVB Endpoint did not provide
 *        RTC increment information as hardware signals.  In order to maintain a
 *        syntonized counter elsewhere, the driver must perform duplicate register
 *        writes whenever the RTC increment is updated, and this callback mechanism
 *        achieves that.
 *
 *        The Lab X PTP hardware *does* provide the RTC increment as an external signal,
 *        and therefore this implementation is entirely empty - a system making use of this
 *        hardware should wire up the hardware signals instead.
 */

void add_syntonize_callback(void (*callbackFunc)(uint32_t, void*),
                            void *callbackParam) {
}
EXPORT_SYMBOL(add_syntonize_callback);

void remove_syntonize_callback(void (*callbackFunc)(uint32_t, void*),
                               void *callbackParam) {
}
EXPORT_SYMBOL(remove_syntonize_callback);

/* Driver initialization and exit */
static int __init ptp_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": PTP hardware %d.%d driver\n",
         HARDWARE_VERSION_MAJOR, HARDWARE_VERSION_MINOR);
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Lab X Technologies, LLC\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&of_ptp_driver);
#endif
  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&ptp_driver)) < 0) {
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

static void __exit ptp_driver_exit(void)
{
  /* Unregister as a platform device driver */
  platform_driver_unregister(&ptp_driver);
}

module_init(ptp_driver_init);
module_exit(ptp_driver_exit);

MODULE_AUTHOR("Eldridge M. Mount IV <eldridge.mount@labxtechnologies.com>");
MODULE_DESCRIPTION("Lab X Technologies PTP 2.0 driver");
MODULE_LICENSE("GPL");
