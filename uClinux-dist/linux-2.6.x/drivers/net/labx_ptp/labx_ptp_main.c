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

/* Enable this to get some extra link debug messages */
/* #define DEBUG_LINK */
#ifdef DEBUG_LINK
#define DLPRINTK(args...) printk(##args)
#else
#define DLPRINTK(args...)
#endif

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF

/* Driver name and the revision of hardware expected. */
#define DRIVER_VERSION_MIN  0x11
#define DRIVER_VERSION_MAX  0x12

/* Major device number for the driver */
#define DRIVER_MAJOR 233

/* Maximum number of packetizers and instance count */
#define MAX_INSTANCES 64
static uint32_t instanceCount;

/* Default MAC address to use prior to an ioctl() setting it from userspace.
 * This is a Xilinx address since Lab X doesn't have a MAC address range
 * allocated... yet. ;)
 */
static uint8_t DEFAULT_SOURCE_MAC[MAC_ADDRESS_BYTES] = {
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

/*
 * Character device hook functions
 */

static int ptp_device_open(struct inode *inode, struct file *filp)
{
  struct ptp_device *ptp;
  unsigned long flags;
  int returnValue = 0;

  ptp = container_of(inode->i_cdev, struct ptp_device, cdev);
  filp->private_data = ptp;

  /* Lock the mutex and ensure there is only one client */
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  ptp->opened++;
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();

  return(returnValue);
}

static int ptp_device_release(struct inode *inode, struct file *filp)
{
  struct ptp_device *ptp = (struct ptp_device*)filp->private_data;
  unsigned long flags;

  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  ptp->opened--;
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();
  return(0);
}

static int ptp_device_event(struct notifier_block *nb, unsigned long event, void *ptr)
{
  struct net_device *dev = ptr;
  struct ptp_device *ptp = container_of(nb, struct ptp_device, notifier);
  int on;
  int i;
 
  if (event != NETDEV_CHANGE) return NOTIFY_DONE; /* Only interrested in carrier changes */

  on = netif_carrier_ok(dev);

  DLPRINTK(KERN_DEBUG "%s: ptp_device_event NETDEV_CHANGE, carrier %i\n",
    dev->name, on);

  for (i=0; i<ptp->numPorts; i++) {
#ifdef CONFIG_OF
    struct of_device* interfaceDev = of_find_device_by_node(ptp->ports[i].interfaceNode);
    struct device* matchDev = &dev->dev;
    int match = 0;

    DLPRINTK(KERN_DEBUG "%s: interfaceDev %p, name %s\n", dev->name, &interfaceDev->dev, kobject_name(&interfaceDev->dev.kobj));

    while (match == 0 && matchDev != NULL) {

      DLPRINTK(KERN_DEBUG "%s: matchDev %p, name %s\n", dev->name, matchDev, kobject_name(&matchDev->kobj));

      if (&interfaceDev->dev == matchDev) {
        match = 1;
      }
      matchDev = matchDev->parent;
    }
    if (match) {
#else
    if (0 == strncmp(dev->name, ptp->ports[i].interfaceName, IFNAMSIZ)) {
#endif

      DLPRINTK(KERN_DEBUG "%s: PTP port %d link state change\n", dev->name, i+1);

      if (on) {
        /* Enable Rx/Tx when the link comes up */

        ptp_enable_port(ptp,i);
        ptp->ports[i].portEnabled = 1;
      } else {
        /* Disable Rx/Tx when the link goes down */
        ptp_disable_port(ptp,i);
        ptp->ports[i].portEnabled = 0;
      }

      break;
    }
  }

  return NOTIFY_DONE;
}

/* Stops the PTP service */
void ptp_stop_service(struct ptp_device *ptp) {
  int i;

  /* Stopping the service is as simple as disabling all the interrupts */
  for (i=0; i<ptp->numPorts; i++) {
    ptp_disable_irqs(ptp,i);
  }

  /* Also stop the RTC */
  // disable_rtc(ptp);
}

/* Starts the PTP service */
void ptp_start_service(struct ptp_device *ptp) {
  int i;

  /* Initialize state machines */
  init_state_machines(ptp);

  for (i=0; i<ptp->numPorts; i++) {
    /* Clear the pending transmit interrupt flags */
    ptp->ports[i].pendingTxFlags = PTP_TX_BUFFER_NONE;

    /* Enable all of the interrupt sources, packet transmission of the messages we
     * need to capture Tx timestamps or send followups for, and packet reception.
     */
    ptp_enable_irqs(ptp,i);
    ptp_enable_port(ptp,i);
  }
}

static void PopulateDataSet(struct ptp_device *ptp, uint32_t port, PtpAsPortDataSet *dataSet) {
  
  memcpy(&dataSet->clockIdentity, ptp->properties.grandmasterIdentity, sizeof(PtpClockIdentity));
  dataSet->portNumber              = port + 1;
  
  if (!ptp->ports[port].portEnabled) {
    dataSet->portRole = PTP_DISABLED;
  } else if (ptp->presentRole == PTP_MASTER) {
    dataSet->portRole = PTP_MASTER;
  } else if ((ptp->presentRole == PTP_SLAVE) &&
           (ptp->presentMasterPort.portNumber == dataSet->portNumber)) {
    dataSet->portRole = PTP_SLAVE;
  } else {
    dataSet->portRole = PTP_PASSIVE;
  }
  dataSet->pttPortEnabled                 = ptp->ports[port].pttPortEnabled;
  dataSet->isMeasuringDelay               = ptp->ports[port].isMeasuringDelay;
  dataSet->asCapable                      = ptp->ports[port].asCapable;
  dataSet->neighborPropDelay              = ptp->ports[port].neighborPropDelay;
  dataSet->neighborPropDelayThresh        = ptp->ports[port].neighborPropDelayThresh;
  dataSet->delayAsymmetry                 = 0;
  dataSet->neighborRateRatio              = ptp->ports[port].neighborRateRatio;
  dataSet->initialLogAnnounceInterval     = ptp->ports[port].initialLogAnnounceInterval;
  dataSet->currentLogAnnounceInterval     = ptp->ports[port].currentLogAnnounceInterval;
  dataSet->announceReceiptTimeout         = ptp->ports[port].announceReceiptTimeout;
  dataSet->initialLogSyncInterval         = ptp->ports[port].initialLogSyncInterval;
  dataSet->currentLogSyncInterval         = ptp->ports[port].currentLogSyncInterval;
  dataSet->syncReceiptTimeout             = ptp->ports[port].syncReceiptTimeout;
  dataSet->syncReceiptTimeoutTimeInterval = 0; /* TODO */
  dataSet->initialLogPdelayReqInterval    = ptp->ports[port].initialLogPdelayReqInterval;
  dataSet->currentLogPdelayReqInterval    = ptp->ports[port].currentLogPdelayReqInterval;
  dataSet->allowedLostResponses           = ptp->ports[port].allowedLostResponses;
  dataSet->versionNumber                  = 2;
  dataSet->nup                            = 0;
  dataSet->ndown                          = 0;
  dataSet->acceptableMasterTableEnabled   = 0;
}

/* I/O control operations for the driver */
static int ptp_device_ioctl(struct inode *inode, struct file *filp,
                            unsigned int command, unsigned long arg)
{
  // Switch on the request
  struct ptp_device *ptp = (struct ptp_device*) filp->private_data;
  unsigned long flags;

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

      /* Copy the userspace argument into the device */
      preempt_disable();
      spin_lock_irqsave(&ptp->mutex, flags);
      copyResult = copy_from_user(&ptp->properties, (void __user*)arg, sizeof(PtpProperties));

      /* Having set the properties, load a set of default coefficients in depending
       * upon the selected delay mechanism
       */
      if(ptp->properties.delayMechanism == PTP_DELAY_MECHANISM_P2P) {
        /* Apply coefficients for the peer-to-peer delay mechanism */
        ptp->coefficients.P = DEFAULT_P2P_COEFF_P;
        ptp->coefficients.I = DEFAULT_P2P_COEFF_I;
        ptp->coefficients.D = DEFAULT_P2P_COEFF_D;
      } else {
        /* Apply coefficients for the end-to-end delay mechanism */
        printk("Using E2E coefficients\n");
        ptp->coefficients.P = DEFAULT_E2E_COEFF_P;
        ptp->coefficients.I = DEFAULT_E2E_COEFF_I;
        ptp->coefficients.D = DEFAULT_E2E_COEFF_D;
      }

      /* Convert the millisecond values for RTC lock settings into timer ticks.
       * Pre-calculating this avoids several divisions in the real-time code.
       */
      ptp->rtcLockTicks   = (ptp->properties.lockTimeMsec / PTP_TIMER_TICK_MS);
      ptp->rtcUnlockTicks = (ptp->properties.unlockTimeMsec / PTP_TIMER_TICK_MS);

      spin_unlock_irqrestore(&ptp->mutex, flags);
      preempt_enable();
      if(copyResult != 0) return(-EFAULT);
    }
    break;

  case IOC_PTP_GET_PORT_PROPERTIES:
    {
      uint32_t copyResult;
      PtpPortProperties properties = {};

      /* Copy the port properties from userspace to get the port number */
      copyResult = copy_from_user(&properties, (void __user*)arg, sizeof(PtpPortProperties));
      if(copyResult != 0) return(-EFAULT);

      /* Verify that it is a valid port number */
      if(properties.portNumber >= ptp->numPorts) return (-EINVAL);

      /* Copy data from the port structure */
      memcpy(properties.sourceMacAddress, ptp->ports[properties.portNumber].portProperties.sourceMacAddress, MAC_ADDRESS_BYTES);
      properties.stepsRemoved = ptp->ports[properties.portNumber].portProperties.stepsRemoved;

      /* Copy the properties into the userspace argument */
      copyResult = copy_to_user((void __user*)arg, &properties, sizeof(PtpPortProperties));
      if(copyResult != 0) return(-EFAULT);
    }
    break;

  case IOC_PTP_SET_PORT_PROPERTIES:
    {
      uint32_t copyResult;
      PtpPortProperties properties = {};

      /* Copy the userspace argument into the device */
      copyResult = copy_from_user(&properties, (void __user*)arg, sizeof(PtpPortProperties));
      if(copyResult != 0) return(-EFAULT);

      /* Verify that it is a valid port number */
      if(properties.portNumber >= ptp->numPorts) return (-EINVAL);

      preempt_disable();
      spin_lock_irqsave(&ptp->mutex, flags);

      /* Copy writeable data to the port structure */
      memcpy(ptp->ports[properties.portNumber].portProperties.sourceMacAddress, properties.sourceMacAddress, MAC_ADDRESS_BYTES);

      /* Reload the packet templates to propagate the new configuration information. */
      init_tx_templates(ptp, properties.portNumber);

      spin_unlock_irqrestore(&ptp->mutex, flags);
      preempt_enable();
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
      PtpCoefficients c = {};

      if(copy_from_user(&c, (void __user*)arg, sizeof(c)) != 0) {
        return(-EFAULT);
      }

      ptp->coefficients.P = c.P;
      ptp->coefficients.I = c.I;
      ptp->coefficients.D = c.D;
    }
    break;

  case IOC_PTP_GET_AS_PORT_DATA_SET:
    {
      uint32_t copyResult;
      PtpAsPortDataSet dataSet = {};

      /* Copy the port properties from userspace to get the port number */
      copyResult = copy_from_user(&dataSet, (void __user*)arg, sizeof(PtpAsPortDataSet));
      if(copyResult != 0) return(-EFAULT);

      /* Verify that it is a valid port number */
      if(dataSet.index >= ptp->numPorts) return (-EINVAL);

      PopulateDataSet(ptp, dataSet.index, &dataSet);

      /* Copy the properties into the userspace argument */
      copyResult = copy_to_user((void __user*)arg, &dataSet, sizeof(PtpAsPortDataSet));
      if(copyResult != 0) return(-EFAULT);
    }
    break;

  case IOC_PTP_GET_AS_PORT_STATISTICS:
    {
      uint32_t copyResult;
      PtpAsPortStatistics stats = {};

      /* Copy the port properties from userspace to get the port number */
      copyResult = copy_from_user(&stats, (void __user*)arg, sizeof(PtpAsPortStatistics));
      if(copyResult != 0) return(-EFAULT);

      /* Verify that it is a valid port number */
      if(stats.index >= ptp->numPorts) return (-EINVAL);

      /* Copy the properties into the userspace argument */
      copyResult = copy_to_user((void __user*)arg, &ptp->ports[stats.index].stats, sizeof(PtpAsPortStatistics));
      if(copyResult != 0) return(-EFAULT);
    }
    break;

  case IOC_PTP_ACK_GM_CHANGE:
    /* Simply acknowledge the Grandmaster change for the instance */
    ack_grandmaster_change(ptp);
    break;

  case IOC_PTP_GET_RTC_LOCKED:
    /* Copy the properties into the userspace argument */
    if (0 != copy_to_user((void __user*)arg, &ptp->rtcLockState, sizeof(uint32_t))) {
      return (-EFAULT);
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
  uint32_t versionCompare;
  int returnValue;
  int byteIndex;
  PtpClockQuality *quality;
  int i;

  /* Create and populate a device structure */
  ptp = (struct ptp_device*) kmalloc(sizeof(struct ptp_device), GFP_KERNEL);
  if(!ptp) return(-ENOMEM);
  memset(ptp, 0, sizeof(struct ptp_device));

  ptp->ports = (struct ptp_port*) kmalloc(sizeof(struct ptp_port)*platformData->numPorts, GFP_KERNEL);
  if(!ptp->ports) {
    returnValue = -ENOMEM;
    goto free_ptp;
  }
  memset(ptp->ports, 0, sizeof(struct ptp_port)*platformData->numPorts);

  /* Assign basic port configuration information */
  ptp->numPorts  = platformData->numPorts;
  ptp->portWidth = platformData->portWidth;

  /* Request and map the device's I/O memory region into uncacheable space */
  ptp->physicalAddress = addressRange->start;
  ptp->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
  snprintf(ptp->name, NAME_MAX_SIZE, "%s", name);
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
  versionWord = ptp_get_version(ptp);
  versionMajor = ((versionWord >> REVISION_FIELD_BITS) & REVISION_FIELD_MASK);
  versionMinor = (versionWord & REVISION_FIELD_MASK);
  versionCompare = ((versionMajor << REVISION_FIELD_BITS) | versionMinor);
  if((versionCompare < DRIVER_VERSION_MIN) | 
     (versionCompare > DRIVER_VERSION_MAX)) {
    printk(KERN_INFO "%s: Found incompatible hardware version %d.%d at 0x%08X\n",
           ptp->name, versionMajor, versionMinor, (uint32_t)ptp->physicalAddress);
    returnValue = -ENXIO;
    goto unmap;
  }

  /* Ensure that the interrupts are disabled */
  for (i=0; i<ptp->numPorts; i++) {
    ptp_disable_irqs(ptp,i);
    ptp->ports[i].pendingTxFlags = PTP_TX_BUFFER_NONE;
  }

  /* Retain the IRQ and register our handler */
  ptp->irq = irq->start;

  returnValue = ptp_setup_interrupt(ptp);
  if (returnValue) {
    printk(KERN_ERR "%s: : Could not allocate Lab X PTP interrupt (%d).\n",
           ptp->name, ptp->irq);
    goto unmap;
  }

  /* Older versions may not provide a port width, as they pre-date support
   * for 10G Ethernet
   */
  if((ptp->portWidth != 8) & (ptp->portWidth != 64)) {
    printk(KERN_INFO "%s: No port width specified by platform, assuming 8-bit\n",
           ptp->name);
    ptp->portWidth = 8;
  }

  /* Announce the device */
  printk(KERN_INFO "%s: Found Lab X PTP hardware %d.%d at 0x%08X, IRQ %d, Ports %d, Width %d bits\n", 
         ptp->name,
         versionMajor,
         versionMinor,
         (uint32_t)ptp->physicalAddress,
         ptp->irq,
         ptp->numPorts,
         ptp->portWidth);

  /* Initialize other resources */
  spin_lock_init(&ptp->mutex);
  ptp->opened = false;

  /* Provide navigation from the platform device structure */
  platform_set_drvdata(pdev, ptp);

  /* Add as a character device to make the instance available for use */
  cdev_init(&ptp->cdev, &ptp_device_fops);
  ptp->cdev.owner = THIS_MODULE;
  ptp->instanceNumber = instanceCount++;
  kobject_set_name(&ptp->cdev.kobj, "%s.%d", name, ptp->instanceNumber);
  cdev_add(&ptp->cdev, MKDEV(DRIVER_MAJOR, ptp->instanceNumber), 1);

  ptp_setup_event_timer(ptp, 0, platformData);

  /* Configure defaults and initialize the transmit templates */
  quality = &ptp->properties.grandmasterClockQuality;
  for(i=0; i<ptp->numPorts; i++) {
    ptp->ports[i].portProperties.portNumber = i;

    for(byteIndex = 0; byteIndex < MAC_ADDRESS_BYTES; byteIndex++) {
      ptp->ports[i].portProperties.sourceMacAddress[byteIndex] = DEFAULT_SOURCE_MAC[byteIndex];
    }

    ptp->ports[i].portProperties.stepsRemoved = 0;
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

  for(i=0; i<ptp->numPorts; i++) {
    init_tx_templates(ptp, i);
  }

  /* Configure the instance's RTC control loop based on data provided by
   * the platform when it defines the device.  The default coefficients will
   * likely be reconfigured by userspace at run-time; however the nominal
   * increment is directly tied to the period of the RTC clock supplied to
   * the Ptp_Hardware module by the platform - therefore this *must* have a
   * sane value!
   */
  if((platformData->nominalIncrement.mantissa < LABX_PTP_RTC_INC_MIN) |
     (platformData->nominalIncrement.mantissa > LABX_PTP_RTC_INC_MAX)) {
    returnValue = -EINVAL;
    printk(KERN_ERR "%s: Nominal RTC increment mantissa (%d) is out of range [%d, %d]\n",
           ptp->name, platformData->nominalIncrement.mantissa,
           LABX_PTP_RTC_INC_MIN, LABX_PTP_RTC_INC_MAX);
    goto unmap;
  }    
  if((platformData->nominalIncrement.fraction & ~RTC_FRACTION_MASK) != 0) {
    returnValue = -EINVAL;
    printk(KERN_ERR "%s: Nominal RTC increment fraction (0x%08X) has > %d significant bits\n",
           ptp->name, platformData->nominalIncrement.fraction, LABX_PTP_RTC_FRACTION_BITS);
    goto unmap;
  }    
  ptp->nominalIncrement.mantissa = platformData->nominalIncrement.mantissa;
  ptp->nominalIncrement.fraction = platformData->nominalIncrement.fraction;

  /* Zero the coefficients initially; they will be inferred from the port mode */
  ptp->coefficients.P = 0x00000000;
  ptp->coefficients.I = 0x00000000;
  ptp->coefficients.D = 0x00000000;

  /* Assign the MAC transmit and receive latency */
  for(i=0; i<ptp->numPorts; i++) {
    ptp->ports[i].rxPhyMacDelay = platformData->rxPhyMacDelay;
    ptp->ports[i].txPhyMacDelay = platformData->txPhyMacDelay;
  }

  /* Set up linked ethernet devices */
  if (NULL != platformData->interfaceName) {
    for(i=0; i<ptp->numPorts; i++) {
      strncpy(ptp->ports[i].interfaceName, platformData->interfaceName[i], IFNAMSIZ);
    }
  }
#ifdef CONFIG_OF
  if (NULL != platformData->interfaceNode) {
    for(i=0; i<ptp->numPorts; i++) {
      ptp->ports[i].interfaceNode = platformData->interfaceNode[i];
    }
  }
#endif

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
  kfree(ptp->ports);
 free_ptp:
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
  const char *name = dev_name(&ofdev->dev);
  PtpPlatformData platformData;
  int rc = 0;
  int i;

  /* Obtain the resources for this instance */
  rc = of_address_to_resource(ofdev->node, 0, addressRange);
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
  platformData.numPorts                  = get_u32(ofdev,"xlnx,num-ports");
  platformData.portWidth                 = get_u32(ofdev,"xlnx,port-width");
  platformData.timerPrescaler            = get_u32(ofdev,"xlnx,timer-prescaler");
  platformData.timerDivider              = get_u32(ofdev,"xlnx,timer-divider");
  platformData.nominalIncrement.mantissa = get_u32(ofdev,"xlnx,nominal-increment-mantissa");
  platformData.nominalIncrement.fraction = get_u32(ofdev,"xlnx,nominal-increment-fraction");
  platformData.coefficients.P            = get_u32(ofdev,"xlnx,rtc-p-coefficient");
  platformData.coefficients.I            = get_u32(ofdev,"xlnx,rtc-i-coefficient");
  platformData.coefficients.D            = get_u32(ofdev,"xlnx,rtc-d-coefficient");

  platformData.rxPhyMacDelay.secondsUpper = 0;
  platformData.rxPhyMacDelay.secondsLower = 0;
  platformData.rxPhyMacDelay.nanoseconds = get_u32(ofdev,"xlnx,phy-mac-rx-delay");
  platformData.txPhyMacDelay.secondsUpper = 0;
  platformData.txPhyMacDelay.secondsLower = 0;
  platformData.txPhyMacDelay.nanoseconds = get_u32(ofdev,"xlnx,phy-mac-tx-delay");


  /* Get the attached ethernet device nodes; NULL the string pointer */
  platformData.interfaceName = NULL;
  platformData.interfaceNode = kmalloc(sizeof(void*)*platformData.numPorts, GFP_KERNEL);
  for (i=0; i<platformData.numPorts; i++) {
    char propName[64];
    const phandle *interface_handle = NULL;
    struct device_node *interface_node = NULL;
    snprintf(propName, 64, "port-interface-%d", i);

    interface_handle = of_get_property(ofdev->node, propName, NULL);
    if (!interface_handle) {
      dev_warn(&ofdev->dev, "no PTP ethernet connection specified for port %d.\n", i+1);
    } else {
      interface_node = of_find_node_by_phandle(*interface_handle);
      if (!interface_node) {
        dev_warn(&ofdev->dev, "no PTP ethernet connection found for port %d.\n", i+1);
      }
    }
    platformData.interfaceNode[i] = interface_node;
  }

  /* Dispatch to the common probe function */
  return(ptp_probe(name, pdev, addressRange, irq, &platformData));
}

static int __devexit ptp_of_remove(struct of_device *dev)
{
	struct platform_device *pdev = to_platform_device(&dev->dev);
	ptp_remove(pdev);
	return(0);
}

static struct of_device_id ptp_of_match[] = {
	{ .compatible = "xlnx,labx-ptp-1.00.a", },
	{ .compatible = "xlnx,labx-ptp-1.01.a", },
	{ .compatible = "xlnx,labx-ptp-1.02.a", },
	{ .compatible = "xlnx,labx-ptp-1.03.a", },
	{ .compatible = "xlnx,labx-ptp-1.04.a", },
	{ .compatible = "xlnx,labx-ptp-1.05.a", },
	{ .compatible = "xlnx,labx-ptp-1.06.a", },
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
  printk(KERN_INFO DRIVER_NAME ": PTP hardware driver\n");
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
    printk(KERN_INFO DRIVER_NAME ": Failed to allocate character device range\n");
  }

  /* Initialize the Netlink layer for the driver */
  register_ptp_netlink();

  return(0);
}

static void __exit ptp_driver_exit(void)
{
  /* Unregister Generic Netlink family */
  unregister_ptp_netlink();

  /* Unregister as a platform device driver */
  platform_driver_unregister(&ptp_driver);
}

module_init(ptp_driver_init);
module_exit(ptp_driver_exit);

MODULE_AUTHOR("Eldridge M. Mount IV <eldridge.mount@labxtechnologies.com>");
MODULE_DESCRIPTION("Lab X Technologies PTP 2.0 driver");
MODULE_LICENSE("GPL");

