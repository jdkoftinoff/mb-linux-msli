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
 * Locally Administered, zero value. This should never be sent.
 */
static uint8_t DEFAULT_SOURCE_MAC[MAC_ADDRESS_BYTES] = {
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00
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
        ptp->ports[i].portEnabled = TRUE;
      } else {
        /* Disable Rx/Tx when the link goes down */
        ptp_disable_port(ptp,i);
        ptp->ports[i].portEnabled = FALSE;
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

  struct ptp_port *pPort = &ptp->ports[port];

  memcpy(&dataSet->clockIdentity, ptp->properties.grandmasterIdentity, sizeof(PtpClockIdentity));
  dataSet->portNumber                     = (port + 1);
  dataSet->portRole                       = pPort->selectedRole;
  dataSet->pttPortEnabled                 = pPort->pttPortEnabled;
  dataSet->isMeasuringDelay               = pPort->isMeasuringDelay;
  dataSet->asCapable                      = pPort->asCapable;
  dataSet->neighborPropDelay              = pPort->neighborPropDelay;
  dataSet->neighborPropDelayThresh        = pPort->neighborPropDelayThresh;
  dataSet->delayAsymmetry                 = 0;
  dataSet->neighborRateRatio              = pPort->neighborRateRatio;
  dataSet->initialLogAnnounceInterval     = pPort->initialLogAnnounceInterval;
  dataSet->currentLogAnnounceInterval     = pPort->currentLogAnnounceInterval;
  dataSet->announceReceiptTimeout         = pPort->announceReceiptTimeout;
  dataSet->initialLogSyncInterval         = pPort->initialLogSyncInterval;
  dataSet->currentLogSyncInterval         = pPort->currentLogSyncInterval;
  dataSet->syncReceiptTimeout             = pPort->syncReceiptTimeout;
  dataSet->syncReceiptTimeoutTimeInterval = 0; /* TODO */
  dataSet->initialLogPdelayReqInterval    = pPort->initialLogPdelayReqInterval;
  dataSet->currentLogPdelayReqInterval    = pPort->currentLogPdelayReqInterval;
  dataSet->allowedLostResponses           = pPort->allowedLostResponses;
  dataSet->versionNumber                  = 2;
  dataSet->nup                            = 0;
  dataSet->ndown                          = 0;
  dataSet->acceptableMasterTableEnabled   = 0;

  dataSet->setMask                        = 0; /* Always zero on get */
}

static void SetFromDataSet(struct ptp_device *ptp, uint32_t port, PtpAsPortDataSet *dataSet) {

  struct ptp_port *pPort = &ptp->ports[port];

  if (dataSet->setMask & PTP_SET_PORT_ENABLED) {
    pPort->pttPortEnabled = dataSet->pttPortEnabled;
  }

  if (dataSet->setMask & PTP_SET_NEIGHBOR_PROP_DELAY_THRESH) {
    pPort->neighborPropDelayThresh = dataSet->neighborPropDelayThresh;
  }

  if (dataSet->setMask & PTP_SET_CURRENT_LOG_ANNOUNCE_INTERVAL) {
    pPort->currentLogAnnounceInterval = dataSet->currentLogAnnounceInterval;
  }

  if (dataSet->setMask & PTP_SET_CURRENT_LOG_SYNC_INTERVAL) {
    pPort->currentLogSyncInterval = dataSet->currentLogSyncInterval;
  }

  if (dataSet->setMask & PTP_SET_CURRENT_LOG_PDELAY_REQ_INTERVAL) {
    pPort->currentLogPdelayReqInterval = dataSet->currentLogPdelayReqInterval;
  }

  if (dataSet->setMask & PTP_SET_ALLOWED_LOST_RESPONSES) {
    pPort->allowedLostResponses = dataSet->allowedLostResponses;
  }
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
      int i;
      uint32_t currentDelayMechanism = ptp->properties.delayMechanism;

      /* Copy the userspace argument into the device */
      preempt_disable();
      spin_lock_irqsave(&ptp->mutex, flags);
      copyResult = copy_from_user(&ptp->properties, (void __user*)arg, sizeof(PtpProperties));

      if (ptp->properties.delayMechanism != currentDelayMechanism) {
        /* Having set the properties, load a set of default coefficients in depending
         * upon the selected delay mechanism if it has changed.
         */
        if(ptp->properties.delayMechanism == PTP_DELAY_MECHANISM_P2P) {
          /* Apply coefficients for the peer-to-peer delay mechanism */
          printk("Using P2P coefficients\n");
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
      }

      /* Convert the millisecond values for RTC lock settings into timer ticks.
       * Pre-calculating this avoids several divisions in the real-time code.
       */
      ptp->rtcLockTicks   = (ptp->properties.lockTimeMsec / PTP_TIMER_TICK_MS);
      ptp->rtcUnlockTicks = (ptp->properties.unlockTimeMsec / PTP_TIMER_TICK_MS);

      /* Update the system priority vector to match the new properties */
      ptp->systemPriority.rootSystemIdentity.priority1     = ptp->properties.grandmasterPriority1;
      ptp->systemPriority.rootSystemIdentity.clockClass    = ptp->properties.grandmasterClockQuality.clockClass;
      ptp->systemPriority.rootSystemIdentity.clockAccuracy = ptp->properties.grandmasterClockQuality.clockAccuracy;
      set_offset_scaled_log_variance(ptp->systemPriority.rootSystemIdentity.offsetScaledLogVariance,
                                     ptp->properties.grandmasterClockQuality.offsetScaledLogVariance);
      ptp->systemPriority.rootSystemIdentity.priority2     = ptp->properties.grandmasterPriority2;
      memcpy(ptp->systemPriority.rootSystemIdentity.clockIdentity, ptp->properties.grandmasterIdentity, sizeof(PtpClockIdentity));
      set_steps_removed(ptp->systemPriority.stepsRemoved, 0);
      memcpy(ptp->systemPriority.sourcePortIdentity.clockIdentity, ptp->properties.grandmasterIdentity, sizeof(PtpClockIdentity));
      set_port_number(ptp->systemPriority.sourcePortIdentity.portNumber, 0);
      set_port_number(ptp->systemPriority.portNumber, 0);

      spin_unlock_irqrestore(&ptp->mutex, flags);
      preempt_enable();
      if(copyResult != 0) return(-EFAULT);
      for (i=0; i<ptp->numPorts; i++) {
        ptp->ports[i].reselect = TRUE;
      }
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

  case IOC_PTP_SET_AS_PORT_DATA_SET:
    {
      uint32_t copyResult;
      PtpAsPortDataSet dataSet = {};

      /* Copy the port properties from userspace to get the port number */
      copyResult = copy_from_user(&dataSet, (void __user*)arg, sizeof(PtpAsPortDataSet));
      if(copyResult != 0) return(-EFAULT);

      /* Verify that it is a valid port number */
      if(dataSet.index >= ptp->numPorts) return (-EINVAL);

      SetFromDataSet(ptp, dataSet.index, &dataSet);
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

  case IOC_PTP_CLEAR_AS_PORT_STATISTICS:
    {
      uint32_t copyResult;
      uint32_t portIndex = 0;

      /* Copy the port properties from userspace to get the port number */
      copyResult = copy_from_user(&portIndex, (void __user*)arg, sizeof(portIndex));
      if(copyResult != 0) return(-EFAULT);

      /* Verify that it is a valid port number */
      if(portIndex >= ptp->numPorts) return (-EINVAL);

      /* Clear stats for this port */
      memset(&ptp->ports[portIndex].stats, 0, sizeof(ptp->ports[portIndex].stats));
    }

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

  case IOC_PTP_GET_AS_GRANDMASTER:
    {
      PtpProperties presentMaster;
      memset(&presentMaster, 0, sizeof(PtpProperties));
      presentMaster.grandmasterPriority1                  = ptp->gmPriority->rootSystemIdentity.priority1;
      presentMaster.grandmasterClockQuality.clockClass    = ptp->gmPriority->rootSystemIdentity.clockClass;
      presentMaster.grandmasterClockQuality.clockAccuracy = ptp->gmPriority->rootSystemIdentity.clockAccuracy;
      presentMaster.grandmasterClockQuality.offsetScaledLogVariance =
        get_offset_scaled_log_variance(ptp->gmPriority->rootSystemIdentity.offsetScaledLogVariance);
      presentMaster.grandmasterPriority2    = ptp->gmPriority->rootSystemIdentity.priority2;
      memcpy(presentMaster.grandmasterIdentity, ptp->gmPriority->rootSystemIdentity.clockIdentity, sizeof(PtpClockIdentity));
      if (0 != copy_to_user((void __user*)arg, &presentMaster, sizeof(PtpProperties))) {
        return (-EFAULT);
      }
    }
    break;

  case IOC_PTP_GET_PATH_TRACE:
    {

      uint32_t copyResult;
      PtpPathTrace pathTrace = {};

      /* Copy the port properties from userspace to get the port number */
      copyResult = copy_from_user(&pathTrace, (void __user*)arg, sizeof(PtpPathTrace));
      if(copyResult != 0) return(-EFAULT);

      /* Verify that it is a valid port number */
      if(pathTrace.index >= ptp->numPorts) return (-EINVAL);

      /* Copy the pathTrace into the userspace argument */
      pathTrace.pathTraceLength = ptp->ports[pathTrace.index].pathTraceLength;
      memcpy(&pathTrace.pathTrace,ptp->ports[pathTrace.index].pathTrace,sizeof(PtpClockIdentity)*pathTrace.pathTraceLength);
      copyResult = copy_to_user((void __user*)arg,&pathTrace,sizeof(PtpPathTrace));
      if(copyResult != 0) return(-EFAULT);
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

  /* Set up netlink workers */
  INIT_WORK(&ptp->work_send_gm_change, ptp_work_send_gm_change);
  INIT_WORK(&ptp->work_send_rtc_change, ptp_work_send_rtc_change);
  INIT_WORK(&ptp->work_send_heartbeat, ptp_work_send_heartbeat);
  INIT_WORK(&ptp->work_send_rtc_increment_change, ptp_work_send_rtc_increment_change);

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

  ptp->properties.delayMechanism = DEFAULT_DELAY_MECHANISM;
  if (DEFAULT_DELAY_MECHANISM == PTP_DELAY_MECHANISM_P2P) {
    ptp->coefficients.P = DEFAULT_P2P_COEFF_P;
    ptp->coefficients.I = DEFAULT_P2P_COEFF_I;
    ptp->coefficients.D = DEFAULT_P2P_COEFF_D;
  } else {
    ptp->coefficients.P = DEFAULT_E2E_COEFF_P;
    ptp->coefficients.I = DEFAULT_E2E_COEFF_I;
    ptp->coefficients.D = DEFAULT_E2E_COEFF_D;
  }

  /* Update the system priority vector to match the new properties */
  ptp->systemPriority.rootSystemIdentity.priority1     = ptp->properties.grandmasterPriority1;
  ptp->systemPriority.rootSystemIdentity.clockClass    = ptp->properties.grandmasterClockQuality.clockClass;
  ptp->systemPriority.rootSystemIdentity.clockAccuracy = ptp->properties.grandmasterClockQuality.clockAccuracy;
  set_offset_scaled_log_variance(ptp->systemPriority.rootSystemIdentity.offsetScaledLogVariance,
                                 ptp->properties.grandmasterClockQuality.offsetScaledLogVariance);
  ptp->systemPriority.rootSystemIdentity.priority2     = ptp->properties.grandmasterPriority2;
  memcpy(ptp->systemPriority.rootSystemIdentity.clockIdentity, ptp->properties.grandmasterIdentity, sizeof(PtpClockIdentity));
  set_steps_removed(ptp->systemPriority.stepsRemoved, 0);
  memcpy(ptp->systemPriority.sourcePortIdentity.clockIdentity, ptp->properties.grandmasterIdentity, sizeof(PtpClockIdentity));
  set_port_number(ptp->systemPriority.sourcePortIdentity.portNumber, 0);
  set_port_number(ptp->systemPriority.portNumber, 0);

  ptp->pathTraceLength = 1;
  memcpy(&ptp->pathTrace[0], ptp->systemPriority.rootSystemIdentity.clockIdentity, sizeof(PtpClockIdentity));

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

  ptp->timerTicks = 0;

  if (register_netdevice_notifier(&ptp->notifier) != 0)
  {
    /* TODO: anything to do if we can't register for events? */
  }

#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
  switch_timestamp_init();
#endif
#if 0
  write_avb_ptp(0x0f,0x00,0x00);
  write_avb_ptp(0x0f,0x01,0x00);
  write_avb_ptp(0x0f,0x02,0x00);
  write_avb_policy(0x0,0x00,0x00);
  write_avb_policy(0x1,0x00,0x00);
  write_avb_policy(0x2,0x00,0x00);
  write_avb_policy(0x3,0x00,0x00);
  write_avb_policy(0x4,0x00,0x00);
  write_avb_policy(0x5,0x00,0x00);
  write_avb_policy(0x6,0x00,0x00);
  write_avb_qav(0x00,0x08,0x00);
  write_avb_qav(0x01,0x08,0x00);
  write_avb_qav(0x02,0x08,0x00);
  write_avb_qav(0x03,0x08,0x00);
  write_avb_qav(0x04,0x08,0x00);
  write_avb_qav(0x05,0x08,0x00);
  write_avb_qav(0x06,0x08,0x00);
  write_avb_qav(0x0f,0x0c,0x00);
#endif

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
  unregister_chrdev_region(MKDEV(DRIVER_MAJOR, 0),MAX_INSTANCES);

  /* Unregister Generic Netlink family */
  unregister_ptp_netlink();

  /* Unregister as a platform device driver */
  platform_driver_unregister(&ptp_driver);
}

#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS

#define PTP_ETHER_TYPE 0x88f7
/* CAL_ICS constants; ultimately these and the corresponding code
 * should migrate into board-specific init code.
 */
#define REG_PORT(p)		(0x10 + (p))
#define REG_GLOBAL		0x1b
#define REG_GLOBAL2		0x1c

/* 88E6350R ports assigned to the two CPU ports */
#define CAL_ICS_CPU_PORT_0 (5)
#define CAL_ICS_CPU_PORT_1 (6)

/* Register constant definitions for the 88E6350R LinkStreet switch */
#define PHYS_CTRL_REG  (1)
#  define RGMII_MODE_RXCLK_DELAY   (0x8000)
#  define RGMII_MODE_GTXCLK_DELAY  (0x4000)
#  define FLOW_CTRL_FORCE_DISABLED (0x0040)
#  define FLOW_CTRL_FORCE_ENABLED  (0x00C0)
#  define FORCE_LINK_DOWN          (0x0010)
#  define FORCE_LINK_UP            (0x0030)
#  define FORCE_DUPLEX_HALF        (0x0004)
#  define FORCE_DUPLEX_FULL        (0x000C)
#  define FORCE_SPEED_10           (0x0000)
#  define FORCE_SPEED_100          (0x0001)
#  define FORCE_SPEED_1000         (0x0002)
#  define SPEED_AUTO_DETECT        (0x0003)

/* Register settings assigned to the CPU port:
 * Link forced up, 1 Gbps full-duplex
 * Using RGMII delay on switch IND input data
 * Using RGMII delay on switch OUTD output data
 */

#define CAL_ICS_CPU_PORT_0_PHYS_CTRL (RGMII_MODE_RXCLK_DELAY  | \
                                         /*RGMII_MODE_GTXCLK_DELAY |*/ \
                                         FORCE_LINK_UP           | \
                                         FORCE_DUPLEX_FULL       | \
                                         FORCE_SPEED_1000)



/* Register settings assigned to the SFP port:
 * Link forced up, 1 Gbps full-duplex
 * Using RGMII delay on switch IND input data
 * Using RGMII delay on switch OUTD output data
 */

#define CAL_ICS_CPU_PORT_1_PHYS_CTRL (RGMII_MODE_RXCLK_DELAY  | \
                                         /*RGMII_MODE_GTXCLK_DELAY |*/ \
                                         FORCE_LINK_UP           | \
                                         FORCE_DUPLEX_FULL       | \
                                         FORCE_SPEED_1000)


/* Bit-mask of enabled ports  */
#define CAL_ICS_ENABLED_PORTS ((1 << 6) | (1 << 5) | (1 << 1) | (1 << 0))

/* Number of copper 1000Base-TX ports for CAL_ICS */
#define CAL_ICS_COPPER_PORTS (2)

/* E6350R-related */
#define MV_E6350R_MAX_PORTS_NUM					7

struct ll_fifo_s {
  int isr;  /* Interrupt Status Register 0x0 */
  int ier;  /* Interrupt Enable Register 0x4 */
  int tdfr; /* Transmit data FIFO reset 0x8 */
  int tdfv; /* Transmit data FIFO Vacancy 0xC */
  int tdfd; /* Transmit data FIFO 32bit wide data write port 0x10 */
  int tlf;  /* Write Transmit Length FIFO 0x14 */
  int rdfr; /* Read Receive data FIFO reset 0x18 */
  int rdfo; /* Receive data FIFO Occupancy 0x1C */
  int rdfd; /* Read Receive data FIFO 32bit wide data read port 0x20 */
  int rlf;  /* Read Receive Length FIFO 0x24 */
} ll_fifo_s;

#define MAC_MATCH_NONE 0
#define MAC_MATCH_ALL 1

#define MDIO_CONTROL_REG      (0x00000000)
#  define PHY_MDIO_BUSY       (0x80000000)
#  define PHY_REG_ADDR_MASK   (0x01F)
#  define PHY_ADDR_MASK       (0x01F)
#  define PHY_ADDR_SHIFT      (5)
#  define PHY_MDIO_READ       (0x0400)
#  define PHY_MDIO_WRITE      (0x0000)
#define MDIO_DATA_REG         (0x00000004)
#  define PHY_DATA_MASK       (0x0000FFFF)
#define INT_MASK_REG          (0x00000008)
#  define PHY_IRQ_FALLING     (0x00000000)
#  define PHY_IRQ_RISING      (0x80000000)
#define INT_FLAGS_REG         (0x0000000C)
#  define MDIO_IRQ_MASK       (0x00000001)
#  define PHY_IRQ_MASK        (0x00000002)
#define VLAN_MASK_REG         (0x00000010)
#define MAC_SELECT_REG        (0x00000014)
#define MAC_CONTROL_REG       (0x00000018)
#define   MAC_ADDRESS_LOAD_ACTIVE 0x00000100
#define   MAC_ADDRESS_LOAD_LAST   0x00000200
#define MAC_LOAD_REG          (0x0000001C)
#define REVISION_REG          (0x0000003C)
#  define REVISION_MINOR_MASK  0x0000000F
#  define REVISION_MINOR_SHIFT 0
#  define REVISION_MAJOR_MASK  0x000000F0
#  define REVISION_MAJOR_SHIFT 4
#  define REVISION_MATCH_MASK  0x0000FF00
#  define REVISION_MATCH_SHIFT 8

/* Base address for registers internal to the MAC */
#define LABX_MAC_REGS_BASE    (0x00001000)

/* Base address for the Tx & Rx FIFOs */
#define LABX_FIFO_REGS_BASE   (0x00002000)

#ifdef XILINX_HARD_MAC
#define MAC_RX_CONFIG_REG     (LABX_MAC_REGS_BASE + 0x0040)
#else
#define MAC_RX_CONFIG_REG     (LABX_MAC_REGS_BASE + 0x0004)
#endif
#  define RX_SOFT_RESET          (0x80000000)
#  define RX_JUMBO_FRAME_ENABLE  (0x40000000)
#  define RX_IN_BAND_FCS_ENABLE  (0x20000000)
#  define RX_DISABLE             (0x00000000)
#  define RX_ENABLE              (0x10000000)
#  define RX_VLAN_TAGS_ENABLE    (0x08000000)
#  define RX_HALF_DUPLEX_MODE    (0x04000000)
#  define RX_DISABLE_LTF_CHECK   (0x02000000)
#  define RX_DISABLE_CTRL_CHECK  (0x01000000)

#ifdef XILINX_HARD_MAC
#define MAC_TX_CONFIG_REG     (LABX_MAC_REGS_BASE + 0x0080)
#else
#define MAC_TX_CONFIG_REG     (LABX_MAC_REGS_BASE + 0x0008)
#endif
#  define TX_SOFT_RESET          (0x80000000)
#  define TX_JUMBO_FRAME_ENABLE  (0x40000000)
#  define TX_IN_BAND_FCS_ENABLE  (0x20000000)
#  define TX_DISABLE             (0x00000000)
#  define TX_ENABLE              (0x10000000)
#  define TX_VLAN_TAGS_ENABLE    (0x08000000)
#  define TX_HALF_DUPLEX_MODE    (0x04000000)
#  define TX_IFG_ADJUST_ENABLE   (0x02000000)


/* Masks, etc. for use with the register file */
#define RLF_MASK 0x000007FF

/* Interrupt status register mnemonics */
#define FIFO_ISR_RPURE  0x80000000
#define FIFO_ISR_RPORE  0x40000000
#define FIFO_ISR_RPUE   0x20000000
#  define FIFO_ISR_RX_ERR (FIFO_ISR_RPURE | FIFO_ISR_RPORE | FIFO_ISR_RPUE)
#define FIFO_ISR_TPOE   0x10000000
#define FIFO_ISR_TC     0x08000000
#define FIFO_ISR_RC     0x04000000
#  define FIFO_ISR_ALL     0xFC000000

/* "Magic" value for FIFO reset operations, and timeout, in msec */
#define FIFO_RESET_MAGIC    0x000000A5
#define FIFO_RESET_TIMEOUT  500

volatile static struct ll_fifo_s * ll_fifo = (struct ll_fifo_s *)(ETH_BASEADDR + LABX_FIFO_REGS_BASE);

#define RX_BUFFER_BITS 11
#define RX_MASK ((1<<RX_BUFFER_BITS)-1)



static int eth_send_fifo(const uint8_t * buffer, int length) {
  unsigned int *buf = (unsigned int*) buffer;
  unsigned int len, i, val;

  len = ((length + 3) / 4);

  for (i = 0; i < len; i++) {
    val = *buf++;
    ll_fifo->tdfd = val;
  }

  ll_fifo->tlf = length;

  return length;
}

static uint8_t rx_buffer[1<<RX_BUFFER_BITS];

static int eth_recv_fifo(void) {
  int len, len2, i, val;
  int *buf = (int*) &rx_buffer[0];

  if (ll_fifo->isr & FIFO_ISR_RC) {
    /* One or more packets have been received.  Check the read occupancy register
     * to see how much data is ready to be processed.
     */
    len = ll_fifo->rlf & RLF_MASK;
    len2 = ((len + 3) / 4);

    for (i = 0; i < len2; i++) {
      val = ll_fifo->rdfd;
      *buf++ = val ;
    }

    /* Re-check the occupancy register; if there are still FIFO contents
     * remaining, they are for valid packets.  Not sure if it's okay to call
     * NetReceive() more than once for each invocation, so we'll just leave
     * the ISR flag set instead and let the NetLoop invoke us again.
     */
    if(ll_fifo->rdfo == 0) ll_fifo->isr = FIFO_ISR_RC;

    /* Enqueue the received packet! */
    //    NetReceive (&rx_buffer[0], len);
    // FIXME -- process data here
    /* just return the length for now */
    return len;
  } else if(ll_fifo->isr & FIFO_ISR_RX_ERR) {
    DEBUG_TIMESTAMP_PRINTF("rx error 0x%04x\r\n",ll_fifo->isr);
    /* A receiver error has occurred, reset the Rx logic */
    ll_fifo->isr = FIFO_ISR_ALL;
    ll_fifo->rdfr = FIFO_RESET_MAGIC;
  }
  return 0;
}

#define NUM_SRL16E_CONFIG_WORDS 8
#define NUM_SRL16E_INSTANCES    12

/* Busy loops until the match unit configuration logic is idle.  The hardware goes
 * idle very quickly and deterministically after a configuration word is written,
 * so this should not consume very much time at all.
 */
static void wait_match_config(void) {
  unsigned int addr;
  uint32_t statusWord;
  uint32_t timeout = 10000;

  addr = (ETH_BASEADDR + MAC_CONTROL_REG);
  do {
    statusWord = *((volatile unsigned int *) addr);
    if (timeout-- == 0)
      {
        DEBUG_TIMESTAMP_PRINTF("switch_ethernet : wait_match_config timeout! %08x\r\n",statusWord);
        break;
      }
  } while(statusWord & MAC_ADDRESS_LOAD_ACTIVE);
}

/* Selects a set of match units for subsequent configuration loads */
typedef enum { SELECT_NONE, SELECT_SINGLE, SELECT_ALL } SelectionMode;
static void select_matchers(SelectionMode selectionMode, uint32_t matchUnit) {
  unsigned int addr;

  addr = (ETH_BASEADDR + MAC_SELECT_REG);

  switch(selectionMode) {
  case SELECT_NONE:
    /* De-select all the match units */
    //printk("MAC SELECT %08X\n", 0);
    *((volatile unsigned int *) addr) = 0x00000000;
    break;

  case SELECT_SINGLE:
    /* Select a single unit */
    //printk("MAC SELECT %08X\n", 1 << matchUnit);
    *((volatile unsigned int *) addr) = (1 << matchUnit);
    break;

  default:
    /* Select all match units at once */
    //printk("MAC SELECT %08X\n", 0xFFFFFFFF);
    *((volatile unsigned int *) addr) = 0xFFFFFFFF;
    break;
  }
}

/* Sets the loading mode for any selected match units.  This revolves around
 * automatically disabling the match units' outputs while they're being
 * configured so they don't fire false matches, and re-enabling them as their
 * last configuration word is loaded.
 */
typedef enum { LOADING_MORE_WORDS, LOADING_LAST_WORD } LoadingMode;
static void set_matcher_loading_mode(LoadingMode loadingMode) {
  unsigned int addr;
  uint32_t controlWord;

  addr = (ETH_BASEADDR + MAC_CONTROL_REG);

  controlWord = *((volatile unsigned int *) addr);

  if(loadingMode == LOADING_MORE_WORDS) {
    /* Clear the "last word" bit to suppress false matches while the units are
     * only partially cleared out
     */
    controlWord &= ~MAC_ADDRESS_LOAD_LAST;
  } else {
    /* Loading the final word, flag the match unit(s) to enable after the
     * next configuration word is loaded.
     */
    controlWord |= MAC_ADDRESS_LOAD_LAST;
  }
  //printk("CONTROL WORD %08X\n", controlWord);

  *((volatile unsigned int *) addr) = controlWord;
}

/* Clears any selected match units, preventing them from matching any packets */
static void clear_selected_matchers(void) {
  unsigned int addr;
  uint32_t wordIndex;

  /* Ensure the unit(s) disable as the first word is load to prevent erronous
   * matches as the units become partially-cleared
   */
  set_matcher_loading_mode(LOADING_MORE_WORDS);

  for(wordIndex = 0; wordIndex < NUM_SRL16E_CONFIG_WORDS; wordIndex++) {
    /* Assert the "last word" flag on the last word required to complete the clearing
     * of the selected unit(s).
     */
    if(wordIndex == (NUM_SRL16E_CONFIG_WORDS - 1)) {
      set_matcher_loading_mode(LOADING_LAST_WORD);
    }

    //printk("MAC LOAD %08X\n", 0);
    addr = (ETH_BASEADDR + MAC_LOAD_REG);
    *((volatile unsigned int *) addr) = 0x00000000;
  }
}

/* Loads truth tables into a match unit using the newest, "unified" match
 * architecture.  This is SRL16E based (not cascaded) due to the efficient
 * packing of these primitives into Xilinx LUT6-based architectures.
 */
static void load_unified_matcher(const uint8_t matchMac[6]) {
  unsigned int addr;
  int32_t wordIndex;
  int32_t lutIndex;
  uint32_t configWord = 0x00000000;
  uint32_t matchChunk;

  /* All local writes will be to the MAC filter load register */
  addr = (ETH_BASEADDR + MAC_LOAD_REG);

  /* In this architecture, all of the SRL16Es are loaded in parallel, with each
   * configuration word supplying two bits to each.  Only one of the two bits can
   * ever be set, so there is just an explicit check for one.
   */
  for(wordIndex = (NUM_SRL16E_CONFIG_WORDS - 1); wordIndex >= 0; wordIndex--) {
    for(lutIndex = (NUM_SRL16E_INSTANCES - 1); lutIndex >= 0; lutIndex--) {
      matchChunk = ((matchMac[5-(lutIndex/2)] >> ((lutIndex&1) << 2)) & 0x0F);
      configWord <<= 2;
      if(matchChunk == (wordIndex << 1)) configWord |= 0x01;
      if(matchChunk == ((wordIndex << 1) + 1)) configWord |= 0x02;
    }
    /* 12 nybbles are packed to the MSB */
    configWord <<= 8;

    /* Two bits of truth table have been determined for each SRL16E, load the
     * word and wait for the configuration to occur.  Be sure to flag the last
     * word to automatically re-enable the match unit(s) as the last word completes.
     */
    if(wordIndex == 0) set_matcher_loading_mode(LOADING_LAST_WORD);
    //printk("MAC LOAD %08X\n", configWord);
    *((volatile unsigned int *) addr) = configWord;
    wait_match_config();
  }
}

static void configure_mac_filter(int unitNum, const uint8_t mac[6], int mode) {
  //printk("CONFIGURE MAC MATCH %d (%d), %02X:%02X:%02X:%02X:%02X:%02X\n", unitNum, mode,
  //    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  /* Ascertain that the configuration logic is ready, then select the matcher */
  wait_match_config();
  select_matchers(SELECT_SINGLE, unitNum);

  if (mode == MAC_MATCH_NONE) {
    clear_selected_matchers();
  } else {
    /* Set the loading mode to disable as we load the first word */
    set_matcher_loading_mode(LOADING_MORE_WORDS);

    /* Calculate matching truth tables for the LUTs and load them */
    load_unified_matcher(mac);
  }

  /* De-select the match unit */
  select_matchers(SELECT_NONE, 0);
}

#define REG_PORT(p) (0x10 + (p))
#define REG_GLOBAL 0x1b
#define REG_GLOBAL2 0x1c

#define AVB_COMMAND_REG 0x16
#define AVB_DATA_REG 0x17
#define AVB_COMMAND_WRITE 0x3
#define AVB_COMMAND_READ 0x4
#define AVB_COMMAND_BLOCK_READ 0x6
#define AVB_BLOCK_PTP 0
#define AVB_BLOCK_POLICY 1
#define AVB_BLOCK_QAV 2
#define AVB_COMMAND_BUSY_BIT 15
#define AVB_COMMAND_READ_PTP(PORT,ADDRESS) ((1<<15)|(AVB_COMMAND_READ<<12)|(PORT<<8)|(AVB_BLOCK_PTP<<5)|ADDRESS)
#define AVB_COMMAND_BLOCK_READ_PTP(PORT,ADDRESS) ((1<<15)|(AVB_COMMAND_BLOCK_READ<<12)|(PORT<<8)|(AVB_BLOCK_PTP<<5)|ADDRESS)
#define AVB_COMMAND_WRITE_PTP(PORT,ADDRESS) ((1<<15)|(AVB_COMMAND_WRITE<<12)|(PORT<<8)|(AVB_BLOCK_PTP<<5)|ADDRESS)
#define AVB_COMMAND_WRITE_POLICY(PORT,ADDRESS) ((1<<15)|(AVB_COMMAND_WRITE<<12)|(PORT<<8)|(AVB_BLOCK_POLICY<<5)|ADDRESS)
#define AVB_COMMAND_WRITE_QAV(PORT,ADDRESS) ((1<<15)|(AVB_COMMAND_WRITE<<12)|(PORT<<8)|(AVB_BLOCK_QAV<<5)|ADDRESS)

#define RMU_READ(ADDR,OFFSET) (0x2<<2)|((ADDR>>3)&0x3),((ADDR&7)<<5)|(OFFSET&0x1f),0x00,0x00
#define RMU_WRITE(ADDR,OFFSET,DATA) (0x1<<2)|((ADDR>>3)&0x3),((ADDR&7)<<5)|(OFFSET&0x1f),(DATA>>8)&0xff,((DATA>>0)&0xff)
#define RMU_WAIT_UNTIL_HIGH(ADDR,OFFSET,BIT) (0x1<<4)|(0x3<<2)|((ADDR>>3)&0x3),((ADDR&7)<<5)|(OFFSET&0x1f),BIT&0xf,0x00
#define RMU_WAIT_UNTIL_LOW(ADDR,OFFSET,BIT) (0x1<<4)|(0x0<<2)|((ADDR>>3)&0x3),((ADDR&7)<<5)|(OFFSET&0x1f),BIT&0xf,0x00
#define RMU_END() 0xff,0xff,0xff,0xff

#define RMU_AVB_PTP(PORT,REG) \
        RMU_WRITE(REG_GLOBAL2,AVB_COMMAND_REG,AVB_COMMAND_BLOCK_READ_PTP(PORT,REG)), \
        RMU_WAIT_UNTIL_LOW(REG_GLOBAL2,AVB_COMMAND_REG,AVB_COMMAND_BUSY_BIT), \
        RMU_READ(REG_GLOBAL2,AVB_DATA_REG), \
        RMU_READ(REG_GLOBAL2,AVB_DATA_REG), \
        RMU_READ(REG_GLOBAL2,AVB_DATA_REG), \
        RMU_READ(REG_GLOBAL2,AVB_DATA_REG)

#define RMU_AVB_PTP_CLEAR(PORT,REG) \
        RMU_WRITE(REG_GLOBAL2,AVB_COMMAND_REG,AVB_COMMAND_WRITE_PTP(PORT,(REG+0))), \
        RMU_WAIT_UNTIL_LOW(REG_GLOBAL2,AVB_COMMAND_REG,AVB_COMMAND_BUSY_BIT), \
        RMU_WRITE(REG_GLOBAL2,AVB_DATA_REG,0x00), \
        RMU_WAIT_UNTIL_LOW(REG_GLOBAL2,AVB_COMMAND_REG,AVB_COMMAND_BUSY_BIT), \
        RMU_WRITE(REG_GLOBAL2,AVB_COMMAND_REG,AVB_COMMAND_WRITE_PTP(PORT,(REG+3))), \
        RMU_WAIT_UNTIL_LOW(REG_GLOBAL2,AVB_COMMAND_REG,AVB_COMMAND_BUSY_BIT), \
        RMU_WRITE(REG_GLOBAL2,AVB_DATA_REG,0x00)

static const uint8_t poll_packet[] = {
    0x01, 0x50, 0x43, 0x00, 0x00, 0x00, /* DA */
    0x03, 0x50, 0x43, 0x00, 0x00, 0x00, /* SA */
    0x91, 0x00,
    0x00, /* reserved */
    0x00, /* reserved */
    0x40, /* DSA Tag Format, from CPU, 0b010, Trg_Dev (from Global1 Offset 0x1c */
    0xfa, /* DSA Tag Format, from CPU, 0x3e,0b10 */
    0xcf, /* DSA Tag Format, from CPU, PRI[2:0],0b0,0xf */
    0x00, /* DSA Tag Format, from CPU, sequence number */
    0x00,0x00, /* length/type */
    0x00,0x01, /* Request Format: SOHO */
    0x00,0x00, /* Pad */
    0x20,0x00, /* Request Code, register read or write */
    RMU_AVB_PTP(0,0x08), /* AVB2 incomming sync t1 */
    RMU_AVB_PTP(0,0x0c), /* AVB2 incomming delay t1 */
    RMU_AVB_PTP(0,0x10), /* AVB2 outgoing t2 */
    RMU_AVB_PTP(5,0x08), /* AVB2 outgoing sync t1 */
    RMU_AVB_PTP(5,0x0c), /* AVB2 outgoing delay t1 */
    RMU_AVB_PTP(5,0x10), /* AVB2 incomming t2 */
    RMU_AVB_PTP(1,0x08), /* AVB1 incomming sync t1 */
    RMU_AVB_PTP(1,0x0c), /* AVB1 incomming delay t1 */
    RMU_AVB_PTP(1,0x10), /* AVB1 outgoing t2 */
    RMU_AVB_PTP(6,0x08), /* AVB1 outoing sync t1 */
    RMU_AVB_PTP(6,0x0c), /* AVB1 outgoing delay t1 */
    RMU_AVB_PTP(6,0x10), /* AVB1 incomming t2 */
    RMU_END(),
};

#define RESPONSE_OFFSET (14+14)
#define BYTES_PER_TIMESTAMP (6*4)
#define BYTES_PER_SIDE (3*BYTES_PER_TIMESTAMP)
#define BYTES_PER_PORT (6*BYTES_PER_TIMESTAMP)
#define INGRESS1 0
#define INGRESS2 1
#define EGRESS 2
#define CPU 1
#define PHY 0
#define AVB2_PORT 0
#define AVB1_PORT 1

#define SWITCH_BUF_INDEX(PORT,SIDE,REG) (RESPONSE_OFFSET+(PORT*BYTES_PER_PORT)+(SIDE*BYTES_PER_SIDE)+(REG*BYTES_PER_TIMESTAMP))
#define SWITCH_BUF_INDEX_FLAGS(PORT,SIDE,REG) (SWITCH_BUF_INDEX(PORT,SIDE,REG)+(2*4)+2)
#define SWITCH_BUF_INDEX_LOW(PORT,SIDE,REG) (SWITCH_BUF_INDEX(PORT,SIDE,REG)+(3*4)+2)
#define SWITCH_BUF_INDEX_HIGH(PORT,SIDE,REG) (SWITCH_BUF_INDEX(PORT,SIDE,REG)+(4*4)+2)
#define SWITCH_BUF_INDEX_SEQUENCE_ID(PORT,SIDE,REG) (SWITCH_BUF_INDEX(PORT,SIDE,REG)+(5*4)+2)

#define SWITCH_FRAME_REG_OVERFLOW(BUF,PORT,SIDE,REG) \
    (*((uint16_t *)&BUF[SWITCH_BUF_INDEX_FLAGS(PORT,SIDE,REG)])&2)

#define SWITCH_FRAME_REG_VALID(BUF,PORT,SIDE,REG) \
    (*((uint16_t *)&BUF[SWITCH_BUF_INDEX_FLAGS(PORT,SIDE,REG)])&1)

#define SWITCH_FRAME_REG(BUF,DEST,PORT,SIDE,REG) \
    DEST.low=*((uint16_t *)&BUF[SWITCH_BUF_INDEX_LOW(PORT,SIDE,REG)]); \
    DEST.high=*((uint16_t *)&BUF[SWITCH_BUF_INDEX_HIGH(PORT,SIDE,REG)]); \
    DEST.sequence_id=*((uint16_t *)&BUF[SWITCH_BUF_INDEX_SEQUENCE_ID(PORT,SIDE,REG)])

static int timestamp_buffer_index;
#define TIMESTAMPS_BITS 6
#define TIMESTAMPS_SIZE (1<<TIMESTAMPS_BITS)
#define TIMESTAMPS_INDEX_MASK (TIMESTAMPS_SIZE-1)
static switch_timestamp_t timestamp_buffer[TIMESTAMPS_SIZE];

static void dump_timestamp(switch_timestamp_t * timestamp) {
    switch(timestamp->type) {
        case TIMESTAMP_AVB2_OUTGOING_SYNC_T1:
            DEBUG_TIMESTAMP_PRINTF("%04x %04x%04x TIMESTAMP_AVB2_OUTGOING_SYNC_T1\r\n",(unsigned int)timestamp->sequence_id,(unsigned int)timestamp->high,(unsigned int)timestamp->low);
            break;
        case TIMESTAMP_AVB2_OUTGOING_DELAY_T1:
            DEBUG_TIMESTAMP_PRINTF("%04x %04x%04x TIMESTAMP_AVB2_OUTGOING_DELAY_T1\r\n",(unsigned int)timestamp->sequence_id,(unsigned int)timestamp->high,(unsigned int)timestamp->low);
            break;
        case TIMESTAMP_AVB2_OUTGOING_T2:
            DEBUG_TIMESTAMP_PRINTF("%04x %04x%04x \t\t\tTIMESTAMP_AVB2_OUTGOING_T2\r\n",(unsigned int)timestamp->sequence_id,(unsigned int)timestamp->high,(unsigned int)timestamp->low);
            break;
        case TIMESTAMP_AVB2_INCOMMING_SYNC_T1:
            DEBUG_TIMESTAMP_PRINTF("%04x %04x%04x TIMESTAMP_AVB2_INCOMMING_SYNC_T1\r\n",(unsigned int)timestamp->sequence_id,(unsigned int)timestamp->high,(unsigned int)timestamp->low);
            break;
        case TIMESTAMP_AVB2_INCOMMING_DELAY_T1:
            DEBUG_TIMESTAMP_PRINTF("%04x %04x%04x TIMESTAMP_AVB2_INCOMMING_DELAY_T1\r\n",(unsigned int)timestamp->sequence_id,(unsigned int)timestamp->high,(unsigned int)timestamp->low);
            break;
        case TIMESTAMP_AVB2_INCOMMING_T2:
            DEBUG_TIMESTAMP_PRINTF("%04x %04x%04x \t\t\tTIMESTAMP_AVB2_INCOMMING_T2\r\n",(unsigned int)timestamp->sequence_id,(unsigned int)timestamp->high,(unsigned int)timestamp->low);
            break;
        case TIMESTAMP_AVB1_OUTGOING_SYNC_T1:
            DEBUG_TIMESTAMP_PRINTF("%04x %04x%04x TIMESTAMP_AVB1_OUTGOING_SYNC_T1\r\n",(unsigned int)timestamp->sequence_id,(unsigned int)timestamp->high,(unsigned int)timestamp->low);
            break;
        case TIMESTAMP_AVB1_OUTGOING_DELAY_T1:
            DEBUG_TIMESTAMP_PRINTF("%04x %04x%04x TIMESTAMP_AVB1_OUTGOING_DELAY_T1\r\n",(unsigned int)timestamp->sequence_id,(unsigned int)timestamp->high,(unsigned int)timestamp->low);
            break;
        case TIMESTAMP_AVB1_OUTGOING_T2:
            DEBUG_TIMESTAMP_PRINTF("%04x %04x%04x \t\t\tTIMESTAMP_AVB1_OUTGOING_T2\r\n",(unsigned int)timestamp->sequence_id,(unsigned int)timestamp->high,(unsigned int)timestamp->low);
            break;
        case TIMESTAMP_AVB1_INCOMMING_SYNC_T1:
            DEBUG_TIMESTAMP_PRINTF("%04x %04x%04x TIMESTAMP_AVB1_INCOMMING_SYNC_T1\r\n",(unsigned int)timestamp->sequence_id,(unsigned int)timestamp->high,(unsigned int)timestamp->low);
            break;
        case TIMESTAMP_AVB1_INCOMMING_DELAY_T1:
            DEBUG_TIMESTAMP_PRINTF("%04x %04x%04x TIMESTAMP_AVB1_INCOMMING_DELAY_T1\r\n",(unsigned int)timestamp->sequence_id,(unsigned int)timestamp->high,(unsigned int)timestamp->low);
            break;
        case TIMESTAMP_AVB1_INCOMMING_T2:
            DEBUG_TIMESTAMP_PRINTF("%04x %04x%04x \t\t\tTIMESTAMP_AVB1_INCOMMING_T2\r\n",(unsigned int)timestamp->sequence_id,(unsigned int)timestamp->high,(unsigned int)timestamp->low);
            break;
    }
}

static void switch_frame_rx(uint8_t * data,int len) {
    int i;
    switch_timestamp_t timestamps[12];
    static switch_timestamp_t prev_timestamps[12];
    int timestamps_index;

    timestamps_index=0;

    /* process T1s first */
    SWITCH_FRAME_REG(data,timestamps[timestamps_index],AVB2_PORT,CPU,INGRESS2);
    timestamps[timestamps_index++].type=TIMESTAMP_AVB2_OUTGOING_SYNC_T1;

    SWITCH_FRAME_REG(data,timestamps[timestamps_index],AVB2_PORT,CPU,INGRESS1);
    timestamps[timestamps_index++].type=TIMESTAMP_AVB2_OUTGOING_DELAY_T1;

    SWITCH_FRAME_REG(data,timestamps[timestamps_index],AVB2_PORT,PHY,INGRESS2);
    timestamps[timestamps_index++].type=TIMESTAMP_AVB2_INCOMMING_SYNC_T1;

    SWITCH_FRAME_REG(data,timestamps[timestamps_index],AVB2_PORT,PHY,INGRESS1);
    timestamps[timestamps_index++].type=TIMESTAMP_AVB2_INCOMMING_DELAY_T1;

    SWITCH_FRAME_REG(data,timestamps[timestamps_index],AVB1_PORT,CPU,INGRESS2);
    timestamps[timestamps_index++].type=TIMESTAMP_AVB1_OUTGOING_SYNC_T1;

    SWITCH_FRAME_REG(data,timestamps[timestamps_index],AVB1_PORT,CPU,INGRESS1);
    timestamps[timestamps_index++].type=TIMESTAMP_AVB1_OUTGOING_DELAY_T1;

    SWITCH_FRAME_REG(data,timestamps[timestamps_index],AVB1_PORT,PHY,INGRESS2);
    timestamps[timestamps_index++].type=TIMESTAMP_AVB1_INCOMMING_SYNC_T1;

    SWITCH_FRAME_REG(data,timestamps[timestamps_index],AVB1_PORT,PHY,INGRESS1);
    timestamps[timestamps_index++].type=TIMESTAMP_AVB1_INCOMMING_DELAY_T1;

    SWITCH_FRAME_REG(data,timestamps[timestamps_index],AVB2_PORT,PHY,EGRESS);
    timestamps[timestamps_index++].type=TIMESTAMP_AVB2_OUTGOING_T2;

    SWITCH_FRAME_REG(data,timestamps[timestamps_index],AVB2_PORT,CPU,EGRESS);
    timestamps[timestamps_index++].type=TIMESTAMP_AVB2_INCOMMING_T2;

    SWITCH_FRAME_REG(data,timestamps[timestamps_index],AVB1_PORT,PHY,EGRESS);
    timestamps[timestamps_index++].type=TIMESTAMP_AVB1_OUTGOING_T2;

    SWITCH_FRAME_REG(data,timestamps[timestamps_index],AVB1_PORT,CPU,EGRESS);
    timestamps[timestamps_index++].type=TIMESTAMP_AVB1_INCOMMING_T2;

    for(i=0;i<(sizeof(timestamps)/sizeof(timestamps[0]));i++) {
        if(memcmp(&timestamps[i],&prev_timestamps[i],sizeof(switch_timestamp_t))!=0) {
            timestamp_buffer[timestamp_buffer_index++&TIMESTAMPS_INDEX_MASK]=timestamps[i];
            memcpy(&prev_timestamps[i],&timestamps[i],sizeof(switch_timestamp_t));
            //dump_timestamp(&timestamps[i]);
        }
    }
}

static void process_timestamps(void) {
    int n;
    while((ll_fifo->isr&FIFO_ISR_RC)==FIFO_ISR_RC) {
        n=eth_recv_fifo();
        if(n>0) {
            switch_frame_rx(&rx_buffer[0],n);
        }
    }
}

void switch_timestamp_poll(void) {
    eth_send_fifo(poll_packet,sizeof(poll_packet));
}

void switch_timestamp_wait(void) {
    int timeout;
    process_timestamps();
    eth_send_fifo(poll_packet,sizeof(poll_packet));
    timeout=100;
    while((ll_fifo->isr&FIFO_ISR_RC)!=FIFO_ISR_RC) {
        udelay(1);
        timeout--;
        if(timeout--<=0) {
            ERROR_TIMESTAMP_PRINTF("timed out waiting for switch timestamps\r\n");
            break;
        }
    }
}

void switch_timestamp(enum TimeStampPair type,switch_timestamp_t ** t1, switch_timestamp_t ** t2,uint16_t sequence_id) {
    int i;
    int t1_type;
    int t2_type;
    process_timestamps();
    *t1=NULL;
    *t2=NULL;
    switch(type) {
        case TIMESTAMP_AVB2_INCOMMING_SYNC:
                t1_type=TIMESTAMP_AVB2_INCOMMING_SYNC_T1;
                t2_type=TIMESTAMP_AVB2_INCOMMING_T2;
                break;
        case TIMESTAMP_AVB2_INCOMMING_REQUEST:
        case TIMESTAMP_AVB2_INCOMMING_RESPONSE:
                t1_type=TIMESTAMP_AVB2_INCOMMING_DELAY_T1;
                t2_type=TIMESTAMP_AVB2_INCOMMING_T2;
                break;
        case TIMESTAMP_AVB2_OUTGOING_SYNC:
                t1_type=TIMESTAMP_AVB2_OUTGOING_SYNC_T1;
                t2_type=TIMESTAMP_AVB2_OUTGOING_T2;
                break;
        case TIMESTAMP_AVB2_OUTGOING_RESPONSE:
        case TIMESTAMP_AVB2_OUTGOING_REQUEST:
                t1_type=TIMESTAMP_AVB2_OUTGOING_DELAY_T1;
                t2_type=TIMESTAMP_AVB2_OUTGOING_T2;
                break;
        case TIMESTAMP_AVB1_INCOMMING_SYNC:
                t1_type=TIMESTAMP_AVB1_INCOMMING_SYNC_T1;
                t2_type=TIMESTAMP_AVB1_INCOMMING_T2;
                break;
        case TIMESTAMP_AVB1_INCOMMING_REQUEST:
        case TIMESTAMP_AVB1_INCOMMING_RESPONSE:
                t1_type=TIMESTAMP_AVB1_INCOMMING_DELAY_T1;
                t2_type=TIMESTAMP_AVB1_INCOMMING_T2;
                break;
        case TIMESTAMP_AVB1_OUTGOING_SYNC:
                t1_type=TIMESTAMP_AVB1_OUTGOING_SYNC_T1;
                t2_type=TIMESTAMP_AVB1_OUTGOING_T2;
                break;
        case TIMESTAMP_AVB1_OUTGOING_REQUEST:
        case TIMESTAMP_AVB1_OUTGOING_RESPONSE:
                t1_type=TIMESTAMP_AVB1_OUTGOING_DELAY_T1;
                t2_type=TIMESTAMP_AVB1_OUTGOING_T2;
                break;
        default:
                return;
    }
    for(i=((timestamp_buffer_index-1)&TIMESTAMPS_INDEX_MASK);i!=(timestamp_buffer_index&TIMESTAMPS_INDEX_MASK);i=((i-1)&TIMESTAMPS_INDEX_MASK)) {
        if(timestamp_buffer[i].type==t1_type) {
            if(*t1==NULL) {
               if(timestamp_buffer[i].sequence_id==sequence_id) {
                    *t1=&timestamp_buffer[i];
                }
            }
        } else if(timestamp_buffer[i].type==t2_type) {
            if(*t2==NULL) {
               if(timestamp_buffer[i].sequence_id==sequence_id) {
                 *t2=&timestamp_buffer[i];
               }
            }
        }
    }
}

void switch_timestamp_init(void) {
  static const uint8_t macaddr[] = { 0x03, 0x50, 0x43, 0x00, 0x00, 0x00 };

  write_avb_ptp(0xf,0,PTP_ETHER_TYPE);
  write_avb_ptp(0xf,1,(1<<3)|(1<<2)|(1<<0)); /* timestamp packets messageIDs 1,2 and 3 */
  write_avb_ptp(0xf,2,(0<<3)|(0<<2)|(1<<0)); /* put messageId 2 in arrival 1, messageId 3 in arrival 2 */
  write_avb_ptp(0xe,1,8*1000); /* the period for a 125Mhz clock is 8000ps */

  /* Clear ISR flags and reset both transmit and receive FIFO logic */
  ll_fifo->isr = FIFO_ISR_ALL;
  ll_fifo->tdfr = FIFO_RESET_MAGIC;
  ll_fifo->rdfr = FIFO_RESET_MAGIC;
  ll_fifo->isr = FIFO_ISR_ALL;
  ll_fifo->tdfr = FIFO_RESET_MAGIC;
  ll_fifo->rdfr = FIFO_RESET_MAGIC;
  configure_mac_filter(0, macaddr, MAC_MATCH_ALL);
}

#endif /* CONFIG_LABX_PTP_MARVELL_TIMESTAMPS */

module_init(ptp_driver_init);
module_exit(ptp_driver_exit);

MODULE_AUTHOR("Eldridge M. Mount IV <eldridge.mount@labxtechnologies.com>");
MODULE_DESCRIPTION("Lab X Technologies PTP 2.0 driver");
MODULE_LICENSE("GPL");

