/*
 *  linux/drivers/char/labx_audio_meters.c
 *
 *  Lab X Technologies Audio Metering
 *
 *  Written by Chris Wulff (chris.wulff@labxtechnologies.com)
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

#include <linux/autoconf.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/labx_audio_meters_defs.h>
#include <linux/platform_device.h>
#include <xio.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/io.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF


#define DRIVER_NAME "labx_audio_meters"

#define MAX_AM_DEVICES 16
#define NAME_MAX_SIZE 256

#define AUDIO_METERS_VALUE_REG       0x0
#define AUDIO_METERS_COEFFICIENT_REG 0x1
#define AUDIO_METERS_STATUS_REG      0x2
#define   AUDIO_METERS_STATUS_READY    0x00000001
#define AUDIO_METERS_INT_MASK_REG    0x3
#define AUDIO_METERS_INT_FLAG_REG    0x4
#define AUDIO_METERS_MODULO_REG      0x5

#define AUDIO_METERS_REGISTER(am, reg)                             \
  ((uintptr_t)(am)->virtualAddress | ((reg) * 4))

/* Audio Meters Platform device structure */
struct labx_audio_meters_pdev {
  /* Misc device */
  struct miscdevice miscdev;

  /* Pointer back to the platform device */
  struct platform_device *pdev;

  char     name[NAME_MAX_SIZE];

  /* Physical and virtual base address */
  uint32_t physicalAddress;
  uint32_t addressRangeSize;

  /* Virtual address pointer for the memory-mapped hardware */
  void __iomem *virtualAddress;

  /* Number of audio channels supported in this hardware peripheral */
  uint32_t numChannels;
};

static uint32_t instanceCount;
static struct labx_audio_meters_pdev* devices[MAX_AM_DEVICES] = {};

static int labx_audio_meters_open(struct inode *inode, struct file *filp) {
  int deviceIndex;
  struct labx_audio_meters_pdev *audio_meters_pdev;

  /* Search for the device amongst those which successfully were probed */
  for(deviceIndex = 0; deviceIndex < MAX_AM_DEVICES; deviceIndex++) {
    if ((devices[deviceIndex] != NULL) && (devices[deviceIndex]->miscdev.minor == iminor(inode))) {
      /* Retain the device pointer within the file pointer for future navigation */
      filp->private_data = devices[deviceIndex];
      break;
    }
  }

  /* Ensure the device was actually found */
  if(deviceIndex >= MAX_AM_DEVICES) {
    printk(KERN_WARNING DRIVER_NAME ": Could not find device for node (%d, %d)\n",
           imajor(inode), iminor(inode));
    return(-ENODEV);
  }

  audio_meters_pdev = (struct labx_audio_meters_pdev*)filp->private_data;

  return(0);
}

static int labx_audio_meters_release(struct inode *inode, struct file *filp) {
  struct labx_audio_meters_pdev *audio_meters_pdev = (struct labx_audio_meters_pdev*)filp->private_data;

  return(0);
}

static int labx_audio_meters_ioctl_cdev(struct inode *inode, struct file *filp,
                                       unsigned int command, unsigned long arg)
{
  struct labx_audio_meters_pdev *audio_meters_pdev = (struct labx_audio_meters_pdev*)filp->private_data;

  switch(command) {
  case IOC_AM_SET_COEFFICIENT:
    XIo_Out32(AUDIO_METERS_REGISTER(audio_meters_pdev, AUDIO_METERS_COEFFICIENT_REG), (u32)arg);
    break;

  case IOC_AM_GET_METER_COUNT:
    if (copy_to_user((void __user*)arg, &audio_meters_pdev->numChannels, sizeof(u32)) != 0) {
      return (-EFAULT);
    }
    break;

  case IOC_AM_READ_METERS:
    {
      u32 timeout = 1000000;
      u32 meterDataCount = 0;
      u32 meterDataValue = 0;

      /* Copy the number of meter values that the buffer can hold */
      if(copy_from_user(&meterDataCount, (void __user*)arg, sizeof(u32)) != 0) {
        return(-EFAULT);
      }
      arg += sizeof(u32);

      /* Make sure that no more than the supported channels are being read */
      if (meterDataCount > audio_meters_pdev->numChannels) {
        return(-EINVAL);
      }

      /* Switch meter data banks to snapshot the values */
      XIo_Out32(AUDIO_METERS_REGISTER(audio_meters_pdev, AUDIO_METERS_VALUE_REG), 0);

      /* Wait for meter data to be ready. TODO: use the interrupt instead of this spin-loop */
      while ((XIo_In32(AUDIO_METERS_REGISTER(audio_meters_pdev, AUDIO_METERS_STATUS_REG)) & AUDIO_METERS_STATUS_READY) == 0) {
        if (timeout-- == 0) {
          printk("Meter read timeout\n");
          break;
        }
      }

      while(meterDataCount-- > 0) {
        meterDataValue = XIo_In32(AUDIO_METERS_REGISTER(audio_meters_pdev, AUDIO_METERS_VALUE_REG));

        if(copy_to_user((void __user*)arg, &meterDataValue, sizeof(u32)) != 0) {
          return(-EFAULT);
        }
        arg += sizeof(u32);
      }
    }
    break;

  default:
    /* We don't recognize this command */
    return(-EINVAL);
  }

  /* Command handled without incident, return success */
  return(0);
}

static const struct file_operations labx_audio_meters_fops = {
  .open    = labx_audio_meters_open,
  .release = labx_audio_meters_release,
  .ioctl   = labx_audio_meters_ioctl_cdev,
};

/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 * This is exported to allow polymorphic drivers to invoke it.
 * @param name - Name of the instance
 * @param pdev - Platform device structure
 * @param addressRange  - Resource describing the hardware's I/O range
 * @param numChannels   - Number of channels of output
 */
int labx_audio_meters_probe(const char *name, 
                           struct platform_device *pdev,
                           struct resource *addressRange,
                           u32 numChannels) {
  struct labx_audio_meters_pdev *audio_meters_pdev;
  uint32_t deviceIndex;
  int32_t ret;

  /* Create and populate a device structure */
  audio_meters_pdev = (struct labx_audio_meters_pdev*) kzalloc(sizeof(struct labx_audio_meters_pdev), GFP_KERNEL);
  if(!audio_meters_pdev) return(-ENOMEM);

  /* Request and map the device's I/O memory region into uncacheable space */
  audio_meters_pdev->physicalAddress = addressRange->start;
  audio_meters_pdev->addressRangeSize = ((addressRange->end - addressRange->start) + 1);

  snprintf(audio_meters_pdev->name, NAME_MAX_SIZE, "%s%d", name, instanceCount++);
  audio_meters_pdev->name[NAME_MAX_SIZE - 1] = '\0';
  if(request_mem_region(audio_meters_pdev->physicalAddress, audio_meters_pdev->addressRangeSize,
                        audio_meters_pdev->name) == NULL) {
    ret = -ENOMEM;
    goto free;
  }

  audio_meters_pdev->virtualAddress = 
    (void*) ioremap_nocache(audio_meters_pdev->physicalAddress, audio_meters_pdev->addressRangeSize);
  if(!audio_meters_pdev->virtualAddress) {
    ret = -ENOMEM;
    goto release;
  }
  printk("AM virtualAddress = 0x%08X, phys = 0x%08X, size = 0x%08X\n", 
         (uint32_t) audio_meters_pdev->virtualAddress,
         (uint32_t) audio_meters_pdev->physicalAddress,
         audio_meters_pdev->addressRangeSize);

  audio_meters_pdev->numChannels = numChannels;
  printk(" Audio Meters interface found at 0x%08X: %d channels\n", 
         (uint32_t) audio_meters_pdev->physicalAddress,
         audio_meters_pdev->numChannels);
  audio_meters_pdev->miscdev.minor = MISC_DYNAMIC_MINOR;
  audio_meters_pdev->miscdev.name = audio_meters_pdev->name;
  audio_meters_pdev->miscdev.fops = &labx_audio_meters_fops;
  ret = misc_register(&audio_meters_pdev->miscdev);
  if (ret) {
    printk(KERN_WARNING DRIVER_NAME ": Unable to register misc device.\n");
    goto unmap;
  }
  platform_set_drvdata(pdev, audio_meters_pdev);
  audio_meters_pdev->pdev = pdev;
  dev_set_drvdata(audio_meters_pdev->miscdev.this_device, audio_meters_pdev);

  /* Locate and occupy the first available device index for future navigation in
   * the call to labx_audio_meters_open()
   */
  for (deviceIndex = 0; deviceIndex < MAX_AM_DEVICES; deviceIndex++) {
    if (NULL == devices[deviceIndex]) {
      devices[deviceIndex] = audio_meters_pdev;
      break;
    }
  }

  /* Ensure that we haven't been asked to probe for too many devices */
  if(deviceIndex >= MAX_AM_DEVICES) {
    printk(KERN_WARNING DRIVER_NAME ": Maximum device count (%d) exceeded during probe\n",
           MAX_AM_DEVICES);
    goto unmap;
  }

  /* Return success */
  return(0);

 unmap:
  iounmap(audio_meters_pdev->virtualAddress);
 release:
  release_mem_region(audio_meters_pdev->physicalAddress, 
                     audio_meters_pdev->addressRangeSize);
 free:
  kfree(audio_meters_pdev);
  return(ret);
}

/* OpenFirmware support section */
#ifdef CONFIG_OF
static u32 get_u32(struct of_device *ofdev, const char *s) {
  u32 *p = (u32 *)of_get_property(ofdev->node, s, NULL);
  if(p) {
    return *p;
  } else {
    dev_warn(&ofdev->dev, "Parameter %s not found, defaulting to false.\n", s);
    return FALSE;
  }
}

static int labx_audio_meters_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
  struct resource r_mem_struct;
  struct resource *addressRange = &r_mem_struct;
  struct platform_device *pdev = to_platform_device(&ofdev->dev);
  u32 numChannels;
  int ret;
  u32 has_serializer;
  
  /* Obtain the resources for this instance; use the device tree node name */
  const char *name = ofdev->node->name;
  ret = of_address_to_resource(ofdev->node, 0, addressRange);
  if (ret) {
    dev_warn(&ofdev->dev, "Invalid address\n");
    return(ret);
  }

  /* Look up the number of channels in the device tree */
  numChannels = (1 << get_u32(ofdev, "xlnx,channel-addr-width"));

  /* Dispatch to the generic function */
  return(labx_audio_meters_probe(name, pdev, addressRange, numChannels));
}

static int __devexit labx_audio_meters_pdev_remove(struct platform_device *pdev);

static int __devexit labx_audio_meters_of_remove(struct of_device *dev)
{
  struct platform_device *pdev = to_platform_device(&dev->dev);
  labx_audio_meters_pdev_remove(pdev);
  return(0);
}

static struct of_device_id labx_audio_meters_of_match[] = {
  { .compatible = "xlnx,cal-audio-meters-1.00.a", },
  { /* end of list */ },
};

static struct of_platform_driver labx_audio_meters_of_driver = {
  .name        = DRIVER_NAME,
  .match_table = labx_audio_meters_of_match,
  .probe       = labx_audio_meters_of_probe,
  .remove      = __devexit_p(labx_audio_meters_of_remove),
};

#endif /* CONFIG_OF */

static int labx_audio_meters_pdev_probe(struct platform_device *pdev)
{
  struct resource *addressRange;

  /* Obtain the resources for this instance */
  addressRange = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!addressRange) return(-ENXIO);

  /* Dispatch to the generic function */
  return(labx_audio_meters_probe(pdev->name, pdev, addressRange, 16));
}

/* This is exported to allow polymorphic drivers to invoke it */
int labx_audio_meters_remove(struct labx_audio_meters_pdev *audio_meters_pdev) {
  int deviceIndex;

  misc_deregister(&audio_meters_pdev->miscdev);

  for (deviceIndex = 0; deviceIndex < MAX_AM_DEVICES; deviceIndex++) {
    if (audio_meters_pdev == devices[deviceIndex]) {
      devices[deviceIndex] = NULL;
      break;
    }
  }
  return(0);
}

static __devexit int labx_audio_meters_pdev_remove(struct platform_device *pdev)
{
  struct labx_audio_meters_pdev *audio_meters_pdev = (struct labx_audio_meters_pdev*)platform_get_drvdata(pdev);
  return(labx_audio_meters_remove(audio_meters_pdev));
}

/* Platform device driver structure */
static struct platform_driver labx_audio_meters_platform_driver = {
  .probe  = labx_audio_meters_pdev_probe,
  .remove = labx_audio_meters_pdev_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __init labx_audio_meters_driver_init(void)
{
  int returnValue;

  printk(KERN_INFO DRIVER_NAME ": Audio Meters Driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright (c) Lab X Technologies, LLC\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&labx_audio_meters_of_driver);
#endif
 
  /* Initialize the instance counter */
  instanceCount = 0;

  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&labx_audio_meters_platform_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }

  return(0);
}

static void __exit labx_audio_meters_driver_exit(void)
{
  /* Unregister as a platform device driver */
  platform_driver_unregister(&labx_audio_meters_platform_driver);
}

module_init(labx_audio_meters_driver_init);
module_exit(labx_audio_meters_driver_exit);

MODULE_AUTHOR("Chris Wulff");
MODULE_LICENSE("GPL");
