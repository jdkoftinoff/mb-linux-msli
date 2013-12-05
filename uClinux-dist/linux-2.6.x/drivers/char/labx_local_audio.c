/*
 *  linux/drivers/char/labx_local_audio.c
 *
 *  Lab X Technologies Local Audio
 *
 *  Written by Chris Wulff (chris.wulff@labxtechnologies.com)
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
#include <linux/labx_dma_coprocessor_defs.h>
#include <linux/labx_local_audio.h>
#include <linux/labx_local_audio_defs.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <xio.h>
#include <linux/slab.h>
#include <linux/mm.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF


#define DRIVER_NAME "labx_local_audio"

#define MAX_LA_DEVICES 16
static uint32_t instanceCount;
static struct labx_local_audio_pdev* devices[MAX_LA_DEVICES] = {};
static int open_count=0;

static int labx_local_audio_open(struct inode *inode, struct file *filp) {
  int deviceIndex;
  struct labx_local_audio_pdev *local_audio_pdev;

  /* Search for the device amongst those which successfully were probed */
  for(deviceIndex = 0; deviceIndex < MAX_LA_DEVICES; deviceIndex++) {
    if ((devices[deviceIndex] != NULL) && (devices[deviceIndex]->miscdev.minor == iminor(inode))) {
      /* Retain the device pointer within the file pointer for future navigation */
      filp->private_data = devices[deviceIndex];
      break;
    }
  }

  /* Ensure the device was actually found */
  if(deviceIndex >= MAX_LA_DEVICES) {
    printk(KERN_WARNING DRIVER_NAME ": Could not find device for node (%d, %d)\n",
           imajor(inode), iminor(inode));
    return(-ENODEV);
  }

  /* TODO - Ensure the local audio hardware is reset */
  local_audio_pdev = (struct labx_local_audio_pdev*)filp->private_data;

  if(open_count==0) {
    /* Also open the DMA instance, if there is one */
    if(local_audio_pdev->dma != NULL) labx_dma_open(local_audio_pdev->dma);
  }
  open_count++;

  /* Invoke the open() operation on the derived driver, if there is one */
  if((local_audio_pdev->derivedFops != NULL) && 
     (local_audio_pdev->derivedFops->open != NULL)) {
    local_audio_pdev->derivedFops->open(inode, filp);
  }

  return(0);
}

static int labx_local_audio_release(struct inode *inode, struct file *filp) {
  struct labx_local_audio_pdev *local_audio_pdev = (struct labx_local_audio_pdev*)filp->private_data;

  /* Invoke the release() operation on the derived driver, if there is one */
  if((local_audio_pdev->derivedFops != NULL) && 
     (local_audio_pdev->derivedFops->release != NULL)) {
    local_audio_pdev->derivedFops->release(inode, filp);
  }

  open_count--;
  if(open_count==0) {
    /* Also release the DMA instance, if there is one */
    if(local_audio_pdev->dma != NULL) labx_dma_release(local_audio_pdev->dma);
  }

  return(0);
}

static int labx_local_audio_ioctl_cdev(struct inode *inode, struct file *filp,
                                       unsigned int command, unsigned long arg)
{
  struct labx_local_audio_pdev *local_audio_pdev = (struct labx_local_audio_pdev*)filp->private_data;
  int returnValue = 0;

  switch(command) {
  case IOC_LA_SET_CHANNEL_MAPPING:
    {
      struct LocalAudioChannelMapping mapping;
      if(copy_from_user(&mapping, (void __user*)arg, sizeof(mapping)) != 0) {
        printk("CFU failed\n");
        return(-EFAULT);
      }
			
      if (mapping.channel >= local_audio_pdev->numChannels) {
        printk("Channel check failed: %d, %d\n", mapping.channel, local_audio_pdev->numChannels);
        return(-EINVAL);
      }

      XIo_Out32(LOCAL_AUDIO_REGISTER_BASE(local_audio_pdev, LOCAL_AUDIO_CHANNEL_REG + mapping.channel),
                mapping.streams);
    }
    break;

  case IOC_LA_GET_CHANNEL_MAPPING:
    {
      struct LocalAudioChannelMapping mapping;

      if(copy_from_user(&mapping, (void __user*)arg, sizeof(mapping)) != 0) {
        return(-EFAULT);
      }
			
      if (mapping.channel >= local_audio_pdev->numChannels) {
        return(-EINVAL);
      }
			
      mapping.streams = XIo_In32(LOCAL_AUDIO_REGISTER_BASE(local_audio_pdev, LOCAL_AUDIO_CHANNEL_REG + mapping.channel));

      if(copy_to_user((void __user*)arg, &mapping, sizeof(mapping)) != 0) {
        return(-EFAULT);
      }
    }
    break;

  case IOC_LA_SET_INSERT_MODE:
    {
      struct LocalAudioInsertConfig config;
      uint32_t value = 0;

      if(copy_from_user(&config, (void __user*)arg, sizeof(config)) != 0) {
        return(-EFAULT);
      }

      switch(config.mode)
        {
        default:
          return(-EINVAL);

        case LA_INSERT_MODE_OFF:
          break;

        case LA_INSERT_MODE_ZERO:
          value = LOCAL_AUDIO_INSERTER_ENABLE | LOCAL_AUDIO_INSERTER_ZERO;
          break;

        case LA_INSERT_MODE_DC:
          value = LOCAL_AUDIO_INSERTER_ENABLE | LOCAL_AUDIO_INSERTER_DC;
          break;

        case LA_INSERT_MODE_RAMP:
          value = LOCAL_AUDIO_INSERTER_ENABLE | LOCAL_AUDIO_INSERTER_RAMP;
          break;

        case LA_INSERT_MODE_LFSR:
          value = LOCAL_AUDIO_INSERTER_ENABLE;
          break;
        }

      value |= (config.stream << LOCAL_AUDIO_INSERTER_STREAM_SHIFT) & LOCAL_AUDIO_INSERTER_STREAM_MASK;
      value |= (config.slot << LOCAL_AUDIO_INSERTER_SLOT_SHIFT) & LOCAL_AUDIO_INSERTER_SLOT_MASK;
      
      XIo_Out32(LOCAL_AUDIO_INSERTER_BASE(local_audio_pdev, LOCAL_AUDIO_INSERTER_TDM_CTRL_REG),
                value);
    }
    break;
  case IOC_LA_SET_TESTER_MODE:
    {
      struct LocalAudioTesterConfig config;
      uint32_t value = 0;

      if(copy_from_user(&config, (void __user*)arg, sizeof(config)) != 0) {
        return(-EFAULT);
      }

      switch(config.mode)
        {
        default:
          return(-EINVAL);

        case LA_TESTER_MODE_OFF:
          break;

        case LA_TESTER_MODE_RAMP:
          value = LOCAL_AUDIO_TESTER_ENABLE | LOCAL_AUDIO_TESTER_RAMP;
          break;

        case LA_TESTER_MODE_LFSR:
          value = LOCAL_AUDIO_TESTER_ENABLE;
          break;

        }

      value |= (config.stream << LOCAL_AUDIO_TESTER_STREAM_SHIFT) & LOCAL_AUDIO_TESTER_STREAM_MASK;
      value |= (config.slot << LOCAL_AUDIO_TESTER_SLOT_SHIFT) & LOCAL_AUDIO_TESTER_SLOT_MASK;
      
      XIo_Out32(LOCAL_AUDIO_TESTER_BASE(local_audio_pdev, LOCAL_AUDIO_TESTER_TDM_CTRL_REG),
                value);
    }
    break;
  case IOC_LA_GET_TESTER_RESULTS:
    {
      struct LocalAudioTesterResults results;

      results.ctrl = XIo_In32(LOCAL_AUDIO_TESTER_BASE(local_audio_pdev, LOCAL_AUDIO_TESTER_TDM_CTRL_REG));
      results.error = XIo_In32(LOCAL_AUDIO_TESTER_BASE(local_audio_pdev, LOCAL_AUDIO_TESTER_ANALYSIS_ERROR_REG));
      results.predict = XIo_In32(LOCAL_AUDIO_TESTER_BASE(local_audio_pdev, LOCAL_AUDIO_TESTER_ANALYSIS_PREDICT_REG));
      results.actual = XIo_In32(LOCAL_AUDIO_TESTER_BASE(local_audio_pdev, LOCAL_AUDIO_TESTER_ANALYSIS_ACTUAL_REG));
      results.irq_mask = XIo_In32(LOCAL_AUDIO_TESTER_BASE(local_audio_pdev, LOCAL_AUDIO_TESTER_TDM_IRQ_MASK_REG));
      results.irq_flags = XIo_In32(LOCAL_AUDIO_TESTER_BASE(local_audio_pdev, LOCAL_AUDIO_TESTER_TDM_IRQ_FLAGS_REG));

      if(copy_to_user((void __user*)arg, &results, sizeof(results)) != 0) {
        return(-EFAULT);
      }

    }
    break;


  default:
    /* We don't recognize this command; first give an encapsulated DMA controller
     * a crack at it (if one exists), and then our derived driver (if one exists).
     */
    if(local_audio_pdev->dma != NULL) returnValue = labx_dma_ioctl(local_audio_pdev->dma, command, arg);
    if(((local_audio_pdev->dma == NULL) | (returnValue == -EINVAL)) &&
       (local_audio_pdev->derivedFops != NULL) && 
       (local_audio_pdev->derivedFops->ioctl != NULL)) {
      returnValue = local_audio_pdev->derivedFops->ioctl(inode, filp, command, arg);
    }
    return(returnValue);
  }

  /* Command handled locally without incident, return success */
  return(0);
}

static const struct file_operations labx_local_audio_fops = {
  .open    = labx_local_audio_open,
  .release = labx_local_audio_release,
  .ioctl   = labx_local_audio_ioctl_cdev,
};

/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 * This is exported to allow polymorphic drivers to invoke it.
 * @param name - Name of the instance
 * @param pdev - Platform device structure
 * @param addressRange  - Resource describing the hardware's I/O range
 * @param interfaceType - String identifying the interface type
 * @param numChannels   - Number of channels of output
 * @param derivedFops   - File operations to delegate to, NULL if unused
 * @param derivedData   - Pointer for derived driver to make use of
 * @param newInstance   - Pointer to the new driver instance, NULL if unused
 */
int labx_local_audio_probe(const char *name, 
                           struct platform_device *pdev,
                           struct resource *addressRange,
                           const char *interfaceType,
                           u32 numChannels,
                           struct file_operations *derivedFops,
                           void *derivedData,
                           struct labx_local_audio_pdev **newInstance) {
  struct labx_local_audio_pdev *local_audio_pdev;
  uint32_t deviceIndex;
  int32_t ret;

  /* Create and populate a device structure */
  local_audio_pdev = (struct labx_local_audio_pdev*) kzalloc(sizeof(struct labx_local_audio_pdev), GFP_KERNEL);
  if(!local_audio_pdev) return(-ENOMEM);

  /* Request and map the device's I/O memory region into uncacheable space */
  local_audio_pdev->physicalAddress = addressRange->start;
  local_audio_pdev->addressRangeSize = ((addressRange->end - addressRange->start) + 1);

  snprintf(local_audio_pdev->name, NAME_MAX_SIZE, "%s%d", name, instanceCount++);
  local_audio_pdev->name[NAME_MAX_SIZE - 1] = '\0';
  if(request_mem_region(local_audio_pdev->physicalAddress, local_audio_pdev->addressRangeSize,
                        local_audio_pdev->name) == NULL) {
    ret = -ENOMEM;
    goto free;
  }
  //printk("DMA Physical %08X\n", local_audio_pdev->physicalAddress);

  local_audio_pdev->virtualAddress = 
    (void*) ioremap_nocache(local_audio_pdev->physicalAddress, local_audio_pdev->addressRangeSize);
  if(!local_audio_pdev->virtualAddress) {
    ret = -ENOMEM;
    goto release;
  }
  printk("LA virtualAddress = 0x%08X, phys = 0x%08X, size = 0x%08X\n", 
         (uint32_t) local_audio_pdev->virtualAddress,
         (uint32_t) local_audio_pdev->physicalAddress,
         local_audio_pdev->addressRangeSize);

  /* Allocate and probe for a DMA if the hardware contains one */
  if(strcmp(interfaceType, LA_DMA_INTERFACE_EXTERNAL) != 0) {
    local_audio_pdev->dma = (struct labx_dma*) kzalloc(sizeof(struct labx_dma), GFP_KERNEL);
    local_audio_pdev->dma->virtualAddress = local_audio_pdev->virtualAddress;
    //printk("DMA Virtual %p\n", local_audio_pdev->dma.virtualAddress);
  } else local_audio_pdev->dma = NULL;
  
  local_audio_pdev->numChannels = numChannels;
  printk("  Local Audio interface found at 0x%08X: %d channels, %s DMA\n", 
         (uint32_t) local_audio_pdev->physicalAddress,
         local_audio_pdev->numChannels,
         ((local_audio_pdev->dma == NULL) ? "no" : "internal"));
  local_audio_pdev->miscdev.minor = MISC_DYNAMIC_MINOR;
  local_audio_pdev->miscdev.name = local_audio_pdev->name;
  local_audio_pdev->miscdev.fops = &labx_local_audio_fops;
  ret = misc_register(&local_audio_pdev->miscdev);
  if (ret) {
    printk(KERN_WARNING DRIVER_NAME ": Unable to register misc device.\n");
    goto unmap;
  }
  platform_set_drvdata(pdev, local_audio_pdev);
  local_audio_pdev->pdev = pdev;
  dev_set_drvdata(local_audio_pdev->miscdev.this_device, local_audio_pdev);

  /* Continue with DMA setup if appropriate */
  if(local_audio_pdev->dma != NULL) {
    /* Point the DMA at our name */
    local_audio_pdev->dma->name = local_audio_pdev->name;

    /* Call the general-purpose probe function for the Lab X Dma_Coprocessor.
     * Provide our device name, and assign no IRQ to the encapsulated DMA; it 
     * does not require one for this application.  Ask the underlying driver
     * code to infer the microcode RAM size for us.
     */
    ret = labx_dma_probe(local_audio_pdev->dma, 
                         MISC_MAJOR,
                         local_audio_pdev->miscdev.minor,
                         local_audio_pdev->name, 
                         DMA_UCODE_SIZE_UNKNOWN, 
                         DMA_NO_IRQ_SUPPLIED,
                         NULL);
    if(ret) {
      printk(KERN_WARNING DRIVER_NAME ": Probe of encapsulated DMA failed\n");
      goto unmap_dereg;
    }
  }

  /* Locate and occupy the first available device index for future navigation in
   * the call to labx_local_audio_open()
   */
  for (deviceIndex = 0; deviceIndex < MAX_LA_DEVICES; deviceIndex++) {
    if (NULL == devices[deviceIndex]) {
      devices[deviceIndex] = local_audio_pdev;
      break;
    }
  }

  /* Ensure that we haven't been asked to probe for too many devices */
  if(deviceIndex >= MAX_LA_DEVICES) {
    printk(KERN_WARNING DRIVER_NAME ": Maximum device count (%d) exceeded during probe\n",
           MAX_LA_DEVICES);
    goto unmap_dereg;
  }

  /* Retain any derived file operations & data to dispatch to */
  local_audio_pdev->derivedFops = derivedFops;
  local_audio_pdev->derivedData = derivedData;

  /* Return success, setting the return pointer if valid */
  if(newInstance != NULL) *newInstance = local_audio_pdev;
  return(0);

 unmap_dereg:
  misc_deregister(&local_audio_pdev->miscdev);
 unmap:
  iounmap(local_audio_pdev->virtualAddress);
 release:
  release_mem_region(local_audio_pdev->physicalAddress, 
                     local_audio_pdev->addressRangeSize);
 free:
  kfree(local_audio_pdev->dma);
  kfree(local_audio_pdev);
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

static int labx_local_audio_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
  struct resource r_mem_struct;
  struct resource *addressRange = &r_mem_struct;
  struct platform_device *pdev = to_platform_device(&ofdev->dev);
  const char *interfaceType;
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
  numChannels = (get_u32(ofdev, "xlnx,num-i2s-streams") * 2);

  /* Check the interface type to see if a DMA exists */
  has_serializer = get_u32(ofdev, "xlnx,has-serializer");
  if (has_serializer)
    interfaceType = (char *) of_get_property(ofdev->node, "xlnx,interface-type", NULL);
  else
    interfaceType = LA_DMA_INTERFACE_EXTERNAL;
  
  if(interfaceType == NULL) {
    dev_warn(&ofdev->dev, "labx_local_audio : (%s) No interface type defined\n", name);
    return(-1);
  }

  /* Dispatch to the generic function */
  return(labx_local_audio_probe(name, pdev, addressRange, interfaceType, numChannels, NULL, NULL, NULL));
}

static int __devexit labx_local_audio_pdev_remove(struct platform_device *pdev);

static int __devexit labx_local_audio_of_remove(struct of_device *dev)
{
  struct platform_device *pdev = to_platform_device(&dev->dev);
  labx_local_audio_pdev_remove(pdev);
  return(0);
}

static struct of_device_id labx_local_audio_of_match[] = {
  { .compatible = "xlnx,labx-local-audio-1.00.a", },
  { .compatible = "xlnx,labx-local-audio-1.01.a", },
  { .compatible = "xlnx,labx-local-audio-1.02.a", },
  { /* end of list */ },
};

static struct of_platform_driver labx_local_audio_of_driver = {
  .name        = DRIVER_NAME,
  .match_table = labx_local_audio_of_match,
  .probe       = labx_local_audio_of_probe,
  .remove      = __devexit_p(labx_local_audio_of_remove),
};

#endif /* CONFIG_OF */

static int labx_local_audio_pdev_probe(struct platform_device *pdev)
{
  struct resource *addressRange;

  /* Obtain the resources for this instance */
  addressRange = platform_get_resource(pdev, IORESOURCE_MEM, LABX_DMA_ADDRESS_RANGE_RESOURCE);
  if (!addressRange) return(-ENXIO);

  /* Dispatch to the generic function */
  /* TEMPORARY - We are always setting up for 24 streams and faking out a PLB interface
   * to make sure a DMA is included!
   * Need to define a platform data structure to encapsulate this information.
   */
  return(labx_local_audio_probe(pdev->name, pdev, addressRange, "DMA_PLB", 24, NULL, NULL, NULL));
}

/* This is exported to allow polymorphic drivers to invoke it */
int labx_local_audio_remove(struct labx_local_audio_pdev *local_audio_pdev) {
  int deviceIndex;

  /* Make sure the DMA unit is no longer running */
  labx_dma_release(local_audio_pdev->dma);

  misc_deregister(&local_audio_pdev->miscdev);

  for (deviceIndex = 0; deviceIndex < MAX_LA_DEVICES; deviceIndex++) {
    if (local_audio_pdev == devices[deviceIndex]) {
      devices[deviceIndex] = NULL;
      break;
    }
  }
  return(0);
}

static __devexit int labx_local_audio_pdev_remove(struct platform_device *pdev)
{
  struct labx_local_audio_pdev *local_audio_pdev = (struct labx_local_audio_pdev*)platform_get_drvdata(pdev);
  return(labx_local_audio_remove(local_audio_pdev));
}

/* Platform device driver structure */
static struct platform_driver labx_local_audio_platform_driver = {
  .probe  = labx_local_audio_pdev_probe,
  .remove = labx_local_audio_pdev_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __init labx_local_audio_driver_init(void)
{
  int returnValue;

  printk(KERN_INFO DRIVER_NAME ": Local Audio Driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright (c) Lab X Technologies, LLC\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&labx_local_audio_of_driver);
#endif
 
  /* Initialize the instance counter */
  instanceCount = 0;

  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&labx_local_audio_platform_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }

  return(0);
}

static void __exit labx_local_audio_driver_exit(void)
{
  /* Unregister as a platform device driver */
  platform_driver_unregister(&labx_local_audio_platform_driver);
}

module_init(labx_local_audio_driver_init);
module_exit(labx_local_audio_driver_exit);

MODULE_AUTHOR("Chris Wulff");
MODULE_LICENSE("GPL");
