/*
 *  linux/drivers/net/labx_avb/labx_audio_packetizer.c
 *
 *  Lab X Technologies AVB flexible audio packetizer driver
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

#include "labx_audio_packetizer.h"
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <xio.h>

/* Driver name and the revision range of hardware expected.
 * This driver will work with revision 1.1 only.
 */
#define DRIVER_NAME "labx_audio_packetizer"
#define HARDWARE_MIN_VERSION_MAJOR  1
#define HARDWARE_MIN_VERSION_MINOR  1
#define HARDWARE_MAX_VERSION_MAJOR  1
#define HARDWARE_MAX_VERSION_MINOR  1

/* Major device number for the driver */
#define DRIVER_MAJOR 250

/* Maximum number of packetizers and instance count */
#define MAX_INSTANCES 64
static uint32_t instanceCount;

#if 0
#define DBG(f, x...) pr_debug(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

/* Disables the passed instance */
static void disable_packetizer(struct audio_packetizer *packetizer) {
  XIo_Out32(REGISTER_ADDRESS(packetizer, CONTROL_REG), PACKETIZER_DISABLE);
}

/* Enables the passed instance */
static void enable_packetizer(struct audio_packetizer *packetizer) {
  XIo_Out32(REGISTER_ADDRESS(packetizer, CONTROL_REG), PACKETIZER_ENABLE);
}

/* Resets the state of the passed instance */
static void reset_packetizer(struct audio_packetizer *packetizer) {
  /* Disable the instance, and wipe its registers */
  disable_packetizer(packetizer);
}

/* Loads the passed microcode descriptor into the instance */
static void load_descriptor(struct audio_packetizer *packetizer,
                            ConfigWords *descriptor) {
  uint32_t wordIndex;
  uint32_t wordAddress;

  wordAddress = (MICROCODE_RAM_BASE(packetizer) + (descriptor->offset * sizeof(uint32_t)));
  for(wordIndex = 0; wordIndex < descriptor->numWords; wordIndex++) {
    XIo_Out32(wordAddress, descriptor->configWords[wordIndex]);
    wordAddress += sizeof(uint32_t);
  }
}

/* Reads back and copies a descriptor from the packetizer into the passed 
 * structure, using the address and size it specifies.
 */
static void copy_descriptor(struct audio_packetizer *packetizer,
                            ConfigWords *descriptor) {
  uint32_t wordIndex;
  uint32_t wordAddress;

  wordAddress = (MICROCODE_RAM_BASE(packetizer) + (descriptor->offset * sizeof(uint32_t)));
  for(wordIndex = 0; wordIndex < descriptor->numWords; wordIndex++) {
    descriptor->configWords[wordIndex] = XIo_In32(wordAddress);
    wordAddress += sizeof(uint32_t);
  }
}

/* Loads the passed packet template into the instance */
static void load_packet_template(struct audio_packetizer *packetizer, ConfigWords *template) {
  uint32_t wordIndex;
  uint32_t wordAddress;

  wordAddress = (TEMPLATE_RAM_BASE(packetizer) + (template->offset * sizeof(uint32_t)));
  for(wordIndex = 0; wordIndex < template->numWords; wordIndex++) {
    XIo_Out32(wordAddress, template->configWords[wordIndex]);
    wordAddress += sizeof(uint32_t);
  }
}

/* Sets the start vector the packetizer jumps to at the beginning of each 
 * isochronous interval 
 */
static void set_start_vector(struct audio_packetizer *packetizer, uint32_t startVector) {
  XIo_Out32(REGISTER_ADDRESS(packetizer, START_VECTOR_REG), startVector);
}

/* Configures a clock domain, including whether it is enabled */
static void configure_clock_domain(struct audio_packetizer *packetizer, 
                                   ClockDomainSettings *clockDomainSettings) {
  /* Set the timestamp interval, then enable or disable since we need to enable
   * last (it doesn't really matter if we disable last or not.)
   *
   * Actually use the SYT interval setting minus one, as the hardware uses this
   * as a terminal count value.
   */
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(packetizer, clockDomainSettings->clockDomain, 
                                          TS_INTERVAL_REG),
            (clockDomainSettings->sytInterval - 1));
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(packetizer, clockDomainSettings->clockDomain,
                                          DOMAIN_ENABLE_REG),
            ((clockDomainSettings->enabled != 0) ? DOMAIN_ENABLED : DOMAIN_DISABLED));
}

/*
 * Sets the presentation time offset applied to all media packets produced by
 * the passed packetizer.
 *
 * @param packetizer         Packetizer instance
 * @param presentationOffset Offset, in nanoseconds.  Has a range of 21 bits.
 */
static void set_presentation_offset(struct audio_packetizer *packetizer,
                                    uint32_t presentationOffset) {
  XIo_Out32(REGISTER_ADDRESS(packetizer, TS_OFFSET_REG), presentationOffset);
}

/*
 * Character device hook functions
 */

static int audio_packetizer_open(struct inode *inode, struct file *filp)
{
  struct audio_packetizer *packetizer;
  unsigned long flags;
  int returnValue = 0;

  packetizer = container_of(inode->i_cdev, struct audio_packetizer, cdev);
  filp->private_data = packetizer;

  /* Lock the mutex and ensure there is only one owner */
  preempt_disable();
  spin_lock_irqsave(&packetizer->mutex, flags);
  if(packetizer->opened) {
    returnValue = -1;
  } else {
    packetizer->opened = true;
  }

  spin_unlock_irqrestore(&packetizer->mutex, flags);
  preempt_enable();
  
  return(returnValue);
}

static int audio_packetizer_release(struct inode *inode, struct file *filp)
{
  struct audio_packetizer *packetizer = (struct audio_packetizer*)filp->private_data;
  unsigned long flags;

  preempt_disable();
  spin_lock_irqsave(&packetizer->mutex, flags);
  packetizer->opened = false;
  spin_unlock_irqrestore(&packetizer->mutex, flags);
  preempt_enable();
  return(0);
}

/* Buffer for storing configuration words */
static uint32_t configWords[MAX_CONFIG_WORDS];

/* I/O control operations for the driver */
static int audio_packetizer_ioctl(struct inode *inode, struct file *filp,
                                  unsigned int command, unsigned long arg)
{
  // Switch on the request
  struct audio_packetizer *packetizer = (struct audio_packetizer*)filp->private_data;

  switch(command) {
  case IOC_START_ENGINE:
    enable_packetizer(packetizer);
    break;

  case IOC_STOP_ENGINE:
    disable_packetizer(packetizer);
    break;

  case IOC_LOAD_DESCRIPTOR:
    {
      ConfigWords descriptor;

      if(copy_from_user(&descriptor, (void __user*)arg, sizeof(descriptor)) != 0) {
        return(-EFAULT);
      }
      if(copy_from_user(configWords, (void __user*)descriptor.configWords, 
                        (descriptor.numWords * sizeof(uint32_t))) != 0) {
        return(-EFAULT);
      }
      descriptor.configWords = configWords;
      load_descriptor(packetizer, &descriptor);
    }
    break;

  case IOC_COPY_DESCRIPTOR:
    {
      ConfigWords userDescriptor;
      ConfigWords localDescriptor;

      /* Copy into our local descriptor, then obtain buffer pointer from userland */
      if(copy_from_user(&userDescriptor, (void __user*)arg, sizeof(userDescriptor)) != 0) {
        return(-EFAULT);
      }
      localDescriptor.offset = userDescriptor.offset;
      localDescriptor.numWords = userDescriptor.numWords;
      localDescriptor.configWords = configWords;
      copy_descriptor(packetizer, &localDescriptor);
      if(copy_to_user((void __user*)userDescriptor.configWords, configWords, 
                      (userDescriptor.numWords * sizeof(uint32_t))) != 0) {
        return(-EFAULT);
      }
    }
    break;

  case IOC_LOAD_PACKET_TEMPLATE:
    {
      ConfigWords template;

      if(copy_from_user(&template, (void __user*)arg, sizeof(template)) != 0) {
        return(-EFAULT);
      }
      if(copy_from_user(configWords, (void __user*)template.configWords, 
                        (template.numWords * sizeof(uint32_t))) != 0) {
        return(-EFAULT);
      }
      template.configWords = configWords;
      load_packet_template(packetizer, &template);
    }
    break;

  case IOC_SET_START_VECTOR:
    {
      uint32_t startVector;
      if(copy_from_user(&startVector, (void __user*)arg, sizeof(startVector)) != 0) {
        return(-EFAULT);
      }
      set_start_vector(packetizer, startVector);
    }
    break;

  case IOC_CONFIG_CLOCK_DOMAIN:
    {
      ClockDomainSettings clockDomainSettings;
      if(copy_from_user(&clockDomainSettings, (void __user*)arg, sizeof(clockDomainSettings)) != 0) {
        return(-EFAULT);
      }
      configure_clock_domain(packetizer, &clockDomainSettings);
    }
    break;

  case IOC_GET_PACKETIZER_CAPS:
    {
      if(copy_to_user((void __user*)arg, &packetizer->capabilities, sizeof(PacketizerCaps)) != 0) {
        return(-EFAULT);
      }
    }
    break;

  case IOC_SET_PRESENTATION_OFFSET:
    {
      uint32_t presentationOffset;

      if(copy_from_user(&presentationOffset, (void __user*)arg, sizeof(presentationOffset)) != 0) {
        return(-EFAULT);
      }
      set_presentation_offset(packetizer, presentationOffset);
      break;
    }
      
  default:
    return(-EINVAL);
  }

  return(0);
}

/* Character device file operations structure */
static struct file_operations audio_packetizer_fops = {
  .open	   = audio_packetizer_open,
  .release = audio_packetizer_release,
  .ioctl   = audio_packetizer_ioctl,
  .owner   = THIS_MODULE,
};

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int audio_packetizer_probe(struct platform_device *pdev)
{
  struct resource *addressRange;
  struct audio_packetizer *packetizer;
  uint32_t capsWord;
  uint32_t versionWord;
  uint32_t versionMajor;
  uint32_t versionMinor;
  int returnValue;


  /* Obtain the resources for this instance */
  addressRange = platform_get_resource(pdev, IORESOURCE_MEM, PACKET_ENGINE_ADDRESS_RANGE_RESOURCE);
  if (!addressRange) return(-ENXIO);

  /* Create and populate a device structure */
  packetizer = (struct audio_packetizer*) kmalloc(sizeof(struct audio_packetizer), GFP_KERNEL);
  if(!packetizer) return(-ENOMEM);

  /* Request and map the device's I/O memory region into uncacheable space */
  packetizer->physicalAddress = addressRange->start;
  packetizer->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
  snprintf(packetizer->name, NAME_MAX_SIZE, "%s%d", pdev->name, pdev->id);
  packetizer->name[NAME_MAX_SIZE - 1] = '\0';
  if(request_mem_region(packetizer->physicalAddress, packetizer->addressRangeSize,
                        packetizer->name) == NULL) {
    returnValue = -ENOMEM;
    goto free;
  }

  packetizer->virtualAddress = 
    (void*) ioremap_nocache(packetizer->physicalAddress, packetizer->addressRangeSize);
  if(!packetizer->virtualAddress) {
    returnValue = -ENOMEM;
    goto release;
  }

  /* Read the capabilities word to determine how many of the lowest
   * address bits are used to index into the microcode RAM, and therefore how
   * many bits an address sub-range field gets shifted up.  Each instruction is
   * 32 bits, and therefore inherently eats two lower address bits.
   */
  capsWord = XIo_In32(REGISTER_ADDRESS(packetizer, CAPABILITIES_REG));
  packetizer->regionShift = ((capsWord & CODE_ADDRESS_BITS_MASK) + 2);

  /* Inspect and check the version to ensure it lies within the range of hardware
   * we support.
   */
  versionWord = XIo_In32(REGISTER_ADDRESS(packetizer, REVISION_REG));
  versionMajor = ((versionWord >> REVISION_FIELD_BITS) & REVISION_FIELD_MASK);
  versionMinor = (versionWord & REVISION_FIELD_MASK);
  if(((versionMajor < HARDWARE_MIN_VERSION_MAJOR) & 
      (versionMinor < HARDWARE_MIN_VERSION_MINOR)) |
     ((versionMajor > HARDWARE_MAX_VERSION_MAJOR) & 
      (versionMinor > HARDWARE_MAX_VERSION_MINOR))) {
    printk(KERN_INFO "%s: Found incompatible hardware version %d.%d at 0x%08X\n",
           packetizer->name, versionMajor, versionMinor, (uint32_t)packetizer->physicalAddress);
    returnValue = -ENXIO;
    goto unmap;
  }
  packetizer->capabilities.versionMajor = versionMajor;
  packetizer->capabilities.versionMinor = versionMinor;

  /* Capture more capabilities information */
  packetizer->capabilities.maxInstructions = (0x01 << (capsWord & CODE_ADDRESS_BITS_MASK));
  packetizer->capabilities.maxTemplateBytes = 
    (0x04 << ((capsWord >> TEMPLATE_ADDRESS_SHIFT) & TEMPLATE_ADDRESS_BITS_MASK));
  packetizer->capabilities.maxClockDomains = ((capsWord >> CLOCK_DOMAINS_SHIFT) & CLOCK_DOMAINS_MASK);

  /* Announce the device */
  printk(KERN_INFO "%s: Found Lab X packetizer %d.%d at 0x%08X\n", 
         packetizer->name, versionMajor, versionMinor, 
         (uint32_t)packetizer->physicalAddress);

  /* Initialize other resources */
  spin_lock_init(&packetizer->mutex);
  packetizer->opened = false;

  /* Provide navigation between the device structures */
  platform_set_drvdata(pdev, packetizer);
  packetizer->pdev = pdev;

  /* Reset the state of the packetizer */
  reset_packetizer(packetizer);

  /* Add as a character device to make the instance available for use */
  cdev_init(&packetizer->cdev, &audio_packetizer_fops);
  packetizer->cdev.owner = THIS_MODULE;
  kobject_set_name(&packetizer->cdev.kobj, "%s%d", pdev->name, pdev->id);
  packetizer->instanceNumber = instanceCount++;
  cdev_add(&packetizer->cdev, MKDEV(DRIVER_MAJOR, packetizer->instanceNumber), 1);

  /* Return success */
  return(0);

 unmap:
  iounmap(packetizer->virtualAddress);
 release:
  release_mem_region(packetizer->physicalAddress, packetizer->addressRangeSize);
 free:
  kfree(packetizer);
  return(returnValue);
}

static int audio_packetizer_remove(struct platform_device *pdev)
{
  struct audio_packetizer *packetizer;

  /* Get a handle to the packetizer and begin shutting it down */
  packetizer = platform_get_drvdata(pdev);
  if(!packetizer) return(-1);
  cdev_del(&packetizer->cdev);
  reset_packetizer(packetizer);
  iounmap(packetizer->virtualAddress);
  release_mem_region(packetizer->physicalAddress, packetizer->addressRangeSize);
  kfree(packetizer);
  return(0);
}

/* Platform device driver structure */
static struct platform_driver audio_packetizer_driver = {
  .probe  = audio_packetizer_probe,
  .remove = audio_packetizer_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __init audio_packetizer_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": AVB Audio Packetizer driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Lab X Technologies, LLC\n");

  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&audio_packetizer_driver)) < 0) {
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

static void __exit audio_packetizer_driver_exit(void)
{
  /* Unregister as a platform device driver */
  platform_driver_unregister(&audio_packetizer_driver);
}

module_init(audio_packetizer_driver_init);
module_exit(audio_packetizer_driver_exit);

MODULE_AUTHOR("Eldridge M. Mount IV <eldridge.mount@labxtechnologies.com>");
MODULE_DESCRIPTION("Lab X Technologies AVB Audio Packetizer driver");
MODULE_LICENSE("GPL");
