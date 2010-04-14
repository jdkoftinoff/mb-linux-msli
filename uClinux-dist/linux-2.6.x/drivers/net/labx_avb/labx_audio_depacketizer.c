/*
 *  linux/drivers/net/labx_avb/labx_audio_depacketizer.c
 *
 *  Lab X Technologies AVB flexible audio de-packetizer driver
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

#include <linux/autoconf.h>
#include "labx_audio_depacketizer.h"
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <xio.h>

/* Driver name and the revision of hardware expected. */
#define DRIVER_NAME "labx_audio_depacketizer"
#define HARDWARE_VERSION_MAJOR  1
#define HARDWARE_VERSION_MINOR  0

/* Major device number for the driver */
#define DRIVER_MAJOR 252

/* Maximum number of depacketizers and instance count */
#define MAX_INSTANCES 64
static uint32_t instanceCount;

#if 0
#define DBG(f, x...) pr_debug(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

/* Syntonization callback */
static void depacketizer_syntonize(uint32_t rtcIncrement, void *callbackParam) {
  struct audio_depacketizer *depacketizer = (struct audio_depacketizer*)callbackParam;
  
  /* Write the new RTC increment to the depacketizer's local syntonization
   * counter so that it remains syntonized to 802.1AS network time.
   */
  XIo_Out32(REGISTER_ADDRESS(depacketizer, RTC_INCREMENT_REG), rtcIncrement);
}

/* Disables the passed instance */
static void disable_depacketizer(struct audio_depacketizer *depacketizer) {
  /* Disable the micro-engine, and unregister its syntonization callback */
  XIo_Out32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG), DEPACKETIZER_DISABLE);
  remove_syntonize_callback(&depacketizer_syntonize, (void*)depacketizer);
}

/* Enables the passed instance */
static void enable_depacketizer(struct audio_depacketizer *depacketizer) {
  /* Initialize the local syntonized counter to start at a nominal rate, and
   * then register the instance with a callback for syntonization updates 
   */
  XIo_Out32(REGISTER_ADDRESS(depacketizer, RTC_INCREMENT_REG), NOMINAL_RTC_INCREMENT);
  add_syntonize_callback(&depacketizer_syntonize, (void*) depacketizer);

  /* Enable the micro-engine */
  XIo_Out32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG), DEPACKETIZER_ENABLE);
}

/* Loads the passed microcode descriptor into the instance */
static void load_descriptor(struct audio_depacketizer *depacketizer,
                            ConfigWords *descriptor) {
  uint32_t wordIndex;
  uint32_t wordAddress;

  wordAddress = (MICROCODE_RAM_BASE(depacketizer) + (descriptor->offset * sizeof(uint32_t)));
  for(wordIndex = 0; wordIndex < descriptor->numWords; wordIndex++) {
    XIo_Out32(wordAddress, descriptor->configWords[wordIndex]);
    wordAddress += sizeof(uint32_t);
  }
}

/* Reads back and copies a descriptor from the packetizer into the passed 
 * structure, using the address and size it specifies.
 */
static void copy_descriptor(struct audio_depacketizer *depacketizer,
                            ConfigWords *descriptor) {
  uint32_t wordIndex;
  uint32_t wordAddress;

  wordAddress = (MICROCODE_RAM_BASE(depacketizer) + (descriptor->offset * sizeof(uint32_t)));
  for(wordIndex = 0; wordIndex < descriptor->numWords; wordIndex++) {
    descriptor->configWords[wordIndex] = XIo_In32(wordAddress);
    wordAddress += sizeof(uint32_t);
  }
}

/* Busy loops until the match unit configuration logic is idle.  The hardware goes 
 * idle very quickly and deterministically after a configuration word is written, 
 * so this should not consume very much time at all.
 */
static void wait_match_config(struct audio_depacketizer *depacketizer) {
  uint32_t statusWord;
  do {
    statusWord = XIo_In32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG));
  } while(statusWord & ID_LOAD_ACTIVE);
}

/* Clears all of the match units within the passed instance */
static void clear_matchers(struct audio_depacketizer *depacketizer) {
  uint32_t numClearWords;
  uint32_t wordIndex;

  /* Ascertain that the configuration logic is ready, then clear all in one shot */
  wait_match_config(depacketizer);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_0_REG), ID_SELECT_ALL);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_1_REG), ID_SELECT_ALL);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_2_REG), ID_SELECT_ALL);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_3_REG), ID_SELECT_ALL);
  
  /* Load the appropriate number of clearing words to the LUTs */
  switch(depacketizer->matchArchitecture) {
  case STREAM_MATCH_SRLC16E:
    numClearWords = NUM_SRLC16E_CONFIG_WORDS;
    break;

  case STREAM_MATCH_SRLC32E:
    numClearWords = NUM_SRLC32E_CONFIG_WORDS;
    break;

  default:
    numClearWords = 0;
  }
  for(wordIndex = 0; wordIndex < numClearWords; wordIndex++) {
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_CONFIG_DATA_REG), SRLCXXE_CLEARING_WORD);
  }

  /* De-select all of the match units */
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_0_REG), ID_SELECT_NONE);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_1_REG), ID_SELECT_NONE);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_2_REG), ID_SELECT_NONE);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_3_REG), ID_SELECT_NONE);
}

/* Configures one of the passed instance's match units */
static int configure_matcher(struct audio_depacketizer *depacketizer,
                             MatcherConfig *matcherConfig) {
  uint32_t selectionWord;
  uint32_t vectorAddress;
  uint32_t vectorWord;
  uint32_t matchUnit;

  /* Sanity-check the input structure */
  matchUnit = matcherConfig->matchUnit;
  if(matchUnit >= MAX_CONCURRENT_STREAMS) return(-EINVAL);

  /* Ascertain that the configuration logic is ready, then select the matcher */
  wait_match_config(depacketizer);
  selectionWord = ((matchUnit < 32) ? (0x01 << matchUnit) : ID_SELECT_NONE);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_0_REG), selectionWord);
  selectionWord = (((matchUnit < 64) & (matchUnit > 32)) ? (0x01 << (matchUnit - 32)) : ID_SELECT_NONE);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_1_REG), selectionWord);
  selectionWord = (((matchUnit < 96) & (matchUnit > 64)) ? (0x01 << (matchUnit - 64)) : ID_SELECT_NONE);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_2_REG), selectionWord);
  selectionWord = ((matchUnit > 96) ? (0x01 << (matchUnit - 96)) : ID_SELECT_NONE);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_3_REG), selectionWord);

  /* Calculate matching truth tables for the LUTs and load them */
  switch(depacketizer->matchArchitecture) {
  case STREAM_MATCH_SRLC16E:
    {
      int32_t lutIndex;
      uint32_t configWord = 0x00000000;
      uint32_t matchChunk;
      bool loadLutPair = false;
      
      for(lutIndex = (NUM_SRLC16E_INSTANCES - 1); lutIndex >= 0; lutIndex--) {
        configWord <<= 16;
        matchChunk = ((matcherConfig->matchStreamId >> (lutIndex * 4)) & 0x0F);
        if(matcherConfig->enable == MATCHER_ENABLE) {
          configWord |= (0x01 << matchChunk);
        }
        if(loadLutPair) {
          XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_CONFIG_DATA_REG), configWord);
          wait_match_config(depacketizer);
        }
        loadLutPair = !loadLutPair;
      }
    }
    break;

  case STREAM_MATCH_SRLC32E:
    {
      int32_t lutIndex;
      uint32_t configWord;
      uint32_t matchChunk;
      
      for(lutIndex = (NUM_SRLC32E_INSTANCES - 1); lutIndex >= 0; lutIndex--) {
        matchChunk = ((matcherConfig->matchStreamId >> (lutIndex * 5)) & 0x01F);
        configWord = ((matcherConfig->enable == MATCHER_ENABLE) ? 
                      (0x01 << matchChunk) : SRLCXXE_CLEARING_WORD);
        XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_CONFIG_DATA_REG), configWord);
        wait_match_config(depacketizer);
      }
    }
    break;

  default:
    ;
  }

  /* De-select all of the match units */
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_0_REG), ID_SELECT_NONE);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_1_REG), ID_SELECT_NONE);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_2_REG), ID_SELECT_NONE);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_3_REG), ID_SELECT_NONE);

  /* 
   * Locate the unit's vector in the relocatable table and write it.  Match vectors
   * are packed in sixteen-bit pairs, in little-endian order.
   */
  vectorAddress = XIo_In32(REGISTER_ADDRESS(depacketizer, VECTOR_BAR_REG));
  vectorAddress += (matchUnit / MATCH_VECTORS_PER_WORD);
  vectorAddress = (MICROCODE_RAM_BASE(depacketizer) + (vectorAddress * sizeof(uint32_t)));
  vectorWord = XIo_In32(vectorAddress);
  if(matchUnit % MATCH_VECTORS_PER_WORD) {
    vectorWord &= (MATCH_VECTOR_MASK << MATCH_VECTOR_EVEN_SHIFT);
    vectorWord |= (matcherConfig->matchVector << MATCH_VECTOR_ODD_SHIFT);
  } else {
    vectorWord &= (MATCH_VECTOR_MASK << MATCH_VECTOR_ODD_SHIFT);
    vectorWord |= (matcherConfig->matchVector << MATCH_VECTOR_EVEN_SHIFT);
  }
  XIo_Out32(vectorAddress, vectorWord);
  return(0);
}

/* Sets the base address of the match vector table; all subsequent calls to configure
 * match units will make use of this value.  All of the match units are cleared first 
 * as a side-effect of this call, since it is unsafe to move the vector table with enabled
 * match units.
 */
static void set_vector_base_address(struct audio_depacketizer *depacketizer,
                                    uint32_t vectorBaseAddress) {
  clear_matchers(depacketizer);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, VECTOR_BAR_REG), vectorBaseAddress);
}

/* Configures a clock recovery domain, including whether it is enabled */
static void configure_clock_recovery(struct audio_depacketizer *depacketizer, 
                                     ClockRecoverySettings *clockRecoverySettings) {
  ClockDomainSettings *clockDomainSettings;
  uint32_t clockDomain;
  uint32_t recoveryIndex;

  /* Configure the timestamp interval for the domain first.  This informs the basic
   * reference clock recovery hardware of how many samples are being averaged each
   * time it receives a valid timestamp from the depacketizer.
   */
  clockDomainSettings = &clockRecoverySettings->clockDomainSettings;
  clockDomain = clockDomainSettings->clockDomain;
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, TS_INTERVAL_REG),
            clockDomainSettings->sytInterval);

  /* Configure the clock domain with which match unit it gets its temporal 
   * information from.  The match units, in turn, link a stream index to its AVBTP
   * stream ID, so a descriptor must be configured to receive a stream on this
   * index.
   */
  recoveryIndex = (clockRecoverySettings->matchUnit & STREAM_INDEX_MASK);
  if(clockDomainSettings->enabled == DOMAIN_ENABLED) {
    recoveryIndex |= RECOVERY_ENABLED;
  }
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, RECOVERY_INDEX_REG),
            recoveryIndex);

  /* Finally, configure the VCO control logic for the domain.  If the domain is enabled,
   * it is configured as a proportional controller to servo an external VCO via a DAC
   * to the audio clock frequency, based upon a feedback word clock.
   *
   * If the domain is disabled, the VCO control is set to output a constant zero value.
   * If a VCXO is being used, this will run it at the nominal frequency.
   *
   * For the moment, only an Analog Devices' AD5662 DAC and a Citizen CSX-750V VCXO are
   * being supported, so the parameters are hard-coded.  Once this is relaxed to permit
   * support for different clock recovery hardware, the parameters should be brought out
   * via the ioctl() interface.
   */
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, DAC_OFFSET_REG),
            DAC_OFFSET_ZERO);
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, DAC_P_COEFF_REG),
            ((clockDomainSettings->enabled == DOMAIN_ENABLED) ? DAC_COEFF_MAX : DAC_COEFF_ZERO));
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, LOCK_COUNT_REG),
            ((512 << VCO_LOCK_COUNT_SHIFT) | 0));

  /* TODO: Need to introduce some locked status and interrupt mask / flag bits in the hardware!
   *       Once they exist, a lock detection kernel event mechanism can be added to the driver
   *       and used when in slave mode.
   */
}

/* Resets the state of the passed instance */
static void reset_depacketizer(struct audio_depacketizer *depacketizer) {
  /* Disable the instance and all of its match units */
  disable_depacketizer(depacketizer);
  clear_matchers(depacketizer);
  set_vector_base_address(depacketizer, 0x00000000);
}

/*
 * Character device hook functions
 */

static int audio_depacketizer_open(struct inode *inode, struct file *filp)
{
  struct audio_depacketizer *depacketizer;
  unsigned long flags;
  int returnValue = 0;

  depacketizer = container_of(inode->i_cdev, struct audio_depacketizer, cdev);
  filp->private_data = depacketizer;

  /* Lock the mutex and ensure there is only one owner */
  preempt_disable();
  spin_lock_irqsave(&depacketizer->mutex, flags);
  if(depacketizer->opened) {
    returnValue = -1;
  } else {
    depacketizer->opened = true;
  }
  spin_unlock_irqrestore(&depacketizer->mutex, flags);
  preempt_enable();
  
  return(returnValue);
}

static int audio_depacketizer_release(struct inode *inode, struct file *filp)
{
  struct audio_depacketizer *depacketizer = (struct audio_depacketizer*)filp->private_data;
  unsigned long flags;

  preempt_disable();
  spin_lock_irqsave(&depacketizer->mutex, flags);
  depacketizer->opened = false;
  spin_unlock_irqrestore(&depacketizer->mutex, flags);
  preempt_enable();
  return(0);
}

/* Buffer for storing configuration words */
static uint32_t configWords[MAX_CONFIG_WORDS];

/* I/O control operations for the driver */
static int audio_depacketizer_ioctl(struct inode *inode, struct file *filp,
                                    unsigned int command, unsigned long arg)
{
  struct audio_depacketizer *depacketizer = (struct audio_depacketizer*)filp->private_data;
  int returnValue = 0;

  // Switch on the request
  switch(command) {
  case IOC_START_ENGINE:
    enable_depacketizer(depacketizer);
    break;

  case IOC_STOP_ENGINE:
    disable_depacketizer(depacketizer);
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
      load_descriptor(depacketizer, &descriptor);
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
      copy_descriptor(depacketizer, &localDescriptor);
      if(copy_to_user((void __user*)userDescriptor.configWords, configWords, 
                      (userDescriptor.numWords * sizeof(uint32_t))) != 0) {
        return(-EFAULT);
      }
    }
    break;

  case IOC_CLEAR_MATCHERS:
    clear_matchers(depacketizer);
    break;

  case IOC_CONFIG_MATCHER:
    {
      MatcherConfig matcherConfig;
      if(copy_from_user(&matcherConfig, (void __user*)arg, sizeof(matcherConfig)) != 0) return(-EFAULT);
      returnValue = configure_matcher(depacketizer, &matcherConfig);
    }
    break;

  case IOC_LOCATE_VECTOR_TABLE:
    {
      uint32_t vectorBaseAddress;

      if(copy_from_user(&vectorBaseAddress, (void __user*)arg, sizeof(vectorBaseAddress)) != 0) {
        return(-EFAULT);
      }
      set_vector_base_address(depacketizer, vectorBaseAddress);
    }
    break;

  case IOC_CONFIG_CLOCK_RECOVERY:
    {
      ClockRecoverySettings clockRecoverySettings;
      if(copy_from_user(&clockRecoverySettings, (void __user*)arg, sizeof(clockRecoverySettings)) != 0) {
        return(-EFAULT);
      }
      configure_clock_recovery(depacketizer, &clockRecoverySettings);
    }
    break;

  case IOC_GET_DEPACKETIZER_CAPS:
    {
      if(copy_to_user((void __user*)arg, &depacketizer->capabilities, sizeof(DepacketizerCaps)) != 0) {
        return(-EFAULT);
      }
    }
    break;
      
  default:
#ifdef CONFIG_LABX_AUDIO_DEPACKETIZER_DMA
    return labx_dma_ioctl(&depacketizer->dma, command, arg);
#else
    return(-EINVAL);
#endif
  }

  return(returnValue);
}

/* Character device file operations structure */
static struct file_operations audio_depacketizer_fops = {
  .open	   = audio_depacketizer_open,
  .release = audio_depacketizer_release,
  .ioctl   = audio_depacketizer_ioctl,
  .owner   = THIS_MODULE,
};

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int audio_depacketizer_probe(struct platform_device *pdev)
{
  struct resource *addressRange;
  struct audio_depacketizer *depacketizer;
  uint32_t capsWord;
  uint32_t versionWord;
  uint32_t versionMajor;
  uint32_t versionMinor;
  int returnValue;


  /* Obtain the resources for this instance */
  addressRange = platform_get_resource(pdev, IORESOURCE_MEM, PACKET_ENGINE_ADDRESS_RANGE_RESOURCE);
  if (!addressRange) return(-ENXIO);

  /* Create and populate a device structure */
  depacketizer = (struct audio_depacketizer*) kmalloc(sizeof(struct audio_depacketizer), GFP_KERNEL);
  if(!depacketizer) return(-ENOMEM);

  /* Request and map the device's I/O memory region into uncacheable space */
  depacketizer->physicalAddress = addressRange->start;
  depacketizer->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
  snprintf(depacketizer->name, NAME_MAX_SIZE, "%s%d", pdev->name, pdev->id);
  depacketizer->name[NAME_MAX_SIZE - 1] = '\0';
  if(request_mem_region(depacketizer->physicalAddress, depacketizer->addressRangeSize,
                        depacketizer->name) == NULL) {
    returnValue = -ENOMEM;
    goto free;
  }

  depacketizer->virtualAddress = 
    (void*) ioremap_nocache(depacketizer->physicalAddress, depacketizer->addressRangeSize);
  if(!depacketizer->virtualAddress) {
    returnValue = -ENOMEM;
    goto release;
  }

  /* Read the capabilities word to determine how many of the lowest
   * address bits are used to index into the microcode RAM, and therefore how
   * many bits an address sub-range field gets shifted up.  Each instruction is
   * 32 bits, and therefore inherently eats two lower address bits.
   */
  capsWord = XIo_In32(REGISTER_ADDRESS(depacketizer, CAPABILITIES_REG));
  depacketizer->regionShift = ((capsWord & CODE_ADDRESS_BITS_MASK) + 2);

  /* Inspect and check the version */
  versionWord = XIo_In32(REGISTER_ADDRESS(depacketizer, REVISION_REG));
  versionMajor = ((versionWord >> REVISION_FIELD_BITS) & REVISION_FIELD_MASK);
  versionMinor = (versionWord & REVISION_FIELD_MASK);
  if((versionMajor != HARDWARE_VERSION_MAJOR) | 
     (versionMinor != HARDWARE_VERSION_MINOR)) {
    printk(KERN_INFO "%s: Found incompatible hardware version %d.%d at 0x%08X\n",
           depacketizer->name, versionMajor, versionMinor, (uint32_t)depacketizer->physicalAddress);
    returnValue = -ENXIO;
    goto unmap;
  }
  depacketizer->capabilities.versionMajor = versionMajor;
  depacketizer->capabilities.versionMinor = versionMinor;

  /* Test and sanity-check the stream-matching architecture */
  depacketizer->matchArchitecture = ((capsWord >> MATCH_ARCH_SHIFT) & MATCH_ARCH_MASK);
  switch(depacketizer->matchArchitecture) {
  case STREAM_MATCH_SRLC16E:
  case STREAM_MATCH_SRLC32E:
    break;
  default:
    printk(KERN_INFO "%s: Invalid match architecture 0x%02X\n", 
           depacketizer->name, depacketizer->matchArchitecture);
    returnValue = -ENXIO;
    goto unmap;
  }

  /* Capture more capabilities information */
  depacketizer->capabilities.maxInstructions = (0x01 << (capsWord & CODE_ADDRESS_BITS_MASK));
  depacketizer->capabilities.maxParameters   = (0x01 << ((capsWord >> PARAM_ADDRESS_BITS_SHIFT) & 
                                                         PARAM_ADDRESS_BITS_MASK));
  depacketizer->capabilities.maxClockDomains = ((capsWord >> CLOCK_DOMAINS_SHIFT) & CLOCK_DOMAINS_MASK);
  depacketizer->capabilities.maxStreams      = ((capsWord >> MAX_STREAMS_SHIFT) & MAX_STREAMS_MASK);

  /* Announce the device */
  printk(KERN_INFO "%s: Found Lab X depacketizer %d.%d at 0x%08X\n", 
         depacketizer->name,
         HARDWARE_VERSION_MAJOR,
         HARDWARE_VERSION_MINOR,
         (uint32_t)depacketizer->physicalAddress);

  /* Initialize other resources */
  spin_lock_init(&depacketizer->mutex);
  depacketizer->opened = false;

#ifdef CONFIG_LABX_AUDIO_DEPACKETIZER_DMA
  depacketizer->dma.virtualAddress = depacketizer->virtualAddress + LABX_DMA_MEMORY_OFFSET;
 
  labx_dma_probe(&depacketizer->dma); 
#endif

  /* Provide navigation from the platform device structure */
  platform_set_drvdata(pdev, depacketizer);

  /* Reset the state of the depacketizer */
  reset_depacketizer(depacketizer);

  /* Add as a character device to make the instance available for use */
  cdev_init(&depacketizer->cdev, &audio_depacketizer_fops);
  depacketizer->cdev.owner = THIS_MODULE;
  kobject_set_name(&depacketizer->cdev.kobj, "%s%d", pdev->name, pdev->id);
  depacketizer->instanceNumber = instanceCount++;
  cdev_add(&depacketizer->cdev, MKDEV(DRIVER_MAJOR, depacketizer->instanceNumber), 1);

  platform_set_drvdata(pdev, depacketizer);
  depacketizer->pdev = pdev;

  /* Return success */
  return(0);

 unmap:
  iounmap(depacketizer->virtualAddress);
 release:
  release_mem_region(depacketizer->physicalAddress, depacketizer->addressRangeSize);
 free:
  kfree(depacketizer);
  return(returnValue);
}

static int audio_depacketizer_remove(struct platform_device *pdev)
{
  struct audio_depacketizer *depacketizer;

  /* Get a handle to the packetizer and begin shutting it down */
  depacketizer = platform_get_drvdata(pdev);
  if(!depacketizer) return(-1);
  cdev_del(&depacketizer->cdev);
  reset_depacketizer(depacketizer);
  iounmap(depacketizer->virtualAddress);
  release_mem_region(depacketizer->physicalAddress, depacketizer->addressRangeSize);
  kfree(depacketizer);
  return(0);
}

/* Platform device driver structure */
static struct platform_driver audio_depacketizer_driver = {
  .probe  = audio_depacketizer_probe,
  .remove = audio_depacketizer_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __init audio_depacketizer_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": AVB Audio Depacketizer %d.%d driver\n",
         HARDWARE_VERSION_MAJOR, HARDWARE_VERSION_MINOR);
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Lab X Technologies, LLC\n");

  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&audio_depacketizer_driver)) < 0) {
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

static void __exit audio_depacketizer_driver_exit(void)
{
  /* Unregister as a platform device driver */
  platform_driver_unregister(&audio_depacketizer_driver);
}

module_init(audio_depacketizer_driver_init);
module_exit(audio_depacketizer_driver_exit);

MODULE_AUTHOR("Eldridge M. Mount IV <eldridge.mount@labxtechnologies.com>");
MODULE_DESCRIPTION("Lab X Technologies AVB Audio Depacketizer driver");
MODULE_LICENSE("GPL");
