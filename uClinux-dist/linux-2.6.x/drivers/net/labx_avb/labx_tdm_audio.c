/*
 *  linux/drivers/net/labx_avb/labx_tdm_audio.c
 *
 *  Lab X Technologies AVB time domain multiplexer (TDM) driver
 *
 *  Written by Scott Wagner (scott.wagner@labxtechnologies.com)
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

#include "linux/labx_tdm_audio_defs.h"
#include "labx_tdm_audio.h"
#ifdef CONFIG_LABX_AUDIO_TDM_ANALYZER
#include "labx_tdm_analyzer.h"
#endif
#include <asm/io.h>
#include <linux/fs.h>
#include <linux/highmem.h>
#include <linux/platform_device.h>
#include <xio.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF

#define _LABXDEBUG

/* Driver name and the revision range of hardware expected.
 * This driver will work with revision 1.1 only.
  */
#define DRIVER_NAME "labx_tdm_audio"
#define DRIVER_VERSION_MIN  0x13
#define DRIVER_VERSION_MAX  0x13
#define REVISION_FIELD_BITS  4
#define REVISION_FIELD_MASK  (0x0F)

/* Maximum number of tdm modules and instance count */
#define MAX_INSTANCES 16
static uint32_t instanceCount;

/* Buffer for storing stream map entries */
static StreamMapEntry mapEntryBuffer[MAX_MAP_ENTRIES];
static uint8_t MAP_MUTE_MODE_SHIFT = 0x00;
static struct audio_tdm* devices[MAX_INSTANCES] = {};

/*
 * Character device hook functions
 */

static int tdm_open(struct inode *inode, struct file *filp)
{
  struct audio_tdm *tdm;
  unsigned long flags;
  int returnValue = 0;
  int deviceIndex;

  /* Search for the device amongst those which successfully were probed */
  for(deviceIndex = 0; deviceIndex < MAX_INSTANCES; deviceIndex++) {
    if ((devices[deviceIndex] != NULL) && (devices[deviceIndex]->miscdev.minor == iminor(inode))) {
      /* Retain the device pointer within the file pointer for future navigation */
      filp->private_data = devices[deviceIndex];
      break;
    }
  }

  /* Ensure the device was actually found */
  if(deviceIndex >= MAX_INSTANCES) {
    printk(KERN_WARNING DRIVER_NAME ": Could not find device for node (%d, %d)\n",
           imajor(inode), iminor(inode));
    return(-ENODEV);
  }

  /* TODO - Ensure the local audio hardware is reset */
  tdm = (struct audio_tdm*)filp->private_data;

  /* Lock the mutex and ensure there is only one owner */
  preempt_disable();
  spin_lock_irqsave(&tdm->mutex, flags);
  if(tdm->opened) {
    returnValue = -1;
  } else {
    tdm->opened = true;
  }

  /* Invoke the open() operation on the derived driver, if there is one */
  if((tdm->derivedFops != NULL) &&
     (tdm->derivedFops->open != NULL)) {
    tdm->derivedFops->open(inode, filp);
  }

  spin_unlock_irqrestore(&tdm->mutex, flags);
  preempt_enable();
  
#ifdef CONFIG_LABX_AUDIO_TDM_ANALYZER
  /* Open the analyzer, if we have one */
  if(tdm->tdmCaps.hasAnalyzer) {
    labx_tdm_analyzer_reset(&tdm->analyzer);
  }
#endif

  return(returnValue);
}

static int tdm_release(struct inode *inode, struct file *filp)
{
  struct audio_tdm *tdm = (struct audio_tdm *)filp->private_data;
  unsigned long flags;

  preempt_disable();
  spin_lock_irqsave(&tdm->mutex, flags);
  tdm->opened = false;

  /* Invoke the release() operation on the derived driver, if there is one */
  if((tdm->derivedFops != NULL) &&
     (tdm->derivedFops->release != NULL)) {
    tdm->derivedFops->release(inode, filp);
  }

  spin_unlock_irqrestore(&tdm->mutex, flags);
  preempt_enable();
  return(0);
}

static void configure_auto_mute(struct audio_tdm *tdm,
                                AutoMuteConfig *autoMuteConfig) {
  uint32_t entryIndex;
  uint32_t bankIndex;
  uint32_t entryWord;
  uint32_t numChannels;

  /* Grab the TDM mode */  
  numChannels = tdm->opConfig.TdmSlotDensity * tdm->tdmCaps.laneCount;

  /* Set the shift value for the muting on load */
  if(MAP_MUTE_MODE_SHIFT == 0x00) {
    if(tdm->tdmCaps.maxNumStreams == 8) {
      MAP_MUTE_MODE_SHIFT = 0x03;
    } else if(tdm->tdmCaps.maxNumStreams == 16) {
      MAP_MUTE_MODE_SHIFT = 0x04;
    } else if(tdm->tdmCaps.maxNumStreams == 32) {
      MAP_MUTE_MODE_SHIFT = 0x05;
    } else {
      MAP_MUTE_MODE_SHIFT = 0x06;
    }
  }

  /* Set any new stream map entries in both banks */
  for(bankIndex = 0; bankIndex < STREAM_MAP_BANKS; bankIndex++) {
    for(entryIndex = 0; entryIndex < autoMuteConfig->numMapEntries; entryIndex++) {
      StreamMapEntry *entryPtr = &(autoMuteConfig->mapEntries[entryIndex]);

      /* Each entry consists of a map from a TDM channel to its stream */
      entryWord = (((((entryPtr->tdmChannel/(numChannels/tdm->tdmCaps.laneCount)) 
        * tdm->tdmCaps.maxSlotDensity) + (entryPtr->tdmChannel % (numChannels/tdm->tdmCaps.laneCount)))
            << MAP_CHANNEL_SHIFT) & MAP_CHANNEL_MASK);

      if(entryPtr->avbStream != AVB_STREAM_NONE) { 
        entryWord |= (entryPtr->avbStream & MAP_STREAM_MASK);
        entryWord |= ((autoMuteConfig->enable < entryPtr->enable ? autoMuteConfig->enable : entryPtr->enable) << MAP_MUTE_MODE_SHIFT);
      } else {
        entryWord |= AVB_STREAM_RESET;
        entryWord |= (AUTO_MUTE_ALWAYS << MAP_MUTE_MODE_SHIFT);      
      }

      entryWord |= (autoMuteConfig->enable << MAP_MUTE_MODE_SHIFT);

      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_STREAM_MAP_REG), entryWord);
    }

    /* The same register address is overloaded for the bank swap operation;
     * simply hit the appropriate control bit and the map load function is suppressed
     */
    XIo_Out32(REGISTER_ADDRESS(tdm, TDM_STREAM_MAP_REG), MAP_SWAP_BANK);
  }
}

static int set_audio_tdm_control(struct audio_tdm *tdm,
                                  AudioTdmControl *tdmControl) {
  int returnValue = 0;
  uint32_t reg;

  switch (tdmControl->bitMask)
  {
    case SLOT_DENSITY: 
      /* Evaluate the slot density to see if is one of the supported configurations */
      if(tdmControl->slotDensity != 2 && tdmControl->slotDensity != 4 &&
            tdmControl->slotDensity != 8 && tdmControl->slotDensity != 16 &&
              tdmControl->slotDensity != 32 && tdmControl->slotDensity != 64) {
        return -ESLTDSTYNOTSUPPORTED;
      }
      
      /* Evaluate the current sample rate, the maximum number channels supported
         scales based on the current TDM configuration. TdmSampleRate << 1 results
         in the multiplier from 48K to the current sample rate. */
      if(tdmControl->slotDensity >
         (tdm->tdmCaps.maxSlotDensity / (tdm->opConfig.TdmSampleRate << 1 | !tdm->opConfig.TdmSampleRate))) {
        return -ESLTDSTYEXCDSCONF;
      }

      tdm->opConfig.TdmSlotDensity = tdmControl->slotDensity;
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & ~TDM_SLOT_DENSITY_MASK;
      reg |= tdm->opConfig.TdmSlotDensity;
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
      printk("TDM (ioctl): slot density set to %u\n", tdmControl->slotDensity);
#endif
      break;

    case NUM_CHANNELS:
      /* Evaluate the number of channels to see if it is one of the supported configurations */
      if(tdmControl->numChannels != 8 && tdmControl->numChannels != 16 &&
            tdmControl->numChannels != 32 && tdmControl->numChannels != 64 &&
              tdmControl->numChannels != 128 && tdmControl->numChannels != 256) {
        return -ENUMCHNOTSUPPORTED;
      }
     
      /* Evaluate the number of channels to ensure it doesn't exceed the maximum */
      if(tdmControl->numChannels > (tdm->tdmCaps.laneCount * tdm->tdmCaps.maxSlotDensity)) {
        return -ENUMCHNOTSUPPORTED;
      }

      /* Evaluate the number of channels to see if it is not less than the minimum supported */
      if(tdmControl->numChannels < (tdm->tdmCaps.laneCount * LABX_TDM_MIN_SLOT_DENSITY)) {
        return -ENUMCHNOTSUPPORTED;
      }

      /* Evaluate the current sample rate, the maximum number channels supported
         scales based on the current TDM configuration. TdmSampleRate << 1 results
         in the multiplier from 48K to the current sample rate. */
      if(((tdmControl->numChannels / tdm->tdmCaps.laneCount) >
          (tdm->tdmCaps.maxSlotDensity / (tdm->opConfig.TdmSampleRate << 1 | !tdm->opConfig.TdmSampleRate)))) {
        return -ENUMCHEXCDSCONF;
      }

      tdm->opConfig.TdmSlotDensity = (tdmControl->numChannels / tdm->tdmCaps.laneCount);
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & ~TDM_SLOT_DENSITY_MASK;
      reg |= tdm->opConfig.TdmSlotDensity;
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
      printk("TDM (ioctl): set num channels to %u\n", tdmControl->numChannels);
#endif
      break;

    case BURST_LENGTH_DIVIDER:
      if(tdmControl->burstLengthDivider != 1 && tdmControl->burstLengthDivider != 2 &&
          tdmControl->burstLengthDivider != 4) {
         return -EBADBURSTLENDIV;
      }
      
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & ~TDM_BURST_LENGTH_DIV_MASK;
      reg |= (tdmControl-> burstLengthDivider << TDM_BURST_LENGTH_DIV_BITS);
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
      printk("TDM (ioctl): set burst length to %u\n", (tdm->tdmCaps.maxBurstLength / tdmControl->burstLengthDivider));
#endif
      break;

    case I2S_ALIGN:
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      if (tdmControl->i2sAlign == BIT_ALIGNMENT_LEFT_JUSTIFIED) {
        reg &= ~TDM_BIT_ALIGNMENT_I2S_DELAYED;
      } else {
        reg |= TDM_BIT_ALIGNMENT_I2S_DELAYED;
      }
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
      printk("TDM (ioctl): set alignment to %s\n",
             (tdmControl->i2sAlign == BIT_ALIGNMENT_LEFT_JUSTIFIED ? "left justified" : "delayed"));
#endif
      break;
  
    case LR_CLOCK_MODE:
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      if (tdmControl->lrClockMode == LRCLK_MODE_NORMAL) {
        reg &= ~TDM_LRCLK_MODE_PULSE;
      } else {
        reg |= TDM_LRCLK_MODE_PULSE;
      }
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
      printk("TDM (ioctl): set LR clock mode to %s\n",
             (tdmControl->lrClockMode == LRCLK_MODE_NORMAL ? "normal" : "pulsed"));
#endif
      break;

    case SAMPLE_EDGE:
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      if (tdmControl->sampleEdge == LRCLK_RISING_EDGE_CH0) {
        reg &= ~TDM_LRCLK_FALLING_EDGE_CH0;
      } else {
        reg |= TDM_LRCLK_FALLING_EDGE_CH0;
      }
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
      printk("TDM (ioctl): set LR clock edge to %s\n",
             (tdmControl->sampleEdge == LRCLK_FALLING_EDGE_CH0 ? "falling" : "rising"));
#endif
      break;
    
    case SAMPLE_RATE:
      /* Set the sample rate for the clock domain */
      switch(tdmControl->sampleRate) {
      case SAMPLE_RATE_32_KHZ:
      case SAMPLE_RATE_44_1_KHZ:
      case SAMPLE_RATE_48_KHZ:
        tdm->opConfig.TdmSampleRate = SINGLE_SAMPLE_RATE;
#ifdef _LABXDEBUG
        printk("TDM (ioctl): set sample rate to 48K\n");
#endif
        break;

      case SAMPLE_RATE_88_2_KHZ:
      case SAMPLE_RATE_96_KHZ:
        tdm->opConfig.TdmSampleRate = DOUBLE_SAMPLE_RATE;
#ifdef _LABXDEBUG
        printk("TDM (ioctl): set sample rate to 96K\n");
#endif
        break;
  
      case SAMPLE_RATE_176_4_KHZ:
      case SAMPLE_RATE_192_KHZ:
        tdm->opConfig.TdmSampleRate = QUAD_SAMPLE_RATE;
#ifdef _LABXDEBUG
        printk("TDM (ioctl): set sample rate to 192K\n");
#endif
        break;

      default:
#ifdef _LABXDEBUG
        printk("TDM (ioctl): invalid sample rate provided\n");
#endif
        return -ESMPLRATEINVALID;
      }

      /* Evaluate the current slot density, the maximum sample rate supported
         scales based on the current slot density. tdm->opConfig.TdmSampleRate << 1
         is the multiplier from 48KHz to the specified sample rate. */
      if((tdm->opConfig.TdmSlotDensity > 
         (tdm->tdmCaps.maxSlotDensity / (tdm->opConfig.TdmSampleRate << 1 | !tdm->opConfig.TdmSampleRate)))) {
        return -ESMPLRATENOTSUPPORTED;
      }

      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & ~TDM_SAMPLE_RATE_MASK;
      reg |= (tdm->opConfig.TdmSampleRate << TDM_SAMPLE_RATE_BITS);
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
      break;

    case SAMPLE_DEPTH:    
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      if (tdmControl->sampleDepth == SAMPLE_DEPTH_24BIT) {
        reg &= ~TDM_SAMPLE_DEPTH_16BIT;
      } else {
        reg |= TDM_SAMPLE_DEPTH_16BIT;
      }
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
      printk("TDM (ioctl): set sample depth to %u-bit\n",
             (tdmControl->sampleDepth == SAMPLE_DEPTH_24BIT ? 24 : 16));
#endif
      break;

    case TDM_MODULE_OWNER:   
      // Ensure the slave clock manager is built into the TDM
      if(!tdm->tdmCaps.hasSlaveManager) {
        return -ESCMNOTIMPL;
      }

      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      if (tdmControl->tdmModuleOwner == MASTER_MODE) {
        reg &= ~TDM_MODULE_SLAVE_MODE;
      } else {
        reg |= TDM_MODULE_SLAVE_MODE;
      }
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg); 
#ifdef _LABXDEBUG
      printk("TDM (ioctl): set module owner to %s\n",
             (tdmControl->tdmModuleOwner == MASTER_MODE ? "master" : "slave"));
#endif
      break;

    case TDM_TX_OWNER:    
      // Ensure the slave clock manager is built into the TDM
      if(!tdm->tdmCaps.hasSlaveManager) {
        return -ESCMNOTIMPL;
      }

      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      if (tdmControl->tdmTxOwner == MASTER_MODE) {
        reg &= ~TDM_TX_SLAVE_MODE;
      } else {
        reg |= TDM_TX_SLAVE_MODE;
      }
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg); 
#ifdef _LABXDEBUG
      printk("TDM (ioctl): tx owner set to %s\n",
             (tdmControl->tdmTxOwner == MASTER_MODE ? "master" : "slave"));
#endif
      break;

    case TDM_RX_OWNER:    
      // Ensure the slave clock manager is built into the TDM
      if(!tdm->tdmCaps.hasSlaveManager) {
        return -ESCMNOTIMPL;
      }

      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      if (tdmControl->tdmRxOwner == MASTER_MODE) {
        reg &= ~TDM_RX_SLAVE_MODE;
      } else {
        reg |= TDM_RX_SLAVE_MODE;
      }
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg); 
#ifdef _LABXDEBUG
      printk("TDM (ioctl): set rx owner to %s\n",
             (tdmControl->tdmRxOwner == MASTER_MODE ? "master" : "slave"));
#endif
      break;

    case MCLK_DIVIDER:    
      // Evaluate the slot density to see if is one of the supported configurations
      if(tdmControl->mclkDivider != 1 && tdmControl->mclkDivider != 2 &&
            tdmControl->mclkDivider != 4 && tdmControl->mclkDivider != 8) {
        return -EMCLKDNOTSUPPORTED;
      }

      /* Check the frequency of the master to clock, return an
         invalid value if the divider will scale the master clock
         output beyond the nominal frequency */
      if((tdm->tdmCaps.mclkRatio / tdmControl->mclkDivider) < 256) {
        return -EMCLKDTOOHIGH;
      }

      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & ~TDM_MCLK_DIVIDER_MASK;
      reg |= (tdmControl->mclkDivider << TDM_MCLK_DIVIDER_BITS);
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
      printk("TDM (ioctl): set mclk divider to %u\n", tdmControl->mclkDivider);
#endif
      break;

    default:
      returnValue = -EINVAL;
      break;
  }

  /* Return an error code appropriate to the command */
  return(returnValue);
}

static int get_audio_tdm_control(struct audio_tdm *tdm,
                                 AudioTdmControl *tdmControl) {
  uint32_t reg;
  int returnValue;

  switch (tdmControl->bitMask)
  {
    case SLOT_DENSITY: 
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      tdmControl->slotDensity = reg & TDM_SLOT_DENSITY_MASK;
      break;

    case NUM_CHANNELS:
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      tdmControl->numChannels = ((reg & TDM_SLOT_DENSITY_MASK) * tdm->tdmCaps.laneCount);
      break;

    case BURST_LENGTH_DIVIDER:
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      tdmControl->burstLengthDivider = (reg & TDM_BURST_LENGTH_DIV_MASK) >> TDM_BURST_LENGTH_DIV_BITS;
      break;

    case I2S_ALIGN:
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      tdmControl->i2sAlign = ((reg & TDM_BIT_ALIGNMENT_I2S_DELAYED) != 0);
      break;
  
    case LR_CLOCK_MODE:
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      tdmControl->lrClockMode = ((reg & TDM_LRCLK_MODE_PULSE) != 0);
      break;

    case SAMPLE_EDGE:
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      tdmControl->sampleEdge = ((reg & TDM_LRCLK_RISING_EDGE_CH0) != 0);
      break;

    case SAMPLE_RATE:
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      tdmControl->sampleRate = (reg & TDM_SAMPLE_RATE_MASK) >> TDM_SAMPLE_RATE_BITS;
      break;

    case SAMPLE_DEPTH:
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      tdmControl->sampleDepth = ((reg & TDM_SAMPLE_DEPTH_16BIT) != 0);
      break;

    case TDM_MODULE_OWNER:
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      tdmControl->tdmModuleOwner = ((XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) 
                                     & TDM_MODULE_OWNER_MASK) >> TDM_MODULE_OWNER_BITS) == 0x3;
      break;

    case TDM_TX_OWNER:
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      tdmControl->tdmTxOwner = ((reg & TDM_TX_SLAVE_MODE) != 0);
      break;

    case TDM_RX_OWNER:
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      tdmControl->tdmRxOwner = ((reg & TDM_RX_SLAVE_MODE) != 0);
      break;

    case MCLK_DIVIDER:
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      tdmControl->mclkDivider = (reg & TDM_MCLK_DIVIDER_MASK) >> TDM_MCLK_DIVIDER_BITS;
      break;

    default:
      returnValue = -EINVAL;
      break;
  }
  /* Return an error code appropriate to the command */
  return(returnValue);
}

/* I/O control operations for the driver */
static int tdm_ioctl(struct inode *inode,
                     struct file *filp,
                     unsigned int command,
                     unsigned long arg) {
  // Switch on the request
  AudioTdmControl tdmControl;
  int returnValue = 0;
  struct audio_tdm *tdm = (struct audio_tdm *)filp->private_data;

  switch(command) {
  case IOC_GET_AUDIO_TDM_CONTROL:
    if(copy_from_user(&tdmControl, (void __user*)arg, sizeof(AudioTdmControl)) != 0) {
      return(-EFAULT);
    }
    returnValue = get_audio_tdm_control(tdm, &tdmControl);
    if(copy_to_user((void __user*)arg, &tdmControl, sizeof(AudioTdmControl)) != 0) {
      return(-EFAULT);
    }
    break;

  case IOC_SET_AUDIO_TDM_CONTROL:
    if(copy_from_user(&tdmControl, (void __user*)arg, sizeof(AudioTdmControl)) != 0) {
      return(-EFAULT);
    }
    returnValue = set_audio_tdm_control(tdm, &tdmControl);
    break;

  case IOC_GET_AUDIO_TDM_CAPS:
    {
      if(copy_to_user((void __user*)arg, &tdm->tdmCaps, sizeof(tdm->tdmCaps)) != 0) {
        return(-EFAULT);
      }
    }
    break;

  case IOC_CONFIG_AUTO_MUTE:
    {
      AutoMuteConfig autoMuteConfig;

      /* First copy the main configuration structure */
      if(copy_from_user(&autoMuteConfig, (void __user*)arg, sizeof(autoMuteConfig)) != 0) {
        return(-EFAULT);
      }
 
      /* Sanity-check the number of map entries user space is attempting to
       * configure in this single call, then copy them to the statically-
       * allocated buffer for this purpose.
       */
      if(autoMuteConfig.numMapEntries > MAX_MAP_ENTRIES) return(-EFAULT);
      if(copy_from_user(mapEntryBuffer, (void __user*)autoMuteConfig.mapEntries,
                        (autoMuteConfig.numMapEntries * sizeof(StreamMapEntry))) != 0) {
        return(-EFAULT);
      }
      autoMuteConfig.mapEntries = mapEntryBuffer;
      configure_auto_mute(tdm, &autoMuteConfig);
    }
    break;

  case IOC_ARM_ERROR_IRQS:
    {
      // Enable the DMA error interrupt as a "one-shot"
      uint32_t irqMask = XIo_In32(REGISTER_ADDRESS(tdm, TDM_IRQ_MASK_REG));
      irqMask |= DMA_ERROR_IRQ;
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_IRQ_FLAGS_REG), DMA_ERROR_IRQ);
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_IRQ_MASK_REG), irqMask);
    }
    break;

  default:
#ifdef CONFIG_LABX_AUDIO_TDM_ANALYZER
    if(tdm->tdmCaps.hasAnalyzer) {
      return labx_tdm_analyzer_ioctl(&tdm->analyzer, command, arg);
    } else return(-EINVAL);
#else
#ifdef _LABXDEBUG
    printk("TDM (ioctl): bad ioctl call (command = %u)\n", command);
#endif
    returnValue = -EINVAL;
    break;
#endif

  }
  /* Return an error code appropriate to the command */
  return(returnValue);
}

/* Interrupt service routine for the driver */
static irqreturn_t labx_audio_tdm_interrupt(int irq, void *dev_id) {
  struct audio_tdm *tdm = (struct audio_tdm*) dev_id;
  uint32_t maskedFlags;
  uint32_t irqMask;

  // Read the interrupt flags and immediately clear them 
  maskedFlags = XIo_In32(REGISTER_ADDRESS(tdm, TDM_IRQ_FLAGS_REG));
  irqMask = XIo_In32(REGISTER_ADDRESS(tdm, TDM_IRQ_MASK_REG));
  maskedFlags &= irqMask;
  XIo_Out32(REGISTER_ADDRESS(tdm, TDM_IRQ_FLAGS_REG), maskedFlags);

  // Detect the DMA error IRQ 
  if((maskedFlags & DMA_ERROR_IRQ) != 0) {
    // TEMPORARY - Just announce this and treat it as a one-shot.
    //             Ultimately this should be communicated via generic Netlink.
    irqMask &= ~DMA_ERROR_IRQ;
    //XIo_Out32(REGISTER_ADDRESS(tdm, TDM_IRQ_MASK_REG), irqMask);
    //printk("%s: DMA error!\n", tdm->name);
  }
  
  // Detect the pseudorandom analysis error IRQ 
#ifdef CONFIG_LABX_AUDIO_TDM_ANALYZER
  if((maskedFlags & ANALYSIS_ERROR_IRQ) != 0) {
    labx_tdm_analyzer_interrupt(&tdm->analyzer, maskedFlags);
  }
#endif

  return(IRQ_HANDLED);
}

/* Character device file operations structure */
static struct file_operations tdm_fops = {
  .open	   = tdm_open,
  .release = tdm_release,
  .ioctl   = tdm_ioctl,
  .owner   = THIS_MODULE,
};

/* Resets the state of the passed instance */
static void reset_tdm(struct audio_tdm *tdm) {
  uint32_t bankIndex;
  uint32_t channelIndex;
  uint32_t numChannels;

  /* Restore the instance registers to initial values */
  XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), tdm->initialVal);

#ifdef CONFIG_LABX_AUDIO_TDM_ANALYZER
  /* Disable the pseudorandom analyzer */
  labx_tdm_analyzer_reset(&tdm->analyzer);
#endif

  /* Clear any old assignments from the auto-mute stream map; all channels
   * are presumed to have no valid stream assignment.
   */
  numChannels = tdm->tdmCaps.maxSlotDensity * tdm->tdmCaps.laneCount;
  for(bankIndex = 0; bankIndex < STREAM_MAP_BANKS; bankIndex++) {
    for(channelIndex = 0; channelIndex < numChannels; channelIndex++) {
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_STREAM_MAP_REG),
                (channelIndex << MAP_CHANNEL_SHIFT));
    }
    XIo_Out32(REGISTER_ADDRESS(tdm, TDM_STREAM_MAP_REG), MAP_SWAP_BANK);
  }
  return;
}

static ssize_t tdm_r_channels(struct class *c, char *buf)
{
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
  return (snprintf(buf, PAGE_SIZE, "%d\n",
           (XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & TDM_SLOT_DENSITY_MASK) * tdm->tdmCaps.laneCount));
}

static ssize_t tdm_w_channels(struct class *c, const char *buf, size_t count) 
{
  uint32_t reg;
  unsigned long int val;
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);

  if (strict_strtoul(buf, 0, &val) == 0) {
    /* Evaluate the number of channels to see if is one of the supported configurations */
    if(val != 8 && val != 16 && val != 32 && val != 64 && val != 128 && val != 256) {
      return -ENUMCHNOTSUPPORTED;
    }
      
    /* Evaluate the number of channels to ensure it doesn't exceed the maximum */
    if(val > (tdm->tdmCaps.laneCount * tdm->tdmCaps.maxSlotDensity)) {
      return -ENUMCHNOTSUPPORTED;
    }
   
    /* Evaluate the number of channels to see if it is not less than the minimum supported */
    if(val < (tdm->tdmCaps.laneCount * LABX_TDM_MIN_SLOT_DENSITY)) {
      return -ENUMCHNOTSUPPORTED;
    }

    /* Evaluate the current sample rate, the maximum number channels supported
       scales based on the current TDM configuration. TdmSampleRate << 1 results
       in the multiplier from 48K to the current sample rate. */
    if((val / tdm->tdmCaps.laneCount) > (tdm->tdmCaps.maxSlotDensity / (tdm->opConfig.TdmSampleRate << 1 | !tdm->opConfig.TdmSampleRate))) {
      return -ENUMCHEXCDSCONF;
    }

    tdm->opConfig.TdmSlotDensity = (val / tdm->tdmCaps.laneCount);
    reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & ~TDM_SLOT_DENSITY_MASK;
    reg |= tdm->opConfig.TdmSlotDensity;
    XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
    printk("TDM: set num channels to %u\n", (uint32_t)val);
#endif
  }
  return count;
}

static ssize_t tdm_r_slot_density(struct class *c, char *buf)
{
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
  return (snprintf(buf, PAGE_SIZE, "%d\n",
           XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & TDM_SLOT_DENSITY_MASK));
}


static ssize_t tdm_w_slot_density(struct class *c, const char * buf, size_t count)
{
  uint32_t reg;
  int32_t err = 0;
  unsigned long int val;
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);

  if (strict_strtoul(buf, 0, &val) == 0) {
    /* Evaluate the slot density to see if is one of the supported configurations */
    if(val != 2 && val != 4 && val != 8 && val != 16 && val != 32 && val != 64) {
      err = -ESLTDSTYNOTSUPPORTED;
    }
      
    /* Evaluate the current sample rate, the maximum number channels supported
       scales based on the current TDM configuration. TdmSampleRate << 1 results
       in the multiplier from 48K to the current sample rate. */
    if(val > (tdm->tdmCaps.maxSlotDensity / (tdm->opConfig.TdmSampleRate << 1 | !tdm->opConfig.TdmSampleRate))) {
      err = -ESLTDSTYEXCDSCONF;
    }

    if(!(err < 0)) {
      tdm->opConfig.TdmSlotDensity = val;
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & ~TDM_SLOT_DENSITY_MASK;
      reg |= val;
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
      printk("TDM: slot density set to %u\n", (uint32_t)val);
#endif
    }
  }
  return (err < 0 ? err : count);
}

static ssize_t tdm_r_burst_length_div(struct class *c, char *buf)
{
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
  return (snprintf(buf, PAGE_SIZE, "%d\n",
           (XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) 
              & TDM_BURST_LENGTH_DIV_MASK) >> TDM_BURST_LENGTH_DIV_BITS));
}


static ssize_t tdm_w_burst_length_div(struct class *c, const char * buf, size_t count)
{
  uint32_t reg;
  bool setVal = true;
  unsigned long int val;
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);

  if (strict_strtoul(buf, 0, &val) == 0) {
    if(val != 1 && val != 2 && val != 4) {
      setVal = false;
    }
      
    if(setVal) {
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & ~TDM_BURST_LENGTH_DIV_MASK;
      reg |= (val << TDM_BURST_LENGTH_DIV_BITS);
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
      printk("TDM: set burst length to %u\n", (tdm->tdmCaps.maxBurstLength / (uint32_t)val));
#endif
    }
  }
  return count;
}

static ssize_t tdm_r_sample_edge(struct class *c, char *buf)
{
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
  return (snprintf(buf, PAGE_SIZE, "%d\n",
           ((XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & TDM_LRCLK_FALLING_EDGE_CH0) != 0)));
}


static ssize_t tdm_w_sample_edge(struct class *c, const char * buf, size_t count)
{
  unsigned long int val;
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);

  if (strict_strtoul(buf, 0, &val) == 0) {
    uint32_t reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
    if (val == LRCLK_FALLING_EDGE_CH0) {
      reg &= ~TDM_LRCLK_FALLING_EDGE_CH0;
    } else {
      reg |= TDM_LRCLK_FALLING_EDGE_CH0;
    }
    XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
    printk("TDM: set LR clock edge to %s\n",
           (val == LRCLK_FALLING_EDGE_CH0 ? "falling" : "rising"));
#endif
  }
  return count;
}

static ssize_t tdm_r_i2s_align(struct class *c, char *buf)
{
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
  return (snprintf(buf, PAGE_SIZE, "%d\n",
          ((XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & TDM_BIT_ALIGNMENT_I2S_DELAYED) != 0)));
}


static ssize_t tdm_w_i2s_align(struct class *c, const char * buf, size_t count)
{
  unsigned long int val;
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);

  if (strict_strtoul(buf, 0, &val) == 0) {
    uint32_t reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
    if (val == BIT_ALIGNMENT_LEFT_JUSTIFIED) {
      reg &= ~TDM_BIT_ALIGNMENT_I2S_DELAYED;
    } else {
      reg |= TDM_BIT_ALIGNMENT_I2S_DELAYED;
    }
    XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
    printk("TDM: set alignment to %s\n",
           (val == BIT_ALIGNMENT_LEFT_JUSTIFIED ? "left justified" : "delayed"));
#endif
  }
  return count;
}


static ssize_t tdm_r_lr_clock_mode(struct class *c, char *buf)
{
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
  return (snprintf(buf, PAGE_SIZE, "%d\n",
           ((XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & TDM_LRCLK_MODE_PULSE) != 0)));
}


static ssize_t tdm_w_lr_clock_mode(struct class *c, const char * buf, size_t count)
{
  unsigned long int val;
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);

  if (strict_strtoul(buf, 0, &val) == 0) {
      uint32_t reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      if (val == LRCLK_MODE_NORMAL) {
        reg &= ~TDM_LRCLK_MODE_PULSE;
      } else {
        reg |= TDM_LRCLK_MODE_PULSE;
      }
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
      printk("TDM: set LR clock mode to %s\n",
             (val == LRCLK_MODE_NORMAL ? "normal" : "pulsed"));
#endif
  }
  return count;
}

static ssize_t tdm_r_sample_rate(struct class *c, char *buf)
{
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
  return (snprintf(buf, PAGE_SIZE, "%d\n",
           ((XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) 
               & TDM_SAMPLE_RATE_MASK) >> TDM_SAMPLE_RATE_BITS)));
}

static ssize_t tdm_w_sample_rate(struct class *c, const char * buf, size_t count)
{
  uint32_t reg;
  int32_t err = 0;
  unsigned long int val;
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);

  if (strict_strtoul(buf, 0, &val) == 0) {
    /* Evaluate the current slot density, the maximum sample rate supported
       scales based on the current slot density */
    if((tdm->opConfig.TdmSlotDensity > (tdm->tdmCaps.maxSlotDensity / 2)) && 
        (val == DOUBLE_SAMPLE_RATE)) {
      err = -ESMPLRATENOTSUPPORTED;
    } else if((tdm->opConfig.TdmSlotDensity > (tdm->tdmCaps.maxSlotDensity / 4)) &&
               (val == QUAD_SAMPLE_RATE)) {
      err = -ESMPLRATENOTSUPPORTED;
    }

    if(!(err < 0)) {
      reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & ~TDM_SAMPLE_RATE_MASK;
      reg |= (val << TDM_SAMPLE_RATE_BITS);
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
      tdm->opConfig.TdmSampleRate = val;
#ifdef _LABXDEBUG
      printk("TDM: sample rate set to %uK\n",
             (val == QUAD_SAMPLE_RATE ? 192 : (val == DOUBLE_SAMPLE_RATE ? 96 : 48)));
#endif
    }
  }
  return (err < 0 ? err : count);
}

static ssize_t tdm_r_sample_depth(struct class *c, char *buf)
{
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
  return (snprintf(buf, PAGE_SIZE, "%d\n",
           ((XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & TDM_SAMPLE_DEPTH_16BIT) != 0)));
}

static ssize_t tdm_w_sample_depth(struct class *c, const char * buf, size_t count)
{
  unsigned long int val;
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);

  if (strict_strtoul(buf, 0, &val) == 0) {
    uint32_t reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
    if (val == SAMPLE_DEPTH_24BIT) {
      reg &= ~TDM_SAMPLE_DEPTH_16BIT;
    } else {
      reg |= TDM_SAMPLE_DEPTH_16BIT;
    }
    XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
    printk("TDM: set sample depth to %u-bit\n",
           (val == SAMPLE_DEPTH_24BIT ? 24 : 16));
#endif
  }
  return count;
}

static ssize_t tdm_r_module_owner(struct class *c, char *buf)
{
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
  return (snprintf(buf, PAGE_SIZE, "%d\n",
           ((XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) 
              & TDM_MODULE_OWNER_MASK) >> TDM_MODULE_OWNER_BITS) == 0x3));
}

static ssize_t tdm_w_module_owner(struct class *c, const char * buf, size_t count)
{
  int32_t err = 0;
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
  unsigned long int val;


  if (strict_strtoul(buf, 0, &val) == 0) {
    // Ensure the slave clock manager is built into the TDM
    if(!tdm->tdmCaps.hasSlaveManager) {
      err = -ESCMNOTIMPL;
    }

    if(!(err < 0)) {
      uint32_t reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      if (val == MASTER_MODE) {
        reg &= ~TDM_MODULE_SLAVE_MODE;
      } else {
        reg |= TDM_MODULE_SLAVE_MODE;
      }
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
    printk("TDM: set module owner to %s\n",
           (val == MASTER_MODE ? "master" : "slave"));
#endif
    }
  }
  return (err < 0 ? err : count);
}

static ssize_t tdm_r_tx_owner(struct class *c, char *buf)
{
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
  return (snprintf(buf, PAGE_SIZE, "%d\n",
           ((XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & TDM_TX_SLAVE_MODE) != 0)));
}

static ssize_t tdm_w_tx_owner(struct class *c, const char * buf, size_t count)
{
  int32_t err = 0;
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
  unsigned long int val;


  if (strict_strtoul(buf, 0, &val) == 0) {
    // Ensure the slave clock manager is built into the TDM
    if(!tdm->tdmCaps.hasSlaveManager) {
      err = -ESCMNOTIMPL;
    }

    if(!(err < 0)) {
      uint32_t reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      if (val == MASTER_MODE) {
        reg &= ~TDM_TX_SLAVE_MODE;
      } else {
        reg |= TDM_TX_SLAVE_MODE;
      }
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg); 
#ifdef _LABXDEBUG
      printk("TDM: tx owner set to %s\n",
             (val == MASTER_MODE ? "master" : "slave"));
#endif
    }
  }
  return (err < 0 ? err : count);
}

static ssize_t tdm_r_rx_owner(struct class *c, char *buf)
{
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
  return (snprintf(buf, PAGE_SIZE, "%d\n",
           ((XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & TDM_RX_SLAVE_MODE) != 0)));
}

static ssize_t tdm_w_rx_owner(struct class *c, const char * buf, size_t count)
{
  int32_t err = 0;
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
  unsigned long int val;

  if (strict_strtoul(buf, 0, &val) == 0) {
    // Ensure the slave clock manager is built into the TDM
    if(!tdm->tdmCaps.hasSlaveManager) {
      err = -ESCMNOTIMPL;
    }
  
    if(!(err < 0)) {
      uint32_t reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
      if (val == MASTER_MODE) {
        reg &= ~TDM_RX_SLAVE_MODE;
      } else {
        reg |= TDM_RX_SLAVE_MODE;
      }
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg); 
#ifdef _LABXDEBUG
      printk("TDM: set rx owner to %s\n",
             (val == MASTER_MODE ? "master" : "slave"));
#endif
    }
  }
  return (err < 0 ? err : count);
}

static ssize_t tdm_r_mclk_divider(struct class *c, char *buf)
{
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
  return (snprintf(buf, PAGE_SIZE, "%d\n",
           ((XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) 
               & TDM_MCLK_DIVIDER_MASK) >> TDM_MCLK_DIVIDER_BITS)));
}

static ssize_t tdm_w_mclk_divider(struct class *c, const char * buf, size_t count)
{
  int32_t err = 0;
  unsigned long int val;
  struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);

  if (strict_strtoul(buf, 0, &val) == 0) {

    // Evaluate the divider to see if is one of the supported configurations
    if(val != 1 && val != 2 && val != 4 && val != 8) {
      err = -EMCLKDNOTSUPPORTED;
    }

    /* Check the frequency of the master to clock, return an
       invalid value if the divider will scale the master clock
       output to below a supported frequency */
    if((tdm->tdmCaps.mclkRatio / val) < 256) {
      err = -EMCLKDTOOHIGH;
    }

    if(!(err < 0)) {
      uint32_t reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & ~TDM_MCLK_DIVIDER_MASK;
      reg |= (val << TDM_MCLK_DIVIDER_BITS);
      XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
#ifdef _LABXDEBUG
      printk("TDM: set mclk divider to %u\n", (uint32_t)val);
#endif
    }
  }
  return (err < 0 ? err : count);
}

static struct class_attribute audio_tdm_class_attrs[] = {
  __ATTR(channels,          S_IRUGO | S_IWUGO, tdm_r_channels,         tdm_w_channels),
  __ATTR(slot_density,      S_IRUGO | S_IWUGO, tdm_r_slot_density,     tdm_w_slot_density),
  __ATTR(burst_length_div,  S_IRUGO | S_IWUGO, tdm_r_burst_length_div, tdm_w_burst_length_div),
  __ATTR(sample_edge,       S_IRUGO | S_IWUGO, tdm_r_sample_edge,      tdm_w_sample_edge),
  __ATTR(i2s_align,         S_IRUGO | S_IWUGO, tdm_r_i2s_align,        tdm_w_i2s_align),
  __ATTR(lr_clock_mode,     S_IRUGO | S_IWUGO, tdm_r_lr_clock_mode,    tdm_w_lr_clock_mode),
  __ATTR(sample_rate,       S_IRUGO | S_IWUGO, tdm_r_sample_rate,      tdm_w_sample_rate),
  __ATTR(sample_depth,      S_IRUGO | S_IWUGO, tdm_r_sample_depth,     tdm_w_sample_depth),
  __ATTR(module_owner,      S_IRUGO | S_IWUGO, tdm_r_module_owner,     tdm_w_module_owner),
  __ATTR(tx_owner,          S_IRUGO | S_IWUGO, tdm_r_tx_owner,         tdm_w_tx_owner),
  __ATTR(rx_owner,          S_IRUGO | S_IWUGO, tdm_r_rx_owner,         tdm_w_rx_owner),
  __ATTR(mclk_divider,      S_IRUGO | S_IWUGO, tdm_r_mclk_divider,     tdm_w_mclk_divider),
  __ATTR_NULL,
};

/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 * This is exported to allow polymorphic drivers to invoke it.
 * @param name - Name of the instance
 * @param pdev - Platform device structure
 * @param addressRange - Resource describing the hardware's I/O range
 * @param irq          - Resource describing the hardware's IRQ
 * @param derivedData  - Pointer for derived driver to make use of
 * @param newInstance  - Pointer to the new driver instance, NULL if unused
 */
int audio_tdm_probe(const char *name,
                    struct platform_device *pdev,
                    struct resource *addressRange,
                    struct resource *irq,
                    void *derivedData,
                    struct audio_tdm **newInstance,
                    struct labx_tdm_platform_data *pdata) {
  struct audio_tdm *tdm;
  unsigned int versionMajor;
  unsigned int versionMinor;
  unsigned int versionCompare;
  int returnValue;
  uint32_t deviceIndex;

  /* Create and populate a device structure */
  tdm = (struct audio_tdm*) kzalloc(sizeof(struct audio_tdm), GFP_KERNEL);
  if(!tdm) return(-ENOMEM);

  tdm->physicalAddress = addressRange->start;
  tdm->addressRangeSize = ((addressRange->end - addressRange->start) + 1);

  snprintf(tdm->name, NAME_MAX_SIZE, "%s%u", name, instanceCount++);
  tdm->name[NAME_MAX_SIZE - 1] = '\0';
  if(request_mem_region(tdm->physicalAddress, tdm->addressRangeSize,
		  tdm->name) == NULL) {
    returnValue = -ENOMEM;
    goto free;
  }

  tdm->virtualAddress =
    (void*) ioremap_nocache(tdm->physicalAddress, tdm->addressRangeSize);
  if(!tdm->virtualAddress) {
    returnValue = -ENOMEM;
    goto release;
  }

  printk("TDM virtualAddress = 0x%08X, phys = 0x%08X, size = 0x%08X\n",
         (uint32_t) tdm->virtualAddress,
         (uint32_t) tdm->physicalAddress,
         (uint32_t) tdm->addressRangeSize);

#ifdef CONFIG_LABX_AUDIO_TDM_ANALYZER
  strcpy(tdm->analyzer.tdmName, tdm->name);
  tdm->analyzer.baseAddress = tdm->virtualAddress + TDM_ANALYZER_BASE_ADDRESS;
  tdm->analyzer.errorIrq    = ANALYSIS_ERROR_IRQ;
  tdm->analyzer.irqFlagsReg = TDM_IRQ_FLAGS_REG;
  tdm->analyzer.irqMaskReg  = TDM_IRQ_MASK_REG;
#endif

  /* Retain the IRQ and register our handler, if an IRQ resource was supplied.
   * For now, there is no TDM IRQ, so this is unused.
   */
  if(irq != NULL) {
    tdm->irq = irq->start;
    returnValue = request_irq(tdm->irq, &labx_audio_tdm_interrupt, IRQF_DISABLED,
    		              tdm->name, tdm);
    if (returnValue) {
      printk(KERN_ERR "%s : Could not allocate Lab X Audio TDM interrupt (%d).\n",
    		  tdm->name, tdm->irq);
      goto unmap;
    }
  } else tdm->irq = NO_IRQ_SUPPLIED;

  /* Read the initial value of the TDM instance and save it. */
  tdm->initialVal = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));

  /* Inspect and check the version to ensure it lies within the range of hardware
   * we support.  For now, there is no TDM version register, so this is hardwired.
   */
#if 0
  tdm->version = XIo_In32(REGISTER_ADDRESS(tdm, TDM_REVISION_REG));
#else
  tdm->version = DRIVER_VERSION_MIN;
#endif
  versionMajor = ((tdm->version >> REVISION_FIELD_BITS) & REVISION_FIELD_MASK);
  versionMinor = (tdm->version & REVISION_FIELD_MASK);
  versionCompare = ((versionMajor << REVISION_FIELD_BITS) | versionMinor);
  if((versionCompare < DRIVER_VERSION_MIN) | 
     (versionCompare > DRIVER_VERSION_MAX)) {
    printk(KERN_INFO "%s: Found incompatible hardware version %u.%u at 0x%08X\n",
    		tdm->name, versionMajor, versionMinor, (uint32_t)tdm->physicalAddress);
    returnValue = -ENXIO;
    goto unmap;
  }
 
  /* Initialize other resources */
  spin_lock_init(&tdm->mutex);
  tdm->opened = false;

  /* Reset the state of the tdm */
  reset_tdm(tdm);

  /* Add as a misc device */
  tdm->miscdev.minor = MISC_DYNAMIC_MINOR;
  tdm->miscdev.name = tdm->name;
  tdm->miscdev.fops = &tdm_fops;
  returnValue = misc_register(&tdm->miscdev);
  if (returnValue) {
    printk(KERN_WARNING DRIVER_NAME ": Unable to register misc device.\n");
    goto unmap;
  }

  /* Provide navigation between the device structures */
  platform_set_drvdata(pdev, tdm);
  tdm->pdev = pdev;
  dev_set_drvdata(tdm->miscdev.this_device, tdm);

  /* Set up the sysfs class interface */
  tdm->tdmclass.name = tdm->name;
  tdm->tdmclass.owner = THIS_MODULE;
  tdm->tdmclass.class_release = NULL;
  tdm->tdmclass.class_attrs = audio_tdm_class_attrs;
  returnValue = class_register(&tdm->tdmclass);

  /* Setup the Config structure */
  tdm->tdmCaps.versionMajor          = versionMajor;
  tdm->tdmCaps.versionMinor          = versionMinor;
  tdm->tdmCaps.laneCount             = pdata->lane_count;
  tdm->tdmCaps.mclkRatio             = pdata->mclk_ratio;
  tdm->tdmCaps.maxSlotDensity        = pdata->slot_density;
  tdm->tdmCaps.maxBurstLength        = pdata->burst_length;
  tdm->tdmCaps.maxNumStreams         = pdata->num_streams;
  tdm->tdmCaps.hasLoopback           = pdata->has_loopback;
  tdm->tdmCaps.hasSlaveManager       = pdata->slave_manager;
#ifdef CONFIG_LABX_AUDIO_TDM_ANALYZER
  tdm->tdmCaps.hasAnalyzer           = pdata->analyzer;
#endif
  tdm->tdmCaps.hasDynamicSampleRates = pdata->has_dynamic_sample_rates;

  /* Announce the device */
  printk(KERN_INFO "%s: Found Lab X Audio TDM v %u.%u at 0x%08X: %d lanes, %d max slots, %d mclk ratio, max burst length %d, %s",
		  tdm->name, versionMajor, versionMinor, (uint32_t)tdm->physicalAddress, tdm->tdmCaps.laneCount,
                  tdm->tdmCaps.maxSlotDensity, tdm->tdmCaps.mclkRatio, tdm->tdmCaps.maxBurstLength,
                  (tdm->tdmCaps.hasSlaveManager ? "has slave capabilities" : "no slave capabilities"));
#ifdef CONFIG_LABX_AUDIO_TDM_ANALYZER
  printk(KERN_INFO "%s\n", (tdm->tdmCaps.hasAnalyzer ? "has analyzer" : "no analyzer"));
#else
  printk(KERN_INFO "\n");
#endif

  /* Locate and occupy the first available device index for future navigation in
   * the call to tdm_open()
   */
  for (deviceIndex = 0; deviceIndex < MAX_INSTANCES; deviceIndex++) {
    if (NULL == devices[deviceIndex]) {
      devices[deviceIndex] = tdm;
      break;
    }
  }

  /* Ensure that we haven't been asked to probe for too many devices */
  if(deviceIndex >= MAX_INSTANCES) {
    printk(KERN_WARNING DRIVER_NAME ": Maximum device count (%d) exceeded during probe\n",
           MAX_INSTANCES);
    goto unmap;
  }

  /* Return success, setting the return pointer if valid */
  if(newInstance != NULL) *newInstance = tdm;
  return(returnValue);

 unmap:
  iounmap(tdm->virtualAddress);
 release:
  release_mem_region(tdm->physicalAddress, tdm->addressRangeSize);
 free:
  kfree(tdm);
  return(returnValue);
}

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

static int audio_tdm_platform_remove(struct platform_device *pdev);

/* Probe for registered devices */
static int __devinit audio_tdm_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
  struct resource r_mem_struct = {};
  struct resource r_irq_struct = {};
  struct resource *addressRange = &r_mem_struct;
  struct resource *irq          = &r_irq_struct;
  struct platform_device *pdev  = to_platform_device(&ofdev->dev);
  const char *full_name = dev_name(&ofdev->dev);
  const char *name;
  int rc = 0;

  struct labx_tdm_platform_data pdata_struct = {};

  struct labx_tdm_platform_data *pdata = &pdata_struct;

  printk(KERN_INFO "Device Tree Probing \'%s\'\n",ofdev->node->name);

  if ((name = strchr(full_name, '.')) == NULL) {
	  name = full_name;
  } else {
	  ++name;
  }
  /* Obtain the resources for this instance */
  rc = of_address_to_resource(ofdev->node, 0, addressRange);
  if(rc) {
    dev_warn(&ofdev->dev, "Invalid address\n");
    return(rc);
  }

  rc = of_irq_to_resource(ofdev->node, 0, irq);
  if(rc == NO_IRQ) {
    /* No IRQ was defined; null the resource pointer to indicate interrupt unused */
    irq = NULL;
  }

  pdata_struct.lane_count               = get_u32(ofdev, "xlnx,tdm-lane-count");
  pdata_struct.num_streams              = get_u32(ofdev, "xlnx,max-num-streams");
  pdata_struct.slot_density             = get_u32(ofdev, "xlnx,tdm-max-slot-density");
  pdata_struct.burst_length             = get_u32(ofdev, "xlnx,tdm-max-burst-length");
  pdata_struct.mclk_ratio               = get_u32(ofdev, "xlnx,mclk-ratio");
  pdata_struct.has_loopback             = get_u32(ofdev, "xlnx,has-loopback");
  pdata_struct.slave_manager            = get_u32(ofdev, "xlnx,has-tdm-slave-manager");
#ifdef CONFIG_LABX_AUDIO_TDM_ANALYZER
  pdata_struct.analyzer                 = get_u32(ofdev, "xlnx,has-tdm-analyzer");
#endif
  pdata_struct.has_dynamic_sample_rates = get_u32(ofdev, "xlnx,has-dynamic-sample-rates");

  /* Dispatch to the generic function */
  return(audio_tdm_probe(name, pdev, addressRange, irq, NULL, NULL, pdata));
}

static int __devexit audio_tdm_of_remove(struct of_device *dev)
{
  struct platform_device *pdev = to_platform_device(&dev->dev);
  audio_tdm_platform_remove(pdev);
  return(0);
}


/* Directly compatible with Lab X Audio TDM peripherals.
 *
 */
static struct of_device_id tdm_of_match[] = {
  { .compatible = "xlnx,labx-tdm-audio-1.01.a", },
  { .compatible = "xlnx,labx-tdm-audio-1.02.a", },
  { .compatible = "xlnx,labx-tdm-audio-1.03.a", },
  { /* end of list */ },
};


static struct of_platform_driver of_audio_tdm_driver = {
  .name		= DRIVER_NAME,
  .match_table	= tdm_of_match,
  .probe   	= audio_tdm_of_probe,
  .remove       = __devexit_p(audio_tdm_of_remove),
};
#endif

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int audio_tdm_platform_probe(struct platform_device *pdev) {
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
  return(audio_tdm_probe(pdev->name, pdev, addressRange, irq, NULL, NULL, NULL));
}

/* This is exported to allow polymorphic drivers to invoke it. */
int audio_tdm_remove(struct audio_tdm *tdm) {
  int deviceIndex;
  
  reset_tdm(tdm);
  iounmap(tdm->virtualAddress);
  release_mem_region(tdm->physicalAddress, tdm->addressRangeSize);
  kfree(tdm);

  misc_deregister(&tdm->miscdev);

  for (deviceIndex = 0; deviceIndex < MAX_INSTANCES; deviceIndex++) {
    if (tdm == devices[deviceIndex]) {
      devices[deviceIndex] = NULL;
      break;
    }
  }
  return(0);
}

static int audio_tdm_platform_remove(struct platform_device *pdev) {
  struct audio_tdm *tdm;

  /* Get a handle to the tdm and begin shutting it down */
  tdm = platform_get_drvdata(pdev);
  if(!tdm) return(-1);
  return(audio_tdm_remove(tdm));
}

/* Platform device driver structure */
static struct platform_driver audio_tdm_driver = {
  .probe  = audio_tdm_platform_probe,
  .remove = audio_tdm_platform_remove,
  .driver = {
  .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __init audio_tdm_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": AVB Audio Time Domain Multiplexer driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Lab X Technologies, LLC\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&of_audio_tdm_driver);
#endif

  /* Initialize the instance counter */ 
  instanceCount = 0;

  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&audio_tdm_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }

  return(0);
}

static void __exit audio_tdm_driver_exit(void)
{
  /* Unregister as a platform device driver */
  platform_driver_unregister(&audio_tdm_driver);
}

module_init(audio_tdm_driver_init);
module_exit(audio_tdm_driver_exit);

MODULE_AUTHOR("Scott Wagner <scott.wagner@labxtechnologies.com>");
MODULE_AUTHOR("Albert M. Hajjar <albert.hajjar@labxtechnologies.com");
MODULE_DESCRIPTION("Lab X Technologies AVB Audio TDM driver");
MODULE_LICENSE("GPL");
