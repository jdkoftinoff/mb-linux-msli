/*
 *  linux/arch/microblaze/platform/Harman/AVB_EP/avb_ep384_tdm_output.c
 *
 *  Lab X Technologies AVB local audio output derived driver,
 *  adding some Studer-specific extensions
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *
 *  Copyright (C) 2012 Harman, All Rights Reserved.
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

#include "avb_ep384_tdm_output.h"
#include <asm/uaccess.h>
#include <linux/labx_local_audio.h>
#include <linux/labx_local_audio_defs.h>
#include <linux/labx_tdm_analyzer_defs.h>
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

/* Driver name */
#define DRIVER_NAME "avb_ep384_tdm_output"

/* Number of audio channels emitted from memory by this peripheral;
 * this represents the maximum which can be extracted from AVB
 * streams over Gigabit Ethernet; 7 streams of 60 channels each.
 */
#define AVB_EP384_TDM_NUM_CHANNELS  (384)

/* Private register constants and macros */

/* Global control registers */
#define TDM_CONTROL_REG        (0x000)
#  define ANALYZER_ENABLE      (0x80000000)
#  define TDM_DEBUG_PATTERN    (0x40000000)
#  define ANALYZER_RAMP        (0x20000000)
#  define AUTO_MUTE_ACTIVE     (0x10000000)
#  define ANALYZER_SLOT_MASK   (0x01F)
#  define ANALYZER_LANE_MASK   (0x1E0)
#  define ANALYZER_LANE_SHIFT      (5)

#define TDM_ERROR_COUNT_REG    (0x001)
#define TDM_ERROR_PREDICT_REG  (0x002)
#define TDM_ERROR_ACTUAL_REG   (0x003)

#define TDM_IRQ_MASK_REG       (0x004)
#define TDM_IRQ_FLAGS_REG      (0x005)
#  define DMA_ERROR_IRQ       (0x001)
#  define ANALYSIS_ERROR_IRQ  (0x002)

#define TDM_STREAM_MAP_REG     (0x006)
#  define MAP_SWAP_BANK      (0x80000000)
#  define MAP_CHANNEL_MASK   (0x0001FF00)
#  define MAP_CHANNEL_SHIFT  (8)
#  define MAP_STREAM_VALID   (0x00000080)
#  define MAP_STREAM_MASK    (0x0000003F)

#define STREAM_MAP_BANKS  (2)

/* Locates a register within the TDM demultiplexer logic, using the
 * hardware-configured region shift detected by the DMA driver.
 */
#define TDM_DEMUX_RANGE  (0x01)
#define TDM_DEMUX_ADDRESS(device, offset)                    \
  ((uintptr_t)device->labxLocalAudio->virtualAddress |  \
   (TDM_DEMUX_RANGE << 11) | (offset << 2))

/* Buffer for storing stream map entries */
static StreamMapEntry mapEntryBuffer[MAX_MAP_ENTRIES];

/* Configures the pseudorandom analyzer */
static void configure_analyzer(struct avb_ep384_tdm_output *tdmOutput,
                               AnalyzerConfig *analyzerConfig) {
  uint32_t controlRegister;
  uint32_t irqMask;

  controlRegister = XIo_In32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_CONTROL_REG));
  irqMask = XIo_In32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_IRQ_MASK_REG));
  if(analyzerConfig->enable == LFSR_ANALYZER_ENABLE) {
    /* Enable the analyzer on the appropriate channel */
    controlRegister &= ~(ANALYZER_LANE_MASK | ANALYZER_SLOT_MASK);
    controlRegister |= ((analyzerConfig->tdmLane << ANALYZER_LANE_SHIFT) &
                        ANALYZER_LANE_MASK);
    controlRegister |= (analyzerConfig->tdmChannel & ANALYZER_SLOT_MASK);
    controlRegister |= ANALYZER_ENABLE;

    /* Set up the analyzer to predict either a pseudorandom or linear ramp */
    if(analyzerConfig->signalControl == ANALYSIS_PSEUDORANDOM) {
      controlRegister &= ~ANALYZER_RAMP;
    } else controlRegister |= ANALYZER_RAMP;

    /* Enable the analysis error interrupt as a "one-shot" */
    irqMask |= ANALYSIS_ERROR_IRQ;
    XIo_Out32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_IRQ_FLAGS_REG), ANALYSIS_ERROR_IRQ);
  } else {
    /* Just disable the analyzer and its IRQ */
    irqMask &= ~ANALYSIS_ERROR_IRQ;
    controlRegister &= ~ANALYZER_ENABLE;
  }
  XIo_Out32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_IRQ_MASK_REG), irqMask);
  XIo_Out32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_CONTROL_REG), controlRegister);
}

static void get_analyzer_results(struct avb_ep384_tdm_output *tdmOutput,
                                 AnalyzerResults *analyzerResults) {
  /* Fetch the most recent snapshot of analysis results */
  analyzerResults->errorCount = XIo_In32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_ERROR_COUNT_REG));
  analyzerResults->predictedSample = XIo_In32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_ERROR_PREDICT_REG));
  analyzerResults->actualSample = XIo_In32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_ERROR_ACTUAL_REG));
}

static void configure_auto_mute(struct avb_ep384_tdm_output *tdmOutput,
                                AutoMuteConfig *autoMuteConfig) {
  uint32_t controlRegister;
  uint32_t bankIndex;
  uint32_t entryIndex;
  uint32_t entryWord;

  /* First look at the enable setting and apply it */
  controlRegister = XIo_In32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_CONTROL_REG));
  if(autoMuteConfig->enable == AUTO_MUTE_ENABLE) {
    controlRegister |= AUTO_MUTE_ACTIVE;
  } else controlRegister &= ~AUTO_MUTE_ACTIVE;
  XIo_Out32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_CONTROL_REG), controlRegister);

  /* Set any new stream map entries in both banks */
  for(bankIndex = 0; bankIndex < STREAM_MAP_BANKS; bankIndex++) {
    for(entryIndex = 0; entryIndex < autoMuteConfig->numMapEntries; entryIndex++) {
      StreamMapEntry *entryPtr = &(autoMuteConfig->mapEntries[entryIndex]);

      /* Each entry consists of a map from a TDM channel to its stream */
      entryWord = ((entryPtr->tdmChannel << MAP_CHANNEL_SHIFT) & MAP_CHANNEL_MASK);
      if(entryPtr->avbStream != AVB_STREAM_NONE) {
        entryWord |= (entryPtr->avbStream & MAP_STREAM_MASK);
        entryWord |= MAP_STREAM_VALID;
      }
      XIo_Out32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_STREAM_MAP_REG), entryWord);
    }

    /* The same register address is overloaded for the bank swap operation;
     * simply hit the appropriate control bit and the map load function is suppressed
     */
    XIo_Out32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_STREAM_MAP_REG), MAP_SWAP_BANK);
  }
}

static void reset_avb_ep384_tdm(struct avb_ep384_tdm_output *tdmOutput) {
  AnalyzerConfig analyzerConfig;
  uint32_t bankIndex;
  uint32_t channelIndex;

  /* Disable the pseudorandom analyzer */
  analyzerConfig.enable = LFSR_ANALYZER_DISABLE;
  configure_analyzer(tdmOutput, &analyzerConfig);

  /* Clear any old assignments from the auto-mute stream map; all channels
   * are presumed to have no valid stream assignment.
   */
  for(bankIndex = 0; bankIndex < STREAM_MAP_BANKS; bankIndex++) {
    for(channelIndex = 0; channelIndex < AVB_EP384_TDM_NUM_CHANNELS; channelIndex++) {
      XIo_Out32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_STREAM_MAP_REG),
                (channelIndex << MAP_CHANNEL_SHIFT));
    }
    XIo_Out32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_STREAM_MAP_REG), MAP_SWAP_BANK);
  }
}

/*
 * Character device hook functions
 */

static int avb_ep384_tdm_open(struct inode *inode, struct file *filp)
{
  int returnValue = 0;
  struct labx_local_audio_pdev *labxLocalAudio;
  struct avb_ep384_tdm_output *tdmOutput;

  labxLocalAudio = (struct labx_local_audio_pdev *) filp->private_data;
  tdmOutput = (struct avb_ep384_tdm_output *) labxLocalAudio->derivedData;

  /* Ensure the hardware is reset */
  reset_avb_ep384_tdm(tdmOutput);

  return(returnValue);
}

/* I/O control operations for the driver */
static int avb_ep384_tdm_ioctl(struct inode *inode, struct file *filp,
                              unsigned int command, unsigned long arg) {
  int returnValue = 0;
  struct labx_local_audio_pdev *labxLocalAudio;
  struct avb_ep384_tdm_output *tdmOutput;

  labxLocalAudio = (struct labx_local_audio_pdev *) filp->private_data;
  tdmOutput = (struct avb_ep384_tdm_output *) labxLocalAudio->derivedData;

  switch(command) {
  case IOC_CONFIG_ANALYZER:
    {
      AnalyzerConfig analyzerConfig;

      if(copy_from_user(&analyzerConfig, (void __user*)arg, sizeof(analyzerConfig)) != 0) {
        return(-EFAULT);
      }
      configure_analyzer(tdmOutput, &analyzerConfig);
    }
    break;

  case IOC_GET_ANALYZER_RESULTS:
    {
      AnalyzerResults analyzerResults;

      get_analyzer_results(tdmOutput, &analyzerResults);
      if(copy_to_user((void __user*)arg, &analyzerResults, sizeof(analyzerResults)) != 0) {
        return(-EFAULT);
      }
    }
    break;

  case IOC_ARM_ERROR_IRQS:
    {
      /* Enable the DMA error interrupt as a "one-shot" */
      uint32_t irqMask = XIo_In32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_IRQ_MASK_REG));
      irqMask |= DMA_ERROR_IRQ;
      XIo_Out32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_IRQ_FLAGS_REG), DMA_ERROR_IRQ);
      XIo_Out32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_IRQ_MASK_REG), irqMask);
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
      configure_auto_mute(tdmOutput, &autoMuteConfig);
    }
    break;

  default:
    /* We are only invoked from the parent driver if it doesn't recognize
     * an ioctl(); anything we don't recognize is invalid.
     */
    return(-EINVAL);
  }

  /* Return an error code appropriate to the command */
  return(returnValue);
}

/* Character device file operations structure */
static struct file_operations avb_ep384_tdm_fops = {
  .open	   = avb_ep384_tdm_open,
  .ioctl   = avb_ep384_tdm_ioctl,
  .owner   = THIS_MODULE,
};

/* Interrupt service routine for the driver */
static irqreturn_t avb_ep384_tdm_interrupt(int irq, void *dev_id) {
  struct avb_ep384_tdm_output *tdmOutput = (struct avb_ep384_tdm_output*) dev_id;
  uint32_t maskedFlags;
  uint32_t irqMask;

  /* Read the interrupt flags and immediately clear them */
  maskedFlags = XIo_In32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_IRQ_FLAGS_REG));
  irqMask = XIo_In32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_IRQ_MASK_REG));
  maskedFlags &= irqMask;
  XIo_Out32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_IRQ_FLAGS_REG), maskedFlags);

  /* Detect the pseudorandom analysis error IRQ */
  if((maskedFlags & ANALYSIS_ERROR_IRQ) != 0) {
    /* TEMPORARY - Just announce this and treat it as a one-shot.
     *             Ultimately this should be communicated via generic Netlink.
     */
    irqMask &= ~ANALYSIS_ERROR_IRQ;
    XIo_Out32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_IRQ_MASK_REG), irqMask);
    printk("%s: Analysis error!\n", tdmOutput->labxLocalAudio->name);
  }

  /* Detect the pseudorandom analysis error IRQ */
  if((maskedFlags & DMA_ERROR_IRQ) != 0) {
    /* TEMPORARY - Just announce this and treat it as a one-shot.
     *             Ultimately this should be communicated via generic Netlink.
     */
    irqMask &= ~DMA_ERROR_IRQ;
    XIo_Out32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_IRQ_MASK_REG), irqMask);
    printk("%s: DMA error!\n", tdmOutput->labxLocalAudio->name);
  }

  return(IRQ_HANDLED);
}

/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 * This is exported to allow polymorphic drivers to invoke it.
 * @param name - Name of the instance
 * @param pdev - Platform device structure
 * @param addressRange - Resource describing the hardware's I/O 
 * @param irq          - Resource describing the hardware's interrupt
 */
int avb_ep384_tdm_probe(const char *name, 
                       struct platform_device *pdev,
                       struct resource *addressRange,
                       struct resource *irq) {
  struct avb_ep384_tdm_output *tdmOutput;
  int returnValue;

  /* Create and populate a device structure */
  tdmOutput = (struct avb_ep384_tdm_output*) kmalloc(sizeof(struct avb_ep384_tdm_output), GFP_KERNEL);
  if(!tdmOutput) return(-ENOMEM);

  /* Dispatch to the Lab X local audio driver for most of the setup.
   * We pass it our file operations structure to be invoked polymorphically.
   */
  returnValue = labx_local_audio_probe(name, 
                                       pdev, 
                                       addressRange,
                                       LA_DMA_INTERFACE_MCB,
                                       AVB_EP384_TDM_NUM_CHANNELS,
                                       &avb_ep384_tdm_fops, 
                                       tdmOutput,
                                       &tdmOutput->labxLocalAudio);
  if(returnValue != 0) goto free;

  /* Announce the device */
  printk(KERN_INFO "%s: Found AVB EP384 TDM Output at 0x%08X\n", name,
	 (uint32_t)tdmOutput->labxLocalAudio->physicalAddress);

  /* Retain the IRQ and register our handler */
  if(irq == NULL) {
    printk(KERN_ERR "%s: No IRQ supplied by platform!", name);
    goto remove;
  }
  tdmOutput->irq = irq->start;
  returnValue = request_irq(tdmOutput->irq, &avb_ep384_tdm_interrupt, 
                            IRQF_DISABLED, name, tdmOutput);
  if(returnValue) {
    printk(KERN_ERR "%s : Could not allocate AVB EP384 TDM Output interrupt (%d).\n",
           name, tdmOutput->irq);
    goto remove;
  }

  /* Return success */
  return(0);

 remove:
  labx_local_audio_remove(tdmOutput->labxLocalAudio);

 free:
  kfree(tdmOutput);
  return(returnValue);
}

#ifdef CONFIG_OF
static int avb_ep384_tdm_platform_remove(struct platform_device *pdev);

/* Probe for registered devices */
static int __devinit avb_ep384_tdm_of_probe(struct of_device *ofdev, const struct of_device_id *match)
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
  return(avb_ep384_tdm_probe(name, pdev, addressRange, irq));
}

static int __devexit avb_ep384_tdm_of_remove(struct of_device *dev)
{
  struct platform_device *pdev = to_platform_device(&dev->dev);
  avb_ep384_tdm_platform_remove(pdev);
  return(0);
}

/* Define the devices from the tree we are compatible with */
static struct of_device_id avb_ep384_tdm_of_match[] = {
  { .compatible = "xlnx,avb-ep-tdm-output-1.00.a", },
  { .compatible = "xlnx,avb-ep-tdm-output-1.01.a", },
  { /* end of list */ },
};


static struct of_platform_driver of_avb_ep384_tdm_driver = {
  .name        = DRIVER_NAME,
  .match_table = avb_ep384_tdm_of_match,
  .probe       = avb_ep384_tdm_of_probe,
  .remove      = __devexit_p(avb_ep384_tdm_of_remove),
};
#endif

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int avb_ep384_tdm_platform_probe(struct platform_device *pdev) {
  struct resource *addressRange;
  struct resource *irq;

  /* Obtain the resources for this instance */
  addressRange = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!addressRange) {
    printk(KERN_ERR "%s: IO resource address not found.\n", pdev->name);
    return(-ENXIO);
  }
  irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

  /* Dispatch to the generic function */
  return(avb_ep384_tdm_probe(pdev->name, pdev, addressRange, irq));
}

static int avb_ep384_tdm_platform_remove(struct platform_device *pdev) {
  struct avb_ep384_tdm_output *tdmOutput;
  int returnValue = 0;

  /* Get a handle to the TDM and begin shutting it down */
  tdmOutput = platform_get_drvdata(pdev);
  if(!tdmOutput) return(-1);
  if(tdmOutput->labxLocalAudio) {
    returnValue = labx_local_audio_remove(tdmOutput->labxLocalAudio);
  } else returnValue = -1;

  /* Reset our portion of the device and free our structure */
  reset_avb_ep384_tdm(tdmOutput);
  kfree(tdmOutput);
  return(returnValue);
}

/* Platform device driver structure */
static struct platform_driver avb_ep384_tdm_driver = {
  .probe  = avb_ep384_tdm_platform_probe,
  .remove = avb_ep384_tdm_platform_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __devinit avb_ep384_tdm_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": AVB EP384 Audio TDM Output driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Harman\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&of_avb_ep384_tdm_driver);
#endif
 
  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&avb_ep384_tdm_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }

  return(0);
}

static void __devexit avb_ep384_tdm_driver_exit(void)
{
  /* Unregister as a platform device driver */
  platform_driver_unregister(&avb_ep384_tdm_driver);
}

module_init(avb_ep384_tdm_driver_init);
module_exit(avb_ep384_tdm_driver_exit);

MODULE_AUTHOR("Eldridge M. Mount IV <eldridge.mount@labxtechnologies.com>");
MODULE_DESCRIPTION("AVB EP384 TDM Output driver");
MODULE_LICENSE("GPL");
