/*
 *  linux/arch/microblaze/platform/Biamp/labrinth_tdm_output.c
 *
 *  Lab X Technologies AVB local audio output derived driver,
 *  adding some Labrinth-specific extensions
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *
 *  Copyright (C) 2010 Biamp Systems, All Rights Reserved.
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

#include "labrinth_tdm_output.h"
#include <asm/uaccess.h>
#include <linux/labx_local_audio.h>
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
#define DRIVER_NAME "labrinth_tdm_output"

/* Number of audio channels emitted from memory by this peripheral;
 * this represents the maximum which can be extracted from AVB
 * streams over Gigabit Ethernet; 60 streams of four channels each.
 */
/* #define LABRINTH_TDM_NUM_CHANNELS  (420) */
/* TEMPORARY lower channel count */
#define LABRINTH_TDM_NUM_CHANNELS  (24)

/* Private register constants and macros */

/* Global control registers */
#define TDM_CONTROL_REG       (0x000)
#  define ANALYZER_ENABLE      (0x80000000)
#  define ANALYZER_SLOT_MASK   (0x01F)
#  define ANALYZER_LANE_MASK   (0x1E0)
#  define ANALYZER_LANE_SHIFT      (5)

/* Locates a register within the TDM demultiplexer logic, using the
 * hardware-configured region shift detected by the audio packetizer
 * driver.
 */
#define TDM_DEMUX_RANGE  (0x03)
#define TDM_DEMUX_ADDRESS(device, offset)                    \
  ((uintptr_t)device->labxLocalAudio->dma.virtualAddress |  \
   (TDM_DEMUX_RANGE << device->labxLocalAudio->dma.regionShift) | (offset << 2))

/* Configures the psuedorandom analyzer */
static void configure_analyzer(struct labrinth_tdm_output *tdmOutput,
			       AnalyzerConfig *analyzerConfig) {
  uint32_t controlRegister;

  controlRegister = XIo_In32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_CONTROL_REG));
  if(analyzerConfig->enable == LFSR_ANALYZER_ENABLE) {
    /* Enable the analyzer on the appropriate channel */
    controlRegister &= ~(ANALYZER_LANE_MASK | ANALYZER_SLOT_MASK);
    controlRegister |= ((analyzerConfig->sportPort << ANALYZER_LANE_SHIFT) &
			ANALYZER_LANE_MASK);
    controlRegister |= (analyzerConfig->sportChannel & ANALYZER_SLOT_MASK);
    controlRegister |= ANALYZER_ENABLE;
  } else {
    /* Just disable the analyzer */
    controlRegister &= ~ANALYZER_ENABLE;
  }
  XIo_Out32(TDM_DEMUX_ADDRESS(tdmOutput, TDM_CONTROL_REG), controlRegister);
}

static void reset_labrinth_tdm(struct labrinth_tdm_output *tdmOutput) {
  AnalyzerConfig analyzerConfig;

  /* Disable the psuedorandom analyzer */
  analyzerConfig.enable = LFSR_ANALYZER_DISABLE;
  configure_analyzer(tdmOutput, &analyzerConfig);
}

/*
 * Character device hook functions
 */

static int labrinth_tdm_open(struct inode *inode, struct file *filp)
{
  int returnValue = 0;
  struct labx_local_audio_pdev *labxLocalAudio;
  struct labrinth_tdm_output *tdmOutput;

  labxLocalAudio = (struct labx_local_audio_pdev *) filp->private_data;
  tdmOutput = (struct labrinth_tdm_output *) labxLocalAudio->derivedData;

  printk("FOO: labrinth_tdm_open()\n");

  return(returnValue);
}

/* I/O control operations for the driver */
static int labrinth_tdm_ioctl(struct inode *inode, struct file *filp,
			      unsigned int command, unsigned long arg) {
  int returnValue = 0;
  struct labx_local_audio_pdev *labxLocalAudio;
  struct labrinth_tdm_output *tdmOutput;

  labxLocalAudio = (struct labx_local_audio_pdev *) filp->private_data;
  tdmOutput = (struct labrinth_tdm_output *) labxLocalAudio->derivedData;

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
static struct file_operations labrinth_tdm_fops = {
  .open	   = labrinth_tdm_open,
  .ioctl   = labrinth_tdm_ioctl,
  .owner   = THIS_MODULE,
};

/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 * This is exported to allow polymorphic drivers to invoke it.
 * @param name - Name of the instance
 * @param pdev - Platform device structure
 * @param addressRange - Resource describing the hardware's I/O range
 * @param derivedFops  - File operations to delegate to, NULL if unused
 * @param derivedData  - Pointer for derived driver to make use of
 * @param newInstance  - Pointer to the new driver instance, NULL if unused
 */
int labrinth_tdm_probe(const char *name, 
		       struct platform_device *pdev,
		       struct resource *addressRange) {
  struct labrinth_tdm_output *tdmOutput;
  int returnValue;

  /* Create and populate a device structure */
  tdmOutput = (struct labrinth_tdm_output*) kmalloc(sizeof(struct labrinth_tdm_output), GFP_KERNEL);
  if(!tdmOutput) return(-ENOMEM);

  printk("FOO: labrinth_tdm_probe()\n");

  /* Dispatch to the Lab X audio tdmOutput driver for most of the setup.
   * We pass it our file operations structure to be invoked polymorphically.
   */
  returnValue = 
    labx_local_audio_probe(name, pdev, addressRange,
			   LABRINTH_TDM_NUM_CHANNELS,
			   &labrinth_tdm_fops, tdmOutput,
			   &tdmOutput->labxLocalAudio);
  if(returnValue != 0) goto free;

  /* Announce the device */
  printk(KERN_INFO "%s: Found Labrinth TDM Output at 0x%08X\n", name,
	 (uint32_t)tdmOutput->labxLocalAudio->physicalAddress);

  /* Return success */
  return(0);

 free:
  kfree(tdmOutput);
  return(returnValue);
}

#ifdef CONFIG_OF
static int labrinth_tdm_platform_remove(struct platform_device *pdev);

/* Probe for registered devices */
static int __devinit labrinth_tdm_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
  struct resource r_mem_struct = {};
  struct resource *addressRange = &r_mem_struct;
  struct platform_device *pdev  = to_platform_device(&ofdev->dev);
  const char *name = ofdev->node->name;
  int rc = 0;

  /* Obtain the resources for this instance */
  rc = of_address_to_resource(ofdev->node, 0, addressRange);
  if(rc) {
    dev_warn(&ofdev->dev, "Invalid address\n");
    return(rc);
  }

  /* Dispatch to the generic function */
  return(labrinth_tdm_probe(name, pdev, addressRange));
}

static int __devexit labrinth_tdm_of_remove(struct of_device *dev)
{
  struct platform_device *pdev = to_platform_device(&dev->dev);
  labrinth_tdm_platform_remove(pdev);
  return(0);
}

/* Define the devices from the tree we are compatible with */
static struct of_device_id labrinth_tdm_of_match[] = {
  { .compatible = "xlnx,labrinth-tdm-output-1.00.a", },
  { /* end of list */ },
};


static struct of_platform_driver of_labrinth_tdm_driver = {
  .name		= DRIVER_NAME,
  .match_table	= labrinth_tdm_of_match,
  .probe   	= labrinth_tdm_of_probe,
  .remove       = __devexit_p(labrinth_tdm_of_remove),
};
#endif

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int labrinth_tdm_platform_probe(struct platform_device *pdev) {
  struct resource *addressRange;

  /* Obtain the resources for this instance */
  addressRange = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!addressRange) {
    printk(KERN_ERR "%s: IO resource address not found.\n", pdev->name);
    return(-ENXIO);
  }

  /* Dispatch to the generic function */
  return(labrinth_tdm_probe(pdev->name, pdev, addressRange));
}

static int labrinth_tdm_platform_remove(struct platform_device *pdev) {
  struct labrinth_tdm_output *tdmOutput;
  int returnValue = 0;

  /* Get a handle to the TDM and begin shutting it down */
  tdmOutput = platform_get_drvdata(pdev);
  if(!tdmOutput) return(-1);
  if(tdmOutput->labxLocalAudio) {
    returnValue = labx_local_audio_remove(tdmOutput->labxLocalAudio);
  } else returnValue = -1;

  /* Reset our portion of the device and free our structure */
  reset_labrinth_tdm(tdmOutput);
  kfree(tdmOutput);
  return(returnValue);
}

/* Platform device driver structure */
static struct platform_driver labrinth_tdm_driver = {
  .probe  = labrinth_tdm_platform_probe,
  .remove = labrinth_tdm_platform_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __init labrinth_tdm_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": Labrinth Audio TDM Output driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Biamp Systems\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&of_labrinth_tdm_driver);
#endif
 
  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&labrinth_tdm_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }

  return(0);
}

static void __exit labrinth_tdm_driver_exit(void)
{
  /* Unregister as a platform device driver */
  platform_driver_unregister(&labrinth_tdm_driver);
}

module_init(labrinth_tdm_driver_init);
module_exit(labrinth_tdm_driver_exit);

MODULE_AUTHOR("Eldridge M. Mount IV <eldridge.mount@labxtechnologies.com>");
MODULE_DESCRIPTION("Labrinth TDM Output driver");
MODULE_LICENSE("GPL");
