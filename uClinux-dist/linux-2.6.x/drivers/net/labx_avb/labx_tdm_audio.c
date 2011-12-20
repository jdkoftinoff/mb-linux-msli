/*
 *  linux/drivers/net/labx_avb/labx_tdm_audio.c
 *
 *  Lab X Technologies AVB time domain multiplexer (TDM) driver
 *
 *  Written by Scott Wagner (scott.wagner@labxtechnologies.com)
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

#include <asm/io.h>
#include <linux/platform_device.h>
#include <xio.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF

// There's a single register, at offset 0x00000008 (byte address, is actually register 0x02 in 32-bit offset-speak)
//
//           -- Bit [9]   - Bit alignment:
//           --              '0' - Left-justified, or "normal" mode
//           --              '1' - I2S-like mode, MSB of channel 0 lags the LRCK edge by one clock
//           -- Bit [8]   - LRCK polarity:
//           --              '0' - Rising edge signifies channel zero
//           --              '1' - Falling edge signifies channel zero
//           -- Bits[6:0] - Number of audio channels (8, 16, 32 or 64 are the only valid number of input channels)
/* Global control registers */
#define TDM_CONTROL_REG       (0x02)
#  define TDM_BIT_ALIGNMENT_LEFT_JUSTIFIED   (0x0)
#  define TDM_BIT_ALIGNMENT_I2S_DELAYED   (0x200)
#  define TDM_LRCLK_RISING_EDGE_CH0   (0x0)
#  define TDM_LRCLK_FALLING_EDGE_CH0   (0x100)
#  define TDM_NUM_AUDIO_CHANNELS_MASK  (0x7F)

#define REGISTER_ADDRESS(device, offset) \
  ((uintptr_t)device->virtualAddress | (offset << 2))

#define NAME_MAX_SIZE    (256)
#define NO_IRQ_SUPPLIED   (-1)
struct audio_tdm {
  /* Pointer back to the platform device */
  struct platform_device *pdev;
  struct class tdmclass;

  /* Character device data */
//  struct cdev cdev;
//  dev_t       deviceNumber;
//  uint32_t    instanceNumber;

  /* Name for use in identification */
  char name[NAME_MAX_SIZE];
  /* Device version */
  uint32_t version;

  /* Physical and virtual base address */
  uintptr_t      physicalAddress;
  uintptr_t      addressRangeSize;
  void __iomem  *virtualAddress;

  /* Interrupt request number */
  int32_t irq;

  uint32_t initialVal;
};

/* Driver name and the revision range of hardware expected.
 * This driver will work with revision 1.1 only.
 */
#define DRIVER_NAME "labx_tdm_audio"
#define DRIVER_VERSION_MIN  0x11
#define DRIVER_VERSION_MAX  0x11
#define REVISION_FIELD_BITS  4
#define REVISION_FIELD_MASK  (0x0F)



#if 0
#define DBG(f, x...) printk(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

/* Resets the state of the passed instance */
static void reset_tdm(struct audio_tdm *tdm) {

  /* Restore the instance registers to initial values */
  XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), tdm->initialVal);
  return;
}

static ssize_t tdm_r_channels(struct class *c, char *buf)
{
	struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
	return (snprintf(buf, PAGE_SIZE, "%d\n",
			XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & TDM_NUM_AUDIO_CHANNELS_MASK));
}


static ssize_t tdm_w_channels(struct class *c, const char * buf, size_t count)
{
	unsigned long int val;
	struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);

	if (strict_strtoul(buf, 0, &val) == 0) {
		if (val == 8 || val == 16 || val == 32 || val == 64) {
			uint32_t reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & ~TDM_NUM_AUDIO_CHANNELS_MASK;
			reg |= val;
			XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
		}
	}
	return count;
}

static ssize_t tdm_r_lr_polarity(struct class *c, char *buf)
{
	struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);
	return (snprintf(buf, PAGE_SIZE, "%d\n",
			((XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG)) & TDM_LRCLK_FALLING_EDGE_CH0) != 0)));
}


static ssize_t tdm_w_lr_polarity(struct class *c, const char * buf, size_t count)
{
	unsigned long int val;
	struct audio_tdm *tdm = container_of(c, struct audio_tdm, tdmclass);

	if (strict_strtoul(buf, 0, &val) == 0) {
		uint32_t reg = XIo_In32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG));
		if (val == 0) {
			reg &= ~TDM_LRCLK_FALLING_EDGE_CH0;
		} else {
			reg |= TDM_LRCLK_FALLING_EDGE_CH0;
		}
		XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
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
		if (val == 0) {
			reg &= ~TDM_BIT_ALIGNMENT_I2S_DELAYED;
		} else {
			reg |= TDM_BIT_ALIGNMENT_I2S_DELAYED;
		}
		XIo_Out32(REGISTER_ADDRESS(tdm, TDM_CONTROL_REG), reg);
	}
	return count;
}

static struct class_attribute audio_tdm_class_attrs[] = {
	__ATTR(channels, S_IRUGO | S_IWUGO, tdm_r_channels, tdm_w_channels),
	__ATTR(lr_polarity, S_IRUGO | S_IWUGO, tdm_r_lr_polarity, tdm_w_lr_polarity),
	__ATTR(i2s_align, S_IRUGO | S_IWUGO, tdm_r_i2s_align, tdm_w_i2s_align),
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
                    struct audio_tdm **newInstance) {
  struct audio_tdm *tdm;
  unsigned int versionMajor;
  unsigned int versionMinor;
  unsigned int versionCompare;
  int returnValue;

  /* Create and populate a device structure */
  tdm = (struct audio_tdm*) kmalloc(sizeof(struct audio_tdm), GFP_KERNEL);
  if(!tdm) return(-ENOMEM);

  /* Request and map the device's I/O memory region into uncacheable space */
  tdm->physicalAddress = addressRange->start;
  tdm->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
  snprintf(tdm->name, NAME_MAX_SIZE, "%s", name);
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

  /* Retain the IRQ and register our handler, if an IRQ resource was supplied.
   * For now, there is no TDM IRQ, so this is unused.
   */
#if 0
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
#else
  tdm->irq = NO_IRQ_SUPPLIED;
#endif

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


  /* Announce the device */
  printk(KERN_INFO "%s: Found Lab X Audio TDM v %u.%u at 0x%08X, ",
		  tdm->name, versionMajor, versionMinor,
         (uint32_t)tdm->physicalAddress);
#if 0
  if(tdm->irq == NO_IRQ_SUPPLIED) {
    printk("polled operation\n");
  } else {
    printk("IRQ %d\n", tdm->irq);
  }
#else
  printk("No interrupts supported\n");
#endif

  /* Provide navigation between the device structures */
  platform_set_drvdata(pdev, tdm);
  tdm->pdev = pdev;

  /* Reset the state of the tdm */
  reset_tdm(tdm);

  /* Set up the sysfs class interface */
  tdm->tdmclass.name = tdm->name;
  tdm->tdmclass.owner = THIS_MODULE;
  tdm->tdmclass.class_release = NULL;
  tdm->tdmclass.class_attrs = audio_tdm_class_attrs;
  returnValue = class_register(&tdm->tdmclass);

  /* Return success, setting the return pointer if valid */
  if(newInstance != NULL) {
	  *newInstance = tdm;
  }
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
static int audio_tdm_platform_remove(struct platform_device *pdev);

/* Probe for registered devices */
static int __devinit audio_tdm_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
  struct resource r_mem_struct = {};
  struct resource r_irq_struct = {};
  struct resource *addressRange = &r_mem_struct;
  struct resource *irq          = &r_irq_struct;
  struct platform_device *pdev  = to_platform_device(&ofdev->dev);
  const char *name = dev_name(&ofdev->dev);
  int rc = 0;

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

  /* Dispatch to the generic function */
  return(audio_tdm_probe(name, pdev, addressRange, irq, NULL, NULL));
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
  return(audio_tdm_probe(pdev->name, pdev, addressRange, irq, NULL, NULL));
}

/* This is exported to allow polymorphic drivers to invoke it. */
int audio_tdm_remove(struct audio_tdm *tdm) {
  reset_tdm(tdm);
  iounmap(tdm->virtualAddress);
  release_mem_region(tdm->physicalAddress, tdm->addressRangeSize);
  kfree(tdm);
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
MODULE_DESCRIPTION("Lab X Technologies AVB Audio TDM driver");
MODULE_LICENSE("GPL");
