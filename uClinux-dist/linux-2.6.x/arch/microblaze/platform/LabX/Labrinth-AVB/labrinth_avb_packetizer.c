/*
 *  linux/arch/microblaze/platform/LabX/Labrinth-AVB/labrinth_avb_packetizer.h
 *
 *  Lab X Technologies AVB flexible audio packetizer derived driver,
 *  adding some Labrinth-specific extensions
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *
 *  Copyright (C) 2010 Lab X Technologies, LLC, All Rights Reserved.
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

#include "labrinth_avb_packetizer.h"
#include "labx_audio_packetizer.h"
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
#define DRIVER_NAME "labrinth_avb_packetizer"

/* Private register constants and macros */

/* Global control registers */
#define TDM_CONTROL_REG       (0x000)
#  define GENERATOR_ENABLE      (0x80000000)
#  define GENERATOR_MUTE        (0x40000000)
#  define GENERATOR_DC_PATTERN  (0x20000000)
#  define GENERATOR_RAMP        (0x10000000)
#  define GENERATOR_SLOT_MASK   (0x01F)
#  define GENERATOR_LANE_MASK   (0x1E0)
#  define GENERATOR_LANE_SHIFT      (5)

#define TDM_LATENCY_REG       (0x001)
#  define TDM_LATENCY_MASK  (0x0FF)

/* Locates a register within the TDM multiplexer logic, using the
 * hardware-configured region shift detected by the audio packetizer
 * driver.
 */
#define TDM_MUX_RANGE  (0x01 << PACKETIZER_RANGE_BITS)
#define TDM_MUX_ADDRESS(device, offset)                                 \
  ((uintptr_t)device->labxPacketizer->virtualAddress |                  \
   (TDM_MUX_RANGE << device->labxPacketizer->regionShift) | (offset << 2))

/* Configures the psuedorandom generator */
static void configure_generator(struct labrinth_packetizer *packetizer,
                                GeneratorConfig *generatorConfig) {
  uint32_t controlRegister;

  controlRegister = XIo_In32(TDM_MUX_ADDRESS(packetizer, TDM_CONTROL_REG));
  if(generatorConfig->enable == LFSR_GENERATOR_ENABLE) {
    /* Enable the generator on the appropriate channel */
    controlRegister &= ~(GENERATOR_LANE_MASK | GENERATOR_SLOT_MASK);
    controlRegister |= ((generatorConfig->sportPort << GENERATOR_LANE_SHIFT) &
                        GENERATOR_LANE_MASK);
    controlRegister |= (generatorConfig->sportChannel & GENERATOR_SLOT_MASK);
    controlRegister |= GENERATOR_ENABLE;

    /* Test to see whether we are muting the channel or applying the
     * psuedo-random sequence to it
     */
    switch(generatorConfig->signalControl) {
    case SIGNAL_PSUEDORANDOM:
      controlRegister &= ~(GENERATOR_MUTE | GENERATOR_DC_PATTERN | GENERATOR_RAMP);
      break;

    case SIGNAL_RAMP:
      controlRegister &= ~(GENERATOR_MUTE | GENERATOR_DC_PATTERN);
      controlRegister |= GENERATOR_RAMP;
      break;

    case SIGNAL_MUTE:
      controlRegister &= ~(GENERATOR_DC_PATTERN | GENERATOR_RAMP);
      controlRegister |= GENERATOR_MUTE;
      break;

    default:
      /* DC pattern */
      controlRegister &= ~(GENERATOR_MUTE | GENERATOR_RAMP);
      controlRegister |= GENERATOR_DC_PATTERN;
    }
  } else {
    /* Just disable the generator */
    controlRegister &= 
      ~(GENERATOR_ENABLE | GENERATOR_MUTE | GENERATOR_DC_PATTERN | GENERATOR_RAMP);
  }
  XIo_Out32(TDM_MUX_ADDRESS(packetizer, TDM_CONTROL_REG), controlRegister);
}

static void reset_labrinth_packetizer(struct labrinth_packetizer *packetizer) {
  GeneratorConfig generatorConfig;

  /* Disable the psuedorandom generator */
  generatorConfig.enable = LFSR_GENERATOR_DISABLE;
  configure_generator(packetizer, &generatorConfig);
}

/*
 * Character device hook functions
 */

static int labrinth_packetizer_open(struct inode *inode, struct file *filp)
{
  int returnValue = 0;
  struct audio_packetizer *labxPacketizer;
  struct labrinth_packetizer *packetizer;

  labxPacketizer = container_of(inode->i_cdev, struct audio_packetizer, cdev);
  packetizer = (struct labrinth_packetizer *) labxPacketizer->derivedData;

  return(returnValue);
}

static int labrinth_packetizer_release(struct inode *inode, struct file *filp)
{
  struct audio_packetizer *labxPacketizer;
  struct labrinth_packetizer *packetizer;

  labxPacketizer = container_of(inode->i_cdev, struct audio_packetizer, cdev);
  packetizer = (struct labrinth_packetizer *) labxPacketizer->derivedData;
  
  return(0);
}

/* I/O control operations for the driver */
static int labrinth_packetizer_ioctl(struct inode *inode, struct file *filp,
                                     unsigned int command, unsigned long arg)
{
  int returnValue = 0;
  struct audio_packetizer *labxPacketizer;
  struct labrinth_packetizer *packetizer;

  labxPacketizer = container_of(inode->i_cdev, struct audio_packetizer, cdev);
  packetizer = (struct labrinth_packetizer *) labxPacketizer->derivedData;

  switch(command) {
  case IOC_CONFIG_GENERATOR:
    {
      GeneratorConfig generatorConfig;

      if(copy_from_user(&generatorConfig, (void __user*)arg, sizeof(generatorConfig)) != 0) {
        return(-EFAULT);
      }
      configure_generator(packetizer, &generatorConfig);
    }
    break;

  case IOC_GET_LATENCY:
    {
      /* Simply fetch the latency register and return it */
      uint32_t latency = XIo_In32(TDM_MUX_ADDRESS(packetizer, TDM_LATENCY_REG));
      if(copy_to_user((void __user*)arg, &latency, sizeof(uint32_t)) != 0) {
        return(-EFAULT);
      }
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
static struct file_operations labrinth_packetizer_fops = {
  .open	   = labrinth_packetizer_open,
  .release = labrinth_packetizer_release,
  .ioctl   = labrinth_packetizer_ioctl,
  .owner   = THIS_MODULE,
};

/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 * This is exported to allow polymorphic drivers to invoke it.
 * @param name - Name of the instance
 * @param pdev - Platform device structure
 * @param addressRange - Resource describing the hardware's I/O range
 * @param irq          - Resource describing the hardware's IRQ
 * @param derivedFops  - File operations to delegate to, NULL if unused
 * @param derivedData  - Pointer for derived driver to make use of
 * @param newInstance  - Pointer to the new driver instance, NULL if unused
 */
int labrinth_packetizer_probe(const char *name, 
                              struct platform_device *pdev,
                              struct resource *addressRange,
                              struct resource *irq) {
  struct labrinth_packetizer *packetizer;
  int returnValue;

  /* Create and populate a device structure */
  packetizer = (struct labrinth_packetizer*) kmalloc(sizeof(struct labrinth_packetizer), GFP_KERNEL);
  if(!packetizer) return(-ENOMEM);

  /* Dispatch to the Lab X audio packetizer driver for most of the setup.
   * We pass it our file operations structure to be invoked polymorphically.
   */
  returnValue = 
    audio_packetizer_probe(name, pdev, addressRange, irq,
                           &labrinth_packetizer_fops, packetizer,
                           &packetizer->labxPacketizer);
  if(returnValue != 0) goto free;

  /* Announce the device */
  printk(KERN_INFO "%s: Found Labrinth packetizer at 0x%08X\n", name,
         (uint32_t)packetizer->labxPacketizer->physicalAddress);

  /* Return success */
  return(0);

 free:
  kfree(packetizer);
  return(returnValue);
}

#ifdef CONFIG_OF
static int labrinth_packetizer_platform_remove(struct platform_device *pdev);

/* Probe for registered devices */
static int __devinit labrinth_packetizer_of_probe(struct of_device *ofdev, const struct of_device_id *match)
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
    dev_warn(&ofdev->dev, "Invalid address\n");
    return(rc);
  }

  rc = of_irq_to_resource(ofdev->node, 0, irq);
  if(rc == NO_IRQ) {
    /* No IRQ was defined; null the resource pointer to indicate polled mode */
    irq = NULL;
    return(rc);
  }

  /* Dispatch to the generic function */
  return(labrinth_packetizer_probe(name, pdev, addressRange, irq));
}

static int __devexit labrinth_packetizer_of_remove(struct of_device *dev)
{
  struct platform_device *pdev = to_platform_device(&dev->dev);
  labrinth_packetizer_platform_remove(pdev);
  return(0);
}


/* Define the devices from the tree we are compatible with */
static struct of_device_id labrinth_packetizer_of_match[] = {
  { .compatible = "xlnx,labrinth-avb-packetizer-1.00.a", },
  { /* end of list */ },
};


static struct of_platform_driver of_labrinth_packetizer_driver = {
  .name		= DRIVER_NAME,
  .match_table	= labrinth_packetizer_of_match,
  .probe   	= labrinth_packetizer_of_probe,
  .remove       = __devexit_p(labrinth_packetizer_of_remove),
};
#endif

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int labrinth_packetizer_platform_probe(struct platform_device *pdev) {
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
  return(labrinth_packetizer_probe(pdev->name, pdev, addressRange, irq));
}

static int labrinth_packetizer_platform_remove(struct platform_device *pdev) {
  struct labrinth_packetizer *packetizer;
  int returnValue = 0;

  /* Get a handle to the packetizer and begin shutting it down */
  packetizer = platform_get_drvdata(pdev);
  if(!packetizer) return(-1);
  if(packetizer->labxPacketizer) {
    returnValue = audio_packetizer_remove(packetizer->labxPacketizer);
  } else returnValue = -1;

  /* Reset our portion of the device and free our structure */
  reset_labrinth_packetizer(packetizer);
  kfree(packetizer);
  return(returnValue);
}

/* Platform device driver structure */
static struct platform_driver labrinth_packetizer_driver = {
  .probe  = labrinth_packetizer_platform_probe,
  .remove = labrinth_packetizer_platform_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __devinit labrinth_packetizer_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": Labrinth Audio Packetizer driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Lab X Technologies, LLC\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&of_labrinth_packetizer_driver);
#endif
 
  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&labrinth_packetizer_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }

  return(0);
}

static void __devexit labrinth_packetizer_driver_exit(void)
{
  /* Unregister as a platform device driver */
  platform_driver_unregister(&labrinth_packetizer_driver);
}

module_init(labrinth_packetizer_driver_init);
module_exit(labrinth_packetizer_driver_exit);

MODULE_AUTHOR("Eldridge M. Mount IV <eldridge.mount@labxtechnologies.com>");
MODULE_DESCRIPTION("Labrinth AVB Audio Packetizer driver");
MODULE_LICENSE("GPL");
