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

#define MAX_DMA_DEVICES 16
static struct labx_local_audio_pdev* devices[MAX_DMA_DEVICES] = {};

static int labx_local_audio_open(struct inode *inode, struct file *file)
{
	int i;
	for (i = 0; i<MAX_DMA_DEVICES; i++)
	{
		//printk("lookup %d = %p, %d (looking for %d)\n", i, devices[i], (devices[i]) ? devices[i]->miscdev.minor : -1, iminor(inode));
		if ((devices[i] != NULL) && (devices[i]->miscdev.minor == iminor(inode)))
		{
			//printk("labx_local_audio_open: found %p\n", devices[i]);
			file->private_data = devices[i];
			break;
		}
	}

	return 0;
}

static int labx_local_audio_ioctl_cdev(struct inode *inode, struct file *filp,
                                    unsigned int command, unsigned long arg)
{
	struct labx_local_audio_pdev *local_audio_pdev = (struct labx_local_audio_pdev*)filp->private_data;

	switch(command) {
		case IOC_LA_SET_CHANNEL_MAPPING:
		{
			struct LocalAudioChannelMapping mapping;

			if(copy_from_user(&mapping, (void __user*)arg, sizeof(mapping)) != 0) {
				return(-EFAULT);
			}
			
			if (mapping.channel >= local_audio_pdev->numChannels) {
				return(-EINVAL);
			}
			
			XIo_Out32(LOCAL_AUDIO_REGISTER_BASE(&local_audio_pdev->dma, LOCAL_AUDIO_CHANNEL_REG + mapping.channel),
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
			
			mapping.streams = XIo_In32(LOCAL_AUDIO_REGISTER_BASE(&local_audio_pdev->dma, LOCAL_AUDIO_CHANNEL_REG + mapping.channel));

			if(copy_to_user(&mapping, (void __user*)arg, sizeof(mapping)) != 0) {
				return(-EFAULT);
			}
		}
		break;

		default:
			return labx_dma_ioctl(&local_audio_pdev->dma, command, arg);
	}

	return 0;
}

static const struct file_operations labx_local_audio_fops = {
	.open = labx_local_audio_open,
	.ioctl = labx_local_audio_ioctl_cdev,
};

#ifdef CONFIG_OF
static int labx_local_audio_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
	struct resource r_mem_struct;
	struct resource *addressRange = &r_mem_struct;
	struct labx_local_audio_pdev *local_audio_pdev;
	int ret;
	int i;
  	struct platform_device *pdev = to_platform_device(&ofdev->dev);

  	printk(KERN_INFO "Device Tree Probing \'%s\' %d (%s)\n", ofdev->node->name, pdev->id, ofdev->node->full_name);
	/* Obtain the resources for this instance */
	ret = of_address_to_resource(ofdev->node, 0, addressRange);
	if (ret) {
		dev_warn(&ofdev->dev, "invalid address\n");
		return ret;
	}

	/* Create and populate a device structure */
	local_audio_pdev = (struct labx_local_audio_pdev*) kzalloc(sizeof(struct labx_local_audio_pdev), GFP_KERNEL);
	if(!local_audio_pdev) return(-ENOMEM);

	/* Request and map the device's I/O memory region into uncacheable space */
	local_audio_pdev->physicalAddress = addressRange->start;
	local_audio_pdev->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
	snprintf(local_audio_pdev->name, NAME_MAX_SIZE, "%s%d", ofdev->node->name, pdev->id);
	local_audio_pdev->name[NAME_MAX_SIZE - 1] = '\0';
	if(request_mem_region(local_audio_pdev->physicalAddress, local_audio_pdev->addressRangeSize,
			local_audio_pdev->name) == NULL) {
		ret = -ENOMEM;
		goto free;
	}
	//printk("DMA Physical %08X\n", local_audio_pdev->physicalAddress);

	local_audio_pdev->dma.virtualAddress = 
		(void*) ioremap_nocache(local_audio_pdev->physicalAddress, local_audio_pdev->addressRangeSize);
	if(!local_audio_pdev->dma.virtualAddress) {
		ret = -ENOMEM;
		goto release;
	}
	//printk("DMA Virtual %p\n", local_audio_pdev->dma.virtualAddress);

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

	labx_dma_probe(&local_audio_pdev->dma);

	for (i=0; i<MAX_DMA_DEVICES; i++)
	{
		if (NULL == devices[i])
		{
			//printk(DRIVER_NAME ": Device %d = %p\n", i, local_audio_pdev);
			devices[i] = local_audio_pdev;
			break;
		}
	}
	return 0;

unmap:
	iounmap(local_audio_pdev->dma.virtualAddress);
release:
	release_mem_region(local_audio_pdev->physicalAddress, local_audio_pdev->addressRangeSize);
free:
	kfree(local_audio_pdev);
	return ret;
}

static int __exit labx_local_audio_pdev_remove(struct platform_device *pdev);

static int __devexit labx_local_audio_of_remove(struct of_device *dev)
{
	struct platform_device *pdev = to_platform_device(&dev->dev);
	labx_local_audio_pdev_remove(pdev);
	return(0);
}

static struct of_device_id labx_local_audio_of_match[] = {
	{ .compatible = "xlnx,labx-local-audio-1.00.a", },
	{ /* end of list */ },
};

static struct of_platform_driver labx_local_audio_of_driver = {
	.name		= DRIVER_NAME,
	.match_table	= labx_local_audio_of_match,
	.probe		= labx_local_audio_of_probe,
	.remove		= __devexit_p(labx_local_audio_of_remove),
};
#endif /* CONFIG_OF */

static int labx_local_audio_pdev_probe(struct platform_device *pdev)
{
	struct resource *addressRange;
	struct labx_local_audio_pdev *local_audio_pdev;
	int ret;
	int i;

	/* Obtain the resources for this instance */
	addressRange = platform_get_resource(pdev, IORESOURCE_MEM, LABX_DMA_ADDRESS_RANGE_RESOURCE);
	if (!addressRange) return(-ENXIO);

	/* Create and populate a device structure */
	local_audio_pdev = (struct labx_local_audio_pdev*) kzalloc(sizeof(struct labx_local_audio_pdev), GFP_KERNEL);
	if(!local_audio_pdev) return(-ENOMEM);

	/* Request and map the device's I/O memory region into uncacheable space */
	local_audio_pdev->physicalAddress = addressRange->start;
	local_audio_pdev->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
	snprintf(local_audio_pdev->name, NAME_MAX_SIZE, "%s%d", pdev->name, pdev->id);
	local_audio_pdev->name[NAME_MAX_SIZE - 1] = '\0';
	if(request_mem_region(local_audio_pdev->physicalAddress, local_audio_pdev->addressRangeSize,
			local_audio_pdev->name) == NULL) {
		ret = -ENOMEM;
		goto free;
	}
	//printk("DMA Physical %08X\n", local_audio_pdev->physicalAddress);

	local_audio_pdev->dma.virtualAddress = 
		(void*) ioremap_nocache(local_audio_pdev->physicalAddress, local_audio_pdev->addressRangeSize);
	if(!local_audio_pdev->dma.virtualAddress) {
		ret = -ENOMEM;
		goto release;
	}
	//printk("DMA Virtual %p\n", local_audio_pdev->dma.virtualAddress);

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

	labx_dma_probe(&local_audio_pdev->dma);

	for (i=0; i<MAX_DMA_DEVICES; i++)
	{
		if (NULL == devices[i])
		{
			//printk(DRIVER_NAME ": Device %d = %p\n", i, local_audio_pdev);
			devices[i] = local_audio_pdev;
			break;
		}
	}
	return 0;

unmap:
	iounmap(local_audio_pdev->dma.virtualAddress);
release:
	release_mem_region(local_audio_pdev->physicalAddress, local_audio_pdev->addressRangeSize);
free:
	kfree(local_audio_pdev);
	return ret;
}

static int __exit labx_local_audio_pdev_remove(struct platform_device *pdev)
{
	int i;
	struct labx_local_audio_pdev *local_audio_pdev = (struct labx_local_audio_pdev*)platform_get_drvdata(pdev);

	/* Make sure the DMA unit is no longer running */
	XIo_Out32(DMA_REGISTER_ADDRESS(&local_audio_pdev->dma, DMA_CONTROL_REG), DMA_DISABLE);

	misc_deregister(&local_audio_pdev->miscdev);

	for (i=0; i<MAX_DMA_DEVICES; i++)
	{
		if (local_audio_pdev == devices[i])
		{
			devices[i] = NULL;
			break;
		}
	}
	return 0;
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

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&labx_local_audio_of_driver);
#endif
 
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
