/*
 *  linux/drivers/char/labx_dma.c
 *
 *  Lab X Technologies DMA coprocessor
 *
 *  Written by Chris Wulff (chris.wulff@labxtechnologies.com)
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
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/labx_dma_coprocessor_defs.h>
#include <linux/labx_dma.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <xio.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF


#define DRIVER_NAME "labx_dma"

#define MAX_DMA_DEVICES 16
static struct labx_dma_pdev* devices[MAX_DMA_DEVICES] = {};

static int labx_dma_open(struct inode *inode, struct file *file)
{
	int i;
	for (i = 0; i<MAX_DMA_DEVICES; i++)
	{
		//printk("lookup %d = %p, %d (looking for %d)\n", i, devices[i], (devices[i]) ? devices[i]->miscdev.minor : -1, iminor(inode));
		if ((devices[i] != NULL) && (devices[i]->miscdev.minor == iminor(inode)))
		{
			//printk("labx_dma_open: found %p\n", devices[i]);
			file->private_data = devices[i];
			break;
		}
	}

	return 0;
}

static int labx_dma_ioctl_cdev(struct inode *inode, struct file *filp,
                                    unsigned int command, unsigned long arg)
{
	struct labx_dma_pdev *dma_pdev = (struct labx_dma_pdev*)filp->private_data;

	return labx_dma_ioctl(&dma_pdev->dma, command, arg);
}

static const struct file_operations labx_dma_fops = {
	.open = labx_dma_open,
	.ioctl = labx_dma_ioctl_cdev,
};

#ifdef CONFIG_OF
static int labx_dma_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
	struct resource r_mem_struct;
	struct resource *addressRange = &r_mem_struct;
	struct labx_dma_pdev *dma_pdev;
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
	dma_pdev = (struct labx_dma_pdev*) kzalloc(sizeof(struct labx_dma_pdev), GFP_KERNEL);
	if(!dma_pdev) return(-ENOMEM);

	/* Request and map the device's I/O memory region into uncacheable space */
	dma_pdev->physicalAddress = addressRange->start;
	dma_pdev->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
	snprintf(dma_pdev->name, NAME_MAX_SIZE, "%s%d", ofdev->node->name, pdev->id);
	dma_pdev->name[NAME_MAX_SIZE - 1] = '\0';
	if(request_mem_region(dma_pdev->physicalAddress, dma_pdev->addressRangeSize,
			dma_pdev->name) == NULL) {
		ret = -ENOMEM;
		goto free;
	}
	//printk("DMA Physical %08X\n", dma_pdev->physicalAddress);

	dma_pdev->dma.virtualAddress = 
		(void*) ioremap_nocache(dma_pdev->physicalAddress, dma_pdev->addressRangeSize);
	if(!dma_pdev->dma.virtualAddress) {
		ret = -ENOMEM;
		goto release;
	}
	//printk("DMA Virtual %p\n", dma_pdev->dma.virtualAddress);

	dma_pdev->miscdev.minor = MISC_DYNAMIC_MINOR;
	dma_pdev->miscdev.name = dma_pdev->name;
	dma_pdev->miscdev.fops = &labx_dma_fops;
	ret = misc_register(&dma_pdev->miscdev);
	if (ret) {
		printk(KERN_WARNING DRIVER_NAME ": Unable to register misc device.\n");
		goto unmap;
	}
	platform_set_drvdata(pdev, dma_pdev);
	dma_pdev->pdev = pdev;
	dev_set_drvdata(dma_pdev->miscdev.this_device, dma_pdev);

	labx_dma_probe(&dma_pdev->dma);

	for (i=0; i<MAX_DMA_DEVICES; i++)
	{
		if (NULL == devices[i])
		{
			//printk(DRIVER_NAME ": Device %d = %p\n", i, dma_pdev);
			devices[i] = dma_pdev;
			break;
		}
	}
	return 0;

unmap:
	iounmap(dma_pdev->dma.virtualAddress);
release:
	release_mem_region(dma_pdev->physicalAddress, dma_pdev->addressRangeSize);
free:
	kfree(dma_pdev);
	return ret;
}

static int __exit labx_dma_pdev_remove(struct platform_device *pdev);

static int __devexit labx_dma_of_remove(struct of_device *dev)
{
	struct platform_device *pdev = to_platform_device(&dev->dev);
	labx_dma_pdev_remove(pdev);
	return(0);
}

static struct of_device_id labx_dma_of_match[] = {
	{ .compatible = "xlnx,labx-local-audio-1.00.a", },
	{ .compatible = "xlnx,labx-dma-1.00.a", },
	{ /* end of list */ },
};

static struct of_platform_driver labx_dma_of_driver = {
	.name		= DRIVER_NAME,
	.match_table	= labx_dma_of_match,
	.probe		= labx_dma_of_probe,
	.remove		= __devexit_p(labx_dma_of_remove),
};
#endif /* CONFIG_OF */

static int labx_dma_pdev_probe(struct platform_device *pdev)
{
	struct resource *addressRange;
	struct labx_dma_pdev *dma_pdev;
	int ret;
	int i;

	/* Obtain the resources for this instance */
	addressRange = platform_get_resource(pdev, IORESOURCE_MEM, LABX_DMA_ADDRESS_RANGE_RESOURCE);
	if (!addressRange) return(-ENXIO);

	/* Create and populate a device structure */
	dma_pdev = (struct labx_dma_pdev*) kzalloc(sizeof(struct labx_dma_pdev), GFP_KERNEL);
	if(!dma_pdev) return(-ENOMEM);

	/* Request and map the device's I/O memory region into uncacheable space */
	dma_pdev->physicalAddress = addressRange->start;
	dma_pdev->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
	snprintf(dma_pdev->name, NAME_MAX_SIZE, "%s%d", pdev->name, pdev->id);
	dma_pdev->name[NAME_MAX_SIZE - 1] = '\0';
	if(request_mem_region(dma_pdev->physicalAddress, dma_pdev->addressRangeSize,
			dma_pdev->name) == NULL) {
		ret = -ENOMEM;
		goto free;
	}
	//printk("DMA Physical %08X\n", dma_pdev->physicalAddress);

	dma_pdev->dma.virtualAddress = 
		(void*) ioremap_nocache(dma_pdev->physicalAddress, dma_pdev->addressRangeSize);
	if(!dma_pdev->dma.virtualAddress) {
		ret = -ENOMEM;
		goto release;
	}
	//printk("DMA Virtual %p\n", dma_pdev->dma.virtualAddress);

	dma_pdev->miscdev.minor = MISC_DYNAMIC_MINOR;
	dma_pdev->miscdev.name = dma_pdev->name;
	dma_pdev->miscdev.fops = &labx_dma_fops;
	ret = misc_register(&dma_pdev->miscdev);
	if (ret) {
		printk(KERN_WARNING DRIVER_NAME ": Unable to register misc device.\n");
		goto unmap;
	}
	platform_set_drvdata(pdev, dma_pdev);
	dma_pdev->pdev = pdev;
	dev_set_drvdata(dma_pdev->miscdev.this_device, dma_pdev);

	labx_dma_probe(&dma_pdev->dma);

	for (i=0; i<MAX_DMA_DEVICES; i++)
	{
		if (NULL == devices[i])
		{
			//printk(DRIVER_NAME ": Device %d = %p\n", i, dma_pdev);
			devices[i] = dma_pdev;
			break;
		}
	}
	return 0;

unmap:
	iounmap(dma_pdev->dma.virtualAddress);
release:
	release_mem_region(dma_pdev->physicalAddress, dma_pdev->addressRangeSize);
free:
	kfree(dma_pdev);
	return ret;
}

static int __exit labx_dma_pdev_remove(struct platform_device *pdev)
{
	int i;
	struct labx_dma_pdev *dma_pdev = (struct labx_dma_pdev*)platform_get_drvdata(pdev);

	/* Make sure the DMA unit is no longer running */
	XIo_Out32(DMA_REGISTER_ADDRESS(&dma_pdev->dma, DMA_CONTROL_REG), DMA_DISABLE);

	misc_deregister(&dma_pdev->miscdev);

	for (i=0; i<MAX_DMA_DEVICES; i++)
	{
		if (dma_pdev == devices[i])
		{
			devices[i] = NULL;
			break;
		}
	}
	return 0;
}

/* Platform device driver structure */
static struct platform_driver labx_dma_platform_driver = {
	.probe  = labx_dma_pdev_probe,
	.remove = labx_dma_pdev_remove,
	.driver = {
		.name = DRIVER_NAME,
	}
};

/* Driver initialization and exit */
static int __init labx_dma_driver_init(void)
{
  int returnValue;

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&labx_dma_of_driver);
#endif
 
  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&labx_dma_platform_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }

  return(0);
}

static void __exit labx_dma_driver_exit(void)
{
  /* Unregister as a platform device driver */
  platform_driver_unregister(&labx_dma_platform_driver);
}

module_init(labx_dma_driver_init);
module_exit(labx_dma_driver_exit);

MODULE_AUTHOR("Chris Wulff");
MODULE_LICENSE("GPL");
