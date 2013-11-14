/*
 *  linux/drivers/char/labx_tcp_cdev.c
 *
 *  Lab X Technologies TCP Accelerator
 *
 *  Written by Chris Wulff (chris.wulff@labxtechnologies.com)
 *  Written by Tom Bottom  ( tom.bottom@labxtechnologies.com)
 *
 *  Copyright (C) 2013 Lab X Technologies LLC, All Rights Reserved.
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
#include <linux/platform_device.h>
#include <xio.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF


#define DRIVER_NAME "labx_tcp"

#define MAX_TCP_DEVICES 16
static uint32_t instanceCount;
static struct labx_tcp_pdev* devices[MAX_TCP_DEVICES] = {};

static int labx_tcp_open_cdev(struct inode *inode, struct file *filp) {
  int i;
  struct labx_tcp_pdev *tcp_pdev = NULL;

  for (i = 0; i<MAX_TCP_DEVICES; i++) {
    /* printk("lookup %d = %p, %d (looking for %d)\n", i, devices[i], (devices[i]) ? devices[i]->miscdev.minor : -1, iminor(inode));*/
    if ((devices[i] != NULL) && (devices[i]->miscdev.minor == iminor(inode)))
      {
        /* printk("labx_tcp_open: found %p\n", devices[i]);*/
        tcp_pdev = devices[i];
        filp->private_data = tcp_pdev;
        break;
      }
  }

  if(tcp_pdev == NULL) return(-1);

  /* Inform the encapsulated tcp driver that it is being opened */
  return(labx_tcp_open(&tcp_pdev->tcp));
}

static int labx_tcp_release_cdev(struct inode *inode, struct file *filp) {
	struct labx_tcp_pdev *tcp_pdev = (struct labx_tcp_pdev*)filp->private_data;

	/* Simply let our tcp know it's closing */
	return(labx_tcp_release(&tcp_pdev->tcp));
}

static int labx_tcp_ioctl_cdev(struct inode *inode,
                               struct file *filp,
                               unsigned int command, unsigned long arg) {
	struct labx_tcp_pdev *tcp_pdev = (struct labx_tcp_pdev*)filp->private_data;

	return labx_tcp_ioctl(&tcp_pdev->tcp, command, arg);
}

static const struct file_operations labx_tcp_fops = {
	.open    = labx_tcp_open_cdev,
	.release = labx_tcp_release_cdev,
	.ioctl   = labx_tcp_ioctl_cdev,
};

#ifdef CONFIG_OF
static int labx_tcp_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
	struct resource r_mem_struct = {};
	struct resource r_irq_struct = {};
	struct resource *addressRange = &r_mem_struct;
	struct resource *irq          = &r_irq_struct;
	struct labx_tcp_pdev *tcp_pdev;
	int32_t irqParam;
	int32_t *int32Ptr;
	int32_t microcodeWords;
	int ret;
	int i;
  	struct platform_device *pdev = to_platform_device(&ofdev->dev);

  	printk(KERN_INFO "TCP Accelerator Probing \'%s\' %d (%s)\n", ofdev->node->name, pdev->id, ofdev->node->full_name);
	/* Obtain the resources for this instance */
	ret = of_address_to_resource(ofdev->node, 0, addressRange);
	if (ret) {
		dev_warn(&ofdev->dev, "invalid address\n");
		return ret;
	}

	/* Create and populate a device structure */
	tcp_pdev = (struct labx_tcp_pdev*) kzalloc(sizeof(struct labx_tcp_pdev), GFP_KERNEL);
	if(!tcp_pdev) return(-ENOMEM);

	/* Request and map the device's I/O memory region into uncacheable space */
	tcp_pdev->physicalAddress = addressRange->start;
	tcp_pdev->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
	snprintf(tcp_pdev->name, NAME_MAX_SIZE, "%s%d", ofdev->node->name, instanceCount++);
	tcp_pdev->name[NAME_MAX_SIZE - 1] = '\0';
	if(request_mem_region(tcp_pdev->physicalAddress, tcp_pdev->addressRangeSize,
                          tcp_pdev->name) == NULL) {
		ret = -ENOMEM;
		goto free;
	}
	//printk("tcp Physical %08X\n", tcp_pdev->physicalAddress);

	tcp_pdev->tcp.virtualAddress = 
		(void*) ioremap_nocache(tcp_pdev->physicalAddress, tcp_pdev->addressRangeSize);
	if(!tcp_pdev->tcp.virtualAddress) {
		ret = -ENOMEM;
		goto release;
	}
	//printk("tcp Virtual %p\n", tcp_pdev->tcp.virtualAddress);

    /* Obtain the interrupt request number for the instance */
    ret = of_irq_to_resource(ofdev->node, 0, irq);
    if(ret == NO_IRQ) {
      /* No IRQ was defined; indicate as much */
      irqParam = TCP_NO_IRQ_SUPPLIED;
    } else {
      irqParam = irq->start;
    }

	tcp_pdev->miscdev.minor = MISC_DYNAMIC_MINOR;
	tcp_pdev->miscdev.name = tcp_pdev->name;
	tcp_pdev->miscdev.fops = &labx_tcp_fops;
	ret = misc_register(&tcp_pdev->miscdev);
	if (ret) {
		printk(KERN_WARNING DRIVER_NAME ": Unable to register misc device.\n");
		goto unmap;
	}
	platform_set_drvdata(pdev, tcp_pdev);
	tcp_pdev->pdev = pdev;
	dev_set_drvdata(tcp_pdev->miscdev.this_device, tcp_pdev);

    /* See if the device tree has a valid parameter to tell us our microcode size */
    int32Ptr = (int32_t *) of_get_property(ofdev->node, "xlnx,microcode-words", NULL);
    if(int32Ptr == NULL) {
      /* Allow the tcp driver to infer its microcode size */
      microcodeWords = tcp_UCODE_SIZE_UNKNOWN;
    } else {
      /* Specify the known size */
      microcodeWords = *int32Ptr;
    }
    
    /* Invoke the base device driver's probe function */
	labx_tcp_probe(&tcp_pdev->tcp, 
                   MISC_MAJOR,
                   tcp_pdev->miscdev.minor,
                   tcp_pdev->name, 
                   microcodeWords, 
                   irqParam,
                   NULL);

	for (i=0; i<MAX_TCP_DEVICES; i++) {
      if (NULL == devices[i]) {
        /* printk(DRIVER_NAME ": Device %d = %p\n", i, tcp_pdev);*/
        /* printk(DRIVER_NAME ": Misc minor is %d\n", tcp_pdev->miscdev.minor);*/
        devices[i] = tcp_pdev;
        break;
      }
	}
	return(0);

unmap:
	iounmap(tcp_pdev->tcp.virtualAddress);
release:
	release_mem_region(tcp_pdev->physicalAddress, tcp_pdev->addressRangeSize);
free:
	kfree(tcp_pdev);
	return ret;
}

static int __devexit labx_tcp_pdev_remove(struct platform_device *pdev);

static int __devexit labx_tcp_of_remove(struct of_device *dev)
{
	struct platform_device *pdev = to_platform_device(&dev->dev);
	labx_tcp_pdev_remove(pdev);
	return(0);
}

static struct of_device_id labx_tcp_of_match[] = {
	{ .compatible = "xlnx,labx-tcp-1.00.a", },
	{ .compatible = "xlnx,labx-tcp-1.01.a", },
	{ .compatible = "xlnx,labx-tcp-1.02.a", },
	{ .compatible = "xlnx,labx-tcp-1.03.a", },
	{ .compatible = "xlnx,labx-tcp-1.04.a", },
	{ /* end of list */ },
};

static struct of_platform_driver labx_tcp_of_driver = {
	.name		= DRIVER_NAME,
	.match_table	= labx_tcp_of_match,
	.probe		= labx_tcp_of_probe,
	.remove		= __devexit_p(labx_tcp_of_remove),
};
#endif /* CONFIG_OF */

static int labx_tcp_pdev_probe(struct platform_device *pdev)
{
	struct resource *addressRange;
    struct resource *irq;
	struct labx_tcp_pdev *tcp_pdev;
    int32_t irqParam;
	int ret;
	int i;

	/* Obtain the resources for this instance */
	addressRange = platform_get_resource(pdev, IORESOURCE_MEM, LABX_TCP_ADDRESS_RANGE_RESOURCE);
	if (!addressRange) return(-ENXIO);

	/* Create and populate a device structure */
	tcp_pdev = (struct labx_tcp_pdev*) kzalloc(sizeof(struct labx_tcp_pdev), GFP_KERNEL);
	if(!tcp_pdev) return(-ENOMEM);

    /* Attempt to obtain the IRQ; if none is specified, the resource pointer is
     * NULL, and polling will be used.
     */
    irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if(irq == NULL) {
      irqParam = TCP_NO_IRQ_SUPPLIED;
    } else {
      irqParam = irq->start;
    }

	/* Request and map the device's I/O memory region into uncacheable space */
	tcp_pdev->physicalAddress = addressRange->start;
	tcp_pdev->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
	snprintf(tcp_pdev->name, NAME_MAX_SIZE, "%s.%d", pdev->name, pdev->id);
	tcp_pdev->name[NAME_MAX_SIZE - 1] = '\0';
	if(request_mem_region(tcp_pdev->physicalAddress, 
                          tcp_pdev->addressRangeSize,
                          tcp_pdev->name) == NULL) {
		ret = -ENOMEM;
		goto free;
	}
	//printk("tcp Physical %08X\n", tcp_pdev->physicalAddress);
	instanceCount++;

	tcp_pdev->tcp.virtualAddress = 
		(void*) ioremap_nocache(tcp_pdev->physicalAddress, tcp_pdev->addressRangeSize);
	if(!tcp_pdev->tcp.virtualAddress) {
		ret = -ENOMEM;
		goto release;
	}
	//printk("tcp Virtual %p\n", tcp_pdev->tcp.virtualAddress);

	tcp_pdev->miscdev.minor = MISC_DYNAMIC_MINOR;
	tcp_pdev->miscdev.name = tcp_pdev->name;
	tcp_pdev->miscdev.fops = &labx_tcp_fops;
	ret = misc_register(&tcp_pdev->miscdev);
	if (ret) {
		printk(KERN_WARNING DRIVER_NAME ": Unable to register misc device.\n");
		goto unmap;
	}
	platform_set_drvdata(pdev, tcp_pdev);
	tcp_pdev->pdev = pdev;
	dev_set_drvdata(tcp_pdev->miscdev.this_device, tcp_pdev);

    /* Call the base driver probe function, passing our name and IRQ selection.
     * Since we have no "platform data" structure defined, there is no mechanism
     * for allowing the platform to specify the exact amount of microcode RAM; the
     * tcp driver will assume the entire microcode address space is backed with RAM.
     */
	labx_tcp_probe(&tcp_pdev->tcp, 
                   MISC_MAJOR,
                   tcp_pdev->miscdev.minor,
                   tcp_pdev->name, 
                   TCP_UCODE_SIZE_UNKNOWN, 
                   irqParam,
                   NULL);

	for (i=0; i<MAX_TCP_DEVICES; i++)
	{
		if (NULL == devices[i])
		{
			//printk(DRIVER_NAME ": Device %d = %p\n", i, tcp_pdev);
			devices[i] = tcp_pdev;
			break;
		}
	}
	return 0;

unmap:
	iounmap(tcp_pdev->tcp.virtualAddress);
release:
	release_mem_region(tcp_pdev->physicalAddress, tcp_pdev->addressRangeSize);
free:
	kfree(tcp_pdev);
	return ret;
}

static int __devexit labx_tcp_pdev_remove(struct platform_device *pdev)
{
	int i;
	struct labx_tcp_pdev *tcp_pdev = (struct labx_tcp_pdev*)platform_get_drvdata(pdev);

	/* Make sure the tcp unit is no longer running */
	XIo_Out32(TCP_REGISTER_ADDRESS(&tcp_pdev->tcp, TCP_CONTROL_REG), TCP_DISABLE);

	labx_tcp_remove(&tcp_pdev->tcp);

	misc_deregister(&tcp_pdev->miscdev);

	for (i=0; i<MAX_TCP_DEVICES; i++)
	{
		if (tcp_pdev == devices[i])
		{
			devices[i] = NULL;
			break;
		}
	}
	return 0;
}

/* Platform device driver structure */
static struct platform_driver labx_tcp_platform_driver = {
  .probe  = labx_tcp_pdev_probe,
  .remove = labx_tcp_pdev_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __init labx_tcp_cdev_driver_init(void)
{
  int returnValue;

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&labx_tcp_of_driver);
#endif
 
  instanceCount = 0;
  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&labx_tcp_platform_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }

  return(0);
}

static void __exit labx_tcp_cdev_driver_exit(void)
{
  /* Unregister as a platform device driver */
  platform_driver_unregister(&labx_tcp_platform_driver);
}

module_init(labx_tcp_cdev_driver_init);
module_exit(labx_tcp_cdev_driver_exit);

MODULE_AUTHOR("Chris Wulff");
MODULE_LICENSE("GPL");
