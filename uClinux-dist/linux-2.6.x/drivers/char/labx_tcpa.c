/*
 *  linux/drivers/char/labx_tcpa_cdev.c
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
#include <linux/labx_tcpa_defs.h>
#include "labx_tcpa.h"
#include <linux/if_ether.h>
#include <linux/tcp.h>
#include <linux/file.h>
#include <net/route.h>
#include <xio.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF


#define DRIVER_NAME "labx_tcp"

#define MAX_TCPA_DEVICES 16
static uint32_t instanceCount;
static struct labx_tcpa_pdev* devices[MAX_TCPA_DEVICES] = {};

static int labx_tcpa_open_cdev(struct inode *inode, struct file *filp) {
  int i;
  struct labx_tcpa_pdev *tcpa_pdev = NULL;

  for (i = 0; i<MAX_TCPA_DEVICES; i++) {
    /* printk("lookup %d = %p, %d (looking for %d)\n", i, devices[i], (devices[i]) ? devices[i]->miscdev.minor : -1, iminor(inode));*/
    if ((devices[i] != NULL) && (devices[i]->miscdev.minor == iminor(inode)))
      {
        /* printk("labx_tcpa_open: found %p\n", devices[i]);*/
        tcpa_pdev = devices[i];
        filp->private_data = tcpa_pdev;
        break;
      }
  }

  if(tcpa_pdev == NULL) return(-1);

  return 0;
}

static int labx_tcpa_release_cdev(struct inode *inode, struct file *filp) {
	//struct labx_tcpa_pdev *tcpa_pdev = (struct labx_tcpa_pdev*)filp->private_data;

	return 0;
}

static int start_transfer(struct labx_tcpa_pdev* tcpa_pdev, int fd, u32 size) {
	int               err = -EINVAL;
	u8               *src_mac = NULL;
	u8                dst_mac[ETH_ALEN];
	u32               headers[TCPA_TEMPLATE_WORDS]; /* Headers are in little-endian format when written to regs */
	u32               ip_csum = 0;
	u32               tcp_csum = 0x0600; /* Protocol portion of the pseudoheader */
	struct socket    *sock = NULL;
	struct sock      *sk = NULL;
	struct tcp_sock  *tp = NULL;
	struct rtable    *rt = NULL;
	struct neighbour *neigh = NULL;
        int i;

        printk("TCPA: Start Transfer on fd %d, size %08X\n", fd, size);

	/* Get the socket from the file descriptor */
	sock = sockfd_lookup(fd, &err);
	if (!sock) goto out;
	sk = sock->sk;
	tp = tcp_sk(sk);

	if (NULL == tp) goto out;

        printk("TCPA: Got TCP socket: snd_nxt %08X\n", tp->snd_nxt);

	/* Get the route info for the socket */
	rt = (struct rtable*)sk_dst_get(sk);
	if (NULL == rt) goto out;
        printk("TCPA: Got rtable\n");
	if (NULL == rt->u.dst.dev) goto out;

        printk("TCPA: Got Route\n");

	neigh = rt->u.dst.neighbour;
	if (NULL == neigh) goto out;

        printk("TCPA: Got Neighbour\n");

	src_mac = rt->u.dst.dev->dev_addr;
        memcpy(dst_mac, neigh->ha, ETH_ALEN);

	if (NULL == src_mac) goto out;

        printk("TCPA: Got MACS\n");

	/* Ethernet header */
	headers[0] = (dst_mac[1] << 24) | (dst_mac[0] << 16); /* Lower two bytes reserved for internal size */
	headers[1] = (dst_mac[5] << 24) | (dst_mac[4] << 16) | (dst_mac[3] << 8) | dst_mac[2];
	headers[2] = (src_mac[3] << 24) | (src_mac[2] << 16) | (src_mac[1] << 8) | src_mac[0];
	headers[3] = (0x0008 << 16) | (src_mac[5] << 8) | src_mac[4]; /* Ethertype 0x0800 */

	/* IP Header */
	headers[4] = 0x00000045; /* Total length will be filled by gateware */
	headers[5] = 0x00000000; /* ID will be filled by gateware and no fragments/flags */
	headers[6] = (0x06 << 8) | (tp->inet_conn.icsk_inet.uc_ttl & 0xFF); /* Header CSUM to be done by gateware */
	headers[7] = __swab32(rt->rt_src);
	headers[8] = __swab32(rt->rt_dst);

	/* TCP Header */
	headers[9] = (__swab16(tp->inet_conn.icsk_inet.dport) << 16) | __swab16(tp->inet_conn.icsk_inet.sport);
	headers[10] = 0x00000000; /* Sequence number filled in by gateware */
	headers[11] = __swab32(tp->rcv_nxt);
	headers[12] = 0xFFFF1050; /* TODO: Receive window size */
	headers[13] = 0x00000000; /* Checksum filled in by gateware */

	/* Program registers */
	for (i=0; i<TCPA_TEMPLATE_WORDS; i++) {
                printk("TCPA: Header %d = %08X\n", i, headers[i]);
		XIo_Out32(TCPA_TEMPLATE_ADDRESS(tcpa_pdev, i), headers[i]);
		if (i>=4 && i<=8) ip_csum += (headers[i] & 0xffff) + (headers[i] >> 16);
		if (i>=7 && i<=13) tcp_csum += (headers[i] & 0xffff) + (headers[i] >> 16); /* Includes pseudo header IPs */
	}
	ip_csum = (ip_csum + (ip_csum >> 16)) & 0xFFFF;
	tcp_csum = (tcp_csum + (tcp_csum >> 16)) & 0xFFFF;

	XIo_Out32(TCPA_REGISTER_ADDRESS(tcpa_pdev, TCPA_STREAM_WORDS_REG),     (size>>2));
	XIo_Out32(TCPA_REGISTER_ADDRESS(tcpa_pdev, TCPA_INITIAL_CHECKSUM_REG), (ip_csum << 16) | tcp_csum);
	XIo_Out32(TCPA_REGISTER_ADDRESS(tcpa_pdev, TCPA_INITIAL_SEQUENCE_REG), tp->snd_nxt);
	XIo_Out32(TCPA_REGISTER_ADDRESS(tcpa_pdev, TCPA_INITIAL_IP_ID_REG),    tp->inet_conn.icsk_inet.id);
	XIo_Out32(TCPA_REGISTER_ADDRESS(tcpa_pdev, TCPA_TEMPLATE_SIZE_REG),    TCPA_TEMPLATE_WORDS);
	XIo_Out32(TCPA_REGISTER_ADDRESS(tcpa_pdev, TCPA_RETRANSMIT_TICKS_REG), 0x00BEBC20); // TODO - Get the timeout from somewhere

	XIo_Out32(TCPA_REGISTER_ADDRESS(tcpa_pdev, TCPA_CONTROL_REG), TCPA_ENABLE);

	while (TCPA_ENABLE & XIo_In32(TCPA_REGISTER_ADDRESS(tcpa_pdev, TCPA_CONTROL_REG))) {
          schedule();
        }

        /* Move the stack along to the next sequence number */
        tp->snd_nxt += size;

        err = 0;

out:
        if (rt) dst_release(&rt->u.dst);

	if (sock) sockfd_put(sock);

	return err;
}

static int labx_tcpa_ioctl_cdev(struct inode *inode,
                               struct file *filp,
                               unsigned int command, unsigned long arg) {
	struct labx_tcpa_pdev *tcpa_pdev = (struct labx_tcpa_pdev*)filp->private_data;

	/* Switch on the request */
	switch(command) {
		case TCPA_IOC_START_TRANSFER:
		{
			TcpaTransferRequest transferRequest;

			if(copy_from_user(&transferRequest, (void __user*)arg, sizeof(transferRequest)) != 0) {
				return -EFAULT;
			}

			start_transfer(tcpa_pdev, transferRequest.fd, transferRequest.size);
		}
		break;

		default:
			return -EINVAL;
	}

	return 0;
}

static const struct file_operations labx_tcpa_fops = {
	.open    = labx_tcpa_open_cdev,
	.release = labx_tcpa_release_cdev,
	.ioctl   = labx_tcpa_ioctl_cdev,
};

#ifdef CONFIG_OF
static int labx_tcpa_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
	struct resource r_mem_struct = {};
	struct resource r_irq_struct = {};
	struct resource *addressRange = &r_mem_struct;
	struct resource *irq          = &r_irq_struct;
	struct labx_tcpa_pdev *tcpa_pdev;
	int32_t irqParam;
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
	tcpa_pdev = (struct labx_tcpa_pdev*) kzalloc(sizeof(struct labx_tcpa_pdev), GFP_KERNEL);
	if(!tcpa_pdev) return(-ENOMEM);

	/* Request and map the device's I/O memory region into uncacheable space */
	tcpa_pdev->physicalAddress = addressRange->start;
	tcpa_pdev->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
	snprintf(tcpa_pdev->name, NAME_MAX_SIZE, "%s%d", ofdev->node->name, instanceCount++);
	tcpa_pdev->name[NAME_MAX_SIZE - 1] = '\0';
	if(request_mem_region(tcpa_pdev->physicalAddress, tcpa_pdev->addressRangeSize,
                          tcpa_pdev->name) == NULL) {
		ret = -ENOMEM;
		goto free;
	}
	//printk("tcp Physical %08X\n", tcpa_pdev->physicalAddress);

	tcpa_pdev->virtualAddress =
		(void*) ioremap_nocache(tcpa_pdev->physicalAddress, tcpa_pdev->addressRangeSize);
	if(!tcpa_pdev->virtualAddress) {
		ret = -ENOMEM;
		goto release;
	}
	//printk("tcp Virtual %p\n", tcpa_pdev->virtualAddress);

    /* Obtain the interrupt request number for the instance */
    ret = of_irq_to_resource(ofdev->node, 0, irq);
    if(ret == NO_IRQ) {
      /* No IRQ was defined; indicate as much */
      irqParam = TCPA_NO_IRQ_SUPPLIED;
    } else {
      irqParam = irq->start;
    }

	tcpa_pdev->miscdev.minor = MISC_DYNAMIC_MINOR;
	tcpa_pdev->miscdev.name = tcpa_pdev->name;
	tcpa_pdev->miscdev.fops = &labx_tcpa_fops;
	ret = misc_register(&tcpa_pdev->miscdev);
	if (ret) {
		printk(KERN_WARNING DRIVER_NAME ": Unable to register misc device.\n");
		goto unmap;
	}
	platform_set_drvdata(pdev, tcpa_pdev);
	tcpa_pdev->pdev = pdev;
	dev_set_drvdata(tcpa_pdev->miscdev.this_device, tcpa_pdev);

	for (i=0; i<MAX_TCPA_DEVICES; i++) {
      if (NULL == devices[i]) {
        /* printk(DRIVER_NAME ": Device %d = %p\n", i, tcpa_pdev);*/
        /* printk(DRIVER_NAME ": Misc minor is %d\n", tcpa_pdev->miscdev.minor);*/
        devices[i] = tcpa_pdev;
        break;
      }
	}
	return(0);

unmap:
	iounmap(tcpa_pdev->virtualAddress);
release:
	release_mem_region(tcpa_pdev->physicalAddress, tcpa_pdev->addressRangeSize);
free:
	kfree(tcpa_pdev);
	return ret;
}

static int __devexit labx_tcpa_pdev_remove(struct platform_device *pdev);

static int __devexit labx_tcpa_of_remove(struct of_device *dev)
{
	struct platform_device *pdev = to_platform_device(&dev->dev);
	labx_tcpa_pdev_remove(pdev);
	return(0);
}

static struct of_device_id labx_tcpa_of_match[] = {
	{ .compatible = "xlnx,tcp-accelerator-1.00.a", },
	{ /* end of list */ },
};

static struct of_platform_driver labx_tcpa_of_driver = {
	.name		= DRIVER_NAME,
	.match_table	= labx_tcpa_of_match,
	.probe		= labx_tcpa_of_probe,
	.remove		= __devexit_p(labx_tcpa_of_remove),
};
#endif /* CONFIG_OF */

static int labx_tcpa_pdev_probe(struct platform_device *pdev)
{
	struct resource *addressRange;
    struct resource *irq;
	struct labx_tcpa_pdev *tcpa_pdev;
    int32_t irqParam;
	int ret;
	int i;

	/* Obtain the resources for this instance */
	addressRange = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!addressRange) return(-ENXIO);

	/* Create and populate a device structure */
	tcpa_pdev = (struct labx_tcpa_pdev*) kzalloc(sizeof(struct labx_tcpa_pdev), GFP_KERNEL);
	if(!tcpa_pdev) return(-ENOMEM);

    /* Attempt to obtain the IRQ; if none is specified, the resource pointer is
     * NULL, and polling will be used.
     */
    irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if(irq == NULL) {
      irqParam = TCPA_NO_IRQ_SUPPLIED;
    } else {
      irqParam = irq->start;
    }

	/* Request and map the device's I/O memory region into uncacheable space */
	tcpa_pdev->physicalAddress = addressRange->start;
	tcpa_pdev->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
	snprintf(tcpa_pdev->name, NAME_MAX_SIZE, "%s.%d", pdev->name, pdev->id);
	tcpa_pdev->name[NAME_MAX_SIZE - 1] = '\0';
	if(request_mem_region(tcpa_pdev->physicalAddress,
                          tcpa_pdev->addressRangeSize,
                          tcpa_pdev->name) == NULL) {
		ret = -ENOMEM;
		goto free;
	}
	//printk("tcp Physical %08X\n", tcpa_pdev->physicalAddress);
	instanceCount++;

	tcpa_pdev->virtualAddress =
		(void*) ioremap_nocache(tcpa_pdev->physicalAddress, tcpa_pdev->addressRangeSize);
	if(!tcpa_pdev->virtualAddress) {
		ret = -ENOMEM;
		goto release;
	}
	//printk("tcp Virtual %p\n", tcpa_pdev->virtualAddress);

	tcpa_pdev->miscdev.minor = MISC_DYNAMIC_MINOR;
	tcpa_pdev->miscdev.name = tcpa_pdev->name;
	tcpa_pdev->miscdev.fops = &labx_tcpa_fops;
	ret = misc_register(&tcpa_pdev->miscdev);
	if (ret) {
		printk(KERN_WARNING DRIVER_NAME ": Unable to register misc device.\n");
		goto unmap;
	}
	platform_set_drvdata(pdev, tcpa_pdev);
	tcpa_pdev->pdev = pdev;
	dev_set_drvdata(tcpa_pdev->miscdev.this_device, tcpa_pdev);

	for (i=0; i<MAX_TCPA_DEVICES; i++)
	{
		if (NULL == devices[i])
		{
			//printk(DRIVER_NAME ": Device %d = %p\n", i, tcpa_pdev);
			devices[i] = tcpa_pdev;
			break;
		}
	}
	return 0;

unmap:
	iounmap(tcpa_pdev->virtualAddress);
release:
	release_mem_region(tcpa_pdev->physicalAddress, tcpa_pdev->addressRangeSize);
free:
	kfree(tcpa_pdev);
	return ret;
}

static int __devexit labx_tcpa_pdev_remove(struct platform_device *pdev)
{
	int i;
	struct labx_tcpa_pdev *tcpa_pdev = (struct labx_tcpa_pdev*)platform_get_drvdata(pdev);

	/* Make sure the tcp unit is no longer running */
	XIo_Out32(TCPA_REGISTER_ADDRESS(tcpa_pdev, TCPA_CONTROL_REG), TCPA_DISABLE);

	misc_deregister(&tcpa_pdev->miscdev);

	for (i=0; i<MAX_TCPA_DEVICES; i++)
	{
		if (tcpa_pdev == devices[i])
		{
			devices[i] = NULL;
			break;
		}
	}
	return 0;
}

/* Platform device driver structure */
static struct platform_driver labx_tcpa_platform_driver = {
  .probe  = labx_tcpa_pdev_probe,
  .remove = labx_tcpa_pdev_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __init labx_tcpa_cdev_driver_init(void)
{
  int returnValue;

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&labx_tcpa_of_driver);
#endif

  instanceCount = 0;
  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&labx_tcpa_platform_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }

  return(0);
}

static void __exit labx_tcpa_cdev_driver_exit(void)
{
  /* Unregister as a platform device driver */
  platform_driver_unregister(&labx_tcpa_platform_driver);
}

module_init(labx_tcpa_cdev_driver_init);
module_exit(labx_tcpa_cdev_driver_exit);

MODULE_AUTHOR("Chris Wulff");
MODULE_LICENSE("GPL");
