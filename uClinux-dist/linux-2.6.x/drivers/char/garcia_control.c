/*
 * garcia_control.c -- Control device for Avid Garcia sound cards
 *
 * Copyright (C) 2011 Lab X Technologies, LLC
 *	Scott Wagner <scott.wagner@labxtechnologies.com>
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>

#include <linux/garcia_fpga.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF


/* Driver has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/agctlB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular device.
 */
#define AGCTL_MAJOR			101	/* We assume no Motorola DSP 56xxx board */
#define N_AGCTL_MINORS		32	/* ... up to 256 */
#define DRIVER_NAME			"agctl"
#define DRIVER_VERSION		"0.4"
#define AC_BUFSIZE			32		/* Size of shift register buffer */

static unsigned long	minors[N_AGCTL_MINORS / BITS_PER_LONG];


struct agctl_data {
	void __iomem		*regs;	/* virt. address of the control registers */
	uint32_t			irq;
	dev_t				devt;
	spinlock_t			ac_lock;
	struct list_head	device_entry;
	struct completion   done;
	struct fasync_struct *async_queue; /* asynchronous readers */

	/* buffer is NULL unless this device is open (users > 0) */
	uint16_t			users;
	uint16_t			transfer_size;
	uint16_t			strobedelay;
	uint16_t			flags;
	__be32				rxbuf[AC_BUFSIZE>>2]; // Note: guaranteed big-endian
	__be32				txbuf[AC_BUFSIZE>>2]; // Note: guaranteed big-endian
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

#define AC_SR_17_OFFSET		0x00	/* 8-bit Shift Register 17 (s:RO, m:R/W) */
#define AC_SR_16_13_OFFSET	0x04	/* 32-bit Shift Registers 16 - 13 (s:RO, m:R/W) */
#define AC_SR_12_9_OFFSET	0x08	/* 32-bit Shift Registers 12 - 9 (s:RO, m:R/W) */
#define AC_SR_8_5_OFFSET	0x0C	/* 32-bit Shift Registers 8 - 5 (s:RO, m:R/W) */
#define AC_SR_4_1_OFFSET	0x10	/* 32-bit Shift Registers 4 - 1 (s:RO, m:R/W) */
#define AC_IDREG_OFFSET		0x14	/* 8-bit ID Register (also SR 0) (s:R/W, m:RO) */
#define AC_CONTROL_OFFSET	0x18	/* 6-bit Status/Control Register */
#define AC_CLK_COUNT_OFFSET	0x1C	/* 8-bit Clock Count Register (s:RO, m:R/W) */

#define AC_CTL_SLAVE_MODE	BIT(5)	/* Driver is slave to Hub48 (RO) */
#define AC_CTL_RESET_SIG	BIT(4)	/* Reset signal is asserted (s:in, m:out) */
#define AC_CTL_MUTE_SIG 	BIT(3)	/* Mute signal is asserted (s:in, m:out) */
#define AC_CTL_STROBE		BIT(2)	/* Strobe signal is asserted (s:in, m:out) */
#define AC_CTL_INT_REQ		BIT(1)	/* Latched IRQ condition (s:strobe edge,
									 * m:transaction complete)  Write 1 to clear */
#define AC_CTL_INT_ENA		BIT(0)	/* Interrupt enabled */

inline void agc_regw_be(struct agctl_data *agc, uint32_t offs, __be32 val)
{
	out_be32(agc->regs + offs, val);
	#ifdef AGC_DEBUG_REGIO
	printk(KERN_DEBUG "%x=>%p  ", val, agc->regs + offs);
	#endif
}

inline void agc_regw(struct agctl_data *agc, uint32_t offs, uint32_t val)
{
	agc_regw_be(agc, offs, cpu_to_be32(val));
}

inline __be32 agc_regr_be(struct agctl_data *agc, uint32_t offs)
{
	__be32 val = in_be32(agc->regs + offs);
	#ifdef AGC_DEBUG_REGIO
	printk("%x<=%p  ", val, agc->regs + offs);
	#endif
	return val;
}

inline uint32_t agc_regr(struct agctl_data *agc, uint32_t offs)
{
	return be32_to_cpu(agc_regr_be(agc, offs));
}

/*-------------------------------------------------------------------------*/

static void agdev_strobe(struct agctl_data *agctl)
{
	volatile int i;
	uint32_t status;
	spin_lock_irq(&agctl->ac_lock);
	status = agc_regr(agctl, AC_CONTROL_OFFSET);
	if ((status & AC_CTL_SLAVE_MODE) == 0) {
		agc_regw(agctl, AC_CONTROL_OFFSET, status & AC_CTL_STROBE);
		for (i = 0; i < 4; i++) {
			;
		}
		agc_regw(agctl, AC_CONTROL_OFFSET, status);
	}
	spin_unlock_irq(&agctl->ac_lock);
	return;
}

irqreturn_t agc_irq_callback(int irqnum, void *data)
{
	struct agctl_data *agctl = (struct agctl_data *)data;
	uint32_t status = agc_regr(agctl, AC_CONTROL_OFFSET);
	agc_regw(agctl, AC_CONTROL_OFFSET, status);	// Clear interrupt
	/* Be careful to preserve big-endianness of receive buffers */
	/* After this transfer, the byte array representing the shift
	 * register contents will be in rxbuf, with the most recent
	 * (newest) bit as the MSB of the byte at offset 3, and increasing
	 * memory addresses representing older (shifted in first) data.  If
	 * this is an 18 byte master transaction, Byte 0 will contain the
	 * slave's first byte transmitted, which will be the ID byte.
	 */
	agctl->rxbuf[1] = agc_regr_be(agctl, AC_SR_16_13_OFFSET);
	agctl->rxbuf[2] = agc_regr_be(agctl, AC_SR_12_9_OFFSET);
	agctl->rxbuf[3] = agc_regr_be(agctl, AC_SR_8_5_OFFSET);
	agctl->rxbuf[4] = agc_regr_be(agctl, AC_SR_4_1_OFFSET);
	if ((status & AC_CTL_SLAVE_MODE) != 0 ) {
		agctl->rxbuf[0] = agc_regr_be(agctl, AC_SR_17_OFFSET) & 0xFF;
		agctl->transfer_size = (uint16_t) ((0x100 -
				agc_regr(agctl, AC_CLK_COUNT_OFFSET)) & 0xFF);
	} else {	// Put ID register contents in MS byte of rxbuf
		agctl->rxbuf[0] = cpu_to_be32((agc_regr(agctl, AC_SR_17_OFFSET) & 0xFF) |
				((agc_regr(agctl, AC_IDREG_OFFSET) & 0xFF) << 24));
	}
	if (!completion_done(&agctl->done)) {
		complete(&agctl->done);
	}
	/* signal asynchronous readers */
	if (agctl->async_queue) {
		kill_fasync(&agctl->async_queue, SIGIO, POLL_IN);
	}
	return IRQ_HANDLED;
}

static int agctl_open(struct inode *inode, struct file *filp)
{
	struct agctl_data	*agctl;
	int			status = -ENXIO;

	lock_kernel();
	mutex_lock(&device_list_lock);

	list_for_each_entry(agctl, &device_list, device_entry) {
		if (agctl->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		agctl->users++;
		filp->private_data = agctl;
		nonseekable_open(inode, filp);
	} else
		pr_debug("agctl: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	unlock_kernel();
	return status;
}

/* Read-only message with current device setup */
static ssize_t agctl_read(struct file *filp, char __user *buf,
		size_t len, loff_t *f_pos)
{
	struct agctl_data *agctl = filp->private_data;
	int count;
	uint32_t status;
	uint8_t tbuf[AC_BUFSIZE];
	uint8_t *rxbuf;
	uint8_t *tbuf_p;

	if (agctl == NULL) {
		return -ESHUTDOWN;
	}

	if (filp->f_flags & O_NONBLOCK && !completion_done(&agctl->done)) {
		return -EAGAIN;
	}

	if (wait_for_completion_interruptible(&agctl->done)) {
			return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
	}

	spin_lock_irq(&agctl->ac_lock);
	status = agc_regr(agctl, AC_CONTROL_OFFSET);

	count = ((agctl->transfer_size + 7) >> 3) + 2;
	tbuf_p = tbuf;
	rxbuf = (uint8_t *)agctl->rxbuf;
	if (count > 17) {
		if ((status & AC_CTL_SLAVE_MODE) == 0) {
			*tbuf_p++ = rxbuf[0];
		}
		count = 17;
	}
	while (count > 0) {
		*tbuf_p++ = rxbuf[count + 2];
	}
	spin_unlock_irq(&agctl->ac_lock);

	count = tbuf_p - tbuf;
	if (count > len) {
		count = len;
	}
	if (count > 0 && copy_to_user(buf, tbuf, count) != 0) {
		return -EFAULT;
	}

	return count;
}

/* Write-only message with current device setup */
static ssize_t agctl_write(struct file *filp, const char __user *buf,
		size_t len, loff_t *f_pos)
{
	struct agctl_data *agctl = filp->private_data;
	__be32 tbuf[AC_BUFSIZE>>2];
	size_t count = len;
	uint8_t id = 0;
	uint32_t status;

	if (count > AC_BUFSIZE)
		return -EMSGSIZE;
	if (agctl == NULL) {
		return -ESHUTDOWN;
	}
	if (count <= 0) {
		return 0;
	}

	status = agc_regr(agctl, AC_CONTROL_OFFSET);
	if ((status & AC_CTL_SLAVE_MODE) != 0) { // slave - write ID
		if (copy_from_user(&id, buf, 1) != 0) {
			return -EFAULT;
		}
		++buf;
		--count;
	} else { // master - set transfer size
		agctl->transfer_size = count << 3;
	}
	if (count > 0) {
		memset(tbuf, 0, AC_BUFSIZE);
		if (copy_from_user(tbuf, buf, count) != 0) {
			return -EFAULT;
		}
		spin_lock_irq(&agctl->ac_lock);
		/* Be careful of endianness here - tbuf is already big-endian! */
		if (id != 0) {
			agc_regw(agctl, AC_IDREG_OFFSET, id);
		}
		agc_regw_be(agctl, AC_SR_4_1_OFFSET, tbuf[0]);
		agc_regw_be(agctl, AC_SR_8_5_OFFSET, tbuf[1]);
		agc_regw_be(agctl, AC_SR_12_9_OFFSET, tbuf[2]);
		agc_regw_be(agctl, AC_SR_16_13_OFFSET, tbuf[3]);
		agc_regw(agctl, AC_SR_17_OFFSET, (be32_to_cpu(tbuf[3]) >> 3) & 0xFF);
		if ((status & AC_CTL_SLAVE_MODE) == 0) { // master - start transfer
			agctl->transfer_size = count << 3;
			agc_regw(agctl, AC_CLK_COUNT_OFFSET, agctl->transfer_size);
			spin_unlock_irq(&agctl->ac_lock);
			if ((filp->f_flags & O_NONBLOCK) == 0) {
				if (wait_for_completion_interruptible(&agctl->done)) {
					len = -ERESTARTSYS; /* signal: tell the fs layer to handle it */
				} else {
					if (agctl->strobedelay > 0) {
						mdelay(agctl->strobedelay);
					}
					agdev_strobe(agctl);
				}
			}
		} else {
			spin_unlock_irq(&agctl->ac_lock);
		}
	}
	return len;
}

static unsigned int agctl_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct agctl_data	*agctl = filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &agctl->done.wait, wait);
	if (completion_done(&agctl->done))
		mask = POLLIN | POLLRDNORM;	/* readable */
	return mask;
}

static int agctl_fasync(int fd, struct file *filp, int mode)
{
	struct agctl_data	*agctl = filp->private_data;
	return fasync_helper(fd, filp, mode, &agctl->async_queue);
}

static int agctl_release(struct inode *inode, struct file *filp)
{
	struct agctl_data	*agctl;
	int			status = 0;

	mutex_lock(&device_list_lock);
	agctl = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	agctl->users--;
	mutex_unlock(&device_list_lock);

	return status;
}

static struct file_operations agctl_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.open =		agctl_open,
	.write =	agctl_write,
	.read =		agctl_read,
	.poll =     agctl_poll,
	.fasync =	agctl_fasync,
	.release =	agctl_release,
};

/*-------------------------------------------------------------------------*/
static ssize_t agdev_w_reset(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	uint32_t status;
	long int val;
	struct agctl_data *agctl = dev_get_drvdata(dev);
	if (strict_strtol(buf, 0, &val) != 0) {
		return -EINVAL;
	}
	if (agctl != NULL) {
		spin_lock_irq(&agctl->ac_lock);
		status = agc_regr(agctl, AC_CONTROL_OFFSET);
		if (val == 0) {
			status &= ~AC_CTL_RESET_SIG;
		} else {
			status |= AC_CTL_RESET_SIG;
		}
		if ((status & AC_CTL_SLAVE_MODE) == 0) {
			agc_regw(agctl, AC_CONTROL_OFFSET, status);
		}
		spin_unlock_irq(&agctl->ac_lock);
	}
	return count;
}

static ssize_t agdev_r_reset(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	uint32_t rst = 0;
	struct agctl_data *agctl = dev_get_drvdata(dev);
	if (agctl != NULL) {
		rst = ((agc_regr(agctl, AC_CONTROL_OFFSET) & AC_CTL_RESET_SIG) != 0);
	}
	return (snprintf(buf, PAGE_SIZE, "%u\n", rst));
}

static DEVICE_ATTR(reset, S_IRUGO | S_IWUSR, agdev_r_reset, agdev_w_reset);

static ssize_t agdev_w_mute(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	uint32_t status;
	long int val;
	struct agctl_data *agctl = dev_get_drvdata(dev);
	if (strict_strtol(buf, 0, &val) != 0) {
		return -EINVAL;
	}
	if (agctl != NULL) {
		spin_lock_irq(&agctl->ac_lock);
		status = agc_regr(agctl, AC_CONTROL_OFFSET);
		if (val == 0) {
			status &= ~AC_CTL_MUTE_SIG;
		} else {
			status |= AC_CTL_MUTE_SIG;
		}
		if ((status & AC_CTL_SLAVE_MODE) == 0) {
			agc_regw(agctl, AC_CONTROL_OFFSET, status);
		}
		spin_unlock_irq(&agctl->ac_lock);
	}
	return count;
}

static ssize_t agdev_r_mute(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	uint32_t rst = 0;
	struct agctl_data *agctl = dev_get_drvdata(dev);
	if (agctl != NULL) {
		rst = ((agc_regr(agctl, AC_CONTROL_OFFSET) & AC_CTL_MUTE_SIG) != 0);
	}
	return (snprintf(buf, PAGE_SIZE, "%u\n", rst));
}

static DEVICE_ATTR(mute, S_IRUGO | S_IWUSR, agdev_r_mute, agdev_w_mute);

static ssize_t agdev_w_strobe(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	long int val;
	struct agctl_data *agctl = dev_get_drvdata(dev);
	if (strict_strtol(buf, 0, &val) != 0) {
		return -EINVAL;
	}
	if (val < 0) {
		agdev_strobe(agctl);
	} else {
		agctl->strobedelay = (int)val;
	}
	return count;
}

static ssize_t agdev_r_strobe(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct agctl_data *agctl = dev_get_drvdata(dev);
	return (snprintf(buf, PAGE_SIZE, "%u\n", abs(agctl->strobedelay)));
}

static DEVICE_ATTR(strobe, S_IRUGO | S_IWUSR, agdev_r_strobe, agdev_w_strobe);

static ssize_t agctl_r_minors(struct class *c,char *buf)
{
	return(sprintf(buf, "%ld\n", find_first_zero_bit(minors, N_AGCTL_MINORS)));
}

/* This class makes mdev/udev create the
 * /dev/agctl-B.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class_attribute agctl_class_attrs[] = {
	__ATTR(minors, S_IRUGO, agctl_r_minors, NULL),
	__ATTR_NULL,
};

static struct class agctl_class = {
	.name =		DRIVER_NAME,
	.owner =	THIS_MODULE,
	.class_attrs =	agctl_class_attrs,
};

/*-------------------------------------------------------------------------*/

static int garcia_control_probe(const char *name, struct platform_device *pdev,
		void __iomem *address, uint32_t irq, const char *interfaceType)
{
	struct agctl_data *agctl;
	unsigned long minor;
	int status;

	/* Allocate driver data */
	printk("Probe Garcia control type \"%s\", name \"%s\" at address %p IRQ %d",
			interfaceType, name, address, irq);
	agctl = kzalloc(sizeof(*agctl), GFP_KERNEL);
	if (!agctl)
		return -ENOMEM;

	/* Initialize the driver data */
	spin_lock_init(&agctl->ac_lock);

	INIT_LIST_HEAD(&agctl->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_AGCTL_MINORS);
	if (minor < N_AGCTL_MINORS) {
		struct device *dev;

		agctl->devt = MKDEV(AGCTL_MAJOR, minor);
		init_completion(&agctl->done);

		agctl->irq = irq;
		/* Register for interrupt */
		status = request_irq(agctl->irq, agc_irq_callback, 0, name, agctl);
		if (status != 0) {
			dev_warn(&pdev->dev, "irq request failure: %d\n", agctl->irq);
			kfree(agctl);
			return -ENXIO;
		}

		agctl->regs = address;
		dev = device_create(&agctl_class, NULL, agctl->devt,
				    agctl, "agctl-%d.%ld", AGCTL_MAJOR, minor);
		status = device_create_file(dev, &dev_attr_reset);
		status = device_create_file(dev, &dev_attr_mute);
		status = device_create_file(dev, &dev_attr_strobe);
		sysfs_create_link(agctl_class.dev_kobj, &dev->kobj, dev->kobj.name);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&pdev->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&agctl->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status != 0)
		kfree(agctl);

	return status;
}

/* Probe for registered devices */
static int garcia_control_platform_probe(struct platform_device *pdev)
{
	struct resource *r_irq;
	struct resource *r_mem;
	char *interfaceType;
	void __iomem *iom;

	/* Obtain the resources for this instance */
	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_mem) {
		printk(KERN_ERR "%s: IO resource address not found.\n", pdev->name);
		return -ENXIO;
	}
	if (!request_mem_region(r_mem->start,
			r_mem->end - r_mem->start + 1, pdev->name)) {
		dev_warn(&pdev->dev, "memory request failure\n");
		return -ENXIO;
	}

	iom = ioremap(r_mem->start, r_mem->end - r_mem->start + 1);


	/* Attempt to obtain the IRQ; if none is specified, the resource pointer is
	 * NULL, and polling will be used.
	 */
	r_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	/* The only other platform data provided is a string specifying the
	 * interface type for the instance
	 */
	interfaceType = (char *) pdev->dev.platform_data;
	if(interfaceType == NULL) {
		printk(KERN_ERR "%s: No interface type string specified\n", pdev->name);
		return -EFAULT;
	}

	/* Dispatch to the generic function */
	return(garcia_control_probe(pdev->name, pdev, iom, r_irq->start, interfaceType));
}

static int __match_devt(struct device *dev, void *data)
{
	return (dev->devt == *(dev_t *)data);
}

static int garcia_control_platform_remove(struct platform_device *pdev)
{
	struct agctl_data *agctl = platform_get_drvdata(pdev);
	struct device *dev;

 	if (!agctl) {
 		return(-1);
 	}
	dev = class_find_device(&agctl_class, NULL,
			&agctl->devt, __match_devt);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&agctl->device_entry);
	device_remove_file(dev, &dev_attr_reset);
	device_remove_file(dev, &dev_attr_mute);
	device_remove_file(dev, &dev_attr_strobe);
	device_destroy(&agctl_class, agctl->devt);
	clear_bit(MINOR(agctl->devt), minors);
	iounmap(agctl->regs);
	if (agctl->users == 0)
		kfree(agctl);
	mutex_unlock(&device_list_lock);
	return 0;
}

#ifdef CONFIG_OF
static int __devinit garcia_control_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
	int			rc;
	struct resource r_irq_struct;
	struct resource r_mem_struct;
	struct resource *r_irq = &r_irq_struct;
	struct resource *r_mem = &r_mem_struct;
	struct platform_device *pdev = to_platform_device(&ofdev->dev);
	const char *name = dev_name(&ofdev->dev);
	const char *interfaceType;
	void __iomem *iom;

	/* Obtain the resources for this instance */
	rc = of_address_to_resource(ofdev->node, 0, r_mem);
	if (rc) {
		dev_warn(&ofdev->dev,"invalid address\n");
		return rc;
	}
	if (!request_mem_region(r_mem->start,
			r_mem->end - r_mem->start + 1, name)) {
		rc = -ENXIO;
		dev_warn(&ofdev->dev, "memory request failure\n");
	}

	iom = ioremap(r_mem->start, r_mem->end - r_mem->start + 1);

	rc = of_irq_to_resource(ofdev->node, 0, r_irq);
	if (rc) {
		dev_warn(&ofdev->dev,"invalid interrupt\n");
		return rc;
	}

	interfaceType = (char *) of_get_property(ofdev->node, "xlnx,interface-type", NULL);
	if(interfaceType == NULL) {
		dev_warn(&ofdev->dev, "No interface type specified in device tree\n");
		return(-EFAULT);
	}

	/* Dispatch to the generic function */
	return(garcia_control_probe(name, pdev, iom, r_irq->start, interfaceType));
}

static int __devexit garcia_control_of_remove(struct of_device *dev)
{
	struct platform_device *pdev = to_platform_device(&dev->dev);
	garcia_control_platform_remove(pdev);
	return(0);
}

static struct of_device_id garcia_control_of_match[] = {
	{ .compatible = "xlnx,labx-garcia-control-1.00.a", },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, garcia_control_of_match);

static struct of_platform_driver of_garcia_control_driver = {
	.name		= DRIVER_NAME,
	.match_table	= garcia_control_of_match,
	.probe		= garcia_control_of_probe,
	.remove		= __devexit_p(garcia_control_of_remove),
};
#endif // CONFIG_OF

/* Platform device driver structure */
static struct platform_driver garcia_control_driver = {
  .probe  = garcia_control_platform_probe,
  .remove = garcia_control_platform_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/*-------------------------------------------------------------------------*/

static int __init agctl_init(void)
{
	int status;

#ifdef CONFIG_OF
	status = of_register_platform_driver(&of_garcia_control_driver);
#endif

  /* Register as a platform device driver */
	if((status = platform_driver_register(&garcia_control_driver)) < 0) {
		printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
		return status;
	}

	/* Claim our reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_AGCTL_MINORS > 256);
	status = register_chrdev(AGCTL_MAJOR, DRIVER_NAME, &agctl_fops);
	if (status < 0)
		return status;

	status = class_register(&agctl_class);

	if (status < 0) {
		unregister_chrdev(AGCTL_MAJOR, DRIVER_NAME);
	}
	return status;
}
module_init(agctl_init);

static void __exit agctl_exit(void)
{
	class_unregister(&agctl_class);
	unregister_chrdev(AGCTL_MAJOR, DRIVER_NAME);
}
module_exit(agctl_exit);

MODULE_AUTHOR("Scott Wagner <scott.wagner@labxtechnologies.com>");
MODULE_DESCRIPTION("Avid Garcia audio control bus interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
