/*
 * agspi.c -- SPI device for Avid Garcia sound cards
 *
 * Copyright (C) 2011 Lab X Technologies, LLC
 *	Scott Wagner <scott.wagner@labxtechnologies.com>
 * Based on spidev.c by Andrea Paterniani, <a.paterniani@swapp-eng.it>
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
#include <linux/poll.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>

#include <linux/spi/spi.h>
#include <linux/spi/xilinx_spi.h>
#include <linux/garcia_fpga.h>

#include <asm/uaccess.h>


/*#include <linux/spi/agspi.h>
 *
 * This supports acccess to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/agspiB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define AGSPI_MAJOR			101	/* We assume no Motorola DSP 56xxx board */
#define N_SPI_MINORS		32	/* ... up to 256 */
#define DRIVER_NAME			"agspi"
#define DRIVER_VERSION		"0.4"
#define SPI_SLAVE_BUFSIZE	512		/* Size of slave transmit buffer */

static unsigned long	minors[N_SPI_MINORS / BITS_PER_LONG];


/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for CS_HIGH and 3WIRE can cause *lots* of trouble for other
 * devices on a shared bus:  CS_HIGH, because this device will be
 * active when it shouldn't be;  3WIRE, because when active it won't
 * behave as it should.
 *
 * REVISIT should changing those two modes be privileged?
 */

struct agspi_data {
	dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;
	struct completion   read_complete;
	struct completion   done;
	struct fasync_struct *async_queue; /* asynchronous readers */

	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned int		users;
	int					strobedelay;
	struct spi_message	m;
	struct spi_transfer	t;
	u8					buffer[SPI_SLAVE_BUFSIZE];
};

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/*-------------------------------------------------------------------------*/

void xspi_ss_irq_callback(uint32_t gpioval, void *data)
{
	struct agspi_data *agspi = (struct agspi_data *)data;
	if ((agspi->spi->mode & SPI_SLAVE) != 0 ) {
		spi_reset_transmit_buffer(agspi->spi);
		if (!completion_done(&agspi->read_complete)) {
			complete(&agspi->read_complete);
		}
		/* signal asynchronous readers */
		if (agspi->async_queue) {
			kill_fasync(&agspi->async_queue, SIGIO, POLL_IN);
		}
	}
	return;
}

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void agspi_complete(void *data)
{
	struct agspi_data *agspi = (struct agspi_data *)data;
	complete(&agspi->done);
	return;
}

/*-------------------------------------------------------------------------*/
static ssize_t agspi_do_xfer(struct agspi_data *agspi, void *tx, void *rx, size_t count) {
	ssize_t rc;
	mutex_lock(&agspi->buf_lock);
	spi_message_init(&agspi->m);
	memset(&agspi->t, 0, sizeof(struct spi_transfer));
	agspi->t.tx_buf = tx;
	agspi->t.rx_buf = rx;
	agspi->t.len = count;
	spi_message_add_tail(&agspi->t, &agspi->m);
	agspi->m.complete = agspi_complete;
	agspi->m.context = agspi;
	spin_lock_irq(&agspi->spi_lock);
	rc = spi_async(agspi->spi, &agspi->m);
	spin_unlock_irq(&agspi->spi_lock);
	wait_for_completion_interruptible(&agspi->done);
	mutex_unlock(&agspi->buf_lock);
	if (rc == 0) {
		rc = (agspi->m.status == 0) ? agspi->m.actual_length : agspi->m.status;
	}
	return rc;
}

static int agspi_open(struct inode *inode, struct file *filp)
{
	struct agspi_data	*agspi;
	int			status = -ENXIO;

	lock_kernel();
	mutex_lock(&device_list_lock);

	list_for_each_entry(agspi, &device_list, device_entry) {
		if (agspi->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		agspi->users++;
		filp->private_data = agspi;
		nonseekable_open(inode, filp);
	} else
		pr_debug("agspi: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
	unlock_kernel();
	return status;
}

/* Read-only message with current device setup */
static ssize_t agspi_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct agspi_data	*agspi = filp->private_data;
	ssize_t			len;
	if (agspi == NULL || agspi->spi == NULL) {
		return -ESHUTDOWN;
	}
	if ((agspi->spi->mode & SPI_SLAVE) != 0) {
		if (filp->f_flags & O_NONBLOCK && !completion_done(&agspi->read_complete)) {
			return -EAGAIN;
		}
		if (wait_for_completion_interruptible(&agspi->read_complete)) {
				return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
		}
	}

	len = agspi_do_xfer(agspi, NULL, agspi->buffer, count);
	if (len == -EREMOTEIO) { /* It's OK if received bytes < buffer length */
		len = (agspi->m.status == -EREMOTEIO) ? agspi->m.actual_length : agspi->m.status;
	}
	if (len > 0 && copy_to_user(buf, agspi->buffer, len) != 0) {
		len = -EFAULT;
	}
	return len;
}

/* Write-only message with current device setup */
static ssize_t agspi_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct agspi_data	*agspi = filp->private_data;
	u8				slave_tx_buf[SPI_SLAVE_BUFSIZE]; /* Transmit buffer for slave transfers */
	ssize_t			len;

	if (count > sizeof(slave_tx_buf))
		return -EMSGSIZE;
	if (agspi == NULL || agspi->spi == NULL) {
		return -ESHUTDOWN;
	}
	if (copy_from_user(slave_tx_buf, buf, count) != 0) {
		return -EFAULT;
	}
	len = agspi_do_xfer(agspi, slave_tx_buf, NULL, count);
	return len;
}

static unsigned int agspi_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct agspi_data	*agspi = filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &agspi->read_complete.wait, wait);
	if (completion_done(&agspi->read_complete))
		mask = POLLIN | POLLRDNORM;	/* readable */
	return mask;
}

static int agspi_fasync(int fd, struct file *filp, int mode)
{
	struct agspi_data	*agspi = filp->private_data;
	return fasync_helper(fd, filp, mode, &agspi->async_queue);
}

static int agspi_release(struct inode *inode, struct file *filp)
{
	struct agspi_data	*agspi;
	int			status = 0;

	mutex_lock(&device_list_lock);
	agspi = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	agspi->users--;
	if (!agspi->users) {
		int		dofree;
		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&agspi->spi_lock);
		dofree = (agspi->spi == NULL);
		spin_unlock_irq(&agspi->spi_lock);

		if (dofree)
			kfree(agspi);
	}
	mutex_unlock(&device_list_lock);

	return status;
}

static struct file_operations agspi_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.open =		agspi_open,
	.write =	agspi_write,
	.read =		agspi_read,
	.poll =     agspi_poll,
	.fasync =	agspi_fasync,
	.release =	agspi_release,
};

/*-------------------------------------------------------------------------*/
static ssize_t agdev_w_mode(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long int val;
	struct agspi_data *agspi = dev_get_drvdata(dev);
	u8 mode = agspi->spi->mode;
	if (strict_strtoul(buf, 0, &val) != 0 ||
			(val & ~(SPI_CPHA | SPI_CPOL)) != 0) {
		return -EINVAL;
	}

	agspi->spi->mode = (u8)((mode & ~(SPI_CPHA | SPI_CPOL)) |
			(val & (SPI_CPHA | SPI_CPOL)));
	if (spi_setup(agspi->spi) < 0) {
		agspi->spi->mode = mode;
	} else {
		dev_dbg(&agspi->spi->dev, "spi mode %02x\n", agspi->spi->mode &
				(SPI_CPHA | SPI_CPOL));
	}
	return count;
}

static ssize_t agdev_r_mode(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct agspi_data *agspi = dev_get_drvdata(dev);
	return (snprintf(buf, PAGE_SIZE, "%d\n", agspi->spi->mode &
			(SPI_CPHA | SPI_CPOL)));
}

static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, agdev_r_mode, agdev_w_mode);

static ssize_t agdev_w_bitsperword(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long int val;
	struct agspi_data *agspi = dev_get_drvdata(dev);
	u8 bits_per_word = agspi->spi->bits_per_word;
	if (strict_strtoul(buf, 0, &val) != 0 || val > 0xFF) {
		return -EINVAL;
	}

	agspi->spi->bits_per_word = (u8)val;
	if (spi_setup(agspi->spi) < 0) {
		agspi->spi->bits_per_word = bits_per_word;
	} else {
		dev_dbg(&agspi->spi->dev, "spi bits per word: %d\n",
				agspi->spi->bits_per_word);
	}
	return count;
}

static ssize_t agdev_r_bitsperword(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct agspi_data *agspi = dev_get_drvdata(dev);
	return (snprintf(buf, PAGE_SIZE, "%d\n", agspi->spi->bits_per_word));
}

static DEVICE_ATTR(bitsperword, S_IRUGO | S_IWUSR, agdev_r_bitsperword, agdev_w_bitsperword);

static ssize_t agdev_w_maxspeed(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long int val;
	struct agspi_data *agspi = dev_get_drvdata(dev);
	u32 max_speed_hz = agspi->spi->max_speed_hz;
	if (strict_strtoul(buf, 0, &val) != 0) {
		return -EINVAL;
	}

	agspi->spi->max_speed_hz = val;
	if (spi_setup(agspi->spi) < 0) {
		agspi->spi->max_speed_hz = max_speed_hz;
	} else {
		dev_dbg(&agspi->spi->dev, "spi max speed (hz): %lu\n",
				(unsigned long int)agspi->spi->max_speed_hz);
	}
	return count;
}

static ssize_t agdev_r_maxspeed(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct agspi_data *agspi = dev_get_drvdata(dev);
	return (snprintf(buf, PAGE_SIZE, "%lu\n", (unsigned long int)agspi->spi->max_speed_hz));
}

static DEVICE_ATTR(maxspeed, S_IRUGO | S_IWUSR, agdev_r_maxspeed, agdev_w_maxspeed);

static ssize_t agdev_w_lsbfirst(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long int val;
	struct agspi_data *agspi = dev_get_drvdata(dev);
	u8 mode = agspi->spi->mode;
	if (strict_strtoul(buf, 0, &val) != 0) {
		return -EINVAL;
	}

	agspi->spi->mode = (u8)((mode & ~SPI_LSB_FIRST) |
			((val != 0) ? SPI_LSB_FIRST : 0));
	if (spi_setup(agspi->spi) < 0) {
		agspi->spi->mode = mode;
	} else {
		dev_dbg(&agspi->spi->dev, "spi lsb first: %d\n",
				(agspi->spi->mode & SPI_LSB_FIRST) != 0);
	}
	return count;
}

static ssize_t agdev_r_lsbfirst(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct agspi_data *agspi = dev_get_drvdata(dev);
	return (snprintf(buf, PAGE_SIZE, "%d\n",
			(agspi->spi->mode & SPI_LSB_FIRST) != 0));
}

static DEVICE_ATTR(lsbfirst, S_IRUGO | S_IWUSR, agdev_r_lsbfirst, agdev_w_lsbfirst);

static ssize_t agdev_w_slave(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	unsigned long int val;
	struct agspi_data *agspi = dev_get_drvdata(dev);
	u8 mode = agspi->spi->mode;
	if (strict_strtoul(buf, 0, &val) != 0) {
		return -EINVAL;
	}

	agspi->spi->mode = (u8)((mode & ~SPI_SLAVE) |
			((val != 0) ? SPI_SLAVE : 0));
	if (spi_setup(agspi->spi) < 0) {
		agspi->spi->mode = mode;
	} else {
		dev_dbg(&agspi->spi->dev, "spi slave: %d\n",
				(agspi->spi->mode & SPI_SLAVE) != 0);
	}
	return count;
}

static ssize_t agdev_r_slave(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct agspi_data *agspi = dev_get_drvdata(dev);
	return (snprintf(buf, PAGE_SIZE, "%d\n",
			(agspi->spi->mode & SPI_SLAVE) != 0));
}

static DEVICE_ATTR(slave, S_IRUGO | S_IWUSR, agdev_r_slave, agdev_w_slave);

static ssize_t agdev_w_strobe(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	long int val;
	struct agspi_data *agspi = dev_get_drvdata(dev);
	if (strict_strtol(buf, 0, &val) != 0) {
		return -EINVAL;
	}
	if (val < 0) {
		spi_strobe_ssel(agspi->spi);
	} else {
		agspi->strobedelay = (int)val;
	}
	return count;
}

static ssize_t agdev_r_strobe(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct agspi_data *agspi = dev_get_drvdata(dev);
	return (snprintf(buf, PAGE_SIZE, "%u\n", abs(agspi->strobedelay)));
}

static DEVICE_ATTR(strobe, S_IRUGO | S_IWUSR, agdev_r_strobe, agdev_w_strobe);

static ssize_t agspi_r_minors(struct class *c,char *buf)
{
	return(sprintf(buf, "%ld\n", find_first_zero_bit(minors, N_SPI_MINORS)));
}

/* This class makes mdev/udev create the
 * /dev/agspi-B.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class_attribute agspi_class_attrs[] = {
	__ATTR(minors, S_IRUGO, agspi_r_minors, NULL),
	__ATTR_NULL,
};

static struct class agspi_class = {
	.name =		DRIVER_NAME,
	.owner =	THIS_MODULE,
	.class_attrs =	agspi_class_attrs,
};

/*-------------------------------------------------------------------------*/

static int agspi_probe(struct spi_device *spi)
{
	struct agspi_data	*agspi;
	int			status;
	unsigned long		minor;

	/* Allocate driver data */
	agspi = kzalloc(sizeof(*agspi), GFP_KERNEL);
	if (!agspi)
		return -ENOMEM;

	/* Initialize the driver data */
	agspi->spi = spi;
	spi->controller_data = &agspi->strobedelay;
	spin_lock_init(&agspi->spi_lock);
	mutex_init(&agspi->buf_lock);

	INIT_LIST_HEAD(&agspi->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		agspi->devt = MKDEV(AGSPI_MAJOR, minor);
		init_completion(&agspi->read_complete);
		init_completion(&agspi->done);
		set_gpio_irq_callback(0, GARCIA_FPGA_GPIO_SLOT_SPISEL_1 << minor,
				xspi_ss_irq_callback, agspi);

		dev = device_create(&agspi_class, &spi->dev, agspi->devt,
				    agspi, "agspi-%d.%ld", AGSPI_MAJOR, minor);
		status = device_create_file(dev, &dev_attr_mode);
		status = device_create_file(dev, &dev_attr_bitsperword);
		status = device_create_file(dev, &dev_attr_maxspeed);
		status = device_create_file(dev, &dev_attr_lsbfirst);
		status = device_create_file(dev, &dev_attr_slave);
		status = device_create_file(dev, &dev_attr_strobe);
		sysfs_create_link(agspi_class.dev_kobj, &dev->kobj, dev->kobj.name);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&agspi->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0)
		spi_set_drvdata(spi, agspi);
	else
		kfree(agspi);

	return status;
}

static int __match_devt(struct device *dev, void *data)
{
	return (dev->devt == *(dev_t *)data);
}

static int agspi_remove(struct spi_device *spi)
{
	struct agspi_data	*agspi = spi_get_drvdata(spi);
	struct device *dev = class_find_device(&agspi_class, NULL,
			&agspi->devt, __match_devt);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&agspi->spi_lock);
	agspi->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&agspi->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&agspi->device_entry);
	device_remove_file(dev, &dev_attr_mode);
	device_remove_file(dev, &dev_attr_bitsperword);
	device_remove_file(dev, &dev_attr_maxspeed);
	device_remove_file(dev, &dev_attr_lsbfirst);
	device_remove_file(dev, &dev_attr_slave);
	device_remove_file(dev, &dev_attr_strobe);
	device_destroy(&agspi_class, agspi->devt);
	clear_bit(MINOR(agspi->devt), minors);
	if (agspi->users == 0)
		kfree(agspi);
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver agspi_spi = {
	.driver = {
		.name =		DRIVER_NAME,
		.owner =	THIS_MODULE,
	},
	.probe =	agspi_probe,
	.remove =	__devexit_p(agspi_remove),

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init agspi_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(AGSPI_MAJOR, "spi", &agspi_fops);
	if (status < 0)
		return status;

	status = class_register(&agspi_class);

	status = spi_register_driver(&agspi_spi);

	if (status < 0) {
		unregister_chrdev(AGSPI_MAJOR, agspi_spi.driver.name);
	}
	return status;
}
module_init(agspi_init);

static void __exit agspi_exit(void)
{
	spi_unregister_driver(&agspi_spi);
	class_unregister(&agspi_class);
	unregister_chrdev(AGSPI_MAJOR, agspi_spi.driver.name);
}
module_exit(agspi_exit);

MODULE_AUTHOR("Scott Wagner <scott.wagner@labxtechnologies.com>");
MODULE_DESCRIPTION("Avid Garcia SPI control bus interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
