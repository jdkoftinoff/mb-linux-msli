/*
 * i2c_serial.c 
 *
 * Copyright (c) 2010 Meyer Sound Laboratories, Inc.
 *
 * Driver for Meyer Sound D-Mitri I/O module GNET I2S interface.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/of_platform.h>
#include <linux/types.h>
#include "../xilinx_common/xio.h"

#define DRIVER_NAME "i2c-serial"
#define DEVICES_COUNT 9

#define RX_BUFFER_BITS 8
#define TX_BUFFER_BITS 8

#define RX_MASK ((1<<RX_BUFFER_BITS)-1)
#define RX_IN (dev->rx_in&RX_MASK)
#define RX_OUT (dev->rx_out&RX_MASK)

#define TX_MASK ((1<<TX_BUFFER_BITS)-1)
#define TX_IN (dev->tx_in&TX_MASK)
#define TX_OUT (dev->tx_out&TX_MASK)



/* 
   shared static data:
   array of used minor numbers and lock for  adding/removing those numbers
*/
static unsigned major=0,ndevs=0;
static unsigned char minors[DEVICES_COUNT];
static DEFINE_SPINLOCK(static_dev_lock);

/* device data structure */
struct i2c_serial
{
  void __iomem *base;
  struct semaphore sem;
  struct cdev cdev;
  unsigned int minor;

  wait_queue_head_t rxq;
  wait_queue_head_t txq;

  int write_count;
  /* 
     the following arrays only gets updated for read/write operations,
     and only for registers that are involved in it
  */
#if (RX_BUFFER_BITS<=8)
  uint8_t rx_in;
  uint8_t rx_out;
#else
  uint16_t rx_in;
  uint16_t rx_out;
#endif
  uint8_t rx_buffer[1<<RX_BUFFER_BITS];

#if (TX_BUFFER_BITS<=8)
  uint8_t tx_in;
  uint8_t tx_out;
#else
  uint16_t tx_in;
  uint16_t tx_out;
#endif
  uint16_t tx_buffer[1<<TX_BUFFER_BITS];

};

/* file operations */
static int i2c_serial_open(struct inode *inode, struct file *filp)
{
  struct i2c_serial *dev;
  dev=container_of(inode->i_cdev,struct i2c_serial,cdev);
  filp->private_data=dev;
  init_waitqueue_head(&dev->rxq);
  init_waitqueue_head(&dev->txq);
  return 0;
}

static int i2c_serial_release(struct inode *inode, struct file *filp)
{
  return 0;
}


static unsigned int i2c_serial_poll(struct file *filp, poll_table *wait)
{
  struct i2c_serial *dev;
  unsigned int mask = 0;
  if(filp->private_data==NULL)
    return 0;
  dev=(struct i2c_serial*)filp->private_data;
  down(&dev->sem);
  poll_wait(filp, &dev->rxq, wait);
  poll_wait(filp, &dev->txq, wait);
  if((dev->rx_in&RX_MASK)==(dev->rx_out&RX_MASK))
  {
    mask |= POLLIN | POLLRDNORM; /* readable */
  }
  if((((dev->tx_in+1)&TX_MASK)==(dev->tx_out&TX_MASK)) ||
     (((dev->tx_in+2)&TX_MASK)==(dev->tx_out&TX_MASK)))
  {
    mask |= POLLOUT | POLLWRNORM; /* writeable */
  }
  up(&dev->sem);
  return(mask);
}

static ssize_t i2c_serial_read(struct file *filp, char __user *buf,
			       size_t count,loff_t *offset)
{
  struct i2c_serial *dev;
  unsigned int i;
  char str[100];
  dev=(struct i2c_serial*)filp->private_data;
  
  /* 
     determine the amount of data that can be read
     and its mapping to the regs
  */
  if(down_interruptible(&dev->sem))
    return -ERESTARTSYS;

  i=0;
  while(i<count)
  {
    while(((dev->rx_in&RX_MASK)!=(dev->rx_out&RX_MASK)) && (i<count))
    {
//      if(i>=offset)
//      {
        if(copy_to_user(&buf[i],(char*)&dev->rx_buffer[dev->rx_out&RX_MASK],1))
        {
          up(&dev->sem);
          return -EFAULT;
        }
//      } else {
//      }
      i++;
      dev->rx_out++;
    }

    if(i<count)
    {
      up(&dev->sem);
      if(filp->f_flags & O_NONBLOCK)
        return i;
      else
      {
        if(wait_event_interruptible(dev->rxq,(dev->rx_in&RX_MASK)!=(dev->rx_out&RX_MASK)))
          return -EINTR;
        if(down_interruptible(&dev->sem))
          return -EINTR;
      }
    }
  }
  up(&dev->sem);
  return(i);
#if 0
  snprintf(str,sizeof(str),"[%d]",dev->write_count);
  str[sizeof(str)-1]='\0';
  if(count<(strlen(str)+1)) {
    str[count-1]='\0';
  } else {
    count=strlen(str)+1;
  }
  copy_to_user(buf,str,count);
  up(&dev->sem);
  return i;
#endif
}

static ssize_t i2c_serial_write(struct file *filp, const char __user *buf,
				size_t count,loff_t *offset)
{
  struct i2c_serial *dev;
  unsigned int i;

  dev=(struct i2c_serial*)filp->private_data;

  if(down_interruptible(&dev->sem))
    return -ERESTARTSYS;

  i=0;
  while(i<count)
  {
    while((i<count) &&
          (((dev->tx_in+1)&TX_MASK)!=(dev->tx_out&TX_MASK)) &&
          (((dev->tx_in+2)&TX_MASK)!=(dev->tx_out&TX_MASK)))
    {
      if(copy_from_user((char*)&dev->tx_buffer[dev->tx_in&TX_MASK],&buf[i],1))
      {
        up(&dev->sem);
        return -EFAULT;
      }
      i++;
      dev->tx_in++;
    }
    if(i==0)
    {
      up(&dev->sem);
      if(filp->f_flags & O_NONBLOCK)
        return i;
      else
      {
        if(wait_event_interruptible(dev->txq,
            ((((dev->tx_in+1)&TX_MASK)!=(dev->tx_out&TX_MASK)) &&
             (((dev->tx_in+2)&TX_MASK)!=(dev->tx_out&TX_MASK)))))
          return -EINTR;
        if(down_interruptible(&dev->sem))
          return -EINTR;
      }
    }
  }
  dev->write_count+=i;
  up(&dev->sem);
  return i;
}


static struct file_operations i2c_serial_fops =
  {
    .owner = THIS_MODULE,
    .poll = i2c_serial_poll,
    .read = i2c_serial_read,
    .write = i2c_serial_write,
    .open = i2c_serial_open,
    .release = i2c_serial_release,
  };


/* device initialization and exit */
static int __devinit i2c_serial_probe(struct of_device *ofdev,
				      const struct of_device_id *match)
{
  struct i2c_serial *i2c_serial_data;
  struct resource res;
  int retval;
  unsigned int minor;
  unsigned long flags;

  retval=of_address_to_resource(ofdev->node,0,&res);
  if(retval)
    {
      printk(KERN_ERR "%s: I/O memory resource is missing\n",
	     dev_name(&ofdev->dev));
      return retval;
    }
  i2c_serial_data = kzalloc(sizeof(*i2c_serial_data), GFP_KERNEL);
  if(!i2c_serial_data)
    {
      printk(KERN_ERR "%s: Failed to allocate device data\n",
	     dev_name(&ofdev->dev));
      return -ENOMEM;
    }

  spin_lock_irqsave(&static_dev_lock,flags);

  for(minor=0;minor<ndevs&&minors[minor];minor++);
  if(minor>=DEVICES_COUNT)
    {
      spin_unlock_irqrestore(&static_dev_lock,flags);
      printk(KERN_ERR "%s: %d devices already allocated\n",
	     dev_name(&ofdev->dev),DEVICES_COUNT);
      kfree(i2c_serial_data);
      return -ENODEV;
    }

  if(minor>=ndevs) ndevs=minor+1;
  minors[minor]=1;
  i2c_serial_data->minor=minor;

  spin_unlock_irqrestore(&static_dev_lock,flags);

  cdev_init(&i2c_serial_data->cdev,&i2c_serial_fops);
  i2c_serial_data->cdev.owner=THIS_MODULE;

  dev_set_drvdata(&ofdev->dev,i2c_serial_data);

  if(!request_mem_region(res.start,resource_size(&res),dev_name(&ofdev->dev)))
    {
      printk(KERN_ERR "%s: I/O memory region busy\n",
	     dev_name(&ofdev->dev));
      kfree(i2c_serial_data);
      return -EBUSY;
    }

  i2c_serial_data->base = ioremap(res.start,resource_size(&res));
  if(!i2c_serial_data->base)
    {
      printk(KERN_ERR "%s: Unable to map I/O memory region\n",
	     dev_name(&ofdev->dev));
     release_mem_region(res.start,resource_size(&res));
     kfree(i2c_serial_data);
      return -EIO;
    }

  init_MUTEX(&i2c_serial_data->sem);

  retval=cdev_add(&i2c_serial_data->cdev,
		  MKDEV(major,i2c_serial_data->minor),1);
  if(retval)
    {
      printk(KERN_ERR "%s: Unable to register device\n",
	     dev_name(&ofdev->dev));
      iounmap(i2c_serial_data->base);
      release_mem_region(res.start,resource_size(&res));
      kfree(i2c_serial_data);
      return -ENODEV;
    }

  printk(KERN_INFO "%s: i2c serial device initialized, device (%d,%d)\n",
	 dev_name(&ofdev->dev),major,i2c_serial_data->minor);
  return 0;
}


static int __devexit i2c_serial_remove(struct of_device *ofdev)
{
  struct i2c_serial *i2c_serial_data;
  struct resource res;
  unsigned long flags;

  i2c_serial_data = dev_get_drvdata(&ofdev->dev);

  spin_lock_irqsave(&static_dev_lock,flags);
  minors[i2c_serial_data->minor]=0;
  spin_unlock_irqrestore(&static_dev_lock,flags);

  cdev_del(&i2c_serial_data->cdev);
  iounmap(i2c_serial_data->base);

  if(!of_address_to_resource(ofdev->node,0,&res))
    release_mem_region(res.start,resource_size(&res));
  kfree(i2c_serial_data);

  return 0;
}


/* Match table for of_platform binding */
static struct of_device_id __devinitdata i2c_serial_match[] = {
	{ .compatible = "xlnx,xps-iic-2.03.a", },
	{},
};
MODULE_DEVICE_TABLE(of, i2c_serial_match);

static struct of_platform_driver i2c_serial_driver =
  {
    .name = DRIVER_NAME,
    .match_table = i2c_serial_match,
    .probe = i2c_serial_probe,
    .remove = __devexit_p(i2c_serial_remove)
  };


/* driver initialization and exit */
static int __init i2c_serial_init(void)
{
  dev_t dev;
  int retval;

  retval = alloc_chrdev_region(&dev,0,DEVICES_COUNT,DRIVER_NAME);
  if(retval)
    {
      printk(KERN_ERR "%s: Can't allocate major/minor device numbers\n",
	     DRIVER_NAME);
      return retval;
    }

  major=MAJOR(dev);
  retval = of_register_platform_driver(&i2c_serial_driver);
  if(retval)
    {
      printk(KERN_ERR "%s: Can't register driver\n",
	     DRIVER_NAME);
      return retval;
    }
  return 0;
}

static void __exit i2c_serial_exit(void)
{
  unregister_chrdev_region(MKDEV(major,0),DEVICES_COUNT);
  of_unregister_platform_driver(&i2c_serial_driver);
}

module_init(i2c_serial_init);
module_exit(i2c_serial_exit);

MODULE_AUTHOR("andrel@meyersound.com");
MODULE_DESCRIPTION("I2C Serial");
MODULE_LICENSE("GPL v2");
