/*
 * i2c_serial.c 
 *
 * Copyright (c) 2012 Meyer Sound Laboratories, Inc.
 *
 * Driver for Meyer Sound CAL I2C interface.
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
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/types.h>
#include <linux/slab.h>

#define DRIVER_NAME "i2c_serial"
#define DEVICES_COUNT 9

#define RX_BUFFER_BITS 8
#define TX_BUFFER_BITS 8

#define RX_MASK ((1<<RX_BUFFER_BITS)-1)
#define RX_IN (dev->rx_in&RX_MASK)
#define RX_OUT (dev->rx_out&RX_MASK)

#define TX_MASK ((1<<TX_BUFFER_BITS)-1)
#define TX_IN (dev->tx_in&TX_MASK)
#define TX_OUT (dev->tx_out&TX_MASK)

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
#if (RX_BUFFER_BITS<=8)
  volatile uint8_t rx_in;
  volatile uint8_t rx_out;
#else
  volatile uint16_t rx_in;
  volatile uint16_t rx_out;
#endif
  uint8_t rx_buffer[1<<RX_BUFFER_BITS];

#if (TX_BUFFER_BITS<=8)
  volatile uint8_t tx_in;
  volatile uint8_t tx_out;
#else
  volatile uint16_t tx_in;
  volatile uint16_t tx_out;
#endif
  uint8_t tx_buffer[1<<TX_BUFFER_BITS];

};

static int i2c_serial_open(struct inode *inode, struct file *filp)
{
  struct i2c_serial *dev;

  dev=container_of(inode->i_cdev,struct i2c_serial,cdev);
  filp->private_data=dev;
  return 0;
}

static int i2c_serial_release(struct inode *inode, struct file *filp)
{
  return 0;
}

void loopback_test(struct i2c_serial * dev) {
    bool wakeup;
    wakeup=false;
    while(
          ((dev->tx_in&TX_MASK)!=(dev->tx_out&TX_MASK)) &&
          (((dev->rx_in+1)&RX_MASK)!=(dev->rx_out&RX_MASK)) ) {
            dev->rx_buffer[dev->rx_in&RX_MASK]=dev->tx_buffer[dev->tx_out&TX_MASK];
            printk(KERN_INFO "looping back [%02x]=[%02x]\n",
                dev->rx_buffer[dev->rx_in&RX_MASK],
                dev->tx_buffer[dev->tx_out&TX_MASK]);
            dev->rx_in++;
            dev->tx_out++;
            wakeup=true;
    }
    if(wakeup) {
        wake_up(&dev->rxq);
        wake_up(&dev->txq);
        printk(KERN_INFO "sent wake up!!! %p\n",&dev->rxq);
    }
    printk(KERN_INFO "rx_in:%d, rx_out:%d tx_in:%d, tx_out:%d\n",dev->rx_in,dev->rx_out,dev->tx_in,dev->tx_out);
}

static unsigned int i2c_serial_poll(struct file *filp, poll_table *wait)
{
  struct i2c_serial *dev;
  unsigned int mask = 0;
  if(filp->private_data==NULL)
    return 0;
  dev=(struct i2c_serial*)filp->private_data;
  down(&dev->sem);
  loopback_test(dev);
  poll_wait(filp, &dev->rxq, wait);
  poll_wait(filp, &dev->txq, wait);
  if((dev->rx_in&RX_MASK)!=(dev->rx_out&RX_MASK))
  {
    mask |= POLLIN | POLLRDNORM; /* readable */
  }
  if((((dev->tx_in+1)&TX_MASK)!=(dev->tx_out&TX_MASK)) &&
     (((dev->tx_in+2)&TX_MASK)!=(dev->tx_out&TX_MASK)))
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
  char * ptr_src;
  bool f;
  int r;

  dev=(struct i2c_serial*)filp->private_data;
  
  if(down_interruptible(&dev->sem))
    return -ERESTARTSYS;

  i=0;
  while(i<count)
  {
    f=false;
    while(((dev->rx_in&RX_MASK)!=(dev->rx_out&RX_MASK)) && (i<count))
    {
      ptr_src=(char*)&dev->rx_buffer[dev->rx_out&RX_MASK];
      if(copy_to_user(&buf[i],ptr_src,1))
      {
        up(&dev->sem);
        return -EFAULT;
      }
      printk(KERN_INFO "read [%02x]\n",*ptr_src);
      i++;
      dev->rx_out++;
      loopback_test(dev);
      f=true;
    }

    if(!f)
    {
      up(&dev->sem);
      if(filp->f_flags & O_NONBLOCK) {
        printk(KERN_INFO "non blocking read returns nothing\n");
        return i;
      } else {
        printk(KERN_INFO "read blocking: %p\n",&dev->rxq);
        r=wait_event_interruptible(dev->rxq,((dev->rx_in&RX_MASK)!=(dev->rx_out&RX_MASK)));
        if(r < 0) {
          if (r == -ERESTARTSYS) {
            printk(KERN_INFO "\t\tread restart sys\n");
          } else {
            printk(KERN_INFO "\t\treturned\n");
          }
          return(r);
        }
        if(down_interruptible(&dev->sem))
          return -EINTR;
        printk(KERN_INFO "read unblocking\n");
      }
    } else {
        break;
    }
  }
  up(&dev->sem);
  printk(KERN_INFO "read done\n");
  return(i);
}

static ssize_t i2c_serial_write(struct file *filp, const char __user *buf,
                                size_t count,loff_t *offset)
{
  struct i2c_serial *dev;
  unsigned int i;
  char * ptr_dst;
  bool f;

  dev=(struct i2c_serial*)filp->private_data;

  if(down_interruptible(&dev->sem))
    return -ERESTARTSYS;

  i=0;
  while(i<count)
  {
    f=false;
    while((i<count) &&
          (((dev->tx_in+1)&TX_MASK)!=(dev->tx_out&TX_MASK)) &&
          (((dev->tx_in+2)&TX_MASK)!=(dev->tx_out&TX_MASK)))
    {
      ptr_dst=(char*)&dev->tx_buffer[dev->tx_in&TX_MASK];
      if(copy_from_user(ptr_dst,&buf[i],1))
      {
        up(&dev->sem);
        return -EFAULT;
      }
      i++;
      dev->tx_in++;
      printk(KERN_INFO "write [%02x]\n",*ptr_dst);
      loopback_test(dev);
      f=true;
    }
    if(!f)
    {
      up(&dev->sem);
      if(filp->f_flags & O_NONBLOCK)
        return i;
      else
      {
        break;
        if(wait_event_interruptible(dev->txq,
            ((((dev->tx_in+1)&TX_MASK)!=(dev->tx_out&TX_MASK)) &&
             (((dev->tx_in+2)&TX_MASK)!=(dev->tx_out&TX_MASK)))))
          return -EINTR;
        if(down_interruptible(&dev->sem))
          return -EINTR;
      }
    } else {
        break;
    }
  }
  dev->write_count+=i;
  up(&dev->sem);
  return i;
}

static struct file_operations i2c_serial_fops =
  {
    .owner = THIS_MODULE,
    .read = i2c_serial_read,
    .write = i2c_serial_write,
    .poll = i2c_serial_poll,
    .open = i2c_serial_open,
    .release = i2c_serial_release,
  };

static int __devinit i2c_serial_probe(struct of_device *pdev,
				      const struct of_device_id *match)
{
  struct i2c_serial *i2c_serial_data;
#if 0
  struct resource res;
#endif
  int retval;
  unsigned int minor;
  unsigned long flags;

#if 0
  retval=of_address_to_resource(ofdev->node,0,&res);
  if(retval)
    {
      printk(KERN_ERR "%s: I/O memory resource is missing\n",
             dev_name(&ofdev->dev));
      return retval;
    }
#endif
  i2c_serial_data = kzalloc(sizeof(*i2c_serial_data), GFP_KERNEL);
  if(!i2c_serial_data)
    {
      printk(KERN_ERR "%s: Failed to allocate device data\n",
             dev_name(&pdev->dev));
      return -ENOMEM;
    }

  spin_lock_irqsave(&static_dev_lock,flags);

  for(minor=0;minor<ndevs&&minors[minor];minor++);
  if(minor>=DEVICES_COUNT)
    {
      spin_unlock_irqrestore(&static_dev_lock,flags);
      printk(KERN_ERR "%s: %d devices already allocated\n",
             dev_name(&pdev->dev),DEVICES_COUNT);
      kfree(i2c_serial_data);
      return -ENODEV;
    }

  if(minor>=ndevs) ndevs=minor+1;
  minors[minor]=1;
  i2c_serial_data->minor=minor;

  spin_unlock_irqrestore(&static_dev_lock,flags);

  cdev_init(&i2c_serial_data->cdev,&i2c_serial_fops);
  i2c_serial_data->cdev.owner=THIS_MODULE;

  dev_set_drvdata(&pdev->dev,i2c_serial_data);

#if 0
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

//  init_MUTEX(&i2c_serial_data->sem);
#endif

  retval=cdev_add(&i2c_serial_data->cdev,
                  MKDEV(major,i2c_serial_data->minor),1);
  if(retval)
    {
      printk(KERN_ERR "%s: Unable to register device\n",
             dev_name(&pdev->dev));
      iounmap(i2c_serial_data->base);
#if 0
      release_mem_region(res.start,resource_size(&res));
#endif
      kfree(i2c_serial_data);
      return -ENODEV;
    }

  sema_init(&i2c_serial_data->sem,1);
  init_waitqueue_head(&i2c_serial_data->rxq);
  init_waitqueue_head(&i2c_serial_data->txq);

  printk(KERN_INFO "%s: i2c serial device initialized, device (%d,%d)\n",
         dev_name(&pdev->dev),major,i2c_serial_data->minor);

  return 0;
}


static int __devexit i2c_serial_remove(struct of_device *pdev)
{
  struct i2c_serial *i2c_serial_data;
#if 0
  struct resource res;
  unsigned long flags;
#endif

  i2c_serial_data = dev_get_drvdata(&pdev->dev);

#if 0
  spin_lock_irqsave(&static_dev_lock,flags);
  minors[i2c_serial_data->minor]=0;
  spin_unlock_irqrestore(&static_dev_lock,flags);

#endif
  cdev_del(&i2c_serial_data->cdev);
#if 0
  iounmap(i2c_serial_data->base);

  if(!of_address_to_resource(ofdev->node,0,&res))
    release_mem_region(res.start,resource_size(&res));
#endif

  printk(KERN_INFO "%s: removed i2c serial device %d:%d\n",
         dev_name(&pdev->dev),major,i2c_serial_data->minor);

  kfree(i2c_serial_data);

  return 0;
}

/* Match table for of_platform binding */
static struct of_device_id __devinitdata i2c_serial_match[] = {
        { .compatible = "xlnx,xps-iic-2.03.a", },
        {},
};

static struct of_platform_driver i2c_serial_driver = {
    .name = DRIVER_NAME,
    .match_table = i2c_serial_match,
    .probe = i2c_serial_probe,
    .remove = __devexit_p(i2c_serial_remove),
  };

static struct platform_device *i2c_serial_platform_device;

/* driver initialization and exit */
static int __init i2c_serial_init(void)
{
    dev_t dev;
    int ret;

    ret = alloc_chrdev_region(&dev,0,DEVICES_COUNT,DRIVER_NAME);
    if(ret)
    {
      printk(KERN_ERR "%s: Can't allocate major/minor device numbers\n",
             DRIVER_NAME);
      return ret;
    }
    major=MAJOR(dev);

    ret = of_register_platform_driver(&i2c_serial_driver);
    if (ret == -ENODEV) {
        goto err_unregister_chrdev;
    }
    return(ret);

 err_unregister_chrdev:
    printk(KERN_ERR "%s: error initializing 3\n", DRIVER_NAME);
    unregister_chrdev_region(MKDEV(major,0),DEVICES_COUNT);
    return(ret);
}

static void __exit i2c_serial_exit(void)
{
  int i;
  unregister_chrdev_region(MKDEV(major,0),DEVICES_COUNT);
  platform_device_unregister(i2c_serial_platform_device);
  platform_driver_unregister(&i2c_serial_driver);
}

module_init(i2c_serial_init);
module_exit(i2c_serial_exit);

MODULE_AUTHOR("andrel@meyersound.com");
MODULE_DESCRIPTION("I2C Serial");
MODULE_LICENSE("GPL v2");
