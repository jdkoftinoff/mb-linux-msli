/*
 * i2s_switch.c 
 *
 * Copyright (c) 2010 Meyer Sound Laboratories, Inc.
 *
 * Driver for Meyer Sound D-Mitri I/O module I2S switch.
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
#include <linux/of_platform.h>
#include "../xilinx_common/xio.h"

/* 
   this driver supports up to 16 i2s switches (16 switches would be a massive
   waste of fpga resources, so more would be pointless)
*/
#define DRIVER_NAME "i2s-switch"
#define DEVICES_COUNT 16


/* 
   shared static data:
   array of used minor numbers and lock for  adding/removing those numbers
*/
static unsigned major=0,ndevs=0;
static unsigned char minors[DEVICES_COUNT];
static DEFINE_SPINLOCK(static_dev_lock);

/* 
   all devices control 24 channels -- 
   this is the size of D-Mitri I/O module
*/
#define CHANNEL_COUNT 24

/* 
   this driver supports read/write operations that do not start and end
   on channel boundaries, however for now each channel is represented
   by a single byte, so it serves no purpose until more members will be
   added to this structure
*/
typedef struct
{
  unsigned char in;
} __attribute__((packed)) i2s_channel_t;

/* device data structure */
struct i2s_switch
{
  void __iomem *base;
  struct semaphore sem;
  struct cdev cdev;
  unsigned int minor;
  /* 
     the following arrays only gets updated for read/write operations,
     and only for channels that are involved in it
  */
  i2s_channel_t channels [CHANNEL_COUNT];
};

/* access to device registers */
static inline void read_channel(struct i2s_switch *dev,
				i2s_channel_t *dst,int n)
{
  dst->in=(unsigned char)(XIo_In32(((unsigned int)dev->base)+n*4)&0xff);
}

static inline void write_channel(struct i2s_switch *dev,
				 i2s_channel_t *dst,int n)
{
  XIo_Out32(((unsigned int)dev->base)+n*4,(u32)dst->in);
}


/* file operations */
static int i2s_switch_open(struct inode *inode, struct file *filp)
{
  struct i2s_switch *dev;
  dev=container_of(inode->i_cdev,struct i2s_switch,cdev);
  filp->private_data=dev;
  return 0;
}

static int i2s_switch_release(struct inode *inode, struct file *filp)
{
  return 0;
}

static ssize_t i2s_switch_read(struct file *filp, char __user *buf,
			       size_t count,loff_t *offset)
{
  struct i2s_switch *dev;
  ssize_t count_available;
  unsigned int channel_start,channel_end,i;
  dev=(struct i2s_switch*)filp->private_data;
  
  /* 
     determine the amount of data that can be read
     and its mapping to the channels
  */
  if(down_interruptible(&dev->sem))
    return -ERESTARTSYS;

  count_available=CHANNEL_COUNT*sizeof(i2s_channel_t)-*offset;
  if(count_available<=0||count<=0)
    {
      up(&dev->sem);
      return 0;
    }
  else
    {
      if(count>count_available)
	count=count_available;
    }
  channel_start=(*offset)/sizeof(i2s_channel_t);
  channel_end=(*offset+count)/sizeof(i2s_channel_t);
  if(channel_end*sizeof(i2s_channel_t)!=(*offset+count))
    channel_end++;

  for(i=channel_start;i<channel_end;i++)
    read_channel(dev,&dev->channels[i],i);

  if(copy_to_user(buf,((char*)&dev->channels)+*offset,count))
    {
      up(&dev->sem);
      return -EFAULT;
    }
  *offset+=count;
  up(&dev->sem);
  return count;
}

static ssize_t i2s_switch_write(struct file *filp, const char __user *buf,
				size_t count,loff_t *offset)
{
  struct i2s_switch *dev;
  ssize_t count_available;
  unsigned int channel_start,channel_end,i;
  dev=(struct i2s_switch*)filp->private_data;
  
  /* 
     determine the amount of data that can be read
     and its mapping to the channels
  */
  if(down_interruptible(&dev->sem))
    return -ERESTARTSYS;

  count_available=CHANNEL_COUNT*sizeof(i2s_channel_t)-*offset;
  if(count_available<=0||count<=0)
    {
      up(&dev->sem);
      return 0;
    }
  else
    {
    if(count>count_available)
      count=count_available;
    }

  /* if write partially overlaps with channels, read them first */
  
  channel_start=(*offset)/sizeof(i2s_channel_t);
  if(channel_start*sizeof(i2s_channel_t)!=*offset)
    {
      read_channel(dev,&dev->channels[channel_start],channel_start);
    }
  channel_end=(*offset+count)/sizeof(i2s_channel_t);
  if(channel_end*sizeof(i2s_channel_t)!=(*offset+count))
    {
      read_channel(dev,&dev->channels[channel_end],channel_end);
      channel_end++;
    }

  if(copy_from_user(((char*)&dev->channels)+*offset,buf,count))
    {
      up(&dev->sem);
      return -EFAULT;
    }
  for(i=channel_start;i<channel_end;i++)
    write_channel(dev,&dev->channels[i],i);
  *offset+=count;
  up(&dev->sem);
  return count;
}

static loff_t i2s_switch_llseek(struct file *filp, loff_t offset, int origin)
{
  loff_t newoffset;

  switch(origin)
    {
    case SEEK_SET:
      newoffset=offset;
      break;
    case SEEK_CUR:
      newoffset=filp->f_pos+offset;
      break;
    case SEEK_END:
      /* device end is at fixed position */
      newoffset=CHANNEL_COUNT*sizeof(i2s_channel_t)+offset;
      break;
    default:
      return -EINVAL;
    }

  /* return error for impossible positions */
  if(newoffset<0||newoffset>CHANNEL_COUNT*sizeof(i2s_channel_t))
    return -EINVAL;
  else
    {
      filp->f_pos=newoffset;
      return newoffset;
    }
}

static struct file_operations i2s_switch_fops =
  {
    .owner = THIS_MODULE,
    .llseek = i2s_switch_llseek,
    .read = i2s_switch_read,
    .write = i2s_switch_write,
    .open = i2s_switch_open,
    .release = i2s_switch_release,
  };

/* device initialization and exit */
static int __devinit i2s_switch_probe(struct of_device *ofdev,
				      const struct of_device_id *match)
{
  struct i2s_switch *i2s_switch_data;
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
  i2s_switch_data = kzalloc(sizeof(*i2s_switch_data), GFP_KERNEL);
  if(!i2s_switch_data)
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
      kfree(i2s_switch_data);
      return -ENODEV;
    }

  if(minor>=ndevs) ndevs=minor+1;
  minors[minor]=1;
  i2s_switch_data->minor=minor;

  spin_unlock_irqrestore(&static_dev_lock,flags);

  cdev_init(&i2s_switch_data->cdev,&i2s_switch_fops);
  i2s_switch_data->cdev.owner=THIS_MODULE;

  dev_set_drvdata(&ofdev->dev,i2s_switch_data);

  if(!request_mem_region(res.start,resource_size(&res),dev_name(&ofdev->dev)))
    {
      printk(KERN_ERR "%s: I/O memory region busy\n",
	     dev_name(&ofdev->dev));
      kfree(i2s_switch_data);
      return -EBUSY;
    }

  i2s_switch_data->base = ioremap(res.start,resource_size(&res));
  if(!i2s_switch_data->base)
    {
      printk(KERN_ERR "%s: Unable to map I/O memory region\n",
	     dev_name(&ofdev->dev));
     release_mem_region(res.start,resource_size(&res));
     kfree(i2s_switch_data);
      return -EIO;
    }

  init_MUTEX(&i2s_switch_data->sem);

  retval=cdev_add(&i2s_switch_data->cdev,
		  MKDEV(major,i2s_switch_data->minor),1);
  if(retval)
    {
      printk(KERN_ERR "%s: Unable to register device\n",
	     dev_name(&ofdev->dev));
      iounmap(i2s_switch_data->base);
      release_mem_region(res.start,resource_size(&res));
      kfree(i2s_switch_data);
      return -ENODEV;
    }

  printk(KERN_INFO "%s: I2S switch device initialized, device (%d,%d)\n",
	 dev_name(&ofdev->dev),major,i2s_switch_data->minor);
  return 0;
}

static int __devexit i2s_switch_remove(struct of_device *ofdev)
{
  struct i2s_switch *i2s_switch_data;
  struct resource res;
  unsigned long flags;

  i2s_switch_data = dev_get_drvdata(&ofdev->dev);

  spin_lock_irqsave(&static_dev_lock,flags);
  minors[i2s_switch_data->minor]=0;
  spin_unlock_irqrestore(&static_dev_lock,flags);

  cdev_del(&i2s_switch_data->cdev);
  iounmap(i2s_switch_data->base);

  if(!of_address_to_resource(ofdev->node,0,&res))
    release_mem_region(res.start,resource_size(&res));
  kfree(i2s_switch_data);

  return 0;
}

/* openfirmware data structures */
static const struct of_device_id i2s_switch_match[] =
  {
    { .compatible = "xlnx,i2s-switch-1.00.a" },
    {}
  };

static struct of_platform_driver i2s_switch_driver =
  {
    .name = DRIVER_NAME,
    .match_table = i2s_switch_match,
    .probe = i2s_switch_probe,
    .remove = __devexit_p(i2s_switch_remove)
  };


/* driver initialization and exit */
static int __init i2s_switch_init(void)
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
  retval = of_register_platform_driver(&i2s_switch_driver);
  if(retval)
    {
      printk(KERN_ERR "%s: Can't register driver\n",
	     DRIVER_NAME);
      return retval;
    }
  return 0;
}

static void __exit i2s_switch_exit(void)
{
  unregister_chrdev_region(MKDEV(major,0),DEVICES_COUNT);
  of_unregister_platform_driver(&i2s_switch_driver);
}

module_init(i2s_switch_init);
module_exit(i2s_switch_exit);

MODULE_AUTHOR("alexb@meyersound.com");
MODULE_DESCRIPTION("I2S Switch");
MODULE_LICENSE("GPL v2");
