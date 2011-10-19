/*
 * xilinx_mailbox.c 
 *
 * Copyright (c) 2011 Meyer Sound Laboratories, Inc.
 *
 * Driver for Xilinx mailbox device.
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
#include "../xilinx_common/xio.h"

/* 
   this driver supports up to 16 mailbox interfaces
*/
#define DRIVER_NAME "xilinx-mailbox"
#define DEVICES_COUNT 16

#define MAILBOX_BUFFER_SIZE 2048

/* register offsets */
#define MAILBOX_REG_WRDATA 0x00
#define MAILBOX_REG_RDDATA 0x08
#define MAILBOX_REG_STATUS 0x10
#define MAILBOX_REG_ERROR  0x14
#define MAILBOX_REG_SIT    0x18
#define MAILBOX_REG_RIT    0x1c
#define MAILBOX_REG_IS     0x20
#define MAILBOX_REG_IE     0x24
#define MAILBOX_REG_IP     0x28

#define MAILBOX_BIT_IS_ERR 0x04
#define MAILBOX_BIT_IS_RTI 0x02
#define MAILBOX_BIT_IS_STI 0x01

#define MAILBOX_BIT_STATUS_EMPTY 0x01
#define MAILBOX_BIT_STATUS_FULL  0x02

#define MAILBOX_BIT_IE_ERROR 0x04
#define MAILBOX_BIT_IE_RTI 0x02
#define MAILBOX_BIT_IE_STI 0x01

/* 
   shared static data:
   array of used minor numbers and lock for adding/removing those numbers
*/
static unsigned major=0,ndevs=0;
static unsigned char minors[DEVICES_COUNT];
static DEFINE_SPINLOCK(static_dev_lock);



/* device data structure */
struct xil_mailbox
{
  void __iomem *base;
  unsigned int irq;
  spinlock_t lock;
  struct semaphore sem;
  struct cdev cdev;
  unsigned int minor;

  int inuse; /* currently only 0 and 1 are valid values */

  wait_queue_head_t wait_read_queue;
  wait_queue_head_t wait_write_queue;

  unsigned int send_rptr;
  unsigned int send_wptr;
  unsigned int send_buffer[MAILBOX_BUFFER_SIZE];

  unsigned int recv_rptr;
  unsigned int recv_wptr;
  unsigned int recv_buffer[MAILBOX_BUFFER_SIZE];
};

/* file operations */
static int xil_mailbox_open(struct inode *inode, struct file *filp)
{
  struct xil_mailbox *dev;
  dev=container_of(inode->i_cdev,struct xil_mailbox,cdev);
  filp->private_data=dev;

  down(&dev->sem);
  if(dev->inuse)
    {
      up(&dev->sem);
      return -EBUSY;
    }
  dev->inuse=1;
  up(&dev->sem);

  return 0;
}

static int xil_mailbox_release(struct inode *inode, struct file *filp)
{
  struct xil_mailbox *dev;
  dev=(struct xil_mailbox*)filp->private_data;
  if(!dev) return 0; /* should never happen */
  down(&dev->sem);
  dev->inuse=0;
  up(&dev->sem);
  return 0;
}

static unsigned xil_mailbox_poll(struct file *filp, 
				 struct poll_table_struct *wait)
{
  struct xil_mailbox *dev;
  unsigned long flags;
  unsigned mask=0;
  int delta;

  dev=(struct xil_mailbox*)filp->private_data;
  if(!dev) return 0; /* should never happen */

  down(&dev->sem);

  poll_wait(filp, &dev->wait_read_queue, wait);
  poll_wait(filp, &dev->wait_write_queue, wait);

  /* check for available buffers and return corresponding mask here */
  spin_lock_irqsave(dev->lock,flags);

  if(dev->recv_wptr!=dev->recv_rptr)
    mask|=POLLIN|POLLRDNORM;

  delta=dev->send_wptr-dev->send_rptr;
  if(delta!=-1&&delta!=MAILBOX_BUFFER_SIZE-1)
    mask|=POLLOUT|POLLWRNORM;

  spin_unlock_irqrestore(&dev->lock,flags);

  up(&dev->sem);
  return mask;
  
}

int mailbox_rx(struct xil_mailbox *xil_mailbox_data)
{
  int counter,retval=0;
 counter=xil_mailbox_data->recv_wptr-xil_mailbox_data->recv_rptr;
 if(counter<0) counter+=MAILBOX_BUFFER_SIZE;
 counter=MAILBOX_BUFFER_SIZE-1-counter;
 /*
   read data into the buffer until either data buffer is full,
   or FIFO is empty
 */
 while((counter>0)
       &&(((XIo_In32(((unsigned int)xil_mailbox_data->base
		      +MAILBOX_REG_STATUS)))&MAILBOX_BIT_STATUS_EMPTY)==0))
   {
     xil_mailbox_data->recv_buffer[xil_mailbox_data->recv_wptr]=
       XIo_In32(((unsigned int)xil_mailbox_data->base
		 +MAILBOX_REG_RDDATA));
     xil_mailbox_data->recv_wptr++;
     retval=1;
     if(xil_mailbox_data->recv_wptr>=MAILBOX_BUFFER_SIZE)
       xil_mailbox_data->recv_wptr=0;
     counter--;
   }
 return retval;
}

int mailbox_tx(struct xil_mailbox *xil_mailbox_data)
{
  int counter,retval=0;
  counter=xil_mailbox_data->send_wptr-xil_mailbox_data->send_rptr;
  if(counter<0) counter+=MAILBOX_BUFFER_SIZE;
  while((counter>0)
	&&(((XIo_In32(((unsigned int)xil_mailbox_data->base
		       +MAILBOX_REG_STATUS)))&MAILBOX_BIT_STATUS_FULL)==0))
    {
#ifdef DEBUG
      printk(KERN_ALERT "mbx-> %08x\n", 
	     xil_mailbox_data->send_buffer[xil_mailbox_data->send_rptr]);
#endif
      XIo_Out32(((unsigned int)xil_mailbox_data->base
		 +MAILBOX_REG_WRDATA),
		xil_mailbox_data->send_buffer[xil_mailbox_data->send_rptr]);
      xil_mailbox_data->send_rptr++;
      retval=1;
      if(xil_mailbox_data->send_rptr>=MAILBOX_BUFFER_SIZE)
	xil_mailbox_data->send_rptr=0;
      counter--;
    }
  return retval;
}


static ssize_t xil_mailbox_read(struct file *filp, char __user *buf,
				size_t count,loff_t *offset)
{
  struct xil_mailbox *dev;
  unsigned long flags;
  ssize_t copied,copy_remaining,copy_count;
  unsigned int rptr,wptr;
  unsigned int zero=0;

  dev=(struct xil_mailbox*)filp->private_data;
  if(!dev) return 0; /* should never happen */

  if(down_interruptible(&dev->sem))
    return -ERESTARTSYS;

  copied=0;

  while(count>0)
    {
      /* process i/o buffers */
      spin_lock_irqsave(&dev->lock,flags);
      rptr=dev->recv_rptr;
      wptr=dev->recv_wptr;
      spin_unlock_irqrestore(&dev->lock,flags);
      
      if(wptr!=rptr)
	{
	  if(wptr>rptr)
	    {
	      /* copy one chunk from read to write index (or less) */
	      copy_count=wptr-rptr;
	      if(copy_count>count/sizeof(dev->recv_buffer[0]))
		copy_count=count/sizeof(dev->recv_buffer[0]);
	      copy_to_user(buf,(char*)&dev->recv_buffer[rptr],
			   copy_count*sizeof(dev->recv_buffer[0]));
	      rptr+=copy_count;
	      if(rptr>=MAILBOX_BUFFER_SIZE)
		rptr=0;
	    }
	  else
	    {
	      /* copy from read index to the end of buffer (or less) */
	      copy_count=MAILBOX_BUFFER_SIZE-rptr;
	      if(copy_count>count/sizeof(dev->recv_buffer[0]))
		copy_count=count/sizeof(dev->recv_buffer[0]);
	      copy_to_user(buf,(char*)&dev->recv_buffer[rptr],
			   copy_count*sizeof(dev->recv_buffer[0]));
	      rptr+=copy_count;
	      if(rptr>=MAILBOX_BUFFER_SIZE)
		rptr=0;
	      /* copy from start of buffer to the write pointer (or less) */
	      copy_remaining=count/sizeof(dev->recv_buffer[0])-copy_count;
	      if(wptr<copy_remaining)
		copy_remaining=wptr;
	      if(copy_remaining>0)
		{
		  copy_to_user(buf+copy_count*sizeof(dev->recv_buffer[0]),
			       (char*)&dev->recv_buffer[0],
			       copy_remaining*sizeof(dev->recv_buffer[0]));
		  rptr=copy_remaining;
		  copy_count+=copy_remaining;
		}
	    }
	}
      else
	copy_count=0;

      /* update read pointer (write pointer could have been moved by now) */
      spin_lock_irqsave(&dev->lock,flags);
      dev->recv_rptr=rptr;
      spin_unlock_irqrestore(&dev->lock,flags);

      copy_remaining=count-copy_count*sizeof(dev->recv_buffer[0]);
      /* 
	 copy_remaining is now the remaining available number of bytes
	 in the user buffer
      */

      /* pad unaligned with 0 if buffer given with unaligned size */
      if(copy_remaining<sizeof(dev->recv_buffer[0]))
	{
	  copy_to_user(buf+count-copy_remaining,&zero,copy_remaining);
	  copy_remaining=0;
	}
      /* update counters */
      copied+=count-copy_remaining;
      buf+=count-copy_remaining;
      count=copy_remaining;

      spin_lock_irqsave(&dev->lock,flags);
      /* 
	 check if:
	 1. there is still space in user's destination buffer
	 2. attempt to receive more data failed
	 3. pointers still show empty buffer

	 if so, read operation can not be completed -- return partial
	 read or sleep
      */
      if(count&&!mailbox_rx(dev)&&(dev->recv_wptr==dev->recv_rptr))
	{
	  spin_unlock_irqrestore(&dev->lock,flags);
	  /* buffer is not filled completely */
	  up(&dev->sem);

	  /* return partial read if nonblocking */
	  if(filp->f_flags & O_NONBLOCK)
	    return copied;
	  else
	    {
	      /* wait for pointers to move */
	      if(wait_event_interruptible(dev->wait_read_queue,
					  (dev->recv_wptr!=dev->recv_rptr)))
		return -EINTR;
	      
	      if(down_interruptible(&dev->sem))
		return -EINTR;
	    }
	  spin_lock_irqsave(&dev->lock,flags);
	}
      spin_unlock_irqrestore(&dev->lock,flags);
    }

  up(&dev->sem);
  return copied;
}

static ssize_t xil_mailbox_write(struct file *filp, const char __user *buf,
				 size_t count,loff_t *offset)
{
  struct xil_mailbox *dev;
  unsigned long flags;
  unsigned int rptr,wptr;
  ssize_t spaceleft,copied,write_remaining,write_count;

  dev=(struct xil_mailbox*)filp->private_data;
  if(!dev) return 0; /* should never happen */
  
  if(down_interruptible(&dev->sem))
    return -ERESTARTSYS;

  copied=0;

  while(count>0)
    {
      /* process i/o buffers */
      spin_lock_irqsave(&dev->lock,flags);
      rptr=dev->send_rptr;
      wptr=dev->send_wptr;
      spin_unlock_irqrestore(&dev->lock,flags);
      spaceleft=rptr-wptr-1;
      if(spaceleft<0) spaceleft+=MAILBOX_BUFFER_SIZE;

#ifdef DEBUG
      printk(KERN_ALERT "writing: %d words left in the buffer\n",spaceleft);
#endif
      if(spaceleft>0)
	{
	  if(rptr>wptr)
	    {
	      write_count=rptr-wptr-1;
	      if(count/sizeof(dev->send_buffer[0])<write_count)
		write_count=count/sizeof(dev->send_buffer[0]);
	      if(write_count>0)
		{
		  copy_from_user((char*)&dev->send_buffer[wptr],buf,
				 write_count*sizeof(dev->send_buffer[0]));
		  wptr+=write_count;
		  if(wptr>=MAILBOX_BUFFER_SIZE)
		    wptr=0;
		}
	    }
	  else
	    {
	      /* special case for read pointer at 0 */
	      if(rptr==0)
		{
		  write_count=MAILBOX_BUFFER_SIZE-wptr-1;
		  if(write_count>count/sizeof(dev->send_buffer[0]))
		    write_count=count/sizeof(dev->send_buffer[0]);
		  copy_from_user((char*)&dev->send_buffer[wptr],buf,
				 write_count*sizeof(dev->send_buffer[0]));
		  wptr+=write_count;
		  /* write pointer does not wrap here */
		}
	      else
		{
		  /* write pointer can wrap */
		  write_count=MAILBOX_BUFFER_SIZE-wptr;
		  if(write_count>count/sizeof(dev->send_buffer[0]))
		    write_count=count/sizeof(dev->send_buffer[0]);
		  copy_from_user((char*)&dev->send_buffer[wptr],buf,
				 write_count*sizeof(dev->send_buffer[0]));
		  wptr+=write_count;
		  if(wptr>=MAILBOX_BUFFER_SIZE)
		    {
		      wptr=0;
		      write_remaining=count/sizeof(dev->send_buffer[0])
			-write_count;
		      if(write_remaining>0)
			{
			  if(write_remaining>rptr-1)
			    write_remaining=rptr-1;
			  if(write_remaining>0)
			    copy_from_user((char*)&dev->send_buffer[0],
				  buf+write_count*sizeof(dev->send_buffer[0]),
				  write_remaining*sizeof(dev->send_buffer[0]));
			  wptr+=write_remaining;
			  write_count+=write_remaining;
			}
		    }
		}
	    }
	}
      else
	write_count=0;

#ifdef DEBUG
      printk(KERN_ALERT "writing: %d words\n",write_count);
#endif
      
      /* update write pointer (read pointer could have been moved by now) */
      spin_lock_irqsave(&dev->lock,flags);
      dev->send_wptr=wptr;
      spin_unlock_irqrestore(&dev->lock,flags);
      
      write_remaining=count-write_count*sizeof(dev->send_buffer[0]);
      /*
	write_remaining is now the remaining number of bytes
	supplied in the user buffer
      */
      
      /* discard the end of unaligned buffer */
      if(write_remaining<sizeof(dev->send_buffer[0]))
	write_remaining=0;
      /* update_counters */
      copied+=count-write_remaining;
      buf+=count-write_remaining;
      count=write_remaining;

#ifdef DEBUG
      printk(KERN_ALERT "writing: %d bytes remaining\n",count);
#endif
      
      spin_lock_irqsave(&dev->lock,flags);

      /* 
	 check if:
	 1. attempt to send data failed
	 2. there is still data to copy
	 3. pointers still show completely filled up buffer

	 if so, write operation can not be completed -- return partial
	 write or sleep
      */
      if(!mailbox_tx(dev)&&count&&((dev->send_wptr+1)%MAILBOX_BUFFER_SIZE
				   ==dev->send_rptr))
	{
	  spin_unlock_irqrestore(&dev->lock,flags);
	  /* buffer is not filled completely */
	  up(&dev->sem);
	  
	  /* return partial read if nonblocking */
	  if(filp->f_flags & O_NONBLOCK)
	    return copied;
	  else
	    {
	      /* wait for pointers to move */
	      if(wait_event_interruptible(dev->wait_write_queue,
				       ((dev->send_wptr+1)%MAILBOX_BUFFER_SIZE
					!=dev->send_rptr)))
		return -EINTR;
	      
	      if(down_interruptible(&dev->sem))
		return -EINTR;
	    }
	  spin_lock_irqsave(&dev->lock,flags);
	}
      spin_unlock_irqrestore(&dev->lock,flags);
    }

  up(&dev->sem);
  return copied;
}

static struct file_operations xil_mailbox_fops =
  {
    .owner = THIS_MODULE,
    .poll = xil_mailbox_poll,
    .read = xil_mailbox_read,
    .write = xil_mailbox_write,
    .open = xil_mailbox_open,
    .release = xil_mailbox_release,
  };

static irqreturn_t xil_mailbox_irqhandler(int irq, void *p)
{
  struct xil_mailbox *xil_mailbox_data;
  unsigned long flags;
  unsigned int intflags,actions=0;

  xil_mailbox_data=(struct xil_mailbox*)p;
  intflags=XIo_In32(((unsigned int)xil_mailbox_data->base
		     +MAILBOX_REG_IS));

#ifdef DEBUG
  printk(KERN_ALERT "mailbox interrupt, flags 0x%08x\n",intflags);
#endif
  /* perform i/o */
  spin_lock_irqsave(&xil_mailbox_data->lock,flags);
  if(intflags&MAILBOX_BIT_IS_RTI&&mailbox_rx(xil_mailbox_data))
    actions|=1;
  
  if(intflags&MAILBOX_BIT_IS_STI&&mailbox_tx(xil_mailbox_data))
    actions|=2;
  spin_unlock_irqrestore(&xil_mailbox_data->lock,flags);

  /* acknowledge interrupts */
  XIo_Out32(((unsigned int)xil_mailbox_data->base
	     +MAILBOX_REG_IS),intflags);
  
  /* wake up whatever is waiting on those buffers */
  if(actions&1)
    wake_up_interruptible(&xil_mailbox_data->wait_read_queue);
  if(actions&2)
    wake_up_interruptible(&xil_mailbox_data->wait_write_queue);
  
  return IRQ_HANDLED;
}

/* device initialization and exit */
static int __devinit xil_mailbox_probe(struct of_device *ofdev,
				      const struct of_device_id *match)
{
  struct xil_mailbox *xil_mailbox_data;
  struct resource res_mem,res_irq;
  int retval;
  unsigned int minor;
  unsigned long flags;

  retval=of_address_to_resource(ofdev->node,0,&res_mem);
  if(retval)
    {
      printk(KERN_ERR "%s: I/O memory resource is missing\n",
	     dev_name(&ofdev->dev));
      return retval;
    }

  if(of_irq_to_resource(ofdev->node,0,&res_irq)==NO_IRQ)
    {
      printk(KERN_ERR "%s: interrupt resource is missing\n",
	     dev_name(&ofdev->dev));
      return -ENODEV;
    }

  xil_mailbox_data = kzalloc(sizeof(*xil_mailbox_data), GFP_KERNEL);
  if(!xil_mailbox_data)
    {
      printk(KERN_ERR "%s: Failed to allocate device data\n",
	     dev_name(&ofdev->dev));
      return -ENOMEM;
    }
  spin_lock_init(&xil_mailbox_data->lock);
  spin_lock_irqsave(&static_dev_lock,flags);

  for(minor=0;minor<ndevs&&minors[minor];minor++);
  if(minor>=DEVICES_COUNT)
    {
      spin_unlock_irqrestore(&static_dev_lock,flags);
      printk(KERN_ERR "%s: %d devices already allocated\n",
	     dev_name(&ofdev->dev),DEVICES_COUNT);
      kfree(xil_mailbox_data);
      return -ENODEV;
    }

  if(minor>=ndevs) ndevs=minor+1;
  minors[minor]=1;
  xil_mailbox_data->minor=minor;

  spin_unlock_irqrestore(&static_dev_lock,flags);

  cdev_init(&xil_mailbox_data->cdev,&xil_mailbox_fops);
  xil_mailbox_data->cdev.owner=THIS_MODULE;

  dev_set_drvdata(&ofdev->dev,xil_mailbox_data);

  if(!request_mem_region(res_mem.start,resource_size(&res_mem),
			 dev_name(&ofdev->dev)))
    {
      printk(KERN_ERR "%s: I/O memory region busy\n",
	     dev_name(&ofdev->dev));
      kfree(xil_mailbox_data);
      return -EBUSY;
    }

  xil_mailbox_data->base = ioremap(res_mem.start,resource_size(&res_mem));
  if(!xil_mailbox_data->base)
    {
      printk(KERN_ERR "%s: Unable to map I/O memory region\n",
	     dev_name(&ofdev->dev));
      release_mem_region(res_mem.start,resource_size(&res_mem));
      kfree(xil_mailbox_data);
      return -EIO;
    }

  xil_mailbox_data->irq=res_irq.start;

  init_MUTEX(&xil_mailbox_data->sem);

  init_waitqueue_head(&xil_mailbox_data->wait_read_queue);
  init_waitqueue_head(&xil_mailbox_data->wait_write_queue);

  if(request_irq(xil_mailbox_data->irq,xil_mailbox_irqhandler,IRQF_DISABLED,
		 dev_name(&ofdev->dev),xil_mailbox_data))
    {
      printk(KERN_ERR "%s: Unable to request interrupt\n",
	     dev_name(&ofdev->dev));
      iounmap(xil_mailbox_data->base);
      release_mem_region(res_mem.start,resource_size(&res_mem));
      kfree(xil_mailbox_data);
      return -EIO;
    }

  XIo_Out32((unsigned int)xil_mailbox_data->base+MAILBOX_REG_IE,
	    MAILBOX_BIT_IE_RTI|MAILBOX_BIT_IE_STI|MAILBOX_BIT_IE_ERROR);

  XIo_Out32((unsigned int)xil_mailbox_data->base+MAILBOX_REG_RIT,0);
  XIo_Out32((unsigned int)xil_mailbox_data->base+MAILBOX_REG_SIT,0);

  retval=cdev_add(&xil_mailbox_data->cdev,
		  MKDEV(major,xil_mailbox_data->minor),1);
  if(retval)
    {
      printk(KERN_ERR "%s: Unable to register device\n",
	     dev_name(&ofdev->dev));
      free_irq(xil_mailbox_data->irq,xil_mailbox_data);
      XIo_Out32((unsigned int)xil_mailbox_data->base+MAILBOX_REG_IE,0);
      iounmap(xil_mailbox_data->base);
      release_mem_region(res_mem.start,resource_size(&res_mem));
      kfree(xil_mailbox_data);
      return -ENODEV;
    }

  printk(KERN_INFO "%s: Xilinx mailbox device initialized, device (%d,%d)\n",
	 dev_name(&ofdev->dev),major,xil_mailbox_data->minor);
  return 0;
}

static int __devexit xil_mailbox_remove(struct of_device *ofdev)
{
  struct xil_mailbox *xil_mailbox_data;
  struct resource res_mem;
  unsigned long flags;

  xil_mailbox_data = dev_get_drvdata(&ofdev->dev);

  spin_lock_irqsave(&static_dev_lock,flags);
  minors[xil_mailbox_data->minor]=0;
  XIo_Out32((unsigned int)xil_mailbox_data->base+MAILBOX_REG_IE,0);

  spin_unlock_irqrestore(&static_dev_lock,flags);
  cdev_del(&xil_mailbox_data->cdev);
  free_irq(xil_mailbox_data->irq,xil_mailbox_data);
  iounmap(xil_mailbox_data->base);

  if(!of_address_to_resource(ofdev->node,0,&res_mem))
    release_mem_region(res_mem.start,resource_size(&res_mem));
  kfree(xil_mailbox_data);

  return 0;
}

/* openfirmware data structures */
static const struct of_device_id xil_mailbox_match[] =
  {
    { .compatible = "xlnx,xps-mailbox-2.00.b" },
    {}
  };

static struct of_platform_driver xil_mailbox_driver =
  {
    .name = DRIVER_NAME,
    .match_table = xil_mailbox_match,
    .probe = xil_mailbox_probe,
    .remove = __devexit_p(xil_mailbox_remove)
  };


/* driver initialization and exit */
static int __init xil_mailbox_init(void)
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
  retval = of_register_platform_driver(&xil_mailbox_driver);
  if(retval)
    {
      printk(KERN_ERR "%s: Can't register driver\n",
	     DRIVER_NAME);
      return retval;
    }
  return 0;
}

static void __exit xil_mailbox_exit(void)
{
  unregister_chrdev_region(MKDEV(major,0),DEVICES_COUNT);
  of_unregister_platform_driver(&xil_mailbox_driver);
}

module_init(xil_mailbox_init);
module_exit(xil_mailbox_exit);

MODULE_AUTHOR("alexb@meyersound.com");
MODULE_DESCRIPTION("Xilinx mailbox");
MODULE_LICENSE("GPL v2");
