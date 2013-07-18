/*
 * meyer_smpte.c 
 *
 * Copyright (c) 2011 Meyer Sound Laboratories, Inc.
 *
 * Driver for Meyer Sound SMPTE interface.
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
   this driver supports up to 16 smpte interfaces
*/
#define DRIVER_NAME "meyer-smpte"
#define DEVICES_COUNT 16
#define SMPTE_DATA_BUF_SIZE 1024

/* Control register (RW) */
#define SMPTE_REG_CTRL        0x00

/* Monitor registers (RO) */
#define SMPTE_REG_T_SEC       0x04
#define SMPTE_REG_DATA_HI     0x08
#define SMPTE_REG_DATA_LO     0x0c
#define SMPTE_REG_T_NSEC      0x10

/* Generator control registers (WO) */
#define SMPTE_REG_GEN_OPTS    0x04
#define SMPTE_REG_GEN_LOAD_HI 0x10
#define SMPTE_REG_GEN_LOAD_LO 0x14

/* Control register bits */
#define SMPTE_BIT_CTRL_RX            0x80000000
#define SMPTE_BIT_CTRL_IE            0x40000000

/* Generator control register bits */
#define SMPTE_REG_GEN_OPTS_DECODE    0x01000000
#define SMPTE_REG_GEN_OPTS_LOAD      0x02000000
#define SMPTE_REG_GEN_OPTS_TIME_RUN  0x04000000
#define SMPTE_REG_GEN_OPTS_GEN_RUN   0x08000000
#define SMPTE_REG_GEN_OPTS_GEN_LEVEL 0x0000ffff
#define SMPTE_REG_GEN_OPTS_DROP      0x10000000
#define SMPTE_REG_GEN_OPTS_RATE      0xe0000000
#define SMPTE_REG_GEN_OPTS_24_00     0x00000000
#define SMPTE_REG_GEN_OPTS_25_00     0x20000000
#define SMPTE_REG_GEN_OPTS_30_00     0x40000000
#define SMPTE_REG_GEN_OPTS_23_97     0x60000000
#define SMPTE_REG_GEN_OPTS_29_97     0x80000000

static unsigned major=0,ndevs=0;
static unsigned char minors[DEVICES_COUNT];
static DEFINE_SPINLOCK(static_dev_lock);

/* device data structure */
struct smpte_dev
{
  void __iomem *base;
  unsigned int irq;
  spinlock_t lock;
  //struct semaphore sem;
  struct cdev cdev;
  unsigned int minor;
  struct smpte_client *clients;

  int inuse;

};

struct smpte_data
{
  u32 smpte_data_hi;
  u32 smpte_data_lo;
  u64 gptp_sec;
  u32 gptp_nsec;
}__attribute__((packed));

#define SMPTE_WRITE_AREA_SIZE 14

struct smpte_client
{
  struct smpte_dev *dev;
  struct semaphore sem;
  wait_queue_head_t wait_read_queue;
  struct smpte_client *next;
  struct smpte_data databuffer[SMPTE_DATA_BUF_SIZE];
  int read_index;
  int write_index;
  int read_offset;
  loff_t wr_offset_start;
  /* unformatted write area */
  char wr_buffer[SMPTE_WRITE_AREA_SIZE];
  /* data from 14-byte write area */
  u8 wr_opt; /* written to offset 0 */
  u8 wr_cmd; /* written ro offset 1 */
  u32 wr_level; /* written to offsets 2-5 */
  u32 wr_data_hi; /* written to offsets 6-9 */
  u32 wr_data_lo; /* written to offsets 10-13 */
  u32 cmd_gen;
};


/* file operations */
static int smpte_dev_open(struct inode *inode, struct file *filp)
{
  unsigned long flags;
  struct smpte_client *client,**client_prev_ptr;
  filp->private_data=kzalloc(sizeof(struct smpte_client), GFP_KERNEL);
  if(filp->private_data==NULL)
    return -ENOMEM;

  client=(struct smpte_client*)filp->private_data;
  client->cmd_gen=SMPTE_REG_GEN_OPTS_GEN_LEVEL&0xaaaa;
  client->dev=container_of(inode->i_cdev,struct smpte_dev,cdev);
  init_MUTEX(&client->sem);
  init_waitqueue_head(&client->wait_read_queue);
  //down(&client->dev->sem);
  spin_lock_irqsave(client->dev->lock,flags);
  client_prev_ptr=&client->dev->clients;
  while(*client_prev_ptr)
    client_prev_ptr=&(*client_prev_ptr)->next;
  *client_prev_ptr=client;
  client->dev->inuse++;
  spin_unlock_irqrestore(client->dev->lock,flags);
  //up(&client->dev->sem);
  return 0;
}

static int smpte_dev_release(struct inode *inode, struct file *filp)
{
  unsigned long flags;
  struct smpte_client *client,**client_prev_ptr;
  if(filp->private_data)
    {
      client=(struct smpte_client*)filp->private_data;
      down(&client->sem);
      spin_lock_irqsave(client->dev->lock,flags);
      client_prev_ptr=&client->dev->clients;
      while(*client_prev_ptr&&*client_prev_ptr!=client)
	client_prev_ptr=&(*client_prev_ptr)->next;
      if(*client_prev_ptr==client)
	*client_prev_ptr=client->next;
      else
	printk(KERN_ERR "meyer-smpte: inconsistent list of clients\n");
      client->dev->inuse--;
      spin_unlock_irqrestore(client->dev->lock,flags);
      up(&client->sem);
      kfree(client);
    }
  return 0;
}

static unsigned smpte_dev_poll(struct file *filp, 
				 struct poll_table_struct *wait)
{
  struct smpte_client *client;
  unsigned long flags;
  unsigned mask=0;

  if(filp->private_data==NULL)
    return 0;
  client=(struct smpte_client*)filp->private_data;
  down(&client->sem);
  poll_wait(filp,&client->wait_read_queue,wait);
  if(client->read_offset)
    mask|=POLLIN|POLLRDNORM;
  else
    {
    spin_lock_irqsave(&client->dev->lock,flags);
    if(client->read_index!=client->write_index)
      mask|=POLLIN|POLLRDNORM;
    spin_unlock_irqrestore(&client->dev->lock,flags);
    }
  up(&client->sem);
  return mask;
}

static ssize_t smpte_dev_read(struct file *filp, char __user *buf,
			      size_t count,loff_t *offset)
{
  unsigned long flags;
  struct smpte_client *client;
  size_t orig_count,copy_count_1,copy_count_2;
  int records_count,records_available,data_left,copy_start,copy_end;

  orig_count=count;

  if(filp->private_data==NULL)
    return 0;

  client=(struct smpte_client*)filp->private_data;

  /*
    check for offset not aligned to struct smpte_data boundaries -- if it is
    nonzero, there must be some immediately available buffered data
    
    this only requires semaphore because it does not affect anything modified
    in interrupt handler
  */
  if(down_interruptible(&client->sem))
    return -ERESTARTSYS;
  
  if(client->read_offset)
    {
      data_left=sizeof(struct smpte_data)-client->read_offset;
      if(count<data_left)
	{
	  if(copy_to_user(buf,
			  ((char*)&client->databuffer[client->read_index])
			  +client->read_offset,count))
	    {
	      up(&client->sem);
	      return -EFAULT;
	    }
	  client->read_offset+=count;
	  up(&client->sem);
	  return count;
	}
      else
	{
	  if(copy_to_user(buf,
			  ((char*)&client->databuffer[client->read_index])
			  +client->read_offset,data_left))
	    {
	      up(&client->sem);
	      return -EFAULT;
	    }
	  count-=data_left;
	  buf+=data_left;
	  client->read_offset=0;
	  client->read_index=(client->read_index+1)
	    %SMPTE_DATA_BUF_SIZE;
	}
    }
  
  while(count>0)
    {
      /* number of full records and size of partial record at the end */
      records_count=count/sizeof(struct smpte_data);
      data_left=count%sizeof(struct smpte_data);
      
      /* lock here, to handle buffer indexes */
      spin_lock_irqsave(client->dev->lock,flags);
      
      /* number of records available in the ring buffer */
      records_available=client->write_index-client->read_index;
      if(records_available<0) records_available+=SMPTE_DATA_BUF_SIZE;
      
      /* truncate if necessary */
      if(records_available<=records_count)
	{
	  data_left=0;
	  records_count=records_available;
	}
      
      copy_start=client->read_index*sizeof(struct smpte_data);
      copy_end=copy_start+records_count*sizeof(struct smpte_data)+data_left;
      
      if(copy_end>sizeof(struct smpte_data)*SMPTE_DATA_BUF_SIZE)
	{
	  copy_count_1=sizeof(struct smpte_data)*SMPTE_DATA_BUF_SIZE
	    -copy_start;
	  copy_count_2=copy_end-sizeof(struct smpte_data)*SMPTE_DATA_BUF_SIZE;
	}
      else
	{
	  copy_count_1=copy_end-copy_start;
	  copy_count_2=0;
	}
      
      if(copy_to_user(buf,
		      ((char*)&client->databuffer[client->read_index]),
		      copy_count_1))
	{
	  spin_unlock_irqrestore(client->dev->lock,flags);
	  up(&client->sem);
	  return -EFAULT;
	}
      count-=copy_count_1;
      buf+=copy_count_1;
      
      if(copy_count_2)
	{
	  if(copy_to_user(buf,
			  ((char*)&client->databuffer[0]),copy_count_2))
	    {
	      spin_unlock_irqrestore(client->dev->lock,flags);
	      up(&client->sem);
	      return -EFAULT;
	    }
	  count-=copy_count_2;
	  buf+=copy_count_2;
	  
	  client->read_index=copy_count_2/sizeof(struct smpte_data);
	  client->read_offset=copy_count_2%sizeof(struct smpte_data);
	}
      else
	{
	  client->read_index+=copy_count_1/sizeof(struct smpte_data);
	  client->read_offset=copy_count_1%sizeof(struct smpte_data);
	  if(client->read_index>=SMPTE_DATA_BUF_SIZE)
	    client->read_index=0;
	}
      /* available and requested data is received, unlock everything */
      spin_unlock_irqrestore(client->dev->lock,flags);
      
      /* 
	 if at this point there is still space in user's buffer, return from
	 nonblocking read operation or wait in blocking one
      */
      if(count)
	{
	  up(&client->sem);
	  if(filp->f_flags & O_NONBLOCK)
	    return orig_count-count;
	  else
	    {
	      /* wait for pointers to move */
	      if(wait_event_interruptible(client->wait_read_queue,
					  (client->read_index!=
					   client->write_index)))
		return -EINTR;
	      
	      if(down_interruptible(&client->sem))
		return -EINTR;
	    }
	}
    }
  up(&client->sem);
  return orig_count-count;
}

static ssize_t smpte_dev_write(struct file *filp, const char __user *buf,
			      size_t count,loff_t *offset)
{
  unsigned long flags;
  struct smpte_client *client;
  size_t curr_count,count_orig;
  loff_t curr_offset;
  u32 level;

  if(filp->private_data==NULL)
    return 0;

  client=(struct smpte_client*)filp->private_data;

  /*
    this only requires semaphore because it does not affect anything modified
    in interrupt handler
  */
  if(down_interruptible(&client->sem))
    return -ERESTARTSYS;

  count_orig=count;

  /* determine where writing starts and ends */
  curr_offset=*offset%SMPTE_WRITE_AREA_SIZE;

  while(count>0)
    {
      curr_count=(count<(SMPTE_WRITE_AREA_SIZE-curr_offset))?
	count:(SMPTE_WRITE_AREA_SIZE-curr_offset);

      if(client->wr_offset_start!=curr_offset)
	client->wr_offset_start=curr_offset;

      if(copy_from_user(&client->wr_buffer[curr_offset],buf,curr_count))
	{
	  client->wr_offset_start=0;
	  up(&client->sem);
	  return -EFAULT;
	}

      curr_offset+=curr_count;

      if(client->wr_offset_start==0&&curr_offset>=1)
	{
	  
	  /* options written */
	  client->cmd_gen&=~(SMPTE_REG_GEN_OPTS_RATE|SMPTE_REG_GEN_OPTS_DROP);
	  switch(client->wr_buffer[0])
	    {
	    case 8:
	      client->cmd_gen|=SMPTE_REG_GEN_OPTS_DROP;
	    case 0:
	      client->cmd_gen|=SMPTE_REG_GEN_OPTS_24_00;
	      break;
	    case 9:
	      client->cmd_gen|=SMPTE_REG_GEN_OPTS_DROP;
	    case 1:
	      client->cmd_gen|=SMPTE_REG_GEN_OPTS_25_00;
	      break;
	    case 10:
	      client->cmd_gen|=SMPTE_REG_GEN_OPTS_DROP;
	    case 2:
	      client->cmd_gen|=SMPTE_REG_GEN_OPTS_30_00;
	      break;
	    case 11:
	      client->cmd_gen|=SMPTE_REG_GEN_OPTS_DROP;
	    case 3:
	      client->cmd_gen|=SMPTE_REG_GEN_OPTS_23_97;
	      break;
	    case 12:
	      client->cmd_gen|=SMPTE_REG_GEN_OPTS_DROP;
	    case 4:
	      client->cmd_gen|=SMPTE_REG_GEN_OPTS_29_97;
	      break;
	    default:
	      break;
	    }
	}
      if(client->wr_offset_start<=1&&curr_offset>=2)
	{
	  /* command written */
	  switch(client->wr_buffer[1])
	    {
	    case 1:
	      /* start */
	      client->cmd_gen&=~SMPTE_REG_GEN_OPTS_DECODE;
	      client->cmd_gen|=(SMPTE_REG_GEN_OPTS_TIME_RUN
				|SMPTE_REG_GEN_OPTS_GEN_RUN);
	      break;
	    case 2:
	      /* pause */
	      client->cmd_gen&=~SMPTE_REG_GEN_OPTS_DECODE;
	      client->cmd_gen&=~SMPTE_REG_GEN_OPTS_TIME_RUN;
	      client->cmd_gen|=SMPTE_REG_GEN_OPTS_GEN_RUN;
	      break;
            case 3:
	      client->cmd_gen&=~(SMPTE_REG_GEN_OPTS_TIME_RUN
			 |SMPTE_REG_GEN_OPTS_GEN_RUN);
	      client->cmd_gen|=SMPTE_REG_GEN_OPTS_DECODE;
              break;
	    default:
	      /* stop */
	      client->cmd_gen&=~SMPTE_REG_GEN_OPTS_DECODE;
	      client->cmd_gen&=~(SMPTE_REG_GEN_OPTS_TIME_RUN
			 |SMPTE_REG_GEN_OPTS_GEN_RUN);
	      break;
	    }
	}
      if(client->wr_offset_start<=2&&curr_offset>=6)
	{
	  /* level written */
	  memcpy(&level,&client->wr_buffer[2],sizeof(level));
	  client->cmd_gen&=~SMPTE_REG_GEN_OPTS_GEN_LEVEL;
	  client->cmd_gen|=SMPTE_REG_GEN_OPTS_GEN_LEVEL&(level>>16);
	}
      client->cmd_gen&=~SMPTE_REG_GEN_OPTS_LOAD;
      if(client->wr_offset_start<=6&&curr_offset>=14)
	{
	  /* data written */
	  memcpy(&client->wr_data_hi,&client->wr_buffer[6],
		 sizeof(client->wr_data_hi));
	  memcpy(&client->wr_data_lo,&client->wr_buffer[10],
		 sizeof(client->wr_data_lo));
	  client->cmd_gen|=SMPTE_REG_GEN_OPTS_LOAD;
	}

      /* lock here, to handle I/O */
      spin_lock_irqsave(client->dev->lock,flags);

      if(client->cmd_gen&SMPTE_REG_GEN_OPTS_LOAD)
	{
	  XIo_Out32((unsigned int)client->dev->base+SMPTE_REG_GEN_LOAD_HI,
		    client->wr_data_hi);
	  XIo_Out32((unsigned int)client->dev->base+SMPTE_REG_GEN_LOAD_LO,
		    client->wr_data_lo);
	}
      XIo_Out32((unsigned int)client->dev->base+SMPTE_REG_GEN_OPTS,
		client->cmd_gen);

      spin_unlock_irqrestore(client->dev->lock,flags);

      if(curr_offset>=SMPTE_WRITE_AREA_SIZE)
	{
	  curr_offset=0;
	  client->wr_offset_start=0;
	}
      count-=curr_count;
      buf+=curr_count;
    }
  
  *offset=curr_offset;
  up(&client->sem);
  return count_orig;
}

static struct file_operations smpte_dev_fops =
  {
    .owner = THIS_MODULE,
    .poll = smpte_dev_poll,
    .read = smpte_dev_read,
    .write = smpte_dev_write,
    .open = smpte_dev_open,
    .release = smpte_dev_release,
  };

static irqreturn_t smpte_dev_irqhandler(int irq, void *p)
{
  unsigned long flags;
  struct smpte_dev *dev;
  struct smpte_client *client;
  int new_write_index;
  struct smpte_data data;
  u32 ctrlflags;

  dev=(struct smpte_dev*)p;

  spin_lock_irqsave(dev->lock,flags);

  /* read control register */
  ctrlflags=
    XIo_In32(((unsigned int)dev->base+SMPTE_REG_CTRL));
  if(ctrlflags&SMPTE_BIT_CTRL_RX)
    {
      /* read everything */
      data.smpte_data_hi=
	XIo_In32(((unsigned int)dev->base+SMPTE_REG_DATA_HI));
      data.smpte_data_lo=
	XIo_In32(((unsigned int)dev->base+SMPTE_REG_DATA_LO));
      data.gptp_sec=
	((u64)XIo_In32(((unsigned int)dev->base+SMPTE_REG_T_SEC)))
	|(((u64)(ctrlflags&0x0000ffff))<<32);
      data.gptp_nsec=
	XIo_In32(((unsigned int)dev->base+SMPTE_REG_T_NSEC));

      /* copy received data to the clients' buffers */
      for(client=dev->clients;client;client=client->next)
	{
	  new_write_index=(client->write_index+1)%SMPTE_DATA_BUF_SIZE;
	  if(new_write_index!=client->read_index)
	    {
	      memcpy(&client->databuffer[client->write_index],
		     &data,sizeof(data));
	      client->write_index=new_write_index;
	      wake_up_interruptible(&client->wait_read_queue);
	    }
	}
    }

  /* acknowledge interrupt */
  XIo_Out32(((unsigned int)dev->base+SMPTE_REG_CTRL),ctrlflags);
  spin_unlock_irqrestore(dev->lock,flags);
  return IRQ_HANDLED;
}

/* device initialization and exit */
static int __devinit smpte_dev_probe(struct of_device *ofdev,
				      const struct of_device_id *match)
{
  struct smpte_dev *dev;
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

  dev = kzalloc(sizeof(*dev), GFP_KERNEL);
  if(!dev)
    {
      printk(KERN_ERR "%s: Failed to allocate device data\n",
	     dev_name(&ofdev->dev));
      return -ENOMEM;
    }
  spin_lock_init(&dev->lock);
  spin_lock_irqsave(&static_dev_lock,flags);

  for(minor=0;minor<ndevs&&minors[minor];minor++);
  if(minor>=DEVICES_COUNT)
    {
      spin_unlock_irqrestore(&static_dev_lock,flags);
      printk(KERN_ERR "%s: %d devices already allocated\n",
	     dev_name(&ofdev->dev),DEVICES_COUNT);
      kfree(dev);
      return -ENODEV;
    }

  if(minor>=ndevs) ndevs=minor+1;
  minors[minor]=1;
  dev->minor=minor;

  spin_unlock_irqrestore(&static_dev_lock,flags);

  cdev_init(&dev->cdev,&smpte_dev_fops);
  dev->cdev.owner=THIS_MODULE;

  dev_set_drvdata(&ofdev->dev,dev);

  if(!request_mem_region(res_mem.start,resource_size(&res_mem),
			 dev_name(&ofdev->dev)))
    {
      printk(KERN_ERR "%s: I/O memory region busy\n",
	     dev_name(&ofdev->dev));
      kfree(dev);
      return -EBUSY;
    }

  dev->base = ioremap(res_mem.start,resource_size(&res_mem));
  if(!dev->base)
    {
      printk(KERN_ERR "%s: Unable to map I/O memory region\n",
	     dev_name(&ofdev->dev));
      release_mem_region(res_mem.start,resource_size(&res_mem));
      kfree(dev);
      return -EIO;
    }

  dev->irq=res_irq.start;

  //  init_MUTEX(&dev->sem);

  //init_waitqueue_head(&dev->wait_read_queue);
  //init_waitqueue_head(&dev->wait_write_queue);

  if(request_irq(dev->irq,smpte_dev_irqhandler,IRQF_DISABLED,
		 dev_name(&ofdev->dev),dev))
    {
      printk(KERN_ERR "%s: Unable to request interrupt\n",
	     dev_name(&ofdev->dev));
      iounmap(dev->base);
      release_mem_region(res_mem.start,resource_size(&res_mem));
      kfree(dev);
      return -EIO;
    }

  XIo_Out32((unsigned int)dev->base+SMPTE_REG_CTRL,0);

  XIo_Out32((unsigned int)dev->base+SMPTE_REG_GEN_LOAD_HI,0);
  XIo_Out32((unsigned int)dev->base+SMPTE_REG_GEN_LOAD_LO,0);
  XIo_Out32((unsigned int)dev->base+SMPTE_REG_GEN_OPTS,
	    SMPTE_REG_GEN_OPTS_LOAD|SMPTE_REG_GEN_OPTS_24_00);

  XIo_Out32((unsigned int)dev->base+SMPTE_REG_CTRL,
	    /*SMPTE_BIT_CTRL_RX|*/SMPTE_BIT_CTRL_IE);

  retval=cdev_add(&dev->cdev,
		  MKDEV(major,dev->minor),1);
  if(retval)
    {
      printk(KERN_ERR "%s: Unable to register device\n",
	     dev_name(&ofdev->dev));
      free_irq(dev->irq,dev);
      XIo_Out32((unsigned int)dev->base+SMPTE_REG_CTRL,0);
      iounmap(dev->base);
      release_mem_region(res_mem.start,resource_size(&res_mem));
      kfree(dev);
      return -ENODEV;
    }

  printk(KERN_INFO
	 "%s: Meyer Sound SMPTE interface initialized, device (%d,%d)\n",
	 dev_name(&ofdev->dev),major,dev->minor);


#if 0
  unsigned int ctrlflags,statflags;
  u32 data_hi,data_lo,tstamp;

  ctrlflags=XIo_In32(((unsigned int)dev->base
		      +SMPTE_REG_CTRL));
  statflags=XIo_In32(((unsigned int)dev->base
		      +SMPTE_REG_STAT));
  data_hi=XIo_In32(((unsigned int)dev->base
		    +SMPTE_REG_DATA_HI));
  data_lo=XIo_In32(((unsigned int)dev->base
		    +SMPTE_REG_DATA_LO));
  tstamp=XIo_In32(((unsigned int)dev->base
		      +SMPTE_REG_T_NSEC));
  //#ifdef DEBUG
  printk(KERN_ALERT "Initial SMPTE time code 0x%08x%08x timestamp 0x%08x, ctrl 0x%08x stat 0x%08x\n",data_hi,data_lo,tstamp,ctrlflags,statflags);
  //#endif
  /* acknowledge interrupts */
  ctrlflags|=SMPTE_BIT_CTRL_RX/*|SMPTE_BIT_CTRL_IE*/;
  XIo_Out32(((unsigned int)dev->base
	     +SMPTE_REG_CTRL),ctrlflags);
#endif
  return 0;
}

static int __devexit smpte_dev_remove(struct of_device *ofdev)
{
  struct smpte_dev *dev;
  struct resource res_mem;
  unsigned long flags;

  dev = dev_get_drvdata(&ofdev->dev);

  spin_lock_irqsave(&static_dev_lock,flags);
  minors[dev->minor]=0;
  XIo_Out32((unsigned int)dev->base+SMPTE_REG_CTRL,0);
  XIo_Out32((unsigned int)dev->base+SMPTE_REG_GEN_LOAD_HI,0);
  XIo_Out32((unsigned int)dev->base+SMPTE_REG_GEN_LOAD_LO,0);
  XIo_Out32((unsigned int)dev->base+SMPTE_REG_GEN_OPTS,
	    SMPTE_REG_GEN_OPTS_LOAD|SMPTE_REG_GEN_OPTS_24_00);

  spin_unlock_irqrestore(&static_dev_lock,flags);
  cdev_del(&dev->cdev);
  free_irq(dev->irq,dev);
  iounmap(dev->base);

  if(!of_address_to_resource(ofdev->node,0,&res_mem))
    release_mem_region(res_mem.start,resource_size(&res_mem));
  kfree(dev);

  return 0;
}

/* openfirmware data structures */
static const struct of_device_id smpte_dev_match[] =
  {
    { .compatible = "xlnx,smpte-decoder-1.00.a" },
    { .compatible = "xlnx,smpte-generator-1.00.a" },
    {}
  };

static struct of_platform_driver smpte_dev_driver =
  {
    .name = DRIVER_NAME,
    .match_table = smpte_dev_match,
    .probe = smpte_dev_probe,
    .remove = __devexit_p(smpte_dev_remove)
  };


/* driver initialization and exit */
static int __init smpte_dev_init(void)
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
  retval = of_register_platform_driver(&smpte_dev_driver);
  if(retval)
    {
      printk(KERN_ERR "%s: Can't register driver\n",
	     DRIVER_NAME);
      return retval;
    }
  return 0;
}

static void __exit smpte_dev_exit(void)
{
  unregister_chrdev_region(MKDEV(major,0),DEVICES_COUNT);
  of_unregister_platform_driver(&smpte_dev_driver);
}

module_init(smpte_dev_init);
module_exit(smpte_dev_exit);

MODULE_AUTHOR("alexb@meyersound.com");
MODULE_DESCRIPTION("Meyer Sound SMPTE interface");
MODULE_LICENSE("GPL v2");
