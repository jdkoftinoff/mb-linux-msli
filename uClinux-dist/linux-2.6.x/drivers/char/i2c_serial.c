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
#include <linux/xilinx_devices.h>

#include "xbasic_types.h"
#include "xiic.h"
#include "xiic_i.h"


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
static DECLARE_MUTEX(cfg_sem);
static bool resources_loaded;

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

void loopback_test(struct i2c_serial * i2c_serial_data) {
#if 0
    bool wakeup;
    wakeup=false;
    while(
          ((i2c_serial_data->tx_in&TX_MASK)!=(i2c_serial_data->tx_out&TX_MASK)) &&
          (((i2c_serial_data->rx_in+1)&RX_MASK)!=(i2c_serial_data->rx_out&RX_MASK)) ) {
            i2c_serial_data->rx_buffer[i2c_serial_data->rx_in&RX_MASK]=i2c_serial_data->tx_buffer[i2c_serial_data->tx_out&TX_MASK];
            printk(KERN_INFO "looping back [%02x]=[%02x]\n",
                i2c_serial_data->rx_buffer[i2c_serial_data->rx_in&RX_MASK],
                i2c_serial_data->tx_buffer[i2c_serial_data->tx_out&TX_MASK]);
            i2c_serial_data->rx_in++;
            i2c_serial_data->tx_out++;
            wakeup=true;
    }
    if(wakeup) {
        wake_up(&i2c_serial_data->rxq);
        wake_up(&i2c_serial_data->txq);
        printk(KERN_INFO "sent wake up!!! %p\n",&i2c_serial_data->rxq);
    }
#endif
    printk(KERN_INFO "poll -- rx_in:%d, rx_out:%d tx_in:%d, tx_out:%d\n",
        i2c_serial_data->rx_in,
        i2c_serial_data->rx_out,
        i2c_serial_data->tx_in,
        i2c_serial_data->tx_out);
}

/* Our private per device data. */
struct xiic_data {
//	struct i2c_adapter adap;	/* The Linux I2C core data  */
	int index;		/* index taken from platform_device */
	struct completion complete;	/* for waiting for interrupts */
	u32 base;		/* base memory address */
	unsigned int irq;	/* device IRQ number    */
    volatile u32 transmit_intr_flag;   /* semaphore across task and interrupt - ECM */
    volatile u32 receive_intr_flag;   /* semaphore across task and interrupt - ECM */
    volatile u32 status_intr_flag;   /* semaphore across task and interrupt - ECM */
	/*
	 * The underlying OS independent code needs space as well.  A
	 * pointer to the following XIic structure will be passed to
	 * any XIic_ function that requires it.  However, we treat the
	 * data as an opaque object in this file (meaning that we never
	 * reference any of the fields inside of the structure).
	 */
	XIic Iic;

	/*
	 * The following bit fields are used to keep track of what
	 * all has been done to initialize the xiic_dev to make
	 * error handling out of probe() easier.
	 */
	unsigned int reqirq:1;	/* Has request_irq() been called? */
	unsigned int remapped:1;	/* Has ioremap() been called? */
	unsigned int started:1;	/* Has XIic_Start() been called? */
	unsigned int added:1;	/* Has i2c_add_adapter() been called? */
};


static struct xiic_data * i2c_dev_data;
static uint8_t receive_buffer[1];
static uint8_t xxx[255];
static uint8_t arr[1];

/*
 * This routine is registered with the OS as the function to call when
 * the IIC interrupts.  It in turn, calls the Xilinx OS independent
 * interrupt function.  The Xilinx OS independent interrupt function
 * will in turn call any callbacks that we have registered for various
 * conditions.
 */
static irqreturn_t xiic_interrupt(int irq, void *dev_id)
{
	struct xiic_data *dev = dev_id;

	XIic_InterruptHandler(&dev->Iic);
	return IRQ_HANDLED;
}

static void RecvHandler(void *CallBackRef, int ByteCount) {
  struct i2c_serial * i2c_serial_data = (struct i2c_serial *)CallBackRef;
  bool wakeup;

  if(ByteCount==0) {
    i2c_serial_data->rx_buffer[i2c_serial_data->rx_in&RX_MASK]=receive_buffer[0];
    i2c_serial_data->rx_in++;
    wakeup=true;
  }
  if(wakeup) {
    wake_up(&i2c_serial_data->rxq);
  }
  printk(KERN_INFO "RecvHandler -- rx_in:%d, rx_out:%d tx_in:%d, tx_out:%d ByteCount:%d\n",
        i2c_serial_data->rx_in,
        i2c_serial_data->rx_out,
        i2c_serial_data->tx_in,
        i2c_serial_data->tx_out,
        ByteCount);
}

static void SendHandler(void *CallBackRef, int ByteCount) {
  struct i2c_serial * i2c_serial_data = (struct i2c_serial *)CallBackRef;
  bool wakeup;
  int i;

  if(XIic_IsIicBusy(&i2c_dev_data->Iic)) {
    return;
  }
//  if(ByteCount==0) {
      i=0;
      while((i2c_serial_data->tx_in&TX_MASK)!=(i2c_serial_data->tx_out&TX_MASK)) {
        xxx[i++]=i2c_serial_data->tx_buffer[i2c_serial_data->tx_out++&TX_MASK];
      }
      if(i>0) {
        XIic_MasterSend(&i2c_dev_data->Iic, xxx, i);
      }
//  }
}

uint8_t test;

static void StatusHandler(void * CallBackRef, int Event)
{
  struct i2c_serial * i2c_serial_data = (struct i2c_serial *)CallBackRef;
  int i;
  /*
   * Check whether the Event is to write or read the data from the slave.
   */
  if (Event == XII_ARB_LOST_EVENT) {
    XIic_WriteReg(i2c_dev_data->Iic.BaseAddress, XIIC_CR_REG_OFFSET,
             XIIC_CR_ENABLE_DEVICE_MASK);
    XIic_WriteIisr(i2c_dev_data->Iic.BaseAddress, XIIC_INTR_BNB_MASK);
    XIic_WriteIier(i2c_dev_data->Iic.BaseAddress, XIIC_INTR_BNB_MASK);
//    XIIC_WRITE_IISR(i2c_dev_data->Iic.BaseAddress, XIIC_INTR_BNB_MASK);
//    XIIC_WRITE_IIER(i2c_dev_data->Iic.BaseAddress, XIIC_INTR_BNB_MASK);
    i2c_dev_data->Iic.BNBOnly = TRUE;
  }
  else if (Event == XII_BUS_NOT_BUSY_EVENT) {
//    int8_t StatusRegister;
    if((i2c_serial_data->tx_in&TX_MASK)!=(i2c_serial_data->tx_out&TX_MASK)) { /* there is a byte to be sent */
      //XIic_WriteReg(i2c_dev_data->Iic.BaseAddress, XIIC_CR_REG_OFFSET, 0x0);
      //XIic_Start(&i2c_dev_data->Iic);
      i=0;
      while((i2c_serial_data->tx_out&TX_MASK)<(i2c_serial_data->tx_in&TX_MASK)) {
        xxx[i++]=i2c_serial_data->tx_buffer[i2c_serial_data->tx_out++&TX_MASK];
      }
      if(i>0) {
      XIic_MasterSend(&i2c_dev_data->Iic, xxx, i);
      }
    }
  }
  else if (Event == XII_MASTER_WRITE_EVENT) {
    /*
     * Its a Write request from Master.
     */
    XIic_SlaveRecv(&i2c_dev_data->Iic,receive_buffer,sizeof(receive_buffer));
  }
  else {
    /*
     * Its a Read request from the master.
     */
    arr[0]=test;
    XIic_SlaveSend(&i2c_dev_data->Iic, arr, sizeof(arr));
  }
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
  struct i2c_serial *i2c_serial_data;
  unsigned int i,j;
  char * ptr_dst;
  bool f;

  i2c_serial_data=(struct i2c_serial*)filp->private_data;

  if(down_interruptible(&i2c_serial_data->sem))
    return -ERESTARTSYS;

  i=0;
  while(i<count)
  {
    f=false;
    while((i<count) &&
          (((i2c_serial_data->tx_in+1)&TX_MASK)!=(i2c_serial_data->tx_out&TX_MASK)) &&
          (((i2c_serial_data->tx_in+2)&TX_MASK)!=(i2c_serial_data->tx_out&TX_MASK)))
    {
      ptr_dst=(char*)&i2c_serial_data->tx_buffer[i2c_serial_data->tx_in&TX_MASK];
      if(copy_from_user(ptr_dst,&buf[i],1))
      {
        up(&i2c_serial_data->sem);
        return -EFAULT;
      }
      i++;
      i2c_serial_data->tx_in++;
      printk(KERN_INFO "write [%02x]\n",*ptr_dst);
      loopback_test(i2c_serial_data);
      f=true;
    }
    if(!f)
    {
      up(&i2c_serial_data->sem);
      if(filp->f_flags & O_NONBLOCK)
        return i;
      else
      {
        break;
        if(wait_event_interruptible(i2c_serial_data->txq,
            ((((i2c_serial_data->tx_in+1)&TX_MASK)!=(i2c_serial_data->tx_out&TX_MASK)) &&
             (((i2c_serial_data->tx_in+2)&TX_MASK)!=(i2c_serial_data->tx_out&TX_MASK)))))
          return -EINTR;
        if(down_interruptible(&i2c_serial_data->sem))
          return -EINTR;
      }
    } else {

    if(!XIic_IsIicBusy(&i2c_dev_data->Iic)) {
      j=0;
      while((i2c_serial_data->tx_out&TX_MASK)!=(i2c_serial_data->tx_in&TX_MASK) && (j<sizeof(xxx))) {
        xxx[j++]=i2c_serial_data->tx_buffer[i2c_serial_data->tx_out++&TX_MASK];
      }
      XIic_MasterSend(&i2c_dev_data->Iic, xxx, j);
    }

        break;
    }
  }
  i2c_serial_data->write_count+=i;
  up(&i2c_serial_data->sem);
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


/** Shared device initialization code */
static int __devinit xilinx_iic_setup(
                struct device *device,
                struct device_node *node,
                struct resource *r_mem,
                struct resource *r_irq,
                u32 ten_bit_addr, 
                u32 gpo_width) {

    struct i2c_serial *i2c_serial_data;
    XIic_Config xiic_cfg;
    struct xiic_data *dev;
    char *scan_results;
    int error;

    error=0;
    i2c_serial_data = dev_get_drvdata(device);
    /* Allocate the dev and zero it out. */
    dev = kmalloc(sizeof(struct xiic_data), GFP_KERNEL);
    if (!dev) {
        printk(KERN_INFO "Cannot allocate struct xiic_data\n");
        error = -ENOMEM;
        goto out2;
    }
    memset(dev, 0, sizeof(struct xiic_data));

#if 0
    dev_set_drvdata(device, dev);
#else
    i2c_dev_data = dev;
#endif

    i2c_dev_data->irq = r_irq->start;

    /* initialize fields to satisfy i2c  */
    i2c_dev_data->index = 0;

    init_completion(&i2c_dev_data->complete);

    memset(&xiic_cfg, 0, sizeof(XIic_Config));
    xiic_cfg.DeviceId = 0;

    /* Change the addresses to be virtual; save the old ones to restore. */
    i2c_dev_data->base = r_mem->start;
    xiic_cfg.BaseAddress =
        (u32) ioremap(r_mem->start, r_mem->end - r_mem->start + 1);

    i2c_dev_data->remapped = 1;
    down(&cfg_sem);

    xiic_cfg.Has10BitAddr = (int)ten_bit_addr;
    xiic_cfg.GpOutWidth = (u8)gpo_width;

    /* Tell the Xilinx code to bring this IIC interface up. */
    if (XIic_CfgInitialize(&i2c_dev_data->Iic, &xiic_cfg, xiic_cfg.BaseAddress) !=
        XST_SUCCESS) {
        up(&cfg_sem);
        printk(KERN_INFO  "could not initialize device.\n");
        error = -ENODEV;
        goto out;
    }
    up(&cfg_sem);
    XIic_SetRecvHandler(&i2c_dev_data->Iic, (void *)i2c_serial_data, RecvHandler);
    XIic_SetSendHandler(&i2c_dev_data->Iic, (void *)i2c_serial_data, SendHandler);
    XIic_SetStatusHandler(&i2c_dev_data->Iic, (void *)i2c_serial_data, StatusHandler);
    XIic_SlaveInclude();
    XIic_MultiMasterInclude();

    XIic_SetAddress(&i2c_dev_data->Iic, XII_ADDR_TO_RESPOND_TYPE, 0x12);
    XIic_SetAddress(&i2c_dev_data->Iic, XII_ADDR_TO_SEND_TYPE, 0x32);


    /* Grab the IRQ */
    error = request_irq(i2c_dev_data->irq, xiic_interrupt, 0, dev_name(device), i2c_dev_data);
    if (error) {
        printk(KERN_INFO  "could not allocate interrupt %d.\n", i2c_dev_data->irq);
        goto out;
    }
    i2c_dev_data->reqirq = 1;
#if 0
#endif

    if (XIic_Start(&i2c_dev_data->Iic) != XST_SUCCESS) {
        printk(KERN_INFO  "could not start device\n");
        error = -ENODEV;
        goto out;
    }
    i2c_dev_data->started = 1;

    /* Now tell the core I2C code about our new device. */

#if 0
    strcpy(i2c_dev_data->adap.name, "i2c_serial");
    i2c_dev_data->adap.dev.of_node = node;
    i2c_dev_data->adap.algo = &xiic_algo;
    i2c_dev_data->adap.algo_data = NULL;
    i2c_dev_data->adap.timeout = XIIC_TIMEOUT;
    i2c_dev_data->adap.retries = XIIC_RETRY;
    error = i2c_add_adapter(&i2c_dev_data->adap);

    if (error) {
        printk(KERN_INFO  "could not add i2c adapter\n");
        goto out;
    }
    i2c_dev_data->added = 1;

#endif
    printk(KERN_INFO "%s #%d at 0x%08X mapped to 0x%08X, irq=%d\n",
           dev_name(device), i2c_dev_data->index,
           i2c_dev_data->base, (unsigned int)i2c_dev_data->Iic.BaseAddress, i2c_dev_data->irq);

#if 0
    if (scan) {
        scan_results = xilinx_iic_do_scan(dev);
        if (scan_results) {
            printk(scan_results);
            kfree(scan_results);
        }
    }
#endif
    
#if 0
    of_register_i2c_devices(&i2c_dev_data->adap, node);

    error = device_create_file(device, &dev_attr_scan);
      out:
    if (error)
        xilinx_iic_remove(device);
#endif
      out2:
    return error;
      out:
        return(0);
}

static u32 get_u32(struct of_device *ofdev, const char *s) {
  u32 *p = (u32 *)of_get_property(ofdev->node, s, NULL);
  if(p) {
    return *p;
  } else {
    dev_warn(&ofdev->dev, "Parameter %s not found, defaulting to false.\n", s);
    return FALSE;
  }
}


static int __devinit load_i2c_resources(struct of_device *ofdev) {
    u32 ten_bit_addr, gpo_width;
    struct resource r_irq_struct;
    struct resource r_mem_struct;

    struct resource *r_irq = &r_irq_struct; /* Interrupt resources */
    struct resource *r_mem = &r_mem_struct; /* IO mem resources */
    int rc = 0;
    int retdev;

    if(resources_loaded) {
        return 0;
    }

    printk(KERN_INFO "Device Tree Probing \'%s\'\n",
                        ofdev->node->name);

    /* Get iospace for the device */
    rc = of_address_to_resource(ofdev->node, 0, r_mem);
    if(rc) {
        dev_warn(&ofdev->dev, "invalid address\n");
        return rc;
    }

    /* Get IRQ for the device */
    rc = of_irq_to_resource(ofdev->node, 0, r_irq);
    if(rc == NO_IRQ) {
        dev_warn(&ofdev->dev, "no IRQ found.\n");
        return rc;
    }

    ten_bit_addr = get_u32(ofdev, "xlnx,ten-bit-adr");
    gpo_width = get_u32(ofdev, "xlnx,gpo-width");

    retdev = xilinx_iic_setup(&ofdev->dev, ofdev->node, r_mem, r_irq, ten_bit_addr, gpo_width);
    if(retdev==0) {
        resources_loaded=true;
    }
    return(retdev);
}

static int __devinit i2c_serial_probe(struct of_device *pdev,
                      const struct of_device_id *match)
{
    u32 ten_bit_addr, gpo_width;
    struct resource r_irq_struct;
    struct resource r_mem_struct;

    struct resource *r_irq = &r_irq_struct; /* Interrupt resources */
    struct resource *r_mem = &r_mem_struct; /* IO mem resources */

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

  retval=load_i2c_resources(pdev);

  printk(KERN_INFO "%s: i2c serial device initialized, device (%d,%d)\n",
         dev_name(&pdev->dev),major,i2c_serial_data->minor);

  return 0;
}


static int __devexit i2c_serial_remove(struct of_device *pdev)
{
  struct i2c_serial *i2c_serial_data;
  struct xiic_data *dev;

  dev=i2c_dev_data;

#if 0
  struct resource res;
  unsigned long flags;
#endif

  i2c_serial_data = dev_get_drvdata(&pdev->dev);


    /* Tell the Xilinx code to take this IIC interface down. */
    if (i2c_dev_data->started) {
        while (XIic_Stop(&i2c_dev_data->Iic) != XST_SUCCESS) {
            /* The bus was busy.  Retry. */
#if 0
            printk(KERN_WARNING
                   "%s #%d: Could not stop device.  Will retry.\n",
                   i2c_dev_data->adap.name, i2c_dev_data->index);
#endif
            set_current_state(TASK_INTERRUPTIBLE);
            schedule_timeout(HZ / 2);
        }
    }

    /*
     * Now that the Xilinx code isn't using the IRQ or registers,
     * unmap the registers and free the IRQ.
     */
    if (i2c_dev_data->remapped) {
        iounmap((void *)i2c_dev_data->Iic.BaseAddress);
    }

    if (i2c_dev_data->reqirq) {
        disable_irq(i2c_dev_data->irq);
        free_irq(i2c_dev_data->irq, dev);
    }

#if 0
    device_remove_file(device, &dev_attr_scan);
#endif
    kfree(dev);


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
  of_unregister_platform_driver(&i2c_serial_driver);
}

module_init(i2c_serial_init);
module_exit(i2c_serial_exit);

MODULE_AUTHOR("andrel@meyersound.com");
MODULE_DESCRIPTION("I2C Serial");
MODULE_LICENSE("GPL v2");
