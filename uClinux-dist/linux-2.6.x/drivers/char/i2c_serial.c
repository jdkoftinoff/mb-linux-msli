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
#include <linux/ctype.h>

#include "xbasic_types.h"
#include "xiic.h"
#include "xiic_i.h"

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINTK(...) printk( KERN_INFO __VA_ARGS__)
#else
#define DEBUG_PRINTK(...)
#endif


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
  uint8_t tx_buffer[1<<TX_BUFFER_BITS];

};

static unsigned major=0,ndevs=0;
static unsigned char minors[DEVICES_COUNT];
static DEFINE_SPINLOCK(static_dev_lock);
static DECLARE_MUTEX(cfg_sem);
static bool resources_loaded;
static struct i2c_serial * all_serial_data[DEVICES_COUNT+1];

const uint8_t address_vs_minor[DEVICES_COUNT] = {
        0x12+(0x10*1)+0,
        0x12+(0x10*1)+1,
        0x12+(0x10*1)+2,
        0x12+(0x10*2)+0,
        0x12+(0x10*2)+1,
        0x12+(0x10*2)+2,
        0x12+(0x10*3)+0,
        0x12+(0x10*3)+1,
        0x12+(0x10*3)+2
    };

#define ADDRESS_TO_I2C_ADDRESS(ADDRESS) ((ADDRESS&0x70)+2)

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

struct xiic_data {
     int index;          /* index taken from platform_device */
     struct completion complete;   /* for waiting for interrupts */
     u32 base;      /* base memory address */
     unsigned int irq;   /* device IRQ number    */
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
     unsigned int reqirq:1;   /* Has request_irq() been called? */
     unsigned int remapped:1; /* Has ioremap() been called? */
     unsigned int started:1;  /* Has XIic_Start() been called? */
};

static struct xiic_data * i2c_dev_data;
static uint8_t slot_temp[3];

static inline void isr_read_rx_fifo(uint32_t base_address) {
    static struct i2c_serial * i2c_serial_data;
    uint8_t ch,nibble,slot,address;
    uint8_t bytes_in_fifo;
    bytes_in_fifo = XIic_ReadReg(base_address, XIIC_RFO_REG_OFFSET) + 1;
    while(bytes_in_fifo-- && ((XIic_ReadReg(base_address, XIIC_SR_REG_OFFSET)&XIIC_SR_RX_FIFO_EMPTY_MASK)!=XIIC_SR_RX_FIFO_EMPTY_MASK)) {
        ch=XIic_ReadReg(base_address, XIIC_DRR_REG_OFFSET);
        nibble=(ch>>4)&0xf;
        slot=(ch>>2)&0x3;
        address=(ch>>0)&0x3;
        if(slot==3) { /* valid slots are 0,1,2 */
            printk( KERN_INFO "invalid slot\n");
            slot=0;
        }
        if(address==0) {
            slot_temp[slot] =(nibble<<4);
        } else {
            slot_temp[slot]|=nibble<<0;
            i2c_serial_data=all_serial_data[(slot*3)+address-1];
            i2c_serial_data->rx_buffer[i2c_serial_data->rx_in&RX_MASK]=slot_temp[slot];
            i2c_serial_data->rx_in++;
            wake_up(&i2c_serial_data->rxq);
        }
    }
}

static irqreturn_t xiic_interrupt(int irq, void *dev_id)
{
    struct xiic_data *dev = dev_id;
    uint32_t status;
    uint32_t interrupt_status;
    uint32_t clear;
    uint32_t base_address;
    static int tx_index;
    
    base_address=dev->Iic.BaseAddress;
    interrupt_status =  XIic_ReadIisr(base_address) & XIic_ReadIier(base_address);
    if(interrupt_status==0) {
        return IRQ_HANDLED;
    }
    clear=0;
    status = XIic_ReadReg(base_address, XIIC_SR_REG_OFFSET);
    if(interrupt_status & XIIC_INTR_ARB_LOST_MASK) {
        DEBUG_PRINTK("XIIC_INTR_ARB_LOST_MASK %d\n",tx_index);
        status = XIic_ReadReg(base_address, XIIC_CR_REG_OFFSET);
        XIic_WriteReg(base_address, XIIC_CR_REG_OFFSET, status | XIIC_CR_TX_FIFO_RESET_MASK);
        XIic_WriteReg(base_address, XIIC_CR_REG_OFFSET, status );
        while( all_serial_data[tx_index] && ((all_serial_data[tx_index]->tx_in&TX_MASK) != (all_serial_data[tx_index]->tx_out&TX_MASK)) ) {
            all_serial_data[tx_index]->tx_out++;
            wake_up(&all_serial_data[tx_index]->txq);
        }
#if 0
        wake_up(&all_serial_data[tx_index]->txq);
#endif
        XIic_DisableIntr(base_address,
                XIIC_INTR_ARB_LOST_MASK|
                XIIC_INTR_TX_ERROR_MASK|
                XIIC_INTR_TX_EMPTY_MASK|
                XIIC_INTR_TX_HALF_MASK);
        XIic_EnableIntr(i2c_dev_data->Iic.BaseAddress,
                XIIC_INTR_BNB_MASK);
        clear = XIIC_INTR_ARB_LOST_MASK;
    } else if(interrupt_status & XIIC_INTR_TX_ERROR_MASK) {
        DEBUG_PRINTK("XIIC_INTR_TX_ERROR_MASK %d\n",tx_index);
        status = XIic_ReadReg(base_address, XIIC_CR_REG_OFFSET);
        XIic_WriteReg(base_address, XIIC_CR_REG_OFFSET, status | XIIC_CR_TX_FIFO_RESET_MASK);
        XIic_WriteReg(base_address, XIIC_CR_REG_OFFSET, status );
        while( all_serial_data[tx_index] && ((all_serial_data[tx_index]->tx_in&TX_MASK) != (all_serial_data[tx_index]->tx_out&TX_MASK)) ) {
            all_serial_data[tx_index]->tx_out++;
            wake_up(&all_serial_data[tx_index]->txq);
        }
        XIic_DisableIntr(base_address,
                XIIC_INTR_ARB_LOST_MASK|
                XIIC_INTR_TX_ERROR_MASK|
                XIIC_INTR_TX_EMPTY_MASK|
                XIIC_INTR_TX_HALF_MASK);
        XIic_EnableIntr(base_address,
                XIIC_INTR_BNB_MASK);
        clear = XIIC_INTR_TX_ERROR_MASK;
    } else if(interrupt_status & XIIC_INTR_NAAS_MASK) {
        DEBUG_PRINTK("XIIC_INTR_NAAS_MASK %d\n",tx_index);
        isr_read_rx_fifo(base_address);
        XIic_DisableIntr(base_address,
                XIIC_INTR_NAAS_MASK|
                XIIC_INTR_RX_FULL_MASK);
        XIic_EnableIntr(base_address,
                XIIC_INTR_AAS_MASK|
                XIIC_INTR_BNB_MASK);
        clear = XIIC_INTR_NAAS_MASK;
    } else if(interrupt_status & XIIC_INTR_RX_FULL_MASK) {
        DEBUG_PRINTK("XIIC_INTR_RX_FULL_MASK %d\n",tx_index);
        if(status & XIIC_SR_ADDR_AS_SLAVE_MASK) {
            isr_read_rx_fifo(base_address);
        } else {
            printk( KERN_INFO "master rx full\n");
        }
        clear = XIIC_INTR_RX_FULL_MASK;
    } else if(interrupt_status & XIIC_INTR_AAS_MASK) {
        DEBUG_PRINTK("XIIC_INTR_AAS_MASK %d\n",tx_index);
        XIic_DisableIntr(base_address,
                XIIC_INTR_AAS_MASK|
                XIIC_INTR_BNB_MASK|
                XIIC_INTR_ARB_LOST_MASK|
                XIIC_INTR_TX_ERROR_MASK|
                XIIC_INTR_TX_EMPTY_MASK|
                XIIC_INTR_TX_HALF_MASK);
        XIic_EnableIntr(base_address,
                XIIC_INTR_RX_FULL_MASK|
                XIIC_INTR_NAAS_MASK);
        clear = XIIC_INTR_AAS_MASK;
    } else if(interrupt_status & (XIIC_INTR_TX_EMPTY_MASK|XIIC_INTR_TX_HALF_MASK)) {
        DEBUG_PRINTK("XIIC_INTR_TX_EMPTY_MASK %d\n",tx_index);
        while( all_serial_data[tx_index] && ((all_serial_data[tx_index]->tx_in&TX_MASK) != (all_serial_data[tx_index]->tx_out&TX_MASK)) ) {
            if((XIic_ReadReg(base_address,XIIC_TFO_REG_OFFSET)+1)!=IIC_TX_FIFO_DEPTH) {
                if( (all_serial_data[tx_index]->tx_in&TX_MASK) == ((all_serial_data[tx_index]->tx_out+1)&TX_MASK) ) { /* last byte */
                    DEBUG_PRINTK("XIIC_INTR_TX_EMPTY_MASK last %d\n",tx_index);
                    XIic_WriteReg(base_address, XIIC_DTR_REG_OFFSET,all_serial_data[tx_index]->tx_buffer[all_serial_data[tx_index]->tx_out&TX_MASK]|XIIC_TX_DYN_STOP_MASK);
                } else {
                    XIic_WriteReg(base_address, XIIC_DTR_REG_OFFSET,all_serial_data[tx_index]->tx_buffer[all_serial_data[tx_index]->tx_out&TX_MASK]);
                }
                all_serial_data[tx_index]->tx_out++;
            } else {
                break;
            }
        }
        if(all_serial_data[tx_index] && ((all_serial_data[tx_index]->tx_in&TX_MASK) == (all_serial_data[tx_index]->tx_out&TX_MASK)) ) {
            XIic_DisableIntr(base_address,
                    XIIC_INTR_ARB_LOST_MASK|
                    XIIC_INTR_TX_ERROR_MASK|
                    XIIC_INTR_TX_EMPTY_MASK|
                    XIIC_INTR_TX_HALF_MASK);
            wake_up(&all_serial_data[tx_index]->txq);
        }
        interrupt_status = XIic_ReadIisr(base_address);
        clear = interrupt_status & (XIIC_INTR_TX_EMPTY_MASK|XIIC_INTR_TX_HALF_MASK);
    } else if(interrupt_status & XIIC_INTR_BNB_MASK) {
        DEBUG_PRINTK("XIIC_INTR_BNB_MASK %d\n",tx_index);
#if 0
        isr_read_rx_fifo(base_address);
        XIic_DisableIntr(base_address,
                XIIC_INTR_NAAS_MASK|
                XIIC_INTR_RX_FULL_MASK);
        XIic_EnableIntr(base_address,
                XIIC_INTR_AAS_MASK);
        if(all_serial_data[tx_index]) {
            wake_up(&all_serial_data[tx_index]->txq);
        }
#endif
        for(tx_index=0;tx_index<DEVICES_COUNT;tx_index++) {
            if( (all_serial_data[tx_index]->tx_in&TX_MASK) != (all_serial_data[tx_index]->tx_out&TX_MASK) ) {
                DEBUG_PRINTK("XIIC_INTR_BNB_MASK start %d\n",tx_index);
                XIic_WriteReg(base_address, XIIC_DTR_REG_OFFSET,(ADDRESS_TO_I2C_ADDRESS(address_vs_minor[all_serial_data[tx_index]->minor%DEVICES_COUNT])<<1)&0xfe);
                XIic_WriteReg(base_address, XIIC_DTR_REG_OFFSET,address_vs_minor[all_serial_data[tx_index]->minor%DEVICES_COUNT]);
                XIic_ClearEnableIntr(base_address,
                    XIIC_INTR_ARB_LOST_MASK|
                    XIIC_INTR_TX_ERROR_MASK|
                    XIIC_INTR_TX_EMPTY_MASK|
                    XIIC_INTR_TX_HALF_MASK);
                XIic_WriteReg(base_address, XIIC_CR_REG_OFFSET, XIic_ReadReg(base_address, XIIC_CR_REG_OFFSET) | (XIIC_CR_MSMS_MASK | XIIC_CR_DIR_IS_TX_MASK));
                break;
            }
        }
        if(tx_index==DEVICES_COUNT) { /* no data to send */
            DEBUG_PRINTK("XIIC_INTR_BNB_MASK quit %d\n",tx_index);
            XIic_DisableIntr(base_address, XIIC_INTR_BNB_MASK);
        }
        clear = XIIC_INTR_BNB_MASK;
    }
    XIic_WriteIisr(base_address, clear);
    return IRQ_HANDLED;
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
  if((dev->rx_in&RX_MASK)!=(dev->rx_out&RX_MASK))
  {
    mask |= POLLIN | POLLRDNORM; /* readable */
  }
  if(((dev->tx_in+1)&TX_MASK)!=(dev->tx_out&TX_MASK))
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
  uint8_t * ptr_src;
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
      i++;
      dev->rx_out++;
      f=true;
    }

    if(!f)
    {
      up(&dev->sem);
      if(filp->f_flags & O_NONBLOCK) {
        return i;
      } else {
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
      }
    } else {
        break;
    }
  }
  up(&dev->sem);
  return(i);
}

static ssize_t i2c_serial_write(struct file *filp, const char __user *buf,
                                size_t count,loff_t *offset)
{
  struct i2c_serial *i2c_serial_data;
  unsigned int i;
  char * ptr_dst;
  bool f;

  i2c_serial_data=(struct i2c_serial*)filp->private_data;

  if(down_interruptible(&i2c_serial_data->sem))
    return -ERESTARTSYS;

  i=0;
  while(i<count)
  {
    f=false;
    while((i<count) && (((i2c_serial_data->tx_in+1)&TX_MASK)!=(i2c_serial_data->tx_out&TX_MASK)))
    {
      ptr_dst=(char*)&i2c_serial_data->tx_buffer[i2c_serial_data->tx_in&TX_MASK];
      if(copy_from_user(ptr_dst,&buf[i],1))
      {
        up(&i2c_serial_data->sem);
        return -EFAULT;
      }
      i++;
      i2c_serial_data->tx_in++;
      f=true;
    }
    if(!f)
    {
      up(&i2c_serial_data->sem);
      if(filp->f_flags & O_NONBLOCK)
        return i;
      else
      {
        if(wait_event_interruptible(i2c_serial_data->txq,((((i2c_serial_data->tx_in+1)&TX_MASK)!=(i2c_serial_data->tx_out&TX_MASK)))))
          return -EINTR;
        if(down_interruptible(&i2c_serial_data->sem))
          return -EINTR;
      }
    } else {
        XIic_EnableIntr(i2c_dev_data->Iic.BaseAddress, XIIC_INTR_BNB_MASK);
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

    i2c_dev_data = dev;

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


    i2c_dev_data->Iic.BaseAddress=xiic_cfg.BaseAddress;

    /* Tell the Xilinx code to bring this IIC interface up. */
    XIic_WriteReg(i2c_dev_data->Iic.BaseAddress, XIIC_RESETR_OFFSET, XIIC_RESET_MASK);

    up(&cfg_sem);

    XIic_WriteReg(i2c_dev_data->Iic.BaseAddress, XIIC_ADR_REG_OFFSET, 0x12<<1);
    XIic_WriteReg(i2c_dev_data->Iic.BaseAddress, XIIC_RFD_REG_OFFSET, IIC_RX_FIFO_DEPTH - 1);

    /* Grab the IRQ */
    error = request_irq(i2c_dev_data->irq, xiic_interrupt, 0, dev_name(device), i2c_dev_data);
    if (error) {
        printk(KERN_INFO  "could not allocate interrupt %d.\n", i2c_dev_data->irq);
        goto out;
    }
    i2c_dev_data->reqirq = 1;

    XIic_WriteIier(i2c_dev_data->Iic.BaseAddress, 0);
    XIic_ClearIntr(i2c_dev_data->Iic.BaseAddress, 0xFFFFFFFF);
    XIic_EnableIntr(i2c_dev_data->Iic.BaseAddress,
            XIIC_INTR_AAS_MASK);
    XIic_WriteReg(i2c_dev_data->Iic.BaseAddress, XIIC_CR_REG_OFFSET, XIIC_CR_ENABLE_DEVICE_MASK);
    XIic_IntrGlobalEnable(i2c_dev_data->Iic.BaseAddress);

    i2c_dev_data->started = 1;

    /* Now tell the core I2C code about our new device. */
    printk(KERN_INFO "%s #%d at 0x%08X mapped to 0x%08X, irq=%d\n",
           dev_name(device), i2c_dev_data->index,
           i2c_dev_data->base, (unsigned int)i2c_dev_data->Iic.BaseAddress, i2c_dev_data->irq);

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
  struct i2c_serial *i2c_serial_data;
  int retval;
  unsigned int minor;
  unsigned long flags;

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

  retval=cdev_add(&i2c_serial_data->cdev,
                  MKDEV(major,i2c_serial_data->minor),1);
  if(retval)
    {
      printk(KERN_ERR "%s: Unable to register device\n",
             dev_name(&pdev->dev));
      iounmap(i2c_serial_data->base);
      kfree(i2c_serial_data);
      return -ENODEV;
    }

  sema_init(&i2c_serial_data->sem,1);
  init_waitqueue_head(&i2c_serial_data->rxq);
  init_waitqueue_head(&i2c_serial_data->txq);

  retval=load_i2c_resources(pdev);

  if(minor<DEVICES_COUNT) {
    all_serial_data[i2c_serial_data->minor]=i2c_serial_data;
    all_serial_data[i2c_serial_data->minor+1]=NULL;
  }

  printk(KERN_INFO "%s: i2c serial device initialized, device (%d,%d)\n",
         dev_name(&pdev->dev),major,i2c_serial_data->minor);

  return 0;
}


static int __devexit i2c_serial_remove(struct of_device *pdev)
{
  struct i2c_serial *i2c_serial_data;
  struct xiic_data *dev;

  dev=i2c_dev_data;
  i2c_serial_data = dev_get_drvdata(&pdev->dev);

  /* Tell the Xilinx code to take this IIC interface down. */
  if (i2c_dev_data->started) {
      while(1) {
          XIic_IntrGlobalDisable(i2c_dev_data->Iic.BaseAddress);
          if((XIic_ReadReg(i2c_dev_data->Iic.BaseAddress, XIIC_CR_REG_OFFSET)&XIIC_CR_MSMS_MASK) || (XIic_ReadReg(i2c_dev_data->Iic.BaseAddress, XIIC_SR_REG_OFFSET)&XIIC_SR_ADDR_AS_SLAVE_MASK)) {
              /* The bus was busy.  Retry. */
              printk(KERN_WARNING "%s #%d: Could not stop device.  Will retry.\n", dev_name(&pdev->dev), i2c_dev_data->index);
              set_current_state(TASK_INTERRUPTIBLE);
              schedule_timeout(HZ / 2);
          } else {
              XIic_IntrGlobalEnable(i2c_dev_data->Iic.BaseAddress);
          }
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
  kfree(dev);

  cdev_del(&i2c_serial_data->cdev);

  printk(KERN_INFO "%s: removed i2c serial device %d:%d\n",
         dev_name(&pdev->dev),major,i2c_serial_data->minor);

  if(i2c_serial_data->minor<DEVICES_COUNT) {
    all_serial_data[i2c_serial_data->minor]=NULL;
  }
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
  unregister_chrdev_region(MKDEV(major,0),DEVICES_COUNT);
  of_unregister_platform_driver(&i2c_serial_driver);
}

module_init(i2c_serial_init);
module_exit(i2c_serial_exit);

MODULE_AUTHOR("andrel@meyersound.com");
MODULE_DESCRIPTION("I2C Serial");
MODULE_LICENSE("GPL v2");
