/*
 *  linux/drivers/net/labx_avb/labx_mtd_bridge.c
 *
 *  Lab X Technologies MTD bridge Flash driver
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *
 *  Copyright (C) 2009 Lab X Technologies LLC, All Rights Reserved.
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

#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <xio.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif /* CONFIG_OF */


#if 0
#define DBG(f, x...) printk(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

#define DRIVER_NAME "labx_mtd_bridge"

#define	MAX_WAIT_JIFFIES (HZ / 4)

/* Driver structure to maintain state for each device instance */
#define NAME_MAX_SIZE    (256)
#define NO_IRQ_SUPPLIED   (-1)
struct mtd_bridge {
  /* Pointer back to the platform device */
  struct platform_device *pdev;

  /* MTD driver info structure */
	struct mtd_info mtd;

  /* Name for use in identification */
  char name[NAME_MAX_SIZE];

  /* Physical and virtual base address */
  uintptr_t      physicalAddress;
  uintptr_t      addressRangeSize;
  void __iomem  *virtualAddress;

  /* Interrupt request number */
  int32_t irq;

  /* Partitioned flag */
  uint32_t partitioned;

  /* Wait queue for putting threads to sleep */
  wait_queue_head_t queue;

  /* Mutex for the device instance */
  spinlock_t mutex;
};

/* MTD bridge register definitions */
#define UART_FIFO_READ_ADDR        0x000
#define UART_FIFO_WRITE_ADDR       0x001
#define UART_STATUS_REG_ADDR       0x002
#define UART_CTRL_REG_ADDR         0x003
#define MTDBRIDGE_IRQ_REG_ADDR     0x004
#define MTDBRIDGE_MASK_REG_ADDR    0x005
#define MTDBRIDGE_COMMAND_REG_ADDR 0x006
#define MTDBRIDGE_STATUS_REG_ADDR  0x007
#define MTDBRIDGE_ADDRESS_REG_ADDR 0x008
#define MTDBRIDGE_LENGTH_REG_ADDR  0x009
#define MTDBRIDGE_MAILBOX_RAM_ADDR 0x200

#define UART_STATUS_RX_DATA_BIT    (1 << 0)
#define UART_STATUS_RX_FULL_BIT    (1 << 1)
#define UART_STATUS_TX_EMPTY_BIT   (1 << 2)
#define UART_STATUS_TX_FULL_BIT    (1 << 3)
#define UART_STATUS_INT_EN_BIT     (1 << 4)
#define MTDBRIDGE_IRQ_COMPLETE     (1 << 0)
#define MTDBRIDGE_NO_IRQS          0
#define MTDBRIDGE_OPCODE_WRITE     0x02
#define MTDBRIDGE_OPCODE_READ      0x03
#define MTDBRIDGE_OPCODE_SE        0xD8

/* Status Register bits. */
#define	MTDBRIDGE_SR_OIP           1	/* Operation in progress */
#define	MTDBRIDGE_SR_WEL           2	/* Write enable latch */
/* meaning of other SR_* bits may differ between vendors */
#define MTDBRIDGE_SR_NORESP        0x04	/* No response from MTD bridge */
#define MTDBRIDGE_SR_RWERROR       0x08	/* Error in read/write operation on file */
#define	MTDBRIDGE_SR_UNMAPPED      0x10	/* Address specified in command is not mapped to a file */
#define	MTDBRIDGE_SR_RANGE_ERR     0x20	/* Specified block goes beyond mapped area end  */
#define	MTDBRIDGE_SR_RDONLY        0x40	/* Attempt to write or erase a read-only map */
#define MTDBRIDGE_SR_INVALID       0x80	/* Invalid MTD command */
#define MTDBRIDGE_BUFFER_SIZE      2048 /* Size of mtdbridge mailbox RAM */

#define REGISTER_ADDRESS(device, offset) ((uintptr_t)device->virtualAddress | (offset << 2))

/* Interrupt service routine for the instance */
static irqreturn_t labx_mtd_bridge_interrupt(int irq, void *dev_id) {
  struct mtd_bridge *bridge = (struct mtd_bridge*) dev_id;
  uint32_t maskedFlags;
  uint32_t irqMask;

  /* Read the interrupt flags and immediately clear them */
  maskedFlags = XIo_In32(REGISTER_ADDRESS(bridge, MTDBRIDGE_IRQ_REG_ADDR));
  irqMask = XIo_In32(REGISTER_ADDRESS(bridge, MTDBRIDGE_MASK_REG_ADDR));
  maskedFlags &= irqMask;
  XIo_Out32(REGISTER_ADDRESS(bridge, MTDBRIDGE_IRQ_REG_ADDR), maskedFlags);

  /* Detect the command completion IRQ */
  if((maskedFlags & MTDBRIDGE_IRQ_COMPLETE) != 0) {
    /* Wake up all threads waiting for a synchronization event */
    printk("<<COMPLETE>>\n");
    wake_up_interruptible(&(bridge->queue));
  }

  return(IRQ_HANDLED);
}

/* Helper function which issues a command to the MTD bridge and returns
 * the resulting status.  Timeouts are implemented.
 */
static int mtd_bridge_cmd(struct mtd_bridge *bridge, uint32_t offset, uint32_t len, uint32_t opcode) {
  unsigned long deadline;
	int32_t rc;

  /* Issue the requested command to the MTD bridge:
   *
   * * Clear the "operation complete" IRQ flag bit
   * * Set the Flash offset and length
   * * Trigger the operation by writing the opcode
   */
  XIo_Out32(REGISTER_ADDRESS(bridge, MTDBRIDGE_IRQ_REG_ADDR), MTDBRIDGE_IRQ_COMPLETE);
  XIo_Out32(REGISTER_ADDRESS(bridge, MTDBRIDGE_ADDRESS_REG_ADDR), offset);
  XIo_Out32(REGISTER_ADDRESS(bridge, MTDBRIDGE_LENGTH_REG_ADDR), len);
  XIo_Out32(REGISTER_ADDRESS(bridge, MTDBRIDGE_COMMAND_REG_ADDR), opcode);

  /* Poll until a response is received from the MTD bridge daemon, or we time out */
  rc       = -EFAULT;
	deadline = (jiffies + MAX_WAIT_JIFFIES);
  do {
    if((XIo_In32(REGISTER_ADDRESS(bridge, MTDBRIDGE_IRQ_REG_ADDR)) & MTDBRIDGE_IRQ_COMPLETE) != 0) {
      rc = 0;
      break;
    }
    cond_resched();
  } while(!time_after_eq(jiffies, deadline));

  /* Fetch the response if there was one, and loop waiting if the returned code was
   * "operation in progress", up until the timeout period.
   */
  if(rc == 0) {
    deadline = (jiffies + MAX_WAIT_JIFFIES);
    do {
      rc = XIo_In32(REGISTER_ADDRESS(bridge, MTDBRIDGE_STATUS_REG_ADDR));
      if((rc & MTDBRIDGE_SR_OIP) == 0) {
        break;
      }
      cond_resched();
    } while(!time_after_eq(jiffies, deadline));
  }

  if(rc != 0) printk("<<ERR %d!>>\n", rc);

	return rc;
}

/* MTD Flash operation routines */

static inline struct mtd_bridge *mtd_to_bridge(struct mtd_info *mtd)
{
	return container_of(mtd, struct mtd_bridge, mtd);
}

static int mtd_bridge_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct mtd_bridge *bridge = mtd_to_bridge(mtd);
	uint32_t rem;

	DEBUG(MTD_DEBUG_LEVEL2, "%s: %s %s 0x%llx, len %lld\n",
	      dev_name(&bridge->pdev->dev), __func__, "at",
	      (long long)instr->addr, (long long)instr->len);

	/* Sanity checks */
	if((instr->addr + instr->len) > bridge->mtd.size) return -EINVAL;
	div_u64_rem(instr->len, mtd->erasesize, &rem);
	if (rem) return -EINVAL;

  /* The sector erase command is monolithic - nothing needs to be broken into chunks. */
  return(mtd_bridge_cmd(bridge, instr->addr, instr->len, MTDBRIDGE_OPCODE_SE));
}

static int mtd_bridge_read(struct mtd_info *mtd, loff_t from, size_t len,
                           size_t *retlen, u_char *buf)
{
	struct mtd_bridge *bridge = mtd_to_bridge(mtd);
	int rc;
  size_t len_rem;
  size_t next_len;
  u32 *word_ptr;

  /* Loop, performing reads over the MTD bridge in chunks sized to match the
   * peripheral's memory buffer
   */
  rc       = 0;
  len_rem  = len;
  word_ptr = (u32*) buf;
  *retlen  = 0;
  while((rc == 0) & (len_rem > 0)) {
    /* Size the next request */
    next_len  = (len_rem > MTDBRIDGE_BUFFER_SIZE) ? MTDBRIDGE_BUFFER_SIZE : len_rem;
    len_rem  -= next_len;

    /* Issue a read command to the MTD bridge */
    rc = mtd_bridge_cmd(bridge, from, next_len, MTDBRIDGE_OPCODE_READ);
    
    /* If the read returned without an error code, copy bytes into the destination buffer */
    if(rc == 0) {
      int word_index;
      int word_len;
      int buf_offset = MTDBRIDGE_MAILBOX_RAM_ADDR;
    
      /* Transfer in 32-bit words */
      word_len = ((next_len + 3) >> 2);
      for(word_index = 0; word_index < word_len; word_index++) {
        *word_ptr++ = XIo_In32(REGISTER_ADDRESS(bridge, buf_offset));
        buf_offset++;
      }

      /* Advance the returned length */
      *retlen += next_len;
    }

    /* Advance the offset in Flash */
    from += next_len;
  } /* while(bytes left and no error) */

	return rc;
}

static int mtd_bridge_write(struct mtd_info *mtd, loff_t to, size_t len,
                            size_t *retlen, const u_char *buf)
{
	struct mtd_bridge *bridge = mtd_to_bridge(mtd);
	int rc;
  size_t len_rem;
  size_t next_len;
  u32 *word_ptr;

  /* Loop, performing reads over the MTD bridge in chunks sized to match the
   * peripheral's memory buffer
   */
  rc       = 0;
  len_rem  = len;
  word_ptr = (u32*) buf;
  *retlen  = 0;
  while((rc == 0) & (len_rem > 0)) {
    int word_index;
    int word_len;
    int buf_offset;

    /* Size the next request */
    next_len  = (len_rem > MTDBRIDGE_BUFFER_SIZE) ? MTDBRIDGE_BUFFER_SIZE : len_rem;
    len_rem  -= next_len;

    /* Copy the next buffer's worth of data to the bridge peripheral */
    buf_offset = MTDBRIDGE_MAILBOX_RAM_ADDR;
    word_len = ((next_len + 3) >> 2);
    for(word_index = 0; word_index < word_len; word_index++) {
      XIo_Out32(REGISTER_ADDRESS(bridge, buf_offset), *word_ptr++);
      buf_offset++;
    }

    /* Issue a write command to the MTD bridge */
    rc = mtd_bridge_cmd(bridge, to, next_len, MTDBRIDGE_OPCODE_WRITE);
    if(rc == 0) *retlen += next_len;

    /* Advance the offset in Flash */
    to += next_len;
  } /* while(bytes left and no error) */

	return rc;
}

/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 * This is exported to allow polymorphic drivers to invoke it.
 * @param name - Name of the instance
 * @param pdev - Platform device structure
 * @param addressRange - Resource describing the hardware's I/O range
 * @param irq          - Resource describing the hardware's IRQ
 * @param derivedFops  - File operations to delegate to, NULL if unused
 * @param derivedData  - Pointer for derived driver to make use of
 * @param newInstance  - Pointer to the new driver instance, NULL if unused
 */
int mtd_bridge_probe(const char *name, 
                     struct platform_device *pdev,
                     struct resource *addressRange,
                     struct resource *irq) {
  struct mtd_bridge *bridge;
  int returnValue;
  int i;

  /* Create and populate a device structure */
  bridge = (struct mtd_bridge*) kmalloc(sizeof(struct mtd_bridge), GFP_KERNEL);
  if(!bridge) return(-ENOMEM);
  memset(bridge, 0, sizeof(struct mtd_bridge));

  /* Request and map the device's I/O memory region into uncacheable space */
  bridge->physicalAddress = addressRange->start;
  bridge->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
  snprintf(bridge->name, NAME_MAX_SIZE, "%s", name);
  bridge->name[NAME_MAX_SIZE - 1] = '\0';
  if(request_mem_region(bridge->physicalAddress, bridge->addressRangeSize,
                        bridge->name) == NULL) {
    returnValue = -ENOMEM;
    goto free;
  }

  bridge->virtualAddress = 
    (void*) ioremap_nocache(bridge->physicalAddress, bridge->addressRangeSize);
  if(!bridge->virtualAddress) {
    returnValue = -ENOMEM;
    goto release;
  }

  /* Ensure that the interrupts are disabled */
  XIo_Out32(REGISTER_ADDRESS(bridge, MTDBRIDGE_MASK_REG_ADDR), MTDBRIDGE_NO_IRQS);

  /* Retain the IRQ and register our handler, if an IRQ resource was supplied. */
  if(irq != NULL) {
    bridge->irq = irq->start;
    returnValue = request_irq(bridge->irq, &labx_mtd_bridge_interrupt, IRQF_DISABLED, 
                              bridge->name, bridge);
    if (returnValue) {
      printk(KERN_ERR "%s : Could not allocate Lab X Audio Bridge interrupt (%d).\n",
             bridge->name, bridge->irq);
      goto unmap;
    }
  } else bridge->irq = NO_IRQ_SUPPLIED;

  /* Announce the device */
  printk(KERN_INFO "%s: Found Lab X MTD Bridge at 0x%08X, IRQ %d\n", 
         bridge->name, (uint32_t) bridge->physicalAddress, bridge->irq);

  /* Initialize other resources */
  spin_lock_init(&bridge->mutex);

  /* Provide navigation between the device structures */
  platform_set_drvdata(pdev, bridge);
  bridge->pdev = pdev;

  /* Initialize the waitqueue used for synchronized writes */
  init_waitqueue_head(&(bridge->queue));

  /* Now that the device is configured, enable interrupts if they are to be used */
  if(bridge->irq != NO_IRQ_SUPPLIED) {
    printk("<< TEMPORARY - Not enabling MTD bridge IRQ >>\n");
    /*    XIo_Out32(REGISTER_ADDRESS(bridge, MTDBRIDGE_IRQ_REG_ADDR), MTDBRIDGE_IRQ_COMPLETE); */
    /*    XIo_Out32(REGISTER_ADDRESS(bridge, MTDBRIDGE_MASK_REG_ADDR), MTDBRIDGE_IRQ_COMPLETE); */
  }

  /* Initialize the MTD driver structure and register the device as a memory chip */
  bridge->mtd.name       = "mtdbridge0";
  bridge->mtd.type       = MTD_NORFLASH;
  bridge->mtd.writesize  = 1;
	bridge->mtd.flags      = MTD_CAP_NORFLASH;
	bridge->mtd.size       = (128 * 1024 * 128);  /* Hard code!  128x128KiB sectors = 16 MiB */
	bridge->mtd.erase      = mtd_bridge_erase;
	bridge->mtd.read       = mtd_bridge_read;
	bridge->mtd.write      = mtd_bridge_write;
  bridge->mtd.erasesize  = (128 * 1024);  /* Hard code!  128 KiB sector size */
	bridge->mtd.dev.parent = &pdev->dev;

	DEBUG(MTD_DEBUG_LEVEL2,
		"mtd .name = %s, .size = 0x%llx (%lldMiB) "
			".erasesize = 0x%.8x (%uKiB) .numeraseregions = %d\n",
		bridge->mtd.name,
		(long long)bridge->mtd.size, (long long)(bridge->mtd.size >> 20),
		bridge->mtd.erasesize, bridge->mtd.erasesize / 1024,
		bridge->mtd.numeraseregions);

	if (bridge->mtd.numeraseregions)
		for (i = 0; i < bridge->mtd.numeraseregions; i++)
			DEBUG(MTD_DEBUG_LEVEL2,
				"mtd.eraseregions[%d] = { .offset = 0x%llx, "
				".erasesize = 0x%.8x (%uKiB), "
				".numblocks = %d }\n",
				i, (long long)bridge->mtd.eraseregions[i].offset,
				bridge->mtd.eraseregions[i].erasesize,
				bridge->mtd.eraseregions[i].erasesize / 1024,
				bridge->mtd.eraseregions[i].numblocks);


	/* partitions should match sector boundaries; and it may be good to
	 * use readonly partitions for writeprotected sectors (BP2..BP0).
	 */
	if (mtd_has_partitions()) {
		struct mtd_partition	*parts = NULL;
		int			nr_parts = 0;

		if (mtd_has_cmdlinepart()) {
			static const char *part_probes[]
					= { "cmdlinepart", NULL, };

			nr_parts = parse_mtd_partitions(&bridge->mtd,
					part_probes, &parts, 0);
		}

		if (nr_parts > 0) {
			for (i = 0; i < nr_parts; i++) {
				DEBUG(MTD_DEBUG_LEVEL2, "partitions[%d] = "
					"{.name = %s, .offset = 0x%llx, "
						".size = 0x%llx (%lldKiB) }\n",
					i, parts[i].name,
					(long long)parts[i].offset,
					(long long)parts[i].size,
					(long long)(parts[i].size >> 10));
			}
			bridge->partitioned = 1;
			return add_mtd_partitions(&bridge->mtd, parts, nr_parts);
		}
	}

  /* Map the device into the MTD layer */
  returnValue = add_mtd_device(&bridge->mtd);
  if (returnValue) {
    printk(KERN_ERR "%s : Could not map MTD bridge as MTD device\n", bridge->name);
    goto free_irq;
  }

  return 0;

 free_irq:
  if(bridge->irq != NO_IRQ_SUPPLIED) free_irq(bridge->irq, bridge);
 unmap:
  iounmap(bridge->virtualAddress);
 release:
  release_mem_region(bridge->physicalAddress, bridge->addressRangeSize);
 free:
  kfree(bridge);
  return(returnValue);
}

#ifdef CONFIG_OF
static int mtd_bridge_platform_remove(struct platform_device *pdev);

/* Probe for registered devices */
static int __devinit mtd_bridge_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
  struct resource r_mem_struct = {};
  struct resource r_irq_struct = {};
  struct resource *addressRange = &r_mem_struct;
  struct resource *irq          = &r_irq_struct;
  struct platform_device *pdev  = to_platform_device(&ofdev->dev);
  const char *name = dev_name(&ofdev->dev);
  int rc = 0;

  /* Obtain the resources for this instance */
  rc = of_address_to_resource(ofdev->node, 0, addressRange);
  if(rc) {
    dev_warn(&ofdev->dev, "Invalid address\n");
    return(rc);
  }

  rc = of_irq_to_resource(ofdev->node, 0, irq);
  if(rc == NO_IRQ) {
    /* No IRQ was defined; null the resource pointer to indicate polled mode */
    irq = NULL;
    return(rc);
  }

  /* Dispatch to the generic function */
  return(mtd_bridge_probe(name, pdev, addressRange, irq));
}

static int __devexit mtd_bridge_of_remove(struct of_device *dev)
{
  struct platform_device *pdev = to_platform_device(&dev->dev);
  mtd_bridge_platform_remove(pdev);
  return(0);
}


/* Directly compatible with Lab X MTD Bridge peripherals. */
static struct of_device_id bridge_of_match[] = {
  { .compatible = "xlnx,lawo-mtd-bridge-1.00.a", },
  { /* end of list */ },
};


static struct of_platform_driver of_mtd_bridge_driver = {
  .name		     = DRIVER_NAME,
  .match_table = bridge_of_match,
  .probe   	   = mtd_bridge_of_probe,
  .remove      = __devexit_p(mtd_bridge_of_remove),
};
#endif

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int mtd_bridge_platform_probe(struct platform_device *pdev) {
  struct resource *addressRange;
  struct resource *irq;

  /* Obtain the resources for this instance */
  addressRange = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!addressRange) {
    printk(KERN_ERR "%s: IO resource address not found.\n", pdev->name);
    return(-ENXIO);
  }

  /* Attempt to obtain the IRQ; if none is specified, the resource pointer is
   * NULL, and polling will be used.
   */
  irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

  /* Dispatch to the generic function */
  return(mtd_bridge_probe(pdev->name, pdev, addressRange, irq));
}

/* This is exported to allow polymorphic drivers to invoke it. */
int mtd_bridge_remove(struct mtd_bridge *bridge) {
  if(bridge->irq != NO_IRQ_SUPPLIED) free_irq(bridge->irq, bridge);
  iounmap(bridge->virtualAddress);
  release_mem_region(bridge->physicalAddress, bridge->addressRangeSize);
  kfree(bridge);
  return 0;
}

static int mtd_bridge_platform_remove(struct platform_device *pdev) {
  struct mtd_bridge *bridge;

  /* Get a handle to the bridge and begin shutting it down */
  bridge = platform_get_drvdata(pdev);
  if(!bridge) return(-1);
  return(mtd_bridge_remove(bridge));
}

/* Platform device driver structure */
static struct platform_driver mtd_bridge_driver = {
  .probe  = mtd_bridge_platform_probe,
  .remove = mtd_bridge_platform_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __init mtd_bridge_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": MTD Bridge driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Lab X Technologies, LLC\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&of_mtd_bridge_driver);
#endif
 
  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&mtd_bridge_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }
  return 0;
}

static void __exit mtd_bridge_driver_exit(void)
{
  /* Unregister as a platform device driver */
  platform_driver_unregister(&mtd_bridge_driver);
}

module_init(mtd_bridge_driver_init);
module_exit(mtd_bridge_driver_exit);

MODULE_AUTHOR("Eldridge M. Mount IV <eldridge.mount@labxtechnologies.com>");
MODULE_DESCRIPTION("Lab X Technologies MTD bridge driver");
MODULE_LICENSE("GPL");
