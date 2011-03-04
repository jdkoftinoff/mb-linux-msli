/*
 *  linux/drivers/net/labx_avb/labx_dma.h
 *
 *  Lab X Technologies DMA coprocessor
 *
 *  Written by Chris Wulff (chris.wulff@labxtechnologies.com)
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

#ifndef _LABX_DMA_H_
#define _LABX_DMA_H_

#include <linux/ioport.h>
#include <linux/miscdevice.h>
#include <linux/labx_dma_coprocessor_defs.h>

#define NAME_MAX_SIZE  256

/* Maximum number of status packets which can be pending */
#define MAX_STATUS_PACKETS (8)

/* DMA structure (for use inside other drivers that include DMA) */
#define NO_IRQ_SUPPLIED   (-1)
struct labx_dma {
  void __iomem  *virtualAddress;

  /* Pointer to the enclosing device's name */
  char *name;

  /* Bit shift for the address sub-range */
  uint32_t regionShift;

  /* Capabilites from the CAPS registers */
  DMACapabilities capabilities;

  /* Wait queue for putting threads to sleep */
  wait_queue_head_t syncedWriteQueue;

  /* Interrupt request number */
  int32_t irq;

  /* Circular buffer of status packets */
  spinlock_t statusMutex;
  DMAStatusPacket *statusHead;
  DMAStatusPacket *statusTail;
  DMAStatusPacket  statusPackets[MAX_STATUS_PACKETS];
};

/* DMA Platform device structure */
struct labx_dma_pdev {
  /* Misc device */
  struct miscdevice miscdev;

  /* Pointer back to the platform device */
  struct platform_device *pdev;

  char          name[NAME_MAX_SIZE];

  /* Physical and virtual base address */
  uint32_t      physicalAddress;
  uint32_t      addressRangeSize;

  /* DMA structure */
  struct labx_dma dma;
};

/* DMA device probe */
extern int labx_dma_probe(struct labx_dma *dma);

/* DMA ioctl processing */
extern int labx_dma_ioctl(struct labx_dma* dma, unsigned int command, unsigned long arg);

#define DMA_REGISTER_RANGE 0
#define DMA_MICROCODE_RANGE 1

#define DMA_REGISTER_ADDRESS(dma, offset)                    \
  ((uintptr_t)(dma)->virtualAddress |                          \
   (DMA_REGISTER_RANGE << (dma)->regionShift) | ((offset) << 2))

#define DMA_MICROCODE_BASE(dma)                 \
  ((uintptr_t)(dma)->virtualAddress |           \
   (DMA_MICROCODE_RANGE << (dma)->regionShift))

/* Register address and control field #defines */
#define DMA_CONTROL_REG                 0x00
  #define DMA_DISABLE  0x00000000
  #define DMA_ENABLE   0x00000001

#define DMA_CHANNEL_ENABLE_REG          0x01

#define DMA_CHANNEL_START_REG           0x02

#define DMA_IRQ_ENABLE_REG              0x03
#define DMA_IRQ_FLAGS_REG               0x04
#  define DMA_NO_IRQS                (0x00000000)
#  define DMA_SYNC_IRQ               (0x80000000)
#  define DMA_STAT_OVRFLW_IRQ        (0x40000000)
#  define DMA_STAT_READY_IRQ         (0x20000000)
#  define DMA_CHAN_IRQ(whichChannel) (0x00000001 << whichChannel)
#  define DMA_ALL_IRQS               (0xFFFFFFFF)

#define DMA_SYNC_REG                    0x05
#  define DMA_CANCEL_SYNC      (0x00000000)
#  define DMA_SYNC_NEXT_WRITE  (0x00000001)
#  define DMA_SYNC_PENDING     (0x80000000)

#define DMA_STATUS_FIFO_FLAGS_REG       0x06
#  define DMA_STATUS_FIFO_FULL    (0x00000010)
#  define DMA_STATUS_FIFO_EMPTY   (0x00000008)
#  define DMA_STATUS_READ_POPPED  (0x00000004)
#  define DMA_STATUS_FIFO_BEGIN   (0x00000002)
#  define DMA_STATUS_FIFO_END     (0x00000001)

#define DMA_STATUS_FIFO_DATA_REG        0x07

#define DMA_CAPABILITIES_REG            0x7E
#  define DMA_CAPS_STATUS_FIFO_BIT           0x010000
#  define DMA_CAPS_INDEX_SHIFT               12
#  define DMA_CAPS_INDEX_MASK                0x0F
#  define DMA_CAPS_CHANNELS_SHIFT            10
#  define DMA_CAPS_CHANNELS_MASK             0x03
#  define DMA_CAPS_ALU_SHIFT                 8
#  define DMA_CAPS_ALU_MASK                  0x03
#  define DMA_CAPS_PARAM_ADDRESS_BITS_SHIFT  4
#  define DMA_CAPS_PARAM_ADDRESS_BITS_MASK   0x0F
#  define DMA_CAPS_CODE_ADDRESS_BITS_SHIFT   0
#  define DMA_CAPS_CODE_ADDRESS_BITS_MASK    0x0F

#define DMA_REVISION_REG                0x7F
#  define DMA_REVISION_FIELD_BITS  4
#  define DMA_REVISION_FIELD_MASK  0x0F

#define DMA_VECTORS_BASE_ADDRESS        0x80

#endif /* _LABX_DMA_H_ */

