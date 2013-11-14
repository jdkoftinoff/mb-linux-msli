/*
 *  include/linux/labx_tcp.h
 *
 *  Lab X Technologies TCP Accelerator
 *
 *  Written by Tom Bottom (tom.bottom@labxtechnologies.com)
 *
 *  Copyright (C) 2013 Lab X Technologies LLC, All Rights Reserved.
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

#ifndef _LABX_TCP_H_
#define _LABX_TCP_H_

#include <linux/ioport.h>
#include <linux/miscdevice.h>

#define NAME_MAX_SIZE  256

/* Constant allowing an encapsulating driver to tell us that it has
 * no IRQ resources for us to use
 */
#define TCP_NO_IRQ_SUPPLIED  (-1)

/* Similar constant to indicate that the encapsulating driver has no
 * idea how much microcode RAM an instance being probed has.  If probed
 * with this parameter, the driver will assume that the entire address
 * space mapped for microcode (as reported by the TCP capabilities register)
 * is backed with RAM.
 */
#define TCP_UCODE_SIZE_UNKNOWN  (-1)

/* Flag values for ISR -> Netlink thread handshaking */
#define TCP_STATUS_IDLE      (0)
#define TCP_NEW_STATUS_READY (1)

struct labx_tcp_accel_callbacks;

/* TCPA structure (for use inside other drivers that include TCPA) */
struct labx_tcpa {
  /* Virtual address pointer for the memory-mapped hardware */
  void __iomem  *virtualAddress;

  /* Pointer to the enclosing device's name */
  const char *name;

  /* Device node of the enclosing device */
  dev_t deviceNode;

  /* Bit shift for the address sub-range */
  uint32_t regionShift;

  /* Capabilites from the CAPS registers */
  tcpa_capabilities capabilities;

  /* Wait queue for putting userspace threads to sleep for synced writes */
  wait_queue_head_t syncedWriteQueue;

  /* Interrupt request number */
  int32_t irq;

  /* Callbacks used for overriding some functionality */
  struct labx_tcpa_callbacks *callbacks;
};

/* TCP Platform device structure */
struct labx_tcp_pdev {
  /* Misc device */
  struct miscdevice miscdev;

  /* Pointer back to the platform device */
  struct platform_device *pdev;

  char          name[NAME_MAX_SIZE];

  /* Physical and virtual base address */
  uint32_t      physicalAddress;
  uint32_t      addressRangeSize;

  /* TCP structure */
  struct labx_tcpa tcpa;
};

/* Callback functions for a TCP instance */
struct labx_tcpa_callbacks {
  void (*irqSetup)(struct labx_dma *dma);
  void (*irqTeardown)(struct labx_dma *dma);
};

/**
 * TCP device probe function
 *
 * @param tcpa           - TCP device structure to probe with
 * @param deviceMajor    - Major number of the enclosing device
 * @param deviceMinor    - Minor number of the enclosing device
 * @param name           - Pointer to the name of the enclosing device.  The memory this
 *                         points to must not be destroyed, e.g. the actual name buffer in
 *                         the enclosing driver's own device structure.
 * @param irq            - Interrupt request index to use; pass TCP_NO_IRQ_SUPPLIED if
 *                         a hardware interrupt is unavailable to the instance.  Some
 *                         capabilities may not be functional without an IRQ
 * @param tcpaCallbacks  - Callback functions for the instance being probed.
 */
extern int32_t labx_tcpa_probe(struct labx_tcpa *tcpa, 
                               uint32_t deviceMajor,
                               uint32_t deviceMinor,
                               const char *name, 
                               int32_t irq,
                               struct labx_tcpa_callbacks *tcpaCallbacks);

/* TCP open and release operations */
extern int32_t labx_tcpa_open(struct labx_tcpa *tcpa);
extern int32_t labx_tcpa_release(struct labx_tcpa *tcpa);

/* TCP ioctl processing */
extern int labx_tcpa_ioctl(struct labx_tcpa *tcpa, unsigned int command, unsigned long arg);

/* TCP Remove */
extern int32_t labx_tcpa_remove(struct labx_tcpa *tcpa);

#define TCP_REGISTER_RANGE 0
#define TCP_MICROCODE_RANGE 1

#define TCP_REGISTER_ADDRESS(tcpa, offset)                    \
  ((uintptr_t)(tcpa)->virtualAddress |                          \
   (TCP_REGISTER_RANGE << (tcpa)->regionShift) | ((offset) << 2))

#define TCP_MICROCODE_BASE(dma)                 \
  ((uintptr_t)(dma)->virtualAddress |           \
   (TCP_MICROCODE_RANGE << (dma)->regionShift))

/* Register address and control field #defines */
#define TCP_CONTROL_REG                 0x00
#  define TCP_DISABLE  0x00000000
#  define TCP_ENABLE   0x00000001

#define TCP_CHANNEL_ENABLE_REG          0x01
#  define TCP_CHANNELS_NONE  (0x00000000)

#define TCP_CHANNEL_START_REG           0x02

#define TCP_IRQ_ENABLE_REG              0x03
#define TCP_IRQ_FLAGS_REG               0x04
#  define TCP_NO_IRQS                (0x00000000)
#  define TCP_SYNC_IRQ               (0x80000000)
#  define TCP_STAT_OVRFLW_IRQ        (0x40000000)
#  define TCP_STAT_READY_IRQ         (0x20000000)
#  define TCP_CHAN_IRQ(whichChannel) (0x00000001 << whichChannel)
#  define TCP_ALL_IRQS               (0xFFFFFFFF)

#define TCP_SYNC_REG                    0x05
#  define TCP_CANCEL_SYNC      (0x00000000)
#  define TCP_SYNC_NEXT_WRITE  (0x00000001)
#  define TCP_SYNC_PENDING     (0x80000000)

#define TCP_STATUS_FIFO_FLAGS_REG       0x06
#  define TCP_STATUS_FIFO_FULL    (0x00000010)
#  define TCP_STATUS_FIFO_EMPTY   (0x00000008)
#  define TCP_STATUS_READ_POPPED  (0x00000004)
#  define TCP_STATUS_FIFO_BEGIN   (0x00000002)
#  define TCP_STATUS_FIFO_END     (0x00000001)

#define TCP_STATUS_FIFO_DATA_REG        0x07

#define TCP_CAPABILITIES_REG            0x7E
#  define TCP_CAPS_STATUS_FIFO_BIT           0x010000
#  define TCP_CAPS_INDEX_SHIFT               12
#  define TCP_CAPS_INDEX_MASK                0x0F
#  define TCP_CAPS_CHANNELS_SHIFT            10
#  define TCP_CAPS_CHANNELS_MASK             0x03
#  define TCP_CAPS_ALU_SHIFT                 8
#  define TCP_CAPS_ALU_MASK                  0x03
#  define TCP_CAPS_PARAM_ADDRESS_BITS_SHIFT  4
#  define TCP_CAPS_PARAM_ADDRESS_BITS_MASK   0x0F
#  define TCP_CAPS_CODE_ADDRESS_BITS_SHIFT   0
#  define TCP_CAPS_CODE_ADDRESS_BITS_MASK    0x0F

#define TCP_REVISION_REG                0x7F
#  define TCP_REVISION_FIELD_BITS  4
#  define TCP_REVISION_FIELD_MASK  0x0F

#define TCP_VECTORS_BASE_ADDRESS        0x80

#endif /* _LABX_TCP_H_ */

