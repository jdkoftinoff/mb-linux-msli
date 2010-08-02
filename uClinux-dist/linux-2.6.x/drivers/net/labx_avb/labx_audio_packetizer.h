/*
 *  linux/drivers/net/labx_avb/labx_audio_packetizer.h
 *
 *  Lab X Technologies AVB flexible audio packetizer driver
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

#ifndef _LABX_AUDIO_PACKETIZER_H_
#define _LABX_AUDIO_PACKETIZER_H_

#include <linux/cdev.h>
#include <linux/highmem.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <net/labx_avb/packet_engine_defs.h>

/* Macros for determining sub-addresses for address ranges and individual registers.
 * These are affected by the amount of address space devoted to packet template and 
 * microcode storage, which is hardware-configurable.
 */
#define REGISTER_RANGE      (0x0)
#define CLOCK_DOMAIN_RANGE  (0x1)
#define MICROCODE_RANGE     (0x2)
#define TEMPLATE_RANGE      (0x3)

/* Global control registers */
#define CONTROL_REG       (0x000)
#  define SHAPER_DISABLE      (0x00)
#  define SHAPER_ENABLE       (0x02)
#  define PACKETIZER_DISABLE  (0x00)
#  define PACKETIZER_ENABLE   (0x01)

#define START_VECTOR_REG  (0x001)

#define TS_OFFSET_REG     (0x002)

#define IRQ_MASK_REG      (0x003)
#define IRQ_FLAGS_REG     (0x004)
#  define NO_IRQS      (0x00000000)
#  define SYNC_IRQ     (0x00000001)
#  define OVERRUN_IRQ  (0x00000002)
#  define ALL_IRQS     (0x00000003)

#define SYNC_REG          (0x005)
#  define CANCEL_SYNC      (0x00000000)
#  define SYNC_NEXT_WRITE  (0x00000001)
#  define SYNC_PENDING     (0x80000000)

#define SEND_SLOPE_REG    (0x006)
#define IDLE_SLOPE_REG    (0x007)

#define CAPABILITIES_REG  (0x0FE)
#  define SHAPER_FRACT_BITS_SHIFT     (24)
#  define SHAPER_FRACT_BITS_MASK      (0x07F)
#  define CLOCK_DOMAINS_SHIFT         (16)
#  define CLOCK_DOMAINS_MASK          (0x0FF)
#  define TEMPLATE_ADDRESS_SHIFT      (8)
#  define TEMPLATE_ADDRESS_BITS_MASK  (0x0FF)
#  define CODE_ADDRESS_BITS_MASK      (0x0FF)

#define REVISION_REG      (0x0FF)
#  define REVISION_FIELD_BITS  4
#  define REVISION_FIELD_MASK  (0x0F)

/* Per-clock-domain registers */
#define REGS_PER_CLOCK_DOMAIN  2
#define TS_INTERVAL_REG    (0x000)
#define DOMAIN_ENABLE_REG  (0x001)

#define REGISTER_ADDRESS(device, offset) \
  ((uintptr_t)device->virtualAddress |                       \
   (REGISTER_RANGE << device->regionShift) | (offset << 2))

#define CLOCK_DOMAIN_REGISTER_ADDRESS(device, clockDomain, offset) \
  ((uintptr_t)device->virtualAddress |                              \
   (CLOCK_DOMAIN_RANGE << device->regionShift) |                    \
   (((clockDomain * REGS_PER_CLOCK_DOMAIN) + offset) << 2))

#define MICROCODE_RAM_BASE(device)              \
  ((uintptr_t)device->virtualAddress |           \
   (MICROCODE_RANGE << device->regionShift))

#define TEMPLATE_RAM_BASE(device)               \
  ((uintptr_t)device->virtualAddress |           \
   (TEMPLATE_RANGE << device->regionShift))

/* Driver structure to maintain state for each device instance */
#define NAME_MAX_SIZE    (256)
#define NO_IRQ_SUPPLIED   (-1)
struct audio_packetizer {
  /* Pointer back to the platform device */
  struct platform_device *pdev;

  /* Character device data */
  struct cdev cdev;
  dev_t       deviceNumber;
  uint32_t    instanceNumber;

  /* Name for use in identification */
  char name[NAME_MAX_SIZE];

  /* Physical and virtual base address */
  uintptr_t      physicalAddress;
  uintptr_t      addressRangeSize;
  void __iomem  *virtualAddress;

  /* Interrupt request number */
  int32_t irq;

  /* Bit shift for the address sub-range */
  uint32_t regionShift;

  /* Capabilities of the packetizer hardware */
  PacketizerCaps capabilities;

  /* Wait queue for putting threads to sleep */
  wait_queue_head_t syncedWriteQueue;

  /* Mutex for the device instance */
  spinlock_t mutex;
  bool opened;
};

#endif
