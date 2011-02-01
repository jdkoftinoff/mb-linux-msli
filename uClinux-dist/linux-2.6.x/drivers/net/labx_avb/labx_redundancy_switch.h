/*
 *  linux/drivers/net/labx_avb/labx_redundancy_switch.h
 *
 *  Lab X Technologies AVB flexible audio packetizer driver
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *
 *  Copyright (C) 2011 Lab X Technologies LLC, All Rights Reserved.
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

#ifndef _LABX_REDUNDANCY_SWITCH_H_
#define _LABX_REDUNDANCY_SWITCH_H_

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/highmem.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <net/labx_avb/packet_engine_defs.h>

/* Register definitions */
#define CONTROL_REG       (0x000)
#  define SWITCH_ENABLE  (0x80000000)

#define CLOCK_STATUS_REG  (0x001)

#define IRQ_FLAGS_REG     (0x002)
#define IRQ_MASK_REG      (0x003)
#  define NO_IRQS       (0x00000000)
#  define BUMP_ACK_IRQ  (0x00000001)
#  define ALL_IRQS      (0x00000001)

/* The stream configuration register has no readback */
#define STREAM_CONFIG_REG    (0x004)

/* Stream status registers */
#define STREAM_STATUS_0_REG  (0x004)
#define STREAM_STATUS_1_REG  (0x005)
#define STREAM_STATUS_2_REG  (0x006)
#define STREAM_STATUS_3_REG  (0x007)

#define REVISION_REG      (0x00F)
#  define REVISION_FIELD_BITS  4
#  define REVISION_FIELD_MASK  (0x0F)

#define REGISTER_ADDRESS(device, offset) \
  ((uintptr_t)device->virtualAddress | (offset << 2))

/* Driver structure to maintain state for each device instance */
#define NAME_MAX_SIZE    (256)
#define NO_IRQ_SUPPLIED   (-1)
struct redundancy_switch {
  /* Pointer back to the platform device */
  struct platform_device *pdev;

  /* Version information read from the hardware */
  uint32_t versionMajor;
  uint32_t versionMinor;

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

  /* Wait queue for putting threads to sleep */
  wait_queue_head_t bumpWaitQueue;

  /* Mutex for the device instance */
  spinlock_t mutex;
  bool opened;

  /* File operations and private data for a polymorphic
   * driver to use
   */
  struct file_operations *derivedFops;
  void *derivedData;
};

/* Function prototypes for derived drivers to use */
int redundancy_switch_probe(const char *name, 
                            struct platform_device *pdev,
                            struct resource *addressRange,
                            struct resource *irq,
                            struct file_operations *derivedFops,
                            void *derivedData,
                            struct redundancy_switch **newInstance);

int redundancy_switch_remove(struct redundancy_switch *packetizer);

#endif
