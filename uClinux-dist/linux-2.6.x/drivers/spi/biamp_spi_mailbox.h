/*
 *  linux/drivers/spi/biamp_spi_mailbox.h
 *
 *  Biamp SPI mailbox peripheral driver
 *
 *  Written by Albert M. Hajjar (albert.hajjar@labxtechnologies.com)
 *
 *  Copyright (C) 2010 Lab X Technologies LLC, All Rights Reserved.
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

#ifndef _BIAMP_SPI_MAILBOX_H_
#define _BIAMP_SPI_MAILBOX_H_

#include <linux/cdev.h>
#include <linux/highmem.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/wait.h>

/* Macros for determining sub-addresses for address ranges and individual registers.
 * These are affected by the amount of address space devoted to packet template and 
 * microcode storage, which is hardware-configurable.
 */

#define REGISTER_RANGE      (0x0)
#define MSG_RAM_RANGE       (0x400)

/* Global control registers */
#define CONTROL_REG       	(0x000)
#  define MAILBOX_DISABLE      	 (0x00)
#  define MAILBOX_ENABLE      	 (0x01)
#  define HOST_MESSAGE_CONSUMED  (0x02)
#  define SPI_MSG_READY   	 (0x04)

#define IRQ_MASK_REG      	(0x001)
#define IRQ_FLAGS_REG     	(0x002)
#  define NO_IRQS      		(0x00000000)
#  define IRQ_S2H_MSG_RX     	(0x00000001)
#  define IRQ_S2H_MSG_TX  	(0x00000002)
#  define ALL_IRQS     		(0xFFFFFFFF)

#define HOST_MSG_LEN_REG        (0x003)

#define SPI_IRQ_FLAGS_REG    (0x004)

#define MAX_MAILBOX_MSG_BYTES (1024)

#define MESSAGE_READ_TIMEOUT_MSECS (1000)

#define REGISTER_ADDRESS(device, offset) \
  ((uintptr_t)device->virtualAddress |                       \
   (REGISTER_RANGE << device->regionShift) | (offset << 2))

#define MSG_RAM_BASE(device)              \
  ((uintptr_t)device->virtualAddress |           \
   (MSG_RAM_RANGE << device->regionShift))

/* Driver structure to maintain state for each device instance */
#define NAME_MAX_SIZE    (256)
#define NO_IRQ_SUPPLIED   (-1)
struct spi_mailbox {
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

  /* Wait queue for putting threads to sleep */
  wait_queue_head_t messageReadQueue;

  /* Mutex for the device instance */
  spinlock_t mutex;
  bool opened;
};

#endif
