/*
 *  linux/arch/microblaze/platform/LabX/Mosaic/aes3_rx.h
 *
 *  Lab X Technologies AVB Lab X Mosaic AES receiver driver
 *
 *  Written by Yi Cao (yi.cao@labxtechnologies.com)
 *
 *  Copyright (C) 2012 Lab X Technologies, LLC, All Rights Reserved.
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
 
#ifndef _AES3_RX_H_
#define _AES3_RX_H_

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/highmem.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/ioctl.h>
#include <linux/labx_aes3_rx_defs.h>

    
/* Output channel select registers */ 
#define AES_STREAM_MASK_REG       (0x00)
#define AES_RX_STREAM_STATUS_REG  (0x01) 
#define AES_CONTROL_REG           (0x02)
#define AES_RX_PCM_MODE_REG       (0x03)
#define AES_TX_PCM_MODE_REG       (0x04)
#define AES_TX_2CHAN_MODE_REG     (0x05)
#define AES_TX_STREAM_STATUS_REG  (0x06)

/* Xilinx FPGA related */
#define REGISTER_ADDRESS(device, offset) \
  ((uintptr_t)device->virtualAddress | (offset << 2))

        
/* Flag values for ISR -> Netlink thread handshaking */
#define AES_STATUS_IDLE      (0)
#define AES_NEW_STATUS_READY (1)  
  
/* Driver structure to maintain state for each device instance */
#define NAME_MAX_SIZE    256   
struct aes3_rx {
  /* Pointer back to the platform device */
  struct platform_device *pdev;

  /* Version information read from the hardware */
  uint32_t versionMajor;
  uint32_t versionMinor;

  /* Character device data */
  struct cdev cdev;
  dev_t       deviceNode;
  uint32_t    instanceNumber;

  /* Name for use in identification */
  char name[NAME_MAX_SIZE];

  /* Physical and virtual base address */
  uintptr_t      physicalAddress;
  uintptr_t      addressRangeSize;
  void __iomem  *virtualAddress;

  /* Wait queue and other state for the Netlink thread */
  wait_queue_head_t statusFifoQueue;
  uint32_t netlinkSequence;
  struct task_struct *netlinkTask;
  uint32_t statusReady;
  
  /* Interrupt request number */
  int32_t irq;

  /* Mutex for the device instance */
  spinlock_t mutex;
  bool opened;

  /* File operations and private data for a polymorphic
   * driver to use
   */
  struct file_operations *derivedFops;
  void *derivedData;
};

#endif
