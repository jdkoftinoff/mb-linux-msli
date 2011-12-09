/*
 *  linux/drivers/char/labx_mailbox.h
 *
 *  LABX mailbox peripheral driver
 *
 *  Written by Albert M. Hajjar (albert.hajjar@labxtechnologies.com)
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

#ifndef _LABX_MAILBOX_H_
#define _LABX_MAILBOX_H_

#include <linux/highmem.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/labx_mailbox_defs.h>

/* Maximum number of mailboxes and instance count */
#define MAX_MAILBOX_DEVICES 2

/* Macros for determining sub-addresses for address ranges and individual registers.
 * These are affected by the amount of address space devoted to packet template and 
 * microcode storage, which is hardware-configurable.
 */

#define REGISTER_RANGE      (0x0)
#define MSG_RAM_RANGE       (0x200)

/* Global control registers */
#define SUPRV_CONTROL_REG        (0x000)
#  define MAILBOX_DISABLE      	  (0x00)
#  define MAILBOX_ENABLE      	  (0x01)
#  define ENABLE                  (0x01)

#define SUPRV_IRQ_MASK_REG   	      (0x001)
#define SUPRV_IRQ_FLAGS_REG    	      (0x002)
#define NO_IRQS      		 (0x00000000)
#define SUPRV_IRQ_0    	         (0x00000001)
#define SUPRV_IRQ_1    	         (0x00000002)
#define ALL_IRQS     		 (0xFFFFFFFF)

#define SUPRV_IRQ_FLAGS_REG      (0x002)

#define SUPRV_MSG_LEN_REG        (0x003)
#define HOST_MSG_LEN_REG         (0x003)  

#define SUPRV_TRIG_ASYNC_REG     (0x005)
#define MAX_MAILBOX_MSG_BYTES     (1024)

#define MESSAGE_NOT_READY (0)
#define MESSAGE_READY (1)

#define REGISTER_ADDRESS(device, offset) \
  ((uintptr_t)device->virtualAddress |                       \
   (REGISTER_RANGE) | (offset << 2))

#define MSG_RAM_BASE(device)              \
  ((uintptr_t)device->virtualAddress |           \
   (MSG_RAM_RANGE))

/* Driver structure to maintain state for each device instance */
#define NO_IRQ_SUPPLIED   (-1)
struct labx_mailbox {
  /* Pointer back to the platform device */
  struct platform_device *pdev;

  /* Name for use in identification */
  char name[NAME_MAX_SIZE];

  /* Physical and virtual base address */
  uintptr_t      physicalAddress;
  uintptr_t      addressRangeSize;
  void __iomem  *virtualAddress;

  /* Interrupt request number */
  int32_t irq;

  /* Mutex for the device instance */
  spinlock_t mutex;
  bool opened;
  
  /* Netlink events */
  wait_queue_head_t messageReadQueue;
  uint32_t messageReadyFlag;
  uint32_t netlinkSequence;
  struct task_struct *netlinkTask;
};

/* From labx_mailbox.c */
extern struct labx_mailbox* labx_mailboxes[MAX_MAILBOX_DEVICES];
extern void enable_mailbox(struct labx_mailbox *mailbox);
extern void disable_mailbox(struct labx_mailbox *mailbox);
extern void async_event_notify(struct labx_mailbox *mailbox);
extern struct labx_mailbox* get_instance(const char* name);


/* From labx_mailbox_netlink.c */
extern int register_mailbox_netlink(void);
extern void unregister_mailbox_netlink(void);
extern int mailbox_event_send_request(struct labx_mailbox *mailbox);
extern int create_mailbox_socket(void);
extern void destroy_mailbox_socket(void);

#endif
