/*
 *  linux/include/linux/labx_mailbox_defs.h
 *
 *  LABX UHI mailbox definitions
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

#ifndef _LABX_MAILBOX_DEFS_H_
#define _LABX_MAILBOX_DEFS_H_

#include <linux/ioctl.h>

/* This header file contains common driver definitions for any LABX
 * UHI mailbox peripheral.  Use of these definitions permits the use
 * of a common base class in the userspace abstraction libraries.
 */

#define MBOX_IOC_CHAR          ('e')

/* I/O control commands and structures common to all packet engines */
#define IOC_START_MBOX           _IO(MBOX_IOC_CHAR, 0x01)
#define IOC_STOP_MBOX            _IO(MBOX_IOC_CHAR, 0x02)

#define MAX_MESSAGE_DATA 2048
typedef struct {
  uint32_t  length;
  uint8_t *messageContent;
} MessageData;

#define READ_UHI_STATUS          _IOR(MBOX_IOC_CHAR, 0x03, uint32_t)

/* Mailbox events Generic Netlink family name, version, and multicast groups */
#define MAILBOX_EVENTS_FAMILY_NAME     "MAILBOX_EVTS"
#define MAILBOX_EVENTS_FAMILY_VERSION  1
#define MAILBOX_EVENTS_GROUP           "MessageGroup"

/* Constant enumeration for Netlink event commands from the audio depacketizer driver */
enum {
  MAILBOX_EVENTS_C_RECEIVE_MESSAGE,
  MAILBOX_EVENTS_C_RESPONSE_MESSAGE,
  __MAILBOX_EVENTS_C_MAX,
};
#define MAILBOX_EVENTS_C_MAX (__MAILBOX_EVENTS_C_MAX - 1)

/* Netlink family attributes */
enum {
  MAILBOX_EVENTS_A_MINOR,
  MAILBOX_EVENTS_A_RECV_MESSAGE,
  MAILBOX_EVENTS_A_RESP_MESSAGE,
  MAILBOX_EVENTS_A_MESSAGE_LENGTH,
  __MAILBOX_EVENTS_A_MAX,
};
#define MAILBOX_EVENTS_A_MAX (__MAILBOX_EVENTS_A_MAX - 1)

#endif
