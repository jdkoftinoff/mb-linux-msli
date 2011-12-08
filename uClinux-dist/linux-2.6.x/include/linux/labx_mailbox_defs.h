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

/* This header file contains common driver definitions for any LABX
 * UHI mailbox peripheral.  Use of these definitions permits the use
 * of a common base class in the userspace abstraction libraries.
 */

/* Constant and type definitions for the message mailbox */
#define NAME_MAX_SIZE    (256)

/* Structure type definition for encapsulating a message */
#define MAX_MESSAGE_DATA 1024
typedef struct {
  uint32_t  length;
  uint32_t *messageContent;
} MessageData;

/* Maximum size, in words, of a single message packet */
#define MAX_MESSAGE_PACKET_WORDS (256)

/* Mailbox events Generic Netlink family name, version, and multicast groups */
#define LABX_MAILBOX_EVENTS_FAMILY_NAME     "MAILBOX_EVTS"
#define LABX_MAILBOX_EVENTS_FAMILY_VERSION  1
#define LABX_MAILBOX_EVENTS_GROUP           "MessageGroup"

/* Constant enumeration for Netlink event commands from the mailbox driver */
enum {
  LABX_MAILBOX_EVENTS_C_ANNOUNCE_MESSAGE,
  LABX_MAILBOX_EVENTS_C_SHUTDOWN_MESSAGE,
  LABX_MAILBOX_EVENTS_C_REQUEST_MESSAGE,
  LABX_MAILBOX_EVENTS_C_RESPONSE_MESSAGE,
  LABX_MAILBOX_EVENTS_C_EVENT_QUEUE_READY,
  __LABX_MAILBOX_EVENTS_C_MAX,
};
#define LABX_MAILBOX_EVENTS_C_MAX (__LABX_MAILBOX_EVENTS_C_MAX - 1)

/* Constant enumeration defining a message packet */
enum {
  LABX_MAILBOX_MESSAGE_A_PACKET,
  LABX_MAILBOX_MESSAGE_PACKET_A_LENGTH,
  LABX_MAILBOX_MESSAGE_PACKET_A_WORDS,
  __LABX_MAILBOX_MESAGE_PACKET_A_MAX,
};
#define LABX_MAILBOX_MESSAGE_PACKET_A_MAX (__LABX_MAILBOX_MESSAGE_PACKET_A_MAX - 1)

/* Netlink family attributes */
enum {
  LABX_MAILBOX_EVENTS_A_UNSPEC,
  LABX_MAILBOX_EVENTS_A_MAILBOX_DEVICE,
  __LABX_MAILBOX_EVENTS_A_MAX,
};
#define LABX_MAILBOX_EVENTS_A_MAX (__LABX_MAILBOX_EVENTS_A_MAX - 1)

#endif
