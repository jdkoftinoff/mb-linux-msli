/*
 *  linux/include/linux/biamp_spi_mailbox_defs.h
 *
 *  Biamp SPI mailbox definitions
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

#ifndef _BIAMP_SPIMAILBOX_DEFS_H_
#define _BIAMP_SPIMAILBOX_DEFS_H_

#include <linux/ioctl.h>

/* This header file contains common driver definitions for any Biamp
 * SPI mailbox peripheral.  Use of these definitions permits the use
 * of a common base class in the userspace abstraction libraries.
 */

#define MBOX_IOC_CHAR          ('e')

/* I/O control commands and structures common to all packet engines */
#define IOC_START_MBOX           _IO(MBOX_IOC_CHAR, 0x01)
#define IOC_STOP_MBOX            _IO(MBOX_IOC_CHAR, 0x02)

#define MAX_MESSAGE_DATA 1024
typedef struct {
  uint32_t  length;
  uint8_t *messageContent;
} MessageData;

#define IOC_READ_MBOX              _IOWR(MBOX_IOC_CHAR, 0x03, MessageData)
#define IOC_WRITE_RESP             _IOW(MBOX_IOC_CHAR,  0x04, MessageData)
#define IOC_SET_IRQ_FLAGS          _IOW(MBOX_IOC_CHAR,  0x05, uint32_t)

#endif
