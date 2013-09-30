/*
 *  linux/include/linux/labx_aes3_rx_defs.h
 *
 *  Lab X Technologies AES3 RX definitions
 *
 *  Written by Yi Cao (yi.cao@labxtechnologies.com)
 *
 *  Copyright (C) 2012 Lab X Technologies LLC, All Rights Reserved.
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

#ifndef _LABX_AES3_RX_DEFS_H_
#define _LABX_AES3_RX_DEFS_H_

/* Constant and type definitions for the optional status FIFO */

/* Maximum size, in words, of a single status packet */
#define MAX_STATUS_PACKET_WORDS (15)

/* Maximum number of status packets which can be packed into a
 * single Netlink datagram
 */
#define MAX_STATUS_PACKETS_PER_DGRAM  (8)

/* Structure type definition for encapsulating a status packet */
typedef struct {
  uint32_t packetLength;
  uint32_t packetData[MAX_STATUS_PACKET_WORDS];
} AESStatusPacket;

/* Generic Netlink family name, version, and multicast groups for Lab X AES events */
#define LABX_AES_EVENTS_FAMILY_NAME     "LABX_AES_EVTS"
#define LABX_AES_EVENTS_FAMILY_VERSION  1
#define LABX_AES_EVENTS_STATUS_GROUP    "StatusGroup"

/* Constant enumeration for Netlink event commands from the Lab X AES driver */
enum {
  LABX_AES_EVENTS_C_UNSPEC,
  LABX_AES_EVENTS_C_AES_STATUS_PACKETS,
  LABX_AES_EVENTS_C_METER_STATUS_PACKETS,
  __LABX_AES_EVENTS_C_MAX,
};
#define LABX_AES_EVENTS_C_MAX (__LABX_AES_EVENTS_C_MAX - 1)

/* Netlink family attributes */
enum {
  LABX_AES_EVENTS_A_UNSPEC,
  LABX_AES_EVENTS_A_AES_DEVICE,
  LABX_AES_EVENTS_A_AES_RX_STATUS = 2,
  LABX_AES_EVENTS_A_AES_TX_STATUS,
  LABX_AES_EVENTS_A_METER_RX_STATUS = 2,
  LABX_AES_EVENTS_A_METER_TX_STATUS,
  __LABX_AES_EVENTS_A_MAX,
};
#define LABX_AES_EVENTS_A_MAX (__LABX_AES_EVENTS_A_MAX - 1)



#define IOC_CHAR                  ('d')
#define IOC_READ_RX_STREAM_STATUS _IOR(IOC_CHAR, 0x01, uint32_t)
#define IOC_READ_TX_STREAM_STATUS _IOR(IOC_CHAR, 0x02, uint32_t)
#define IOC_CONFIG_AES            _IOW(IOC_CHAR, 0x03, uint32_t)
#define IOC_CONFIG_RX_PCM_MODE    _IOW(IOC_CHAR, 0x04, uint32_t)
#define IOC_CONFIG_TX_PCM_MODE    _IOW(IOC_CHAR, 0x05, uint32_t)
#define IOC_CONFIG_2CHAN_MODE     _IOW(IOC_CHAR, 0x06, uint32_t)
#define IOC_READ_AES_MASK         _IOR(IOC_CHAR, 0x07, uint32_t)
#define IOC_SET_AES_MASK          _IOW(IOC_CHAR, 0x08, uint32_t)
#define IOC_READ_RX_METER_STATUS  _IOR(IOC_CHAR, 0x09, uint32_t)
#define IOC_READ_TX_METER_STATUS  _IOR(IOC_CHAR, 0x0A, uint32_t)


#endif /* _LABX_AES3_RX_DEFS_H_ */

