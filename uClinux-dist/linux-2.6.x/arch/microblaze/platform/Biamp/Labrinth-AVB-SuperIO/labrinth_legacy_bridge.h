/*
 *  linux/drivers/net/labx_avb/labrinth_legacy_bridge.h
 *
 *  Legacy packet bridge configuration driver for Labrinth
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

#ifndef _LABRINTH_LEGACY_BRIDGE_H_
#define _LABRINTH_LEGACY_BRIDGE_H_

#include <linux/types.h>

/* Number of AVB bridge ports */
#define NUM_AVB_BRIDGE_PORTS 2

/* Address range definitions */
#define BRIDGE_REGS_BASE   (0x00000000)
#define LABX_MAC_REGS_BASE (0x00004000)

/* Port selection for within the bridge register space */
#define AVB_PORTS_BASE   (0x00000080)
#define AVB_PORT_0       (0x00000000)
#define AVB_PORT_1       (0x00000040)
#define BRIDGE_PORT_MASK (0x00000040)

/* Macro for addressing a global bridge register */
#define BRIDGE_REG_ADDRESS(device, offset)         \
  ((uintptr_t) device->virtualAddress | (offset << 2))

/* Global bridge registers */
#define BRIDGE_CTRL_REG  (0x00000000)
#  define BRIDGE_RX_PORT_0     (0x00000000)
#  define BRIDGE_RX_PORT_1     (0x00000001)
#  define BRIDGE_TX_EN_NONE    (0x00000000)
#  define BRIDGE_TX_EN_PORT_0  (0x00000002)
#  define BRIDGE_TX_EN_PORT_1  (0x00000004)

/* Macro for addressing a per-port bridge register */
#define BRIDGE_PORT_REG_ADDRESS(device, port, offset) \
  ((uintptr_t) device->virtualAddress |          \
   AVB_PORTS_BASE                     |          \
   (port & BRIDGE_PORT_MASK)          |          \
   (offset << 2))

/* Per-port packet filter registers */
#define VLAN_MASK_REG        (0x00000000)
#  define VLAN_PRIORITY_ENABLE(priorityLevel) (0x01 << priorityLevel)

#define FILTER_SELECT_REG    (0x00000001)
#  define FILTER_SELECT_NONE (0x00000000)
#  define FILTER_SELECT_ALL  (0xFFFFFFFF)

#define FILTER_CTRL_STAT_REG (0x00000002)
#  define FILTER_LOAD_ACTIVE (0x00000100)
#  define FILTER_LOAD_LAST   (0x00000200)

#define FILTER_LOAD_REG      (0x00000003)
#  define FILTER_LOAD_CLEAR (0x00000000)

/* Address definitions for the Lab X MAC used for the backplane PHY */

#define BP_MAC_REG_ADDRESS(device, offset) \
  ((uintptr_t) device->virtualAddress | LABX_MAC_REGS_BASE | (offset << 2))

#define MAC_RX_CONFIG_REG  (0x00000001)
#  define MAC_RX_RESET (0x80000000)

#define MAC_TX_CONFIG_REG  (0x00000002)
#  define MAC_TX_RESET (0x80000000)

#define MAC_SPEED_CFG_REG  (0x00000004)
#  define MAC_SPEED_10_MBPS  (0x00000000)
#  define MAC_SPEED_100_MBPS (0x00000001)
#  define MAC_SPEED_1_GBPS   (0x00000002)

/* I/O control commands and structures specific to the redundancy switch
 * hardware which may be used with two depacketizers
 */
#define LEGACY_BRIDGE_IOC_CHAR          ('b')

/* Command to configure an address filter on one of the AVB ports */
#define DISABLE_MAC_FILTER (0)
#define ENABLE_MAC_FILTER  (1)
#define MAC_ADDRESS_BYTES  (6)

typedef struct {
  uint32_t whichAvbPort;
  uint32_t whichFilter;
  uint32_t enabled;
  uint8_t  macAddress[MAC_ADDRESS_BYTES];
} MacFilterConfig;
  
#define IOC_CONFIG_MAC_FILTER _IOW(LEGACY_BRIDGE_IOC_CHAR, 0x01, MacFilterConfig)

/* Command to place the bridge's PHY into one of the test modes defined
 * in phy.h
 */

#define PHY_NORMAL_MODE   0x00
#define PHY_TEST_LOOPBACK 0x01

#define IOC_CONFIG_PHY_TEST_MODE _IOW(LEGACY_BRIDGE_IOC_CHAR, 0x02, uint32_t)

/* Command to configure the transmit and receive port selections */
#define RX_PORT_0_SELECT  (0)
#define RX_PORT_1_SELECT  (1)
#define TX_PORT_DISABLED  (0)
#define TX_PORT_ENABLED   (1)

typedef struct {
  uint32_t rxPortSelection;
  uint32_t txPortsEnable[NUM_AVB_BRIDGE_PORTS];
} BridgePortsConfig;

#define IOC_CONFIG_BRIDGE_PORTS _IOW(LEGACY_BRIDGE_IOC_CHAR, 0x03, BridgePortsConfig)

#endif
