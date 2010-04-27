/*
 *  linux/include/net/labx_ethernet/labx_ethernet_defs.h
 *
 *  Lab X Technologies Ethernet MAC driver definitions
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
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

#ifndef _LABX_ETHERNET_DEFS_H_
#define _LABX_ETHERNET_DEFS_H_

#include <linux/types.h>

/* PHY types supported by XEMAC; Note that these values match XPS design 
 * parameters from the PLB_TEMAC specification.
 */
#define XTE_PHY_TYPE_MII         0
#define XTE_PHY_TYPE_GMII        1
#define XTE_PHY_TYPE_RGMII_1_3   2
#define XTE_PHY_TYPE_RGMII_2_0   3
#define XTE_PHY_TYPE_SGMII       4
#define XTE_PHY_TYPE_1000BASE_X  5

/* Size of register file for each SDMA PIM */
#define SDMA_PIM_REGS_SIZE  (0x080)

/* Possible DMA modes supported by the MAC */
#define MAC_DMA_NONE	1
#define MAC_DMA_SIMPLE	2	/* simple 2 channel DMA */
#define MAC_DMA_SGDMA	3	/* scatter gather DMA */

/**
 * Prototype for a function used to initialize vendor-specific aspects
 * of an Ethernet port's PHY
 */
typedef void (*PhyInitMethod)(const char* portName, void *portHandle);

/**
 * Prototype for a function used to respond to interrupts from an
 * Ethernet port's PHY.  Called from within a kernel thread.
 */
typedef uint32_t (*PhyIsrMethod)(const char* portName, void *portHandle);

/* Return codes for the PHY ISR hook */
#define LINK_DOWN         (0)
#define LINK_UP_10_MBIT   (1)
#define LINK_UP_100_MBIT  (2)
#define LINK_UP_1_GBIT    (3)

/* Platform device data structure */
struct labx_eth_platform_data {
  uint8_t tx_csum;
  uint8_t rx_csum;
  uint8_t dcr_host;

  /* LocalLink datapath definitions */
  uint8_t ll_dev_type;
  uint32_t ll_dev_baseaddress;
  uint32_t ll_dev_dma_rx_irq;
  uint32_t ll_dev_dma_tx_irq;
  uint32_t ll_dev_fifo_irq;
  
  /* Default MAC address for the port */
  uint8_t mac_addr[6];

  /* PHY type, address, and hook function definitions.  Hook functions
   * may be set to NULL for no implementation.
   */
  uint8_t phy_type;
  uint8_t phy_addr;
  PhyInitMethod phy_init;
  PhyIsrMethod phy_isr;
};

/* LocalLink TYPE Enumerations */
#define XPAR_LL_FIFO    1
#define XPAR_LL_DMA     2

/**
 * Writes a value to a register within the specified port's Ethernet PHY
 * @param portHandle - Pointer to an opaque data structure for the port
 * @param regAddress - MDIO register address
 * @param writeData  - Data to be written to the PHY register
 */
void write_phy_register(void *portHandle, uint32_t regAddress, uint16_t writeData);

/**
 * Reads a valus from a register within the specified port's Ethernet PHY
 * @param portHandle - Pointer to an opaque data structure for the port
 * @param regAddress - MDIO register address
 * @return - The register value
 */
uint16_t read_phy_register(void *portHandle, uint32_t regAddress);

#endif
