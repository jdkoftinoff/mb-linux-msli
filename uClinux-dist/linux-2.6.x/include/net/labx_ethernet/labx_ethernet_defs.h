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
#include <linux/phy.h>

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

/* Platform device data structure for LocalLink Ethernet */
struct labx_ll_eth_platform_data {
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

  /* PHY type, address, and name. The PHY name is of the format PHY_ID_FMT.
   * These values are for the PHY connected to this instance.
   */
  uint8_t phy_type;
  uint8_t phy_addr;
  char phy_name[BUS_ID_SIZE];

  /* MDIO bus parameters.
   * phy_mask is a bitmask of MDIO addresses to probe (1's get probed)
   * mdio_bus_name is the name of the MDIO bus to register
   * mdio_phy_irqs are the interrupts from PHYs on this MDIO bus
   */
  uint32_t phy_mask;
  char mdio_bus_name[MII_BUS_ID_SIZE];
  int mdio_phy_irqs[PHY_MAX_ADDR];

  void *rx_ring_ptr;		/* Pointer to RxRing buffer */
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
