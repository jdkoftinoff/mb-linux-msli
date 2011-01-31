/* $Id: */
/******************************************************************************
*
*       XILINX IS PROVIDING THIS DESIGN, CODE, OR INFORMATION "AS IS"
*       AS A COURTESY TO YOU, SOLELY FOR USE IN DEVELOPING PROGRAMS AND
*       SOLUTIONS FOR XILINX DEVICES.  BY PROVIDING THIS DESIGN, CODE,
*       OR INFORMATION AS ONE POSSIBLE IMPLEMENTATION OF THIS FEATURE,
*       APPLICATION OR STANDARD, XILINX IS MAKING NO REPRESENTATION
*       THAT THIS IMPLEMENTATION IS FREE FROM ANY CLAIMS OF INFRINGEMENT,
*       AND YOU ARE RESPONSIBLE FOR OBTAINING ANY RIGHTS YOU MAY REQUIRE
*       FOR YOUR IMPLEMENTATION.  XILINX EXPRESSLY DISCLAIMS ANY
*       WARRANTY WHATSOEVER WITH RESPECT TO THE ADEQUACY OF THE
*       IMPLEMENTATION, INCLUDING BUT NOT LIMITED TO ANY WARRANTIES OR
*       REPRESENTATIONS THAT THIS IMPLEMENTATION IS FREE FROM CLAIMS OF
*       INFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*       FOR A PARTICULAR PURPOSE.
*
*       (c) Copyright 2005-2006 Xilinx Inc.
*       All rights reserved.
*
******************************************************************************/
/*****************************************************************************/
/**
 *
 * @file xlltemac_control.c
 *
 * Functions in this file implement general purpose command and control related
 * functionality. See xlltemac.h for a detailed description of the driver.
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who  Date     Changes
 * ---------------------------------------------------------------------------
 * 1.00a jvb  11/10/06 First release
 * </pre>
 *****************************************************************************/

/***************************** Include Files *********************************/

#include "labx_eth_locallink.h"

/************************** Constant Definitions *****************************/


/**************************** Type Definitions *******************************/


/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/


/************************** Variable Definitions *****************************/


/*****************************************************************************/
/**
 * in the TEMAC channel's multicast filter list.
 *
 * labx_XLlTemac_MulticastAdd adds the Ethernet address, <i>AddressPtr</i> to the
 * TEMAC channel's multicast filter list, at list index <i>Entry</i>. The
 * address referenced by <i>AddressPtr</i> may be of any unicast, multicast, or
 * broadcast address form. The harware for the TEMAC channel can hold up to
 * XTE_MULTI_MAT_ENTRIES addresses in this filter list.<br><br>
 *
 * The device must be stopped to use this function.<br><br>
 *
 * Once an Ethernet address is programmed, the TEMAC channel will begin
 * receiving data sent from that address. The TEMAC hardware does not have a
 * control bit to disable multicast filtering. The only way to prevent the
 * TEMAC channel from receiving messages from an Ethernet address in the
 * Multicast Address Table (MAT) is to clear it with labx_XLlTemac_MulticastClear().
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param AddressPtr is a pointer to the 6-byte Ethernet address to set. The
 *        previous address at the location <i>Entry</i> (if any) is overwritten
 *        with the value at <i>AddressPtr</i>.
 * @param Entry is the hardware storage location to program this address and
 *        must be between 0..XTE_MULTI_MAT_ENTRIES-1.
 *
 * @return On successful completion, labx_XLlTemac_MulticastAdd returns XST_SUCCESS.
 *         Otherwise, if the TEMAC channel is not stopped, labx_XLlTemac_MulticastAdd
 *         returns XST_DEVICE_IS_STARTED.
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
int labx_XLlTemac_MulticastAdd(XLlTemac *InstancePtr, void *AddressPtr, int Entry)
{
	printk("labx_eth_llink: Multicast Add not yet implemented\n");
	return (XST_NO_FEATURE);
}


/*****************************************************************************/
/**
 * labx_XLlTemac_MulticastGet gets the Ethernet address stored at index <i>Entry</i>
 * in the TEMAC channel's multicast filter list.<br><br>
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param AddressPtr references the memory buffer to store the retrieved
 *        Ethernet address. This memory buffer must be at least 6 bytes in
 *        length.
 * @param Entry is the hardware storage location from which to retrieve the
 *        address and must be between 0..XTE_MULTI_MAT_ENTRIES-1.
 *
 * @return N/A
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
void labx_XLlTemac_MulticastGet(XLlTemac *InstancePtr, void *AddressPtr, int Entry)
{
  printk("labx_eth_llink: Multicast Get not yet implemented\n");
}

/*****************************************************************************/
/**
 * labx_XLlTemac_MulticastClear clears the Ethernet address stored at index <i>Entry</i>
 * in the TEMAC channel's multicast filter list.<br><br>
 *
 * The device must be stopped to use this function.<br><br>
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param Entry is the HW storage location used when this address was added.
 *        It must be between 0..XTE_MULTI_MAT_ENTRIES-1.
 * @param Entry is the hardware storage location to clear and must be between
 *        0..XTE_MULTI_MAT_ENTRIES-1.
 *
 * @return On successful completion, labx_XLlTemac_MulticastClear returns XST_SUCCESS.
 *         Otherwise, if the TEMAC channel is not stopped, labx_XLlTemac_MulticastClear
 *         returns XST_DEVICE_IS_STARTED.
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
int labx_XLlTemac_MulticastClear(XLlTemac *InstancePtr, int Entry)
{
	printk("labx_eth_llink: Multicast Clear not yet implemented\n");
	return (XST_NO_FEATURE);
}


/*****************************************************************************/
/**
 * labx_XLlTemac_SetMacPauseAddress sets the MAC address used for pause frames to
 * <i>AddressPtr</i>. <i>AddressPtr</i> will be the address the TEMAC channel
 * will recognize as being for pause frames. Pause frames transmitted with
 * labx_XLlTemac_SendPausePacket() will also use this address.
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param AddressPtr is a pointer to the 6-byte Ethernet address to set.
 *
 * @return On successful completion, labx_XLlTemac_SetMacPauseAddress returns
 *         XST_SUCCESS. Otherwise, if the TEMAC channel is not stopped,
 *         labx_XLlTemac_SetMacPauseAddress returns XST_DEVICE_IS_STARTED.
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
int labx_XLlTemac_SetMacPauseAddress(XLlTemac *InstancePtr, void *AddressPtr)
{
	printk("labx_eth_llink: Set Mac Pause Address not yet implemented\n");
	return (XST_NO_FEATURE);
}


/*****************************************************************************/
/**
 * labx_XLlTemac_GetMacPauseAddress gets the MAC address used for pause frames for the
 * TEMAC channel specified by <i>InstancePtr</i>.
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param AddressPtr references the memory buffer to store the retrieved MAC
 *        address. This memory buffer must be at least 6 bytes in length.
 *
 * @return N/A
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
void labx_XLlTemac_GetMacPauseAddress(XLlTemac *InstancePtr, void *AddressPtr)
{ 
	printk("labx_eth_llink: Get Mac Pause Address not yet implemented\n");
}

/*****************************************************************************/
/**
 * labx_XLlTemac_SendPausePacket sends a pause packet with the value of
 * <i>PauseValue</i>.
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param PauseValue is the pause value in units of 512 bit times.
 *
 * @return On successful completion, labx_XLlTemac_SendPausePacket returns
 *         XST_SUCCESS. Otherwise, if the TEMAC channel is not started,
 *         labx_XLlTemac_SendPausePacket returns XST_DEVICE_IS_STOPPED.
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
int labx_XLlTemac_SendPausePacket(XLlTemac *InstancePtr, u16 PauseValue)
{
	printk("labx_eth_llink: Send Pause Packet not yet implemented\n");
	return (XST_NO_FEATURE);
}

/*****************************************************************************/
/**
 * labx_XLlTemac_GetSgmiiStatus get the state of the link when using the SGMII media
 * interface.
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param SpeedPtr references the location to store the result, which is the
 *        autonegotiated link speed in units of Mbits/sec, either 0, 10, 100,
 *        or 1000.
 *
 * @return On successful completion, labx_XLlTemac_GetSgmiiStatus returns XST_SUCCESS.
 *         Otherwise, if TEMAC channel is not using an SGMII interface,
 *         labx_XLlTemac_GetSgmiiStatus returns XST_NO_FEATURE.
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
int labx_XLlTemac_GetSgmiiStatus(XLlTemac *InstancePtr, u16 *SpeedPtr)
{
	printk("labx_eth_llink: Get SGMII Status not yet implemented\n");
	return (XST_NO_FEATURE);
}


/*****************************************************************************/
/**
 * labx_XLlTemac_GetRgmiiStatus get the state of the link when using the RGMII media
 * interface.
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param SpeedPtr references the location to store the result, which is the
 *        autonegotiaged link speed in units of Mbits/sec, either 0, 10, 100,
 *        or 1000.
 * @param IsFullDuplexPtr references the value to set to indicate full duplex
 *        operation. labx_XLlTemac_GetRgmiiStatus sets <i>IsFullDuplexPtr</i> to TRUE
 *        when the RGMII link is operating in full duplex mode. Otherwise,
 *        labx_XLlTemac_GetRgmiiStatus sets <i>IsFullDuplexPtr</i> to FALSE.
 * @param IsLinkUpPtr references the value to set to indicate the link status.
 *        labx_XLlTemac_GetRgmiiStatus sets <i>IsLinkUpPtr</i> to TRUE when the RGMII
 *        link up. Otherwise, labx_XLlTemac_GetRgmiiStatus sets <i>IsLinkUpPtr</i> to
 *        FALSE.
 *
 * @return On successful completion, labx_XLlTemac_GetRgmiiStatus returns XST_SUCCESS.
 *         Otherwise, if TEMAC channel is not using an RGMII interface,
 *         labx_XLlTemac_GetRgmiiStatus returns XST_NO_FEATURE.
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
int labx_XLlTemac_GetRgmiiStatus(XLlTemac *InstancePtr, u16 *SpeedPtr,
			      int *IsFullDuplexPtr, int *IsLinkUpPtr)
{
	printk("labx_eth_llink: Get RGMII Status not yet implemented\n");
	return (XST_NO_FEATURE);
}
