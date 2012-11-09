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
 * @file xlltemac.c
 *
 * The XLlTemac driver. Functions in this file are the minimum required functions
 * for this driver. See xlltemac.h for a detailed description of the driver.
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who  Date     Changes
 * ------------------------------------------------------------------------
 * 1.00a jvb  11/10/06 First release
 * </pre>
 ******************************************************************************/

/***************************** Include Files *********************************/

#include <linux/string.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/netdevice.h>
#include "labx_ethernet.h"

/************************** Constant Definitions *****************************/


/**************************** Type Definitions *******************************/


/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/

static void InitHw(XLlTemac *InstancePtr);	/* HW reset */

/************************** Variable Definitions *****************************/

xdbg_stmnt(int indent_on = 0;
           
           )
xdbg_stmnt(u32 _xlltemac_rir_value;
           
           )

/*****************************************************************************/
/**
 *
 * labx_eth_CfgInitialize initializes a TEMAC channel along with the
 * <i>InstancePtr</i> that references it. Each TEMAC channel is treated as a
 * separate device from the point of view of this driver.
 *
 * The PHY is setup independently from the TEMAC. Use the MII or whatever other
 * interface may be present for setup.
 *
 * @param  InstancePtr references the memory instance to be associated with
 *         the TEMAC channel upon initialization.
 * @param  CfgPtr references the structure holding the hardware configuration
 *         for the TEMAC channel to initialize.
 * @param  EffectiveAddress is the processor address used to access the
 *         base address of the TEMAC channel. In systems with an MMU and virtual
 *         memory, <i>EffectiveAddress</i> is the virtual address mapped to the
 *         physical in <code>ConfigPtr->Config.BaseAddress</code>. In systems
 *         without an active MMU, <i>EffectiveAddress</i> should be set to the
 *         same value as <code>ConfigPtr->Config.BaseAddress</code>.
 *
 * @return labx_eth_CfgInitialize returns XST_SUCCESS.
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 *
 ******************************************************************************/
int labx_eth_CfgInitialize(XLlTemac *InstancePtr,
                           labx_eth_Config *CfgPtr, u32 EffectiveAddress)
{
	/* Verify arguments */
	XASSERT_NONVOID(InstancePtr != NULL);

	/* Clear instance memory and make copy of configuration */
	memset(InstancePtr, 0, sizeof(XLlTemac));
	memcpy(&InstancePtr->Config, CfgPtr, sizeof(labx_eth_Config));

	xdbg_printf(XDBG_DEBUG_GENERAL, "labx_eth_CfgInitialize\n");
	/* Set device base address */
	InstancePtr->Config.BaseAddress = EffectiveAddress;

	/* Reset the hardware and set default options */
	InstancePtr->IsReady = XCOMPONENT_IS_READY;

	labx_eth_Reset(InstancePtr, XTE_NORESET_HARD);

	xdbg_printf(XDBG_DEBUG_GENERAL,
		    "Temac_CfgInitialize: returning SUCCESS\n");
	return XST_SUCCESS;
}


/*****************************************************************************/
/**
 * labx_eth_Start starts the TEMAC channel as follows:
 *   - Enable transmitter if XTE_TRANSMIT_ENABLE_OPTION is set
 *   - Enable receiver if XTE_RECEIVER_ENABLE_OPTION is set
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
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
void labx_eth_Start(XLlTemac *InstancePtr)
{
	u32 Reg;

	/* Assert bad arguments and conditions */
	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
#if 0
	/*
	 * If the mutual exclusion is enforced properly in the calling code, we
	 * should never get into the following case.
	 */
	XASSERT_VOID(labx_eth_ReadReg(InstancePtr->Config.BaseAddress,
				      XTE_RDY_OFFSET) &
		     XTE_RDY_HARD_ACS_RDY_MASK);
#endif
	/* If already started, then there is nothing to do */
	if (InstancePtr->IsStarted == XCOMPONENT_IS_STARTED) {
		return;
	}

	xdbg_printf(XDBG_DEBUG_GENERAL, "labx_eth_Start\n");
	/* Enable transmitter if not already enabled */
	if (InstancePtr->Options & XTE_TRANSMITTER_ENABLE_OPTION) {
		xdbg_printf(XDBG_DEBUG_GENERAL, "enabling transmitter\n");
		Reg = labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
					       XTE_TC_OFFSET);
		if (!(Reg & XTE_TC_TX_MASK)) {
			xdbg_printf(XDBG_DEBUG_GENERAL,
				    "transmitter not enabled, enabling now\n");
			labx_eth_WriteIndirectReg(InstancePtr->Config.
						  BaseAddress, XTE_TC_OFFSET,
						  Reg | XTE_TC_TX_MASK);
		}
		xdbg_printf(XDBG_DEBUG_GENERAL, "transmitter enabled\n");
	}

	/* Enable receiver */
	if (InstancePtr->Options & XTE_RECEIVER_ENABLE_OPTION) {
		xdbg_printf(XDBG_DEBUG_GENERAL, "enabling receiver\n");
		Reg = labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
					       XTE_RCW1_OFFSET);
		if (!(Reg & XTE_RCW1_RX_MASK)) {
			xdbg_printf(XDBG_DEBUG_GENERAL,
				    "receiver not enabled, enabling now\n");

			labx_eth_WriteIndirectReg(InstancePtr->Config.
						  BaseAddress, XTE_RCW1_OFFSET,
						  Reg | XTE_RCW1_RX_MASK);
		}
		xdbg_printf(XDBG_DEBUG_GENERAL, "receiver enabled\n");
	}

	/* Mark as started */
	InstancePtr->IsStarted = XCOMPONENT_IS_STARTED;
	xdbg_printf(XDBG_DEBUG_GENERAL, "labx_eth_Start: done\n");
}

/*****************************************************************************/
/**
 * labx_eth_Stop gracefully stops the TEMAC channel as follows:
 *   - Disable all interrupts from this device
 *   - Disable the receiver
 *
 * labx_eth_Stop does not modify any of the current device options.
 *
 * Since the transmitter is not disabled, frames currently in internal buffers
 * or in process by a DMA engine are allowed to be transmitted.
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
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
void labx_eth_Stop(XLlTemac *InstancePtr)
{
	u32 Reg;

	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
#if 0
	/*
	 * If the mutual exclusion is enforced properly in the calling code, we
	 * should never get into the following case.
	 */
	XASSERT_VOID(labx_eth_ReadReg(InstancePtr->Config.BaseAddress,
				      XTE_RDY_OFFSET) &
		     XTE_RDY_HARD_ACS_RDY_MASK);
#endif
	/* If already stopped, then there is nothing to do */
	if (InstancePtr->IsStarted == 0) {
		return;
	}

	xdbg_printf(XDBG_DEBUG_GENERAL, "labx_eth_Stop\n");
	xdbg_printf(XDBG_DEBUG_GENERAL,
		    "labx_eth_Stop: disabling interrupts\n");
	/* Disable interrupts */
	//labx_eth_WriteReg(InstancePtr->Config.BaseAddress, XTE_IE_OFFSET, 0);

	xdbg_printf(XDBG_DEBUG_GENERAL, "labx_eth_Stop: disabling receiver\n");
	/* Disable the receiver */
	Reg = labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
				       XTE_RCW1_OFFSET);
	Reg &= ~XTE_RCW1_RX_MASK;
	labx_eth_WriteIndirectReg(InstancePtr->Config.BaseAddress,
				  XTE_RCW1_OFFSET, Reg);

#if 0
	/* Stopping the receiver in mid-packet causes a dropped packet indication
	 * from HW. Clear it.
	 */
	/* get the interrupt pending register */
	Reg = labx_eth_ReadReg(InstancePtr->Config.BaseAddress, XTE_IP_OFFSET);
	if (Reg & XTE_INT_RXRJECT_MASK) {
		/* set the interrupt status register to clear the interrupt */
		labx_eth_WriteReg(InstancePtr->Config.BaseAddress,
				  XTE_IS_OFFSET, XTE_INT_RXRJECT_MASK);
	}
#endif

	/* Mark as stopped */
	InstancePtr->IsStarted = 0;
	xdbg_printf(XDBG_DEBUG_GENERAL, "labx_eth_Stop: done\n");
}


/*****************************************************************************/
/**
 * labx_eth_Reset performs a reset of the TEMAC channel, specified by
 * <i>InstancePtr</i>, or both channels if <i>HardCoreAction</i> is set to
 * XTE_RESET_HARD.
 *
 * labx_eth_Reset also resets the TEMAC channel's options to their default values.
 *
 * The calling software is responsible for re-configuring the TEMAC channel
 * (if necessary) and restarting the MAC after the reset.
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param HardCoreAction describes how labx_eth_Reset should treat the hard core
 *        block of the TEMAC.<br><br>
 *
 *        If XTE_RESET_HARD is set to XTE_RESET_HARD, then labx_eth_Reset asserts
 *        the reset signal to the hard core block which will reset both channels
 *        of the TEMAC. This, of course, will bork any activity that may be
 *        occuring on the other channel. So, be careful here.<br><br>
 *
 *        Otherwise, labx_eth_Reset resets just the transmitter and receiver of
 *        this TEMAC channel.
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
void labx_eth_Reset(XLlTemac *InstancePtr, int HardCoreAction)
{
	u32 Reg;

        printk("Resetting\n");
	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
#if 0
	/*
	 * If the mutual exclusion is enforced properly in the calling code, we
	 * should never get into the following case.
	 */
	XASSERT_VOID(labx_eth_ReadReg(InstancePtr->Config.BaseAddress,
				      XTE_RDY_OFFSET) &
		     XTE_RDY_HARD_ACS_RDY_MASK);
#endif
	xdbg_printf(XDBG_DEBUG_GENERAL, "labx_eth_Reset\n");
	/* Stop the device and reset HW */
	labx_eth_Stop(InstancePtr);
	InstancePtr->Options = XTE_DEFAULT_OPTIONS;

	/* Reset the receiver */
	xdbg_printf(XDBG_DEBUG_GENERAL, "resetting the receiver\n");
	Reg = labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
				       XTE_RCW1_OFFSET);
	Reg |= XTE_RCW1_RST_MASK;
	labx_eth_WriteIndirectReg(InstancePtr->Config.BaseAddress,
				  XTE_RCW1_OFFSET, Reg);

	/* Reset the transmitter */
	xdbg_printf(XDBG_DEBUG_GENERAL, "resetting the transmitter\n");
	Reg = labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
				       XTE_TC_OFFSET);
	Reg |= XTE_TC_RST_MASK;
	labx_eth_WriteIndirectReg(InstancePtr->Config.BaseAddress,
				  XTE_TC_OFFSET, Reg);

	xdbg_printf(XDBG_DEBUG_GENERAL, "waiting until reset is done\n");
	/* Poll until the reset is done */
	while (Reg & (XTE_RCW1_RST_MASK | XTE_TC_RST_MASK)) {
		Reg = labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
					       XTE_RCW1_OFFSET);
		Reg |= labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
						XTE_TC_OFFSET);
	}

#if 0 // CRW - reset not available in AVB endpoint registers
	/* Reset hard core if required */
	/* Resetting hard core will cause both channels to reset :-( */
	if (HardCoreAction == XTE_RESET_HARD) {
		xdbg_printf(XDBG_DEBUG_GENERAL, "hard reset\n");
		Reg = labx_eth_ReadReg(InstancePtr->Config.BaseAddress,
				       XTE_RAF_OFFSET);
		labx_eth_WriteReg(InstancePtr->Config.BaseAddress,
				  XTE_RAF_OFFSET, Reg | XTE_RAF_HTRST_MASK);
		while (TimeoutCount &&
		       (!(labx_eth_ReadReg
			  (InstancePtr->Config.BaseAddress,
			   XTE_RDY_OFFSET) & XTE_RDY_HARD_ACS_RDY_MASK))) {
			udelay(XTE_RESET_HARD_DELAY_US);
			TimeoutCount--;
		}
	}
#endif

	/* Setup HW */
	InitHw(InstancePtr);
}


/******************************************************************************
 * InitHw (internal use only) performs a one-time setup of a TEMAC channel. The
 * setup performed here only need to occur once after any reset.
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
static void InitHw(XLlTemac *InstancePtr)
{
	u32 Reg;
#if 0
	/*
	 * If the mutual exclusion is enforced properly in the calling code, we
	 * should never get into the following case.
	 */
	XASSERT_VOID(labx_eth_ReadReg(InstancePtr->Config.BaseAddress,
				      XTE_RDY_OFFSET) &
		     XTE_RDY_HARD_ACS_RDY_MASK);
#endif
	xdbg_printf(XDBG_DEBUG_GENERAL, "XLlTemac InitHw\n");
	/* Disable the receiver */
	xdbg_printf(XDBG_DEBUG_GENERAL, "XLlTemac InitHw\n");
	xdbg_printf(XDBG_DEBUG_GENERAL,
		    "XLlTemac InitHw: disabling receiver\n");
	Reg = labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
				       XTE_RCW1_OFFSET);
	Reg &= ~XTE_RCW1_RX_MASK;
	labx_eth_WriteIndirectReg(InstancePtr->Config.BaseAddress,
				  XTE_RCW1_OFFSET, Reg);
#if 0
	/*
	 * Stopping the receiver in mid-packet causes a dropped packet
	 * indication from HW. Clear it.
	 */
	/* get the interrupt pending register */
	Reg = labx_eth_ReadReg(InstancePtr->Config.BaseAddress, XTE_IP_OFFSET);
	if (Reg & XTE_INT_RXRJECT_MASK) {
		/*
		 * set the interrupt status register to clear the pending
		 * interrupt
		 */
		labx_eth_WriteReg(InstancePtr->Config.BaseAddress,
				  XTE_IS_OFFSET, XTE_INT_RXRJECT_MASK);
	}
#endif
	/* Sync default options with HW but leave receiver and transmitter
	 * disabled. They get enabled with labx_eth_Start() if
	 * XTE_TRANSMITTER_ENABLE_OPTION and XTE_RECEIVER_ENABLE_OPTION are set
	 */
	labx_eth_SetOptions(InstancePtr, InstancePtr->Options &
			    ~(XTE_TRANSMITTER_ENABLE_OPTION |
			      XTE_RECEIVER_ENABLE_OPTION));

	labx_eth_ClearOptions(InstancePtr, ~InstancePtr->Options);

	/* Set default MDIO divisor */
	labx_eth_PhySetMdioDivisor(InstancePtr, XTE_MDIO_DIV_DFT);
	xdbg_printf(XDBG_DEBUG_GENERAL, "XLlTemac InitHw: done\n");
}

#define MAC_MATCH_NONE 0
#define MAC_MATCH_ALL 1

static const u8 MAC_BROADCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static const u8 MAC_ZERO[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#define NUM_SRL16E_CONFIG_WORDS 8
#define NUM_SRL16E_INSTANCES    12

/* Busy loops until the match unit configuration logic is idle.  The hardware goes 
 * idle very quickly and deterministically after a configuration word is written, 
 * so this should not consume very much time at all.
 */
static void wait_match_config(XLlTemac *InstancePtr) {
  uint32_t statusWord;
  uint32_t timeout = 10000;
  do {
    statusWord = labx_eth_ReadReg(InstancePtr->Config.BaseAddress, MAC_CONTROL_REG);
    if (timeout-- == 0)
    {
      printk("depacketizer: wait_match_config timeout!\n");
      break;
    }
  } while(statusWord & MAC_ADDRESS_LOAD_ACTIVE);
}

/* Selects a set of match units for subsequent configuration loads */
typedef enum { SELECT_NONE, SELECT_SINGLE, SELECT_ALL } SelectionMode;
static void select_matchers(XLlTemac *InstancePtr,
                            SelectionMode selectionMode,
                            uint32_t matchUnit) {

	switch(selectionMode) {
		case SELECT_NONE:
			/* De-select all the match units */
			//printk("MAC SELECT %08X\n", 0);
			labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MAC_SELECT_REG, 0);
			break;

		case SELECT_SINGLE:
			/* Select a single unit */
			//printk("MAC SELECT %08X\n", 1 << matchUnit);
			labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MAC_SELECT_REG, 1 << matchUnit);
			break;

		default:
			/* Select all match units at once */
			//printk("MAC SELECT %08X\n", 0xFFFFFFFF);
			labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MAC_SELECT_REG, 0xFFFFFFFF);
			break;
	}
}

/* Sets the loading mode for any selected match units.  This revolves around
 * automatically disabling the match units' outputs while they're being
 * configured so they don't fire false matches, and re-enabling them as their
 * last configuration word is loaded.
 */
typedef enum { LOADING_MORE_WORDS, LOADING_LAST_WORD } LoadingMode;
static void set_matcher_loading_mode(XLlTemac *InstancePtr,
                                     LoadingMode loadingMode) {
	uint32_t controlWord = labx_eth_ReadReg(InstancePtr->Config.BaseAddress, MAC_CONTROL_REG);

	if(loadingMode == LOADING_MORE_WORDS) {
		/* Clear the "last word" bit to suppress false matches while the units are
		 * only partially cleared out
		 */
		controlWord &= ~MAC_ADDRESS_LOAD_LAST;
	} else {
		/* Loading the final word, flag the match unit(s) to enable after the
		 * next configuration word is loaded.
		 */
		controlWord |= MAC_ADDRESS_LOAD_LAST;
	}
	//printk("CONTROL WORD %08X\n", controlWord);
	labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MAC_CONTROL_REG, controlWord);
}

/* Clears any selected match units, preventing them from matching any packets */
static void clear_selected_matchers(XLlTemac *InstancePtr) {
	uint32_t wordIndex;

	/* Ensure the unit(s) disable as the first word is load to prevent erronous
	 * matches as the units become partially-cleared
	 */
	set_matcher_loading_mode(InstancePtr, LOADING_MORE_WORDS);

	for(wordIndex = 0; wordIndex < NUM_SRL16E_CONFIG_WORDS; wordIndex++) {
		/* Assert the "last word" flag on the last word required to complete the clearing
		 * of the selected unit(s).
		 */
		if(wordIndex == (NUM_SRL16E_CONFIG_WORDS - 1)) {
			set_matcher_loading_mode(InstancePtr, LOADING_LAST_WORD);
		}
		//printk("MAC LOAD %08X\n", 0);
		labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MAC_LOAD_REG, 0);
	}
}

/* Loads truth tables into a match unit using the newest, "unified" match
 * architecture.  This is SRL16E based (not cascaded) due to the efficient
 * packing of these primitives into Xilinx LUT6-based architectures.
 */
static void load_unified_matcher(XLlTemac *InstancePtr,
                                 const uint8_t matchMac[6]) {
	int32_t wordIndex;
	int32_t lutIndex;
	uint32_t configWord = 0x00000000;
	uint32_t matchChunk;
  
	/* In this architecture, all of the SRL16Es are loaded in parallel, with each
	 * configuration word supplying two bits to each.  Only one of the two bits can
	 * ever be set, so there is just an explicit check for one.
	 */
	for(wordIndex = (NUM_SRL16E_CONFIG_WORDS - 1); wordIndex >= 0; wordIndex--) {
		for(lutIndex = (NUM_SRL16E_INSTANCES - 1); lutIndex >= 0; lutIndex--) {
			matchChunk = ((matchMac[5-(lutIndex/2)] >> ((lutIndex&1) << 2)) & 0x0F);
			configWord <<= 2;
			if(matchChunk == (wordIndex << 1)) configWord |= 0x01;
			if(matchChunk == ((wordIndex << 1) + 1)) configWord |= 0x02;
		}
		/* 12 nybbles are packed to the MSB */
		configWord <<= 8;

		/* Two bits of truth table have been determined for each SRL16E, load the
		 * word and wait for the configuration to occur.  Be sure to flag the last
		 * word to automatically re-enable the match unit(s) as the last word completes.
		 */
		if(wordIndex == 0) set_matcher_loading_mode(InstancePtr, LOADING_LAST_WORD);
		//printk("MAC LOAD %08X\n", configWord);
		labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MAC_LOAD_REG, configWord);
		wait_match_config(InstancePtr);
	}
}

static void ConfigureMacFilter(XLlTemac *InstancePtr, int unitNum, const u8 mac[6], int mode)
{
	/* Only allow programming up to the supported number of MAC match units */
	if (unitNum >= InstancePtr->MacMatchUnits) return;

	//printk("CONFIGURE MAC MATCH %d (%d), %02X:%02X:%02X:%02X:%02X:%02X\n", unitNum, mode,
	//	mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	/* Ascertain that the configuration logic is ready, then select the matcher */
	wait_match_config(InstancePtr);
	select_matchers(InstancePtr, SELECT_SINGLE, unitNum);

	if (mode == MAC_MATCH_NONE) {
		clear_selected_matchers(InstancePtr);
	} else {
		/* Set the loading mode to disable as we load the first word */
		set_matcher_loading_mode(InstancePtr, LOADING_MORE_WORDS);
      
		/* Calculate matching truth tables for the LUTs and load them */
		load_unified_matcher(InstancePtr, mac);
	}

	/* De-select the match unit */
	select_matchers(InstancePtr, SELECT_NONE, 0);
}

void labx_eth_UpdateMacFilters(XLlTemac *InstancePtr)
{
	int i;

	/* Always allow our unicast mac */
	ConfigureMacFilter(InstancePtr, 0, InstancePtr->Config.MacAddress, MAC_MATCH_ALL);

	/* Allow broadcasts if configured to do so */
	if (InstancePtr->Options & XTE_BROADCAST_OPTION) {
		ConfigureMacFilter(InstancePtr, 1, MAC_BROADCAST, MAC_MATCH_ALL);
	} else {
		ConfigureMacFilter(InstancePtr, 1, MAC_BROADCAST, MAC_MATCH_NONE);
	}

	/* Allow multicasts if configured to do so */
	if (InstancePtr->Options & XTE_MULTICAST_OPTION) {
		struct dev_mc_list *dmi = InstancePtr->dev->mc_list;
                int i;

                for (i=2; (i<(InstancePtr->dev->mc_count+2)) && (i<InstancePtr->MacMatchUnits); i++) {
			ConfigureMacFilter(InstancePtr, i, dmi->da_addr, MAC_MATCH_ALL);
                        dmi = dmi->next;
                }

	} else {
		/* Disable all multicast filters */
		for (i=2; i<InstancePtr->MacMatchUnits; i++) {
			ConfigureMacFilter(InstancePtr, i, MAC_ZERO, MAC_MATCH_NONE);
		}
	}
}

/*****************************************************************************/
/**
 * labx_eth_SetMacAddress sets the MAC address for the TEMAC channel, specified
 * by <i>InstancePtr</i> to the MAC address specified by <i>AddressPtr</i>.
 * The TEMAC channel must be stopped before calling this function.
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param AddressPtr is a reference to the 6-byte MAC address to set.
 *
 * @return On successful completion, labx_eth_SetMacAddress returns XST_SUCCESS.
 *         Otherwise, if the TEMAC channel has not stopped,
 *         labx_eth_SetMacAddress returns XST_DEVICE_IS_STARTED.
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
int labx_eth_SetMacAddress(XLlTemac *InstancePtr, void *AddressPtr)
{
	u8 *Aptr = (u8 *) AddressPtr;
	int byteIndex;

	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
	XASSERT_NONVOID(AddressPtr != NULL);
#if 0
	/*
	 * If the mutual exclusion is enforced properly in the calling code, we
	 * should never get into the following case.
	 */
	XASSERT_NONVOID(labx_eth_ReadReg(InstancePtr->Config.BaseAddress,
					 XTE_RDY_OFFSET) &
			XTE_RDY_HARD_ACS_RDY_MASK);
#endif
	/* Be sure device has been stopped */
	if (InstancePtr->IsStarted == XCOMPONENT_IS_STARTED) {
		return (XST_DEVICE_IS_STARTED);
	}

	xdbg_printf(XDBG_DEBUG_GENERAL,
		    "labx_eth_SetMacAddress: setting mac address to: 0x%08x%8x%8x%8x%8x%8x\n",
		    Aptr[0], Aptr[1], Aptr[2], Aptr[3], Aptr[4], Aptr[5]);
	/*
	 * Save our MAC in the device structre and update the MAC filters.
	 */
	for(byteIndex = 0; byteIndex < 6; byteIndex++) {
		InstancePtr->Config.MacAddress[byteIndex] = Aptr[byteIndex];
	}

	labx_eth_UpdateMacFilters(InstancePtr);

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
 * labx_eth_GetMacAddress gets the MAC address for the TEMAC channel, specified
 * by <i>InstancePtr</i> into the memory buffer specified by <i>AddressPtr</i>.
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
void labx_eth_GetMacAddress(XLlTemac *InstancePtr, void *AddressPtr)
{
	u8 *Aptr = (u8 *) AddressPtr;
	int byteIndex;

	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
#if 0
	/*
	 * If the mutual exclusion is enforced properly in the calling code, we
	 * should never get into the following case.
	 */
	XASSERT_VOID(labx_eth_ReadReg(InstancePtr->Config.BaseAddress,
				      XTE_RDY_OFFSET) &
		     XTE_RDY_HARD_ACS_RDY_MASK);
#endif

	/* Copy the saved MAC address to the caller */
	for(byteIndex = 0; byteIndex < 6; byteIndex++) {
		Aptr[byteIndex] = InstancePtr->Config.MacAddress[byteIndex];
	}
}

/*****************************************************************************/
/**
 * labx_eth_SetOptions enables the options, <i>Options</i> for the TEMAC channel,
 * specified by <i>InstancePtr</i>. The TEMAC channel should be stopped with
 * labx_eth_Stop() before changing options.
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param Options is a bitmask of OR'd XTE_*_OPTION values for options to
 *        set. Options not specified are not affected.
 *
 * @return On successful completion, labx_eth_SetOptions returns XST_SUCCESS.
 *         Otherwise, if the device has not been stopped, labx_eth_SetOptions
 *         returns XST_DEVICE_IS_STARTED.
 *
 * @note
 * See xlltemac.h for a description of the available options.
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
int labx_eth_SetOptions(XLlTemac *InstancePtr, u32 Options)
{
	u32 RegRcw1;		/* Reflects original contents of RCW1 */
	u32 RegTc;		/* Reflects original contents of TC  */
	u32 RegNewRcw1;		/* Reflects new contents of RCW1 */
	u32 RegNewTc;		/* Reflects new contents of TC  */

	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
#if 0
	/*
	 * If the mutual exclusion is enforced properly in the calling code, we
	 * should never get into the following case.
	 */
	XASSERT_NONVOID(labx_eth_ReadReg(InstancePtr->Config.BaseAddress,
					 XTE_RDY_OFFSET) &
			XTE_RDY_HARD_ACS_RDY_MASK);
#endif
	/* Be sure device has been stopped */
	if (InstancePtr->IsStarted == XCOMPONENT_IS_STARTED) {
		return (XST_DEVICE_IS_STARTED);
	}

	xdbg_printf(XDBG_DEBUG_GENERAL, "labx_eth_SetOptions\n");
	/* Many of these options will change the RCW1 or TC registers.
	 * To reduce the amount of IO to the device, group these options here
	 * and change them all at once.
	 */

	/* Grab current register contents */
	RegRcw1 = labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
					   XTE_RCW1_OFFSET);
	RegTc = labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
					 XTE_TC_OFFSET);
	RegNewRcw1 = RegRcw1;
	RegNewTc = RegTc;

	xdbg_printf(XDBG_DEBUG_GENERAL,
		    "current control regs: RCW1: 0x%0x; TC: 0x%0x\n", RegRcw1,
		    RegTc);
	xdbg_printf(XDBG_DEBUG_GENERAL,
		    "Options: 0x%0x; default options: 0x%0x\n", Options,
		    XTE_DEFAULT_OPTIONS);

	/* Turn on jumbo packet support for both Rx and Tx */
	if (Options & XTE_JUMBO_OPTION) {
		RegNewTc |= XTE_TC_JUM_MASK;
		RegNewRcw1 |= XTE_RCW1_JUM_MASK;
	}

	/* Turn on VLAN packet support for both Rx and Tx */
	if (Options & XTE_VLAN_OPTION) {
		RegNewTc |= XTE_TC_VLAN_MASK;
		RegNewRcw1 |= XTE_RCW1_VLAN_MASK;
	}

	/* Turn on FCS stripping on receive packets */
	if (Options & XTE_FCS_STRIP_OPTION) {
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "setOptions: enabling fcs stripping\n");
		RegNewRcw1 &= ~XTE_RCW1_FCS_MASK;
	}

	/* Turn on FCS insertion on transmit packets */
	if (Options & XTE_FCS_INSERT_OPTION) {
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "setOptions: enabling fcs insertion\n");
		RegNewTc &= ~XTE_TC_FCS_MASK;
	}

	/* Turn on length/type field checking on receive packets */
	if (Options & XTE_LENTYPE_ERR_OPTION) {
		RegNewRcw1 &= ~XTE_RCW1_LT_DIS_MASK;
	}

	/* Enable transmitter */
	if (Options & XTE_TRANSMITTER_ENABLE_OPTION) {
		RegNewTc |= XTE_TC_TX_MASK;
	}

	/* Enable receiver */
	if (Options & XTE_RECEIVER_ENABLE_OPTION) {
		RegNewRcw1 |= XTE_RCW1_RX_MASK;
	}

	/* Change the TC or RCW1 registers if they need to be modified */
	if (RegTc != RegNewTc) {
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "setOptions: writting tc: 0x%0x\n", RegNewTc);
		labx_eth_WriteIndirectReg(InstancePtr->Config.BaseAddress,
					  XTE_TC_OFFSET, RegNewTc);
	}

	if (RegRcw1 != RegNewRcw1) {
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "setOptions: writting rcw1: 0x%0x\n", RegNewRcw1);
		labx_eth_WriteIndirectReg(InstancePtr->Config.BaseAddress,
					  XTE_RCW1_OFFSET, RegNewRcw1);
	}

	xdbg_printf(XDBG_DEBUG_GENERAL,
		    "setOptions: rcw1 is now (raf): 0x%0x\n",
		    labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
					     XTE_RCW1_OFFSET));

	/* The remaining options not handled here are managed elsewhere in the
	 * driver. No register modifications are needed at this time. Reflecting the
	 * option in InstancePtr->Options is good enough for now.
	 */

	/* Set options word to its new value */
	InstancePtr->Options |= Options;

	/* Enable broadcast or multicast address filtering */
	if (Options & (XTE_BROADCAST_OPTION | XTE_MULTICAST_OPTION)) {
		labx_eth_UpdateMacFilters(InstancePtr);
	}

	xdbg_printf(XDBG_DEBUG_GENERAL,
		    "setOptions: rcw1 is now (end): 0x%0x\n",
		    labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
					     XTE_RCW1_OFFSET));
	xdbg_printf(XDBG_DEBUG_GENERAL, "setOptions: returning SUCCESS\n");
	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
 * labx_eth_ClearOptions clears the options, <i>Options</i> for the TEMAC channel,
 * specified by <i>InstancePtr</i>. The TEMAC channel should be stopped with
 * labx_eth_Stop() before changing options.
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param Options is a bitmask of OR'd XTE_*_OPTION values for options to
 *        clear. Options not specified are not affected.
 *
 * @return On successful completion, labx_eth_ClearOptions returns XST_SUCCESS.
 *         Otherwise, if the device has not been stopped, labx_eth_ClearOptions
 *         returns XST_DEVICE_IS_STARTED.
 *
 * @note
 * See xlltemac.h for a description of the available options.
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
int labx_eth_ClearOptions(XLlTemac *InstancePtr, u32 Options)
{
	u32 RegRcw1;		/* Reflects original contents of RCW1 */
	u32 RegTc;		/* Reflects original contents of TC  */
	u32 RegNewRcw1;		/* Reflects new contents of RCW1 */
	u32 RegNewTc;		/* Reflects new contents of TC  */

	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
#if 0
	/*
	 * If the mutual exclusion is enforced properly in the calling code, we
	 * should never get into the following case.
	 */
	XASSERT_NONVOID(labx_eth_ReadReg(InstancePtr->Config.BaseAddress,
					 XTE_RDY_OFFSET) &
			XTE_RDY_HARD_ACS_RDY_MASK);
#endif
	xdbg_printf(XDBG_DEBUG_GENERAL, "Xtemac_ClearOptions: 0x%08x\n",
		    Options);
	/* Be sure device has been stopped */
	if (InstancePtr->IsStarted == XCOMPONENT_IS_STARTED) {
		return (XST_DEVICE_IS_STARTED);
	}

	/* Many of these options will change the RCW1 or TC registers.
	 * Group these options here and change them all at once. What we are
	 * trying to accomplish is to reduce the amount of IO to the device
	 */

	/* Grab current register contents */
	RegRcw1 = labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
					   XTE_RCW1_OFFSET);
	RegTc = labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
					 XTE_TC_OFFSET);
	RegNewRcw1 = RegRcw1;
	RegNewTc = RegTc;

	/* Turn off jumbo packet support for both Rx and Tx */
	if (Options & XTE_JUMBO_OPTION) {
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "Xtemac_ClearOptions: disabling jumbo\n");
		RegNewTc &= ~XTE_TC_JUM_MASK;
		RegNewRcw1 &= ~XTE_RCW1_JUM_MASK;
	}

	/* Turn off VLAN packet support for both Rx and Tx */
	if (Options & XTE_VLAN_OPTION) {
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "Xtemac_ClearOptions: disabling vlan\n");
		RegNewTc &= ~XTE_TC_VLAN_MASK;
		RegNewRcw1 &= ~XTE_RCW1_VLAN_MASK;
	}

	/* Turn off FCS stripping on receive packets */
	if (Options & XTE_FCS_STRIP_OPTION) {
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "Xtemac_ClearOptions: disabling fcs strip\n");
		RegNewRcw1 |= XTE_RCW1_FCS_MASK;
	}

	/* Turn off FCS insertion on transmit packets */
	if (Options & XTE_FCS_INSERT_OPTION) {
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "Xtemac_ClearOptions: disabling fcs insert\n");
		RegNewTc |= XTE_TC_FCS_MASK;
	}

	/* Turn off length/type field checking on receive packets */
	if (Options & XTE_LENTYPE_ERR_OPTION) {
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "Xtemac_ClearOptions: disabling lentype err\n");
		RegNewRcw1 |= XTE_RCW1_LT_DIS_MASK;
	}

	/* Disable transmitter */
	if (Options & XTE_TRANSMITTER_ENABLE_OPTION) {
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "Xtemac_ClearOptions: disabling transmitter\n");
		RegNewTc &= ~XTE_TC_TX_MASK;
	}

	/* Disable receiver */
	if (Options & XTE_RECEIVER_ENABLE_OPTION) {
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "Xtemac_ClearOptions: disabling receiver\n");
		RegNewRcw1 &= ~XTE_RCW1_RX_MASK;
	}

	/* Change the TC and RCW1 registers if they need to be
	 * modified
	 */
	if (RegTc != RegNewTc) {
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "Xtemac_ClearOptions: setting TC: 0x%0x\n",
			    RegNewTc);
		labx_eth_WriteIndirectReg(InstancePtr->Config.BaseAddress,
					  XTE_TC_OFFSET, RegNewTc);
	}

	if (RegRcw1 != RegNewRcw1) {
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "Xtemac_ClearOptions: setting RCW1: 0x%0x\n",
			    RegNewRcw1);
		labx_eth_WriteIndirectReg(InstancePtr->Config.BaseAddress,
					  XTE_RCW1_OFFSET, RegNewRcw1);
	}

	/* The remaining options not handled here are managed elsewhere in the
	 * driver. No register modifications are needed at this time. Reflecting the
	 * option in InstancePtr->Options is good enough for now.
	 */

	/* Set options word to its new value */
	InstancePtr->Options &= ~Options;

	/* Disable broadcast or multicast address filtering */
	if (Options & (XTE_BROADCAST_OPTION | XTE_MULTICAST_OPTION)) {
		labx_eth_UpdateMacFilters(InstancePtr);
	}

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
 * labx_eth_GetOptions returns the current option settings.
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 *
 * @return labx_eth_GetOptions returns a bitmask of XTE_*_OPTION constants,
 *         each bit specifying an option that is currently active.
 *
 * @note
 * See xlltemac.h for a description of the available options.
 *
 ******************************************************************************/
u32 labx_eth_GetOptions(XLlTemac *InstancePtr)
{
	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);

	return (InstancePtr->Options);
}

/*****************************************************************************/
/**
 * labx_eth_GetOperatingSpeed gets the current operating link speed. This may be
 * the value set by labx_eth_SetOperatingSpeed() or a hardware default.
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 *
 * @return labx_eth_GetOperatingSpeed returns the link speed in units of megabits
 *         per second.
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
u16 labx_eth_GetOperatingSpeed(XLlTemac *InstancePtr)
{
	XASSERT_NONVOID(InstancePtr != NULL);
	XASSERT_NONVOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
#if 0
	/*
	 * If the mutual exclusion is enforced properly in the calling code, we
	 * should never get into the following case.
	 */
	XASSERT_NONVOID(labx_eth_ReadReg(InstancePtr->Config.BaseAddress,
					 XTE_RDY_OFFSET) &
			XTE_RDY_HARD_ACS_RDY_MASK);
#endif
	xdbg_printf(XDBG_DEBUG_GENERAL, "labx_eth_GetOperatingSpeed\n");
	switch (labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
					 XTE_EMMC_OFFSET) &
		XTE_EMMC_LINKSPEED_MASK) {
	case XTE_EMMC_LINKSPD_1000:
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "labx_eth_GetOperatingSpeed: returning 1000\n");
		return (1000);

	case XTE_EMMC_LINKSPD_100:
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "labx_eth_GetOperatingSpeed: returning 100\n");
		return (100);

	case XTE_EMMC_LINKSPD_10:
		xdbg_printf(XDBG_DEBUG_GENERAL,
			    "labx_eth_GetOperatingSpeed: returning 10\n");
		return (10);

	default:
		return (0);
	}
}


/*****************************************************************************/
/**
 * labx_eth_SetOperatingSpeed sets the current operating link speed. For any
 * traffic to be passed, this speed must match the current MII/GMII/SGMII/RGMII
 * link speed.
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param Speed is the speed to set in units of Mbps. Valid values are 10, 100,
 *        or 1000. labx_eth_SetOperatingSpeed ignores invalid values.
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
void labx_eth_SetOperatingSpeed(XLlTemac *InstancePtr, u16 Speed)
{
	u32 EmmcReg;
	u32 Reg;

	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY);
	XASSERT_VOID((Speed == 10) || (Speed == 100) || (Speed == 1000)|| (Speed == 10000));
#if 0
	/*
	 * If the mutual exclusion is enforced properly in the calling code, we
	 * should never get into the following case.
	 */
	XASSERT_VOID(labx_eth_ReadReg(InstancePtr->Config.BaseAddress,
				      XTE_RDY_OFFSET) &
		     XTE_RDY_HARD_ACS_RDY_MASK);
#endif
	xdbg_printf(XDBG_DEBUG_GENERAL, "labx_eth_SetOperatingSpeed\n");
	xdbg_printf(XDBG_DEBUG_GENERAL,
		    "labx_eth_SetOperatingSpeed: setting speed to: %d (0x%0x)\n",
		    Speed, Speed);
	/* Get the current contents of the EMAC config register and zero out
	 * speed bits
	 */
	EmmcReg = labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
					   XTE_EMMC_OFFSET) &
		~XTE_EMMC_LINKSPEED_MASK;

	xdbg_printf(XDBG_DEBUG_GENERAL,
		    "labx_eth_SetOperatingSpeed: current speed: 0x%0x\n",
		    EmmcReg);
	switch (Speed) {
	case 10:
		break;

	case 100:
		EmmcReg |= XTE_EMMC_LINKSPD_100;
		break;

	case 1000:
	case 10000:
		EmmcReg |= XTE_EMMC_LINKSPD_1000;
		break;

	default:
		return;
	}

	xdbg_printf(XDBG_DEBUG_GENERAL,
		    "labx_eth_SetOperatingSpeed: new speed: 0x%0x\n", EmmcReg);

	/* Set register and return */
	labx_eth_WriteIndirectReg(InstancePtr->Config.BaseAddress,
				  XTE_EMMC_OFFSET, EmmcReg);

	/* Reset Rx since the speed changed to allow the shim to recalibrate if necessary */
	Reg = labx_eth_ReadIndirectReg(InstancePtr->Config.BaseAddress,
				       XTE_RCW1_OFFSET);
	Reg |= XTE_RCW1_RST_MASK;
	labx_eth_WriteIndirectReg(InstancePtr->Config.BaseAddress,
				  XTE_RCW1_OFFSET, Reg);

	xdbg_printf(XDBG_DEBUG_GENERAL, "labx_eth_SetOperatingSpeed: done\n");
}

/*****************************************************************************/
/**
 * labx_eth_PhySetMdioDivisor sets the MDIO clock divisor in the TEMAC channel,
 * specified by <i>InstancePtr</i> to the value, <i>Divisor</i>. This function
 * must be called once after each reset prior to accessing MII PHY registers.
 *
 * From the Virtex-4 Embedded Tri-Mode Ethernet MAC User's Guide, the
 * following equation governs the MDIO clock to the PHY:
 *
 * <pre>
 *              f[HOSTCLK]
 *   f[MDC] = -----------------
 *            (1 + Divisor) * 2
 * </pre>
 *
 * where f[HOSTCLK] is the bus clock frequency in MHz, and f[MDC] is the
 * MDIO clock frequency in MHz to the PHY. Typically, f[MDC] should not
 * exceed 2.5 MHz. Some PHYs can tolerate faster speeds which means faster
 * access.
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param Divisor is the divisor value to set within the range of 0 to
 *        XTE_MC_CLK_DVD_MAX.
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.
 *
 ******************************************************************************/
void labx_eth_PhySetMdioDivisor(XLlTemac *InstancePtr, u8 Divisor)
{
	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(InstancePtr->IsReady == XCOMPONENT_IS_READY)
		XASSERT_VOID(Divisor <= XTE_MC_CLOCK_DIVIDE_MAX);
#if 0
	/*
	 * If the mutual exclusion is enforced properly in the calling code, we
	 * should never get into the following case.
	 */
	XASSERT_VOID(labx_eth_ReadReg(InstancePtr->Config.BaseAddress,
				      XTE_RDY_OFFSET) &
		     XTE_RDY_HARD_ACS_RDY_MASK);
#endif
	xdbg_printf(XDBG_DEBUG_GENERAL, "labx_eth_PhySetMdioDivisor\n");
	labx_eth_WriteIndirectReg(InstancePtr->Config.BaseAddress,
				  XTE_MC_OFFSET,
				  (u32) Divisor | XTE_MC_MDIOEN_MASK);
}

/*****************************************************************************/
/*
 * labx_eth_PhyRead reads the specified PHY register, <i>RegiseterNum</i> on the
 * PHY specified by <i>PhyAddress</i> into <i>PhyDataPtr</i>. This Ethernet
 * driver does not require the device to be stopped before reading from the PHY.
 * It is the responsibility of the calling code to stop the device if it is
 * deemed necessary.
 *
 * Note that the TEMAC hardware provides the ability to talk to a PHY that
 * adheres to the Media Independent Interface (MII) as defined in the IEEE 802.3
 * standard.
 *
 * <b>It is important that calling code set up the MDIO clock with
 * labx_eth_PhySetMdioDivisor() prior to accessing the PHY with this function.</b>
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param PhyAddress is the address of the PHY to be written (multiple
 *        PHYs supported).
 * @param RegisterNum is the register number, 0-31, of the specific PHY register
 *        to write.
 * @param PhyDataPtr is a reference to the location where the 16-bit result
 *        value is stored.
 *
 * @return N/A
 *
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.<br><br>
 *
 * There is the possibility that this function will not return if the hardware
 * is broken (i.e., it never sets the status bit indicating that the write is
 * done). If this is of concern, the calling code should provide a mechanism
 * suitable for recovery.
 *
 ******************************************************************************/
/*
void labx_eth_PhyRead(XLlTemac *InstancePtr, u32 PhyAddress,
                      u32 RegisterNum, u16 *PhyDataPtr)
{
	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(PhyDataPtr != NULL);

    if(labx_eth_ReadReg(InstancePtr->Config.BaseAddress, MDIO_CONTROL_REG) &
       PHY_MDIO_BUSY) {
      printk("Read issued while PHY busy!\n");
      return;
    }

    // Write the control register first to effect the read, wait for the MDIO
    // ransfer to complete, and then return the read value.
 
    InstancePtr->MdioState = MDIO_STATE_BUSY;
    labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MDIO_CONTROL_REG,
                      (PHY_MDIO_READ | 
                       ((PhyAddress & PHY_ADDR_MASK) << PHY_ADDR_SHIFT) |
                       (RegisterNum & PHY_REG_ADDR_MASK)));

    // The ISR will wake us up when the read completes
    wait_event_interruptible_timeout(InstancePtr->PhyWait,
                                     (InstancePtr->MdioState == MDIO_STATE_READY),
                                     MDIO_TIMEOUT_JIFFIES);

    if(InstancePtr->MdioState == MDIO_STATE_READY) {
      *PhyDataPtr = (u16) labx_eth_ReadReg(InstancePtr->Config.BaseAddress, 
                                           MDIO_DATA_REG);
    } else {
      printk("MDIO read timeout!\n");
    }
}
*/

void labx_eth_PhyRead(XLlTemac *InstancePtr, u32 PhyAddress,
                      u32 RegisterNum, u16 *PhyDataPtr)
{
	XASSERT_VOID(InstancePtr != NULL);
	XASSERT_VOID(PhyDataPtr != NULL);

    if(labx_eth_ReadReg(InstancePtr->Config.BaseAddress, MDIO_CONTROL_REG) &
       PHY_MDIO_BUSY) {
      printk("Read issued while PHY busy!\n");
      return;
    }

     /* the 802.3ae Clause45 protocol for the 10G PHY */  
    if(InstancePtr->Config.MacWidth == 64) {
      labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MDIO_DATA_REG, RegisterNum);
      InstancePtr->MdioState = MDIO_STATE_BUSY;
      //to be deleted
      
      labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MDIO_CONTROL_REG,
                      (PHY_10G_MDIO_ADDR | 
                       ((PhyAddress & PHY_ADDR_MASK) << PHY_ADDR_SHIFT) |
                       (InstancePtr->Config.PhyType & PHY_10G_DEVTYPE_MASK)));  
      
      wait_event_interruptible_timeout(InstancePtr->PhyWait,
                                     (InstancePtr->MdioState == MDIO_STATE_READY),
                                     MDIO_TIMEOUT_JIFFIES);

      if(InstancePtr->MdioState != MDIO_STATE_READY)
        printk("MDIO write timeout!\n");
        
      InstancePtr->MdioState = MDIO_STATE_BUSY;
      
      labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MDIO_CONTROL_REG,
                      (PHY_10G_MDIO_READ | 
                       ((PhyAddress & PHY_ADDR_MASK) << PHY_ADDR_SHIFT) |
                       (InstancePtr->Config.PhyType & PHY_10G_DEVTYPE_MASK)));  
      
    }
       /* the 802.3ae Clause22 protocol, the old standard way */
    else {
    /* Write the control register first to effect the read, wait for the MDIO
     * transfer to complete, and then return the read value.
     */
      InstancePtr->MdioState = MDIO_STATE_BUSY;
      labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MDIO_CONTROL_REG,
                        (PHY_MDIO_READ | 
                         ((PhyAddress & PHY_ADDR_MASK) << PHY_ADDR_SHIFT) |
                         (RegisterNum & PHY_REG_ADDR_MASK)));
    }

    // The ISR will wake us up when the read completes
    wait_event_interruptible_timeout(InstancePtr->PhyWait,
                                     (InstancePtr->MdioState == MDIO_STATE_READY),
                                     MDIO_TIMEOUT_JIFFIES);

    if(InstancePtr->MdioState == MDIO_STATE_READY) {
      *PhyDataPtr = (u16) labx_eth_ReadReg(InstancePtr->Config.BaseAddress, 
                                           MDIO_DATA_REG);
    } else {
      printk("MDIO read timeout!\n");
    }
}


/*****************************************************************************/
/*
 * labx_eth_PhyWrite writes <i>PhyData</i> to the specified PHY register,
 * <i>RegiseterNum</i> on the PHY specified by <i>PhyAddress</i>. This Ethernet
 * driver does not require the device to be stopped before writing to the PHY.
 * It is the responsibility of the calling code to stop the device if it is
 * deemed necessary.
 *
 * Note that the TEMAC hardware provides the ability to talk to a PHY that
 * adheres to the Media Independent Interface (MII) as defined in the IEEE 802.3
 * standard.
 *
 * <b>It is important that calling code set up the MDIO clock with
 * labx_eth_PhySetMdioDivisor() prior to accessing the PHY with this function.</b>
 *
 * @param InstancePtr references the TEMAC channel on which to operate.
 * @param PhyAddress is the address of the PHY to be written (multiple
 *        PHYs supported).
 * @param RegisterNum is the register number, 0-31, of the specific PHY register
 *        to write.
 * @param PhyData is the 16-bit value that will be written to the register.
 *
 * @return N/A
 *
 * @note
 *
 * This routine accesses the hard TEMAC registers through a shared interface
 * between both channels of the TEMAC. Becuase of this, the application/OS code
 * must provide mutual exclusive access to this routine with any of the other
 * routines in this TEMAC driverr.<br><br>
 *
 * There is the possibility that this function will not return if the hardware
 * is broken (i.e., it never sets the status bit indicating that the write is
 * done). If this is of concern, the calling code should provide a mechanism
 * suitable for recovery.
 *
 ******************************************************************************/
/*
void labx_eth_PhyWrite(XLlTemac *InstancePtr, u32 PhyAddress,
                       u32 RegisterNum, u16 PhyData)
{
	XASSERT_VOID(InstancePtr != NULL);

    if(labx_eth_ReadReg(InstancePtr->Config.BaseAddress, MDIO_CONTROL_REG) &
       PHY_MDIO_BUSY) {
      printk("Read issued while PHY busy!\n");
      return;
    }

    // Write the data first, then the control register 
    labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MDIO_DATA_REG, PhyData);
    InstancePtr->MdioState = MDIO_STATE_BUSY;
    labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MDIO_CONTROL_REG,
                      (PHY_MDIO_WRITE | 
                       ((PhyAddress & PHY_ADDR_MASK) << PHY_ADDR_SHIFT) |
                       (RegisterNum & PHY_REG_ADDR_MASK)));

    // The ISR will wake us up when the write completes 
    wait_event_interruptible_timeout(InstancePtr->PhyWait,
                                     (InstancePtr->MdioState == MDIO_STATE_READY),
                                     MDIO_TIMEOUT_JIFFIES);

    if(InstancePtr->MdioState != MDIO_STATE_READY) {
      printk("MDIO write timeout!\n");
    }
}

*/

void labx_eth_PhyWrite(XLlTemac *InstancePtr, u32 PhyAddress,
                       u32 RegisterNum, u16 PhyData)
{
	XASSERT_VOID(InstancePtr != NULL);

    if(labx_eth_ReadReg(InstancePtr->Config.BaseAddress, MDIO_CONTROL_REG) &
       PHY_MDIO_BUSY) {
      printk("Read issued while PHY busy!\n");
      return;
    }

    /* the 802.3ae Clause45 protocol for the 10G PHY */
    if(InstancePtr->Config.MacWidth == 64) {
      labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MDIO_DATA_REG, RegisterNum);
      InstancePtr->MdioState = MDIO_STATE_BUSY;
      
      labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MDIO_CONTROL_REG,
                      (PHY_10G_MDIO_ADDR | 
                       ((PhyAddress & PHY_ADDR_MASK) << PHY_ADDR_SHIFT) |
                       (InstancePtr->Config.PhyType & PHY_10G_DEVTYPE_MASK))); 
        
      wait_event_interruptible_timeout(InstancePtr->PhyWait,
                                     (InstancePtr->MdioState == MDIO_STATE_READY),
                                     MDIO_TIMEOUT_JIFFIES);

      if(InstancePtr->MdioState != MDIO_STATE_READY)
        printk("MDIO write timeout!\n");
        
      labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MDIO_DATA_REG, PhyData);
      InstancePtr->MdioState = MDIO_STATE_BUSY;
      
      labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MDIO_CONTROL_REG,
                      (PHY_10G_MDIO_WRITE | 
                       ((PhyAddress & PHY_ADDR_MASK) << PHY_ADDR_SHIFT) |
                       (InstancePtr->Config.PhyType & PHY_10G_DEVTYPE_MASK))); 
    }
    
    /* The old standard MIDO protocol, Clause 22 */
    else {
    /* Write the data first, then the control register */
      labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MDIO_DATA_REG, PhyData);
      InstancePtr->MdioState = MDIO_STATE_BUSY;
      labx_eth_WriteReg(InstancePtr->Config.BaseAddress, MDIO_CONTROL_REG,
                        (PHY_MDIO_WRITE | 
                         ((PhyAddress & PHY_ADDR_MASK) << PHY_ADDR_SHIFT) |
                         (RegisterNum & PHY_REG_ADDR_MASK)));
    }  

    /* The ISR will wake us up when the write completes */
    wait_event_interruptible_timeout(InstancePtr->PhyWait,
                                     (InstancePtr->MdioState == MDIO_STATE_READY),
                                     MDIO_TIMEOUT_JIFFIES);

    if(InstancePtr->MdioState != MDIO_STATE_READY) {
      printk("MDIO write timeout!\n");
    }
}
