/*
 * phy_marvell.c
 *
 * (c) Copytight 2010 Meyer Sound Laboratories, Inc.
 * (c) Copyright 2005-2008 Xilinx Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/mii.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/xilinx_devices.h>
#include <asm/io.h>
#include <linux/ethtool.h>
#include <linux/vmalloc.h>

#include "xbasic_types.h"
#include "xlltemac.h"
#include "xllfifo.h"
#include "xlldma.h"
#include "xlldma_bdring.h"
#include "xlltemac_phy_common.h"
#include "xlltemac_phy_marvell.h"

#ifdef CONFIG_XILINX_LLTEMAC_MARVELL_88E1112_GMII

/* Page-based register access, used only for 88e1112 */

static inline void _XLlTemac_PhyReadPage(XLlTemac *InstancePtr, u32 PhyAddress,
					 u32 RegisterNum, u16 *PhyDataPtr,
					 u8 Page)
{
  unsigned long flags;
  u16 pagetmp;
  
  spin_lock_irqsave(&XTE_spinlock, flags);
  XLlTemac_PhyRead(InstancePtr, PhyAddress, MARVELL_88E1112_PAGE_REG,
		   &pagetmp);
  XLlTemac_PhyWrite(InstancePtr, PhyAddress, MARVELL_88E1112_PAGE_REG,
		    (pagetmp&0x7f00)|(u16)Page);
  XLlTemac_PhyRead(InstancePtr, PhyAddress, RegisterNum, PhyDataPtr);
  XLlTemac_PhyWrite(InstancePtr, PhyAddress, MARVELL_88E1112_PAGE_REG,
		    pagetmp);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
}

static inline void _XLlTemac_PhyWritePage(XLlTemac *InstancePtr,
					  u32 PhyAddress,
					  u32 RegisterNum, u16 PhyData,
					  u8 Page)
{
  unsigned long flags;
  u16 pagetmp;
  
  spin_lock_irqsave(&XTE_spinlock, flags);
  XLlTemac_PhyRead(InstancePtr, PhyAddress, MARVELL_88E1112_PAGE_REG,
		   &pagetmp);
  XLlTemac_PhyWrite(InstancePtr, PhyAddress, MARVELL_88E1112_PAGE_REG,
		    (pagetmp&0x7f00)|(u16)Page);
  XLlTemac_PhyWrite(InstancePtr, PhyAddress, RegisterNum,
		    PhyData);
  XLlTemac_PhyWrite(InstancePtr, PhyAddress, MARVELL_88E1112_PAGE_REG,
		    pagetmp);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
}
#endif

#ifdef CONFIG_XILINX_LLTEMAC_MARVELL_88E1112_GMII
/* Ethernet LEDs, implemented only for 88e1112 */
static unsigned int led_state_flags[LED_STATE_NSTATES]=
  {
    0,
    LED_STATE_FLAG_GREEN 
    | LED_STATE_FLAG_BLINK_SLOW | LED_STATE_FLAG_BLINK_ACTIVITY,
    LED_STATE_FLAG_YELLOW | LED_STATE_FLAG_GREEN 
    | LED_STATE_FLAG_BLINK_SLOW | LED_STATE_FLAG_BLINK_ACTIVITY,
    LED_STATE_FLAG_GREEN | LED_STATE_FLAG_BLINK_ACTIVITY
  };

static int led_state_polarity=0;

int xlltemac_eth_leds_setup(char *s)
{
  
  led_state curr_state;
  unsigned int new_led_state_flags,flags_defined,i;
  curr_state=LED_STATE_OFF;
  new_led_state_flags=0;
  flags_defined=0;
  for(i=0;s[i];i++)
    {
      switch(s[i])
	{
	case ',':
	  flags_defined=1;
	  led_state_flags[curr_state]=new_led_state_flags;
	  curr_state++;
	  if(curr_state>=LED_STATE_NSTATES)
	    {
	      printk(KERN_ERR
		     "XLlTemac: too many LED states on the command line\n");
	      return 0;
	    }
	  new_led_state_flags=0;
	  break;
	case '+':
	  led_state_polarity=0;
	  break;
	case '-':
	  led_state_polarity=-1;
	  break;
	case 'y':
	case 'Y':
	  flags_defined=1;
	  new_led_state_flags|=LED_STATE_FLAG_YELLOW;
	  break;
	case 'g':
	case 'G':
	  flags_defined=1;
	  new_led_state_flags|=LED_STATE_FLAG_GREEN;
	  break;
	case 'b':
	case 'B':
	  flags_defined=1;
	  new_led_state_flags|=LED_STATE_FLAG_BLINK;
	  break;
	case 'a':
	case 'A':
	  flags_defined=1;
	  new_led_state_flags|=LED_STATE_FLAG_BLINK_ACTIVITY;
	  break;
	case 's':
	case 'S':
	  flags_defined=1;
	  new_led_state_flags|=LED_STATE_FLAG_BLINK_SLOW;
	  break;
	case 'f':
	case 'F':
	  flags_defined=1;
	  new_led_state_flags|=LED_STATE_FLAG_BLINK_FLIP;
	  break;
	case '0':
	  flags_defined=1;
	  new_led_state_flags=0;
	  break;
	default:
	  printk(KERN_ERR
		 "XLlTemac: unknown LED flag '%c'\n",s[i]);
	}
    }
  if(curr_state<LED_STATE_NSTATES&&flags_defined)
    {
      led_state_flags[curr_state]=new_led_state_flags;
    }
  return 0;
}

void xlltemac_leds_initialize(struct xlltemac_net_local *lp,int phy_addr,
			      led_state state)
{
  u32 led_reg16=0;
  lp->led_blink^=1;
  if((led_state_flags[state]&LED_STATE_FLAG_BLINK_SLOW) && lp->led_blink)
    {
      led_reg16=(0x08<<12)|(0x08<<4)|0x08;
    }
  else
    {
      if((led_state_flags[state]&LED_STATE_FLAG_GREEN)
	 ||((led_state_flags[state]&LED_STATE_FLAG_BLINK_FLIP)
	    && lp->led_blink))
	{
	  if(led_state_flags[state]&LED_STATE_FLAG_BLINK_ACTIVITY)
	    {
	      led_reg16|=0x1<<12;
	    }
	  else
	    {
	      if(led_state_flags[state]&LED_STATE_FLAG_BLINK)
		{
		  led_reg16|=0xb<<12;
		}
	      else
		{
		  led_reg16|=0x9<<12;
		}
	    }
	}
      else
	{
	  led_reg16|=0x8<<12;
	}


      if((led_state_flags[state]&LED_STATE_FLAG_YELLOW)
	 ||((led_state_flags[state]&LED_STATE_FLAG_BLINK_FLIP)
	    && !lp->led_blink))
	{
	  if(led_state_flags[state]&LED_STATE_FLAG_BLINK_ACTIVITY)
	    {
	      led_reg16|=(0x1<<4)|0x1;
	    }
	  else
	    {
	      if(led_state_flags[state]&LED_STATE_FLAG_BLINK)
		{
		  led_reg16|=(0xb<<4)|0xb;
		}
	      else
		{
		  led_reg16|=(0x9<<4)|0x9;
		}
	    }
	}
      else
	{
	  led_reg16|=(0x8<<4)|0x8;
	}
    }
  _XLlTemac_PhyWritePage(&lp->Emac, phy_addr, 17, 
			 led_state_polarity ? 0x0044 : 0, 3);
  _XLlTemac_PhyWritePage(&lp->Emac, phy_addr, 16, led_reg16 ,3);

}
#endif

/*
 * Perform any necessary special phy setup.
 */
void xlltemac_phy_setup_marvell(struct xlltemac_net_local *lp)
{
#ifdef CONFIG_XILINX_LLTEMAC_MARVELL_88E1111_RGMII
  u16 Register;
  
  /*
   * Set up MAC interface
   *
   * Write 0x0cc3 to reg 20 in PHY
   *      5432 1098 7654 3210
   *      ---- ---- ---- ----
   * 0cc3=0000 1100 1100 0011
   *           downshift counter (bits 11-9): 110 = 7 times
   *              downshift enable (bit 8): 0 = enable
   *                RGMII timing control (bit 7): 1 = add delay to rx clk ro rxd
   *                outputs
   *                 Default Mac interface speed (bits 6-4): 100 = 10mbps 2.5 mhz
   *                 (between phy and temac - gets renegotiated)
   *                     reserved (bit 3)
   *                      DTE detect (bit 2): 0 disabled
   *                       RGMII transmit timing control (bit 1): 1 = add delay
   *                       to tx clk ro txd outputs
   *                        Transmitter Disable (bit 0): 1 = enabled
   */
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, MARVELL_88E1111_EXTENDED_PHY_CTL_REG_OFFSET, 0x0cc3);
  
  /*
   * Set RGMII to copper with correct hysterisis and correct mode
   * Disable fiber/copper auto sel, choose copper
   * RGMII /Modified MII to copper mode
   *
   * Write 0x848b to reg 27
   *      5432 1098 7654 3210
   *      ---- ---- ---- ----
   * 848b=1000 0100 1000 1011
   *      Fiber/Copper Auto Selection (bit 15): 1 = disable auto selection
   *            Interrupt Polarity (bit 10): 1 = int active low
   *              DTE detect status drop hysteresis (bts 8-5): 0100 = report 20s after DTE power status drop
   *                     HWCFG mode (bits 3-0): 1011 = RGMII/Modified MII to Copper
   */
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, MARVELL_88E1111_EXTENDED_PHY_STATUS_REG_OFFSET, 0x848b);
  
  /*
   * Reset the PHY
   */
  _XLlTemac_PhyRead(&lp->Emac, lp->gmii_addr, MII_BMCR, &Register);
  Register |= BMCR_RESET;
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, MII_BMCR, Register);
#endif /* CONFIG_XILINX_LLTEMAC_MARVELL_88E1111_RGMII */
  
#if defined(CONFIG_XILINX_LLTEMAC_MARVELL_88E1111_GMII)		\
  || defined(CONFIG_XILINX_LLTEMAC_MARVELL_88E1112_GMII)
  
  u16 Register;
#ifdef CONFIG_XILINX_LLTEMAC_MARVELL_88E1112_GMII
  /* LED controls are specific to 88e1112  */
  xlltemac_leds_initialize(lp, lp->gmii_addr, LED_STATE_OFF);
#endif
  
  /*
   * Enable autonegotiation, Reset the PHY if it's not the first time
   */
  _XLlTemac_PhyRead(&lp->Emac, lp->gmii_addr, MII_BMCR, &Register);
  
  Register |= BMCR_ANENABLE;
#ifdef PHY_USE_RESET_FLAG
  if(lp->reset_flag)
    {
#if 0
      printk(KERN_INFO
	     "%s: XLlTemac: Autonegotiation on, Reset\n",
	     lp->ndev->name);
#endif
#endif
      Register |= BMCR_RESET;
#ifdef PHY_USE_RESET_FLAG
    }
  else
    {
#if 0
      printk(KERN_INFO
	     "%s: XLlTemac: Autonegotiation on\n",
	     lp->ndev->name);
#endif
      lp->reset_flag=1;
    }
#endif
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, MII_BMCR, Register);

#ifdef CONFIG_XILINX_LLTEMAC_MARVELL_88E1112_GMII  
  /* configure copper interface and enable phy register 4 */
  // FIXME -- copper is hardcoded here
  _XLlTemac_PhyReadPage(&lp->Emac, lp->gmii_addr, 16, &Register,2);
  Register&=0xfc7f;
  Register|=0x0280;
  _XLlTemac_PhyWritePage(&lp->Emac, lp->gmii_addr, 16, Register,2);
#endif
  
  /* detect DTE, report DTE drop with 5s delay */
  /*
    _XLlTemac_PhyRead(&lp->Emac, lp->gmii_addr, 26, &Register);
    Register&=0x7e01;
    Register|=0x0110;
    _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, 26, Register);
  */
#endif
}
