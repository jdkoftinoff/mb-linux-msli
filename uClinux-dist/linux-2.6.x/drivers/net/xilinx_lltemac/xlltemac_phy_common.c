/*
 * xlltemac_phy_common.c
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
#include "xlltemac_common.h"
#include "xlltemac_phy_common.h"
#include "xlltemac_phy_marvell.h"

void _XLlTemac_PhySetMdioDivisor(XLlTemac *InstancePtr,u8 Divisor)
{
  unsigned long flags;
  
  spin_lock_irqsave(&XTE_spinlock, flags);
  XLlTemac_PhySetMdioDivisor(InstancePtr, Divisor);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
}

void _XLlTemac_PhyRead(XLlTemac *InstancePtr, u32 PhyAddress,
		       u32 RegisterNum, u16 *PhyDataPtr)
{
  unsigned long flags;
  
  spin_lock_irqsave(&XTE_spinlock, flags);
  XLlTemac_PhyRead(InstancePtr, PhyAddress, RegisterNum, PhyDataPtr);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
}

void _XLlTemac_PhyWrite(XLlTemac *InstancePtr, u32 PhyAddress,
			u32 RegisterNum, u16 PhyData)
{
  unsigned long flags;
  
  spin_lock_irqsave(&XTE_spinlock, flags);
  XLlTemac_PhyWrite(InstancePtr, PhyAddress, RegisterNum, PhyData);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
}

/*
 * Perform any necessary special phy setup.
 */
void xlltemac_phy_setup(struct xlltemac_net_local *lp)
{
#ifdef CONFIG_XILINX_LLTEMAC_NATIONAL_DP83865_GMII
  
  u16 RegValue;
  
  printk(KERN_INFO "NATIONAL DP83865 PHY\n");
  RegValue = NATIONAL_DP83865_CONTROL_INIT;
  /*Do not reset phy*/
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr,
		     NATIONAL_DP83865_CONTROL, RegValue);
  
  _XLlTemac_PhyRead(&lp->Emac, lp->gmii_addr,
		    NATIONAL_DP83865_STATUS, &RegValue);
  
  _XLlTemac_PhyRead(&lp->Emac, lp->gmii_addr,
		    NATIONAL_DP83865_STATUS, &RegValue);
#endif
  
#ifdef PHY_VITESSE_8211_SGMII
  u16 Register;
  /* Setup the MAC into SGMII mode */
  Register = 0xAA23;
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, 31, 0x0000);
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, 23, Register);
  
  /* Set SIGDET as an input */
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, 31, 0x0001);
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, 19, 0x0002);
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, 31, 0x0000);
  
  /* Signal all possible autonegotiation modes */
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr,  MII_ADVERTISE,
		     ADVERTISE_ALL);
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, MII_EXADVERTISE,
		     ADVERTISE_1000FULL| ADVERTISE_1000HALF);
  
  /*
   * Reset the PHY
   */
  _XLlTemac_PhyRead(&lp->Emac, lp->gmii_addr, MII_BMCR, &Register);
  Register |= BMCR_RESET;
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, MII_BMCR, Register);
#endif
  
#if defined(CONFIG_XILINX_LLTEMAC_MARVELL_88E1111_RGMII) \
  || defined(CONFIG_XILINX_LLTEMAC_MARVELL_88E1111_GMII) \
  || defined(CONFIG_XILINX_LLTEMAC_MARVELL_88E1112_GMII)
  xlltemac_phy_setup_marvell(lp);
#endif
}


int xlltemac_renegotiate_speed_init(struct net_device *dev, int speed,
				    DUPLEX duplex)
{
  struct xlltemac_net_local *lp =
    (struct xlltemac_net_local *) netdev_priv(dev);
  u16 phy_reg0 = BMCR_ANENABLE | BMCR_ANRESTART;
  u16 phy_reg4;
  u16 phy_reg9 = 0;
  
  
  /*
   * It appears that the 10baset full and half duplex settings
   * are overloaded for gigabit ethernet
   */
  if (speed == 0) {
    phy_reg4 = ADVERTISE_10FULL | ADVERTISE_10HALF 
      | ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_CSMA;
    phy_reg9 = EX_ADVERTISE_1000FULL | EX_ADVERTISE_1000HALF;
    
  }
  else if ((duplex == FULL_DUPLEX) && (speed == 10)) {
    phy_reg4 = ADVERTISE_10FULL | ADVERTISE_CSMA;
  }
  else if ((duplex == FULL_DUPLEX) && (speed == 100)) {
    phy_reg4 = ADVERTISE_100FULL | ADVERTISE_CSMA;
  }
  else if ((duplex == FULL_DUPLEX) && (speed == 1000)) {
    phy_reg4 = ADVERTISE_CSMA;
    phy_reg9 = EX_ADVERTISE_1000FULL;
  }
  else if (speed == 10) {
    phy_reg4 = ADVERTISE_10HALF | ADVERTISE_CSMA;
  }
  else if (speed == 100) {
    phy_reg4 = ADVERTISE_100HALF | ADVERTISE_CSMA;
  }
  else if (speed == 1000) {
    phy_reg4 = ADVERTISE_CSMA;
    phy_reg9 = EX_ADVERTISE_1000HALF;
  }
  else {
    printk(KERN_ERR
	   "%s: XLlTemac: unsupported speed requested: %d\n",
	   dev->name, speed);
    return -1;
  }
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, MII_ADVERTISE, phy_reg4);
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, MII_EXADVERTISE, phy_reg9);
  /* initiate an autonegotiation of the speed */
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, MII_BMCR, phy_reg0);
  printk(KERN_INFO
	 "%s: XLlTemac: Autonegotiation started\n", dev->name);
  return 0;
}

int xlltemac_renegotiate_speed_wait(struct net_device *dev, int speed,
				    DUPLEX duplex)
{
  struct xlltemac_net_local *lp
    = (struct xlltemac_net_local *) netdev_priv(dev);
  int retries = 2;
  int wait_count;
  u16 phy_reg0 = BMCR_ANENABLE | BMCR_ANRESTART;
  u16 phy_reg1;
  u16 phy_reg4;
  u16 phy_reg9 = 0;
  
  
  /*
   * It appears that the 10baset full and half duplex settings
   * are overloaded for gigabit ethernet
   */
  if (speed == 0) {
    phy_reg4 = ADVERTISE_10FULL | ADVERTISE_10HALF 
      | ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_CSMA;
    phy_reg9 = EX_ADVERTISE_1000FULL | EX_ADVERTISE_1000HALF;
    
  }
  else if ((duplex == FULL_DUPLEX) && (speed == 10)) {
    phy_reg4 = ADVERTISE_10FULL | ADVERTISE_CSMA;
  }
  else if ((duplex == FULL_DUPLEX) && (speed == 100)) {
    phy_reg4 = ADVERTISE_100FULL | ADVERTISE_CSMA;
  }
  else if ((duplex == FULL_DUPLEX) && (speed == 1000)) {
    phy_reg4 = ADVERTISE_CSMA;
    phy_reg9 = EX_ADVERTISE_1000FULL;
  }
  else if (speed == 10) {
    phy_reg4 = ADVERTISE_10HALF | ADVERTISE_CSMA;
  }
  else if (speed == 100) {
    phy_reg4 = ADVERTISE_100HALF | ADVERTISE_CSMA;
  }
  else if (speed == 1000) {
    phy_reg4 = ADVERTISE_CSMA;
    phy_reg9 = EX_ADVERTISE_1000HALF;
  }
  else {
    printk(KERN_ERR
	   "%s: XLlTemac: unsupported speed requested: %d\n",
	   dev->name, speed);
    return -1;
  }
  
  /*
   * link status in register 1:
   * first read / second read:
   * 0               0           link is down
   * 0               1           link is up (but it was down earlier)
   * 1               0           link is down (but it was just up)
   * 1               1           link is up
   *
   */
  _XLlTemac_PhyRead(&lp->Emac, lp->gmii_addr, MII_BMSR, &phy_reg1);
  _XLlTemac_PhyRead(&lp->Emac, lp->gmii_addr, MII_BMSR, &phy_reg1);
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, MII_ADVERTISE, phy_reg4);
  _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, MII_EXADVERTISE, phy_reg9);
  
  while (retries--) {
    /* initiate an autonegotiation of the speed */
    _XLlTemac_PhyWrite(&lp->Emac, lp->gmii_addr, MII_BMCR, phy_reg0);
    
    wait_count = 20;	/* so we don't loop forever */
    while (wait_count--) {
      /* wait a bit for the negotiation to complete */
      mdelay(500);
      _XLlTemac_PhyRead(&lp->Emac, lp->gmii_addr, MII_BMSR,
			&phy_reg1);
      _XLlTemac_PhyRead(&lp->Emac, lp->gmii_addr, MII_BMSR,
			&phy_reg1);
      
      if ((phy_reg1 & BMSR_LSTATUS) &&
	  (phy_reg1 & BMSR_ANEGCOMPLETE))
	break;
      
    }
    
    if (phy_reg1 & BMSR_LSTATUS) {
      printk(KERN_INFO
	     "%s: XLlTemac: We renegotiated the speed to: %d\n",
	     dev->name, speed);
      return 0;
    }
    else {
      printk(KERN_ERR
	     "%s: XLlTemac: Not able to set the speed to %d (status: 0x%0x)\n",
	     dev->name, speed, phy_reg1);
      return -1;
    }
  }
  
  printk(KERN_ERR
	 "%s: XLlTemac: Not able to set the speed to %d\n", dev->name,
	 speed);
  return -1;
}

/*
 * The PHY registers read here should be standard registers in all PHY chips
 */
int xlltemac_get_phy_status(struct net_device *dev, DUPLEX * duplex,
			    int *linkup)
{
  struct xlltemac_net_local *lp
    = (struct xlltemac_net_local *) netdev_priv(dev);
  u16 reg;
  
  _XLlTemac_PhyRead(&lp->Emac, lp->gmii_addr, MII_BMCR, &reg);
  *duplex = FULL_DUPLEX;
  
#ifdef CONFIG_XILINX_LLTEMAC_NATIONAL_DP83865_GMII
  _XLlTemac_PhyRead(&lp->Emac, lp->gmii_addr,
		    NATIONAL_DP83865_STATUS, &reg);
  *linkup=(reg & NATIONAL_DP83865_STATUS_LINK) != 0;
  
#else
#if defined(CONFIG_XILINX_LLTEMAC_MARVELL_88E1111_GMII)		\
  || defined(CONFIG_XILINX_LLTEMAC_MARVELL_88E1112_GMII)
  _XLlTemac_PhyRead(&lp->Emac, lp->gmii_addr, MII_BMSR, &reg);
  if(reg & BMSR_LSTATUS)
    {
      _XLlTemac_PhyRead(&lp->Emac, lp->gmii_addr,
			MARVELL_88E1111_PHY_SPECIFIC_STATUS_REG_OFFSET,
			&reg);
      if((reg & MARVELL_88E1111_LINK_DUPLEX) == 0)
	{
	  *duplex = HALF_DUPLEX;
	}
      *linkup = (reg & MARVELL_88E1111_LINK_UP) != 0;
    }
  else
    {
      *linkup = 0;
    }
#else
  _XLlTemac_PhyRead(&lp->Emac, lp->gmii_addr, MII_BMSR, &reg);
  *linkup = (reg & BMSR_LSTATUS) != 0;
#endif
#endif
  return 0;
}

/* Detect the PHY address by scanning addresses 0 to 31 and
 * looking at the MII status register (register 1) and assuming
 * the PHY supports 10Mbps full/half duplex. Feel free to change
 * this code to match your PHY, or hardcode the address if needed.
 */
/* Use MII register 1 (MII status register) to detect PHY */
#define PHY_DETECT_REG  1

/* Mask used to verify certain PHY features (or register contents)
 * in the register above:
 *  0x1000: 10Mbps full duplex support
 *  0x0800: 10Mbps half duplex support
 *  0x0008: Auto-negotiation support
 */
#define PHY_DETECT_MASK 0x1808

int xlltemac_detect_phy(struct xlltemac_net_local *lp, char *dev_name)
{
  u16 phy_reg;
  u32 phy_addr;
  
  for (phy_addr = 1; phy_addr <= 31; phy_addr++) {
    _XLlTemac_PhyRead(&lp->Emac, phy_addr, PHY_DETECT_REG, &phy_reg);
    
    if ((phy_reg != 0xFFFF) &&
	((phy_reg & PHY_DETECT_MASK) == PHY_DETECT_MASK)) {
      /* Found a valid PHY address */
      printk(KERN_INFO "XTemac: PHY detected at address %d.\n", phy_addr);
      return phy_addr;
    }
  }
  
  printk(KERN_WARNING "XTemac: No PHY detected.\n");
  return 0;		/* default to zero */
}


xtenet_phy_bus_t *xtenet_phy_bus=NULL;

spinlock_t xtenet_phy_bus_spinlock = SPIN_LOCK_UNLOCKED;


void xlltemac_remove_phy(struct xlltemac_net_local *lp)
{
  unsigned long flags;
  xtenet_phy_bus_t *phy_bus,**curr_phy_bus_ptr;
  int i;
  unsigned char mask;
  if(lp==NULL) return;

  spin_lock_irqsave(&xtenet_phy_bus_spinlock,flags);

  /* find a bus */
  for(curr_phy_bus_ptr=&xtenet_phy_bus;
      curr_phy_bus_ptr&&*curr_phy_bus_ptr
	&&(*curr_phy_bus_ptr)->BaseAddress!=lp->Emac.Config.PhyBaseAddress;
      curr_phy_bus_ptr=&(*curr_phy_bus_ptr)->next);

  if(curr_phy_bus_ptr&&(*curr_phy_bus_ptr))
    {
      phy_bus=*curr_phy_bus_ptr;

      /* look for the PHY */
      for(i=0,mask=1;i<phy_bus->n_phys;i++,mask<<=1)
	{
	  if(phy_bus->phys[i]==lp->gmii_addr)
	    {
	      /* free this PHY */
	      phy_bus->use_mask&=~mask;
	      printk(KERN_INFO
		     "XLlTemac: PHY 0x%02x removed from the bus for 0x%08x\n",
			 phy_bus->phys[i],phy_bus->BaseAddress);
	      /* if there are no other PHYs used on this bus,
		 remove the bus */
	      if(phy_bus->use_mask=='\0')
		{
		  *curr_phy_bus_ptr=phy_bus->next;
		  printk(KERN_INFO "XLlTemac: Deleting PHY bus for 0x%08x\n",
			 phy_bus->BaseAddress);
		  kfree(phy_bus);
		}
	      spin_unlock_irqrestore(&xtenet_phy_bus_spinlock,flags);
	      return;
	    }
	}
      printk(KERN_ERR
	     "XLlTemac: Can't find PHY 0x%02x on the bus for 0x%08x\n",
	     lp->gmii_addr,lp->Emac.Config.PhyBaseAddress);
    }
  else
    {
      printk(KERN_ERR "XLlTemac: Can't find a PHY bus for 0x%08x\n",
	     lp->Emac.Config.PhyBaseAddress);
    }
  spin_unlock_irqrestore(&xtenet_phy_bus_spinlock,flags);
}

int xlltemac_add_phy(struct xlltemac_net_local *lp)
{
  unsigned long flags;
  u16 phy_reg;
  int phy_addr,i,finished;
  unsigned char mask;
  xtenet_phy_bus_t *phy_bus,*curr_phy_bus;

  spin_lock_irqsave(&xtenet_phy_bus_spinlock,flags);

  /* do we already know a bus for this address? */
  for(phy_bus=xtenet_phy_bus;
      phy_bus && phy_bus->BaseAddress!=lp->Emac.Config.BaseAddress;
      phy_bus=phy_bus->next);

  if(phy_bus)
    {
      /* if we do, only look for the PHY here */
      lp->Emac.Config.PhyBaseAddress=phy_bus->BaseAddress;

      finished=0;
      phy_addr=0;
      /* look for the first available PHY */
      for(i=0,mask=1;i<phy_bus->n_phys;i++,mask<<=1)
	{
	  if((phy_bus->use_mask&mask)==0)
	    {
	      phy_bus->use_mask|=mask;
	      lp->Emac.Config.PhyBaseAddress=phy_bus->BaseAddress;
	      phy_addr=phy_bus->phys[i];
	      i=phy_bus->n_phys;
	      finished=1;
	    }
	}
      if(finished)
	{
	  printk(KERN_INFO
		 "XLlTemac: PHY bus for 0x%08x found at 0x%08x, phy 0x%02x\n",
		 lp->Emac.Config.BaseAddress,
		 lp->Emac.Config.PhyBaseAddress,phy_addr);
	}     
      else
	{
	  /* error, return local address and broadcast */
	  printk(KERN_ERR
 "XLlTemac: bus for 0x%08x is found, however there are no unused PHYs on it\n",
		 lp->Emac.Config.BaseAddress);
	  lp->Emac.Config.PhyBaseAddress=lp->Emac.Config.BaseAddress;
	  spin_unlock_irqrestore(&xtenet_phy_bus_spinlock,flags);
	  return 0;
	}
    }
  else
    {

      if(lp->Emac.Config.BaseAddress!=0)
	{
	  /* create and scan the bus */
	  phy_bus=kmalloc(sizeof(xtenet_phy_bus_t), GFP_KERNEL);

	  if(phy_bus==NULL)
	    {
	      /* error, return local address and broadcast */
	      printk(KERN_ERR
	 "XLlTemac: can't allocate memory for PHY bus descriptor for 0x%08x\n",
		     lp->Emac.Config.BaseAddress);
	      lp->Emac.Config.PhyBaseAddress=lp->Emac.Config.BaseAddress;
	      spin_unlock_irqrestore(&xtenet_phy_bus_spinlock,flags);
	      return 0;
	    }
	  phy_bus->next=NULL;
	  phy_bus->BaseAddress=lp->Emac.Config.BaseAddress;
	  phy_bus->n_phys=0;
	  phy_bus->use_mask=0;
	  for(phy_addr=1;phy_addr<=31;phy_addr++)
	    {
	      _XLlTemac_PhyRead(&lp->Emac, phy_addr, PHY_DETECT_REG, &phy_reg);
	      if ((phy_reg != 0xFFFF) &&
		  ((phy_reg & PHY_DETECT_MASK) == PHY_DETECT_MASK))
		{
		  phy_bus->phys[phy_bus->n_phys]=phy_addr;
#ifdef CONFIG_XILINX_LLTEMAC_MARVELL_88E1112_GMII
		  xlltemac_leds_initialize(lp,phy_addr, LED_STATE_OFF);
#endif
		  phy_bus->n_phys++;
		}
	      
	    }
	}
      else
	{
	  phy_bus = NULL;
	  phy_addr = 0;
	}

      if((phy_bus!=NULL) && (phy_bus->n_phys>0))
	{
	  /* if any PHYs are found, register this bus */
	  lp->Emac.Config.PhyBaseAddress=phy_bus->BaseAddress;
	  phy_addr=phy_bus->phys[0];
	  phy_bus->use_mask=1;

	  if(xtenet_phy_bus)
	    {
	      for(curr_phy_bus=xtenet_phy_bus;
		  curr_phy_bus->next;
		  curr_phy_bus=curr_phy_bus->next);
	      curr_phy_bus->next=phy_bus;
	    }
	  else
	    {
	      xtenet_phy_bus=phy_bus;
	    }
	  printk(KERN_INFO
	     "XLlTemac: PHY bus found at 0x%08x, phys: ",
	     lp->Emac.Config.BaseAddress);
	  for(i=0;i<phy_bus->n_phys;i++)
	    {
	      printk("0x%02x",phy_bus->phys[i]);
	      if(i==0)
		{
		  printk("(local)");
		}
	      if(i<phy_bus->n_phys-1)
		{
		  printk(", ");
		}
	    }
	  printk("\n");
	}
      else
	{
	  /* if no PHYs found, don't register this bus, look for PHYs
	     elsewhere */
	  if(phy_bus != NULL)
	    kfree(phy_bus);

	  /* find a bus on the same adapter with the address 0x40 less than
	     the current base address */

	  for(phy_bus=xtenet_phy_bus;
	      phy_bus && phy_bus->BaseAddress+0x40!=
		lp->Emac.Config.BaseAddress;
	      phy_bus=phy_bus->next);


	  /* if no such bus, use the first bus */
	  if(phy_bus==NULL)
	    {
	      phy_bus=xtenet_phy_bus;
	    }

	  finished=0;
	  while(!finished)
	    {
	      if(phy_bus)
		{
		  /* look for the first available PHY */
		  for(i=0,mask=1;i<phy_bus->n_phys;i++,mask<<=1)
		    {
		      if((phy_bus->use_mask&mask)==0)
			{
			  phy_bus->use_mask|=mask;
			  lp->Emac.Config.PhyBaseAddress=phy_bus->BaseAddress;
			  phy_addr=phy_bus->phys[i];
			  i=phy_bus->n_phys;
			  finished=1;
			}
		    }
		  if(finished)
		    {
		      printk(KERN_INFO
	          "XLlTemac: PHY bus for 0x%08x found at 0x%08x, phy 0x%02x\n",
			     lp->Emac.Config.BaseAddress,
			     lp->Emac.Config.PhyBaseAddress,phy_addr);
		      
		    }
		  else
		    {
		      /* if found nothing, look at the first bus, or 
			 if found nothing on the first bus, return an error */
		      if(phy_bus==xtenet_phy_bus)
			{
			  phy_bus=NULL;
			}
		      else
			{
			  phy_bus=xtenet_phy_bus;
			}
		    }
		}
	      if(phy_bus==NULL)
		{
		  /* error, return local address and broadcast */
		  printk(KERN_ERR
			 "XLlTemac: no PHYs found for 0x%08x\n",
			 lp->Emac.Config.BaseAddress);
		  lp->Emac.Config.PhyBaseAddress=lp->Emac.Config.BaseAddress;
		  spin_unlock_irqrestore(&xtenet_phy_bus_spinlock,flags);
		  return 0;
		}
	    }
	}
    }

  spin_unlock_irqrestore(&xtenet_phy_bus_spinlock,flags);
  return phy_addr;
}

void xlltemac_phy_off(struct xlltemac_net_local *lp)
{
#if defined(CONFIG_XILINX_LLTEMAC_MARVELL_88E1111_RGMII) \
  || defined(CONFIG_XILINX_LLTEMAC_MARVELL_88E1111_GMII) \
  || defined(CONFIG_XILINX_LLTEMAC_MARVELL_88E1112_GMII)
  xlltemac_phy_off_marvell(lp);
#endif
}
