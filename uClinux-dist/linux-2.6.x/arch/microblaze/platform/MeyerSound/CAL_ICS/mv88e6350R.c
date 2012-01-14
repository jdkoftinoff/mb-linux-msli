/*
 *  linux/arch/microblaze/platform/MeyerSound/CAL_ICS/mv88e6350R.c
 *
 *  Marvell 88E6350R ethernet switch driver
 *
 *  Written by Yi Cao (yi.cao@labxtechnologies.com)
 *
 *  Copyright (C) 2011 Meyer Sound Laboratories Inc, All Rights Reserved.
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


#include <linux/list.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/phy.h>


#include "mv88e6350R.h"
#include "mv88e6350R_defs.h"

/* Driver name */
#define DRIVER_NAME "mv88e6350R"

/* Major device number for the driver */
#define DRIVER_MAJOR 255

/* Maximum number of cal_mvEthSwitch and instance count */
#define MAX_INSTANCES 2
static uint32_t instanceCount;
#define NAME_MAX_SIZE    (256)


/* 88E6350R ports assigned to the two CPU ports */
#define CAL_ICS_EXT_PORT_0 (0)
#define CAL_ICS_EXT_PORT_1 (1)
#define CAL_ICS_CPU_PORT_0 (5)
#define CAL_ICS_CPU_PORT_1 (6)

/* Register constant definitions for the 88E6350R LinkStreet switch */
#define PHYS_CTRL_REG  (1)
#  define RGMII_MODE_RXCLK_DELAY   (0x8000)
#  define RGMII_MODE_GTXCLK_DELAY  (0x4000)
#  define FLOW_CTRL_FORCE_DISABLED (0x0040)
#  define FLOW_CTRL_FORCE_ENABLED  (0x00C0)
#  define FORCE_LINK_DOWN          (0x0010)
#  define FORCE_LINK_UP            (0x0030)
#  define FORCE_DUPLEX_HALF        (0x0004)
#  define FORCE_DUPLEX_FULL        (0x000C)
#  define FORCE_SPEED_10           (0x0000)
#  define FORCE_SPEED_100          (0x0001)
#  define FORCE_SPEED_1000         (0x0002)
#  define SPEED_AUTO_DETECT        (0x0003)

/* Register settings assigned to the CPU port:
 * Link forced up, 1 Gbps full-duplex
 * Using RGMII delay on switch IND input data
 * Using RGMII delay on switch OUTD output data
 */
#define CAL_ICS_CPU_PORT_0_PHYS_CTRL (RGMII_MODE_RXCLK_DELAY  | \
                                         RGMII_MODE_GTXCLK_DELAY | \
                                         FORCE_LINK_UP           | \
                                         FORCE_DUPLEX_FULL       | \
                                         FORCE_SPEED_1000)

                                         
/* Register settings assigned to the SFP port:
 * Link forced up, 1 Gbps full-duplex
 * Using RGMII delay on switch IND input data
 * Using RGMII delay on switch OUTD output data
 */
#define CAL_ICS_CPU_PORT_1_PHYS_CTRL (RGMII_MODE_RXCLK_DELAY  | \
                                         RGMII_MODE_GTXCLK_DELAY | \
                                         FORCE_LINK_UP           | \
                                         FORCE_DUPLEX_FULL       | \
                                         FORCE_SPEED_1000)

                                          
/* Bit-mask of enabled ports  */
#define CAL_ICS_ENABLED_PORTS ((1 << 6) | (1 << 5) | (1 << 1) | (1 << 0))

/* Number of copper 1000Base-TX ports for CAL_ICS */
#define CAL_ICS_COPPER_PORTS (2)


typedef char           MV_8;
typedef unsigned char  MV_U8;

typedef int            MV_32;
typedef unsigned int   MV_U32;

typedef short          MV_16;
typedef unsigned short MV_U16;

/* Per-port switch registers */
#define MV_SWITCH_PORT_CONTROL_REG 0x04
#define MV_SWITCH_PORT_VMAP_REG    0x06
#define MV_SWITCH_PORT_VID_REG     0x07

/* Port LED control register and indirect registers */
#define MV_SWITCH_PORT_LED_CTRL_REG              0x16
#  define MV_SWITCH_LED_CTRL_UPDATE     0x8000
#  define MV_SWITCH_LED_CTRL_PTR_MASK      0x7
#  define MV_SWITCH_LED_CTRL_PTR_SHIFT      12
#    define MV_SWITCH_LED_01_CTRL_REG       0x0
#      define MV_SWITCH_LED0_1G_100M_ACT   0x001
#      define MV_SWITCH_LED1_100M_10M_ACT  0x010
#      define MV_SWITCH_LED1_By_BLINK_RATE 0x000 

#    define MV_SWITCH_LED_23_CTRL_REG       0x1
#      define MV_SWITCH_LED2_1G_LINK       0x009
#      define MV_SWITCH_LED3_LINK_ACT      0x0A0

#    define MV_SWITCH_LED_RATE_CTRL_REG     0x6
#    define MV_SWITCH_LED_SPECIAL_CTRL_REG  0x7
#      define MV_SWITCH_LED_SPECIAL_NONE  0x000
#  define MV_SWITCH_LED_CTRL_DATA_MASK  0x03FF

#define MV_SWITCH_LED_WRITE(ledReg, regValue)                                   \
    (MV_SWITCH_LED_CTRL_UPDATE |                                                \
     ((ledReg & MV_SWITCH_LED_CTRL_PTR_MASK) << MV_SWITCH_LED_CTRL_PTR_SHIFT) | \
     (regValue & MV_SWITCH_LED_CTRL_DATA_MASK))

/* E6350R-related */
#define MV_E6350R_MAX_PORTS_NUM 7

#define XPAR_XPS_GPIO_0_BASEADDR 0x820F0000
#define LABX_MDIO_ETH_BASEADDR 0x82050000
#define MDIO_CONTROL_REG      (0x00000000)
#  define PHY_MDIO_BUSY       (0x80000000)
#  define PHY_REG_ADDR_MASK   (0x01F)
#  define PHY_ADDR_MASK       (0x01F)
#  define PHY_ADDR_SHIFT      (5)
#  define PHY_MDIO_READ       (0x0400)
#  define PHY_MDIO_WRITE      (0x0000)
#define MDIO_DATA_REG         (0x00000004)

#define LABX_MAC_REGS_BASE    (0x00001000)
#define MAC_MDIO_CONFIG_REG   (LABX_MAC_REGS_BASE + 0x0014)
#define LABX_ETHERNET_MDIO_DIV  (0x28)
#  define MDIO_DIVISOR_MASK  (0x0000003F)
#  define MDIO_ENABLED       (0x00000040)

/* Performs a register write to a switch register */
void REG_WRITE(int phy_addr, int reg_addr, int phy_data)
{
  unsigned int addr;

  /* Write the data first, then the control register */
  addr = (LABX_MDIO_ETH_BASEADDR + MDIO_DATA_REG);
  *((volatile unsigned int *) addr) = phy_data;
  addr = (LABX_MDIO_ETH_BASEADDR + MDIO_CONTROL_REG);
  *((volatile unsigned int *) addr) = 
    (PHY_MDIO_WRITE | ((phy_addr & PHY_ADDR_MASK) << PHY_ADDR_SHIFT) |
     (reg_addr & PHY_REG_ADDR_MASK));
  while(*((volatile unsigned int *) addr) & PHY_MDIO_BUSY);
}

/* Performs a register read from a switch register */
unsigned int REG_READ(int phy_addr, int reg_addr)
{
  unsigned int addr;
  unsigned int readValue;

  /* Write to the MDIO control register to initiate the read */
  addr = (LABX_MDIO_ETH_BASEADDR + MDIO_CONTROL_REG);
  *((volatile unsigned int *) addr) = 
    (PHY_MDIO_READ | ((phy_addr & PHY_ADDR_MASK) << PHY_ADDR_SHIFT) |
     (reg_addr & PHY_REG_ADDR_MASK));
  while(*((volatile unsigned int *) addr) & PHY_MDIO_BUSY);
  addr = (LABX_MDIO_ETH_BASEADDR + MDIO_DATA_REG);
  readValue = *((volatile unsigned int *) addr);
  return(readValue);
}

/* Performs a register write to a switch PHY register */
void REG_PHY_WRITE(int phy_addr, int reg_addr, int phy_data)
{
  REG_WRITE(MV_REG_GLOBAL2, 25, phy_data);
  REG_WRITE(MV_REG_GLOBAL2, 24, 0x9400 | (phy_addr << 5) | reg_addr);
  while (REG_READ(MV_REG_GLOBAL2, 24) & 0x8000); // Should only take a read or two...
}

/* Performs a register read from a switch PHY register */
unsigned int REG_PHY_READ(int phy_addr, int reg_addr)
{
  REG_WRITE(MV_REG_GLOBAL2, 24, 0x9800 | (phy_addr << 5) | reg_addr);
  while (REG_READ(MV_REG_GLOBAL2, 24) & 0x8000); // Should only take a read or two...
  return REG_READ(MV_REG_GLOBAL2, 25);
}

static int marvell_config_aneg(struct phy_device *phydev)
{
  return 0;
}

static int marvell_read_status(struct phy_device *phydev)
{
  unsigned int portStatusReg = REG_READ(MV_PORT_REG(phydev->phyaddr), 0x00);

  switch((portStatusReg >> 8) & 0x3) {
    case 0: phydev->speed = SPEED_10; break;
    case 1: phydev->speed = SPEED_100; break;
    case 2: phydev->speed = SPEED_1000; break;
    default: printk("88E6350R unknown speeed\n"); break;
  }

  phydev->duplex = (portStatusReg >> 10) ? DUPLEX_FULL : DUPLEX_HALF;
  phydev->link = (portStatusReg >> 11) & 1;

  return 0;
}

static struct phy_driver phy_driver = {
	.phy_id = 0x01410e70,
	.phy_id_mask = 0xfffffff0,
	.name = "Marvell 88E6350R",
	.features = PHY_GBIT_FEATURES,
	.flags = 0,
	.config_aneg = &marvell_config_aneg,
	.read_status = &marvell_read_status,
	.driver = { .owner = THIS_MODULE },
};

static void mv88e6350R_free_mdio_bus(struct mii_bus *bus)
{
}

int mv88e6350R_mdio_read(struct mii_bus *bus, int phy_id, int regnum)
{
  return REG_PHY_READ(phy_id, regnum);
}

int mv88e6350R_mdio_write(struct mii_bus *bus, int phy_id, int regnum, u16 val)
{
  REG_PHY_WRITE(phy_id, regnum, val);
  return 0;
}

int mv88e6350R_mdio_reset(struct mii_bus *bus)
{
  return 0;
}

int mv88e6350R_mdio_bus_init(struct device *dev)
{
  struct mii_bus *new_bus;
  int ret = -ENOMEM;
  int i;
  static int mdio_phy_irqs[PHY_MAX_ADDR] = {};

  new_bus = mdiobus_alloc();
  if (!new_bus) {
    printk("Failed mdiobus_alloc(), returning\n");
    goto out_free_bus;
  }

  new_bus->read = mv88e6350R_mdio_read;
  new_bus->write = mv88e6350R_mdio_write;
  new_bus->reset = mv88e6350R_mdio_reset;

  new_bus->name = "MV88E6350R MDIO Bus";
  ret = -ENODEV;

  new_bus->phy_mask = ~3;
  new_bus->irq = mdio_phy_irqs;
  new_bus->parent = dev;

  if (new_bus->phy_mask == ~0) {
    goto out_free_bus;
  }

  for (i = 0; i < PHY_MAX_ADDR; i++) {
    if (!new_bus->irq[i]) {
      new_bus->irq[i] = PHY_POLL;
    }
  }

  snprintf(new_bus->id, MII_BUS_ID_SIZE, "%s", "mv88e6350r");

  ret = mdiobus_register(new_bus);
  if (ret)
  {
    printk("Failed mdiobus_register() \n");
    goto out_free_bus;
  }

  return 0;

out_free_bus:
  printk("%s: Failed\n",__func__);
  mv88e6350R_free_mdio_bus(new_bus);

  return ret;
}

static void switchVlanInit(MV_U32 switchCpuPort0,
                           MV_U32 switchCpuPort1,
                           MV_U32 switchMaxPortsNum,
                           MV_U32 switchEnabledPortsMask)
{
  MV_U32 prt;
  MV_U16 reg;

  MV_U16 cpu_vid = 0x1;

  /* Setting  Port default priority for all ports to zero, set default VID=0x1 */
  for(prt=0; prt < switchMaxPortsNum; prt++) {
    if (((1 << prt) & switchEnabledPortsMask)) {
      reg = REG_READ(MV_REG_PORT(prt), MV_SWITCH_PORT_VID_REG);
      reg &= ~0xefff;
      reg |= cpu_vid;
      REG_WRITE(MV_REG_PORT(prt), MV_SWITCH_PORT_VID_REG, reg);
    }
  }
	
  /* 
   *  set Ports VLAN Mapping.
   */
  
  /* port 0 is mapped to port 5 (CPU Port) */  
  reg = REG_READ(MV_REG_PORT(CAL_ICS_EXT_PORT_0), MV_SWITCH_PORT_VMAP_REG);  
  reg &= ~0x00ff;
  reg |= 0x20;
  REG_WRITE(MV_REG_PORT(CAL_ICS_EXT_PORT_0), MV_SWITCH_PORT_VMAP_REG, reg); 
  printk("VLAN map for port %d = 0x%08X\n", CAL_ICS_EXT_PORT_0, reg);

  /* port 1 is mapped to port 6 (CPU Port) */
  reg = REG_READ(MV_REG_PORT(CAL_ICS_EXT_PORT_1), MV_SWITCH_PORT_VMAP_REG);  
  reg &= ~0x00ff;
  reg |= 0x40;
  REG_WRITE(MV_REG_PORT(CAL_ICS_EXT_PORT_1), MV_SWITCH_PORT_VMAP_REG, reg);
  printk("VLAN map for port %d = 0x%08X\n", CAL_ICS_EXT_PORT_0, reg);

  /* port 5 (CPU Port) is mapped to port 0*/
  reg = REG_READ(MV_REG_PORT(switchCpuPort0), MV_SWITCH_PORT_VMAP_REG);
  reg &= ~0x00ff;
  reg |= 0x01;
  REG_WRITE(MV_REG_PORT(switchCpuPort0), MV_SWITCH_PORT_VMAP_REG, reg);
  printk("VLAN map for CPU port %d = 0x%08X\n", switchCpuPort0, reg);

  /* port 6 (CPU Port) is mapped to port 1*/
  reg = REG_READ(MV_REG_PORT(switchCpuPort1), MV_SWITCH_PORT_VMAP_REG);
  reg &= ~0x00ff;
  reg |= 0x02;
  REG_WRITE(MV_REG_PORT(switchCpuPort1), MV_SWITCH_PORT_VMAP_REG, reg);
  printk("VLAN map for CPU port %d = 0x%08X\n", switchCpuPort1, reg);

  /* enable only appropriate ports to forwarding mode */
  for(prt=0; prt < switchMaxPortsNum; prt++) {
    if ((1 << prt)& switchEnabledPortsMask) {
      reg = REG_READ(MV_REG_PORT(prt), MV_SWITCH_PORT_CONTROL_REG);
      reg |= 0x3;
      REG_WRITE(MV_REG_PORT(prt), MV_SWITCH_PORT_CONTROL_REG, reg);
    }
  }
}

static void mv88e6350R_hard_reset(void)
{ 
  unsigned long reg;
  //set the MDIO clock divisor
  *(volatile unsigned int *)(LABX_MDIO_ETH_BASEADDR + MAC_MDIO_CONFIG_REG) = (LABX_ETHERNET_MDIO_DIV & MDIO_DIVISOR_MASK) | MDIO_ENABLED;
  
  /* read the GPIO_TRI register*/
  reg = *((unsigned long *)(XPAR_XPS_GPIO_0_BASEADDR + 0x04));  
  /* set bit 20 as output */
  reg &= ~0x4;
  *((unsigned long *)(XPAR_XPS_GPIO_0_BASEADDR + 0x04)) = reg;
  
  /* read the GPIO_Data register*/
  reg = *((unsigned long *)(XPAR_XPS_GPIO_0_BASEADDR));  
  /* set bit 20 as 1, so PHY_RESET_n=0, reset phy */
  reg |= 0x4;
  *((unsigned long *)(XPAR_XPS_GPIO_0_BASEADDR)) = reg;
  /* delay 10 ms */
  msleep(10);
  
  reg = *((unsigned long *)(XPAR_XPS_GPIO_0_BASEADDR)); 
  /* set bit 20 as 0, so PHY_RESET_n=1, free up PHY_Reset_n */
  reg &= ~0x4;
  *((unsigned long *)(XPAR_XPS_GPIO_0_BASEADDR)) = reg;
}

/*
 * Character device hook functions
 */

static int mvEthSwitch_open(struct inode *inode, struct file *filp)
{
  struct mvEthSwitch *mvEthSwitch;
  unsigned long flags;
  int returnValue = 0;

  mvEthSwitch = container_of(inode->i_cdev, struct mvEthSwitch, cdev);
  filp->private_data = mvEthSwitch;

  /* Lock the mutex and ensure there is only one owner */
  preempt_disable();
  spin_lock_irqsave(&mvEthSwitch->mutex, flags);
  if(mvEthSwitch->opened) {
    returnValue = -1;
  } else {
    mvEthSwitch->opened = true;
  }

  spin_unlock_irqrestore(&mvEthSwitch->mutex, flags);
  preempt_enable();
  
  return(returnValue);
}

static int mvEthSwitch_release(struct inode *inode, struct file *filp)
{
  struct mvEthSwitch *mvEthSwitch = (struct mvEthSwitch*)filp->private_data;
  unsigned long flags;

  preempt_disable();
  spin_lock_irqsave(&mvEthSwitch->mutex, flags);
  mvEthSwitch->opened = false;

  spin_unlock_irqrestore(&mvEthSwitch->mutex, flags);
  preempt_enable();
  return(0);
}

/* I/O control operations for the driver */
static int mvEthSwitch_ioctl(struct inode *inode, 
                                   struct file *filp,
                                   unsigned int command, 
                                   unsigned long arg) {
  int returnValue = 0;
  uint32_t Value = 0xFF;
  uint32_t port = 0xFF;
  struct mvEthSwitch *mvEthSwitch = (struct mvEthSwitch*)filp->private_data;

  /* The least significant byte of arg is the Value (sample memory address, or sample offset value)
     The second to the least significant byte of arg is the register address of output channel select
  */
  switch(command) {
      
  case IOC_MV_MDIO_REG_READ:
    {
      struct mv_mdio mdio;
      if(copy_from_user(&mdio, (void __user*)arg, sizeof(struct mv_mdio)) != 0) {
        return(-EFAULT);
      }

      printk("Reading addr %d, reg %d, page %d\n", mdio.addr, mdio.reg, mdio.page);

      if (mdio.addr < 0x06) { // PHY registers must be accessed indirectly
        if (mdio.page != MV_MDIO_NO_PAGE) {
          REG_PHY_WRITE(mdio.addr, 0x16, mdio.page);
          printk("REG PAGE addr %d = %d\n", mdio.addr, REG_PHY_READ(mdio.addr, 0x16));
        }
        mdio.data = REG_PHY_READ(mdio.addr, mdio.reg);
      } else {
        mdio.data = REG_READ(mdio.addr, mdio.reg);
      }
      
      printk("REG addr %d reg %d = %04X\n", mdio.addr, mdio.reg, mdio.data);

      if(copy_to_user((void __user*)arg, &mdio, sizeof(struct mv_mdio)) != 0) {
        return(-EFAULT);
      }
    }
    break;

  case IOC_MV_MDIO_REG_WRITE:
    {
      struct mv_mdio mdio;
      if(copy_from_user(&mdio, (void __user*)arg, sizeof(struct mv_mdio)) != 0) {
        return(-EFAULT);
      }
      printk("Writing addr %d, reg %d, page %d, value %d\n", mdio.addr, mdio.reg, mdio.page, mdio.data);

      if (mdio.addr < 0x06) { // PHY registers must be accessed indirectly
        if (mdio.page != MV_MDIO_NO_PAGE) {
          REG_PHY_WRITE(mdio.addr, 0x16, mdio.page);
          printk("REG PAGE addr %d = %d\n", mdio.addr, REG_PHY_READ(mdio.addr, 0x16));
        }
        printk("REG addr %d reg %d = %04X\n", mdio.addr, mdio.reg, REG_PHY_READ(mdio.addr, mdio.reg));
        REG_PHY_WRITE(mdio.addr, mdio.reg, mdio.data);
        printk("REG addr %d reg %d = %04X after write\n", mdio.addr, mdio.reg, REG_PHY_READ(mdio.addr, mdio.reg));
      } else {
        REG_WRITE(mdio.addr, mdio.reg, mdio.data);
      }
    }
    break;

  case IOC_MV_MDIO_REG_MODIFY:
    {
      struct mv_mdio_rmw mdio;
      if(copy_from_user(&mdio, (void __user*)arg, sizeof(struct mv_mdio_rmw)) != 0) {
        return(-EFAULT);
      }
      printk("Modifying addr %d, reg %d, page %d, ormask 0x%X, andmask 0x%X\n", mdio.addr, mdio.reg, mdio.page, mdio.ormask, mdio.andmask);

      if (mdio.addr < 0x06) { // PHY registers must be accessed indirectly
        if (mdio.page != MV_MDIO_NO_PAGE) {
          REG_PHY_WRITE(mdio.addr, 0x16, mdio.page);
          printk("REG PAGE addr %d = %d\n", mdio.addr, REG_PHY_READ(mdio.addr, 0x16));
        }
        printk("REG addr %d reg %d = %04X\n", mdio.addr, mdio.reg, REG_PHY_READ(mdio.addr, mdio.reg));
        REG_PHY_WRITE(mdio.addr, mdio.reg, (REG_PHY_READ(mdio.addr, mdio.reg) & mdio.andmask) | mdio.ormask);
        printk("REG addr %d reg %d = %04X after modify\n", mdio.addr, mdio.reg, REG_PHY_READ(mdio.addr, mdio.reg));
      } else {
        REG_WRITE(mdio.addr, mdio.reg, (REG_READ(mdio.addr, mdio.reg) & mdio.andmask) | mdio.ormask);
      }
    }
    break;

  default:
    returnValue = -EINVAL;
    break;
  }

  /* Return an error code appropriate to the command */
  return(returnValue);
}

/* Character device file operations structure */
static struct file_operations mvEthSwitch_fops = {
  .open	   = mvEthSwitch_open,
  .release = mvEthSwitch_release,
  .ioctl   = mvEthSwitch_ioctl,
  .owner   = THIS_MODULE,
};

static int mv88e6350R_probe(struct platform_device *pdev)
{
  MV_U32 portIndex;
  MV_U16 reg, saved_g1reg4;
  struct mvEthSwitch *mvEthSwitch;
	
  mv88e6350R_hard_reset();
  msleep(2);
 
  REG_WRITE(MV_REG_PORT(CAL_ICS_CPU_PORT_0), 
            PHYS_CTRL_REG,
            CAL_ICS_CPU_PORT_0_PHYS_CTRL);

  REG_WRITE(MV_REG_PORT(CAL_ICS_CPU_PORT_1), 
            PHYS_CTRL_REG,
            CAL_ICS_CPU_PORT_1_PHYS_CTRL);

  /* Init vlan LAN0-3 <-> CPU port egiga0 */
  printk("CPU port is on 88E6350R port %d and %d\n", CAL_ICS_CPU_PORT_0, CAL_ICS_CPU_PORT_1);

  switchVlanInit(CAL_ICS_CPU_PORT_0,
                 CAL_ICS_CPU_PORT_1,
                 MV_E6350R_MAX_PORTS_NUM,
                 CAL_ICS_ENABLED_PORTS);

  /* Disable PPU */
  saved_g1reg4 = REG_READ(MV_REG_GLOBAL,0x4);
  REG_WRITE(MV_REG_GLOBAL,0x4,0x0);

  for(portIndex = 0; portIndex < MV_E6350R_MAX_PORTS_NUM; portIndex++) {
    /* Reset PHYs for all but the port to the CPU */
    if((portIndex != CAL_ICS_CPU_PORT_0) && (portIndex != CAL_ICS_CPU_PORT_1)) {
      REG_WRITE(portIndex, 0, 0x9140);
    }
  }

  /* Initialize LED control registers
   *
   * Specifically, Titanium-411 is designed to display per-port status on the
   * front panel using LED columns COL_0 and COL_1:
   *
   * Green -   1 Gbit Link Up
   * Amber - 100 Mbit Link Up
   * Red   -  10 Mbit Link Up
   *
   * Regardless of color, activity is indicated by blinking.
   *
   * On the back panel, LED columns COL_2 and COL_3 reflect status via light
   * pipes co-located above each port, with the exception of switch port 4,
   * which is denoted as "Port 5 - SFP Port" on the enclosure.  The SFP port
   * has a vertical two-LED stack reflecting the same:
   *
   * Green - 1 Gbit Link Up
   * Red   - Any Link Up / Activity (blink)
   *
   * This functionality is enabled through the use of the LED control 
   * registers within the 88E6350R switch chip.
   */
  for(portIndex = 0; portIndex < CAL_ICS_COPPER_PORTS; portIndex++) {
    REG_WRITE(MV_REG_PORT(portIndex),
              MV_SWITCH_PORT_LED_CTRL_REG,
              MV_SWITCH_LED_WRITE(MV_SWITCH_LED_01_CTRL_REG, 
                                  (MV_SWITCH_LED1_100M_10M_ACT | MV_SWITCH_LED0_1G_100M_ACT)));

    REG_WRITE(MV_REG_PORT(portIndex),
              MV_SWITCH_PORT_LED_CTRL_REG,
              MV_SWITCH_LED_WRITE(MV_SWITCH_LED_23_CTRL_REG, 
                                  (MV_SWITCH_LED3_LINK_ACT | MV_SWITCH_LED2_1G_LINK)));

    REG_WRITE(MV_REG_PORT(portIndex),
              MV_SWITCH_PORT_LED_CTRL_REG,
              MV_SWITCH_LED_WRITE(MV_SWITCH_LED_SPECIAL_CTRL_REG,
                                  MV_SWITCH_LED_SPECIAL_NONE));
  }

  /* Enable PHY Polling Unit (PPU) */
  saved_g1reg4 |= 0x4000;
  REG_WRITE(MV_REG_GLOBAL,0x4,saved_g1reg4);
	
  /* Create and populate a device structure */
  mvEthSwitch = (struct mvEthSwitch*) kmalloc(sizeof(struct mvEthSwitch), GFP_KERNEL);
  if(!mvEthSwitch) return(-ENOMEM);
  
  snprintf(mvEthSwitch->name, NAME_MAX_SIZE, "%s", "mvEthSwitch");
  mvEthSwitch->name[NAME_MAX_SIZE - 1] = '\0';
  
  /* Initialize other resources */
  spin_lock_init(&mvEthSwitch->mutex);
  mvEthSwitch->opened = false;

  /* Provide navigation between the device structures */
  platform_set_drvdata(pdev, mvEthSwitch);
  mvEthSwitch->pdev = pdev;
  
  /* Add as a character device to make the instance available for use */
  cdev_init(&mvEthSwitch->cdev, &mvEthSwitch_fops);
  mvEthSwitch->cdev.owner = THIS_MODULE;
  
  mvEthSwitch->instanceNumber = instanceCount++;
  kobject_set_name(&mvEthSwitch->cdev.kobj, "%s.%d", mvEthSwitch->name, mvEthSwitch->instanceNumber);
  cdev_add(&mvEthSwitch->cdev, MKDEV(DRIVER_MAJOR, mvEthSwitch->instanceNumber), 1);
	
  (void)phy_driver_register(&phy_driver);
  (void)mv88e6350R_mdio_bus_init(&mvEthSwitch->pdev->dev);
  return 0;
}

static struct platform_driver mv88e6350R_switch_driver = {
  .probe = mv88e6350R_probe,
  .driver = {
    .name = DRIVER_NAME,
  }
};

static int __init mv88e6350R_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": Marvell 88E6350R Ethernet Switch driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Meyer Sound Laboratories Inc\n");
  
  instanceCount = 0;
  if((returnValue = platform_driver_register(&mv88e6350R_switch_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }


  if((returnValue = register_chrdev_region(MKDEV(DRIVER_MAJOR, 0),MAX_INSTANCES, DRIVER_NAME)) < 0) { 
    printk(KERN_INFO DRIVER_NAME "Failed to allocate character device range\n");
  }
  
  return 0;
}
module_init(mv88e6350R_init);

static void __exit mv88e6350R_cleanup(void)
{
  phy_driver_unregister(&phy_driver);
  unregister_chrdev_region(MKDEV(DRIVER_MAJOR, 0),MAX_INSTANCES);
  platform_driver_unregister(&mv88e6350R_switch_driver);
}
module_exit(mv88e6350R_cleanup);
