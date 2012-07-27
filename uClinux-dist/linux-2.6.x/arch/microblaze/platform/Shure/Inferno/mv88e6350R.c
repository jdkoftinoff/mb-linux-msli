/*
 *  linux/arch/microblaze/platform/Shure/Inferno/mv88e6350R.c
 *
 *  Marvell 88E6350R ethernet switch driver
 *
 *  Written by Yi Cao (yi.cao@labxtechnologies.com)
 *
 *  Copyright (C) 2012 Shure Inc, All Rights Reserved.
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

#include "mv88e6350R.h"
#include "mv88e6350R_defs.h"

/* Driver name */
#define DRIVER_NAME "mv88e6350R"

/* Major device number for the driver */
#define DRIVER_MAJOR 255

/* Maximum number of inferno_mvEthSwitch and instance count */
#define MAX_INSTANCES 2
static uint32_t instanceCount;
#define NAME_MAX_SIZE    (256)

/* Pointer to instance of our switch struct. */
static struct phy_device* internalPhy = NULL;


/* 88E6350R ports assigned to the CPU and HMI ports */
#define SHURE_INFERNO_EXT_PORT_0 (0)
#define SHURE_INFERNO_EXT_PORT_1 (1)
#define SHURE_INFERNO_CPU_PORT (5)
#define SHURE_INFERNO_HMI_PORT (6) // Host Management Interface

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
#define SHURE_INFERNO_CPU_PORT_PHYS_CTRL (RGMII_MODE_RXCLK_DELAY  | \
                                          RGMII_MODE_GTXCLK_DELAY | \
                                          FORCE_LINK_UP           | \
                                          FORCE_DUPLEX_FULL       | \
                                          FORCE_SPEED_1000)

                                         
/* Register settings assigned to the SFP port:
 * Link forced up, 1 Gbps full-duplex
 * Using RGMII delay on switch IND input data
 * Using RGMII delay on switch OUTD output data
 */
#define SHURE_INFERNO_HMI_PORT_PHYS_CTRL (RGMII_MODE_RXCLK_DELAY  | \
                                          RGMII_MODE_GTXCLK_DELAY | \
                                          FORCE_LINK_UP           | \
                                          FORCE_DUPLEX_FULL       | \
                                          FORCE_SPEED_1000)

                                          
/* Bit-mask of enabled ports  */
#define SHURE_INFERNO_ENABLED_PORTS ((1 << 6) | (1 << 5) | (1 << 1) | (1 << 0))

/* Number of copper 1000Base-TX ports for SHURE_INFERNO */
#define SHURE_INFERNO_COPPER_PORTS (2)


typedef char           MV_8;
typedef unsigned char  MV_U8;

typedef int            MV_32;
typedef unsigned int   MV_U32;

typedef short          MV_16;
typedef unsigned short MV_U16;

/* MDIO defines */
#define XPAR_XPS_GPIO_0_BASEADDR 0x820F0000
#define LABX_MDIO_ETH_BASEADDR 0x82050000
#define MDIO_CONTROL_REG      (0x00000000)

#define LABX_MAC_REGS_BASE    (0x00001000)
#define MAC_MDIO_CONFIG_REG   (LABX_MAC_REGS_BASE + 0x0014)
#define LABX_ETHERNET_MDIO_DIV  (0x28)
#  define MDIO_DIVISOR_MASK  (0x0000003F)
#  define MDIO_ENABLED       (0x00000040)

/* Per-port switch registers */
#define MV_SWITCH_PORT_CONTROL_REG 0x04
#define MV_SWITCH_PORT_VMAP_REG    0x06
#define MV_SWITCH_PORT_VID_REG     0x07

/* Port LED control register and indirect registers */
#define MV_SWITCH_PORT_LED_CTRL_REG        0x16
#  define MV_SWITCH_LED_CTRL_UPDATE      0x8000
#  define MV_SWITCH_LED_CTRL_PTR_MASK       0x7
#  define MV_SWITCH_LED_CTRL_PTR_SHIFT       12
#    define MV_SWITCH_LED_23_CTRL_REG       0x1
#      define MV_SWITCH_LED23_OFF          0xEE

#    define MV_SWITCH_LED_RATE_CTRL_REG     0x6
#    define MV_SWITCH_LED_SPECIAL_CTRL_REG  0x7
#      define MV_SWITCH_LED_SPECIAL_NONE    0x0
#  define MV_SWITCH_LED_CTRL_DATA_MASK    0x3FF

/* Macros for reading/writing the individual LED
 * control registers. */

/* Puts together a read/write for an LED control register.
 * Reads/writes are indirect, hence the need for a whichReg
 * parameter. */
#define MV_SWITCH_LED_REGVAL(whichReg, regValue)                                      \
        (((whichReg & MV_SWITCH_LED_CTRL_PTR_MASK) << MV_SWITCH_LED_CTRL_PTR_SHIFT) | \
        (regValue & MV_SWITCH_LED_CTRL_DATA_MASK))

/* Performs a write to an LED control register. */
#define MV_SWITCH_LED_WRREGVAL(whichReg, regValue) \
        (MV_SWITCH_LED_CTRL_UPDATE | MV_SWITCH_LED_REGVAL(whichReg, regValue))

/* Write this before performing a read of an LED
   control register -- it sets up a write. */
#define MV_SWITCH_LED_RDREGVAL(whichReg) \
        MV_SWITCH_LED_REGVAL(whichReg, 0)

/* E6350R-related */
#define MV_E6350R_MAX_PORTS_NUM 7

#define XPAR_XPS_GPIO_0_BASEADDR 0x820F0000

#define LABX_MAC_REGS_BASE    (0x00001000)
#define MAC_MDIO_CONFIG_REG   (LABX_MAC_REGS_BASE + 0x0014)
#define LABX_ETHERNET_MDIO_DIV  (0x28)
#  define MDIO_DIVISOR_MASK  (0x0000003F)
#  define MDIO_ENABLED       (0x00000040)

/* Performs a register write to a switch register */
void REG_WRITE(struct phy_device *phydev, int phy_addr, int reg_addr, int phy_data)
{
  mdiobus_write(phydev->bus, phy_addr, reg_addr, phy_data);
}

/* Performs a register read from a switch register */
unsigned int REG_READ(struct phy_device *phydev, int phy_addr, int reg_addr)
{
  return mdiobus_read(phydev->bus, phy_addr, reg_addr);
}

/* Performs a register write to a switch PHY register */
void REG_PHY_WRITE(struct phy_device *phydev, int phy_addr, int reg_addr, int phy_data)
{
  REG_WRITE(phydev, MV_REG_GLOBAL2, 25, phy_data);
  REG_WRITE(phydev, MV_REG_GLOBAL2, 24, 0x9400 | (phy_addr << 5) | reg_addr);
  while (REG_READ(phydev, MV_REG_GLOBAL2, 24) & 0x8000); // Should only take a read or two...
}

/* Performs a register read from a switch PHY register */
unsigned int REG_PHY_READ(struct phy_device *phydev, int phy_addr, int reg_addr)
{
  REG_WRITE(phydev, MV_REG_GLOBAL2, 24, 0x9800 | (phy_addr << 5) | reg_addr);
  while (REG_READ(phydev, MV_REG_GLOBAL2, 24) & 0x8000); // Should only take a read or two...
  return REG_READ(phydev, MV_REG_GLOBAL2, 25);
}

static int marvell_config_aneg(struct phy_device *phydev)
{
  return 0;
}

static int marvell_read_status(struct phy_device *phydev)
{
  struct phy_device *phydev_switch = to_phy_device(phydev->bus->parent);

  /* External PHY routed to the corresponding internal port */
  unsigned int portStatusReg = REG_READ(phydev_switch, MV_REG_PORT(phydev->addr), 0x00);
  /* Internal MAC hooked up to the FPGA */
  unsigned int internalPort = SHURE_INFERNO_CPU_PORT;
  unsigned int physicalControlReg = REG_READ(phydev_switch, MV_REG_PORT(internalPort), 0x01);
  unsigned int phySpeed = (portStatusReg >> 8) & 3;

  /* We are in polling mode, and get called periodically.
     Don't do anything unless there was actually a link
     status change. */
  if (phySpeed != (physicalControlReg & 3)) {
    /* Adjust the CPU port speed to match the external PHY speed */
    printk("Port Status for PHY address %u: %04X\n", phydev->addr, (uint16_t)portStatusReg);
    REG_WRITE(phydev_switch, MV_REG_PORT(internalPort), 0x01, (physicalControlReg & ~0x0023) | 0x0010 | phySpeed); /* Link down, new speed */
    REG_WRITE(phydev_switch, MV_REG_PORT(internalPort), 0x01, (physicalControlReg & ~0x0003) | 0x0030 | phySpeed); /* Link up */
  }

  switch(phySpeed) {
    case 0: phydev->speed = SPEED_10; break;
    case 1: phydev->speed = SPEED_100; break;
    case 2: phydev->speed = SPEED_1000; break;
    default: printk("88E6350R unknown speed\n"); break;
  }

  phydev->duplex = (portStatusReg >> 10) ? DUPLEX_FULL : DUPLEX_HALF;
  phydev->link = (portStatusReg >> 11) & 1;

  return 0;
}

/* Probe function for the phy_device structure that
 * will represent the PHY within the switch (as opposed
 * to the switch itself). We need to keep a reference
 * to that structure around so that we can change it
 * when necessary, such as when re-mapping which PHY
 * within the switch is configured to talk to the FPGA,
 * whose address Linux needs.
 *
 * See the notes in mvEthSwitch_write() when the
 * ports are remapped. */
static int marvell_probe(struct phy_device *phydev)
{
  /* We are being probed and we receive the phy_device
   * that will represent the PHY within the switch that
   * we are currently communicating over. We want to
   * keep it around since we will need it when performing
   * switch operations. See comments on variable internalPhy
   * within struct mvEthSwitch. */
  static int probed = 0;

  printk("mv88e6350R PHY probe: phydev = %p.\n", phydev);

  /* We are called twice, for some reason. Only
     the first time does the call contain the
     pointer to the correct PHY device. */
  if(!probed) {
    internalPhy = phydev;
    probed = 1;
  }

  /* Probed successfully. */
  return 0;
}

static struct phy_driver phy_driver = {
	.phy_id = 0x01410e70,
	.phy_id_mask = 0xfffffff0,
	.name = "Marvell 88E6350R PHY",
	.features = PHY_GBIT_FEATURES,
	.flags = 0,
        .probe = &marvell_probe,
	.config_aneg = &marvell_config_aneg,
	.read_status = &marvell_read_status,
	.driver = { .owner = THIS_MODULE },
};

static void mv88e6350R_free_mdio_bus(struct mii_bus *bus)
{
}

int mv88e6350R_mdio_read(struct mii_bus *bus, int phy_id, int regnum)
{
  return REG_PHY_READ(to_phy_device(bus->parent), phy_id, regnum);
}

int mv88e6350R_mdio_write(struct mii_bus *bus, int phy_id, int regnum, u16 val)
{
  REG_PHY_WRITE(to_phy_device(bus->parent), phy_id, regnum, val);
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

static void switchVlanMapPortPair(struct phy_device *phydev, int port1, int port2, MV_U16 fid) {
  MV_U16 reg;

  /* First port is mapped to the second port. */
  reg = REG_READ(phydev, MV_REG_PORT(port1), MV_SWITCH_PORT_VMAP_REG);  
  reg &= ~0xf0ff;
  reg |= (0x1 << port2) | fid;
  REG_WRITE(phydev, MV_REG_PORT(port1), MV_SWITCH_PORT_VMAP_REG, reg); 
  printk("VLAN map for port %d = 0x%08X\n", port1, reg);

  /* Second port is mapped to the first port. */
  reg = REG_READ(phydev, MV_REG_PORT(port2), MV_SWITCH_PORT_VMAP_REG);
  reg &= ~0xf0ff;
  reg |= (0x1 << port1) | fid;
  REG_WRITE(phydev, MV_REG_PORT(port2), MV_SWITCH_PORT_VMAP_REG, reg);
  printk("VLAN map for port %d = 0x%08X\n", port2, reg);

  /* Enable the two ports for forwarding and disable VLAN tunneling. */
  reg = REG_READ(phydev, MV_REG_PORT(port1), MV_SWITCH_PORT_CONTROL_REG);
  reg |= 0x0003;  /* Port State = Forwarding */
  reg &= ~0x0080; /* VLAN Tunnel = Disabled */
  REG_WRITE(phydev, MV_REG_PORT(port1), MV_SWITCH_PORT_CONTROL_REG, reg);

  reg = REG_READ(phydev, MV_REG_PORT(port2), MV_SWITCH_PORT_CONTROL_REG);
  reg |= 0x0003;  /* Port State = Forwarding */
  reg &= ~0x0080; /* VLAN Tunnel = Disabled */
  REG_WRITE(phydev, MV_REG_PORT(port2), MV_SWITCH_PORT_CONTROL_REG, reg);

  /* All other port mappings for these
     two ports are effectively cleared. */
}

static void switchVlanInit(struct phy_device *phydev,
                           MV_U32 switchCpuPort,
                           MV_U32 switchHmiPort,
                           MV_U32 switchMaxPortsNum,
                           MV_U32 switchEnabledPortsMask)
{
  MV_U32 prt;
  MV_U16 reg;

  MV_U16 cpu_vid = 0x1;

  /* Setting port default priority for all ports to zero, set default VID=0x1 */
  for(prt=0; prt < switchMaxPortsNum; prt++) {
    if (((1 << prt) & switchEnabledPortsMask)) {
      reg = REG_READ(phydev, MV_REG_PORT(prt), MV_SWITCH_PORT_VID_REG);
      reg &= ~0xefff;
      reg |= cpu_vid;
      REG_WRITE(phydev, MV_REG_PORT(prt), MV_SWITCH_PORT_VID_REG, reg);
    }
  }
	
  /* Set Ports VLAN Mapping. */

  /* Ports 0 and 5 are mapped bidirectionally. */
  switchVlanMapPortPair(phydev, SHURE_INFERNO_EXT_PORT_0, switchCpuPort, 0x0000);
  
  /* Ports 1 and 6 are mapped bidirectionally. */
  switchVlanMapPortPair(phydev, SHURE_INFERNO_EXT_PORT_1, switchHmiPort, 0x1000);
  
  /* enable only appropriate ports to forwarding mode and disable VLAN tunneling */
  for(prt=0; prt < switchMaxPortsNum; prt++) {
    if ((1 << prt)& switchEnabledPortsMask) {
      reg = REG_READ(phydev, MV_REG_PORT(prt), MV_SWITCH_PORT_CONTROL_REG);
      reg |= 0x0003;  /* Port State = Forwarding */
      reg &= ~0x0080; /* VLAN Tunnel = Disabled */
      REG_WRITE(phydev, MV_REG_PORT(prt), MV_SWITCH_PORT_CONTROL_REG, reg);
    }
  }
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

static ssize_t mvEthSwitch_write(struct file *filp, const char __user* buf, size_t len, loff_t* offset) {
  struct phy_device *phydev = ((struct mvEthSwitch*)filp->private_data)->pdev;
  unsigned int port1;
  unsigned int port2;
  unsigned int fid = 0;
  unsigned int phyAddr;
  char cmd[4];

  /* Write a single two-digit string to this driver
     to map together the two ports specified in the
     string. Append 'r#' to reset the switch/PHYs
     and tell Linux to use the PHY whose address is
     #. The reset should be done with the first of
     any port mappings. */

  if(len >= 2) {
    copy_from_user(cmd, buf, 4);
    port1 = cmd[0] - '0';
    port2 = cmd[1] - '0';

    if(port1 < 10 && port2 < 10) {
      /* Ports 5 is the CPU port and port 6 is the HMI
         port. One will be mapped to one PHY (one of ports 0-4),
         one to another. The pair that maps one of them to
         some PHY port needs to have an FID field of 0x1
         written to the VLAN Map register in order to truly
         isolate it from the other pair. Choose the HMI
         port pair to have that FID. */
      if(port1 == 6 || port2 == 6) fid = 0x1000;
      switchVlanMapPortPair(phydev, port1, port2, fid);
    }
  }

  if(len >= 4 && cmd[2] == 'r') {
    /* Reset VTU and ATU. */
    printk("Resetting VTU.\n");
    REG_WRITE(phydev, MV_REG_GLOBAL, 0x05, 0x9000);
    printk("Resetting ATU.\n");
    REG_WRITE(phydev, MV_REG_GLOBAL, 0x0B, 0x9000);

    /* Tell Linux the PHY address of the
       PHY that it will be connected to now. */
    phyAddr = cmd[3] - '0';
    if(phyAddr < 10) {
      printk("Linux pointed to PHY address %u.\n", phyAddr);
      internalPhy->addr = phyAddr;
    }
  }

  if(len >= 2) {
    return len;
  } else {
    return -EINVAL;
  }
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
          REG_PHY_WRITE(mvEthSwitch->pdev, mdio.addr, 0x16, mdio.page);
          printk("REG PAGE addr %d = %d\n", mdio.addr, REG_PHY_READ(mvEthSwitch->pdev, mdio.addr, 0x16));
        }
        mdio.data = REG_PHY_READ(mvEthSwitch->pdev, mdio.addr, mdio.reg);
      } else {
        mdio.data = REG_READ(mvEthSwitch->pdev, mdio.addr, mdio.reg);
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
          REG_PHY_WRITE(mvEthSwitch->pdev, mdio.addr, 0x16, mdio.page);
          printk("REG PAGE addr %d = %d\n", mdio.addr, REG_PHY_READ(mvEthSwitch->pdev, mdio.addr, 0x16));
        }
        printk("REG addr %d reg %d = %04X\n", mdio.addr, mdio.reg, REG_PHY_READ(mvEthSwitch->pdev, mdio.addr, mdio.reg));
        REG_PHY_WRITE(mvEthSwitch->pdev, mdio.addr, mdio.reg, mdio.data);
        printk("REG addr %d reg %d = %04X after write\n", mdio.addr, mdio.reg, REG_PHY_READ(mvEthSwitch->pdev, mdio.addr, mdio.reg));
      } else {
        REG_WRITE(mvEthSwitch->pdev, mdio.addr, mdio.reg, mdio.data);
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
          REG_PHY_WRITE(mvEthSwitch->pdev, mdio.addr, 0x16, mdio.page);
          printk("REG PAGE addr %d = %d\n", mdio.addr, REG_PHY_READ(mvEthSwitch->pdev, mdio.addr, 0x16));
        }
        printk("REG addr %d reg %d = %04X\n", mdio.addr, mdio.reg, REG_PHY_READ(mvEthSwitch->pdev, mdio.addr, mdio.reg));
        REG_PHY_WRITE(mvEthSwitch->pdev, mdio.addr, mdio.reg, (REG_PHY_READ(mvEthSwitch->pdev, mdio.addr, mdio.reg) & mdio.andmask) | mdio.ormask);
        printk("REG addr %d reg %d = %04X after modify\n", mdio.addr, mdio.reg, REG_PHY_READ(mvEthSwitch->pdev, mdio.addr, mdio.reg));
      } else {
        REG_WRITE(mvEthSwitch->pdev, mdio.addr, mdio.reg, (REG_READ(mvEthSwitch->pdev, mdio.addr, mdio.reg) & mdio.andmask) | mdio.ormask);
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
  .write   = mvEthSwitch_write,
  .release = mvEthSwitch_release,
  .ioctl   = mvEthSwitch_ioctl,
  .owner   = THIS_MODULE,
};

static int mv88e6350R_probe(struct phy_device *pdev)
{
  MV_U32 portIndex;
  MV_U32 controlReg; // Used for writing LED control register value.
  MV_U16 saved_g1reg4;
  struct mvEthSwitch *mvEthSwitch;

  /* Create and populate a device structure */
  mvEthSwitch = (struct mvEthSwitch*) kmalloc(sizeof(struct mvEthSwitch), GFP_KERNEL);
  if(!mvEthSwitch) return(-ENOMEM);
  
  snprintf(mvEthSwitch->name, NAME_MAX_SIZE, "%s", "mvEthSwitch");
  mvEthSwitch->name[NAME_MAX_SIZE - 1] = '\0';
  
  /* Initialize other resources */
  spin_lock_init(&mvEthSwitch->mutex);
  mvEthSwitch->opened = false;

  /* Provide navigation between the device structures */
  pdev->priv = mvEthSwitch;
  mvEthSwitch->pdev = pdev;
  
  /* Add as a character device to make the instance available for use */
  cdev_init(&mvEthSwitch->cdev, &mvEthSwitch_fops);
  mvEthSwitch->cdev.owner = THIS_MODULE;
  
  mvEthSwitch->instanceNumber = instanceCount++;
  kobject_set_name(&mvEthSwitch->cdev.kobj, "%s.%d", mvEthSwitch->name, mvEthSwitch->instanceNumber);
  cdev_add(&mvEthSwitch->cdev, MKDEV(DRIVER_MAJOR, mvEthSwitch->instanceNumber), 1);

  REG_WRITE(mvEthSwitch->pdev, MV_REG_PORT(SHURE_INFERNO_CPU_PORT), 
            PHYS_CTRL_REG,
            SHURE_INFERNO_CPU_PORT_PHYS_CTRL);

  REG_WRITE(mvEthSwitch->pdev, MV_REG_PORT(SHURE_INFERNO_HMI_PORT), 
            PHYS_CTRL_REG,
            SHURE_INFERNO_HMI_PORT_PHYS_CTRL);

  /* Init vlan LAN0-3 <-> CPU port egiga0 */
  printk("CPU port is on 88E6350R port %d and HMI port is on 88E6350R port %d\n", SHURE_INFERNO_CPU_PORT, SHURE_INFERNO_HMI_PORT);

  switchVlanInit(mvEthSwitch->pdev,
                 SHURE_INFERNO_CPU_PORT,
                 SHURE_INFERNO_HMI_PORT,
                 MV_E6350R_MAX_PORTS_NUM,
                 SHURE_INFERNO_ENABLED_PORTS);

  /* Disable PPU */
  saved_g1reg4 = REG_READ(mvEthSwitch->pdev, MV_REG_GLOBAL,0x4);
  REG_WRITE(mvEthSwitch->pdev, MV_REG_GLOBAL,0x4,0x0);

  for(portIndex = 0; portIndex < MV_E6350R_MAX_PORTS_NUM; portIndex++) {
    /* Reset PHYs for all but the ports to the CPU and the HMI */
    if((portIndex != SHURE_INFERNO_CPU_PORT) && (portIndex != SHURE_INFERNO_HMI_PORT)) {
      REG_WRITE(mvEthSwitch->pdev, portIndex, 0, 0x9140);
    }
  }

  /*
   * Initialize LED control registers.
   *
   * Default (on power up):
   *   LED 0 = green -- link, activity; LED 1 = yellow -- GBIT.
   *
   * We also want LEDs 2 and 3 turned off so that
   * the strobing isn't visible in the adjacent
   * LEDs 0 and 1.
   */
  for(portIndex = 0; portIndex < SHURE_INFERNO_COPPER_PORTS; portIndex++) {
    controlReg  = MV_SWITCH_LED_WRREGVAL(MV_SWITCH_LED_23_CTRL_REG, MV_SWITCH_LED23_OFF);

    /* Write LED control register - 0xEE */
    REG_WRITE(mvEthSwitch->pdev,           // Device
              MV_REG_PORT(portIndex),      // Port
              MV_SWITCH_PORT_LED_CTRL_REG, // Register being indirectly accessed.
              controlReg);                 // Register value - contains register
                                           // value being indirectly accessed.
  }

  /* Enable PHY Polling Unit (PPU) */
  saved_g1reg4 |= 0x4000;
  REG_WRITE(mvEthSwitch->pdev, MV_REG_GLOBAL,0x4,saved_g1reg4);
	
  (void)mv88e6350R_mdio_bus_init(&mvEthSwitch->pdev->dev);
  return 0;
}

static struct phy_driver switch_phy_driver = {
	.phy_id = 0x00003710,
	.phy_id_mask = 0x0000fff0,
	.name = "Marvell 88E6350R",
	.features = PHY_GBIT_FEATURES,
	.flags = 0,
        .probe = mv88e6350R_probe,
	.driver = { .owner = THIS_MODULE },
};


static int __init mv88e6350R_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": Marvell 88E6350R Ethernet Switch driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Shure Inc\n");
  
  instanceCount = 0;
  if((returnValue = phy_driver_register(&switch_phy_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register switch phy driver\n");
    return(returnValue);
  }

  if((returnValue = register_chrdev_region(MKDEV(DRIVER_MAJOR, 0),MAX_INSTANCES, DRIVER_NAME)) < 0) { 
    printk(KERN_INFO DRIVER_NAME "Failed to allocate character device range\n");
  }
  
  if((returnValue = phy_driver_register(&phy_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register phy driver\n");
    return(returnValue);
  }

  return 0;
}
module_init(mv88e6350R_init);

static void __exit mv88e6350R_cleanup(void)
{
  phy_driver_unregister(&phy_driver);
  unregister_chrdev_region(MKDEV(DRIVER_MAJOR, 0),MAX_INSTANCES);
  phy_driver_unregister(&switch_phy_driver);
}
module_exit(mv88e6350R_cleanup);
