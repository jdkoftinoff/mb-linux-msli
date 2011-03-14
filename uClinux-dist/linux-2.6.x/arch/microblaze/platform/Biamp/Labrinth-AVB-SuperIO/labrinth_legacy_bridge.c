/*
 *  linux/drivers/net/labx_avb/labrinth_legacy_bridge.c
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

#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/mii.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <xio.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF

#include "labrinth_legacy_bridge.h"

/* Driver name and the revision range of hardware expected.
 * This driver will work with revision 1.1 only.
 */
#define DRIVER_NAME "labrinth_legacy_bridge"

/* Major device number for the driver */
#define DRIVER_MAJOR 222

/* Maximum number of legacy_bridges and instance count */
#define MAX_INSTANCES 4
static uint32_t instanceCount;

#if 0
#define DBG(f, x...) printk(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

/* Default and maximum number of MAC match units */
#define DEFAULT_MAC_MATCH_UNITS  (4)
#define MAX_MAC_MATCH_UNITS     (32)

#define MAC_MATCH_NONE 0
#define MAC_MATCH_ALL 1

static const u8 MAC_BROADCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static const u8 MAC_ZERO[6]      = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#define NUM_SRL16E_CONFIG_WORDS 8
#define NUM_SRL16E_INSTANCES    12

/* Special value indicating "no PHY supplied" */
#define NO_PHY_SUPPLIED_TYPE (0xFF)

/* Note: This must be <= MII_BUS_ID_SIZE which is currently 17 (including trailing '\0') */
#define MDIO_OF_BUSNAME_FMT "labxeth%08x"

/* Driver structure to maintain state for each device instance */
#define NAME_MAX_SIZE    (256)

struct legacy_bridge {
  /* Pointer back to the platform device */
  struct platform_device *pdev;

  /* Character device data */
  struct cdev cdev;
  dev_t       deviceNumber;
  uint32_t    instanceNumber;

  /* Network device structure for PHY interaction */
  struct net_device *ndev;

  /* Name for use in identification */
  char name[NAME_MAX_SIZE];

  /* Physical and virtual base address */
  uintptr_t      physicalAddress;
  uintptr_t      addressRangeSize;
  void __iomem  *virtualAddress;

  /* Number of MAC match units the hardware has */
  uint32_t macMatchUnits;

  /* PHY type, address, and name. The PHY name is of the format PHY_ID_FMT.
   * These values are for the PHY connected to this instance.
   */
  uint8_t phy_type;
  uint8_t phy_addr;
  char phy_name[BUS_ID_SIZE];
  struct phy_device *phy_dev;

  /* Mutex for the device instance */
  spinlock_t mutex;
  bool opened;
};

/* Busy loops until the match unit configuration logic is idle.  The hardware goes 
 * idle very quickly and deterministically after a configuration word is written, 
 * so this should not consume very much time at all.
 */
static void wait_match_config(struct legacy_bridge *bridge,
                              uint32_t whichPort) {
  uint32_t statusWord;
  uint32_t timeout = 10000;
  do {
    statusWord = XIo_In32(BRIDGE_PORT_REG_ADDRESS(bridge, whichPort, FILTER_CTRL_STAT_REG));
    if (timeout-- == 0) {
      printk("depacketizer: wait_match_config timeout!\n");
      break;
    }
  } while(statusWord & FILTER_LOAD_ACTIVE);
}

/* Selects a set of match units for subsequent configuration loads */
typedef enum { SELECT_NONE, SELECT_SINGLE, SELECT_ALL } SelectionMode;
static void select_matchers(struct legacy_bridge *bridge,
                            uint32_t whichPort,
                            SelectionMode selectionMode,
                            uint32_t matchUnit) {

  switch(selectionMode) {
  case SELECT_NONE:
    /* De-select all the match units */
    //printk("MAC SELECT %08X\n", 0);
    XIo_Out32(BRIDGE_PORT_REG_ADDRESS(bridge, whichPort, FILTER_SELECT_REG),
              FILTER_SELECT_NONE);
    break;

  case SELECT_SINGLE:
    /* Select a single unit */
    //printk("MAC SELECT %08X\n", 1 << matchUnit);
    XIo_Out32(BRIDGE_PORT_REG_ADDRESS(bridge, whichPort, FILTER_SELECT_REG), (1 << matchUnit));
    break;

  default:
    /* Select all match units at once */
    //printk("MAC SELECT %08X\n", 0xFFFFFFFF);
    XIo_Out32(BRIDGE_PORT_REG_ADDRESS(bridge, whichPort, FILTER_SELECT_REG), 
              FILTER_SELECT_ALL);
    break;
  }
}

/* Sets the loading mode for any selected match units.  This revolves around
 * automatically disabling the match units' outputs while they're being
 * configured so they don't fire false matches, and re-enabling them as their
 * last configuration word is loaded.
 */
typedef enum { LOADING_MORE_WORDS, LOADING_LAST_WORD } LoadingMode;
static void set_matcher_loading_mode(struct legacy_bridge *bridge,
                                     uint32_t whichPort,
                                     LoadingMode loadingMode) {
  uint32_t controlWord = XIo_In32(BRIDGE_PORT_REG_ADDRESS(bridge, whichPort, FILTER_CTRL_STAT_REG));

  if(loadingMode == LOADING_MORE_WORDS) {
    /* Clear the "last word" bit to suppress false matches while the units are
     * only partially cleared out
     */
    controlWord &= ~FILTER_LOAD_LAST;
  } else {
    /* Loading the final word, flag the match unit(s) to enable after the
     * next configuration word is loaded.
     */
    controlWord |= FILTER_LOAD_LAST;
  }
  //printk("CONTROL WORD %08X\n", controlWord);
  XIo_Out32(BRIDGE_PORT_REG_ADDRESS(bridge, whichPort, FILTER_CTRL_STAT_REG), controlWord);
}

/* Clears any selected match units, preventing them from matching any packets */
static void clear_selected_matchers(struct legacy_bridge *bridge,
                                    uint32_t whichPort) {
  uint32_t wordIndex;

  /* Ensure the unit(s) disable as the first word is load to prevent erronous
   * matches as the units become partially-cleared
   */
  set_matcher_loading_mode(bridge, whichPort, LOADING_MORE_WORDS);

  for(wordIndex = 0; wordIndex < NUM_SRL16E_CONFIG_WORDS; wordIndex++) {
    /* Assert the "last word" flag on the last word required to complete the clearing
     * of the selected unit(s).
     */
    if(wordIndex == (NUM_SRL16E_CONFIG_WORDS - 1)) {
      set_matcher_loading_mode(bridge, whichPort, LOADING_LAST_WORD);
    }
    //printk("MAC LOAD %08X\n", 0);
    XIo_Out32(BRIDGE_PORT_REG_ADDRESS(bridge, whichPort, FILTER_LOAD_REG), 
              FILTER_LOAD_CLEAR);
  }
}

/* Loads truth tables into a match unit using the newest, "unified" match
 * architecture.  This is SRL16E based (not cascaded) due to the efficient
 * packing of these primitives into Xilinx LUT6-based architectures.
 */
static void load_unified_matcher(struct legacy_bridge *bridge,
                                 uint32_t whichPort,
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
    if(wordIndex == 0) set_matcher_loading_mode(bridge, whichPort, LOADING_LAST_WORD);
    //printk("MAC LOAD %08X\n", configWord);
    XIo_Out32(BRIDGE_PORT_REG_ADDRESS(bridge, whichPort, FILTER_LOAD_REG), configWord);
    wait_match_config(bridge, whichPort);
  }
}

static void configure_mac_filter(struct legacy_bridge *bridge, 
                                 uint32_t whichPort,
                                 uint32_t unitNum, 
                                 const u8 mac[6], 
                                 uint32_t mode) {
  /* Only allow programming up to the supported number of MAC match units */
  if (unitNum >= bridge->macMatchUnits) return;

  printk("CONFIGURE MAC MATCH %d (%d), %02X:%02X:%02X:%02X:%02X:%02X\n", unitNum, mode,
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  /* Ascertain that the configuration logic is ready, then select the matcher */
  wait_match_config(bridge, whichPort);
  select_matchers(bridge, whichPort, SELECT_SINGLE, unitNum);

  if (mode == MAC_MATCH_NONE) {
    clear_selected_matchers(bridge, whichPort);
  } else {
    /* Set the loading mode to disable as we load the first word */
    set_matcher_loading_mode(bridge, whichPort, LOADING_MORE_WORDS);
    
    /* Calculate matching truth tables for the LUTs and load them */
    load_unified_matcher(bridge, whichPort, mac);
  }
  
  /* De-select the match unit */
  select_matchers(bridge, whichPort, SELECT_NONE, 0);
}

/*
 * Resets the packet bridge to a known state - permitting pass-through
 * of data from the backplane to the AVB network, but permitting no
 * traffic in the other direction.
 */
static void reset_legacy_bridge(struct legacy_bridge *bridge) {
  /* Clear out all of the Rx filter match units for both ports */
  printk("Clearing match units\n");
  select_matchers(bridge, AVB_PORT_0, SELECT_ALL, 0);
  clear_selected_matchers(bridge, AVB_PORT_0);
  select_matchers(bridge, AVB_PORT_0, SELECT_NONE, 0);
  select_matchers(bridge, AVB_PORT_1, SELECT_ALL, 0);
  clear_selected_matchers(bridge, AVB_PORT_1);
  select_matchers(bridge, AVB_PORT_1, SELECT_NONE, 0);

  /* Disable transmission on both ports */
  XIo_Out32(BRIDGE_REG_ADDRESS(bridge, BRIDGE_CTRL_REG), BRIDGE_TX_EN_NONE);

  /* Hit the backplane MAC's Tx and Rx reset registers */
  XIo_Out32(BP_MAC_REG_ADDRESS(bridge, MAC_RX_CONFIG_REG),
            MAC_RX_RESET);
  XIo_Out32(BP_MAC_REG_ADDRESS(bridge, MAC_TX_CONFIG_REG),
            MAC_TX_RESET);
}

/* Configures the bridge ports */
static void legacy_bridge_config_ports(struct legacy_bridge *bridge,
                                       BridgePortsConfig *portsConfig) {
  /* If neither Tx port is enabled, disabled everything by resettting the
   * entire bridge, which also clears all of the Rx MAC filters for both ports
   * in addition to disabling Tx on both ports.
   */
  if((portsConfig->txPortsEnable[0] == TX_PORT_DISABLED) &
     (portsConfig->txPortsEnable[1] == TX_PORT_DISABLED)) {
    reset_legacy_bridge(bridge);
  } else {
    uint32_t bridgeConfigWord = BRIDGE_TX_EN_NONE;

    /* Actively bridging, configure appropriately */
    if(portsConfig->rxPortSelection == RX_PORT_1_SELECT) {
      bridgeConfigWord |= BRIDGE_RX_PORT_1;
    }

    if(portsConfig->txPortsEnable[0] == TX_PORT_ENABLED) {
      bridgeConfigWord |= BRIDGE_TX_EN_PORT_0;
    }

    if(portsConfig->txPortsEnable[1] == TX_PORT_ENABLED) {
      bridgeConfigWord |= BRIDGE_TX_EN_PORT_1;
    }
    XIo_Out32(BRIDGE_REG_ADDRESS(bridge, BRIDGE_CTRL_REG), bridgeConfigWord);
  }
}

/* PHY link speed change callback function */
static void legacy_bridge_adjust_link(struct net_device *dev)
{
  struct legacy_bridge *bridge = netdev_priv(dev);

  if (bridge->phy_dev->link != PHY_DOWN) {
    printk("%s : Link up, %d Mb/s\n", bridge->ndev->name, bridge->phy_dev->speed);
  } else {
    printk("%s : Link down\n", bridge->ndev->name);
  }
}

/*
 * Character device hook functions
 */

static int legacy_bridge_open(struct inode *inode, struct file *filp) {
  struct legacy_bridge *bridge;
  unsigned long flags;
  uint8_t broadcastMac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  int returnValue = 0;

  bridge = container_of(inode->i_cdev, struct legacy_bridge, cdev);
  filp->private_data = bridge;

  /* Lock the mutex and ensure there is only one owner */
  preempt_disable();
  spin_lock_irqsave(&bridge->mutex, flags);
  if(bridge->opened) {
    returnValue = -1;
  } else {
    bridge->opened = true;
  }

  spin_unlock_irqrestore(&bridge->mutex, flags);
  preempt_enable();

  /* Reset the hardware */
  reset_legacy_bridge(bridge);
  
  /* Perform PHY setup using the platform-supplied hook method */
  printk("%s: About to connect to phy device\n",__func__);
  if(bridge->phy_dev == NULL) {
    /* Lookup phy device */
    bridge->phy_dev = phy_connect(bridge->ndev, 
                                  bridge->phy_name, 
                                  &legacy_bridge_adjust_link,
                                  0, 
                                  PHY_INTERFACE_MODE_MII);
    if(!IS_ERR(bridge->phy_dev)) {
      int ret;

      printk("%s: About to call phy_start_aneg()\n",__func__);
      ret = phy_start_aneg(bridge->phy_dev);
      if (0 != ret) {
        printk("%s: phy_start_aneg() Failed with code %d\n",__func__,ret);
      } else {
        printk("%s: phy_start_aneg() Passed\n",__func__);
      }
    } else {
      printk("Not able to find Phy");
      bridge->phy_dev = NULL;
    }
  }

  /* TEMPORARY - Auto-configure for broadcast traffic! */
  //  printk("Auto-configuring for BROADCAST traffic\n");
  //  configure_mac_filter(bridge, 0, 0, broadcastMac, MAC_MATCH_ALL);

  return(returnValue);
}

static int legacy_bridge_release(struct inode *inode, struct file *filp) {
  struct legacy_bridge *bridge = (struct legacy_bridge*) filp->private_data;
  unsigned long flags;

  printk("legacy_bridge_release()!\n");
  
  /* Reset the hardware before releasing it */
  reset_legacy_bridge(bridge);
  
  preempt_disable();
  spin_lock_irqsave(&bridge->mutex, flags);
  bridge->opened = false;

  spin_unlock_irqrestore(&bridge->mutex, flags);
  preempt_enable();
  return(0);
}

/* I/O control operations for the driver */
static int legacy_bridge_ioctl(struct inode *inode, 
                               struct file *filp,
                               unsigned int command, 
                               unsigned long arg) {
  // Switch on the request
  int returnValue = 0;
  struct legacy_bridge *bridge = (struct legacy_bridge*) filp->private_data;

  switch(command) {
  case IOC_CONFIG_MAC_FILTER:
    {
      MacFilterConfig filterConfig;

      if(copy_from_user(&filterConfig, (void __user*)arg, sizeof(MacFilterConfig)) != 0) {
        return(-EFAULT);
      }

      /* Validate the parameters */
      if((filterConfig.whichAvbPort >= NUM_AVB_BRIDGE_PORTS) |
         (filterConfig.whichFilter >= bridge->macMatchUnits)) return(-ENODEV);
      
      /* Configure the filter for the instance with the passed parameters */
      configure_mac_filter(bridge,
                           filterConfig.whichAvbPort,
                           filterConfig.whichFilter,
                           filterConfig.macAddress,
                           (filterConfig.enabled ? MAC_MATCH_ALL : MAC_MATCH_NONE));
    }
    break;

  case IOC_CONFIG_PHY_TEST_MODE:
    {
      uint32_t testMode;
      uint32_t phyTestMode;

      /* Dispatch to the PHY driver */
      if(copy_from_user(&testMode, (void __user*)arg, sizeof(testMode)) != 0) {
        return(-EFAULT);
      }

      switch(testMode) {
      case PHY_TEST_LOOPBACK:
        phyTestMode = PHY_TEST_INT_LOOP;
        break;

      default:
        /* Presume normal mode */
        phyTestMode = PHY_TEST_NONE;
      }

      bridge->phy_dev->drv->set_test_mode(bridge->phy_dev, phyTestMode);
    }
    break;
    
  case IOC_CONFIG_BRIDGE_PORTS:
    {
      BridgePortsConfig portsConfig;
      uint32_t testMode;
      uint32_t phyTestMode;

      if(copy_from_user(&portsConfig, (void __user*)arg, sizeof(portsConfig)) != 0) {
        return(-EFAULT);
      }

      /* Configure the bridge ports */
      legacy_bridge_config_ports(bridge, &portsConfig);
    }
    break;

  default:
    returnValue = -EINVAL;
  }

  /* Return an error code appropriate to the command */
  return(returnValue);
}

/* Character device file operations structure */
static const struct file_operations legacy_bridge_fops = {
  .open	   = legacy_bridge_open,
  .release = legacy_bridge_release,
  .ioctl   = legacy_bridge_ioctl,
  .owner   = THIS_MODULE,
};

/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 * This is exported to allow polymorphic drivers to invoke it.
 * @param name          - Base name of the instance
 * @param pdev          - Platform device structure
 * @param addressRange  - Resource describing the hardware's I/O range
 * @param macMatchUnits - Number of MAC match units the hardware has
 * @param phy_type      - PHY type
 * @param phy_addr      - PHY MDIO address
 * @param phy_name      - PHY name string
 */
static int legacy_bridge_probe(const char *name, 
                               struct platform_device *pdev,
                               struct resource *addressRange,
                               uint32_t macMatchUnits,
                               uint32_t phy_type,
                               uint32_t phy_addr,
                               const char *phy_name) {
  struct legacy_bridge *bridge;
  struct net_device *ndev = NULL;
  int returnValue;

  /* Create an Ethernet device instance, sized with enough extra data to
   * encapsulate a struct legacy_bridge as its private data
   */
  ndev = alloc_etherdev(sizeof(struct legacy_bridge));
  if (!ndev) {
    dev_err(&pdev->dev, "%s : Could not allocate net device\n", DRIVER_NAME);
    kfree(ndev);
    return(-ENOMEM);
  }
  SET_NETDEV_DEV(ndev, &pdev->dev);

  /* Allocate the dev name early so we can use it in our messages */
  if(strchr(ndev->name, '%')) {
    returnValue = dev_alloc_name(ndev, ndev->name);
    if(returnValue < 0) goto netdev_error;
  }

  /* Point to the private data structure allocated by the parent net_device,
   * and set up navigation back up to the parent
   */
  bridge = netdev_priv(ndev);
  bridge->ndev = ndev;

  /* Request and map the device's I/O memory region into uncacheable space */
  bridge->physicalAddress = addressRange->start;
  bridge->addressRangeSize = ((addressRange->end - addressRange->start) + 1);

  snprintf(bridge->name, NAME_MAX_SIZE, "%s%d", name, instanceCount);
  bridge->name[NAME_MAX_SIZE - 1] = '\0';

  if(request_mem_region(bridge->physicalAddress, bridge->addressRangeSize,
                        bridge->name) == NULL) {
    returnValue = -ENOMEM;
    goto netdev_error;
  }

  bridge->virtualAddress = 
    (void*) ioremap_nocache(bridge->physicalAddress, bridge->addressRangeSize);
  if(!bridge->virtualAddress) {
    returnValue = -ENOMEM;
    goto release;
  }

  /* Retain other parameters */
  bridge->macMatchUnits = macMatchUnits;
  bridge->phy_type      = phy_type;
  if(phy_type != NO_PHY_SUPPLIED_TYPE) {
    bridge->phy_addr      = phy_addr;
    strncpy(bridge->phy_name, phy_name, BUS_ID_SIZE);
    bridge->phy_name[BUS_ID_SIZE - 1] = '\0';
    bridge->phy_dev = NULL;
  }

  /* Announce the device */
  printk(KERN_INFO "\n%s: Found Labrinth Legacy Bridge at 0x%08X\n",
         bridge->name, 
         (uint32_t)bridge->physicalAddress);
  printk(KERN_INFO "  %d MAC filters per AVB port\n\n", bridge->macMatchUnits);

  /* Initialize other resources */
  spin_lock_init(&bridge->mutex);
  bridge->opened = false;

  /* Provide navigation between the device structures */
  platform_set_drvdata(pdev, bridge);
  bridge->pdev = pdev;

  /* Add as a character device to make the instance available for use */
  cdev_init(&bridge->cdev, &legacy_bridge_fops);
  bridge->cdev.owner = THIS_MODULE;
  bridge->instanceNumber = instanceCount++;
  kobject_set_name(&bridge->cdev.kobj, "%s.%d", bridge->name, bridge->instanceNumber);
  cdev_add(&bridge->cdev, MKDEV(DRIVER_MAJOR, bridge->instanceNumber), 1);

  /* Return success */
  return(0);

 release:
  release_mem_region(bridge->physicalAddress, bridge->addressRangeSize);
 netdev_error:
  if(ndev) free_netdev(ndev);

  return(returnValue);
}

#ifdef CONFIG_OF

static int legacy_bridge_platform_remove(struct platform_device *pdev);

/* Probe for registered devices */
static int __devinit legacy_bridge_of_probe(struct of_device *ofdev, 
                                            const struct of_device_id *match)
{
  struct resource r_mem_struct  = {};
  struct resource *addressRange = &r_mem_struct;
  struct resource r_connected_mdio_mem_struct;
  struct platform_device *pdev  = to_platform_device(&ofdev->dev);
  const char *name = dev_name(&ofdev->dev);
  uint32_t macMatchUnits = DEFAULT_MAC_MATCH_UNITS;
  uint32_t phy_type;
  uint32_t phy_addr = 0;
  char phy_name[BUS_ID_SIZE];
  const phandle *mdio_controller_handle;
  struct device_node *mdio_controller_node;
  uint32_t *uint32Ptr;
  int rc = 0;

  printk("Probing device \"%s\"\n", name);

  /* Obtain the resources for this instance */
  rc = of_address_to_resource(ofdev->node, 0, addressRange);
  if(rc) {
    dev_warn(&ofdev->dev, "Invalid address\n");
    return(rc);
  }

  /* Get the number of MAC match units from the device tree */
  uint32Ptr = (uint32_t *) of_get_property(ofdev->node, "xlnx,rx-filters-per-port", NULL);
  if(uint32Ptr != NULL) {
    /* Specify the known number */
    macMatchUnits = *uint32Ptr;
    if(macMatchUnits > MAX_MAC_MATCH_UNITS) {
      dev_warn(&ofdev->dev, "Match unit count (%d) invalid, using default\n", macMatchUnits);
      macMatchUnits = DEFAULT_MAC_MATCH_UNITS;
    }
  }

  /* Connected PHY information; make sure all properties are specified or mark the
   * parameters as having no PHY specified
   */
  uint32Ptr = (uint32_t *) of_get_property(ofdev->node, "xlnx,phy-type", NULL);
  if(uint32Ptr != NULL) {
    phy_type = (uint8_t) *uint32Ptr;
    
    uint32Ptr = (uint32_t *) of_get_property(ofdev->node, "xlnx,phy-addr", NULL);
    if(uint32Ptr != NULL) {
      phy_addr = (uint8_t) *uint32Ptr;

      phy_name[0] = '\0';
      mdio_controller_handle = of_get_property(ofdev->node, "phy-mdio-controller", NULL);
      if(!mdio_controller_handle) {
        dev_warn(&ofdev->dev, "No MDIO connection specified\n");
        phy_type = NO_PHY_SUPPLIED_TYPE;
      } else {
        mdio_controller_node = of_find_node_by_phandle(*mdio_controller_handle);
        if (!mdio_controller_node) {
          dev_warn(&ofdev->dev, "No MDIO connection found\n");
          phy_type = NO_PHY_SUPPLIED_TYPE;
        } else {
          /* The MDIO controller node is itself the entity able to talk over MDIO;
           * it is not a compound device.
           */
          rc = of_address_to_resource(mdio_controller_node, 0, &r_connected_mdio_mem_struct);
          snprintf(phy_name, BUS_ID_SIZE, MDIO_OF_BUSNAME_FMT ":%02x", (u32)r_connected_mdio_mem_struct.start, phy_addr);
          phy_name[BUS_ID_SIZE - 1] = '\0';
          printk("  %s:phy_name: %s\n",__func__, phy_name);
        }
      }
    } else {
      dev_warn(&ofdev->dev, "No PHY address specified\n");
      phy_type = NO_PHY_SUPPLIED_TYPE;
    }
  } else {
    dev_warn(&ofdev->dev, "No PHY type specified\n");
    phy_type = NO_PHY_SUPPLIED_TYPE;
  }

  /* Dispatch to the generic function */
  return(legacy_bridge_probe(name, pdev, addressRange, macMatchUnits, phy_type, phy_addr, phy_name));
}

static int __devexit legacy_bridge_of_remove(struct of_device *dev)
{
  struct platform_device *pdev = to_platform_device(&dev->dev);
  legacy_bridge_platform_remove(pdev);
  return(0);
}

static struct of_device_id legacy_bridge_of_match[] = {
  { .compatible = "xlnx,labrinth-legacy-bridge-1.00.a", },
  { /* end of list */ },
};


static struct of_platform_driver of_legacy_bridge_driver = {
  .name	       = DRIVER_NAME,
  .match_table = legacy_bridge_of_match,
  .probe       = legacy_bridge_of_probe,
  .remove      = __devexit_p(legacy_bridge_of_remove),
};

#endif /* CONFIG_OF */

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int legacy_bridge_platform_probe(struct platform_device *pdev) {
  struct resource *addressRange;

  /* Obtain the resources for this instance */
  addressRange = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!addressRange) {
    printk(KERN_ERR "%s: IO resource address not found.\n", pdev->name);
    return(-ENXIO);
  }

  /* Dispatch to the generic function, using the default number of match units.
   * NOTE - We do not yet have a platform device structure from which to discern
   *        the PHY information.
   */
  return(legacy_bridge_probe(pdev->name, 
                             pdev, 
                             addressRange, 
                             DEFAULT_MAC_MATCH_UNITS, 
                             NO_PHY_SUPPLIED_TYPE, 
                             0, 
                             NULL));
}

/* Remove a previously-probed device */
static int legacy_bridge_remove(struct legacy_bridge *bridge) {
  iounmap(bridge->virtualAddress);
  release_mem_region(bridge->physicalAddress, bridge->addressRangeSize);
  free_netdev(bridge->ndev);
  return(0);
}

static int legacy_bridge_platform_remove(struct platform_device *pdev) {
  struct legacy_bridge *bridge;

  /* Get a handle to the legacy_bridge and begin shutting it down */
  bridge = platform_get_drvdata(pdev);
  if(!bridge) return(-1);
  return(legacy_bridge_remove(bridge));
}

/* Platform device driver structure */
static struct platform_driver legacy_bridge_driver = {
  .probe  = legacy_bridge_platform_probe,
  .remove = legacy_bridge_platform_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __devinit legacy_bridge_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": Biamp Labrinth Legacy Bridge driver\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&of_legacy_bridge_driver);
#endif
 
  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&legacy_bridge_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    return(returnValue);
  }

  /* Allocate a range of major / minor device numbers for use */
  instanceCount = 0;
  if((returnValue = register_chrdev_region(MKDEV(DRIVER_MAJOR, 0), MAX_INSTANCES, DRIVER_NAME)) < 0) { 
    printk(KERN_INFO DRIVER_NAME "Failed to allocate character device range\n");
  }

  return(0);
}

static void __devexit legacy_bridge_driver_exit(void)
{
  unregister_chrdev_region(MKDEV(DRIVER_MAJOR, 0),MAX_INSTANCES);

  /* Unregister as a platform device driver */
  platform_driver_unregister(&legacy_bridge_driver);
}

module_init(legacy_bridge_driver_init);
module_exit(legacy_bridge_driver_exit);

MODULE_AUTHOR("Eldridge M. Mount IV <eldridge.mount@labxtechnologies.com>");
MODULE_DESCRIPTION("Biamp Labrinth Legacy Bridge driver");
MODULE_LICENSE("GPL");
