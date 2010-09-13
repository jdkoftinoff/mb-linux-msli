/*
 * MDIO Phy controller driver
 *
 * Copyright (C) 2010, Lab X Technologies
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */


#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/kernel.h>
#include <linux/phy.h>

#include "labx_eth_locallink.h"
#include "net/labx_ethernet/labx_ethernet_defs.h"


extern void _labx_XLlTemac_PhyRead(XLlTemac *InstancePtr, u32 PhyAddress, 
		u32 RegisterNum, u16 *PhyDataPtr);
extern void _labx_XLlTemac_PhyWrite(XLlTemac *InstancePtr, u32 PhyAddress, 
		u32 RegisterNum, u16 PhyData);
extern void reset(struct net_device *dev, u32 line_num);

static void labx_eth_ll_free_mdio_bus(struct mii_bus *bus)
{
#if 0
	struct mdiobb_ctrl *ctrl = bus->priv;
	
	module_put(ctrl->ops->owner);
	mdiobus_free(bus);
#endif
}

int labx_eth_ll_mdio_read(struct mii_bus *bus, int phy_id, int regnum)
{
	u16 val=0;
	//printk("MR%d", phy_id);
	_labx_XLlTemac_PhyRead((XLlTemac *)(bus->priv),phy_id,regnum,&val);
	return val;
}

int labx_eth_ll_mdio_write(struct mii_bus *bus, int phy_id, int regnum, u16 val)
{
	//printk("MW%d", phy_id);
	_labx_XLlTemac_PhyWrite((XLlTemac *)bus->priv,phy_id,regnum,val);
	return 0;
}

int labx_eth_ll_mdio_reset(struct mii_bus *bus)
{
	printk("labx_eth_ll_mdio_reset\n");
	return 0;
}

int labx_eth_ll_mdio_bus_init(struct device *dev, struct labx_ll_eth_platform_data *pdata, XLlTemac *InstancePtr)
{
	struct mii_bus *new_bus;
	int ret = -ENOMEM;
	int i;

	new_bus = mdiobus_alloc();
	if (!new_bus) {
		printk("Failed mdiobus_alloc(), returning\n");
		goto out_free_bus;
	}

	new_bus->priv = InstancePtr;
	new_bus->read = labx_eth_ll_mdio_read;
	new_bus->write = labx_eth_ll_mdio_write;
	new_bus->reset = labx_eth_ll_mdio_reset;

	new_bus->name = "LabX Locallink MDIO Bus";
	ret = -ENODEV;

	new_bus->phy_mask = ~pdata->phy_mask;
	new_bus->irq = pdata->mdio_phy_irqs;
	new_bus->parent = dev;

	if (new_bus->phy_mask == ~0) {
		goto out_free_bus;
	}

	for (i = 0; i < PHY_MAX_ADDR; i++) {
		if (!new_bus->irq[i]) {
			new_bus->irq[i] = PHY_POLL;
		}
	}

	snprintf(new_bus->id, MII_BUS_ID_SIZE, "%s", pdata->mdio_bus_name);

	InstancePtr->mdio_bus = new_bus;
	ret = mdiobus_register(new_bus);
	if (ret)
	{
		printk("Failed mdiobus_register() \n");
		goto out_free_bus;
	}

	return 0;

out_free_bus:
	printk("%s: Failed\n",__func__);
	InstancePtr->mdio_bus = NULL;
	labx_eth_ll_free_mdio_bus(new_bus);

	return ret;
}

