#ifndef XLLTEMAC_PHY_COMMON_H
#define XLLTEMAC_PHY_COMMON_H

#include "xlltemac_common.h"

/* from mii.h
 *
 * Items in mii.h but not in gmii.h
 */
#define ADVERTISE_100FULL       0x0100
#define ADVERTISE_100HALF       0x0080
#define ADVERTISE_10FULL        0x0040
#define ADVERTISE_10HALF        0x0020
#define ADVERTISE_CSMA          0x0001

#define EX_ADVERTISE_1000FULL   0x0200
#define EX_ADVERTISE_1000HALF   0x0100

/*
 * items not in mii.h nor gmii.h but should be
 */
#define MII_EXADVERTISE 0x09

void _XLlTemac_PhySetMdioDivisor(XLlTemac *InstancePtr,u8 Divisor);
void _XLlTemac_PhyRead(XLlTemac *InstancePtr, u32 PhyAddress,
		       u32 RegisterNum, u16 *PhyDataPtr);
void _XLlTemac_PhyWrite(XLlTemac *InstancePtr, u32 PhyAddress,
			u32 RegisterNum, u16 PhyData);
void xlltemac_phy_setup(struct xlltemac_net_local *lp);

typedef enum DUPLEX { UNKNOWN_DUPLEX, HALF_DUPLEX, FULL_DUPLEX } DUPLEX;

int xlltemac_renegotiate_speed_init(struct net_device *dev, int speed,
				    DUPLEX duplex);
int xlltemac_renegotiate_speed_wait(struct net_device *dev, int speed,
				    DUPLEX duplex);

/*
 * The PHY registers read here should be standard registers in all PHY chips
 */
int xlltemac_get_phy_status(struct net_device *dev, DUPLEX * duplex,
			    int *linkup);
int xlltemac_detect_phy(struct xlltemac_net_local *lp, char *dev_name);

/*
 * The following structure describes PHYs connected to MDIO interface.
 * Usually it's the same interface as the MAC, however it is handled
 * separatelly, so it will be possible to describe various MDIO bus
 * topologies.
 */

typedef struct xtenet_phy_bus
{
  struct xtenet_phy_bus *next;
  u32 BaseAddress;
  unsigned char n_phys;
  unsigned char use_mask;
  unsigned char phys[31];
} xtenet_phy_bus_t;

void xlltemac_remove_phy(struct xlltemac_net_local *lp);
int xlltemac_add_phy(struct xlltemac_net_local *lp);
#endif
