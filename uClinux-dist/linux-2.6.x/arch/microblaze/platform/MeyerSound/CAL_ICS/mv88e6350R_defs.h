/*
 *  arch/microblaze/platform/MeyerSound/CAL_ICS/_defs.h
 *
 *  Lab X Technologies mv88E6350R switch definitions
 *
 *  Written by Chris Wulff (chris.wulff@labxtechnologies.com)
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

#ifndef _MV88E6350R_DEFS_H_
#define _MV88E6350R_DEFS_H_

#include <linux/types.h>
#include <linux/ioctl.h>

#define MV_MDIO_NO_PAGE 0xff

#define MV_REG_PHY(p)  (p)
#define MV_REG_PORT(p) (0x10 + (p))
#define MV_REG_GLOBAL  0x1b
#define MV_REG_GLOBAL2 0x1c

typedef struct mv_mdio {
  uint32_t data;
  uint32_t addr;
  uint32_t page;
  uint32_t reg;
} mv_mdio_t;

typedef struct mv_mdio_rmw {
  uint32_t ormask;
  uint32_t andmask;
  uint32_t addr;
  uint32_t page;
  uint32_t reg;
} mv_mdio_rmw_t;

/* read/write PHY/MAC/GLOBAL registers in the switch */
#define IOC_MV_MDIO_REG_READ           _IOWR('M', 0x01, struct mv_mdio)
#define IOC_MV_MDIO_REG_WRITE          _IOW ('M', 0x02, struct mv_mdio)
#define IOC_MV_MDIO_REG_MODIFY         _IOW ('M', 0x03, struct mv_mdio_rmw)

#endif /* _MV88E6350R_DEFS_H_ */

