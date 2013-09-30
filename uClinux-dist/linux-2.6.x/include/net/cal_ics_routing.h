/*
 *  linux/include/net/cal_ics_routing.h
 *
 *  Lab X Technologies AVB packet engine definitions
 *
 *  Written by Yi Cao (yi.cao@labxtechnologies.com)
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

#ifndef _CAL_ICS_ROUTING_H_
#define _CAL_ICS_ROUTING_H_

#define COEFF_TABLE_MAX_NUM 3

#define IOC_RESET_ROUTING_MATRIX         0
#define IOC_READ_ROUTING_MATRIX          1
#define IOC_SET_ROUTING_MATRIX           2
#define IOC_UPDATE_SOFT_MUTE_COEFF_TABLE 3
#define IOC_CAL_SET_AVB_MUTE             4
#define   CAL_AVB_MUTE_FORCE 0x8
#define   CAL_AVB_MUTE       0x4
#define IOC_CAL_SET_SRC                  5
#define IOC_CAL_SET_RECOVERY_RATE        6

#endif
