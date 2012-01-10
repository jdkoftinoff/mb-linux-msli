/*
 *  linux/include/linux/labx_audio_meters_defs.h
 *
 *  Lab X Technologies audio meters definitions
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

#ifndef _AUDIO_METERS_DEFS_H_
#define _AUDIO_METERS_DEFS_H_

#include <linux/types.h>
#include <linux/ioctl.h>

struct AudioMeterData {
	uint32_t numChannels;
	uint32_t meterData[];
};

#define IOC_AM_SET_COEFFICIENT _IOW ('M', 0x01, uint32_t)
#define IOC_AM_GET_METER_COUNT _IOR ('M', 0x02, uint32_t)
#define IOC_AM_READ_METERS     _IOWR('M', 0x03, struct AudioMeterData)

#endif /* _AUDIO_METERS_DEFS_H_ */

