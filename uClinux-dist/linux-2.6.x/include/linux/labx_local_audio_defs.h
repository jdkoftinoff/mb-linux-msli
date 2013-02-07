/*
 *  linux/include/linux/labx_local_audio_defs.h
 *
 *  Lab X Technologies local audio definitions
 *
 *  Written by Chris Wulff (chris.wulff@labxtechnologies.com)
 *
 *  Copyright (C) 2010 Lab X Technologies LLC, All Rights Reserved.
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

#ifndef _LOCAL_AUDIO_DEFS_H_
#define _LOCAL_AUDIO_DEFS_H_

#include <linux/types.h>
#include <linux/ioctl.h>

/* Interface type definitions for use by probing drivers */
#define LA_DMA_INTERFACE_EXTERNAL "DMA_EXT"
#define LA_DMA_INTERFACE_NPI      "DMA_NPI"
#define LA_DMA_INTERFACE_PLB      "DMA_PLB"

struct LocalAudioChannelMapping {
	uint32_t channel;
	uint32_t streams;
};

#define IOC_LA_SET_CHANNEL_MAPPING _IOW('L', 0x01, struct LocalAudioChannelMapping)
#define IOC_LA_GET_CHANNEL_MAPPING _IOWR('L', 0x02, struct LocalAudioChannelMapping)

/* Data insertion control */
#define LA_INSERT_MODE_OFF  0
#define LA_INSERT_MODE_ZERO 1
#define LA_INSERT_MODE_DC   2
#define LA_INSERT_MODE_RAMP 3
#define LA_INSERT_MODE_LFSR 4

struct LocalAudioInsertConfig {
	uint32_t mode;
	uint32_t stream;
	uint32_t slot;
};
#define IOC_LA_SET_INSERT_MODE _IOW('L', 0x03, struct LocalAudioInsertConfig)

#endif /* _LOCAL_AUDIO_DEFS_H_ */

