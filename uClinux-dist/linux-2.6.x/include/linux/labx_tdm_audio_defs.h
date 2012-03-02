/*
 *  linux/include/labx_tdm_audio_defs.h 
 *
 *  Lab X Technologies AVB local audio output derived driver,
 *  adding some Labrinth-specific extensions
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *
 *  Copyright (C) 2010 Biamp Systems, All Rights Reserved.
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

#ifndef _LABX_TDM_AUDIO_DEFS_H_
#define _LABX_TDM_AUDIO_DEFS_H_

#include <linux/types.h>
#include <linux/ioctl.h>

/* Definitions for operating contexts */
#  define TDM_BIT_ALIGNMENT_LEFT_JUSTIFIED   (0x0)
#  define TDM_BIT_ALIGNMENT_I2S_DELAYED      (0x200)
#  define TDM_LRCLK_RISING_EDGE_CH0          (0x0)
#  define TDM_LRCLK_FALLING_EDGE_CH0         (0x100)

/* Special definition to indicate "no stream assigned" to a TDM output */
#define AVB_STREAM_NONE  (0xFFFFFFFF)
#define AVB_STREAM_RESET (0x00000000)

/* Structure mapping a single TDM channel to its AVB stream */
typedef struct {
  uint32_t tdmChannel;
  uint32_t avbStream;
} StreamMapEntry;

#define AUTO_MUTE_ALWAYS    (0x00)
#define AUTO_MUTE_STMSTATUS (0x01)
#define AUTO_MUTE_NEVER     (0x02)

/* Max number of map entries which can be configured in one ioctl() call */
#define MAX_MAP_ENTRIES  (64)

/* Master structure for configuring the auto-mute logic.  This is used
 * for both global auto-mute enable / disable control as well as for loading
 * TDM channel-to-AVB stream map assignments.
 */
typedef struct {
  uint32_t        enable;
  uint32_t        numMapEntries;
  StreamMapEntry *mapEntries;
} AutoMuteConfig;


struct labx_tdm_platform_data {
  uint8_t lane_count;
  uint8_t num_streams;
  uint8_t slot_density;
};

/* I/O control commands and structures specific to the audio tdm hardware */
typedef struct {
  uint32_t versionMajor;
  uint32_t versionMinor;
  uint32_t maxChannels;
  uint32_t lrPolarity;
  uint32_t i2sAlign;
} AudioTdmControl;

#define AUDIO_TDM_IOC_CHAR          ('t')
#define IOC_GET_AUDIO_TDM_CONTROL       _IOR(AUDIO_TDM_IOC_CHAR, 0x01, AudioTdmControl)
#define IOC_SET_AUDIO_TDM_CONTROL       _IOW(AUDIO_TDM_IOC_CHAR, 0x02, AudioTdmControl)
#define IOC_CONFIG_AUTO_MUTE            _IOR(AUDIO_TDM_IOC_CHAR, 0x03, AutoMuteConfig)

#endif
