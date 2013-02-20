/*
 *  include/linux/labx_tdm_audio_defs.h
 *
 *  Lab X Technologies TDM audio driver
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *  Written by Albert M. Hajjar (albert.hajjar@labxtechnologies.com)
 *
 *  Copyright (C) 2012 Lab X Technologies, All Rights Reserved.
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

/* Application definitions for operating contexts */
#  define LRCLK_FALLING_EDGE_CH0         0 
#  define LRCLK_RISING_EDGE_CH0          1 
#  define LRCLK_MODE_NORMAL              0
#  define LRCLK_MODE_PULSE               1
#  define BIT_ALIGNMENT_LEFT_JUSTIFIED   0
#  define BIT_ALIGNMENT_I2S_DELAYED      1
#  define MASTER_MODE                    0
#  define SLAVE_MODE                     1
#  define SAMPLE_DEPTH_24BIT             0
#  define SAMPLE_DEPTH_16BIT             1

/* Special definition to indicate "no stream assigned" to a TDM output */
#define AVB_STREAM_NONE  (0xFFFFFFFF)
#define AVB_STREAM_RESET (0x00000000)

/* Structure mapping a single TDM channel to its AVB stream */
typedef struct {
  uint32_t tdmChannel;
  uint32_t avbStream;
  uint32_t enable;
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

typedef struct {
  uint32_t versionMajor;
  uint32_t versionMinor;
  uint32_t laneCount;
  uint32_t mclkRatio;
  uint32_t maxNumStreams;
  uint32_t maxSlotDensity;
  uint32_t minBurstLength;
  uint32_t maxBurstMultiple;
  uint32_t hasLoopback;
  uint32_t hasSlaveManager;
  uint32_t hasAnalyzer;
  uint32_t hasDynamicSampleRates;
} TdmCaps;

/* Enumeration defining bitmask for modifiable parameters */
typedef enum {
  SLOT_DENSITY,
  NUM_CHANNELS,
  BURST_LENGTH_MULTIPLE,
  I2S_ALIGN,
  LR_CLOCK_MODE,
  SAMPLE_EDGE,
  SAMPLE_RATE,
  SAMPLE_DEPTH,
  TDM_MODULE_OWNER,
  TDM_TX_OWNER,
  TDM_RX_OWNER,
  MCLK_DIVIDER,
  NUM_BITMASK_ENTRIES,
} AudioTdmBitMask;

/* Structure containing modifiable parameters */
typedef struct {
  uint32_t slotDensity;
  uint32_t numChannels;
  uint32_t burstLengthMultiple;
  uint32_t i2sAlign;
  uint32_t lrClockMode; 
  uint32_t sampleEdge;
  uint32_t sampleRate;
  uint32_t sampleDepth;
  uint32_t tdmModuleOwner;
  uint32_t tdmTxOwner;
  uint32_t tdmRxOwner;
  uint32_t mclkDivider;
  AudioTdmBitMask bitMask;
} AudioTdmControl;

/* I/O control commands and structures specific to the audio tdm hardware */
#define AUDIO_TDM_IOC_CHAR          ('t')
#define IOC_GET_AUDIO_TDM_CAPS          _IOR(AUDIO_TDM_IOC_CHAR, 0x01, TdmCaps)
#define IOC_GET_AUDIO_TDM_CONTROL       _IOR(AUDIO_TDM_IOC_CHAR, 0x02, AudioTdmControl)
#define IOC_SET_AUDIO_TDM_CONTROL       _IOW(AUDIO_TDM_IOC_CHAR, 0x03, AudioTdmControl)
#define IOC_CONFIG_AUTO_MUTE            _IOR(AUDIO_TDM_IOC_CHAR, 0x04, AutoMuteConfig)
#define IOC_ARM_ERROR_IRQS              _IO(AUDIO_TDM_IOC_CHAR,  0x05)

/* Error number definitions */
#define LABX_TDM_AUDIO_ERRNO_BASE  0x0400
#define IS_LABX_TDM_AUDIO_ERRNO(errno) (errno > LABX_TDM_AUDIO_ERRNO_BASE && errno <= (LABX_TDM_AUDIO_ERRNO_BASE + ETDMERRCNT))

enum TdmErrno {
  TDMERRNOBASE = LABX_TDM_AUDIO_ERRNO_BASE,
  ENUMCHNOTSUPPORTED,
  ENUMCHEXCDSCONF,
  ESLTDSTYNOTSUPPORTED,
  ESLTDSTYEXCDSCONF,
  ESMPLRATENOTSUPPORTED,
  ESMPLRATEINVALID,
  EMCLKDTOOHIGH,
  EMCLKDNOTSUPPORTED,
  ESCMNOTIMPL,
  EBADBURSTLENMUL,
  ETDMERRCNT = EBADBURSTLENMUL - LABX_TDM_AUDIO_ERRNO_BASE
};

#ifndef __KERNEL__
#define LABX_TDM_AUDIO_ERRSTRING(errno) (labxTdmAudioErrnoStrings[((errno) & ~LABX_TDM_AUDIO_ERRNO_BASE) - 1])

#ifdef LABX_TDM_AUDIO_ERRNO_STRINGS
const char* labxTdmAudioErrnoStrings[ETDMERRCNT] = {
  "Number of channels not supported by platform",
  "Number of channels exceeds maximum supported by lane count/slot density/sample rate configuration",
  "Slot density not supported by platform",
  "Slot density exceeds maximum supported by lane/channel count and sample rate configuration",
  "Sample rate not supported by channel configuration",
  "Invalid sample rate",
  "Master clock divider brings master clock below nominal frequency",
  "Master clock divider value not supported",
  "Slave clock manager not implemented",
  "Invalid burst length divider"
};
#else
extern const char* labxTdmAudioErrnoStrings[ETDMERRCNT];
#endif
#endif /* !__KERNEL__ */

#endif
