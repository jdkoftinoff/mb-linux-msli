/*
 *  linux/arch/microblaze/platform/Harman/AVB_EP/avb_ep384_tdm_output.h
 *
 *  Lab X Technologies AVB local audio output derived driver,
 *  adding some Studer-specific extensions
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *
 *  Copyright (C) 2012 Harman, All Rights Reserved.
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

#ifndef _AVB_EP384_TDM_OUTPUT_H_
#define _AVB_EP384_TDM_OUTPUT_H_

#include <linux/types.h>
#include <linux/ioctl.h>

/* Driver structure to maintain state for each device instance */
struct avb_ep384_tdm_output {
  /* Pointer back to the platform device */
  struct labx_local_audio_pdev *labxLocalAudio;

  /* Interrupt request number */
  int32_t irq;
};

/* I/O control commands defined by the driver */
#define LFSR_ANALYZER_DISABLE  (0x00)
#define LFSR_ANALYZER_ENABLE   (0x01)

#define ANALYSIS_PSEUDORANDOM  (0x00)
#define ANALYSIS_RAMP          (0x01)

typedef struct {
  uint32_t enable;
  uint32_t signalControl;
  uint32_t sportPort;
  uint32_t sportChannel;
} AnalyzerConfig;

#define IOC_CONFIG_ANALYZER  _IOR('W', 0x01, AnalyzerConfig)

typedef struct {
  uint32_t errorCount;
  uint32_t predictedSample;
  uint32_t actualSample;
} AnalyzerResults;

#define IOC_GET_ANALYZER_RESULTS  _IOW('W', 0x02, AnalyzerResults)

#define IOC_ARM_ERROR_IRQS        _IO('W', 0x03)

/* Special definition to indicate "no stream assigned" to a TDM output */
#define AVB_STREAM_NONE  (0xFFFFFFFF)

/* Structure mapping a single TDM channel to its AVB stream */
typedef struct {
  uint32_t tdmChannel;
  uint32_t avbStream;
} StreamMapEntry;

#define AUTO_MUTE_DISABLE  (0x00)
#define AUTO_MUTE_ENABLE   (0x01)

/* Max number of map entries which can be configured in one ioctl() call */
#define MAX_MAP_ENTRIES  (384)

/* Master structure for configuring the auto-mute logic.  This is used
 * for both global auto-mute enable / disable control as well as for loading
 * TDM channel-to-AVB stream map assignments.
 */
typedef struct {
  uint32_t        enable;
  uint32_t        numMapEntries;
  StreamMapEntry *mapEntries;
} AutoMuteConfig;

#define IOC_CONFIG_AUTO_MUTE      _IOR('W', 0x04, AutoMuteConfig)

#endif
