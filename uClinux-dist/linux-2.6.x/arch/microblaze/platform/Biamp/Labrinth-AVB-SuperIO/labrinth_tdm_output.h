/*
 *  linux/arch/microblaze/platform/Biamp/labrinth_tdm_output.h
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

#ifndef _LABRINTH_TDM_OUTPUT_H_
#define _LABRINTH_TDM_OUTPUT_H_

#include <linux/types.h>
#include <linux/ioctl.h>

/* Driver structure to maintain state for each device instance */
struct labrinth_tdm_output {
  /* Pointer back to the platform device */
  struct labx_local_audio_pdev *labxLocalAudio;
};

/* I/O control commands defined by the driver */
#define LFSR_ANALYZER_DISABLE  (0x00)
#define LFSR_ANALYZER_ENABLE   (0x01)
typedef struct {
  uint32_t enable;
  uint32_t sportPort;
  uint32_t sportChannel;
} AnalyzerConfig;

#define IOC_CONFIG_ANALYZER  _IOR('l', 0x01, AnalyzerConfig)

typedef struct {
  uint32_t errorCount;
  uint32_t predictedSample;
  uint32_t actualSample;
} AnalyzerResults;

#define IOC_GET_ANALYZER_RESULTS  _IOW('l', 0x02, AnalyzerResults)

#endif
