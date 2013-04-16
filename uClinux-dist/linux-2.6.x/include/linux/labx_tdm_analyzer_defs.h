/*
 *  include/linux/labx_tdm_analyzer.h
 *
 *  Lab X Technologies TDM analyzer driver
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

#ifndef _LABX_TDM_ANALYZER_DEFS_H_
#define _LABX_TDM_ANALYZER_DEFS_H_

#include <linux/types.h>
#include <linux/ioctl.h>

/* Generator status control */
#define LFSR_GENERATOR_DISABLE  (0x00)
#define LFSR_GENERATOR_ENABLE   (0x01)

/* Data insertion control */
#define SIGNAL_PASSTHROUGH  0
#define SIGNAL_MUTE         1
#define SIGNAL_DC_PATTERN   2
#define SIGNAL_RAMP         4 
#define SIGNAL_PSUEDORANDOM 8

/* Analyzer status control */
#define LFSR_ANALYZER_DISABLE  (0x00)
#define LFSR_ANALYZER_ENABLE   (0x01)

/* Data analyzer control */
#define ANALYSIS_PSEUDORANDOM 0
#define ANALYSIS_RAMP 1

/* Maximum number of analyzers, based on max number
   of TDM transmitters allowed*/
#define MAX_NUM_ANALYZERS (4)

typedef struct {
  uint32_t enable;
  uint32_t signalControl;
  uint32_t tdmChannel;
  uint32_t tdmLane;
} AnalyzerConfig;

typedef struct {
  uint32_t enable;
  uint32_t signalControl;
  uint32_t tdmChannel;
  uint32_t tdmLane;
} GeneratorConfig;

typedef struct {
  uint32_t errorCount[MAX_NUM_ANALYZERS];
  uint32_t predictedSample[MAX_NUM_ANALYZERS];
  uint32_t actualSample[MAX_NUM_ANALYZERS];
} AnalyzerResults;

/* I/O control commands and structures specific to the audio tdm hardware */
#define ANALYZER_IOC_CHAR          ('l')
#define IOC_CONFIG_GENERATOR      _IOR(ANALYZER_IOC_CHAR, 0x00, GeneratorConfig)
#define IOC_CONFIG_ANALYZER       _IOR(ANALYZER_IOC_CHAR, 0x01, AnalyzerConfig)
#define IOC_GET_ANALYZER_RESULTS  _IOW(ANALYZER_IOC_CHAR, 0x02, AnalyzerResults)
#define IOC_GET_LATENCY           _IOW(ANALYZER_IOC_CHAR, 0x03, uint32_t)
#endif
