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

#ifndef _LABX_TDM_ANALYZER_H_
#define _LABX_TDM_ANALYZER_H_

#include <linux/types.h>
#include <linux/ioctl.h>

/* Analyzer status control */
#define ANALYZER_DISABLE  (0x00)
#define ANALYZER_ENABLE   (0x01)

/* Data insertion control */
#define INSERT_MODE_ZERO 1
#define INSERT_MODE_DC   2
#define INSERT_MODE_RAMP 4 
#define INSERT_MODE_LFSR 8

/* Data analyzer control */
#define ANALYZE_MODE_LFSR 0
#define ANALYZE_MODE_RAMP 1

typedef struct {
  uint32_t enable;
  uint32_t signalControl;
  uint32_t tdmSlot;
  uint32_t tdmLane;
} AnalyzerConfig;

typedef struct {
  uint32_t errorCount;
  uint32_t predictedSample;
  uint32_t actualSample;
} AnalyzerResults;

/* I/O control commands and structures specific to the audio tdm hardware */
#define ANALYZER_IOC_CHAR          ('l')
#define IOC_CONFIG_GENERATOR      _IOR(ANALYZER_IOC_CHAR, 0x00, AnalyzerConfig)
#define IOC_CONFIG_ANALYZER       _IOR(ANALYZER_IOC_CHAR, 0x01, AnalyzerConfig)
#define IOC_GET_ANALYZER_RESULTS  _IOW(ANALYZER_IOC_CHAR, 0x02, AnalyzerResults)
#endif
