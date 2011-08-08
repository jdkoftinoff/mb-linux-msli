/*
 *  linux/drivers/net/labx_avb/labx_local_audio.h
 *
 *  Lab X Technologies Local Audio
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

#ifndef _LABX_LOCAL_AUDIO_H_
#define _LABX_LOCAL_AUDIO_H_

#include <linux/fs.h>
#include <linux/labx_dma.h>

/* Local Audio Platform device structure */
struct labx_local_audio_pdev {
  /* Misc device */
  struct miscdevice miscdev;

  /* Pointer back to the platform device */
  struct platform_device *pdev;

  char     name[NAME_MAX_SIZE];

  /* Physical and virtual base address */
  uint32_t physicalAddress;
  uint32_t addressRangeSize;

  /* Virtual address pointer for the memory-mapped hardware */
  void __iomem *virtualAddress;

  /* Number of audio channels supported in this hardware peripheral */
  uint32_t numChannels;

  /* DMA structure */
  struct labx_dma *dma;

  /* File operations and private data for a polymorphic
   * driver to use
   */
  struct file_operations *derivedFops;
  void *derivedData;
};

/* Default region shift to use in the case of an external DMA; this
 * corresponds to a DMA with 512 words of microcode; twice this amount
 * of space is mapped by a DMA to also address its register file.
 */
#define LOCAL_AUDIO_DEFAULT_REGION_SHIFT (12)

/* Local audio registers come after the DMA microcode */
#define LOCAL_AUDIO_REGISTER_RANGE 1

#define LOCAL_AUDIO_REGISTER_BASE(la, reg)                             \
  ((uintptr_t)(la)->virtualAddress |                                   \
                  ((LOCAL_AUDIO_REGISTER_RANGE <<                      \
                    (((la)->dma != NULL) ? ((la)->dma->regionShift + 1) : LOCAL_AUDIO_DEFAULT_REGION_SHIFT)) + \
                   ((reg)*4)))

/* Register address and control field #defines */
#define LOCAL_AUDIO_CHANNEL_REG 0x00


/* Pattern inserter registers are after local audio registers */
#define LOCAL_AUDIO_INSERTER_RANGE 2

#define LOCAL_AUDIO_INSERTER_BASE(la, reg)                             \
  ((uintptr_t)(la)->virtualAddress |                                    \
                  ((LOCAL_AUDIO_INSERTER_RANGE <<                       \
                    (((la)->dma != NULL) ? ((la)->dma->regionShift + 1) : LOCAL_AUDIO_DEFAULT_REGION_SHIFT)) + \
                   ((reg)*4)))

/* Inserter Registers */
#define LOCAL_AUDIO_INSERTER_TDM_CTRL_REG 0x00
#define   LOCAL_AUDIO_INSERTER_ENABLE       0x80000000
#define   LOCAL_AUDIO_INSERTER_ZERO         0x40000000
#define   LOCAL_AUDIO_INSERTER_DC           0x20000000
#define   LOCAL_AUDIO_INSERTER_RAMP         0x10000000
#define   LOCAL_AUDIO_INSERTER_STREAM_MASK  0x0000FF00
#define   LOCAL_AUDIO_INSERTER_STREAM_SHIFT 8
#define   LOCAL_AUDIO_INSERTER_SLOT_MASK    0x000000FF
#define   LOCAL_AUDIO_INSERTER_SLOT_SHIFT   0


/* Pattern tester registers are after pattern inserter registers */
#define LOCAL_AUDIO_TESTER_RANGE 3

#define LOCAL_AUDIO_TESTER_BASE(la, reg)                               \
  ((uintptr_t)(la)->virtualAddress |                                    \
                  ((LOCAL_AUDIO_TESTER_RANGE <<                         \
                    (((la)->dma != NULL) ? ((la)->dma->regionShift + 1) : LOCAL_AUDIO_DEFAULT_REGION_SHIFT)) + \
                   ((reg)*4)))

/* Tester Registers */
#define LOCAL_AUDIO_TESTER_TDM_CTRL_REG         0x00
#define   LOCAL_AUDIO_TESTER_ENABLE         0x80000000
#define   LOCAL_AUDIO_TESTER_DEBUG_PATTERN  0x40000000
#define   LOCAL_AUDIO_TESTER_RAMP           0x20000000
#define   LOCAL_AUDIO_TESTER_STREAM_MASK    0x0000FF00
#define   LOCAL_AUDIO_TESTER_STREAM_SHIFT   8
#define   LOCAL_AUDIO_TESTER_SLOT_MASK      0x000000FF
#define   LOCAL_AUDIO_TESTER_SLOT_SHIFT     0
#define LOCAL_AUDIO_TESTER_ANALYSIS_ERROR_REG   0x01
#define LOCAL_AUDIO_TESTER_ANALYSIS_PREDICT_REG 0x02
#define LOCAL_AUDIO_TESTER_ANALYSIS_ACTUAL_REG  0x03
#define LOCAL_AUDIO_TESTER_TDM_IRQ_MASK_REG     0x04
#define LOCAL_AUDIO_TESTER_TDM_IRQ_FLAGS_REG    0x05

/* Function prototypes for use by derived drivers */
int labx_local_audio_probe(const char *name, 
                           struct platform_device *pdev,
                           struct resource *addressRange,
                           const char *interfaceType,
                           u32 numChannels,
                           struct file_operations *derivedFops,
                           void *derivedData,
                           struct labx_local_audio_pdev **newInstance);

int labx_local_audio_remove(struct labx_local_audio_pdev *local_audio_pdev);

#endif /* _LABX_LOCAL_AUDIO_H_ */

