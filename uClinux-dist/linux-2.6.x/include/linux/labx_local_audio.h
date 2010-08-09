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

  /* Number of audio channels supported in this hardware peripheral */
  uint32_t numChannels;

  /* DMA structure */
  struct labx_dma dma;
};

/* Local audio registers come after the DMA microcode */
#define LOCAL_AUDIO_REGISTER_RANGE 2

#define LOCAL_AUDIO_REGISTER_BASE(dma, reg)                     \
  ((uintptr_t)(dma)->virtualAddress |                           \
   ((LOCAL_AUDIO_REGISTER_RANGE << ((dma)->regionShift)) + ((reg)*4)))

/* Register address and control field #defines */
#define LOCAL_AUDIO_CHANNEL_REG 0x00

#endif /* _LABX_LOCAL_AUDIO_H_ */

