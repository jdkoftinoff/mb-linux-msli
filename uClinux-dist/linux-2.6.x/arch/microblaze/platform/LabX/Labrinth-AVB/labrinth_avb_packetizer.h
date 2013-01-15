/*
 *  linux/arch/microblaze/platform/LabX/Labrinth-AVB/labrinth_avb_packetizer.h
 *
 *  Lab X Technologies AVB flexible audio packetizer derived driver,
 *  adding some Labrinth-specific extensions
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *
 *  Copyright (C) 2010 Lab X Technologies, LLC, All Rights Reserved.
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

#ifndef _LABRINTH_AVB_PACKETIZER_H_
#define _LABRINTH_AVB_PACKETIZER_H_

#include <linux/types.h>
#include <linux/ioctl.h>

/* Driver structure to maintain state for each device instance */
struct labrinth_packetizer {
  /* Pointer back to the platform device */
  struct audio_packetizer *labxPacketizer;
};

/* I/O control commands defined by the driver */
#define LFSR_GENERATOR_DISABLE  (0x00)
#define LFSR_GENERATOR_ENABLE   (0x01)

#define SIGNAL_PSUEDORANDOM  (0x00)
#define SIGNAL_MUTE          (0x01)
#define SIGNAL_DC_PATTERN    (0x02)
#define SIGNAL_RAMP          (0x03)

typedef struct {
  uint32_t enable;
  uint32_t signalControl;
  uint32_t sportPort;
  uint32_t sportChannel;
} GeneratorConfig;

#define IOC_CONFIG_GENERATOR  _IOR('l', 0x01, GeneratorConfig)

#define IOC_GET_LATENCY _IOW('l', 0x02, uint32_t)

#endif
