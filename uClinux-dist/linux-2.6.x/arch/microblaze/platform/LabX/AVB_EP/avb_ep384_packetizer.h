/*
 *  linux/arch/microblaze/platform/LabX/AVB_EP/avb_ep384_packetizer.h
 *
 *  Lab X Technologies AVB flexible audio packetizer derived driver,
 *  adding some AVBEP-specific extensions
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

#ifndef _AVB_EP384_PACKETIZER_H_
#define _AVB_EP384_PACKETIZER_H_

#include <linux/types.h>
#include <linux/ioctl.h>

/* Driver structure to maintain state for each device instance */
struct avb_ep384_packetizer {
  /* Pointer back to the platform device */
  struct audio_packetizer *labxPacketizer;
};

/* I/O control commands defined by the driver */

#define EP_SIGNAL_PSUEDORANDOM  (0x00)
#define EP_SIGNAL_MUTE          (0x01)
#define EP_SIGNAL_DC_PATTERN    (0x02)
#define EP_SIGNAL_RAMP          (0x03)

#endif
