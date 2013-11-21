/*
 *  drivers/char/labx_tcpa.h
 *
 *  Lab X Technologies TCP Accelerator
 *
 *  Written by Tom Bottom (tom.bottom@labxtechnologies.com)
 *
 *  Copyright (C) 2013 Lab X Technologies LLC, All Rights Reserved.
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

#ifndef _LABX_TCPA_H_
#define _LABX_TCPA_H_

#include <linux/ioport.h>
#include <linux/miscdevice.h>

#define NAME_MAX_SIZE  256

/* Constant allowing an encapsulating driver to tell us that it has
 * no IRQ resources for us to use
 */
#define TCPA_NO_IRQ_SUPPLIED  (-1)

/* TCP Accelerator Platform device structure */
struct labx_tcpa_pdev {
  /* Misc device */
  struct miscdevice miscdev;

  /* Pointer back to the platform device */
  struct platform_device *pdev;

  char          name[NAME_MAX_SIZE];

  /* Physical and virtual base address */
  uint32_t      physicalAddress;
  uint32_t      addressRangeSize;

  /* Virtual address pointer for the memory-mapped hardware */
  void __iomem  *virtualAddress;

  /* Device node of the enclosing device */
  dev_t deviceNode;

  /* Interrupt request number */
  int32_t irq;
};

#define TCPA_REGISTER_RANGE 0
#define TCPA_TEMPLATE_RANGE 1

#define TCPA_REGISTER_ADDRESS(tcpa, offset)      \
  ((uintptr_t)(tcpa)->virtualAddress |           \
   (TCPA_REGISTER_RANGE << 7) | ((offset) << 2))

#define TCPA_TEMPLATE_ADDRESS(tcpa, offset)      \
  ((uintptr_t)(tcpa)->virtualAddress |           \
   (TCPA_TEMPLATE_RANGE << 7) | ((offset) << 2))

#define TCPA_TEMPLATE_WORDS 14

/* Register address and control field #defines */
#define TCPA_CONTROL_REG                 0x00
#  define TCPA_DISABLE  0x00000000
#  define TCPA_ENABLE   0x00000001
#define TCPA_STREAM_WORDS_REG            0x01
#define TCPA_INITIAL_CHECKSUM_REG        0x02
#define TCPA_INITIAL_SEQUENCE_REG        0x03
#define TCPA_INITIAL_IP_ID_REG           0x04
#define TCPA_TEMPLATE_SIZE_REG           0x05
#define TCPA_RETRANSMIT_TICKS_REG        0x06

#endif /* _LABX_TCPA_H_ */

