/*
 *  linux/drivers/net/labx_avb/labx_tdm_audio.h
 *
 *  Lab X Technologies Audio time division multiplexing driver
 *
 *  Written by Albert M. Hajjar (albert.hajjar@labxtechnologies.com)
 *
 *  Copyright (C) 2012 Lab X Technologies LLC, All Rights Reserved.
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

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF

#define NAME_MAX_SIZE    (256)

struct tdm_analyzer {
  /* Name for used for identification */
  char tdmName[NAME_MAX_SIZE];
  
  /* Base address of analyzer register set */
  void __iomem  *baseAddress;

  /* IRQ associated with errors */
  int32_t        errorIrq;

  /* Register addresses for interrupts */
  uint32_t       irqMaskReg;
  uint32_t       irqFlagsReg;
};


int32_t     labx_tdm_analyzer_reset(struct tdm_analyzer *analyzer);
int         labx_tdm_analyzer_ioctl(struct tdm_analyzer *analyzer, 
                                    unsigned int command, 
                                    unsigned long arg);
irqreturn_t labx_tdm_analyzer_interrupt(struct tdm_analyzer *analyzer, 
                                        uint32_t maskedFlags);

#endif /* _LABX_TDM_ANALYZER_H_ */

