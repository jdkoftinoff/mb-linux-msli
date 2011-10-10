
/*
 *  linux/arch/microblaze/platform/MeyerSound/CAL_ICS/mv88e6350R.h
 *
 *  Marvell 88E6350R ethernet switch driver
 *
 *  Written by Yi Cao (yi.cao@labxtechnologies.com)
 *
 *  Copyright (C) 2011 Meyer Sound Laboratories Inc, All Rights Reserved.
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

#ifndef _MV88E6350R_H_
#define _MV88E6350R_H

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/highmem.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/ioctl.h>

#define NAME_MAX_SIZE    (256)

struct mvEthSwitch {
  /* Pointer back to the platform device */
  struct platform_device *pdev;

  /* Version information read from the hardware */
  uint32_t versionMajor;
  uint32_t versionMinor;

  /* Character device data */
  struct cdev cdev;
  dev_t       deviceNumber;
  uint32_t    instanceNumber;

  /* Name for use in identification */
  char name[NAME_MAX_SIZE];

  /* Physical and virtual base address */
  uintptr_t      physicalAddress;
  uintptr_t      addressRangeSize;
  void __iomem  *virtualAddress;

  /* Interrupt request number */
  int32_t irq;

  /* Mutex for the device instance */
  spinlock_t mutex;
  bool opened;

  /* File operations and private data for a polymorphic
   * driver to use
   */
  struct file_operations *derivedFops;
  void *derivedData;
};

#endif
