/*
 *  linux/include/linux/labx_microengine_defs.h
 *
 *  Lab X Technologies microengine definitions
 *
 *  Written by Chris Wulff (chris.wulff@labxtechnologies.com)
 *
 *  Copyright (C) 2009 Lab X Technologies LLC, All Rights Reserved.
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

#ifndef _LABX_MICROENGINE_DEFS_H_
#define _LABX_MICROENGINE_DEFS_H_

#include <linux/ioctl.h>

/* This header file contains common driver definitions for any Lab X
 * microengine peripheral.  Use of these definitions permits the use
 * of a common base class in the userspace abstraction libraries.
 */

/* I/O control command hash inputs
 * Client drivers may define their own unique I/O control operations beginning
 * at ENGINE_IOC_CLIENT_START
 */
#define ENGINE_IOC_CHAR          ('e')
#define ENGINE_IOC_CLIENT_START  (0x10)

/* I/O control commands and structures common to all packet engines */
#define IOC_START_ENGINE           _IO(ENGINE_IOC_CHAR, 0x01)
#define IOC_STOP_ENGINE            _IO(ENGINE_IOC_CHAR, 0x02)

typedef struct {
  uint32_t  offset;
  uint32_t  numWords;
  uint32_t *configWords;
  uint32_t  interlockedLoad;
  uint32_t  loadFlags;
} ConfigWords;

#define IOC_LOAD_DESCRIPTOR        _IOW(ENGINE_IOC_CHAR,  0x03, ConfigWords)
#define IOC_COPY_DESCRIPTOR        _IOWR(ENGINE_IOC_CHAR, 0x04, ConfigWords)

/* Definitions for the interlockedLoad member.  An "interlocked" load makes
 * use of hardware interlocks to ensure the final word of a descriptor load
 * is written in a manner guaranteed not to disrupt the running microengine.
 */
#define LOAD_NORMAL       (0)
#define LOAD_INTERLOCKED  (1)

#endif
