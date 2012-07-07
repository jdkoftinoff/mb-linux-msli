/*
 * File:         arch/microblaze/oprofile/op_microblaze.h
 * Based on:     arch/blackfin/oprofile/op_blackfin.h
 * Author:       Chris Wulff <chris.wulff@labxtechnologies.com>
 *
 * Created:
 * Description:
 *
 * Modified:
 *               Copyright (C) 2004 Anton Blanchard <anton@au.ibm.com>, IBM
 *               Copyright 2004-2006 Analog Devices Inc.
 *
 * Bugs:         
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
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef OP_MICROBLAZE_H
#define OP_MICROBLAZE_H 1

/* Per-counter configuration as set via oprofilefs.  */
struct op_counter_config {
	unsigned long valid;
	unsigned long enabled;
	unsigned long event;
	unsigned long count;
	unsigned long kernel;
	unsigned long user;
	unsigned long unit_mask;
};

/* System-wide configuration as set via oprofilefs.  */
struct op_system_config {
	unsigned long enable_kernel;
	unsigned long enable_user;
};

#endif
