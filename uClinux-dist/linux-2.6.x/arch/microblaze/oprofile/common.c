/*
 * File:         arch/microblaze/oprofile/common.c
 * Based on:     arch/blackfin/oprofile/common.c
 * Author:       Chris Wulff <chris.wulff@labxtechnologies.com
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

#include <linux/oprofile.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/ptrace.h>
#include <linux/irq.h>
#include <linux/io.h>

#include <asm/system.h>

#include "op_microblaze.h"

#if 0
static int pfmon_enabled;
static struct mutex pfmon_lock;

struct op_counter_config ctr;

static int op_microblaze_setup(void)
{
	int ret = 0;

	return ret;
}

static void op_microblaze_shutdown(void)
{
}

static int op_microblaze_start(void)
{
	int ret = 0;

	mutex_lock(&pfmon_lock);
	if (!pfmon_enabled) {
		pfmon_enabled = 1;
	}
	mutex_unlock(&pfmon_lock);

	return ret;
}

static void op_microblaze_stop(void)
{
	mutex_lock(&pfmon_lock);
	if (pfmon_enabled) {
		pfmon_enabled = 0;
	}
	mutex_unlock(&pfmon_lock);
}

static int op_microblaze_create_files(struct super_block *sb, struct dentry *root)
{
	struct dentry *dir;

	printk(KERN_INFO "Oprofile: creating files... \n");

	dir = oprofilefs_mkdir(sb, root, "0");

	oprofilefs_create_ulong(sb, dir, "enabled", &ctr.enabled);
	oprofilefs_create_ulong(sb, dir, "event", &ctr.event);
	oprofilefs_create_ulong(sb, dir, "count", &ctr.count);
	/*
	 * We dont support per counter user/kernel selection, but
	 * we leave the entries because userspace expects them
	 */
	oprofilefs_create_ulong(sb, dir, "kernel", &ctr.kernel);
	oprofilefs_create_ulong(sb, dir, "user", &ctr.user);
	oprofilefs_create_ulong(sb, dir, "unit_mask", &ctr.unit_mask);

	return 0;
}
#endif

int __init oprofile_arch_init(struct oprofile_operations *ops)
{
#if 0
	mutex_init(&pfmon_lock);

	printk(KERN_INFO "oprofile_arch_init. \n");

	ops->cpu_type = "microblaze";
	ops->create_files = op_microblaze_create_files;
	ops->setup = op_microblaze_setup;
	ops->shutdown = op_microblaze_shutdown;
	ops->start = op_microblaze_start;
	ops->stop = op_microblaze_stop;

	printk(KERN_INFO "oprofile: using %s performance monitoring.\n",
	       ops->cpu_type);

	return 0;
#else
        return -1;
#endif
}

void oprofile_arch_exit(void)
{
}
