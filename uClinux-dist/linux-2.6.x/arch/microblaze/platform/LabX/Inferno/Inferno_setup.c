/*
 * arch/microblaze/platform/Shure/Inferno/Inferno_setup.c
 *
 * Shure Inferno board setup, set up the linkstreet ethernet switch
 * as a platform device
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/resource.h>





static struct platform_device marvell_switch_device = {
	.name		= "mv88e6350R",
	.id		= 0,
	.num_resources	= 0,
};



static void __init mv88e6350R_switch_init(void)
{

    platform_device_register(&marvell_switch_device);
}

subsys_initcall(mv88e6350R_switch_init);
