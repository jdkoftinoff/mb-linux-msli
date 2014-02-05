/*
 *  linux/drivers/net/netconsole.c
 *
 *  Copyright (C) 2001  Ingo Molnar <mingo@redhat.com>
 *
 *  This file contains the implementation of an IRQ-safe, crash-safe
 *  kernel console implementation that outputs kernel messages to the
 *  network.
 *
 * Modification history:
 *
 * 2001-09-17    started by Ingo Molnar.
 * 2003-08-11    2.6 port by Matt Mackall
 *               simplified options
 *               generic card hooks
 *               works non-modular
 * 2003-09-07    rewritten with netpoll api
 */

/****************************************************************
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2, or (at your option)
 *      any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, write to the Free Software
 *      Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 ****************************************************************/

#include <linux/mm.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/netpoll.h>
#include <linux/inet.h>
#include <linux/configfs.h>

MODULE_AUTHOR("Maintainer: Matt Mackall <mpm@selenic.com>");
MODULE_DESCRIPTION("Console driver for network interfaces");
MODULE_LICENSE("GPL");

#define MAX_PARAM_LENGTH	256
#define MAX_17221_PACKET_SIZE 524
#define ETH_VLAN_SIZE 0 /* 0 or 2 */
#define JDKSAVDECC_FRAME_HEADER_LEN (14)
#define JDKSAVDECC_COMMON_CONTROL_HEADER_LEN (12)
#define JDKSAVDECC_AEM_COMMAND_SET_CONTROL_RESPONSE_LEN (JDKSAVDECC_COMMON_CONTROL_HEADER_LEN + 16)
#define JDKSAVDECC_JDKS_LOG_CONTROL_OFFSET_TEXT (JDKSAVDECC_AEM_COMMAND_SET_CONTROL_RESPONSE_LEN + 14)
#define ETH_PACKET_HEADER_SIZE (JDKSAVDECC_FRAME_HEADER_LEN+ETH_VLAN_SIZE)
#define PROXY_OVERHEAD 20
#define MAX_MESSAGE_SIZE (MAX_17221_PACKET_SIZE-ETH_PACKET_HEADER_SIZE-ETH_VLAN_SIZE-PROXY_OVERHEAD-JDKSAVDECC_JDKS_LOG_CONTROL_OFFSET_TEXT)

#define MAX_PRINT_CHUNK		MAX_MESSAGE_SIZE

static char config[MAX_PARAM_LENGTH];
module_param_string(netconsole, config, MAX_PARAM_LENGTH, 0);
MODULE_PARM_DESC(netconsole, " netconsole=[descriptor],[iface]/[primary-iface];[descriptor],[iface]/[primary-iface]");

#ifndef	MODULE
static int __init option_setup(char *opt)
{
	strlcpy(config, opt, MAX_PARAM_LENGTH);
	return 1;
}
__setup("netconsole=", option_setup);
#endif	/* MODULE */

/* Linked list of all configured targets */
static LIST_HEAD(target_list);

/* This needs to be a spinlock because write_msg() cannot sleep */
static DEFINE_SPINLOCK(target_list_lock);

/**
 * struct netconsole_target - Represents a configured netconsole target.
 * @list:	Links this target into the target_list.
 * @item:	Links us into the configfs subsystem hierarchy.
 * @enabled:	On / off knob to enable / disable target.
 *		Visible from userspace (read-write).
 *		We maintain a strict 1:1 correspondence between this and
 *		whether the corresponding netpoll is active or inactive.
 *		Also, other parameters of a target may be modified at
 *		runtime only when it is disabled (enabled == 0).
 * @np:		The netpoll structure for this target.
 *		Contains the other userspace visible parameters:
 *		dev_name	(read-write)
 *		local_mac	(read-only)
 */
struct netconsole_target {
	struct list_head	list;
	int			enabled;
	struct netpoll		np;
};

static int __init dynamic_netconsole_init(void)
{
	return 0;
}

static void __exit dynamic_netconsole_exit(void)
{
}

/*
 * No danger of targets going away from under us when dynamic
 * reconfigurability is off.
 */
static void netconsole_target_get(struct netconsole_target *nt)
{
}

static void netconsole_target_put(struct netconsole_target *nt)
{
}

/* Allocate new target (from boot/module param) and setup netpoll for it */
static struct netconsole_target *alloc_param_target(char *target_config)
{
	int err = -ENOMEM;
	struct netconsole_target *nt;

	/*
	 * Allocate and initialize with defaults.
	 * Note that these targets get their config_item fields zeroed-out.
	 */
	nt = kzalloc(sizeof(*nt), GFP_KERNEL);
	if (!nt) {
		printk(KERN_ERR "netconsole: failed to allocate memory\n");
		goto fail;
	}

	nt->np.name = "netconsole";
	strlcpy(nt->np.dev_name, "eth0", IFNAMSIZ);

	nt->np.descriptor_index=1;

	/* Parse parameters and setup netpoll */
	err = netpoll_parse_options(&nt->np, target_config);
	if (err)
		goto fail;

	err = netpoll_setup(&nt->np);
	if (err)
		goto fail;

	nt->enabled = 1;

	return nt;

fail:
	kfree(nt);
	return ERR_PTR(err);
}

/* Cleanup netpoll for given target (from boot/module param) and free it */
static void free_param_target(struct netconsole_target *nt)
{
	netpoll_cleanup(&nt->np);
	kfree(nt);
}

/* Handle network interface device notifications */
static int netconsole_netdev_event(struct notifier_block *this,
				   unsigned long event,
				   void *ptr)
{
	unsigned long flags;
	struct netconsole_target *nt;
	struct net_device *dev = ptr;

	if (!(event == NETDEV_CHANGENAME || event == NETDEV_UNREGISTER))
		goto done;

	spin_lock_irqsave(&target_list_lock, flags);
	list_for_each_entry(nt, &target_list, list) {
		netconsole_target_get(nt);
		if (nt->np.dev == dev) {
			switch (event) {
			case NETDEV_CHANGENAME:
				strlcpy(nt->np.dev_name, dev->name, IFNAMSIZ);
				break;
			case NETDEV_UNREGISTER:
				if (!nt->enabled)
					break;
				netpoll_cleanup(&nt->np);
				nt->enabled = 0;
				printk(KERN_INFO "netconsole: network logging stopped"
					", interface %s unregistered\n",
					dev->name);
				break;
			}
		}
		netconsole_target_put(nt);
	}
	spin_unlock_irqrestore(&target_list_lock, flags);

done:
	return NOTIFY_DONE;
}

static struct notifier_block netconsole_netdev_notifier = {
	.notifier_call  = netconsole_netdev_event,
};

static void write_msg(struct console *con, const char *msg, unsigned int len)
{
	int frag, left;
	unsigned long flags;
	struct netconsole_target *nt;
	const char *tmp;

	/* Avoid taking lock and disabling interrupts unnecessarily */
	if (list_empty(&target_list))
		return;

	spin_lock_irqsave(&target_list_lock, flags);
	list_for_each_entry(nt, &target_list, list) {
		netconsole_target_get(nt);
		if (nt->enabled && netif_running(nt->np.dev)) {
			/*
			 * We nest this inside the for-each-target loop above
			 * so that we're able to get as much logging out to
			 * at least one target if we die inside here, instead
			 * of unnecessarily keeping all targets in lock-step.
			 */
			tmp = msg;
			for (left = len; left;) {
				frag = min(left, MAX_PRINT_CHUNK);
				netpoll_send_udp(&nt->np, tmp, frag);
				tmp += frag;
				left -= frag;
			}
		}
		netconsole_target_put(nt);
	}
	spin_unlock_irqrestore(&target_list_lock, flags);
}

static struct console netconsole = {
	.name	= "netcon",
	.flags	= CON_ENABLED,
	.write	= write_msg,
};

static int __init init_netconsole(void)
{
	int err;
	struct netconsole_target *nt, *tmp;
	unsigned long flags;
	char *target_config;
	char *input = config;

	if (strnlen(input, MAX_PARAM_LENGTH)) {
		while ((target_config = strsep(&input, ";"))) {
			nt = alloc_param_target(target_config);
			if (IS_ERR(nt)) {
				err = PTR_ERR(nt);
				goto fail;
			}
			/* Dump existing printks when we register */
			netconsole.flags |= CON_PRINTBUFFER;

			spin_lock_irqsave(&target_list_lock, flags);
			list_add(&nt->list, &target_list);
			spin_unlock_irqrestore(&target_list_lock, flags);
		}
	}

	err = register_netdevice_notifier(&netconsole_netdev_notifier);
	if (err)
		goto fail;

	err = dynamic_netconsole_init();
	if (err)
		goto undonotifier;

	register_console(&netconsole);
	printk(KERN_INFO "netconsole: network logging started\n");

	return err;

undonotifier:
	unregister_netdevice_notifier(&netconsole_netdev_notifier);

fail:
	printk(KERN_ERR "netconsole: cleaning up\n");

	/*
	 * Remove all targets and destroy them (only targets created
	 * from the boot/module option exist here). Skipping the list
	 * lock is safe here, and netpoll_cleanup() will sleep.
	 */
	list_for_each_entry_safe(nt, tmp, &target_list, list) {
		list_del(&nt->list);
		free_param_target(nt);
	}

	return err;
}

static void __exit cleanup_netconsole(void)
{
	struct netconsole_target *nt, *tmp;

	unregister_console(&netconsole);
	dynamic_netconsole_exit();
	unregister_netdevice_notifier(&netconsole_netdev_notifier);

	/*
	 * Targets created via configfs pin references on our module
	 * and would first be rmdir(2)'ed from userspace. We reach
	 * here only when they are already destroyed, and only those
	 * created from the boot/module option are left, so remove and
	 * destroy them. Skipping the list lock is safe here, and
	 * netpoll_cleanup() will sleep.
	 */
	list_for_each_entry_safe(nt, tmp, &target_list, list) {
		list_del(&nt->list);
		free_param_target(nt);
	}
}

#ifndef	MODULE
late_initcall(init_netconsole);
#else
module_init(init_netconsole);
#endif
module_exit(cleanup_netconsole);
