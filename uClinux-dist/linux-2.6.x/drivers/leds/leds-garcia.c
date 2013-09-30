/*
 * LEDs driver for Garcia Snake board
 *
 * Copyright (C) 2011 Lab X Technologies LLC
 * Written by Scott Wagner <scott.wagner@labxtechnologies.com>
 *
 * Based on leds-ams-delta.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/garcia_fpga.h>
#include <linux/broadcom_leds.h>

#define DRVNAME "garcia-led"

static void garcia_eth0_act_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	bc_phy_led_set(0, BC_PHY_LED1, value);
}

static struct led_classdev garcia_eth0_act_led = {
	.name		= "eth0_act",
	.brightness_set	= garcia_eth0_act_led_set,
	.flags		= LED_CORE_SUSPENDRESUME,
};

static void garcia_eth0_stat_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	bc_phy_led_set(0, BC_PHY_LED2, value);
}

static struct led_classdev garcia_eth0_stat_led = {
	.name		= "eth0_stat",
	.brightness_set	= garcia_eth0_stat_led_set,
	.flags		= LED_CORE_SUSPENDRESUME,
};

static void garcia_eth1_act_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	bc_phy_led_set(1, BC_PHY_LED1, value);
}

static struct led_classdev garcia_eth1_act_led = {
	.name		= "eth1_act",
	.brightness_set	= garcia_eth1_act_led_set,
	.flags		= LED_CORE_SUSPENDRESUME,
};

static void garcia_eth1_stat_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	bc_phy_led_set(1, BC_PHY_LED2, value);
}

static struct led_classdev garcia_eth1_stat_led = {
	.name		= "eth1_stat",
	.brightness_set	= garcia_eth1_stat_led_set,
	.flags		= LED_CORE_SUSPENDRESUME,
};

static void garcia_eth2_act_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	bc_phy_led_set(2, BC_PHY_LED1, value);
}

static struct led_classdev garcia_eth2_act_led = {
	.name		= "eth2_act",
	.brightness_set	= garcia_eth2_act_led_set,
	.flags		= LED_CORE_SUSPENDRESUME,
};

static void garcia_eth2_stat_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	bc_phy_led_set(2, BC_PHY_LED2, value);
}

static struct led_classdev garcia_eth2_stat_led = {
	.name		= "eth2_stat",
	.brightness_set	= garcia_eth2_stat_led_set,
	.flags		= LED_CORE_SUSPENDRESUME,
};

static void garcia_eth3_act_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	bc_phy_led_set(3, BC_PHY_LED1, value);
}

static struct led_classdev garcia_eth3_act_led = {
	.name		= "eth3_act",
	.brightness_set	= garcia_eth3_act_led_set,
	.flags		= LED_CORE_SUSPENDRESUME,
};

static void garcia_eth3_stat_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	bc_phy_led_set(3, BC_PHY_LED2, value);
}

static struct led_classdev garcia_eth3_stat_led = {
	.name		= "eth3_stat",
	.brightness_set	= garcia_eth3_stat_led_set,
	.flags		= LED_CORE_SUSPENDRESUME,
};

static void garcia_power_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	garcia_led_set(POWER_LED, value);
	led_cdev->brightness = value;
}

static struct led_classdev garcia_power_led = {
	.name		= "power",
	.brightness_set	= garcia_power_led_set,
	.max_brightness = 3,
	.brightness = 2,
	.flags		= LED_CORE_SUSPENDRESUME,
	.default_trigger = "timer"
};

static struct timer_list timer;

static void garcia_status_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	garcia_led_set(STATUS_LED, value);
	led_cdev->brightness = value;
	del_timer_sync(&timer);
}


static struct led_classdev garcia_status_led = {
	.name		= "status",
	.brightness_set	= garcia_status_led_set,
	.max_brightness = 3,
	.brightness = 3,
	.flags		= LED_CORE_SUSPENDRESUME,
	.default_trigger = "timer"
};

static void led_init_failed_set(unsigned long data)
{
	struct timer_list *tp = (struct timer_list *)data;
	garcia_led_set(STATUS_LED, 1);
	printk("Garcia LEDs: init failed\n");
	del_timer_sync(tp);
	return;
}

static int __init garcia_led_init(void)
{
	int status;
	init_timer(&timer);
	timer.function = led_init_failed_set;
	timer.data = (long int)&timer;
	mod_timer(&timer, jiffies + 30*HZ);

	status = led_classdev_register(0, &garcia_eth0_act_led);
	status |= led_classdev_register(0, &garcia_eth0_stat_led);
	status |= led_classdev_register(0, &garcia_eth1_act_led);
	status |= led_classdev_register(0, &garcia_eth1_stat_led);
	status |= led_classdev_register(0, &garcia_eth2_act_led);
	status |= led_classdev_register(0, &garcia_eth2_stat_led);
	status |= led_classdev_register(0, &garcia_eth3_act_led);
	status |= led_classdev_register(0, &garcia_eth3_stat_led);
	status |= led_classdev_register(0, &garcia_power_led);
	garcia_power_led_set(&garcia_power_led, garcia_power_led.brightness);
	status |= led_classdev_register(0, &garcia_status_led);
	garcia_status_led_set(&garcia_status_led, garcia_status_led.brightness);
	printk("Registered Garcia LEDs status %d\n", status);
	return status;
}

static void __exit garcia_led_exit(void)
{
	led_classdev_unregister(&garcia_eth0_act_led);
	led_classdev_unregister(&garcia_eth0_stat_led);
	led_classdev_unregister(&garcia_eth1_act_led);
	led_classdev_unregister(&garcia_eth1_stat_led);
	led_classdev_unregister(&garcia_eth2_act_led);
	led_classdev_unregister(&garcia_eth2_stat_led);
	led_classdev_unregister(&garcia_eth3_act_led);
	led_classdev_unregister(&garcia_eth3_stat_led);
	led_classdev_unregister(&garcia_power_led);
	led_classdev_unregister(&garcia_status_led);
}

module_init(garcia_led_init);
module_exit(garcia_led_exit);

MODULE_AUTHOR("Scott Wagner <scott.wagner@labxtechnologies.com>");
MODULE_DESCRIPTION("Garcia LED driver");
MODULE_LICENSE("GPL");

