/*
	ads7830.c - lm_sensors driver for ads7830 8-bit 8-channel ADC
	(C) 2007 EADS Astrium, 2011 Lab X Technologies LLC

	This driver is based on the lm75 and ads7828 drivers

	Written by Scott Wagner <scott.wagner@labxtechnologies.com>, based
	heavily on the ads7828 driver by Steve Hardy <steve@linuxrealtime.co.uk>

	Datasheet available at: http://focus.ti.com/lit/ds/symlink/ads7830.pdf

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>

/* The ADS7830 registers */
#define ADS7830_NCH 8 /* 8 channels of 8-bit A-D supported */
#define ADS7830_CMD_SD_SE 0x80 /* Single ended inputs */
#define ADS7830_CMD_SD_DIFF 0x00 /* Differential inputs */
#define ADS7830_CMD_PD0 0x0 /* Power Down between A-D conversions */
#define ADS7830_CMD_PD1 0x04 /* Internal ref OFF && A-D ON */
#define ADS7830_CMD_PD2 0x08 /* Internal ref ON && A-D OFF */
#define ADS7830_CMD_PD3 0x0C /* Internal ref ON && A-D ON */
#define ADS7830_INT_VREF_MV 2500 /* Internal vref is 2.5V, 2500mV */

/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x48, 0x49, 0x4a, 0x4b,
	I2C_CLIENT_END };

/* Insmod parameters */
I2C_CLIENT_INSMOD_1(ads7830);

/* Other module parameters */
static int se_input = 1; /* Default is SE, 0 == diff */
static int int_vref = 1; /* Default is internal ref ON */
static int vref_mv = ADS7830_INT_VREF_MV; /* set if vref != 2.5V */
module_param(se_input, bool, S_IRUGO);
module_param(int_vref, bool, S_IRUGO);
module_param(vref_mv, int, S_IRUGO);

/* Global Variables */
static u8 ads7830_cmd_byte; /* cmd byte without channel bits */
static unsigned int ads7830_lsb_resol; /* resolution of the ADC sample lsb */

/* Each client has this additional data */
struct ads7830_data {
	struct device *hwmon_dev;
	struct mutex update_lock; /* mutex protect updates */
	char valid; /* !=0 if following fields are valid */
	unsigned long last_updated; /* In jiffies */
	u8 adc_input[ADS7830_NCH]; /* ADS7830_NCH 8-bit samples */
};

/* Function declaration - necessary due to function dependencies */
static int ads7830_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info);
static int ads7830_probe(struct i2c_client *client,
			 const struct i2c_device_id *id);

/* The ADS7830 returns the 8-bit sample in one byte. */
static u8 ads7830_read_value(struct i2c_client *client, u8 reg)
{
	return (i2c_smbus_read_byte_data(client, reg));
}

static inline u8 channel_cmd_byte(int ch)
{
	/* cmd byte C2,C1,C0 - see datasheet */
	u8 cmd = (((ch>>1) | (ch&0x01)<<2)<<4);
	cmd |= ads7830_cmd_byte;
	return cmd;
}

#if 0
/* Update data for the device (all 8 channels) */
static struct ads7830_data *ads7830_update_device(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ads7830_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->update_lock);

	if (time_after(jiffies, data->last_updated + HZ + HZ / 2)
			|| !data->valid) {
		unsigned int ch;
		dev_dbg(&client->dev, "Starting ads7830 update\n");

		for (ch = 0; ch < ADS7830_NCH; ch++) {
			u8 cmd = channel_cmd_byte(ch);
			data->adc_input[ch] = ads7830_read_value(client, cmd);
		}
		data->last_updated = jiffies;
		data->valid = 1;
	}

	mutex_unlock(&data->update_lock);

	return data;
}
#endif

/* sysfs callback function */
static ssize_t show_in(struct device *dev, struct device_attribute *da,
	char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ads7830_data *data = i2c_get_clientdata(client);
	u8 val;
	u8 cmd;

	mutex_lock(&data->update_lock);
	cmd = channel_cmd_byte(attr->index);
	val = ads7830_read_value(client, cmd);
	mutex_unlock(&data->update_lock);

	/* Print value (in mV as specified in sysfs-interface documentation) */
	return sprintf(buf, "%d\n", ((val *	ads7830_lsb_resol) + 512) >> 10);
}

#define in_reg(offset)\
static SENSOR_DEVICE_ATTR(in##offset##_input, S_IRUGO, show_in,\
	NULL, offset)

in_reg(0);
in_reg(1);
in_reg(2);
in_reg(3);
in_reg(4);
in_reg(5);
in_reg(6);
in_reg(7);

static struct attribute *ads7830_attributes[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,
	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in6_input.dev_attr.attr,
	&sensor_dev_attr_in7_input.dev_attr.attr,
	NULL
};

static const struct attribute_group ads7830_group = {
	.attrs = ads7830_attributes,
};

static int ads7830_remove(struct i2c_client *client)
{
	struct ads7830_data *data = i2c_get_clientdata(client);
	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &ads7830_group);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static const struct i2c_device_id ads7830_id[] = {
	{ "ads7830", ads7830 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ads7830_id);

/* This is the driver that will be inserted */
static struct i2c_driver ads7830_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "ads7830",
	},
	.probe = ads7830_probe,
	.remove = ads7830_remove,
	.id_table = ads7830_id,
	.detect = ads7830_detect,
	.address_data = &addr_data,
};

/* Return 0 if detection is successful, -ENODEV otherwise */
static int ads7830_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	/* Check we have a valid client */
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -ENODEV;

	/* No detection. There is no identification dedicated register, so
	nothing we can do except verify reading works.
	*/
	if (kind <= 0) {
		int ch;
		for (ch = 0; ch < ADS7830_NCH; ch++) {
			u8 in_data;
			u8 cmd = channel_cmd_byte(ch);
			in_data = ads7830_read_value(client, cmd);
		}
	}
	strlcpy(info->type, "ads7830", I2C_NAME_SIZE);

	return 0;
}

static int ads7830_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ads7830_data *data;
	int err;

	data = kzalloc(sizeof(struct ads7830_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &ads7830_group);
	if (err)
		goto exit_free;

	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}

	return 0;

exit_remove:
	sysfs_remove_group(&client->dev.kobj, &ads7830_group);
exit_free:
	kfree(data);
exit:
	return err;
}

static int __init sensors_ads7830_init(void)
{
	/* Initialize the command byte according to module parameters */
	ads7830_cmd_byte = se_input ?
		ADS7830_CMD_SD_SE : ADS7830_CMD_SD_DIFF;
	ads7830_cmd_byte |= int_vref ?
		ADS7830_CMD_PD3 : ADS7830_CMD_PD1;

	/* Calculate the LSB resolution */
	ads7830_lsb_resol = (vref_mv*1024)/256;

	return i2c_add_driver(&ads7830_driver);
}

static void __exit sensors_ads7830_exit(void)
{
	i2c_del_driver(&ads7830_driver);
}

MODULE_AUTHOR("Scott Wagner <scott.wagner@labxtechnologies.com>");
MODULE_DESCRIPTION("ADS7830 driver");
MODULE_LICENSE("GPL");

module_init(sensors_ads7830_init);
module_exit(sensors_ads7830_exit);
