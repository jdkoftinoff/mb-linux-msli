/*
 * nor_otp.c - Test driver for NOR OTP functionality
 *
 * Copyright (C) 2011 Lab X Technologies, LLC
 * Peter McLoone <peter.mcloone@labxtechnologies.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/types.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/cfi_otp.h>

/* otpDeviceIdentifier may be either the direct name of the root MTD device (e.g.
 * "87000000.flash", which is platform dependent and unwieldy, or (preferably) the
 * name of a partition on the MTD device hosting the OTP storage.  The name may be
 * a partial name (e.g. "root" will match "rootfs") and is not case sensitive.
 */
static char *otpDeviceIdentifier = "root";
module_param(otpDeviceIdentifier, charp, 0);
MODULE_PARM_DESC(otpDeviceIdentifier, "Partition name identifying MTD device with OTP storage");

#define N_OTP_ADDRESSES 16

/* Our partition node structure */
struct mtd_part {
	struct mtd_info mtd;
	struct mtd_info *master;
	uint64_t offset;
	int index;
	struct list_head list;
	int registered;
};

/*
 * Given a pointer to the MTD object in the mtd_part structure, we can retrieve
 * the pointer to that structure with this macro.
 */
#define PART(x)  ((struct mtd_part *)(x))

#define DRIVER_NAME "nor_otp"
#define DRIVER_VERSION "0.1"

struct nor_otp_struct {
	struct mtd_info *mtd;
	struct mtd_notifier notify;
	unsigned long int address;
	unsigned long long int data[2];
	securityword_t otp[17];
	bool locks[17];
};

static void nor_otp_notify_add(struct mtd_info *mtd);
static void nor_otp_notify_del(struct mtd_info *mtd);
static struct nor_otp_struct nor_otp = {
	.mtd = NULL,
	.notify = {
		.add = nor_otp_notify_add,
		.remove = nor_otp_notify_del
	}
};

DECLARE_WAIT_QUEUE_HEAD(nor_otp_wait);

static void nor_otp_notify_add(struct mtd_info *mtd)
{
	bool partitionMatch;
	if (mtd != NULL && mtd->type != MTD_ABSENT) {
		partitionMatch = (strnicmp(mtd->name, otpDeviceIdentifier, strlen(otpDeviceIdentifier)) == 0);
		if ((mtd = ((struct mtd_part *)mtd)->master) != NULL) {
			if (!partitionMatch) {
				partitionMatch = (strnicmp(mtd->name, otpDeviceIdentifier, strlen(otpDeviceIdentifier)) == 0);
			}
			if (partitionMatch && nor_otp.mtd == NULL) {
				nor_otp.mtd = mtd;
				wake_up_interruptible(&nor_otp_wait);
			}
		}
	}
	return;
}

static void nor_otp_notify_del(struct mtd_info *mtd)
{
	bool partitionMatch;
	if (mtd != NULL && mtd->type != MTD_ABSENT) {
		partitionMatch = (strncmp(mtd->name, otpDeviceIdentifier, strlen(otpDeviceIdentifier)) == 0);
		if ((mtd = ((struct mtd_part *)mtd)->master) != NULL) {
			if (!partitionMatch) {
				partitionMatch = (strncmp(mtd->name, otpDeviceIdentifier, strlen(otpDeviceIdentifier)) == 0);
			}
			if (partitionMatch && nor_otp.mtd == mtd) {
				nor_otp.mtd = NULL;
			}
		}
	}
	return;
}

static struct mtd_info *get_mtd_master(void)
{
	if (nor_otp.mtd == NULL) {
		wait_event_interruptible(nor_otp_wait, (nor_otp.mtd != NULL));
	}
	return nor_otp.mtd;
}

int read_otp_reg(otp_register addr, securityword_t *otp)
{
	struct mtd_info *mtd;
	size_t retlen = 0;
	int i = 0;

	if (addr >= N_OTP_ADDRESSES || (mtd = get_mtd_master()) == NULL) {
		return 0;
	}

	for(i = 0; i < 16; i=i+2) {
		(*(mtd->read_user_prot_reg))(mtd, 2+i+(addr*16), 2,
					       &retlen, ((u_char *)otp)+(i));
	}
	return 1;
}
EXPORT_SYMBOL(read_otp_reg);

static ssize_t otp_r_data_reg(struct class *c, char *buf)
{
	char temp[33];
	char *tp = temp;
	int i;
	securityword_t otp_ret;
	for(i = 0; i < 16; i++) {
		otp_ret[i] = 0;
		tp = pack_hex_byte(tp,otp_ret[i]);
	}
	*tp = '\0';
	tp = temp;

	read_otp_reg(nor_otp.address, &otp_ret);

	for(i = 0; i < 16; i++) {
		tp = pack_hex_byte(tp, otp_ret[i]);
	}
	*tp = '\0';
	tp = temp;

	return (snprintf(buf, PAGE_SIZE, "%s\n", tp));
}

static inline uint8_t h2b(char c)
{
	if(c > '9') {
		c += 9;
    }
	return (c & 0xf);
}

static void hex2bin(uint8_t *dst, const char *src, size_t count)
{
	while (count--) {
		*dst = h2b(*src++) << 4;
		*dst++ = h2b(*src++);
    }
}

static ssize_t otp_w_data_reg(struct class *c, const char * buf, size_t count)
{
	int i = 0;
	u_char inputbuf[16];
	size_t retlen = 0;
	short unsigned int lockIndex;
	u_char * lock;
	securityword_t test;
	char temp[33];
	char *tp = temp;
	struct mtd_info *mtd;
 
	mtd = get_mtd_master();

	if (mtd != 0 && nor_otp.locks[nor_otp.address] == 0) {
		hex2bin(inputbuf, buf, (count-1));
		//write the register
		for(i=0; i<16; i=i+2) {
			(*(mtd->write_user_prot_reg))(mtd,
					2+i+(nor_otp.address*16), 2,
					&retlen, (inputbuf+i));
		}
		//now read the register back and check the contents
		read_otp_reg(nor_otp.address, &test);
		for(i = 0; i < 16; i++) {
			tp = pack_hex_byte(tp, test[i]);
		}
		*tp = '\0';
		tp = temp;
		if(strncasecmp(tp, buf, 32)) {
			printk("OTP driver: Address read from %lu does not match written\n",
					nor_otp.address);
		}

		//lock the register
		lockIndex = 1 << nor_otp.address;
		lockIndex = lockIndex ^ 0xffff;
		lock = (u_char *) (&lockIndex);
		(*(mtd->write_user_prot_reg))(mtd, 0, 2,
					    &retlen,lock); 
      
		//simulated sysfs stuff - used for testing
		//hex2bin(nor_otp.otp[nor_otp.address], buf, (count - 1));
		nor_otp.locks[nor_otp.address] = 1;
	}
	return count;
}

static ssize_t otp_r_pointer_address(struct class *c, char * buf)
{
	return(snprintf(buf, PAGE_SIZE, "%lu\n", nor_otp.address));
}

static ssize_t otp_w_pointer_address(struct class *c, const char * buf, size_t count)
{
	unsigned long int new_address;
	strict_strtoul(buf, 0, &new_address);
	nor_otp.address = new_address;
	if(nor_otp.address > 17) {
		nor_otp.address = 0;
    }
	return count;
}

static struct class_attribute nor_otp_class_attrs[] = {
	__ATTR(data_reg, S_IRUGO | S_IWUGO, otp_r_data_reg, otp_w_data_reg),
	__ATTR(pointer_address, S_IRUGO | S_IWUGO, otp_r_pointer_address, otp_w_pointer_address),
	__ATTR_NULL,
};

static struct class nor_otp_class = {
	.name = DRIVER_NAME,
	.owner = THIS_MODULE,
	.class_attrs = nor_otp_class_attrs,
};

static int __init nor_otp_init(void)
{
	register_mtd_user(&(nor_otp.notify));
	return (class_register(&nor_otp_class));
}

static void __exit nor_otp_exit(void)
{
	unregister_mtd_user(&(nor_otp.notify));
	class_unregister(&nor_otp_class);
	return;
}

MODULE_AUTHOR("Peter McLoone <peter.mcloone@labxtechnologies.com>");
MODULE_DESCRIPTION("Driver for Numonyx Axcell P-35 OTP Registers");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(nor_otp_init);
module_exit(nor_otp_exit);
