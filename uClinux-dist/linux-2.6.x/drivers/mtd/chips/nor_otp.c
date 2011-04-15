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
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/xilinx_devices.h>
#include <asm/io.h>
#include <linux/vmalloc.h>
#include <linux/ctype.h>
#include <linux/types.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/cfi_otp.h>

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
  unsigned long int address;
  unsigned long long int data[2];
  securityword_t otp[17];
  bool locks[17];
  bool firsttime;
};

static struct nor_otp_struct nor_otp;

int read_otp_reg(otp_register addr, securityword_t *otp)
{
  int i = 0;

  if(addr >= 16)
    {
      return 0;
    }
  else 
    {
      size_t retlen = 0;
      if(!nor_otp.firsttime)
	{
	  nor_otp.mtd = NULL;
	  nor_otp.mtd = get_mtd_device(NULL, 0);
	  nor_otp.firsttime = 1;
	  nor_otp.mtd = (PART(nor_otp.mtd)->master);
	}
  
      for(i = 0; i < 16; i=i+2)
	{
	  //printk("Iteration with offset %u\n", 2+i+(addr*16));
	  (*(nor_otp.mtd->read_user_prot_reg))(nor_otp.mtd, 2+i+(addr*16), 2, 
					       &retlen, ((u_char *)otp)+(i));
	}

      return 1;
    }
}
EXPORT_SYMBOL(read_otp_reg);

static ssize_t otp_r_data_reg(struct class *c, char *buf)
{
  char temp[33];
  char *tp = temp;
  int i;
  securityword_t otp_ret;
  for(i = 0; i < 16; i++)
    {
      otp_ret[i] = 0;
      tp = pack_hex_byte(tp,otp_ret[i]);
    }
  *tp = '\0';
  tp = temp;

  read_otp_reg(nor_otp.address, &otp_ret);

  for(i = 0; i < 16; i++)
    {
      tp = pack_hex_byte(tp, otp_ret[i]);
    }
  *tp = '\0';
  tp = temp;
  
  return (snprintf(buf, PAGE_SIZE, "%s\n", tp));
}

static uint8_t hex_to_bin(char ch)
{
  if((ch >= '0') && (ch <= '9'))
    {
      return ch - '0';
    }
  ch = tolower(ch);
  if ((ch >= 'a') && (ch <= 'f'))
    {
      return ch - 'a' + 10;
    }
  return -1;
}

static void hex2bin(uint8_t *dst, const char *src, size_t count)
{
  size_t ncount;

  if(count & 1)
    {
      *dst = hex_to_bin(*src++);
      dst++;
      ncount = (count-1) >> 1;
    }
  else
    {
      ncount = count >> 1;
    }

  while (ncount--) 
    {
      *dst = hex_to_bin(*src++) << 4;
      *dst += hex_to_bin(*src++);
      dst++;
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
  char zero[2] = {'0', '0'};

  if(!nor_otp.firsttime)
    {
      nor_otp.mtd = NULL;
      nor_otp.mtd = get_mtd_device(NULL, 0);
      nor_otp.firsttime = 1;
      nor_otp.mtd = (PART(nor_otp.mtd)->master);
    }
  
  if(!nor_otp.locks[nor_otp.address])
    {
      hex2bin(inputbuf, buf, (count-1));
      //write the register
      for(i=0; i<16; i=i+2)
	{
	  (*(nor_otp.mtd->write_user_prot_reg))(nor_otp.mtd, 
						2+i+(nor_otp.address*16), 2,
						&retlen, (inputbuf+i));
	}
      //now read the register back and check the contents
      read_otp_reg(nor_otp.address, &test);
      for(i = 0; i < 16; i++)
	{
	  tp = pack_hex_byte(tp, test[i]);
	}
      *tp = '\0';
      tp = temp;
      printk("Compare %s vs %s\n", buf, tp);
      if(strncmp(tp, buf, 32))
	{
	  printk("Address read from %u does not match written\n",
		 nor_otp.address);
	  printk("Now attempting to blank out OTP register\n");
	  
	  for(i=0; i<16; i=i+2)
	     {
	       (*(nor_otp.mtd->write_user_prot_reg))(nor_otp.mtd,
						     2+i+(nor_otp.address*16), 2,
						     &retlen, zero); 
	     }
	}

      //lock the register
      lockIndex = 1 << nor_otp.address;
      lockIndex = lockIndex ^ 0xffff;
      lock = (u_char *) (&lockIndex);
      (*(nor_otp.mtd->write_user_prot_reg))(nor_otp.mtd, 0, 2,
					    &retlen,lock); 
      
      //simulated sysfs stuff - used for testing
      //hex2bin(nor_otp.otp[nor_otp.address], buf, (count - 1));
      nor_otp.locks[nor_otp.address] = 1;
      
      return count;
    }
  else
    {
      return count;
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
  if(nor_otp.address > 17)
    {
      nor_otp.address = 0;
      return count;
    }
  else
    {
      return count;
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

static int __devinit nor_otp_probe(void)
{
  int rc;
  int i, j;
  
  nor_otp.firsttime = 0;
  for(i = 0; i < 17; i++)
    {
      for(j = 0; j < 16; j++)
	{
	  nor_otp.otp[i][j] = 0;
	}
    }
  rc = class_register(&nor_otp_class);

  return rc;
}

static int nor_otp_remove(struct device *dev)
{
  memset(&nor_otp, 0, sizeof(nor_otp));
  return 0;
}

static struct device_driver nor_otp_driver = {
  .name = DRIVER_NAME,
  .bus = &platform_bus_type,

  .probe = nor_otp_probe,
  .remove = nor_otp_remove,
};

static int __init nor_otp_init(void)
{
  nor_otp_probe();
  return 0;
}

static void __exit nor_otp_exit(void)
{
  driver_unregister(&nor_otp_driver);
}

MODULE_AUTHOR("Peter McLoone <peter.mcloone@labxtechnologies.com>");
MODULE_DESCRIPTION("Driver for Numonyx Axcell P-35 OTP Registers");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(nor_otp_init);
module_exit(nor_otp_exit);
