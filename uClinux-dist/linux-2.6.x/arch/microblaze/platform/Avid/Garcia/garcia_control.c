/*
 * garcia_control.c -- Control device for Avid Garcia sound cards
 *
 * Copyright (C) 2011 Lab X Technologies, LLC
 *	Scott Wagner <scott.wagner@labxtechnologies.com>
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>

#include <linux/garcia_fpga.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF

/* Driver has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/agctlB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular device.
 */
#define AGCTL_MAJOR			101	/* We assume no Motorola DSP 56xxx board */
#define N_AGCTL_CHANNELS	5
#define DRIVER_NAME			"agctl"
#define DRIVER_VERSION		"0.9"
#define AC_BUFSIZE			32		/* Size of shift register buffer */
#define N_SHIFT_REGISTER_BYTES 18
//#define AGC_DEBUG_REGIO

struct agctl_data {
	dev_t				devt;
	wait_queue_head_t	wait;
	struct class		agclass;
	struct fasync_struct *async_queue; /* asynchronous readers */
	struct agctl_master *agm; /* Pointer back to master agctl structure */

	/* buffer is NULL unless this device is open (users > 0) */
	uint32_t			saved_status;
	uint16_t			users;
	uint16_t			transfer_size;
	uint16_t			irqflags;
	uint16_t			strobedelay;
	uint8_t				ctlreg_offs;
	uint8_t				flags;           // Slot number and master/slave status
	__be32				rxbuf[AC_BUFSIZE>>2]; // Note: guaranteed big-endian
	__be32				txbuf[AC_BUFSIZE>>2]; // Note: guaranteed big-endian
};
#define AGCTL_FLAG_SLOTMASK 7
#define agctl_slot_number(d) ((d)->flags & AGCTL_FLAG_SLOTMASK) // Get slot number
#define AGCTL_FLAG_MASTER BIT(3)
#define agctl_is_master(d) (((d)->flags & AGCTL_FLAG_MASTER) != 0) // Test for driver is master

struct agctl_master {
	void __iomem		*regs;	/* virt. address of the control registers */
	uint32_t			irq;
	struct mutex		agmutex;
	spinlock_t			aglock;
	struct agctl_data	chan[N_AGCTL_CHANNELS];
};

static struct agctl_master *agm_stat = NULL;

static LIST_HEAD(device_list);

#define AC_SR_16_19_OFFSET	0x00	/* 16-bit Shift Register 17-18 (s:RO, m:R/W) */
#define AC_SR_12_15_OFFSET	0x04	/* 32-bit Shift Registers 16 - 13 (s:RO, m:R/W) */
#define AC_SR_8_11_OFFSET	0x08	/* 32-bit Shift Registers 12 - 9 (s:RO, m:R/W) */
#define AC_SR_4_7_OFFSET	0x0C	/* 32-bit Shift Registers 8 - 5 (s:RO, m:R/W) */
#define AC_SR_0_3_OFFSET	0x10	/* 32-bit Shift Registers 4 - 1 (s:RO, m:R/W) */
#define AC_CLK_COUNT_OFFSET	0x14	/* 8-bit Clock Count Register (s:RO, m:R/W) */
#define AC_CONTROL0_OFFSET	0x20	/* 6-bit Status/Control Register */
#define AC_CONTROL1_OFFSET	0x24	/* 6-bit Status/Control Register */
#define AC_CONTROL2_OFFSET	0x28	/* 6-bit Status/Control Register */
#define AC_CONTROL3_OFFSET	0x2C	/* 6-bit Status/Control Register */
#define AC_CONTROL4_OFFSET	0x30	/* 6-bit Status/Control Register */

#define AC_CTL_IDREG_MASK	0xFF000000	/* 8-bit ID Register (Informational only for Master) (R/W) */
#define AC_CTL_MUTESTREAM_MASK 0x00F00000 /* 4 bit mute stream select mask */
#define AC_CTL_DIAG_ERROR   BIT(19) /* Debug error flag (used e.g. for LFSR) (w1c) */
#define AC_CTL_DIAG_OFFSET	BIT(16) /* Diagnostic bits 18:16 */
#define AC_CTL_DIAG_MASK	0x30000 /* Diagnostic bits 18:16 mask */
#define AC_CTL_ENA_UNMUTE   BIT(15) /* If set, and AC_CTL_MUTE_FORCE is clear, force unmute */
#define AC_CTL_MUTE_FORCE   BIT(14) /* If set, slot is forced to be muted */
#define AC_CTL_SERDES_SYNC	BIT(13)	/* SERDES/buffers are synced (RO) */
#define AC_CTL_LRCLK_ACTIVE	BIT(12)	/* LRCLK is present (RO) */
#define AC_CTL_LRCLK_MASTER	BIT(11)	/* This slot is providing the master LRCLK for the AVB subsystem (RO) */
#define AC_CTL_BUF_COL_IRQ	BIT(10)	/* IRQ: Buffer collision (w1c) */
#define AC_CTL_POPULATED	BIT(9)	/* An I/O card is proxied by this slot (slave only) */
#define AC_CTL_INT_ENA		BIT(8)	/* Interrupt enabled */
#define AC_CTL_RESET_IRQ	BIT(7)	/* IRQ: Reset signal has changed state (slave only, w1c) */
#define AC_CTL_MUTE_IRQ 	BIT(6)	/* IRQ: Mute signal has changed state (slave only, w1c) */
#define AC_CTL_STROBE_IRQ	BIT(5)	/* IRQ: Strobe rising edge condition (slave only, w1c) */
#define AC_CTL_TX_COMPL_IRQ	BIT(4)	/* IRQ: Transaction complete condition (master only, w1c) */
#define AC_CTL_RESET_SIG	BIT(3)	/* Reset signal is asserted (s:in, m:out) */
#define AC_CTL_MUTE_SIG 	BIT(2)	/* Mute signal is asserted (s:in, m:out) */
#define AC_CTL_STROBE		BIT(1)	/* Strobe signal is asserted (s:in, m:out) */
#define AC_CTL_SSI_DDIR		BIT(0)	/* Data direction of SSI (R/W) */
#define AC_CTL_IRQ_MASK		(AC_CTL_BUF_COL_IRQ | AC_CTL_RESET_IRQ | AC_CTL_MUTE_IRQ | \
							AC_CTL_STROBE_IRQ | AC_CTL_TX_COMPL_IRQ | AC_CTL_DIAG_ERROR)
#define AC_CLK_COUNT_MASK   0xFF    /* Clock bit counter */
#define AC_CLK_OVF_MASK     0x80000000 /* A strobe (transaction latch) has occurred before the
                                          previous transaction was serviced */

inline void agc_regw_be(struct agctl_master *agm, uint32_t offs, __be32 val)
{
	out_be32(agm->regs + offs, val);
	ndelay(100);
	#ifdef AGC_DEBUG_REGIO
	printk("%lx=>%p  ", val, agm->regs + offs);
	#endif
}

inline void agc_regw(struct agctl_master *agm, uint32_t offs, uint32_t val)
{
	agc_regw_be(agm, offs, cpu_to_be32(val));
}

inline __be32 agc_regr_be(struct agctl_master *agm, uint32_t offs)
{
	__be32 val = in_be32(agm->regs + offs);
	#ifdef AGC_DEBUG_REGIO
	printk("%lx<=%p  ", val, agm->regs + offs);
	#endif
	return val;
}

inline uint32_t agc_regr(struct agctl_master *agm, uint32_t offs)
{
	return be32_to_cpu(agc_regr_be(agm, offs));
}

/*-------------------------------------------------------------------------*/

static void agdev_strobe(struct agctl_master *agm, struct agctl_data *agctl)
{
	uint32_t status;
	unsigned long flags;
	spin_lock_irqsave(&agm->aglock, flags);
	status = agc_regr(agm, agctl->ctlreg_offs) & ~AC_CTL_IRQ_MASK;
	if (agctl_is_master(agctl)) {
		agc_regw(agm, agctl->ctlreg_offs, status | AC_CTL_STROBE);
		udelay(1);
		agc_regw(agm, agctl->ctlreg_offs, status & ~AC_CTL_STROBE);
	}
	spin_unlock_irqrestore(&agm->aglock, flags);
	return;
}

irqreturn_t agc_irq_callback(int irqnum, void *data)
{
	struct agctl_master *agm = (struct agctl_master *)data;
	int chan;
	int do_wake;
	uint32_t clkcount;
	for (chan = 0; chan < N_AGCTL_CHANNELS; chan++) {
		struct agctl_data *agctl = &agm->chan[chan];
		uint32_t status = agc_regr(agm, agctl->ctlreg_offs);
		do_wake = 0;
		if ((status & (AC_CTL_RESET_IRQ | AC_CTL_MUTE_IRQ)) != 0) {
			do_wake = 1;
			agctl->irqflags |= ((uint16_t)(status & (AC_CTL_RESET_IRQ | AC_CTL_MUTE_IRQ))); // | AC_CTL_BUF_COL_IRQ)));
		}
		/* Be careful to preserve big-endianness of receive buffers */
		/* After this transfer, the byte array representing the shift
		 * register contents will be in rxbuf, with the most recent
		 * (newest) bit as the LSB of the byte at offset
		 * <N_SHIFT_REGISTER_BYTES - transfer_size_bytes>, and increasing
		 * memory addresses representing older (shifted in first) data.
		 * Byte [N_SHIFT_REGISTER_BYTES-1] will contain the slave's
		 * first byte transmitted, which will be the ID byte.
		 */
		if ((status & (AC_CTL_STROBE_IRQ | AC_CTL_TX_COMPL_IRQ)) != 0) {
			if (!agctl_is_master(agctl)) {
				clkcount = agc_regr(agm, AC_CLK_COUNT_OFFSET);
				/* Ignore transfers with overflow, transfers in reset,
				 *  and transfers of 0 or 1 bytes
				 */
				if ((clkcount & AC_CLK_OVF_MASK) != 0) {
					; // printk("Channel %d Control message write overflow\n", chan);
				} else if ((status & AC_CTL_RESET_SIG) == 0 && clkcount > 15) {
					agctl->rxbuf[0] = agc_regr_be(agm, AC_SR_0_3_OFFSET);
					agctl->rxbuf[1] = agc_regr_be(agm, AC_SR_4_7_OFFSET);
					agctl->rxbuf[2] = agc_regr_be(agm, AC_SR_8_11_OFFSET);
					agctl->rxbuf[3] = agc_regr_be(agm, AC_SR_12_15_OFFSET);
					agctl->rxbuf[4] = agc_regr_be(agm, AC_SR_16_19_OFFSET);
					agctl->transfer_size = clkcount;
					agctl->irqflags |= ((uint16_t)(status & AC_CTL_STROBE_IRQ));
					do_wake = 1;
				}
			} else {
				uint8_t cardId;
				agctl->rxbuf[0] = agc_regr_be(agm, AC_SR_0_3_OFFSET);
				agctl->rxbuf[1] = agc_regr_be(agm, AC_SR_4_7_OFFSET);
				agctl->rxbuf[2] = agc_regr_be(agm, AC_SR_8_11_OFFSET);
				agctl->rxbuf[3] = agc_regr_be(agm, AC_SR_12_15_OFFSET);
				agctl->rxbuf[4] = agc_regr_be(agm, AC_SR_16_19_OFFSET);
				cardId = *(((uint8_t *)(agctl->rxbuf)) +
						((N_SHIFT_REGISTER_BYTES-1) - (agctl->transfer_size >> 3)));
				status = (status & ~AC_CTL_IDREG_MASK) |
					(((uint32_t)cardId << 24 ) & AC_CTL_IDREG_MASK);
				agctl->irqflags |= ((uint16_t)(status & AC_CTL_TX_COMPL_IRQ));
				do_wake = 1;
			}
		}
		agc_regw(agm, agctl->ctlreg_offs, status);	// Clear interrupt
		if (do_wake) {
			agctl->saved_status = status;
			if (agctl->async_queue) {
				kill_fasync(&agctl->async_queue, SIGIO, POLL_IN);
			}
			/* signal asynchronous readers */
			wake_up_interruptible(&agctl->wait);
		}
	}
	return IRQ_HANDLED;
}

static int agctl_open(struct inode *inode, struct file *filp)
{
	unsigned int		minor = iminor(inode);
	int					status;
	struct agctl_master *agm = agm_stat;
	struct agctl_data *agctl = &agm->chan[minor];

	if (minor >= N_AGCTL_CHANNELS) {
		return -ENXIO;
	}

	lock_kernel();

	mutex_lock(&agctl->agm->agmutex);
	if (agctl->users == 0) {
	  agc_regw(agm, agctl->ctlreg_offs, AC_CTL_BUF_COL_IRQ | AC_CTL_INT_ENA | AC_CTL_RESET_IRQ |
			AC_CTL_MUTE_IRQ | AC_CTL_STROBE_IRQ | AC_CTL_TX_COMPL_IRQ);
	  agctl->irqflags = 0;
	}
	agctl->users++;
	filp->private_data = agctl;
	status = nonseekable_open(inode, filp);
	mutex_unlock(&agctl->agm->agmutex);

	unlock_kernel();
	return status;
}

//#define __user
/* Read-only message with current device setup */
static ssize_t agctl_read(struct file *filp, char __user *buf,
		size_t len, loff_t *f_pos)
{
	struct agctl_data *agctl = filp->private_data;
	struct agctl_master *agm;
	int count;
	bool master;
	uint8_t *regbuf_p;
	unsigned long flags;
	__u32 status;
	if (agctl == NULL) {
		return -ESHUTDOWN;
	}
	agm = agctl->agm;

	mutex_lock(&agm->agmutex);
	spin_lock_irqsave(&agm->aglock, flags);
	status = agc_regr(agm, agctl->ctlreg_offs);
	master = agctl_is_master(agctl);
	agc_regw(agm, agctl->ctlreg_offs,
			(status | AC_CTL_INT_ENA)  & ~AC_CTL_IRQ_MASK);
	while (!master && (agctl->irqflags & AC_CTL_STROBE_IRQ) == 0) {
		spin_unlock_irqrestore(&agm->aglock, flags);
		mutex_unlock(&agm->agmutex);
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}
		if (wait_event_interruptible(agctl->wait,
				((agctl->irqflags & AC_CTL_STROBE_IRQ) != 0))) {
			return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
		}
		mutex_lock(&agm->agmutex);
		spin_lock_irqsave(&agm->aglock, flags);
	}
	agctl->irqflags &= ~AC_CTL_STROBE_IRQ;

	if ((agctl->transfer_size & 7) != (master ? 7 : 0)) {
		printk(KERN_WARNING "Chan %ld, read transfer_size %u\n", agctl - agm->chan, agctl->transfer_size);
	}
	count = min(((agctl->transfer_size + 7) >> 3), N_SHIFT_REGISTER_BYTES);
	count = min(count, (int)len);
	regbuf_p = (uint8_t *)(agctl->rxbuf) + (N_SHIFT_REGISTER_BYTES - count);
	if (copy_to_user(buf, regbuf_p, count) != 0) {
		count = -EFAULT;
	}
	spin_unlock_irqrestore(&agm->aglock, flags);
	mutex_unlock(&agm->agmutex);
	return count;
}

/* Write-only message with current device setup */
static ssize_t agctl_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct agctl_data *agctl = filp->private_data;
	struct agctl_master *agm;
	bool master;
	unsigned long flags;
	__u32 countreg;
	__u32 status;

	if (agctl == NULL) {
		return -ESHUTDOWN;
	}
	agm = agctl->agm;
	master = agctl_is_master(agctl);
	if ((master && count > N_SHIFT_REGISTER_BYTES) ||
			(!master && count > 1) || count <= 0) {
		return -EMSGSIZE;
	}
	if (copy_from_user(agctl->txbuf, buf, count) != 0) {
		return -EFAULT;
	}

	mutex_lock(&agm->agmutex);
	if (master) { // master - start transfer
		agctl->transfer_size = (count << 3) - 1; // Count must be (nbits - 1)
		countreg = agctl->transfer_size | ((uint32_t)(agctl_slot_number(agctl)) << 24);
		spin_lock_irqsave(&agm->aglock, flags);
		agc_regw_be(agm, AC_SR_0_3_OFFSET, agctl->txbuf[0]);
		agc_regw_be(agm, AC_SR_4_7_OFFSET, agctl->txbuf[1]);
		agc_regw_be(agm, AC_SR_8_11_OFFSET, agctl->txbuf[2]);
		agc_regw_be(agm, AC_SR_12_15_OFFSET, agctl->txbuf[3]);
		agc_regw_be(agm, AC_SR_16_19_OFFSET, agctl->txbuf[4]);
		agc_regw(agm, agctl->ctlreg_offs,
				(agc_regr(agm, agctl->ctlreg_offs) | AC_CTL_INT_ENA)  & ~AC_CTL_IRQ_MASK);
		agc_regw(agm, AC_CLK_COUNT_OFFSET, countreg);
		while ((agctl->irqflags & AC_CTL_TX_COMPL_IRQ) == 0) {
			spin_unlock_irqrestore(&agm->aglock, flags);
			mutex_unlock(&agm->agmutex);
			if (filp->f_flags & O_NONBLOCK) {
				return -EAGAIN;
			}
			if (wait_event_interruptible(agctl->wait,
					((agctl->irqflags & AC_CTL_TX_COMPL_IRQ) != 0))) {
				return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
			}
			mutex_lock(&agm->agmutex);
			spin_lock_irqsave(&agm->aglock, flags);
		}
		agctl->irqflags &= ~AC_CTL_TX_COMPL_IRQ;
		spin_unlock_irqrestore(&agm->aglock, flags);
		if (agctl->strobedelay > 0) {
			mdelay(agctl->strobedelay);
		}
		agdev_strobe(agm, agctl);
	} else {
		spin_lock_irqsave(&agm->aglock, flags);
		status = agc_regr(agm, agctl->ctlreg_offs);
		status = (status & ~(AC_CTL_IDREG_MASK | AC_CTL_IRQ_MASK)) |
				(((__u32)(*(uint8_t *)agctl->txbuf) << 24) & AC_CTL_IDREG_MASK);
		agc_regw(agm, agctl->ctlreg_offs, status);
		spin_unlock_irqrestore(&agm->aglock, flags);
	}
	mutex_unlock(&agm->agmutex);
	return count;
}

static int agctl_ioctl(struct inode *ino, struct file *filp, unsigned int cmd, unsigned long int arg)
{
	struct agctl_data *agctl = filp->private_data;
	struct agctl_master *agm;
	__u32 val = 0;
	__u32 status;
	__u32 __user *p32 = (__u32 __user *)arg;
	unsigned long flags;
	int rc = 0;

	(void)ino;
	if (!arg)
		return -EINVAL;

	if (agctl == NULL) {
		return -ESHUTDOWN;
	}
	agm = agctl->agm;
	switch (cmd) {
	case GARCIA_IOC_READ_STATUS:
		mutex_lock(&agm->agmutex);
		while ((agctl->irqflags & (AC_CTL_MUTE_IRQ | AC_CTL_RESET_IRQ | AC_CTL_BUF_COL_IRQ)) == 0 &&
				!agctl_is_master(agctl)) {
			mutex_unlock(&agm->agmutex);
			if (filp->f_flags & O_NONBLOCK) {
				rc = -EAGAIN;
				break;
			}
			if (wait_event_interruptible(agctl->wait, ((agctl->irqflags &
					(AC_CTL_MUTE_IRQ | AC_CTL_RESET_IRQ | AC_CTL_BUF_COL_IRQ)) != 0))) {
				rc = -ERESTARTSYS; /* signal: tell the fs layer to handle it */
			}
			mutex_lock(&agm->agmutex);
		}
		mutex_unlock(&agm->agmutex);
		if (rc != 0) {
			break;
		}
		// Fall through
	case GARCIA_IOC_READ_STATUS_NB:
		status = agc_regr(agm, agctl->ctlreg_offs);
		if ((status & AC_CTL_SERDES_SYNC) != 0) {
			val |= GARCIA_STATUS_SERDES_SYNC;
		}
		if ((status & AC_CTL_LRCLK_ACTIVE) != 0) {
			val |= GARCIA_STATUS_LRCLK_ACTIVE;
		}
		if ((status & AC_CTL_LRCLK_MASTER) != 0) {
			val |= GARCIA_STATUS_LRCLK_MASTER;
		}
		if (agctl_is_master(agctl)) {
			val |= GARCIA_STATUS_MASTER_MODE;
			if ((agctl->saved_status & AC_CTL_BUF_COL_IRQ) != 0) {
				val |= GARCIA_STATUS_BUFFER_COLL;
			}
			if ((agctl->saved_status & AC_CTL_RESET_SIG) != 0) {
				val |= GARCIA_STATUS_RESET_SIG;
			}
			if ((agctl->saved_status & AC_CTL_MUTE_SIG) != 0) {
				val |= GARCIA_STATUS_MUTE_SIG;
			}
		} else {
			if ((status & AC_CTL_BUF_COL_IRQ) != 0) {
				val |= GARCIA_STATUS_BUFFER_COLL;
			}
			if ((status & AC_CTL_RESET_SIG) != 0) {
				val |= GARCIA_STATUS_RESET_SIG;
			}
			if ((status & AC_CTL_MUTE_SIG) != 0) {
				val |= GARCIA_STATUS_MUTE_SIG;
			}
		}
		if ((status & AC_CTL_INT_ENA) != 0) {
			val |= GARCIA_STATUS_INT_ENA;
		}
		if ((status & AC_CTL_STROBE) != 0) {
			val |= GARCIA_STATUS_STROBE;
		}
		if ((status & AC_CTL_SSI_DDIR) != 0) {
			val |= GARCIA_STATUS_SSI_DDIR;
		}
		if ((status & AC_CTL_MUTE_FORCE) == 0 &&
				(status & AC_CTL_ENA_UNMUTE) != 0) {
			val |= GARCIA_STATUS_ENA_UNMUTE;
		} else if ((status & AC_CTL_MUTE_FORCE) != 0) {
			val |= GARCIA_STATUS_MUTE_FORCE;
		} else if ((status & AC_CTL_MUTE_FORCE) == 0 &&
				(status & AC_CTL_ENA_UNMUTE) == 0) {
			val |= (status & GARCIA_MUTE_CONTROLLER_MASK);
		}
		if ((status & AC_CTL_POPULATED) != 0) {
			val |= GARCIA_STATUS_SLOT_POPULATED;
		}

		if (put_user(val, p32)) {
			rc = -EFAULT;
		} else {
			agctl->irqflags &= ~(AC_CTL_MUTE_IRQ | AC_CTL_RESET_IRQ | AC_CTL_BUF_COL_IRQ);
		}
		break;
	case GARCIA_IOC_WRITE_STATUS:
		get_user(val, p32);
		spin_lock_irqsave(&agm->aglock, flags);
		status = agc_regr(agm, agctl->ctlreg_offs) & ~AC_CTL_IRQ_MASK;
		if ((val & GARCIA_STATUS_RESET_SIG) == 0) {
			status &= ~AC_CTL_RESET_SIG;
		} else {
			status |= AC_CTL_RESET_SIG;
		}
		if ((val & GARCIA_STATUS_MUTE_SIG) == 0) {
			status &= ~AC_CTL_MUTE_SIG;
		} else {
			status |= AC_CTL_MUTE_SIG;
		}
		if ((val & GARCIA_STATUS_SSI_DDIR) == 0) {
			status &= ~AC_CTL_SSI_DDIR;
		} else {
			status |= AC_CTL_SSI_DDIR;
		}
		if ((val & GARCIA_STATUS_MUTE_FORCE) == 0 &&
				(val & GARCIA_STATUS_ENA_UNMUTE) != 0) {
			status &= ~(AC_CTL_MUTE_FORCE);
			status |= AC_CTL_ENA_UNMUTE;
		} else if ((val & GARCIA_STATUS_MUTE_FORCE) != 0 &&
				(val & GARCIA_STATUS_ENA_UNMUTE) == 0) {
			status |= AC_CTL_MUTE_FORCE;
			status &= ~AC_CTL_ENA_UNMUTE;
		} else if ((val & GARCIA_STATUS_MUTE_FORCE) != 0 &&
				(val & GARCIA_STATUS_ENA_UNMUTE) != 0) {
			status &= ~AC_CTL_MUTE_FORCE;
			status &= ~AC_CTL_ENA_UNMUTE;
			status |= (val & GARCIA_MUTE_CONTROLLER_MASK);
		}
		if ((val & GARCIA_STATUS_MASK_SLOT_POPULATED) != 0) {
			if ((val & GARCIA_STATUS_SLOT_POPULATED) != 0) {
				status |= AC_CTL_POPULATED;
			} else {
				status &= ~AC_CTL_POPULATED;
			}
		}
		agc_regw(agm, agctl->ctlreg_offs, status);
		spin_unlock_irqrestore(&agm->aglock, flags);
		if ((val & GARCIA_STATUS_STROBE) != 0) {
			agdev_strobe(agm, agctl);
		}
		break;
	default:
		rc = -ENOTTY;
	}
	return rc;
}

static unsigned int agctl_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct agctl_data	*agctl = filp->private_data;
	unsigned int mask = 0;

	if (agctl == NULL) {
		return -ESHUTDOWN;
	}

	if (agctl_is_master(agctl)) {
		mask = POLLIN;
	} else {
		poll_wait(filp, &agctl->wait, wait);
		if ((agctl->irqflags & (AC_CTL_STROBE_IRQ | AC_CTL_TX_COMPL_IRQ)) != 0) {
			mask = POLLIN;	/* readable */
		}
		if ((agctl->irqflags & (AC_CTL_MUTE_IRQ | AC_CTL_RESET_IRQ | AC_CTL_BUF_COL_IRQ)) != 0) {
			mask = POLLPRI;	/* ioctl-able */
		}
	}
	return mask;
}

static int agctl_fasync(int fd, struct file *filp, int mode)
{
	struct agctl_data	*agctl = filp->private_data;
	return fasync_helper(fd, filp, mode, &agctl->async_queue);
}

static int agctl_release(struct inode *inode, struct file *filp)
{
	struct agctl_data	*agctl = filp->private_data;
	int			status = 0;

	if (agctl == NULL) {
		return -ESHUTDOWN;
	}
	mutex_lock(&agctl->agm->agmutex);
	filp->private_data = NULL;

	/* last close? */
	agctl->users--;
	mutex_unlock(&agctl->agm->agmutex);

	return status;
}

static struct file_operations agctl_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.open =		agctl_open,
	.write =	agctl_write,
	.read =		agctl_read,
	.ioctl =	agctl_ioctl,
	.poll =     agctl_poll,
	.fasync =	agctl_fasync,
	.release =	agctl_release,
};

/*-------------------------------------------------------------------------*/
static ssize_t agdev_w_ssi_ddir(struct class *class, const char *buf, size_t count)
{
	uint32_t status;
	long int val;
	unsigned long flags;
	struct agctl_data *agctl = container_of(class, struct agctl_data, agclass);
	if (strict_strtol(buf, 0, &val) != 0) {
		return -EINVAL;
	}
	if (agctl != NULL) {
		struct agctl_master *agm = agctl->agm;
		spin_lock_irqsave(&agm->aglock, flags);
		status = agc_regr(agm, agctl->ctlreg_offs) & ~AC_CTL_IRQ_MASK;
		if (val == 0) {
			status &= ~AC_CTL_SSI_DDIR;
		} else {
			status |= AC_CTL_SSI_DDIR;
		}
		agc_regw(agm, agctl->ctlreg_offs, status);
		spin_unlock_irqrestore(&agm->aglock, flags);
	}
	return count;
}

static ssize_t agdev_r_ssi_ddir(struct class *class, char *buf)
{
	int ddir = 0;
	struct agctl_data *agctl = container_of(class, struct agctl_data, agclass);
	if (agctl != NULL) {
		struct agctl_master *agm = agctl->agm;
		ddir = ((agc_regr(agm, agctl->ctlreg_offs) & AC_CTL_SSI_DDIR) != 0);
	}
	return (snprintf(buf, PAGE_SIZE, "%d\n", ddir));
}

static CLASS_ATTR(ssi_ddir, S_IRUGO | S_IWUSR, agdev_r_ssi_ddir, agdev_w_ssi_ddir);

static ssize_t agdev_w_reset(struct class *class, const char *buf, size_t count)
{
	uint32_t status;
	long int val;
	unsigned long flags;
	struct agctl_data *agctl = container_of(class, struct agctl_data, agclass);
	if (strict_strtol(buf, 0, &val) != 0) {
		return -EINVAL;
	}
	if (agctl != NULL) {
		struct agctl_master *agm = agctl->agm;
		spin_lock_irqsave(&agm->aglock, flags);
		status = agc_regr(agm, agctl->ctlreg_offs) & ~AC_CTL_IRQ_MASK;
		if (val == 0) {
			status &= ~AC_CTL_RESET_SIG;
		} else {
			status |= AC_CTL_RESET_SIG;
		}
		if (agctl_is_master(agctl)) {
			agc_regw(agm, agctl->ctlreg_offs, status);
		}
		spin_unlock_irqrestore(&agm->aglock, flags);
	}
	return count;
}

static ssize_t agdev_r_reset(struct class *class, char *buf)
{
	int rst = 0;
	struct agctl_data *agctl = container_of(class, struct agctl_data, agclass);
	if (agctl != NULL) {
		struct agctl_master *agm = agctl->agm;
		rst = ((agc_regr(agm, agctl->ctlreg_offs) & AC_CTL_RESET_SIG) != 0);
	}
	return (snprintf(buf, PAGE_SIZE, "%d\n", rst));
}

static CLASS_ATTR(reset, S_IRUGO | S_IWUSR, agdev_r_reset, agdev_w_reset);

static ssize_t agdev_w_mute(struct class *class, const char *buf, size_t count)
{
	uint32_t status;
	long int val;
	unsigned long flags;
	struct agctl_data *agctl = container_of(class, struct agctl_data, agclass);
	if (strict_strtol(buf, 0, &val) != 0) {
		return -EINVAL;
	}
	if (agctl != NULL) {
		struct agctl_master *agm = agctl->agm;
		spin_lock_irqsave(&agm->aglock, flags);
		status = agc_regr(agm, agctl->ctlreg_offs) & ~AC_CTL_IRQ_MASK;
		if (val == 0) {
			status &= ~AC_CTL_MUTE_SIG;
		} else {
			status |= AC_CTL_MUTE_SIG;
		}
		if (agctl_is_master(agctl)) {
			agc_regw(agm, agctl->ctlreg_offs, status);
		}
		spin_unlock_irqrestore(&agm->aglock, flags);
	}
	return count;
}

static ssize_t agdev_r_mute(struct class *class, char *buf)
{
	int mute = 0;
	struct agctl_data *agctl = container_of(class, struct agctl_data, agclass);
	if (agctl != NULL) {
		struct agctl_master *agm = agctl->agm;
		mute = ((agc_regr(agm, agctl->ctlreg_offs) & AC_CTL_MUTE_SIG) != 0);
	}
	return (snprintf(buf, PAGE_SIZE, "%d\n", mute));
}

static CLASS_ATTR(mute, S_IRUGO | S_IWUSR, agdev_r_mute, agdev_w_mute);

static ssize_t agdev_w_diag(struct class *class, const char *buf, size_t count)
{
	uint32_t status;

	long int diag;
	unsigned long flags;
	struct agctl_data *agctl = container_of(class, struct agctl_data, agclass);
	if (strict_strtol(buf, 0, &diag) != 0) {
		return -EINVAL;
	}
	diag = (diag << 16) & AC_CTL_DIAG_MASK;
	if (agctl != NULL) {
		struct agctl_master *agm = agctl->agm;
		spin_lock_irqsave(&agm->aglock, flags);
		status = agc_regr(agm, agctl->ctlreg_offs) & ~AC_CTL_IRQ_MASK;
		status = (status & ~AC_CTL_DIAG_MASK) | diag;
		agc_regw(agm, agctl->ctlreg_offs, status);
		spin_unlock_irqrestore(&agm->aglock, flags);
	}
	return count;
}

static ssize_t agdev_r_diag(struct class *class, char *buf)
{
	uint32_t diag = 0;
	struct agctl_data *agctl = container_of(class, struct agctl_data, agclass);
	if (agctl != NULL) {
		struct agctl_master *agm = agctl->agm;
		diag = (agc_regr(agm, agctl->ctlreg_offs) & AC_CTL_DIAG_MASK) >> 16;
	}
	return (snprintf(buf, PAGE_SIZE, "%lu\n", (long unsigned int)diag));
}

static CLASS_ATTR(diagnostic, S_IRUGO | S_IWUSR, agdev_r_diag, agdev_w_diag);

static ssize_t agdev_w_strobe(struct class *class, const char *buf, size_t count)
{
	long int val;
	struct agctl_data *agctl = container_of(class, struct agctl_data, agclass);
	if (strict_strtol(buf, 0, &val) != 0) {
		return -EINVAL;
	}
	if (val < 0) {
		struct agctl_master *agm = agctl->agm;
		agdev_strobe(agm, agctl);
	} else {
		agctl->strobedelay = (int)val;
	}
	return count;
}

static ssize_t agdev_r_strobe(struct class *class, char *buf)
{
	struct agctl_data *agctl = container_of(class, struct agctl_data, agclass);
	return (snprintf(buf, PAGE_SIZE, "%lu\n", (long unsigned int)abs(agctl->strobedelay)));
}

static CLASS_ATTR(strobe, S_IRUGO | S_IWUSR, agdev_r_strobe, agdev_w_strobe);

static ssize_t agdev_r_cardid(struct class *class, char *buf)
{
	uint32_t id = 0;
	struct agctl_data *agctl = container_of(class, struct agctl_data, agclass);
	if (agctl != NULL) {
		struct agctl_master *agm = agctl->agm;
		id = (agc_regr(agm, agctl->ctlreg_offs) >> 24) & 0xFF;
	}
	return (snprintf(buf, PAGE_SIZE, "%02lX\n", (long unsigned int)id));
}

static CLASS_ATTR(cardid, S_IRUGO, agdev_r_cardid, NULL);

static ssize_t agdev_r_status(struct class *class, char *buf)
{
	uint32_t status = 0;
	struct agctl_data *agctl = container_of(class, struct agctl_data, agclass);
	if (agctl != NULL) {
		struct agctl_master *agm = agctl->agm;
		status = agc_regr(agm, agctl->ctlreg_offs);
	}
	return (snprintf(buf, PAGE_SIZE, "0x%08lX\n", (long unsigned int)status));
}

static CLASS_ATTR(status, S_IRUGO, agdev_r_status, NULL);

static ssize_t agdev_w_avbmute(struct class *class, const char *buf, size_t count)
{
	uint32_t status = 0;

	long int val;
	unsigned long flags;
	struct agctl_data *agctl = container_of(class, struct agctl_data, agclass);
	if (strncmp(buf, "on", 2) == 0) {
		val = -1;
	} else if (strncmp(buf, "off", 3) == 0) {
		val = -2;
	} else if (strict_strtol(buf, 0, &val) != 0 || val < 0 || val > 0xF) {
		return -EINVAL;
	}
	if (agctl != NULL) {
		struct agctl_master *agm = agctl->agm;
		spin_lock_irqsave(&agm->aglock, flags);
		status = agc_regr(agm, agctl->ctlreg_offs);
		if (val == -1) {
			status |= AC_CTL_MUTE_FORCE;
			status &= ~(AC_CTL_ENA_UNMUTE | AC_CTL_MUTESTREAM_MASK);
		} else if (val == -2) {
			status |= AC_CTL_ENA_UNMUTE;
			status &= ~(AC_CTL_MUTE_FORCE | AC_CTL_MUTESTREAM_MASK);
		} else {
			status = (status & ~(AC_CTL_MUTESTREAM_MASK | AC_CTL_ENA_UNMUTE | AC_CTL_MUTE_FORCE)) |
					((val << 20) & AC_CTL_MUTESTREAM_MASK);
		}
		agc_regw(agm, agctl->ctlreg_offs, status);
		spin_unlock_irqrestore(&agm->aglock, flags);
	}
	return count;
}

static ssize_t agdev_r_avbmute(struct class *class, char *buf)
{
	uint32_t status = 0;
	struct agctl_data *agctl = container_of(class, struct agctl_data, agclass);
	if (agctl != NULL) {
		struct agctl_master *agm = agctl->agm;
		status = agc_regr(agm, agctl->ctlreg_offs);
		if ((status & AC_CTL_MUTE_FORCE) == 0 && (status & AC_CTL_ENA_UNMUTE) != 0) {
			strncpy(buf, "off\n", PAGE_SIZE);
		} else if ((status & AC_CTL_MUTE_FORCE) != 0 && (status & AC_CTL_ENA_UNMUTE) == 0) {
			strncpy(buf, "on\n", PAGE_SIZE);
		} else {
			snprintf(buf, PAGE_SIZE, "%u\n", (unsigned int)((status >> 20) & 0xF));
		}
	} else {
		strncpy(buf, "-1\n", PAGE_SIZE);
	}
	return (strlen(buf));
}

static CLASS_ATTR(avbmute, S_IRUGO | S_IWUSR, agdev_r_avbmute, agdev_w_avbmute);

/*-------------------------------------------------------------------------*/

void garcia_control_set_master(int is_master)
{
	unsigned long minor;
	for (minor = 0; minor < N_AGCTL_CHANNELS; minor++) {
		agm_stat->chan[minor].flags = (uint8_t)
					((agm_stat->chan[minor].flags & ~AGCTL_FLAG_MASTER) |
					((is_master != 0) ? AGCTL_FLAG_MASTER : 0));
	}
	return;
}
EXPORT_SYMBOL(garcia_control_set_master);

static int garcia_control_probe(const char *name, struct platform_device *pdev,
		void __iomem *address, uint32_t irq)
{
	struct agctl_data *agctl;
	struct agctl_master *agm;
	unsigned long minor;
	int status = 0;
	char nodename[32];

	/* Allocate driver data */
	agm = kzalloc(sizeof(*agm), GFP_KERNEL);
	if (!agm)
		return -ENOMEM;
	/* Initialize the driver data */
	memset(agm, 0, sizeof(*agm));
	agm_stat = agm;
	spin_lock_init(&agm->aglock);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	printk(KERN_INFO "Probe Garcia control, name \"%s\" at address %p IRQ %d\n",
				name, address, irq);
	agm->irq = irq;
	mutex_init(&agm->agmutex);
	agm->regs = address;
	agm->chan[0].ctlreg_offs = AC_CONTROL0_OFFSET;
	agm->chan[1].ctlreg_offs = AC_CONTROL1_OFFSET;
	agm->chan[2].ctlreg_offs = AC_CONTROL2_OFFSET;
	agm->chan[3].ctlreg_offs = AC_CONTROL3_OFFSET;
	agm->chan[4].ctlreg_offs = AC_CONTROL4_OFFSET;
	agc_regw(agm, AC_CLK_COUNT_OFFSET, 0);

	for (minor = 0; minor < N_AGCTL_CHANNELS; minor++) {
		agctl = &agm->chan[minor];
		agctl->agm = agm;
		agctl->flags = (uint8_t)(minor & AGCTL_FLAG_SLOTMASK);
		agctl->devt = MKDEV(AGCTL_MAJOR, minor);
		init_waitqueue_head(&agctl->wait);

		snprintf(nodename, sizeof(nodename), "agctl-%d.%ld", AGCTL_MAJOR, minor);
		agc_regw(agm, agctl->ctlreg_offs, AC_CTL_BUF_COL_IRQ | AC_CTL_RESET_IRQ |
				AC_CTL_MUTE_IRQ | AC_CTL_STROBE_IRQ | AC_CTL_TX_COMPL_IRQ);
		agctl->saved_status = agc_regr(agm, agctl->ctlreg_offs);
		agctl->agclass.name = nodename;
		agctl->agclass.owner = THIS_MODULE;
		agctl->agclass.class_release = NULL;
		class_register(&agctl->agclass);

		if (IS_ERR(&agctl->agclass)) {
			printk(KERN_ERR "Unable create sysfs class for %s\n", nodename);
			return PTR_ERR(&agctl->agclass);
		}

		status = class_create_file(&agctl->agclass, &class_attr_ssi_ddir);
		status = class_create_file(&agctl->agclass, &class_attr_reset);
		status = class_create_file(&agctl->agclass, &class_attr_mute);
		status = class_create_file(&agctl->agclass, &class_attr_diagnostic);
		status = class_create_file(&agctl->agclass, &class_attr_strobe);
		status = class_create_file(&agctl->agclass, &class_attr_cardid);
		status = class_create_file(&agctl->agclass, &class_attr_status);
		status = class_create_file(&agctl->agclass, &class_attr_avbmute);
	}
	/* Register for interrupt */
	status = request_irq(agm->irq, agc_irq_callback, 0, name, agm);
	if (status != 0) {
		dev_warn(&pdev->dev, "irq request failure: %d\n", agm->irq);
		kfree(agm);
		return -ENXIO;
	}

	if (status != 0)
		kfree(agm);

	return status;
}

/* Probe for registered devices */
static int garcia_control_platform_probe(struct platform_device *pdev)
{
	struct resource *r_irq;
	struct resource *r_mem;
	void __iomem *iom;

	/* Obtain the resources for this instance */
	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r_mem) {
		printk(KERN_ERR "%s: IO resource address not found.\n", pdev->name);
		return -ENXIO;
	}
	if (!request_mem_region(r_mem->start,
			r_mem->end - r_mem->start + 1, pdev->name)) {
		dev_warn(&pdev->dev, "memory request failure\n");
		return -ENXIO;
	}

	iom = ioremap(r_mem->start, r_mem->end - r_mem->start + 1);


	/* Attempt to obtain the IRQ; if none is specified, the resource pointer is
	 * NULL, and polling will be used.
	 */
	r_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	/* Dispatch to the generic function */
	return(garcia_control_probe(pdev->name, pdev, iom, r_irq->start));
}

static int garcia_control_platform_remove(struct platform_device *pdev)
{
	struct agctl_master *agm = platform_get_drvdata(pdev);
	int users = 0;
	int i;

 	if (!agm) {
 		return(-1);
 	}
	/* prevent new opens */
 	for (i = 0; i < N_AGCTL_CHANNELS; i++) {
 		class_remove_file(&agm->chan[i].agclass, &class_attr_ssi_ddir);
 		class_remove_file(&agm->chan[i].agclass, &class_attr_reset);
 		class_remove_file(&agm->chan[i].agclass, &class_attr_mute);
 		class_remove_file(&agm->chan[i].agclass, &class_attr_diagnostic);
 		class_remove_file(&agm->chan[i].agclass, &class_attr_strobe);
 		class_remove_file(&agm->chan[i].agclass, &class_attr_cardid);
 		class_remove_file(&agm->chan[i].agclass, &class_attr_status);
 		class_remove_file(&agm->chan[i].agclass, &class_attr_avbmute);
 		class_unregister(&agm->chan[i].agclass);
 		users += agm->chan[0].users;
 	}
	iounmap(agm->regs);
	free_irq(agm->irq, agm);
	if (users == 0) {
		agm_stat = NULL;
		kfree(agm);
	}
	return 0;
}

#ifdef CONFIG_OF
static int __devinit garcia_control_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
	int			rc;
	struct resource r_irq_struct;
	struct resource r_mem_struct;
	struct resource *r_irq = &r_irq_struct;
	struct resource *r_mem = &r_mem_struct;
	struct platform_device *pdev = to_platform_device(&ofdev->dev);
	const char *name = dev_name(&ofdev->dev);
	void __iomem *iom;
	int irq;

	/* Obtain the resources for this instance */
	rc = of_address_to_resource(ofdev->node, 0, r_mem);
	if (rc) {
		dev_warn(&ofdev->dev,"invalid address\n");
		return rc;
	}
	if (!request_mem_region(r_mem->start,
			r_mem->end - r_mem->start + 1, name)) {
		rc = -ENXIO;
		dev_warn(&ofdev->dev, "memory request failure\n");
	}

	iom = ioremap(r_mem->start, r_mem->end - r_mem->start + 1);

	irq = of_irq_to_resource(ofdev->node, 0, r_irq);
	if (irq == NO_IRQ) {
		dev_warn(&ofdev->dev,"invalid interrupt\n");
		return -ENXIO;
	}

	/* Dispatch to the generic function */
	return(garcia_control_probe(name, pdev, iom, irq));
}

static int __devexit garcia_control_of_remove(struct of_device *dev)
{
	struct platform_device *pdev = to_platform_device(&dev->dev);
	garcia_control_platform_remove(pdev);
	return(0);
}

static struct of_device_id garcia_control_of_match[] = {
	{ .compatible = "xlnx,avid-config-bus-controller-1.00.a", },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, garcia_control_of_match);

static struct of_platform_driver of_garcia_control_driver = {
	.name		= DRIVER_NAME,
	.match_table	= garcia_control_of_match,
	.probe		= garcia_control_of_probe,
	.remove		= __devexit_p(garcia_control_of_remove),
};
#endif // CONFIG_OF

/* Platform device driver structure */
static struct platform_driver garcia_control_driver = {
  .probe  = garcia_control_platform_probe,
  .remove = garcia_control_platform_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/*-------------------------------------------------------------------------*/

static int __init agctl_init(void)
{
	int status;

#ifdef CONFIG_OF
	status = of_register_platform_driver(&of_garcia_control_driver);
#endif

  /* Register as a platform device driver */
	if((status = platform_driver_register(&garcia_control_driver)) < 0) {
		printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
		return status;
	}

	/* Claim our reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	status = register_chrdev(AGCTL_MAJOR, DRIVER_NAME, &agctl_fops);
	return status;
}
module_init(agctl_init);

static void __exit agctl_exit(void)
{
	unregister_chrdev(AGCTL_MAJOR, DRIVER_NAME);
}
module_exit(agctl_exit);

MODULE_AUTHOR("Scott Wagner <scott.wagner@labxtechnologies.com>");
MODULE_DESCRIPTION("Avid Garcia audio control bus interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
