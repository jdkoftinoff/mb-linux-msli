/*
 * Xilinx Ethernet: Linux driver for the XPS_LLTEMAC core.
 *
 * Author: Xilinx, Inc.
 *
 * 2006-2007 (c) Xilinx, Inc. This file is licensed uner the terms of the GNU
 * General Public License version 2.1. This program is licensed "as is" without
 * any warranty of any kind, whether express or implied.
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who  Date     Changes
 * ----- software/linux-2.6.x-petalogix/---- software/linux-2.6.x-petalogix/-------- software/linux-2.6.x-petalogix/-------------------------------------------------------
 * 1.00a jvb  05/08/05 First release
 * </pre>
 *
 */

/*
 * With the way  
the hardened Temac works, the driver needs to communicate
 * with the PHY controller. Since each board will have a different
 * type of PHY, the code that communicates with the MII type controller
 * is inside #ifdef XILINX_PLB_TEMAC_3_00A_ML403_PHY_SUPPORT conditional
 * compilation. For your specific board, you will want to replace this code with
 * code of your own for your specific board.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/mii.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/xilinx_devices.h>
#include <asm/io.h>
#include <linux/ethtool.h>
#include <linux/vmalloc.h>
#include <linux/kthread.h>

#ifdef CONFIG_OF
// For open firmware.
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif

#include "xbasic_types.h"
#include "labx_eth_locallink.h"
#include "xllfifo.h"
#include "xlldma.h"
#include "xlldma_bdring.h"

#include "net/labx_ethernet/labx_ethernet_defs.h"

#define LOCAL_FEATURE_RX_CSUM   0x01

/* CRC error statistics counter, >= 1.2 */
#define RX_CRC_ERRS_MIN_VERSION ((1 << REVISION_MAJOR_SHIFT) | 2)

/* MDIO divisor which should be "safe" for the hard TEMAC; we don't
 * actually use this hardware but it does need to be configured.
 */
#define LABX_ETH_LOCALLINK_MDIO_DIV  (0x28)

/*
 * Default SEND and RECV buffer descriptors (BD) numbers.
 * BD Space needed is (XTE_SEND_BD_CNT+XTE_RECV_BD_CNT)*Sizeof(XLlDma_Bd).
 * Each XLlDma_Bd instance currently takes 40 bytes.
 */
#define XTE_SEND_BD_CNT 256
#define XTE_RECV_BD_CNT 256

/* Must be shorter than length of ethtool_drvinfo.driver field to fit */
#define DRIVER_NAME         "labx_eth_llink"
#define DRIVER_DESCRIPTION  "Lab X Tri-Mode Ethernet MAC driver"
#define DRIVER_VERSION      "1.00a"

#define TX_TIMEOUT   (3*HZ)	/* Transmission timeout is 3 seconds. */

/*
 * This version of the Xilinx TEMAC uses external DMA or FIFO cores.
 * Currently neither the DMA or FIFO cores used require any memory alignment
 * restrictions.
 */
/*
 * ALIGNMENT_RECV = the alignement required to receive
 * ALIGNMENT_SEND = the alignement required to send
 * ALIGNMENT_SEND_PERF = tx alignment for better performance
 *
 * ALIGNMENT_SEND is used to see if we *need* to copy the data to re-align.
 * ALIGNMENT_SEND_PERF is used if we've decided we need to copy anyway, we just
 * copy to this alignment for better performance.
 */

#define ALIGNMENT_RECV          32
#define ALIGNMENT_SEND          8
#define ALIGNMENT_SEND_PERF     32

#define XTE_SEND  1
#define XTE_RECV  2

/* SGDMA buffer descriptors must be aligned on a 8-byte boundary. */
#define ALIGNMENT_BD            XLLDMA_BD_MINIMUM_ALIGNMENT

/* BUFFER_ALIGN(adr) calculates the number of bytes to the next alignment. */
#define BUFFER_ALIGNSEND(adr) ((ALIGNMENT_SEND - ((u32) adr)) % ALIGNMENT_SEND)
#define BUFFER_ALIGNSEND_PERF(adr) ((ALIGNMENT_SEND_PERF - ((u32) adr)) % ALIGNMENT_SEND_PERF)
#define BUFFER_ALIGNRECV(adr) ((ALIGNMENT_RECV - ((u32) adr)) % ALIGNMENT_RECV)

/* Default TX/RX Threshold and waitbound values for SGDMA mode */
#define DFT_TX_THRESHOLD  24
#define DFT_TX_WAITBOUND  254
#define DFT_RX_THRESHOLD  4
#define DFT_RX_WAITBOUND  254

#define XTE_AUTOSTRIPPING 1

/* Put Buffer Descriptors in BRAM?
 * NOTE:
 *   Putting BDs in BRAM only works if there is only ONE instance of the TEMAC
 *   in hardware.  The code does not handle multiple instances, e.g. it does
 *   not manage the memory in BRAM.
 */

#define BD_IN_BRAM        0
#define BRAM_BASEADDR     0xffff8000 /* FIXME not acceptable for Microblaze */


/*
 * Checksum offload macros
 */
#define BdCsumEnable(BdPtr) \
	XLlDma_mBdWrite((BdPtr), XLLDMA_BD_STSCTRL_USR0_OFFSET,             \
		(XLlDma_mBdRead((BdPtr), XLLDMA_BD_STSCTRL_USR0_OFFSET)) | 1 )

/* Used for debugging */
#define BdCsumEnabled(BdPtr) \
	((XLlDma_mBdRead((BdPtr), XLLDMA_BD_STSCTRL_USR0_OFFSET)) & 1)

#define BdCsumDisable(BdPtr) \
	XLlDma_mBdWrite((BdPtr), XLLDMA_BD_STSCTRL_USR0_OFFSET,             \
		(XLlDma_mBdRead((BdPtr), XLLDMA_BD_STSCTRL_USR0_OFFSET)) & 0xFFFFFFFE )

#define BdCsumSetup(BdPtr, Start, Insert) \
    XLlDma_mBdWrite((BdPtr), XLLDMA_BD_USR1_OFFSET, (Start) << 16 | (Insert))

/* Used for debugging */
#define BdCsumInsert(BdPtr) \
    (XLlDma_mBdRead((BdPtr), XLLDMA_BD_USR1_OFFSET) & 0xffff)

#define BdCsumSeed(BdPtr, Seed) \
    XLlDma_mBdWrite((BdPtr), XLLDMA_BD_USR2_OFFSET, 0)

#define BdCsumGet(BdPtr) \
    XLlDma_mBdRead((BdPtr), XLLDMA_BD_USR3_OFFSET)

#define BdGetRxLen(BdPtr) \
    XLlDma_mBdRead((BdPtr), XLLDMA_BD_USR4_OFFSET)

/* accept MAC address of the form macaddr=0x08,0x00,0x20,0x30,0x40,0x50 */
static int macaddr[6];
module_param_array(macaddr, int, NULL, 0);
MODULE_PARM_DESC(macaddr, "MAC address to set");

/*
 * Our private per device data.  When a net_device is allocated we will
 * ask for enough extra space for this.
 */
#define IRQ_NAME_SZ (64)
struct net_local {
	struct phy_device *phy_dev;
	char phy_name[64];

	struct list_head rcv;
	struct list_head xmit;

	struct net_device *ndev;	/* this device */
	struct net_device *next_dev;	/* The next device in dev_list */
	struct net_device_stats stats;	/* Statistics for this device */

	u32 index;		/* Which interface is this */
	u8 gmii_addr;		/* The GMII address of the PHY */

	/* The underlying OS independent code needs space as well.  A
	 * pointer to the following XLlTemac structure will be passed to
	 * any XLlTemac_ function that requires it.  However, we treat the
	 * data as an opaque object in this file (meaning that we never
	 * reference any of the fields inside of the structure). */
	XLlFifo Fifo;
	XLlDma Dma;
	XLlTemac Emac;

	unsigned int fifo_irq;	/* fifo irq */
	unsigned int dma_irq_s;	/* send irq */
	unsigned int dma_irq_r;	/* recv irq */
	unsigned int max_frame_size;

	int cur_speed;

	/* Buffer Descriptor space for both TX and RX BD ring */
	void *desc_space;	/* virtual address of BD space */
	dma_addr_t desc_space_handle;	/* physical address of BD space */
	int desc_space_size;	/* size of BD space */

	/* buffer for one skb in case no room is available for transmission */
	struct sk_buff *deferred_skb;

	/* send buffers for non tx-dre hw */
	void **tx_orig_buffers;	/* Buffer addresses as returned by
				   dma_alloc_coherent() */
	void **tx_buffers;	/* Buffers addresses aligned for DMA */
	dma_addr_t *tx_phys_buffers;	/* Buffer addresses in physical memory */
	size_t tx_buffers_cur;	/* Index of current buffer used */

	/* stats */
	int max_frags_in_a_packet;
	unsigned long realignments;
	unsigned long tx_hw_csums;
	unsigned long rx_hw_csums;
	unsigned long local_features;
#if ! XTE_AUTOSTRIPPING
	unsigned long stripping;
#endif

  /* IRQ names */
  char fifo_irq_name[IRQ_NAME_SZ];
  char mdio_irq_name[IRQ_NAME_SZ];
};

static void labx_eth_ll_mac_adjust_link(struct net_device *dev);

static u32 dma_rx_int_mask = XLLDMA_CR_IRQ_ALL_EN_MASK;
static u32 dma_tx_int_mask = XLLDMA_CR_IRQ_ALL_EN_MASK;

/* for exclusion of all program flows (processes, ISRs and BHs) */
static spinlock_t XTE_spinlock = SPIN_LOCK_UNLOCKED;
static spinlock_t XTE_tx_spinlock = SPIN_LOCK_UNLOCKED;
static spinlock_t XTE_rx_spinlock = SPIN_LOCK_UNLOCKED;



/* Helper function to determine if a given XLlTemac error warrants a reset. */
extern inline int status_requires_reset(int s)
{
	return (s == XST_FIFO_ERROR ||
		s == XST_PFIFO_DEADLOCK ||
		s == XST_DMA_ERROR || s == XST_IPIF_ERROR);
}

/* Queues with locks */
static LIST_HEAD(receivedQueue);
static spinlock_t receivedQueueSpin = SPIN_LOCK_UNLOCKED;

static LIST_HEAD(sentQueue);
static spinlock_t sentQueueSpin = SPIN_LOCK_UNLOCKED;


/* from mii.h
 *
 * Items in mii.h but not in gmii.h
 */
#define ADVERTISE_100FULL       0x0100
#define ADVERTISE_100HALF       0x0080
#define ADVERTISE_10FULL        0x0040
#define ADVERTISE_10HALF        0x0020
#define ADVERTISE_CSMA          0x0001

#define EX_ADVERTISE_1000FULL   0x0200
#define EX_ADVERTISE_1000HALF   0x0100

/*
 * items not in mii.h nor gmii.h but should be
 */
#define MII_EXADVERTISE 0x09

/*
 * Wrap certain temac routines with a lock, so access to the shared hard temac
 * interface is accessed mutually exclusive for dual channel temac support.
 */

static inline void _XLlTemac_Start(XLlTemac *InstancePtr)
{
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	labx_XLlTemac_Start(InstancePtr);
	spin_unlock_irqrestore(&XTE_spinlock, flags);
}

static inline void _XLlTemac_Stop(XLlTemac *InstancePtr)
{
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	labx_XLlTemac_Stop(InstancePtr);
	spin_unlock_irqrestore(&XTE_spinlock, flags);
}

static inline void _XLlTemac_Reset(XLlTemac *InstancePtr, int HardCoreAction)
{
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	labx_XLlTemac_Reset(InstancePtr, HardCoreAction);
	spin_unlock_irqrestore(&XTE_spinlock, flags);
}

static inline int _XLlTemac_SetMacAddress(XLlTemac *InstancePtr,
					  void *AddressPtr)
{
	int status;
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	status = labx_XLlTemac_SetMacAddress(InstancePtr, AddressPtr);
	spin_unlock_irqrestore(&XTE_spinlock, flags);

	return status;
}

static inline void _XLlTemac_GetMacAddress(XLlTemac *InstancePtr,
					   void *AddressPtr)
{
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	labx_XLlTemac_GetMacAddress(InstancePtr, AddressPtr);
	spin_unlock_irqrestore(&XTE_spinlock, flags);
}

static inline int _XLlTemac_SetOptions(XLlTemac *InstancePtr, u32 Options)
{
	int status;
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	status = labx_XLlTemac_SetOptions(InstancePtr, Options);
	spin_unlock_irqrestore(&XTE_spinlock, flags);

	return status;
}

static inline int _XLlTemac_ClearOptions(XLlTemac *InstancePtr, u32 Options)
{
	int status;
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	status = labx_XLlTemac_ClearOptions(InstancePtr, Options);
	spin_unlock_irqrestore(&XTE_spinlock, flags);

	return status;
}

static inline u16 _XLlTemac_GetOperatingSpeed(XLlTemac *InstancePtr)
{
	u16 speed;
	unsigned long flags;
	spin_lock_irqsave(&XTE_spinlock, flags);
	speed = labx_XLlTemac_GetOperatingSpeed(InstancePtr);
	spin_unlock_irqrestore(&XTE_spinlock, flags);
	return speed;
}

static inline void _XLlTemac_SetOperatingSpeed(XLlTemac *InstancePtr, u16 Speed)
{
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	labx_XLlTemac_SetOperatingSpeed(InstancePtr, Speed);
	spin_unlock_irqrestore(&XTE_spinlock, flags);

	/* We need a delay after we set the speed. Otherwise the PHY will not be ready. */
	udelay(10000);
}

static inline void _XLlTemac_PhySetMdioDivisor(XLlTemac *InstancePtr, u8 Divisor)
{
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	labx_XLlTemac_PhySetMdioDivisor(InstancePtr, Divisor);
	spin_unlock_irqrestore(&XTE_spinlock, flags);
}

inline void _labx_XLlTemac_PhyRead(XLlTemac *InstancePtr, u32 PhyAddress,
                                   u32 RegisterNum, u16 *PhyDataPtr)
{
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	labx_XLlTemac_PhyRead(InstancePtr, PhyAddress, RegisterNum, PhyDataPtr);
	spin_unlock_irqrestore(&XTE_spinlock, flags);
}

inline void _labx_XLlTemac_PhyWrite(XLlTemac *InstancePtr, u32 PhyAddress,
                                    u32 RegisterNum, u16 PhyData)
{
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	labx_XLlTemac_PhyWrite(InstancePtr, PhyAddress, RegisterNum, PhyData);
	spin_unlock_irqrestore(&XTE_spinlock, flags);
}


static inline int _XLlTemac_MulticastClear(XLlTemac *InstancePtr, int Entry)
{
	int status;
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	status = labx_XLlTemac_MulticastClear(InstancePtr, Entry);
	spin_unlock_irqrestore(&XTE_spinlock, flags);

	return status;
}

static inline int _XLlTemac_SetMacPauseAddress(XLlTemac *InstancePtr, void *AddressPtr)
{
	int status;
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	status = labx_XLlTemac_SetMacPauseAddress(InstancePtr, AddressPtr);
	spin_unlock_irqrestore(&XTE_spinlock, flags);

	return status;
}

static inline void _XLlTemac_GetMacPauseAddress(XLlTemac *InstancePtr, void *AddressPtr)
{
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	labx_XLlTemac_GetMacPauseAddress(InstancePtr, AddressPtr);
	spin_unlock_irqrestore(&XTE_spinlock, flags);
}

static inline int _XLlTemac_GetSgmiiStatus(XLlTemac *InstancePtr, u16 *SpeedPtr)
{
	int status;
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	status = labx_XLlTemac_GetSgmiiStatus(InstancePtr, SpeedPtr);
	spin_unlock_irqrestore(&XTE_spinlock, flags);

	return status;
}

static inline int _XLlTemac_GetRgmiiStatus(XLlTemac *InstancePtr,
					   u16 *SpeedPtr,
					   int *IsFullDuplexPtr,
					   int *IsLinkUpPtr)
{
	int status;
	unsigned long flags;

	spin_lock_irqsave(&XTE_spinlock, flags);
	status = labx_XLlTemac_GetRgmiiStatus(InstancePtr, SpeedPtr, IsFullDuplexPtr, IsLinkUpPtr);
	spin_unlock_irqrestore(&XTE_spinlock, flags);

	return status;
}

#define DEBUG_ERROR KERN_ERR
#define DEBUG_LOG(level, ...) printk(level __VA_ARGS__)

typedef enum DUPLEX { UNKNOWN_DUPLEX, HALF_DUPLEX, FULL_DUPLEX } DUPLEX;

/*
 * Helper function to reset the underlying hardware.  This is called
 * when we get into such deep trouble that we don't know how to handle
 * otherwise.
 */
static void reset(struct net_device *dev, u32 line_num)
{
	struct net_local *lp = netdev_priv(dev);
	u32 TxThreshold, TxWaitBound, RxThreshold, RxWaitBound;
	u32 Options;
	static u32 reset_cnt = 0;
	int status;

	printk(KERN_INFO "%s: labx_eth_llink: resets (#%u) from adapter code line %d\n",
	       dev->name, ++reset_cnt, line_num);

	/* Shouldn't really be necessary, but shouldn't hurt. */
	netif_stop_queue(dev);

	/* Stop device */
	_XLlTemac_Stop(&lp->Emac);

	/*
	 * XLlTemac_Reset puts the device back to the default state.  We need
	 * to save all the settings we don't already know, reset, restore
	 * the settings, and then restart the TEMAC.
	 */
	Options = labx_XLlTemac_GetOptions(&lp->Emac);

	/*
	 * Capture the dma coalesce settings (if needed) and reset the
	 * connected core, dma or fifo
	 */
	if (XLlTemac_IsDma(&lp->Emac)) {
		XLlDma_BdRingGetCoalesce(&XLlDma_mGetRxRing(&lp->Dma),
					 &RxThreshold, &RxWaitBound);
		XLlDma_BdRingGetCoalesce(&XLlDma_mGetTxRing(&lp->Dma),
					 &TxThreshold, &TxWaitBound);

		XLlDma_Reset(&lp->Dma);
	} else {
		XLlFifo_Reset(&lp->Fifo);
	}

#if 0

	/* now we can reset the device */
	_XLlTemac_Reset(&lp->Emac, XTE_NORESET_HARD);

	/* Reset on TEMAC also resets PHY. Give it some time to finish negotiation
	 * before we move on */
#endif

    /* Configure the MDIO divisor; if the hard TEMAC is being used, this
     * is crucial to getting the hardware to run, even if its MDIO controller
     * is not being used (which, typically it isn't - the soft Lab X MDIO
     * controller is instead).
     */
    _XLlTemac_PhySetMdioDivisor(&lp->Emac, LABX_ETH_LOCALLINK_MDIO_DIV);

	/* Perform PHY setup */
	if (NULL != lp->phy_dev)
	{
		int ret;
		printk("%s: About to call phy_start_aneg()\n",__func__);
		ret = phy_start_aneg(lp->phy_dev);
		if (0 != ret) {
			printk("%s: phy_start_aneg() Failed with code %d\n",__func__,ret);
		} else {
			printk("%s: phy_start_aneg() Passed\n",__func__);
		}
	}

	/*
	 * The following four functions will return an error if the
	 * EMAC is already started.  We just stopped it by calling
	 * _XLlTemac_Reset() so we can safely ignore the return values.
	 */
	(int) _XLlTemac_SetMacAddress(&lp->Emac, dev->dev_addr);
	(int) _XLlTemac_SetOptions(&lp->Emac, Options);
	(int) _XLlTemac_ClearOptions(&lp->Emac, ~Options);
	Options = labx_XLlTemac_GetOptions(&lp->Emac);
	printk(KERN_INFO "%s: labx_eth_llink: Options: 0x%x\n", dev->name, Options);

	if (XLlTemac_IsDma(&lp->Emac)) {	/* SG DMA mode */
		status = XLlDma_BdRingSetCoalesce(&lp->Dma.RxBdRing,
						  RxThreshold, RxWaitBound);
		status |= XLlDma_BdRingSetCoalesce(&lp->Dma.TxBdRing,
						   TxThreshold, TxWaitBound);
		if (status != XST_SUCCESS) {
			/* Print the error, but keep on going as it's not a fatal error. */
			printk(KERN_ERR "%s: labx_eth_llink: error setting coalesce values (probably out of range). status: %d\n",
			       dev->name, status);
		}
		XLlDma_mBdRingIntEnable(&lp->Dma.RxBdRing, dma_rx_int_mask);
		XLlDma_mBdRingIntEnable(&lp->Dma.TxBdRing, dma_tx_int_mask);
	} else {			/* FIFO interrupt mode */
		XLlFifo_IntEnable(&lp->Fifo, XLLF_INT_TC_MASK |
				XLLF_INT_RC_MASK | XLLF_INT_RXERROR_MASK |
				XLLF_INT_TXERROR_MASK);
	}
	//XLlTemac_IntDisable(&lp->Emac, XTE_INT_ALL_MASK);

	if (lp->deferred_skb) {
		dev_kfree_skb_any(lp->deferred_skb);
		lp->deferred_skb = NULL;
		lp->stats.tx_errors++;
	}

	/*
	 * XLlTemac_Start returns an error when: if configured for
	 * scatter-gather DMA and a descriptor list has not yet been created
	 * for the send or receive channel, or if no receive buffer descriptors
	 * have been initialized. Those are not happening. so ignore the returned
	 * result checking.
	 */
	_XLlTemac_Start(&lp->Emac);

	/* We're all ready to go.  Start the queue in case it was stopped. */
	netif_wake_queue(dev);
}

static void FifoSendHandler(struct net_device *dev);
static void FifoRecvHandler(unsigned long p /*struct net_device *dev*/);

static DECLARE_TASKLET(FifoRecvBH, FifoRecvHandler, 0);

static irqreturn_t xenet_fifo_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct net_local *lp = netdev_priv(dev);
	u32 irq_status;

	unsigned long flags;

	/*
	 * Need to:
	 * 1) Read the FIFO IS register
	 * 2) clear all bits in the FIFO IS register
	 * 3) loop on each bit in the IS register, and handle each interrupt event
	 *
	 */
	irq_status = XLlFifo_IntPending(&lp->Fifo);
	XLlFifo_IntClear(&lp->Fifo, irq_status);
	while (irq_status) {
		if (irq_status & XLLF_INT_RC_MASK) {
			/* handle the receive completion */
			struct list_head *cur_lp;
			spin_lock_irqsave(&receivedQueueSpin, flags);
			list_for_each(cur_lp, &receivedQueue) {
				if (cur_lp == &(lp->rcv)) {
					break;
				}
			}
			if (cur_lp != &(lp->rcv)) {
				list_add_tail(&lp->rcv, &receivedQueue);
				XLlFifo_IntDisable(&lp->Fifo, XLLF_INT_ALL_MASK);
                tasklet_schedule(&FifoRecvBH);
			}
			spin_unlock_irqrestore(&receivedQueueSpin, flags);
			irq_status &= ~XLLF_INT_RC_MASK;
		} else if (irq_status & XLLF_INT_TC_MASK) {
			/* handle the transmit completion */
			FifoSendHandler(dev);
			irq_status &= ~XLLF_INT_TC_MASK;
		} else if (irq_status & XLLF_INT_TXERROR_MASK) {
			lp->stats.tx_errors++;
			lp->stats.tx_fifo_errors++;
			XLlFifo_Reset(&lp->Fifo);
			irq_status &= ~XLLF_INT_TXERROR_MASK;
		} else if (irq_status & XLLF_INT_RXERROR_MASK) {
			lp->stats.rx_errors++;
			XLlFifo_Reset(&lp->Fifo);
			irq_status &= ~XLLF_INT_RXERROR_MASK;
		} else {
			/* debug
			 * if (irq_status == 0) printk("Temac: spurious fifo int\n");
			 */
		}
	}

	return IRQ_HANDLED;
}

/* The callback function for completed frames sent in SGDMA mode. */
static void DmaSendHandlerBH(unsigned long p);
static void DmaRecvHandlerBH(unsigned long p);

static DECLARE_TASKLET(DmaSendBH, DmaSendHandlerBH, 0);
static DECLARE_TASKLET(DmaRecvBH, DmaRecvHandlerBH, 0);

static irqreturn_t xenet_dma_rx_interrupt(int irq, void *dev_id)
{
	u32 irq_status;
	struct net_device *dev = dev_id;
	struct net_local *lp = netdev_priv(dev);
	struct list_head *cur_lp;

        unsigned int flags;

	/* Read pending interrupts */
	irq_status = XLlDma_mBdRingGetIrq(&lp->Dma.RxBdRing);

	XLlDma_mBdRingAckIrq(&lp->Dma.RxBdRing, irq_status);

	if ((irq_status & XLLDMA_IRQ_ALL_ERR_MASK)) {
		XLlDma_Reset(&lp->Dma);
		return IRQ_HANDLED;
	}

	if ((irq_status & (XLLDMA_IRQ_DELAY_MASK | XLLDMA_IRQ_COALESCE_MASK))) {
		spin_lock_irqsave(&receivedQueueSpin, flags);
		list_for_each(cur_lp, &receivedQueue) {
			if (cur_lp == &(lp->rcv)) {
				break;
			}
		}
		if (cur_lp != &(lp->rcv)) {
			list_add_tail(&lp->rcv, &receivedQueue);
			XLlDma_mBdRingIntDisable(&lp->Dma.RxBdRing,
						 XLLDMA_CR_IRQ_ALL_EN_MASK);
			tasklet_schedule(&DmaRecvBH);
		}
		spin_unlock_irqrestore(&receivedQueueSpin, flags);
	}
	return IRQ_HANDLED;
}

static irqreturn_t xenet_dma_tx_interrupt(int irq, void *dev_id)
{
	u32 irq_status;
	struct net_device *dev = dev_id;
	struct net_local *lp = netdev_priv(dev);
	struct list_head *cur_lp;

	unsigned int flags;

	/* Read pending interrupts */
	irq_status = XLlDma_mBdRingGetIrq(&(lp->Dma.TxBdRing));

	XLlDma_mBdRingAckIrq(&(lp->Dma.TxBdRing), irq_status);

	if ((irq_status & XLLDMA_IRQ_ALL_ERR_MASK)) {
		XLlDma_Reset(&lp->Dma);
		return IRQ_HANDLED;
	}

	if ((irq_status & (XLLDMA_IRQ_DELAY_MASK | XLLDMA_IRQ_COALESCE_MASK))) {
		spin_lock_irqsave(&sentQueueSpin, flags);
		list_for_each(cur_lp, &sentQueue) {
			if (cur_lp == &(lp->xmit)) {
 				break;
			}
		}
		if (cur_lp != &(lp->xmit)) {
			list_add_tail(&lp->xmit, &sentQueue);
			XLlDma_mBdRingIntDisable(&lp->Dma.TxBdRing,
						 XLLDMA_CR_IRQ_ALL_EN_MASK);
			tasklet_schedule(&DmaSendBH);
		}
		spin_unlock_irqrestore(&sentQueueSpin, flags);
	}
	return IRQ_HANDLED;
}

/*
 * Q:
 * Why doesn't this linux driver use an interrupt handler for the TEMAC itself?
 *
 * A:
 * Let's take a look at all the possible events that could be signaled by the
 * TEMAC core.
 *
 * possible events:
 *    Transmit Complete (TxCmplt) [not handled by this driver]
 *        The TEMAC TxCmplt interrupt status is ignored by software in favor of
 *        paying attention to the transmit complete status in the connected DMA
 *        or FIFO core.
 *    Receive Fifo Overflow (RxFifoOver) [not handled by this driver]
 *        We have discovered that the overhead of an interrupt context switch
 *        to attempt to handle this sort of event actually worsens the
 *        condition, and cuases further dropped packets further increasing the
 *        time spent in this interrupt handler.
 *    Receive Frame Rejected (RxRject) [not handled by this driver]
 *        We could possibly handle this interrupt and gather statistics
 *        information based on these events that occur. However it is not that
 *        critical.
 *    Receive Complete (RxCmplt) [not handled by this driver]
 *        The TEMAC RxCmplt interrupt status is ignored by software in favor of
 *        paying attention to the receive complete status in the connected DMA
 *        or FIFO core.
 *    Autonegotiaion Complete (AutoNeg) [not handled by this driver]
 *        Autonegotiation on the TEMAC is a bit complicated, and is handled in
 *        a way that does not require the use of this interrupt event.
 *    Hard Temac Core Access Complete (HardAcsCmplt) [not handled by this driver]
 *        This event really just indicates if there are any events in the TIS
 *        register. As can be seen below, none of the events from the TIS
 *        register are handled, so there is no need to handle this event
 *        either.
 *    Configuration Write Complete (CfgWst) [not handled by this driver]
 *    Configuration Read Complete (CfgRst) [not handled by this driver]
 *    Address Filter Write Complete (AfWst) [not handled by this driver]
 *    Address Filter Read Complete (AfRst) [not handled by this driver]
 *    MII Management Write Complete (MiimWst) [not handled by this driver]
 *    MII Management Read Complete (MiimRst) [not handled by this driver]
 *    Fabric Read Complete (FabrRst) [not handled by this driver]
 *        All of the above registers indicate when access (read or write) to
 *        one or other of the Hard Temac Core registers is complete. Instead of
 *        relying on an interrupt context switch to be notified that the access
 *        is complete, this driver instead polls for the status, which, in most
 *        cases, should be faster.
 */

static int xenet_open(struct net_device *dev)
{
	struct net_local *lp;
	u32 Options;
	int irqval = 0;

	/*
	 * Just to be safe, stop TX queue and the device first.  If the device is
	 * already stopped, an error will be returned.  In this case, we don't
	 * really care.
	 */
	netif_stop_queue(dev);
	lp = netdev_priv(dev);
	_XLlTemac_Stop(&lp->Emac);

	INIT_LIST_HEAD(&(lp->rcv));
	INIT_LIST_HEAD(&(lp->xmit));

	/* Set the MAC address each time opened. */
	if (_XLlTemac_SetMacAddress(&lp->Emac, dev->dev_addr) != XST_SUCCESS) {
		printk(KERN_ERR "%s: labx_eth_llink: could not set MAC address.\n",
		       dev->name);
		return -EIO;
	}

	/*
	 * If the device is not configured for polled mode, connect to the
	 * interrupt controller and enable interrupts.  Currently, there
	 * isn't any code to set polled mode, so this check is probably
	 * superfluous.
	 */
	Options = labx_XLlTemac_GetOptions(&lp->Emac);
	Options |= XTE_FLOW_CONTROL_OPTION;
	Options |= XTE_JUMBO_OPTION;
	Options |= XTE_TRANSMITTER_ENABLE_OPTION;
	Options |= XTE_RECEIVER_ENABLE_OPTION;
#if XTE_AUTOSTRIPPING
	Options |= XTE_FCS_STRIP_OPTION;
#endif

	(int) _XLlTemac_SetOptions(&lp->Emac, Options);
	(int) _XLlTemac_ClearOptions(&lp->Emac, ~Options);
	Options = labx_XLlTemac_GetOptions(&lp->Emac);
	printk(KERN_INFO "%s: labx_eth_llink: Options: 0x%x\n", dev->name, Options);

	if (XLlTemac_IsDma(&lp->Emac)) {
		printk(KERN_INFO
		       "%s: labx_eth_llink: allocating interrupt %d for dma mode tx.\n",
		       dev->name, lp->dma_irq_s);
		irqval = request_irq(lp->dma_irq_s,
			&xenet_dma_tx_interrupt, 0, "xilinx_dma_tx_int", dev);
		if (irqval) {
			printk(KERN_ERR
			       "%s: labx_eth_llink: could not allocate interrupt %d.\n",
			       dev->name, lp->dma_irq_s);
			return irqval;
		}
		printk(KERN_INFO
		       "%s: labx_eth_llink: allocating interrupt %d for dma mode rx.\n",
		       dev->name, lp->dma_irq_r);
		irqval = request_irq(lp->dma_irq_r,
			&xenet_dma_rx_interrupt, 0, "xilinx_dma_rx_int", dev);
		if (irqval) {
			printk(KERN_ERR
			       "%s: labx_eth_llink: could not allocate interrupt %d.\n",
			       dev->name, lp->dma_irq_r);
			return irqval;
		}
	} else {
		printk(KERN_INFO
		       "%s: labx_eth_llink: allocating interrupt %d for fifo mode.\n",
		       dev->name, lp->fifo_irq);

		/* With the way interrupts are issued on the fifo core, this needs to be
		 * fast interrupt handler.
		 */
		snprintf(lp->fifo_irq_name, IRQ_NAME_SZ, "%s FIFO", dev->name);
		lp->fifo_irq_name[IRQ_NAME_SZ - 1] = '\0';
		irqval = request_irq(lp->fifo_irq,
			&xenet_fifo_interrupt, IRQF_DISABLED, lp->fifo_irq_name, dev);
		if (irqval) {
			printk(KERN_ERR
			       "%s: labx_eth_llink: could not allocate interrupt %d.\n",
			       dev->name, lp->fifo_irq);
			return irqval;
		}
	}

	/* Perform PHY setup using the platform-supplied hook method */
	printk("%s: About to connect (if needed) to phy device\n",__func__);
	if (lp->phy_dev == NULL) {
		/* Lookup phy device */
	     	lp->phy_dev = phy_connect(lp->ndev, lp->phy_name, &labx_eth_ll_mac_adjust_link,
			0, PHY_INTERFACE_MODE_MII);
		if(!IS_ERR(lp->phy_dev)) {
			int ret;
			printk("%s: About to call phy_start_aneg()\n",__func__);
			ret = phy_start_aneg(lp->phy_dev);
			if (0 != ret) {
				printk("%s: phy_start_aneg() Failed with code %d\n",__func__,ret);
			} else {
				printk("%s: phy_start_aneg() Passed\n",__func__);
			}
		} else {
			printk("Not able to find Phy");
			lp->phy_dev = NULL;
		}
	}

	/* Enable interrupts  - no polled mode */
	if (XLlTemac_IsFifo(&lp->Emac)) { /* fifo direct interrupt driver mode */
		XLlFifo_IntEnable(&lp->Fifo, XLLF_INT_TC_MASK |
			XLLF_INT_RC_MASK | XLLF_INT_RXERROR_MASK |
			XLLF_INT_TXERROR_MASK);
	} else {		/* SG DMA mode */
		XLlDma_mBdRingIntEnable(&lp->Dma.RxBdRing, dma_rx_int_mask);
		XLlDma_mBdRingIntEnable(&lp->Dma.TxBdRing, dma_tx_int_mask);
	}

	/* Start TEMAC device */
	_XLlTemac_Start(&lp->Emac);
	if (XLlTemac_IsDma(&lp->Emac)) {
		u32 threshold_s, timer_s, threshold_r, timer_r;

		XLlDma_BdRingGetCoalesce(&lp->Dma.TxBdRing, &threshold_s, &timer_s);
		XLlDma_BdRingGetCoalesce(&lp->Dma.RxBdRing, &threshold_r, &timer_r);
		printk(KERN_INFO
		       "%s: labx_eth_llink: Send Threshold = %d, Receive Threshold = %d\n",
		       dev->name, threshold_s, threshold_r);
		printk(KERN_INFO
		       "%s: labx_eth_llink: Send Wait bound = %d, Receive Wait bound = %d\n",
		       dev->name, timer_s, timer_r);
		if (XLlDma_BdRingStart(&lp->Dma.TxBdRing) == XST_FAILURE) {
			printk(KERN_ERR "%s: labx_eth_llink: could not start dma tx channel\n", dev->name);
			return -EIO;
		}
		if (XLlDma_BdRingStart(&lp->Dma.RxBdRing) == XST_FAILURE) {
			printk(KERN_ERR "%s: labx_eth_llink: could not start dma rx channel\n", dev->name);
			return -EIO;
		}
	}

	/* We're ready to go. */
	netif_start_queue(dev);

	return 0;
}

static int xenet_close(struct net_device *dev)
{
	unsigned long flags;

	struct net_local *lp = netdev_priv(dev);

	/* Stop Send queue */
	netif_stop_queue(dev);

	/* Now we could stop the device */
	_XLlTemac_Stop(&lp->Emac);

	/*
	 * Free the interrupt - not polled mode.
	 */
	if (XLlTemac_IsDma(&lp->Emac)) {
		free_irq(lp->dma_irq_s, dev);
		free_irq(lp->dma_irq_r, dev);
	} else {
		free_irq(lp->fifo_irq, dev);
	}

	spin_lock_irqsave(&receivedQueueSpin, flags);
	list_del(&(lp->rcv));
	spin_unlock_irqrestore(&receivedQueueSpin, flags);

	spin_lock_irqsave(&sentQueueSpin, flags);
	list_del(&(lp->xmit));
	spin_unlock_irqrestore(&sentQueueSpin, flags);

	return 0;
}

static struct net_device_stats *xenet_get_stats(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);

	return &lp->stats;
}

static void xenet_set_multicast_list(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);

	u32 Options = labx_XLlTemac_GetOptions(&lp->Emac);

  if (dev->flags&(IFF_ALLMULTI|IFF_PROMISC) || dev->mc_count > 6)
  {
    dev->flags |= IFF_PROMISC;
		Options |= XTE_PROMISC_OPTION;
	}
	else
	{
		Options &= ~XTE_PROMISC_OPTION;
	}

	//printk("Options: %08X, dev->flags %08X\n", Options, dev->flags);
	_XLlTemac_Stop(&lp->Emac);
	(int) _XLlTemac_SetOptions(&lp->Emac, Options);
	(int) _XLlTemac_ClearOptions(&lp->Emac, ~Options);
  
  labx_eth_UpdateMacFilters(&lp->Emac);
  
	if (dev->mc_count > 0 && dev->mc_count <= 6)
	{
		// TODO: Program multicast filters
	}

	if (dev->flags & IFF_UP)
	{ 
		_XLlTemac_Start(&lp->Emac);

	}
}

static int xenet_set_mac_address(struct net_device *dev, void *p)
{
	struct net_local *lp = netdev_priv(dev);
	struct sockaddr * addr = p;
	int err = 0;

	if (!is_valid_ether_addr(addr->sa_data))
                return -EINVAL;

        memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	_XLlTemac_Stop(&lp->Emac);

	if (_XLlTemac_SetMacAddress(&lp->Emac, dev->dev_addr) != XST_SUCCESS) {
		printk(KERN_ERR "labx_eth_llink: could not set MAC address.\n");
		err = -EIO;
	}

	if (dev->flags & IFF_UP)
	{
		_XLlTemac_Start(&lp->Emac);
	}

	return err;
}

static int xenet_change_mtu(struct net_device *dev, int new_mtu)
{
#ifdef CONFIG_XILINX_GIGE_VLAN
	int head_size = XTE_HDR_VLAN_SIZE;
#else
	int head_size = XTE_HDR_SIZE;
#endif
	struct net_local *lp = netdev_priv(dev);
	int max_frame = new_mtu + head_size + XTE_TRL_SIZE;
	int min_frame = 1 + head_size + XTE_TRL_SIZE;

	if ((max_frame < min_frame) || (max_frame > lp->max_frame_size))
		return -EINVAL;

	dev->mtu = new_mtu;	/* change mtu in net_device structure */
	return 0;
}

static int xenet_FifoSend(struct sk_buff *skb, struct net_device *dev)
{
	struct net_local *lp;
	unsigned long flags, fifo_free_bytes;
	int total_frags = skb_shinfo(skb)->nr_frags + 1;
	unsigned int total_len;
	skb_frag_t *frag;
	int i;
	void *virt_addr;
	
	total_len = skb_headlen(skb);

	frag = &skb_shinfo(skb)->frags[0];
	for (i = 1; i < total_frags; i++, frag++) {
		total_len += frag->size;
	}

	/* The following lock is used to protect TxVacancy, Write
	 * and TxSetLen sequence which could happen from FifoSendHandler
	 * or other processor in SMP case.
	 */
	spin_lock_irqsave(&XTE_tx_spinlock, flags);
	lp = netdev_priv(dev);

	fifo_free_bytes = XLlFifo_TxVacancy(&lp->Fifo) * 4;
	if (fifo_free_bytes < total_len) {
		netif_stop_queue(dev);	/* stop send queue */
		lp->deferred_skb = skb;	/* buffer the sk_buffer and will send
					   it in interrupt context */
		spin_unlock_irqrestore(&XTE_tx_spinlock, flags);
		return 0;
	}

	/* Write frame data to FIFO */
	XLlFifo_Write(&lp->Fifo, (void *) skb->data, skb_headlen(skb));

	frag = &skb_shinfo(skb)->frags[0];
	for (i = 1; i < total_frags; i++, frag++) {
		virt_addr =
			(void *) page_address(frag->page) + frag->page_offset;
		XLlFifo_Write(&lp->Fifo, virt_addr, frag->size);
	}

	/* Initiate transmit */
	XLlFifo_TxSetLen(&lp->Fifo, total_len);
	lp->stats.tx_bytes += total_len;
	spin_unlock_irqrestore(&XTE_tx_spinlock, flags);

	dev_kfree_skb(skb);	/* free skb */
	dev->trans_start = jiffies;
	return 0;
}

/* Callback function for completed frames sent in FIFO interrupt driven mode */
static void FifoSendHandler(struct net_device *dev)
{
	struct net_local *lp;
	struct sk_buff *skb;
	unsigned int flags;

	spin_lock_irqsave(&XTE_tx_spinlock, flags);
	lp = netdev_priv(dev);
	if (lp->cur_speed == 10000) printk("Yi Cao: tx packets: %d \n", lp->stats.tx_packets);
	lp->stats.tx_packets++;

	/*Send out the deferred skb and wake up send queue if a deferred skb exists */
	if (lp->deferred_skb) {
		int total_frags;
		unsigned int total_len;
		unsigned long fifo_free_bytes;
		skb_frag_t *frag;
		int i;
		void *virt_addr;

		skb = lp->deferred_skb;
		total_frags = skb_shinfo(skb)->nr_frags + 1;
		total_len = skb_headlen(skb);

		frag = &skb_shinfo(skb)->frags[0];
		for (i = 1; i < total_frags; i++, frag++) {
			total_len += frag->size;
		}

		fifo_free_bytes = XLlFifo_TxVacancy(&lp->Fifo) * 4;
		if (fifo_free_bytes < total_len) {
  		
  		if(lp->cur_speed == 10000) printk("Yi Cao: fifo_free_bytes < total_len.....\n");
			/* If still no room for the deferred packet, return */
			spin_unlock_irqrestore(&XTE_tx_spinlock, flags);
			return;
		}

		/* Write frame data to FIFO */
		XLlFifo_Write(&lp->Fifo, (void *) skb->data, skb_headlen(skb));

		frag = &skb_shinfo(skb)->frags[0];
		for (i = 1; i < total_frags; i++, frag++) {
			virt_addr =
				(void *) page_address(frag->page) + frag->page_offset;
			XLlFifo_Write(&lp->Fifo, virt_addr, frag->size);
		}

		/* Initiate transmit */
		XLlFifo_TxSetLen(&lp->Fifo, total_len);

		dev_kfree_skb(skb);	/* free skb */
		lp->deferred_skb = NULL;
		lp->stats.tx_packets++;
		if(lp->cur_speed == 10000) printk("Yi Cao: tx packets at the end: %d\n", lp->stats.tx_packets);
		lp->stats.tx_bytes += total_len;
		dev->trans_start = jiffies;
		netif_wake_queue(dev);	/* wake up send queue */
	}
	spin_unlock_irqrestore(&XTE_tx_spinlock, flags);
}

#if 0
/*
 * These are used for debugging purposes, left here in case they are useful
 * for further debugging
 */
static unsigned int _xenet_tx_csum(struct sk_buff *skb)
{
	unsigned int csum = 0;
	long csstart = skb_transport_header(skb) - skb->data;

	if (csstart != skb->len) {
		csum = skb_checksum(skb, csstart, skb->len - csstart, 0);
	}

	return csum;
}

static inline unsigned int _xenet_rx_csum(struct sk_buff *skb)
{
	return skb_checksum(skb, 0, skb->len, 0);
}
#endif

/*
 * xenet_DmaSend_internal is an internal use, send routine.
 * Any locks that need to be acquired, should be acquired
 * prior to calling this routine.
 */
static int xenet_DmaSend_internal(struct sk_buff *skb, struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	XLlDma_Bd *bd_ptr;
	int result;
	int total_frags;
	int i;
	void *virt_addr;
	size_t len;
	dma_addr_t phy_addr;
	XLlDma_Bd *first_bd_ptr;
	XLlDma_Bd *last_bd_ptr;
	skb_frag_t *frag;

	/* get skb_shinfo(skb)->nr_frags + 1 buffer descriptors */
	total_frags = skb_shinfo(skb)->nr_frags + 1;

	/* stats */
	if (lp->max_frags_in_a_packet < total_frags) {
		lp->max_frags_in_a_packet = total_frags;
	}

	if (total_frags < XTE_SEND_BD_CNT) {
		result = XLlDma_BdRingAlloc(&lp->Dma.TxBdRing, total_frags,
					    &bd_ptr);

		if (result != XST_SUCCESS) {
			netif_stop_queue(dev);	/* stop send queue */
			lp->deferred_skb = skb;	/* buffer the sk_buffer and will send
						   it in interrupt context */
			return result;
		}
	} else {
		dev_kfree_skb(skb);
		lp->stats.tx_dropped++;
		printk(KERN_ERR
		       "%s: labx_eth_llink: could not send TX socket buffers (too many fragments).\n",
		       dev->name);
		return XST_FAILURE;
	}

	len = skb_headlen(skb);

	/* get the physical address of the header */
	phy_addr = (u32) dma_map_single(NULL, skb->data, len, DMA_TO_DEVICE);

	/* get the header fragment, it's in the skb differently */
	XLlDma_mBdSetBufAddr(bd_ptr, phy_addr);
	XLlDma_mBdSetLength(bd_ptr, len);
	XLlDma_mBdSetId(bd_ptr, skb);

	/*
	 * if tx checksum offloading is enabled, when the ethernet stack
	 * wants us to perform the checksum in hardware,
	 * skb->ip_summed is CHECKSUM_PARTIAL. Otherwise skb->ip_summed is
	 * CHECKSUM_NONE, meaning the checksum is already done, or
	 * CHECKSUM_UNNECESSARY, meaning checksumming is turned off (e.g.
	 * loopback interface)
	 *
	 * skb->csum is an overloaded value. On send, skb->csum is the offset
	 * into the buffer (skb_transport_header(skb)) to place the csum value.
	 * On receive this feild gets set to the actual csum value, before it's
	 * passed up the stack.
	 *
	 * When we get here, the ethernet stack above will have already
	 * computed the pseudoheader csum value and have placed it in the
	 * TCP/UDP header.
	 *
	 * The IP header csum has also already been computed and inserted.
	 *
	 * Since the IP header with it's own csum should compute to a null
	 * csum, it should be ok to include it in the hw csum. If it is decided
	 * to change this scheme, skb should be examined before dma_map_single()
	 * is called, which flushes the page from the cpu's cache.
	 *
	 * skb->data points to the beginning of the whole packet
	 * skb_transport_header(skb) points to the beginning of the ip header
	 *
	 */
	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		unsigned int csum_start_off = 0/*skb_transport_offset(skb)*/;
		unsigned int csum_index_off = csum_start_off + skb->csum_offset;

#if 0
		{
			unsigned int csum = _xenet_tx_csum(skb);

			*((unsigned short *) (raw + skb->csum)) =
				csum_fold(csum);
			BdCsumDisable(bd_ptr);
		}
#else
		BdCsumEnable(bd_ptr);
		BdCsumSetup(bd_ptr, csum_start_off,
			csum_index_off);
#endif
		lp->tx_hw_csums++;
	}
	else {
		/*
		 * This routine will do no harm even if hardware checksum capability is
		 * off.
		 */
		BdCsumDisable(bd_ptr);
	}

	first_bd_ptr = bd_ptr;
	last_bd_ptr = bd_ptr;

	frag = &skb_shinfo(skb)->frags[0];

	for (i = 1; i < total_frags; i++, frag++) {
		bd_ptr = XLlDma_mBdRingNext(&lp->Dma.TxBdRing, bd_ptr);
		last_bd_ptr = bd_ptr;

		virt_addr =
			(void *) page_address(frag->page) + frag->page_offset;
		phy_addr =
			(u32) dma_map_single(NULL, virt_addr, frag->size,
					     DMA_TO_DEVICE);

		XLlDma_mBdSetBufAddr(bd_ptr, phy_addr);
		XLlDma_mBdSetLength(bd_ptr, frag->size);
		XLlDma_mBdSetId(bd_ptr, NULL);
		BdCsumDisable(bd_ptr);
		XLlDma_mBdSetStsCtrl(bd_ptr, 0);
	}

	if (first_bd_ptr == last_bd_ptr) {
		XLlDma_mBdSetStsCtrl(last_bd_ptr,
				     XLLDMA_BD_STSCTRL_SOP_MASK |
				     XLLDMA_BD_STSCTRL_EOP_MASK);
	} else {
		XLlDma_mBdSetStsCtrl(first_bd_ptr, XLLDMA_BD_STSCTRL_SOP_MASK);
		XLlDma_mBdSetStsCtrl(last_bd_ptr, XLLDMA_BD_STSCTRL_EOP_MASK);
	}


	/* Enqueue to HW */
	result = XLlDma_BdRingToHw(&lp->Dma.TxBdRing, total_frags,
				   first_bd_ptr);
	if (result != XST_SUCCESS) {
		netif_stop_queue(dev);	/* stop send queue */
		dev_kfree_skb(skb);
		XLlDma_mBdSetId(first_bd_ptr, NULL);
		lp->stats.tx_dropped++;
		printk(KERN_ERR
		       "%s: labx_eth_llink: could not send commit TX buffer descriptor (%d).\n",
		       dev->name, result);
		reset(dev, __LINE__);

		return XST_FAILURE;
	}

	dev->trans_start = jiffies;

	return XST_SUCCESS;
}

/* The send function for frames sent in DMA mode */
static int xenet_DmaSend(struct sk_buff *skb, struct net_device *dev)
{
	/* The following spin_lock protects
	 * SgAlloc, SgCommit sequence, which also exists in DmaSendHandlerBH Bottom
	 * Half, or triggered by other processor in SMP case.
	 */
	spin_lock_bh(&XTE_tx_spinlock);

	xenet_DmaSend_internal(skb, dev);

	spin_unlock_bh(&XTE_tx_spinlock);

	return 0;
}


static void DmaSendHandlerBH(unsigned long p)
{
	struct net_device *dev;
	struct net_local *lp;
	XLlDma_Bd *BdPtr, *BdCurPtr;
	unsigned long len;
	unsigned long flags;
	struct sk_buff *skb;
	dma_addr_t skb_dma_addr;
	int result = XST_SUCCESS;
	unsigned int bd_processed, bd_processed_save;

	while (1) {
		spin_lock_irqsave(&sentQueueSpin, flags);
		if (list_empty(&sentQueue)) {
			spin_unlock_irqrestore(&sentQueueSpin, flags);
			break;
		}

		lp = list_entry(sentQueue.next, struct net_local, xmit);

		list_del_init(&(lp->xmit));
		spin_unlock_irqrestore(&sentQueueSpin, flags);

		spin_lock_irqsave(&XTE_tx_spinlock, flags);
		dev = lp->ndev;
		bd_processed_save = 0;
		while ((bd_processed =
			XLlDma_BdRingFromHw(&lp->Dma.TxBdRing, XTE_SEND_BD_CNT,
					    &BdPtr)) > 0) {

			bd_processed_save = bd_processed;
			BdCurPtr = BdPtr;
			do {
				len = XLlDma_mBdGetLength(BdCurPtr);
				skb_dma_addr = (dma_addr_t) XLlDma_mBdGetBufAddr(BdCurPtr);
				dma_unmap_single(NULL, (void *)skb_dma_addr, len,
						 DMA_TO_DEVICE);

				/* get ptr to skb */
				skb = (struct sk_buff *)
					XLlDma_mBdGetId(BdCurPtr);
				if (skb)
					dev_kfree_skb(skb);

				/* reset BD id */
				XLlDma_mBdSetId(BdCurPtr, NULL);

				lp->stats.tx_bytes += len;
				if (XLlDma_mBdGetStsCtrl(BdCurPtr) & XLLDMA_BD_STSCTRL_EOP_MASK) {
					lp->stats.tx_packets++;
				}

				BdCurPtr = XLlDma_mBdRingNext(&lp->Dma.TxBdRing, BdCurPtr);
				bd_processed--;
			} while (bd_processed > 0);

			result = XLlDma_BdRingFree(&lp->Dma.TxBdRing,
						   bd_processed_save, BdPtr);
			if (result != XST_SUCCESS) {
				printk(KERN_ERR
				       "%s: XLlDma: BdRingFree() error %d.\n",
				       dev->name, result);
				reset(dev, __LINE__);
				spin_unlock_irqrestore(&XTE_tx_spinlock, flags);
				return;
			}
		}
		XLlDma_mBdRingIntEnable(&lp->Dma.TxBdRing, dma_tx_int_mask);

		/* Send out the deferred skb if it exists */
		if ((lp->deferred_skb) && bd_processed_save) {
			skb = lp->deferred_skb;
			lp->deferred_skb = NULL;

			result = xenet_DmaSend_internal(skb, dev);
		}

		if (result == XST_SUCCESS) {
			netif_wake_queue(dev);	/* wake up send queue */
		}
		spin_unlock_irqrestore(&XTE_tx_spinlock, flags);
	}
}

static void xenet_tx_timeout(struct net_device *dev)
{
	struct net_local *lp;
	unsigned long flags;

	/*
	 * Make sure that no interrupts come in that could cause reentrancy
	 * problems in reset.
	 */
	spin_lock_irqsave(&XTE_tx_spinlock, flags);

	lp = netdev_priv(dev);
	printk(KERN_ERR
	       "%s: labx_eth_llink: exceeded transmit timeout of %lu ms.  Resetting emac.\n",
	       dev->name, TX_TIMEOUT * 1000UL / HZ);
	lp->stats.tx_errors++;

	reset(dev, __LINE__);

	spin_unlock_irqrestore(&XTE_tx_spinlock, flags);
}

/* The callback function for frames received when in FIFO mode. */
static void FifoRecvHandler(unsigned long p)
{
	struct net_local *lp;
	struct sk_buff *skb;
	u32 len;

	struct net_device *dev;
	unsigned long flags;
	spin_lock_irqsave(&receivedQueueSpin, flags);
	if (list_empty(&receivedQueue)) {
		spin_unlock_irqrestore(&receivedQueueSpin, flags);
		return;
	}
	lp = list_entry(receivedQueue.next, struct net_local, rcv);

	list_del_init(&(lp->rcv));
	spin_unlock_irqrestore(&receivedQueueSpin, flags);
	dev = lp->ndev;

	while (XLlFifo_RxOccupancy(&lp->Fifo) != 0) {
		len = XLlFifo_RxGetLen(&lp->Fifo);

		/*
		 * TODO: Hm this is odd, if we can't allocate the skb, we throw away the next packet. Why?
		 */
		if (!(skb = /*dev_ */ alloc_skb(len + ALIGNMENT_RECV, GFP_ATOMIC))) {
#define XTE_RX_SINK_BUFFER_SIZE 1024
			static u32 rx_buffer_sink[XTE_RX_SINK_BUFFER_SIZE / sizeof(u32)];

			/* Couldn't get memory. */
			lp->stats.rx_dropped++;
			printk(KERN_ERR
			       "%s: labx_eth_llink: could not allocate receive buffer.\n",
			       dev->name);

			/* consume data in Xilinx TEMAC RX data fifo so it is sync with RX length fifo */
			for (; len > XTE_RX_SINK_BUFFER_SIZE;
					len -= XTE_RX_SINK_BUFFER_SIZE) {
				XLlFifo_Read(&lp->Fifo, rx_buffer_sink,
					       XTE_RX_SINK_BUFFER_SIZE);
			}
			XLlFifo_Read(&lp->Fifo, rx_buffer_sink, len);
			break;
		}

		/* Read the packet data */
		XLlFifo_Read(&lp->Fifo, skb->data, len);
		if(lp->cur_speed == 10000) printk("Yi Cao: rx packets: %d\n", lp->stats.rx_packets);
		lp->stats.rx_packets++;
		lp->stats.rx_bytes += len;
#if 0
		printk("Got %d Rx:\n", len);
		{
		  int idx;
		  for(idx = 0; idx < len; idx++) {
		    printk("%02X ", skb->data[idx]);
		    if((idx % 16) == 15) printk("\n");
		  }
		  printk("\n");
		}
#endif
		skb_put(skb, len);	/* Tell the skb how much data we got. */
		skb->dev = dev;		/* Fill out required meta-data. */
		skb->protocol = eth_type_trans(skb, dev);
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		netif_rx(skb);		/* Send the packet up the stack */
	}
	XLlFifo_IntEnable(&lp->Fifo, XLLF_INT_TC_MASK | XLLF_INT_RC_MASK |
			XLLF_INT_RXERROR_MASK | XLLF_INT_TXERROR_MASK);

}


static int
labx_ethtool_get_settings(struct net_device *dev, struct ethtool_cmd *ecmd)
{
  struct net_local *lp = netdev_priv(dev);
  u32 mac_options;
  u16 gmii_cmd, gmii_status, gmii_advControl;

  memset(ecmd, 0, sizeof(struct ethtool_cmd));

  /* Check to be sure we found a PHY */
  if (NULL == lp->phy_dev) {
    return -ENODEV;
  }

  mac_options = labx_XLlTemac_GetOptions(&(lp->Emac));
  gmii_cmd = phy_read(lp->phy_dev, MII_BMCR);
  gmii_status = phy_read(lp->phy_dev, MII_BMSR);
  gmii_advControl = phy_read(lp->phy_dev, MII_ADVERTISE);

  ecmd->duplex = DUPLEX_FULL;

  ecmd->supported |= SUPPORTED_MII;
  ecmd->supported |= SUPPORTED_100baseT_Full;
  ecmd->supported |= SUPPORTED_1000baseT_Full;
  ecmd->supported |= SUPPORTED_Autoneg;

  ecmd->port = PORT_MII;

  ecmd->speed = lp->cur_speed;

  if (gmii_status & BMSR_ANEGCAPABLE) {
    ecmd->supported |= SUPPORTED_Autoneg;
  }
  if (gmii_status & BMSR_ANEGCOMPLETE) {
    ecmd->autoneg = AUTONEG_ENABLE;
    ecmd->advertising |= ADVERTISED_Autoneg;
  }
  else {
    ecmd->autoneg = AUTONEG_DISABLE;
  }
  ecmd->phy_address = lp->Emac.Config.BaseAddress;
  ecmd->transceiver = XCVR_INTERNAL;
  ecmd->supported |= SUPPORTED_10baseT_Full | SUPPORTED_100baseT_Full |
    SUPPORTED_1000baseT_Full | SUPPORTED_Autoneg;

  return 0;
}

static int
labx_ethtool_set_settings(struct net_device *dev, struct ethtool_cmd *ecmd)
{
  struct net_local *lp = netdev_priv(dev);

  if ((ecmd->duplex != DUPLEX_FULL) ||
      (ecmd->transceiver != XCVR_INTERNAL) ||
      (ecmd->phy_address &&
       (ecmd->phy_address != lp->Emac.Config.BaseAddress))) {
    return -EOPNOTSUPP;
  }

  if ((ecmd->speed != 1000) && (ecmd->speed != 100) &&
      (ecmd->speed != 10)) {
    printk(KERN_ERR
	   "%s: labx_ethernet: labx_ethtool_set_settings speed not supported: %d\n",
	   dev->name, ecmd->speed);
    return -EOPNOTSUPP;
  }

  return 0;
}

#define EMAC_REGS_N 32
struct mac_regsDump {
  struct ethtool_regs hd;
  u16 data[EMAC_REGS_N];
};

static void
labx_ethtool_get_regs(struct net_device *dev, struct ethtool_regs *regs,
		      void *ret)
{
  struct net_local *lp = netdev_priv(dev);
  struct mac_regsDump *dump = (struct mac_regsDump *) regs;
  int i;

  if (NULL == lp->phy_dev) {
    *(int*)ret = -ENODEV;
    return;
  }

  dump->hd.version = 0;
  dump->hd.len = sizeof(dump->data);
  memset(dump->data, 0, sizeof(dump->data));

  for (i = 0; i < EMAC_REGS_N; i++) {
    dump->data[i] = phy_read(lp->phy_dev, i);
  }

  *(int *) ret = 0;
}

static void
labx_ethtool_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *ed)
{
  memset(ed, 0, sizeof(struct ethtool_drvinfo));
  strncpy(ed->driver, DRIVER_NAME, sizeof(ed->driver) - 1);
  strncpy(ed->version, DRIVER_VERSION, sizeof(ed->version) - 1);

  /* Also tell how much memory is needed for dumping register values */
  ed->regdump_len = sizeof(u16) * EMAC_REGS_N;
}

/* Array defining all of the statistics we can return */
/*
 * ethtool has a status reporting feature where we can report any sort of
 * status information we'd like. This is the list of strings used for that
 * status reporting. ETH_GSTRING_LEN is defined in ethtool.h
 */
#define RX_CRC_ERRS_INDEX  (0)
static char labx_ethernet_gstrings_stats[][ETH_GSTRING_LEN] = {
  "RxCrcErrors",
};

#define LABX_ETHERNET_STATS_LEN ARRAY_SIZE(labx_ethernet_gstrings_stats)

/* Array defining the test modes we support */
static const char labx_ethernet_gstrings_test[][ETH_GSTRING_LEN] = {
  "Local_Loopback"
};

#define LABX_ETHERNET_TEST_LEN	ARRAY_SIZE(labx_ethernet_gstrings_test)

static int labx_ethtool_get_sset_count(struct net_device *dev, int sset) {
  switch (sset) {
  case ETH_SS_TEST:
    return LABX_ETHERNET_TEST_LEN;
  case ETH_SS_STATS:
    return LABX_ETHERNET_STATS_LEN;
  default:
    return -EOPNOTSUPP;
  }
}

static void
labx_ethtool_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
  switch (stringset) {
  case ETH_SS_TEST:
    memcpy(data, *labx_ethernet_gstrings_test,
	   (LABX_ETHERNET_TEST_LEN * ETH_GSTRING_LEN));
    break;
  case ETH_SS_STATS:
    memcpy(data, *labx_ethernet_gstrings_stats,
	   (LABX_ETHERNET_STATS_LEN * ETH_GSTRING_LEN));
    break;
  }
}

static void
labx_ethtool_self_test(struct net_device *dev, struct ethtool_test *test_info, 
		       u64 *test_results) {
  struct net_local *lp = netdev_priv(dev);
  XLlTemac *InstancePtr = (XLlTemac *) &lp->Emac;
  u32 phy_test_mode;

  /* Clear the test results */
  memset(test_results, 0, (sizeof(uint64_t) * LABX_ETHERNET_TEST_LEN));

  /* We have co-opted this self-test ioctl for use as a means to put the
   * PHY into local loopback mode, or into other PHY-supported test modes.
   */
  if(lp && lp->phy_dev && lp->phy_dev->drv &&
        lp->phy_dev->drv->set_test_mode) {
    if(test_info->flags & ETH_TEST_FL_INT_LOOP) {
      phy_test_mode = PHY_TEST_INT_LOOP;
    } else if(test_info->flags & ETH_TEST_FL_EXT_LOOP) {
      phy_test_mode = PHY_TEST_EXT_LOOP;
    } else if(test_info->flags & ETH_TEST_FL_TX_WAVEFORM) {
      phy_test_mode = PHY_TEST_TX_WAVEFORM;
    } else if(test_info->flags & ETH_TEST_FL_MASTER_JITTER) {
      phy_test_mode = PHY_TEST_MASTER_JITTER;
    } else if(test_info->flags & ETH_TEST_FL_SLAVE_JITTER) {
      phy_test_mode = PHY_TEST_SLAVE_JITTER;
    } else if(test_info->flags & ETH_TEST_FL_TX_DISTORTION) {
      phy_test_mode = PHY_TEST_TX_DISTORTION;
    } else phy_test_mode = PHY_TEST_NONE;

    /* Enter the selected test mode */
    lp->phy_dev->drv->set_test_mode(lp->phy_dev, phy_test_mode);
  } else if(test_info->flags != 0) {
    /* Complain if any test mode is selected (not normal mode) */
    printk("%s PHY driver does not support test modes\n",
           (lp && lp->phy_dev && lp->phy_dev->drv && lp->phy_dev->drv->name) ?
           lp->phy_dev->drv->name : "<Unknown>");
  }

  /* Having switched modes, clear the hardware statistics counters,
   * if they exist
   */
  if((InstancePtr->versionReg & (REVISION_MINOR_MASK | REVISION_MAJOR_MASK)) >=
      RX_CRC_ERRS_MIN_VERSION) {
    XLlTemac_WriteReg(InstancePtr->Config.BaseAddress, BAD_PACKET_REG, 0);
  }
}

static void labx_ethtool_get_stats(struct net_device *dev, 
                                   struct ethtool_stats *stats, 
                                   u64 *data) {
  struct net_local *lp = netdev_priv(dev);
  XLlTemac *InstancePtr = (XLlTemac *) &lp->Emac;

  /* Copy each statistic into the data location for it.  At the moment, the
   * only statistic is hosted directly within a hardware register.
   */

  /* Fetch the CRC count from hardware, if supported */
  if(InstancePtr->versionReg >= RX_CRC_ERRS_MIN_VERSION) {
    data[RX_CRC_ERRS_INDEX] = XLlTemac_ReadReg(InstancePtr->Config.BaseAddress, BAD_PACKET_REG);
  } else data[RX_CRC_ERRS_INDEX] = 0ULL;
}

/* ethtool operations structure */
static const struct ethtool_ops labx_ethtool_ops = {
  .get_settings      = labx_ethtool_get_settings,
  .set_settings      = labx_ethtool_set_settings,
  .get_drvinfo       = labx_ethtool_get_drvinfo,
  .get_regs          = labx_ethtool_get_regs,
  .self_test         = labx_ethtool_self_test,
  .get_sset_count    = labx_ethtool_get_sset_count,
  .get_strings       = labx_ethtool_get_strings,
  .get_ethtool_stats = labx_ethtool_get_stats,
#if 0
  .get_link = ethtool_op_get_link,
  .nway_reset = labx_ethtool_nwayreset,
  .get_msglevel = labx_ethtool_getmsglevel,
  .set_msglevel = labx_ethtool_setmsglevel,
  .get_regs_len = labx_ethtool_getregslen,
  .get_regs = labx_ethtool_getregs,
  .get_eeprom_len = labx_ethtool_get_eeprom_len,
  .get_eeprom = labx_ethtool_get_eeprom,
  .set_eeprom = labx_ethtool_set_eeprom,
#endif
};

/*
 * _xenet_DmaSetupRecvBuffers allocates as many socket buffers (sk_buff's) as it
 * can up to the number of free RX buffer descriptors. Then it sets up the RX
 * buffer descriptors to DMA into the socket_buffers.
 *
 * The net_device, dev, indcates on which device to operate for buffer
 * descriptor allocation.
 */
static void _xenet_DmaSetupRecvBuffers(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);

	int free_bd_count = XLlDma_mBdRingGetFreeCnt(&lp->Dma.RxBdRing);
	int num_sk_buffs;
	struct sk_buff_head sk_buff_list;
	struct sk_buff *new_skb;
	u32 new_skb_baddr;
	XLlDma_Bd *BdPtr, *BdCurPtr;
	u32 align;
	int result;

#if 0
	int align_max = ALIGNMENT_RECV;
#else
	int align_max = 0;
#endif


	skb_queue_head_init(&sk_buff_list);
	for (num_sk_buffs = 0; num_sk_buffs < free_bd_count; num_sk_buffs++) {
		new_skb = alloc_skb(lp->max_frame_size + align_max, GFP_ATOMIC);
		if (new_skb == NULL) {
			break;
		}
		/*
		 * I think the XTE_spinlock, and Recv DMA int disabled will protect this
		 * list as well, so we can use the __ version just fine
		 */
		__skb_queue_tail(&sk_buff_list, new_skb);
	}
	if (!num_sk_buffs) {
		printk(KERN_ERR "%s: labx_eth_llink: alloc_skb unsuccessful\n",
		       dev->name);
		return;
	}

	/* now we got a bunch o' sk_buffs */
	result = XLlDma_BdRingAlloc(&lp->Dma.RxBdRing, num_sk_buffs, &BdPtr);
	if (result != XST_SUCCESS) {
		/* we really shouldn't get this */
		skb_queue_purge(&sk_buff_list);
		printk(KERN_ERR "%s: XLlDma: BdRingAlloc unsuccessful (%d)\n",
		       dev->name, result);
		reset(dev, __LINE__);
		return;
	}

	BdCurPtr = BdPtr;

	new_skb = skb_dequeue(&sk_buff_list);
	while (new_skb) {
		/* make sure we're long-word aligned */
		align = BUFFER_ALIGNRECV(new_skb->data);
		if (align) {
			skb_reserve(new_skb, align);
		}

		/* Get dma handle of skb->data */
		new_skb_baddr = (u32) dma_map_single(NULL, new_skb->data,
						     lp->max_frame_size,
						     DMA_FROM_DEVICE);

		XLlDma_mBdSetBufAddr(BdCurPtr, new_skb_baddr);
		XLlDma_mBdSetLength(BdCurPtr, lp->max_frame_size);
		XLlDma_mBdSetId(BdCurPtr, new_skb);
		XLlDma_mBdSetStsCtrl(BdCurPtr,
				     XLLDMA_BD_STSCTRL_SOP_MASK |
				     XLLDMA_BD_STSCTRL_EOP_MASK);

		BdCurPtr = XLlDma_mBdRingNext(&lp->Dma.RxBdRing, BdCurPtr);

		new_skb = skb_dequeue(&sk_buff_list);
	}

	/* enqueue RxBD with the attached skb buffers such that it is
	 * ready for frame reception */
	result = XLlDma_BdRingToHw(&lp->Dma.RxBdRing, num_sk_buffs, BdPtr);
	if (result != XST_SUCCESS) {
		printk(KERN_ERR
		       "%s: XLlDma: (DmaSetupRecvBuffers) BdRingToHw unsuccessful (%d)\n",
		       dev->name, result);
		skb_queue_purge(&sk_buff_list);
		BdCurPtr = BdPtr;
		while (num_sk_buffs > 0) {
			XLlDma_mBdSetId(BdCurPtr, NULL);
			BdCurPtr = XLlDma_mBdRingNext(&lp->Dma.RxBdRing,
						      BdCurPtr);
			num_sk_buffs--;
		}
		reset(dev, __LINE__);
		return;
	}
}

static void DmaRecvHandlerBH(unsigned long p)
{
	struct net_device *dev;
	struct net_local *lp;
	struct sk_buff *skb;
	u32 len, skb_baddr;
	int result;
	unsigned long flags;
	XLlDma_Bd *BdPtr, *BdCurPtr;
	unsigned int bd_processed, bd_processed_saved;

	while (1) {
		spin_lock_irqsave(&receivedQueueSpin, flags);
		if (list_empty(&receivedQueue)) {
			spin_unlock_irqrestore(&receivedQueueSpin, flags);
			break;
		}
		lp = list_entry(receivedQueue.next, struct net_local, rcv);

		list_del_init(&(lp->rcv));
		spin_unlock_irqrestore(&receivedQueueSpin, flags);
		dev = lp->ndev;

		spin_lock_irqsave(&XTE_rx_spinlock, flags);
		if ((bd_processed =
		     XLlDma_BdRingFromHw(&lp->Dma.RxBdRing, XTE_RECV_BD_CNT, &BdPtr)) > 0) {

			bd_processed_saved = bd_processed;
			BdCurPtr = BdPtr;
			do {
				/*
				 * Regular length field not updated on rx,
				 * USR4 updated instead.
				 */
				len = BdGetRxLen(BdCurPtr);

				/* get ptr to skb */
				skb = (struct sk_buff *)
					XLlDma_mBdGetId(BdCurPtr);

				/* get and free up dma handle used by skb->data */
				skb_baddr = (dma_addr_t) XLlDma_mBdGetBufAddr(BdCurPtr);
				dma_unmap_single(NULL, (void *)skb_baddr,
						 lp->max_frame_size,
						 DMA_FROM_DEVICE);

				/* reset ID */
				XLlDma_mBdSetId(BdCurPtr, NULL);

				/* setup received skb and send it upstream */
				skb_put(skb, len);	/* Tell the skb how much data we got. */
				skb->dev = dev;

				/* this routine adjusts skb->data to skip the header */
				skb->protocol = eth_type_trans(skb, dev);

				/* default the ip_summed value */
				skb->ip_summed = CHECKSUM_UNNECESSARY;

				/* if we're doing rx csum offload, set it up */
				if (((lp->local_features & LOCAL_FEATURE_RX_CSUM) != 0) &&
				    (skb->protocol == __constant_htons(ETH_P_IP)) &&
				    (skb->len > 64)) {
					unsigned int csum;

					/*
					 * This hardware only supports proper checksum calculations
					 * on TCP/UDP packets.
					 *
					 * skb->csum is an overloaded value. On send, skb->csum is
					 * the offset into the buffer (skb_transport_header(skb))
					 * to place the csum value. On receive this feild gets set
					 * to the actual csum value, before it's passed up the stack.
					 *
					 * If we set skb->ip_summed to CHECKSUM_COMPLETE, the ethernet
					 * stack above will compute the pseudoheader csum value and
					 * add it to the partial checksum already computed (to be
					 * placed in skb->csum) and verify it.
					 *
					 * Setting skb->ip_summed to CHECKSUM_NONE means that the
					 * cheksum didn't verify and the stack will (re)check it.
					 *
					 * Setting skb->ip_summed to CHECKSUM_UNNECESSARY means
					 * that the cheksum was verified/assumed to be good and the
					 * stack does not need to (re)check it.
					 *
					 * The ethernet stack above will (re)compute the checksum
					 * under the following conditions:
					 * 1) skb->ip_summed was set to CHECKSUM_NONE
					 * 2) skb->len does not match the length of the ethernet
					 *    packet determined by parsing the packet. In this case
					 *    the ethernet stack will assume any prior checksum
					 *    value was miscomputed and throw it away.
					 * 3) skb->ip_summed was set to CHECKSUM_COMPLETE, skb->csum was
					 *    set, but the result does not check out ok by the
					 *    ethernet stack.
					 *
					 * If the TEMAC hardware stripping feature is off, each
					 * packet will contain an FCS feild which will have been
					 * computed by the hardware checksum operation. This 4 byte
					 * FCS value needs to be subtracted back out of the checksum
					 * value computed by hardware as it's not included in a
					 * normal ethernet packet checksum.
					 *
					 * The minimum transfer packet size over the wire is 64
					 * bytes. If the packet is sent as exactly 64 bytes, then
					 * it probably contains some random padding bytes. It's
					 * somewhat difficult to determine the actual length of the
					 * real packet data, so we just let the stack recheck the
					 * checksum for us.
					 *
					 * After the call to eth_type_trans(), the following holds
					 * true:
					 *    skb->data points to the beginning of the ip header
					 */
					csum = BdCsumGet(BdCurPtr);
					//printk("hw csum is 0x%08x\n",csum);

#if ! XTE_AUTOSTRIPPING
					if (!lp->stripping) {
						/* take off the FCS */
						u16 *data;

						/* FCS is 4 bytes */
						skb_put(skb, -4);

						data = (u16 *) (&skb->
								data[skb->len]);

						/* subtract out the FCS from the csum value */
						csum = csum_sub(csum, *data /* & 0xffff */);
						data++;
						csum = csum_sub(csum, *data /* & 0xffff */);
					}
#endif
					skb->csum = csum;
					skb->ip_summed = CHECKSUM_COMPLETE;

					lp->rx_hw_csums++;
				}

				lp->stats.rx_packets++;
				lp->stats.rx_bytes += len;

				netif_rx(skb);	/* Send the packet upstream. */

				BdCurPtr =
					XLlDma_mBdRingNext(&lp->Dma.RxBdRing,
							   BdCurPtr);
				bd_processed--;
			} while (bd_processed > 0);

			/* give the descriptor back to the driver */
			result = XLlDma_BdRingFree(&lp->Dma.RxBdRing,
						   bd_processed_saved, BdPtr);
			if (result != XST_SUCCESS) {
				printk(KERN_ERR
				       "%s: XLlDma: BdRingFree unsuccessful (%d)\n",
				       dev->name, result);
				reset(dev, __LINE__);
				spin_unlock_irqrestore(&XTE_rx_spinlock, flags);
				return;
			}

			_xenet_DmaSetupRecvBuffers(dev);
		}
		XLlDma_mBdRingIntEnable(&lp->Dma.RxBdRing, dma_rx_int_mask);
		spin_unlock_irqrestore(&XTE_rx_spinlock, flags);
	}
}

static int descriptor_init(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	int recvsize, sendsize;
	int dftsize;
	u32 *recvpoolptr, *sendpoolptr;
	void *recvpoolphy, *sendpoolphy;
	int result;

/*
 * Buffer Descriptr
 * word	byte	description
 * 0	0h	next ptr
 * 1	4h	buffer addr
 * 2	8h	buffer len
 * 3	ch	sts/ctrl | app data (0) [tx csum enable (bit 31 LSB)]
 * 4	10h	app data (1) [tx csum begin (bits 0-15 MSB) | csum insert (bits 16-31 LSB)]
 * 5	14h	app data (2) [tx csum seed (bits 16-31 LSB)]
 * 6	18h	app data (3) [rx raw csum (bits 16-31 LSB)]
 * 7	1ch	app data (4) [rx recv length (bits 18-31 LSB)]
 */
#if 0
	int XferType = XDMAV3_DMACR_TYPE_BFBURST_MASK;
	int XferWidth = XDMAV3_DMACR_DSIZE_64_MASK;
#endif

	/* calc size of descriptor space pool; alloc from non-cached memory */
	dftsize = XLlDma_mBdRingMemCalc(ALIGNMENT_BD,
					XTE_RECV_BD_CNT + XTE_SEND_BD_CNT);
//	printk(KERN_INFO "labx_eth_llink: buffer descriptor size: %d (0x%0x)\n",
//	       dftsize, dftsize);

#if BD_IN_BRAM == 0
	/*
	 * Allow buffer descriptors to be cached.
	 * Old method w/cache on buffer descriptors disabled:
	 *     lp->desc_space = dma_alloc_coherent(NULL, dftsize,
	 *         &lp->desc_space_handle, GFP_KERNEL);
	 * (note if going back to dma_alloc_coherent() the CACHE macros in
	 * xenv_linux.h need to be disabled.
	 */

 //       printk(KERN_INFO "labx_eth_llink: Allocating DMA descriptors with kmalloc\n");
        lp->desc_space = kmalloc(dftsize, GFP_KERNEL);
	lp->desc_space_handle = (dma_addr_t) page_to_phys(virt_to_page(lp->desc_space));
#else
//        printk(KERN_INFO "labx_eth_llink: Allocating DMA descriptors in Block Ram\n");
	lp->desc_space_handle = BRAM_BASEADDR;
	lp->desc_space = ioremap(lp->desc_space_handle, dftsize);
#endif
	if (lp->desc_space == 0) {
		return -1;
	}

	lp->desc_space_size = dftsize;

//	printk(KERN_INFO
//	       "labx_eth_llink: (buffer_descriptor_init) phy: 0x%x, virt: 0x%x, size: 0x%x\n",
//	       lp->desc_space_handle, (unsigned int) lp->desc_space,
//	       lp->desc_space_size);

	/* calc size of send and recv descriptor space */
	recvsize = XLlDma_mBdRingMemCalc(ALIGNMENT_BD, XTE_RECV_BD_CNT);
	sendsize = XLlDma_mBdRingMemCalc(ALIGNMENT_BD, XTE_SEND_BD_CNT);

	recvpoolptr = lp->desc_space;
	sendpoolptr = (void *) ((u32) lp->desc_space + recvsize);

	recvpoolphy = (void *) lp->desc_space_handle;
	sendpoolphy = (void *) ((u32) lp->desc_space_handle + recvsize);

	result = XLlDma_BdRingCreate(&lp->Dma.RxBdRing, (u32) recvpoolphy,
				     (u32) recvpoolptr, ALIGNMENT_BD,
				     XTE_RECV_BD_CNT);
	if (result != XST_SUCCESS) {
		printk(KERN_ERR "labx_eth_llink: DMA Ring Create (RECV). Error: %d\n", result);
		return -EIO;
	}

	result = XLlDma_BdRingCreate(&lp->Dma.TxBdRing, (u32) sendpoolphy,
				     (u32) sendpoolptr, ALIGNMENT_BD,
				     XTE_SEND_BD_CNT);
	if (result != XST_SUCCESS) {
		printk(KERN_ERR "labx_eth_llink: DMA Ring Create (SEND). Error: %d\n", result);
		return -EIO;
	}

	_xenet_DmaSetupRecvBuffers(dev);
	return 0;
}

static void free_descriptor_skb(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	XLlDma_Bd *BdPtr;
	struct sk_buff *skb;
	dma_addr_t skb_dma_addr;
	u32 len, i;

	/* Unmap and free skb's allocated and mapped in descriptor_init() */

	/* Get the virtual address of the 1st BD in the DMA RX BD ring */
	BdPtr = (XLlDma_Bd *) lp->Dma.RxBdRing.FirstBdAddr;

	for (i = 0; i < XTE_RECV_BD_CNT; i++) {
		skb = (struct sk_buff *) XLlDma_mBdGetId(BdPtr);
		if (skb) {
			skb_dma_addr = (dma_addr_t) XLlDma_mBdGetBufAddr(BdPtr);
			dma_unmap_single(NULL, (void *)skb_dma_addr, lp->max_frame_size,
					 DMA_FROM_DEVICE);
			dev_kfree_skb(skb);
		}
		/* find the next BD in the DMA RX BD ring */
		BdPtr = XLlDma_mBdRingNext(&lp->Dma.RxBdRing, BdPtr);
	}

	/* Unmap and free TX skb's that have not had a chance to be freed
	 * in DmaSendHandlerBH(). This could happen when TX Threshold is larger
	 * than 1 and TX waitbound is 0
	 */

	/* Get the virtual address of the 1st BD in the DMA TX BD ring */
	BdPtr = (XLlDma_Bd *) lp->Dma.TxBdRing.FirstBdAddr;

	for (i = 0; i < XTE_SEND_BD_CNT; i++) {
		skb = (struct sk_buff *) XLlDma_mBdGetId(BdPtr);
		if (skb) {
			skb_dma_addr = (dma_addr_t) XLlDma_mBdGetBufAddr(BdPtr);
			len = XLlDma_mBdGetLength(BdPtr);
			dma_unmap_single(NULL, (void *)skb_dma_addr, len,
					 DMA_TO_DEVICE);
			dev_kfree_skb(skb);
		}
		/* find the next BD in the DMA TX BD ring */
		BdPtr = XLlDma_mBdRingNext(&lp->Dma.TxBdRing, BdPtr);
	}

#if BD_IN_BRAM == 0
	dma_free_coherent(NULL,
			  lp->desc_space_size,
			  lp->desc_space, lp->desc_space_handle);
#else
	iounmap(lp->desc_space);
#endif
}



static void disp_bd_ring(XLlDma_BdRing *bd_ring)
{
	int num_bds = bd_ring->AllCnt;
	u32 *cur_bd_ptr = (u32 *) bd_ring->FirstBdAddr;
	int idx;

	printk("ChanBase: %p\n", (void *) bd_ring->ChanBase);
	printk("FirstBdPhysAddr: %p\n", (void *) bd_ring->FirstBdPhysAddr);
	printk("FirstBdAddr: %p\n", (void *) bd_ring->FirstBdAddr);
	printk("LastBdAddr: %p\n", (void *) bd_ring->LastBdAddr);
	printk("Length: %d (0x%0x)\n", bd_ring->Length, bd_ring->Length);
	printk("RunState: %d (0x%0x)\n", bd_ring->RunState, bd_ring->RunState);
	printk("Separation: %d (0x%0x)\n", bd_ring->Separation,
	       bd_ring->Separation);
	printk("BD Count: %d\n", bd_ring->AllCnt);

	printk("\n");

	printk("FreeHead: %p\n", (void *) bd_ring->FreeHead);
	printk("PreHead: %p\n", (void *) bd_ring->PreHead);
	printk("HwHead: %p\n", (void *) bd_ring->HwHead);
	printk("HwTail: %p\n", (void *) bd_ring->HwTail);
	printk("PostHead: %p\n", (void *) bd_ring->PostHead);
	printk("BdaRestart: %p\n", (void *) bd_ring->BdaRestart);

	printk("Ring Contents:\n");
/*
 * Buffer Descriptr
 * word	byte	description
 * 0	0h	next ptr
 * 1	4h	buffer addr
 * 2	8h	buffer len
 * 3	ch	sts/ctrl | app data (0) [tx csum enable (bit 31 LSB)]
 * 4	10h	app data (1) [tx csum begin (bits 0-15 MSB) | csum insert (bits 16-31 LSB)]
 * 5	14h	app data (2) [tx csum seed (bits 16-31 LSB)]
 * 6	18h	app data (3) [rx raw csum (bits 16-31 LSB)]
 * 7	1ch	app data (4) [rx recv length (bits 18-31 LSB)]
 * 8	20h	sw app data (0) [id]
 */
	printk("Idx   NextBD BuffAddr   Length  CTL/CSE CSUM B/I CSUMSeed Raw CSUM  RecvLen       ID\n");
	printk("---------------------------------------------------------------------------\n");

	for (idx = 0; idx < num_bds; idx++) {
		printk("%3d %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
		       idx,
		       cur_bd_ptr[XLLDMA_BD_NDESC_OFFSET / sizeof(*cur_bd_ptr)],
		       cur_bd_ptr[XLLDMA_BD_BUFA_OFFSET / sizeof(*cur_bd_ptr)],
		       cur_bd_ptr[XLLDMA_BD_BUFL_OFFSET / sizeof(*cur_bd_ptr)],
		       cur_bd_ptr[XLLDMA_BD_STSCTRL_USR0_OFFSET /
				  sizeof(*cur_bd_ptr)],
		       cur_bd_ptr[XLLDMA_BD_USR1_OFFSET / sizeof(*cur_bd_ptr)],
		       cur_bd_ptr[XLLDMA_BD_USR2_OFFSET / sizeof(*cur_bd_ptr)],
		       cur_bd_ptr[XLLDMA_BD_USR3_OFFSET / sizeof(*cur_bd_ptr)],
		       cur_bd_ptr[XLLDMA_BD_USR4_OFFSET / sizeof(*cur_bd_ptr)],
		       cur_bd_ptr[XLLDMA_BD_ID_OFFSET / sizeof(*cur_bd_ptr)]);

		cur_bd_ptr += bd_ring->Separation / sizeof(int);
	}
	printk("------------------------------------------------------------------------------\n");
}




static int xenet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct net_local *lp = netdev_priv(dev);

	/* gmii_ioctl_data has 4 u16 fields: phy_id, reg_num, val_in & val_out */
	struct mii_ioctl_data *data = (struct mii_ioctl_data *) &rq->ifr_data;
	struct {
		__u16 threshold;
		__u32 direction;
	} thr_arg;
	struct {
		__u16 waitbound;
		__u32 direction;
	} wbnd_arg;

	int ret;
	u32 threshold, timer;
	XLlDma_BdRing *RingPtr;
	u32 *dma_int_mask_ptr;

	switch (cmd) {
  	
	case SIOCGMIIPHY:	/* Get address of GMII PHY in use. */
	case SIOCGMIIREG:	/* Read GMII PHY register. */
	case SIOCSMIIREG:	/* Write GMII PHY register. */
		if (NULL == lp->phy_dev) {
			return -ENODEV;
		}

		return phy_mii_ioctl(lp->phy_dev, data, cmd);

	case SIOCDEVPRIVATE + 3:	/* set THRESHOLD */
		if (XLlTemac_IsFifo(&lp->Emac))
			return -EFAULT;

		if (copy_from_user(&thr_arg, rq->ifr_data, sizeof(thr_arg)))
			return -EFAULT;

		if (thr_arg.direction == XTE_SEND) {
			RingPtr = &lp->Dma.TxBdRing;
		} else {
			RingPtr = &lp->Dma.RxBdRing;
		}
		XLlDma_BdRingGetCoalesce(RingPtr, &threshold, &timer);
		if (thr_arg.direction == XTE_SEND) {
			RingPtr = &lp->Dma.TxBdRing;
		} else {
			RingPtr = &lp->Dma.RxBdRing;
		}
		if ((ret = XLlDma_BdRingSetCoalesce(RingPtr, thr_arg.threshold,
						    timer)) != XST_SUCCESS) {
			return -EIO;
		}
		return 0;

	case SIOCDEVPRIVATE + 4:	/* set WAITBOUND */
		if (!(XLlTemac_IsDma(&lp->Emac)))
			return -EFAULT;

		if (copy_from_user(&wbnd_arg, rq->ifr_data, sizeof(wbnd_arg)))
			return -EFAULT;

		if (wbnd_arg.direction == XTE_SEND) {
			RingPtr = &lp->Dma.TxBdRing;
		} else {
			RingPtr = &lp->Dma.RxBdRing;
		}
		XLlDma_BdRingGetCoalesce(RingPtr, &threshold, &timer);
		if (wbnd_arg.direction == XTE_SEND) {
			RingPtr = &lp->Dma.TxBdRing;
			dma_int_mask_ptr = &dma_tx_int_mask;
		} else {
			RingPtr = &lp->Dma.RxBdRing;
			dma_int_mask_ptr = &dma_rx_int_mask;
		}
		if (wbnd_arg.waitbound == 0) {
			wbnd_arg.waitbound = 1;
			*dma_int_mask_ptr = XLLDMA_CR_IRQ_ALL_EN_MASK & ~XLLDMA_CR_IRQ_DELAY_EN_MASK;
		}
		if ((ret = XLlDma_BdRingSetCoalesce(RingPtr, threshold,
					wbnd_arg.waitbound)) != XST_SUCCESS) {
			return -EIO;
		}
		XLlDma_mBdRingIntEnable(RingPtr, *dma_int_mask_ptr);

		return 0;

	case SIOCDEVPRIVATE + 5:	/* get THRESHOLD */
		if (!(XLlTemac_IsDma(&lp->Emac)))
			return -EFAULT;

		if (copy_from_user(&thr_arg, rq->ifr_data, sizeof(thr_arg)))
			return -EFAULT;

		if (thr_arg.direction == XTE_SEND) {
			RingPtr = &lp->Dma.TxBdRing;
		} else {
			RingPtr = &lp->Dma.RxBdRing;
		}
		XLlDma_BdRingGetCoalesce(RingPtr,
				(u32 *) &(thr_arg.threshold), &timer);
		if (copy_to_user(rq->ifr_data, &thr_arg, sizeof(thr_arg))) {
			return -EFAULT;
		}
		return 0;

	case SIOCDEVPRIVATE + 6:	/* get WAITBOUND */
		if (!(XLlTemac_IsDma(&lp->Emac)))
			return -EFAULT;

		if (copy_from_user(&wbnd_arg, rq->ifr_data, sizeof(wbnd_arg))) {
			return -EFAULT;
		}
		if (thr_arg.direction == XTE_SEND) {
			RingPtr = &lp->Dma.TxBdRing;
		} else {
			RingPtr = &lp->Dma.RxBdRing;
		}
		XLlDma_BdRingGetCoalesce(RingPtr, &threshold,
					 (u32 *) &(wbnd_arg.waitbound));
		if (copy_to_user(rq->ifr_data, &wbnd_arg, sizeof(wbnd_arg))) {
			return -EFAULT;
		}
		return 0;

	default:
		return -EOPNOTSUPP;
	}
}


/******************************************************************************
 *
 * NEW FUNCTIONS FROM LINUX 2.6
 *
 ******************************************************************************/

static void xtenet_remove_ndev(struct net_device *ndev)
{
	if (ndev) {
		struct net_local *lp = netdev_priv(ndev);

		if (XLlTemac_IsDma(&lp->Emac) && (lp->desc_space))
			free_descriptor_skb(ndev);

		iounmap((void *) (lp->Emac.Config.BaseAddress));
		free_netdev(ndev);
	}
}

static int xtenet_remove(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);

	unregister_netdev(ndev);
	xtenet_remove_ndev(ndev);

	return 0;		/* success */
}

static void labx_eth_ll_mac_adjust_link(struct net_device *dev)
{
	struct net_local *lp = netdev_priv(dev);
	if (lp->phy_dev->link != PHY_DOWN)
	{
		if (lp->cur_speed != lp->phy_dev->speed)
		{
			labx_XLlTemac_SetOperatingSpeed(&lp->Emac, lp->phy_dev->speed);
			lp->cur_speed = lp->phy_dev->speed;
			printk("%s: Link up, %d Mb/s\n", lp->ndev->name, lp->cur_speed);
		}
	}
	else
	{
		if (lp->cur_speed != 0)
		{
			lp->cur_speed = 0;
			printk("%s: Link down\n", lp->ndev->name);
		}
	}
}

/* MDIO interrupt service routine */
static irqreturn_t mdio_interrupt(int irq, void *dev_id)
{
  struct net_local *lp = (struct net_local *) dev_id;
  XLlTemac *InstancePtr = (XLlTemac *) &lp->Emac;
  u32 maskedFlags;

  /* Read the interrupt flags and immediately clear them */
  maskedFlags = XLlTemac_ReadReg(InstancePtr->Config.BaseAddress, INT_FLAGS_REG);
  maskedFlags &= XLlTemac_ReadReg(InstancePtr->Config.BaseAddress, INT_MASK_REG);
  XLlTemac_WriteReg(InstancePtr->Config.BaseAddress, INT_FLAGS_REG, maskedFlags);

  /* Service the MDIO interrupt */
  if((maskedFlags & MDIO_IRQ_MASK) != 0) {
    /* Indicate the MDIO is ready again */
    InstancePtr->MdioState = MDIO_STATE_READY;
    wake_up_interruptible(&InstancePtr->PhyWait);
  }

  return(IRQ_HANDLED);
}

/** Shared device initialization code */
static int xtenet_setup(
		struct device *dev,
		struct resource *r_mem,
		struct resource *r_irq,
		struct labx_ll_eth_platform_data *pdata) {
	int xs;
	u32 virt_baddr;		/* virtual base address of TEMAC */
	int i;

	XLlTemac_Config Temac_Config;

	struct net_device *ndev = NULL;
	struct net_local *lp = NULL;

	int rc = 0;

	/* Create an ethernet device instance */
	ndev = alloc_etherdev(sizeof(struct net_local));
	if (!ndev) {
		dev_err(dev, "labx_eth_llink: Could not allocate net device.\n");
		rc = -ENOMEM;
		goto error;
	}
	SET_NETDEV_DEV(ndev, dev);
	dev_set_drvdata(dev, ndev);

	/* Allocate the dev name early so we can use it in our messages */
	if (strchr(ndev->name, '%')) {
		rc = dev_alloc_name(ndev, ndev->name);
		if (rc < 0) goto error;
 	}

	/* Initialize the private data used by XEmac_LookupConfig().
	 * The private data are zeroed out by alloc_etherdev() already.
	 */
	lp = netdev_priv(ndev);
	lp->ndev = ndev;
	lp->dma_irq_r = pdata->ll_dev_dma_rx_irq;
	lp->dma_irq_s = pdata->ll_dev_dma_tx_irq;
	lp->fifo_irq = pdata->ll_dev_fifo_irq;
	strncpy(lp->phy_name,pdata->phy_name,64);

	/* Setup the Config structure for the XLlTemac_CfgInitialize() call. */
	Temac_Config.BaseAddress = r_mem->start;
#if 0
	Config.RxPktFifoDepth = pdata->rx_pkt_fifo_depth;
	Config.TxPktFifoDepth = pdata->tx_pkt_fifo_depth;
	Config.MacFifoDepth = pdata->mac_fifo_depth;
	Config.IpIfDmaConfig = pdata->dma_mode;
#endif
	Temac_Config.TxCsum = pdata->tx_csum;
	Temac_Config.RxCsum = pdata->rx_csum;
	Temac_Config.LLDevType = pdata->ll_dev_type;
	Temac_Config.LLDevBaseAddress = pdata->ll_dev_baseaddress;
	Temac_Config.PhyType = pdata->phy_type;
	Temac_Config.PhyAddr = pdata->phy_addr;
  Temac_Config.MacWidth = pdata->mac_width;
	/* Get the virtual base address for the device */
	virt_baddr = (u32) ioremap(r_mem->start, r_mem->end - r_mem->start + 1);
	if (0 == virt_baddr) {
		dev_err(dev, "%s: Could not allocate iomem.\n", ndev->name);
		rc = -EIO;
		goto error;
	}

	if (labx_XLlTemac_CfgInitialize(&lp->Emac, &Temac_Config, virt_baddr) !=
	    XST_SUCCESS) {
		dev_err(dev, "%s: Could not initialize device.\n", ndev->name);

		rc = -ENODEV;
		goto error;
	}
	/* Set the MAC address */
	for (i = 0; i < 6; i++) {
		if (macaddr[i] != 0)
			break;
	}
	if (i < 6) { /* a mac address was given */
		for (i = 0; i < 6; i++)
			ndev->dev_addr[i] = macaddr[i];
		macaddr[5]++;
	}
	else {
		/* Use the platform assigned MAC if none specified */
		memcpy(ndev->dev_addr, pdata->mac_addr, 6);
	}

	if (_XLlTemac_SetMacAddress(&lp->Emac, ndev->dev_addr) != XST_SUCCESS) {
		/* should not fail right after an initialize */
		dev_err(dev, "%s: could not set MAC address.\n", ndev->name);
		rc = -EIO;
		goto error;
	}

	printk("%s: MAC address is now %02X:%02X:%02X:%02X:%02X:%02X\n",
           ndev->name,
           pdata->mac_addr[0], pdata->mac_addr[1],
           pdata->mac_addr[2], pdata->mac_addr[3],
           pdata->mac_addr[4], pdata->mac_addr[5]);

	lp->max_frame_size = XTE_MAX_JUMBO_FRAME_SIZE;
	if (ndev->mtu > XTE_JUMBO_MTU)
		ndev->mtu = XTE_JUMBO_MTU;


	if (XLlTemac_IsDma(&lp->Emac)) {
		int result;

		printk("%s: labx_eth_llink: using DMA mode.\n", ndev->name);

#ifndef XDCRIO_H
		virt_baddr = (u32) ioremap(pdata->ll_dev_baseaddress, 4096);
		if (0 == virt_baddr) {
			dev_err(dev,
			       "%s: Could not allocate iomem for local link connected device.\n", ndev->name);
			rc = -EIO;
			goto error;
		}
#endif
//		printk("%s: Dma base address: phy: 0x%x, virt: 0x%x\n", ndev->name, pdata->ll_dev_baseaddress, virt_baddr);
		XLlDma_Initialize(&lp->Dma, virt_baddr);

		ndev->hard_start_xmit = xenet_DmaSend;

		result = descriptor_init(ndev);
		if (result) {
			rc = -EIO;
			goto error;
		}

		/* set the packet threshold and wait bound for both TX/RX directions */
		if (DFT_TX_WAITBOUND == 0) {
			dma_tx_int_mask = XLLDMA_CR_IRQ_ALL_EN_MASK & ~XLLDMA_CR_IRQ_DELAY_EN_MASK;
			xs = XLlDma_BdRingSetCoalesce(&lp->Dma.TxBdRing, DFT_TX_THRESHOLD, 1);
		} else {
			xs = XLlDma_BdRingSetCoalesce(&lp->Dma.TxBdRing, DFT_TX_THRESHOLD, DFT_TX_WAITBOUND);
		}
		if (xs != XST_SUCCESS) {
			dev_err(dev,
			       "%s: could not set SEND pkt threshold/waitbound, ERROR %d",
			       ndev->name, xs);
		}
		XLlDma_mBdRingIntEnable(&lp->Dma.TxBdRing, dma_tx_int_mask);

		if (DFT_RX_WAITBOUND == 0) {
			dma_rx_int_mask = XLLDMA_CR_IRQ_ALL_EN_MASK & ~XLLDMA_CR_IRQ_DELAY_EN_MASK;
			xs = XLlDma_BdRingSetCoalesce(&lp->Dma.RxBdRing, DFT_RX_THRESHOLD, 1);
		} else {
			xs = XLlDma_BdRingSetCoalesce(&lp->Dma.RxBdRing, DFT_RX_THRESHOLD, DFT_RX_WAITBOUND);
		}
		if (xs != XST_SUCCESS) {
			dev_err(dev,
			       "%s: Could not set RECV pkt threshold/waitbound ERROR %d",
			       ndev->name, xs);
		}
		XLlDma_mBdRingIntEnable(&lp->Dma.RxBdRing, dma_rx_int_mask);
	}
	else {
		dev_err(dev,
		       "%s: using FIFO direct interrupt driven mode.\n", ndev->name);

		virt_baddr = (u32) ioremap(pdata->ll_dev_baseaddress, 4096);
		if (0 == virt_baddr) {
			dev_err(dev,
			       "%s: Could not allocate iomem for local link connected device.\n", ndev->name);
			rc = -EIO;
			goto error;
		}
		printk("%s: Fifo base address: 0x%0x\n", ndev->name, virt_baddr);
		XLlFifo_Initialize(&lp->Fifo, virt_baddr);

		ndev->hard_start_xmit = xenet_FifoSend;
	}

	/* initialize the netdev structure */
	ndev->open = xenet_open;
	ndev->stop = xenet_close;
	ndev->change_mtu = xenet_change_mtu;
	ndev->get_stats = xenet_get_stats;
	ndev->set_multicast_list = xenet_set_multicast_list;
	ndev->set_mac_address = xenet_set_mac_address;
	ndev->flags &= ~IFF_MULTICAST;

	if (XLlTemac_IsDma(&lp->Emac)) {
		ndev->features = NETIF_F_SG | NETIF_F_FRAGLIST;

		if (XLlTemac_IsTxCsum(&lp->Emac) == TRUE) {
			/*
			 * This hardware only supports proper checksum calculations
			 * on TCP/UDP packets.
			 */
			ndev->features |= NETIF_F_IP_CSUM;
		}
		if (XLlTemac_IsRxCsum(&lp->Emac) == TRUE) {
			lp->local_features |= LOCAL_FEATURE_RX_CSUM;
		}
	}

	ndev->do_ioctl = xenet_ioctl;
	ndev->tx_timeout = xenet_tx_timeout;
	ndev->watchdog_timeo = TX_TIMEOUT;
  ndev->ethtool_ops = &labx_ethtool_ops;
  
	/* init the stats */
	lp->max_frags_in_a_packet = 0;
	lp->tx_hw_csums = 0;
	lp->rx_hw_csums = 0;

#if ! XTE_AUTOSTRIPPING
	lp->stripping =
		(labx_XLlTemac_GetOptions(&(lp->Emac)) & XTE_FCS_STRIP_OPTION) != 0;
#endif

	rc = register_netdev(ndev);
	if (rc) {
		dev_err(dev,
		       "%s: Cannot register net device, aborting.\n",
		       ndev->name);
		goto error;	/* rc is already set here... */
	}

	/* Setup MDIO IRQ handling */
	if (NULL != r_irq) {
		ndev->irq = r_irq->start;
		snprintf(lp->mdio_irq_name, IRQ_NAME_SZ, "%s MDIO", ndev->name);
		lp->mdio_irq_name[IRQ_NAME_SZ - 1] = '\0';
		rc = request_irq(ndev->irq, &mdio_interrupt, IRQF_DISABLED, lp->mdio_irq_name, lp);
		if(rc) {
			printk(KERN_ERR "%s: Could not allocate Lab X Ethernet MDIO interrupt (%d).\n",
				ndev->name, ndev->irq);
			goto error;
		}
	} else {
		ndev->irq = NO_IRQ;
	}
	init_waitqueue_head(&lp->Emac.PhyWait);
	lp->Emac.MdioState = MDIO_STATE_READY;
	XLlTemac_WriteReg(Temac_Config.BaseAddress, INT_MASK_REG, NO_IRQS);
	XLlTemac_WriteReg(Temac_Config.BaseAddress, INT_FLAGS_REG, (PHY_IRQ_MASK | MDIO_IRQ_MASK));

    /* TODO - Add a configuration option; for now this presumes an active-
     *        low interrupt from the PHY (which is most common)
     */
    XLlTemac_WriteReg(Temac_Config.BaseAddress, INT_MASK_REG, (PHY_IRQ_LOW | MDIO_IRQ_MASK));

	lp->gmii_addr = lp->Emac.Config.PhyAddr;

	printk("%s: Lab X Tri-mode MAC at 0x%08X, IRQ %d using PHY at MDIO 0x%02X\n",
           ndev->name,
           r_mem->start,
           ndev->irq,
           lp->gmii_addr);

	if (pdata->phy_mask != 0) {
		return labx_eth_ll_mdio_bus_init(dev, pdata, &lp->Emac);
	} else {
		return 0;
	}

error:
	if (ndev) {
		xtenet_remove_ndev(ndev);
	}
	return rc;
}

static int xtenet_probe_thread(void *data)
{
	struct device *dev = (struct device *)data;

	struct resource *r_irq     = NULL; /* Interrupt resources */
	struct resource *r_mem     = NULL; /* IO mem resources */
	struct labx_ll_eth_platform_data *pdata;
	struct platform_device *pdev = to_platform_device(dev);

	/* param check */
	if (!pdev) {
		dev_err(dev, "labx_eth_llink: Internal error. Probe called with NULL param.\n");
		return -ENODEV;
	}

	pdata = (struct labx_ll_eth_platform_data *) pdev->dev.platform_data;
	if (!pdata) {
		dev_err(dev, "labx_eth_llink: Couldn't find platform data.\n");

		return -ENODEV;
	}

	/* Get iospace and an irq for the device */
	r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(!r_mem) {
		dev_err(dev, "labx_eth_llink: IO resource MEM not found.\n");
		return -ENODEV;
	}
	r_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	return(xtenet_setup(dev, r_mem, r_irq, pdata));
}


static int xtenet_probe(struct device *dev)
{
	//kthread_run(xtenet_probe_thread, dev, "xtenet_probe_thread");
	//return 0;
	return xtenet_probe_thread(dev);
}

static struct device_driver xtenet_driver = {
	.name = DRIVER_NAME,
	.bus = &platform_bus_type,

	.probe = xtenet_probe,
	.remove = xtenet_remove
};

#ifdef CONFIG_OF
static u32 get_u32(struct of_device *ofdev, const char *s) {
	u32 *p = (u32 *)of_get_property(ofdev->node, s, NULL);
	if(p) {
		return *p;
	} else {
		dev_warn(&ofdev->dev, "Parameter %s not found, defaulting to false.\n", s);
		return FALSE;
	}
}

static struct of_device_id xtenet_fifo_of_match[] = {
	{ .compatible = "xlnx,xps-ll-fifo-1.00.a", },
	{ .compatible = "xlnx,xps-ll-fifo-1.00.b", },
	{ .compatible = "xlnx,xps-ll-fifo-1.01.a", },
	{ .compatible = "xlnx,xps-ll-fifo-1.02.a", },
	{ /* end of list */ },
};

static struct of_device_id xtenet_sdma_of_match[] = {
	{ .compatible = "xlnx,ll-dma-1.00.a", },
	{ /* end of list */ },
};

/* Note: This must be <= MII_BUS_ID_SIZE which is currently 17 (including trailing '\0') */
#define MDIO_OF_BUSNAME_FMT "labxeth%08x"

static int __devinit xtenet_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
	struct resource r_irq_struct;
	struct resource r_irq_phy_struct;
	struct resource r_mem_struct;
	struct resource r_connected_mdio_mem_struct;
	struct resource r_connected_mem_struct;
	struct resource r_connected_irq_struct;
	struct resource *r_irq     = &r_irq_struct;     /* Interrupt resources */
	struct resource *r_irq_phy = &r_irq_phy_struct; /* Interrupt resources */
	struct resource *r_mem     = &r_mem_struct;     /* IO mem resources */

	struct labx_ll_eth_platform_data pdata_struct = {};

	struct labx_ll_eth_platform_data *pdata = &pdata_struct;
	const void *mac_address;
	int rc = 0;
	const phandle *llink_connected_handle;
	struct device_node *llink_connected_node;
	const phandle *mdio_controller_handle;
	struct device_node *mdio_controller_node;
	struct device_node *mdio_controller_node_eth;
	u32 *dcrreg_property;
	u32 phy_addr;
	int i;

	printk(KERN_INFO "Device Tree Probing \'%s\'\n",ofdev->node->name);

	/* Get iospace for the device */
	rc = of_address_to_resource(ofdev->node, 0, r_mem);
	if(rc) {
		dev_warn(&ofdev->dev, "invalid address\n");
		return rc;
	}

	/* Get IRQ for the device */
	rc = of_irq_to_resource(ofdev->node, 0, r_irq);
	if(rc == NO_IRQ) {
		r_irq = NULL;
	}

	/* Get IRQ of the attached phy (if any) */
	rc = of_irq_to_resource(ofdev->node, 1, r_irq_phy);
	if(rc == NO_IRQ) {
		r_irq_phy = NULL;
	}

	pdata_struct.tx_csum  = get_u32(ofdev, "xlnx,txcsum");
	pdata_struct.rx_csum  = get_u32(ofdev, "xlnx,rxcsum");
  pdata_struct.mac_width = get_u32(ofdev, "xlnx,mac-port-width");
	/* Connected PHY information */
	pdata_struct.phy_type = get_u32(ofdev, "xlnx,phy-type");
	pdata_struct.phy_addr = get_u32(ofdev, "xlnx,phy-addr"); //Yi: why don't we have this before?
	phy_addr              = get_u32(ofdev, "xlnx,phy-addr");

	pdata->phy_name[0] = '\0';
	mdio_controller_handle = of_get_property(ofdev->node, "phy-mdio-controller", NULL);
	if(!mdio_controller_handle) {
		dev_warn(&ofdev->dev, "no MDIO connection specified.\n");
	} else {
		mdio_controller_node = of_find_node_by_phandle(*mdio_controller_handle);
		if (!mdio_controller_node) {
			dev_warn(&ofdev->dev, "no MDIO connection found.\n");
		} else {
			/* The ethernet@xxxxxxxx should be the first child... */
			mdio_controller_node_eth = of_get_next_child(mdio_controller_node, NULL);

			if (!mdio_controller_node_eth) {
				dev_warn(&ofdev->dev, "MDIO connection node has no ethernet child.\n");
			} else {
				rc = of_address_to_resource(mdio_controller_node_eth, 0, &r_connected_mdio_mem_struct);
				snprintf(pdata->phy_name, BUS_ID_SIZE, MDIO_OF_BUSNAME_FMT ":%02x", (u32)r_connected_mdio_mem_struct.start, phy_addr);
				printk("%s:phy_name: %s\n",__func__, pdata->phy_name);
			}
		}
	}

	/* Connected MDIO bus information */
	if (0 == get_u32(ofdev, "xlnx,has-mdio")) {
		pdata_struct.phy_mask = 0;
		pdata_struct.mdio_bus_name[0] = '\0';
	} else {
		pdata_struct.phy_mask = get_u32(ofdev, "xlnx,phy-mask");
		snprintf(pdata_struct.mdio_bus_name, MII_BUS_ID_SIZE, MDIO_OF_BUSNAME_FMT, (u32)r_mem->start);
	}

	if (NULL != r_irq_phy) {
		for (i=0; i<PHY_MAX_ADDR; i++) {
          pdata_struct.mdio_phy_irqs[i] = r_irq_phy->start;
		}
	}

	llink_connected_handle = of_get_property(ofdev->node, "llink-connected", NULL);
	if(!llink_connected_handle) {
		dev_warn(&ofdev->dev, "no Locallink connection found.\n");
		return rc;
	}

	llink_connected_node = of_find_node_by_phandle(*llink_connected_handle);
	if (!llink_connected_node) {
		dev_warn(&ofdev->dev, "no Locallink connection found.\n");
		return rc;
	}

	rc = of_address_to_resource( llink_connected_node, 0, &r_connected_mem_struct);

	/** Get the right information from whatever the locallink is connected to. */
	if(of_match_node(xtenet_fifo_of_match, llink_connected_node)) {
		/** Connected to a fifo. */
		if(rc) {
			dev_warn(&ofdev->dev, "invalid address\n");
			return rc;
		}

		pdata_struct.ll_dev_baseaddress	= r_connected_mem_struct.start;
		pdata_struct.ll_dev_type = XPAR_LL_FIFO;
		pdata_struct.ll_dev_dma_rx_irq	= NO_IRQ;
		pdata_struct.ll_dev_dma_tx_irq	= NO_IRQ;

		rc = of_irq_to_resource( llink_connected_node, 0, &r_connected_irq_struct);
		if(rc == NO_IRQ) {
			dev_warn(&ofdev->dev, "no IRQ found.\n");
			return rc;
		}
		pdata_struct.ll_dev_fifo_irq	= r_connected_irq_struct.start;
		pdata_struct.dcr_host = 0x0;
        } 
	else if(of_match_node(xtenet_sdma_of_match, llink_connected_node)) {
		/** Connected to a dma port, default to 405 type dma */
		pdata->dcr_host = 0;
		if(rc) {
			printk("No Address was found, might be 440, check for dcr reg\n");
			/* no address was found, might be 440, check for dcr reg */

			dcrreg_property = (u32 *)of_get_property(llink_connected_node, "dcr-reg",NULL);
			if(dcrreg_property) {
			        r_connected_mem_struct.start = *dcrreg_property;
				pdata->dcr_host = 0xFF;
			} else {
				dev_warn(&ofdev->dev, "invalid address\n");
				return rc;
			}			
		}

		pdata_struct.ll_dev_baseaddress	= r_connected_mem_struct.start;
		pdata_struct.ll_dev_type = XPAR_LL_DMA;

		rc = of_irq_to_resource(llink_connected_node, 0, &r_connected_irq_struct);
		if(rc == NO_IRQ) {
			dev_warn(&ofdev->dev, "First IRQ not found.\n");
			return rc;
		}
		pdata_struct.ll_dev_dma_rx_irq	= r_connected_irq_struct.start;

		rc = of_irq_to_resource(llink_connected_node, 1, &r_connected_irq_struct);
		if(rc == NO_IRQ) {
			dev_warn(&ofdev->dev, "Second IRQ not found.\n");
			return rc;
		}
		pdata_struct.ll_dev_dma_tx_irq	= r_connected_irq_struct.start;

		pdata_struct.ll_dev_fifo_irq	= NO_IRQ;
        } else {
		dev_warn(&ofdev->dev, "Locallink connection not matched.\n");
		printk("Locallink connection no matched\n");
		return rc;
        }

	of_node_put(llink_connected_node);
        mac_address = of_get_mac_address(ofdev->node);
        if(mac_address) {
            memcpy(pdata_struct.mac_addr, mac_address, 6);
        } else {
            dev_warn(&ofdev->dev, "No MAC address found.\n");
        }
		
        rc = xtenet_setup(&ofdev->dev, r_mem, r_irq, pdata);
	if (rc) {
		printk("Error calling xtenet_setup, code: %d\n", rc);
	}

	return rc;
}

static int __devexit xtenet_of_remove(struct of_device *dev)
{
	return xtenet_remove(&dev->dev);
}

static struct of_device_id xtenet_of_match[] = {
	{ .compatible = "xlnx,labx-eth-locallink-1.00.a", },
	{ .compatible = "xlnx,labx-eth-locallink-1.02.a", },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, xtenet_of_match);

static struct of_platform_driver xtenet_of_driver = {
	.name		= DRIVER_NAME,
	.match_table	= xtenet_of_match,
	.probe		= xtenet_of_probe,
	.remove		= __devexit_p(xtenet_of_remove),
};
#endif

static int __init xtenet_init(void)
{
	int status;

	/*
	 * Make sure the locks are initialized
	 */
	spin_lock_init(&XTE_spinlock);
	spin_lock_init(&XTE_tx_spinlock);
	spin_lock_init(&XTE_tx_spinlock);

	INIT_LIST_HEAD(&sentQueue);
	INIT_LIST_HEAD(&receivedQueue);

	spin_lock_init(&sentQueueSpin);
	spin_lock_init(&receivedQueueSpin);

	/*
	 * No kernel boot options used,
	 * so we just need to register the driver
	 */
	status = driver_register(&xtenet_driver);
#ifdef CONFIG_OF
	status |= of_register_platform_driver(&xtenet_of_driver);
#endif
        return status;

}

static void __exit xtenet_cleanup(void)
{
	driver_unregister(&xtenet_driver);
#ifdef CONFIG_OF
	of_unregister_platform_driver(&xtenet_of_driver);
#endif
}

module_init(xtenet_init);
module_exit(xtenet_cleanup);

MODULE_AUTHOR("Xilinx, Inc.");
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_LICENSE("GPL");
