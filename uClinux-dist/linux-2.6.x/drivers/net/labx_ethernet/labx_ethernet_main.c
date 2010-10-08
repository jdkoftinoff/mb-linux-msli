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
 * With the way the hardened Temac works, the driver needs to communicate
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
#include "labx_ethernet.h"

#include "net/labx_ethernet/labx_ethernet_defs.h"

#define LOCAL_FEATURE_RX_CSUM   0x01

/*
 * Default SEND and RECV buffer descriptors (BD) numbers.
 * BD Space needed is (XTE_SEND_BD_CNT+XTE_RECV_BD_CNT)*Sizeof(XLlDma_Bd).
 * Each XLlDma_Bd instance currently takes 40 bytes.
 */
#define XTE_SEND_BD_CNT 256
#define XTE_RECV_BD_CNT 256

/* Must be shorter than length of ethtool_drvinfo.driver field to fit */
#define DRIVER_NAME         "labx_ethernet"
#define DRIVER_DESCRIPTION  "Lab X Direct Ethernet MAC driver"
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

/* Auto-strip frame check sequences */
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
#define BdCsumEnable(BdPtr)						\
  XLlDma_mBdWrite((BdPtr), XLLDMA_BD_STSCTRL_USR0_OFFSET,		\
		  (XLlDma_mBdRead((BdPtr), XLLDMA_BD_STSCTRL_USR0_OFFSET)) | 1 )

/* Used for debugging */
#define BdCsumEnabled(BdPtr)						\
  ((XLlDma_mBdRead((BdPtr), XLLDMA_BD_STSCTRL_USR0_OFFSET)) & 1)

#define BdCsumDisable(BdPtr)						\
  XLlDma_mBdWrite((BdPtr), XLLDMA_BD_STSCTRL_USR0_OFFSET,		\
		  (XLlDma_mBdRead((BdPtr), XLLDMA_BD_STSCTRL_USR0_OFFSET)) & 0xFFFFFFFE )

#define BdCsumSetup(BdPtr, Start, Insert)				\
  XLlDma_mBdWrite((BdPtr), XLLDMA_BD_USR1_OFFSET, (Start) << 16 | (Insert))

/* Used for debugging */
#define BdCsumInsert(BdPtr)					\
  (XLlDma_mBdRead((BdPtr), XLLDMA_BD_USR1_OFFSET) & 0xffff)

#define BdCsumSeed(BdPtr, Seed)				\
  XLlDma_mBdWrite((BdPtr), XLLDMA_BD_USR2_OFFSET, 0)

#define BdCsumGet(BdPtr)				\
  XLlDma_mBdRead((BdPtr), XLLDMA_BD_USR3_OFFSET)

#define BdGetRxLen(BdPtr)				\
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
  XLlTemac Emac;

  unsigned int fifo_irq;	/* fifo irq */
  unsigned int max_frame_size;

  int cur_speed;

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

/* Convenience function to enable a set of FIFO interrupt flags */
static void fifo_int_enable(struct net_local *lp, u32 flags) {
  u32 int_mask;

  int_mask = Read_Fifo32(lp->Emac, FIFO_IER_OFFSET);
  int_mask |= flags;
  Write_Fifo32(lp->Emac, FIFO_IER_OFFSET, int_mask);
}

static void fifo_int_disable(struct net_local *lp, u32 flags) {
  u32 int_mask;

  int_mask = Read_Fifo32(lp->Emac, FIFO_IER_OFFSET);
  int_mask &= ~flags;
  Write_Fifo32(lp->Emac, FIFO_IER_OFFSET, int_mask);
}

static void labx_eth_ll_mac_adjust_link(struct net_device *dev);

/* for exclusion of all program flows (processes, ISRs and BHs) */
spinlock_t XTE_spinlock = SPIN_LOCK_UNLOCKED;
spinlock_t XTE_tx_spinlock = SPIN_LOCK_UNLOCKED;
spinlock_t XTE_rx_spinlock = SPIN_LOCK_UNLOCKED;

/*
 * ethtool has a status reporting feature where we can report any sort of
 * status information we'd like. This is the list of strings used for that
 * status reporting. ETH_GSTRING_LEN is defined in ethtool.h
 */
static char labx_ethtool_gstrings_stats[][ETH_GSTRING_LEN] = {
  "txpkts", "txdropped", "txerr", "txfifoerr",
  "rxpkts", "rxdropped", "rxerr", "rxfifoerr",
  "rxrejerr", "max_frags", "tx_hw_csums", "rx_hw_csums",
};

#define XENET_STATS_LEN sizeof(labx_ethtool_gstrings_stats) / ETH_GSTRING_LEN

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
  XLlTemac_Start(InstancePtr);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
}

static inline void _XLlTemac_Stop(XLlTemac *InstancePtr)
{
  unsigned long flags;

  spin_lock_irqsave(&XTE_spinlock, flags);
  XLlTemac_Stop(InstancePtr);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
}

static inline void _XLlTemac_Reset(XLlTemac *InstancePtr, int HardCoreAction)
{
  unsigned long flags;

  spin_lock_irqsave(&XTE_spinlock, flags);
  XLlTemac_Reset(InstancePtr, HardCoreAction);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
}

static inline int _XLlTemac_SetMacAddress(XLlTemac *InstancePtr,
					  void *AddressPtr)
{
  int status;
  unsigned long flags;

  spin_lock_irqsave(&XTE_spinlock, flags);
  status = XLlTemac_SetMacAddress(InstancePtr, AddressPtr);
  spin_unlock_irqrestore(&XTE_spinlock, flags);

  return status;
}

static inline void _XLlTemac_GetMacAddress(XLlTemac *InstancePtr,
					   void *AddressPtr)
{
  unsigned long flags;

  spin_lock_irqsave(&XTE_spinlock, flags);
  XLlTemac_GetMacAddress(InstancePtr, AddressPtr);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
}

static inline int _XLlTemac_SetOptions(XLlTemac *InstancePtr, u32 Options)
{
  int status;
  unsigned long flags;

  spin_lock_irqsave(&XTE_spinlock, flags);
  status = XLlTemac_SetOptions(InstancePtr, Options);
  spin_unlock_irqrestore(&XTE_spinlock, flags);

  return status;
}

static inline int _XLlTemac_ClearOptions(XLlTemac *InstancePtr, u32 Options)
{
  int status;
  unsigned long flags;

  spin_lock_irqsave(&XTE_spinlock, flags);
  status = XLlTemac_ClearOptions(InstancePtr, Options);
  spin_unlock_irqrestore(&XTE_spinlock, flags);

  return status;
}

static inline u16 _XLlTemac_GetOperatingSpeed(XLlTemac *InstancePtr)
{
  u16 speed;
  unsigned long flags;

  spin_lock_irqsave(&XTE_spinlock, flags);
  speed = XLlTemac_GetOperatingSpeed(InstancePtr);
  spin_unlock_irqrestore(&XTE_spinlock, flags);

  return speed;
}

static inline void _XLlTemac_SetOperatingSpeed(XLlTemac *InstancePtr, u16 Speed)
{
  unsigned long flags;

  spin_lock_irqsave(&XTE_spinlock, flags);
  XLlTemac_SetOperatingSpeed(InstancePtr, Speed);
  spin_unlock_irqrestore(&XTE_spinlock, flags);

  /* We need a delay after we set the speed. Otherwise the PHY will not be ready. */
  udelay(10000);
}

static inline void _XLlTemac_PhySetMdioDivisor(XLlTemac *InstancePtr, u8 Divisor)
{
  unsigned long flags;

  spin_lock_irqsave(&XTE_spinlock, flags);
  XLlTemac_PhySetMdioDivisor(InstancePtr, Divisor);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
}

inline void _XLlTemac_PhyRead(XLlTemac *InstancePtr, u32 PhyAddress,
			      u32 RegisterNum, u16 *PhyDataPtr)
{
  unsigned long flags;

  spin_lock_irqsave(&XTE_spinlock, flags);
  XLlTemac_PhyRead(InstancePtr, PhyAddress, RegisterNum, PhyDataPtr);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
}

inline void _XLlTemac_PhyWrite(XLlTemac *InstancePtr, u32 PhyAddress,
			       u32 RegisterNum, u16 PhyData)
{
  unsigned long flags;

  spin_lock_irqsave(&XTE_spinlock, flags);
  XLlTemac_PhyWrite(InstancePtr, PhyAddress, RegisterNum, PhyData);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
}


static inline int _XLlTemac_MulticastClear(XLlTemac *InstancePtr, int Entry)
{
  int status;
  unsigned long flags;

  spin_lock_irqsave(&XTE_spinlock, flags);
  status = XLlTemac_MulticastClear(InstancePtr, Entry);
  spin_unlock_irqrestore(&XTE_spinlock, flags);

  return status;
}

static inline int _XLlTemac_SetMacPauseAddress(XLlTemac *InstancePtr, void *AddressPtr)
{
  int status;
  unsigned long flags;

  spin_lock_irqsave(&XTE_spinlock, flags);
  status = XLlTemac_SetMacPauseAddress(InstancePtr, AddressPtr);
  spin_unlock_irqrestore(&XTE_spinlock, flags);

  return status;
}

static inline void _XLlTemac_GetMacPauseAddress(XLlTemac *InstancePtr, void *AddressPtr)
{
  unsigned long flags;

  spin_lock_irqsave(&XTE_spinlock, flags);
  XLlTemac_GetMacPauseAddress(InstancePtr, AddressPtr);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
}

static inline int _XLlTemac_GetSgmiiStatus(XLlTemac *InstancePtr, u16 *SpeedPtr)
{
  int status;
  unsigned long flags;

  spin_lock_irqsave(&XTE_spinlock, flags);
  status = XLlTemac_GetSgmiiStatus(InstancePtr, SpeedPtr);
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
  status = XLlTemac_GetRgmiiStatus(InstancePtr, SpeedPtr, IsFullDuplexPtr, IsLinkUpPtr);
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
void reset(struct net_device *dev, u32 line_num)
{
  struct net_local *lp = netdev_priv(dev);
  u32 Options;
  static u32 reset_cnt = 0;

  printk(KERN_INFO "%s: labx_ethernet: resets (#%u) from adapter code line %d\n",
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
  Options = XLlTemac_GetOptions(&lp->Emac);

  /*
   * Reset the FIFO
   */
  Write_Fifo32(lp->Emac, FIFO_TDFR_OFFSET, FIFO_RESET_MAGIC);
  Write_Fifo32(lp->Emac, FIFO_RDFR_OFFSET, FIFO_RESET_MAGIC);

#if 0

  /* now we can reset the device */
  _XLlTemac_Reset(&lp->Emac, XTE_NORESET_HARD);

  /* Reset on TEMAC also resets PHY. Give it some time to finish negotiation
   * before we move on */
#endif

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
  Options = XLlTemac_GetOptions(&lp->Emac);
  printk(KERN_INFO "%s: labx_ethernet: Options: 0x%x\n", dev->name, Options);

  fifo_int_enable(lp, (FIFO_INT_TC_MASK | FIFO_INT_RC_MASK | 
		       FIFO_INT_RXERROR_MASK | FIFO_INT_TXERROR_MASK));

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

DECLARE_TASKLET(FifoRecvBH, FifoRecvHandler, 0);

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
  irq_status = Read_Fifo32(lp->Emac, FIFO_ISR_OFFSET);
  Write_Fifo32(lp->Emac, FIFO_ISR_OFFSET, irq_status);
  while (irq_status) {
    if (irq_status & FIFO_INT_RC_MASK) {
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
	fifo_int_disable(lp, FIFO_INT_ALL_MASK);
	tasklet_schedule(&FifoRecvBH);
      }
      spin_unlock_irqrestore(&receivedQueueSpin, flags);
      irq_status &= ~FIFO_INT_RC_MASK;
    } else if (irq_status & FIFO_INT_TC_MASK) {
      /* handle the transmit completion */
      FifoSendHandler(dev);
      irq_status &= ~FIFO_INT_TC_MASK;
    } else if (irq_status & FIFO_INT_TXERROR_MASK) {
      lp->stats.tx_errors++;
      lp->stats.tx_fifo_errors++;
      Write_Fifo32(lp->Emac, FIFO_TDFR_OFFSET, FIFO_RESET_MAGIC);
      irq_status &= ~FIFO_INT_TXERROR_MASK;
    } else if (irq_status & FIFO_INT_RXERROR_MASK) {
      lp->stats.rx_errors++;
      Write_Fifo32(lp->Emac, FIFO_RDFR_OFFSET, FIFO_RESET_MAGIC);
      irq_status &= ~FIFO_INT_RXERROR_MASK;
    } else {
      /* debug
       * if (irq_status == 0) printk("Temac: spurious fifo int\n");
       */
    }
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
    printk(KERN_ERR "%s: labx_ethernet: could not set MAC address.\n",
	   dev->name);
    return -EIO;
  }

  /*
   * If the device is not configured for polled mode, connect to the
   * interrupt controller and enable interrupts.  Currently, there
   * isn't any code to set polled mode, so this check is probably
   * superfluous.
   */
  Options = XLlTemac_GetOptions(&lp->Emac);
  Options |= XTE_FLOW_CONTROL_OPTION;
  Options |= XTE_JUMBO_OPTION;
  Options |= XTE_TRANSMITTER_ENABLE_OPTION;
  Options |= XTE_RECEIVER_ENABLE_OPTION;
#if XTE_AUTOSTRIPPING
  Options |= XTE_FCS_STRIP_OPTION;
#endif

  (int) _XLlTemac_SetOptions(&lp->Emac, Options);
  (int) _XLlTemac_ClearOptions(&lp->Emac, ~Options);
  Options = XLlTemac_GetOptions(&lp->Emac);
  printk(KERN_INFO "%s: labx_ethernet: Options: 0x%x\n", dev->name, Options);

  printk(KERN_INFO
	 "%s: labx_ethernet: allocating interrupt %d for fifo mode.\n",
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
	   "%s: labx_ethernet: could not allocate interrupt %d.\n",
	   dev->name, lp->fifo_irq);
    return irqval;
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
    }
  }

  /* Reset the FIFO core */
  Write_Fifo32(lp->Emac, FIFO_TDFR_OFFSET, FIFO_RESET_MAGIC);
  Write_Fifo32(lp->Emac, FIFO_RDFR_OFFSET, FIFO_RESET_MAGIC);

  /* Enable FIFO interrupts  - no polled mode */
  fifo_int_enable(lp, (FIFO_INT_TC_MASK | FIFO_INT_RC_MASK | 
		       FIFO_INT_RXERROR_MASK | FIFO_INT_TXERROR_MASK));

  /* Start TEMAC device */
  _XLlTemac_Start(&lp->Emac);

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
  free_irq(lp->fifo_irq, dev);

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

  u32 Options = XLlTemac_GetOptions(&lp->Emac);

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
    printk(KERN_ERR "labx_ethernet: could not set MAC address.\n");
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
  int word_index;
  u32 word_len;
  u32 *buf_ptr;

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

  fifo_free_bytes = (Read_Fifo32(lp->Emac, FIFO_TDFV_OFFSET) << 2);
  if (fifo_free_bytes < total_len) {
    netif_stop_queue(dev);	/* stop send queue */
    lp->deferred_skb = skb;	/* buffer the sk_buffer and will send
				   it in interrupt context */
    spin_unlock_irqrestore(&XTE_tx_spinlock, flags);
    return 0;
  }

  /* Write frame data to FIFO, starting with the header and following
   * up with each of the fragments
   */
  word_len = ((skb_headlen(skb) + 3) >> 2);
  buf_ptr = (u32*) skb->data;
  for(word_index = 0; word_index < word_len; word_index++) {
    Write_Fifo32(lp->Emac, FIFO_TDFD_OFFSET, htonl(*buf_ptr++));
  }

  frag = &skb_shinfo(skb)->frags[0];
  for (i = 1; i < total_frags; i++, frag++) {
    word_len = ((frag->size + 3) >> 2);
    buf_ptr = (u32*) (page_address(frag->page) + frag->page_offset);
    for(word_index = 0; word_index < word_len; word_index++) {
      Write_Fifo32(lp->Emac, FIFO_TDFD_OFFSET, htonl(*buf_ptr++));
    }
  }

  /* Initiate transmit */
  Write_Fifo32(lp->Emac, FIFO_TLF_OFFSET, total_len);
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
  unsigned long flags;

  spin_lock_irqsave(&XTE_tx_spinlock, flags);
  lp = netdev_priv(dev);
  lp->stats.tx_packets++;

  /*Send out the deferred skb and wake up send queue if a deferred skb exists */
  if (lp->deferred_skb) {
    int total_frags;
    unsigned int total_len;
    unsigned long fifo_free_bytes;
    skb_frag_t *frag;
    int i;
    int word_index;
    u32 word_len;
    u32 *buf_ptr;

    skb = lp->deferred_skb;
    total_frags = skb_shinfo(skb)->nr_frags + 1;
    total_len = skb_headlen(skb);

    frag = &skb_shinfo(skb)->frags[0];
    for (i = 1; i < total_frags; i++, frag++) {
      total_len += frag->size;
    }

    fifo_free_bytes = (Read_Fifo32(lp->Emac, FIFO_TDFV_OFFSET) << 2);
    if (fifo_free_bytes < total_len) {
      /* If still no room for the deferred packet, return */
      spin_unlock_irqrestore(&XTE_tx_spinlock, flags);
      return;
    }

    /* Write frame data to FIFO, starting with the header and following
     * up with each of the fragments
     */
    word_len = ((skb_headlen(skb) + 3) >> 2);
    buf_ptr = (u32*) skb->data;
    for(word_index = 0; word_index < word_len; word_index++) {
      Write_Fifo32(lp->Emac, FIFO_TDFD_OFFSET, htonl(*buf_ptr++));
    }

    frag = &skb_shinfo(skb)->frags[0];
    for (i = 1; i < total_frags; i++, frag++) {
      word_len = ((frag->size + 3) >> 2);
      buf_ptr = (u32*) (page_address(frag->page) + frag->page_offset);
      for(word_index = 0; word_index < word_len; word_index++) {
	Write_Fifo32(lp->Emac, FIFO_TDFD_OFFSET, htonl(*buf_ptr++));
      }
    }

    /* Initiate transmit */
    Write_Fifo32(lp->Emac, FIFO_TLF_OFFSET, total_len);

    dev_kfree_skb(skb);	/* free skb */
    lp->deferred_skb = NULL;
    lp->stats.tx_packets++;
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
	 "%s: labx_ethernet: exceeded transmit timeout of %lu ms.  Resetting emac.\n",
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
  u32 word_len;
  u32 word_index;
  u32 *buf_ptr;

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

  /* The Rx FIFO occupancy always reflects whether there is packet data to
   * be consumed still
   */
  while(Read_Fifo32(lp->Emac, FIFO_RDFO_OFFSET) != 0) {
    /* Read the Rx length register to get the length and "lock in" the packet */
    len = Read_Fifo32(lp->Emac, FIFO_RLF_OFFSET);
    word_len = ((len + 3) >> 2);

    if (!(skb = alloc_skb(len + ALIGNMENT_RECV, GFP_ATOMIC))) {
      /* Couldn't get memory. */
      lp->stats.rx_dropped++;
      printk(KERN_ERR
	     "%s: labx_ethernet: could not allocate receive buffer.\n",
	     dev->name);
	    
      /* Consume the packet data anyways to keep the FIFO coherent */
      for(word_index = 0; word_index < word_len; word_index++) {
	Read_Fifo32(lp->Emac, FIFO_RDFD_OFFSET);
      }
      break;
    }

    /* Read the packet data; the occupancy register has already been
     * tested; now read the packet length and then the corresponding
     * number of words.
     */
    buf_ptr = (u32*) skb->data;
    for(word_index = 0; word_index < word_len; word_index++) {
      *buf_ptr++ = ntohl(Read_Fifo32(lp->Emac, FIFO_RDFD_OFFSET));
    }
    lp->stats.rx_packets++;
    lp->stats.rx_bytes += len;

    skb_put(skb, len);	/* Tell the skb how much data we got. */
    skb->dev = dev;		/* Fill out required meta-data. */
    skb->protocol = eth_type_trans(skb, dev);
    skb->ip_summed = CHECKSUM_UNNECESSARY;
    netif_rx(skb);		/* Send the packet upstream. */
  }
  
  fifo_int_enable(lp, (FIFO_INT_TC_MASK | FIFO_INT_RC_MASK |
		       FIFO_INT_RXERROR_MASK | FIFO_INT_TXERROR_MASK));

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

  mac_options = XLlTemac_GetOptions(&(lp->Emac));
  gmii_cmd = phy_read(lp->phy_dev, MII_BMCR);
  gmii_status = phy_read(lp->phy_dev, MII_BMSR);
  gmii_advControl = phy_read(lp->phy_dev, MII_ADVERTISE);

  ecmd->duplex = DUPLEX_FULL;

  ecmd->supported |= SUPPORTED_MII;

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

static int
labx_ethtool_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *ed)
{
  memset(ed, 0, sizeof(struct ethtool_drvinfo));
  strncpy(ed->driver, DRIVER_NAME, sizeof(ed->driver) - 1);
  strncpy(ed->version, DRIVER_VERSION, sizeof(ed->version) - 1);
  /* Also tell how much memory is needed for dumping register values */
  ed->regdump_len = sizeof(u16) * EMAC_REGS_N;
  return 0;
}

/* ethtool operations structure */
static const struct ethtool_ops labx_ethtool_ops = {
  .get_settings = labx_ethtool_get_settings,
  .set_settings = labx_ethtool_set_settings,
  .get_drvinfo  = labx_ethtool_get_drvinfo,
  .get_regs     = labx_ethtool_get_regs
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

/* DEPRECATED ethtool ioctl() */
#if 0
static int xenet_do_ethtool_ioctl(struct net_device *dev, struct ifreq *rq)
{
  struct net_local *lp = netdev_priv(dev);
  struct ethtool_cmd ecmd;
  struct ethtool_drvinfo edrv;
  struct ethtool_pauseparam epp;
  struct mac_regsDump regs;
  int ret = -EOPNOTSUPP;
  u32 Options;

  if (copy_from_user(&ecmd, rq->ifr_data, sizeof(ecmd)))
    return -EFAULT;
  switch (ecmd.cmd) {
  case ETHTOOL_GSET:	/* Get setting. No command option needed w/ ethtool */
    ret = labx_ethtool_get_settings(dev, &ecmd);
    if (ret < 0)
      return -EIO;
    if (copy_to_user(rq->ifr_data, &ecmd, sizeof(ecmd)))
      return -EFAULT;
    ret = 0;
    break;
  case ETHTOOL_SSET:	/* Change setting. Use "-s" command option w/ ethtool */
    ret = labx_ethtool_set_settings(dev, &ecmd);
    break;
  case ETHTOOL_GPAUSEPARAM:	/* Get pause parameter information. Use "-a" w/ ethtool */
    ret = labx_ethtool_get_settings(dev, &ecmd);
    if (ret < 0)
      return ret;
    epp.cmd = ecmd.cmd;
    epp.autoneg = ecmd.autoneg;
    Options = XLlTemac_GetOptions(&lp->Emac);
    if (Options & XTE_FCS_INSERT_OPTION) {
      epp.rx_pause = 1;
      epp.tx_pause = 1;
    }
    else {
      epp.rx_pause = 0;
      epp.tx_pause = 0;
    }
    if (copy_to_user
	(rq->ifr_data, &epp, sizeof(struct ethtool_pauseparam)))
      return -EFAULT;
    ret = 0;
    break;
  case ETHTOOL_SPAUSEPARAM:	/* Set pause parameter. Use "-A" w/ ethtool */
    return -EOPNOTSUPP;	/* TODO: To support in next version */
  case ETHTOOL_GRXCSUM:{	/* Get rx csum offload info. Use "-k" w/ ethtool */
    struct ethtool_value edata = { ETHTOOL_GRXCSUM };

    edata.data =
      (lp->local_features & LOCAL_FEATURE_RX_CSUM) !=
      0;
    if (copy_to_user(rq->ifr_data, &edata, sizeof(edata)))
      return -EFAULT;
    ret = 0;
    break;
  }
  case ETHTOOL_SRXCSUM:{	/* Set rx csum offload info. Use "-K" w/ ethtool */
    struct ethtool_value edata;

    if (copy_from_user(&edata, rq->ifr_data, sizeof(edata)))
      return -EFAULT;

    if (edata.data) {
      if (XLlTemac_IsRxCsum(&lp->Emac) == TRUE) {
	lp->local_features |=
	  LOCAL_FEATURE_RX_CSUM;
      }
    }
    else {
      lp->local_features &= ~LOCAL_FEATURE_RX_CSUM;
    }

    ret = 0;
    break;
  }
  case ETHTOOL_GTXCSUM:{	/* Get tx csum offload info. Use "-k" w/ ethtool */
    struct ethtool_value edata = { ETHTOOL_GTXCSUM };

    edata.data = (dev->features & NETIF_F_IP_CSUM) != 0;
    if (copy_to_user(rq->ifr_data, &edata, sizeof(edata)))
      return -EFAULT;
    ret = 0;
    break;
  }
  case ETHTOOL_STXCSUM:{	/* Set tx csum offload info. Use "-K" w/ ethtool */
    struct ethtool_value edata;

    if (copy_from_user(&edata, rq->ifr_data, sizeof(edata)))
      return -EFAULT;

    if (edata.data) {
      if (XLlTemac_IsTxCsum(&lp->Emac) == TRUE) {
	dev->features |= NETIF_F_IP_CSUM;
      }
    }
    else {
      dev->features &= ~NETIF_F_IP_CSUM;
    }

    ret = 0;
    break;
  }
  case ETHTOOL_GSG:{	/* Get ScatterGather info. Use "-k" w/ ethtool */
    struct ethtool_value edata = { ETHTOOL_GSG };

    edata.data = (dev->features & NETIF_F_SG) != 0;
    if (copy_to_user(rq->ifr_data, &edata, sizeof(edata)))
      return -EFAULT;
    ret = 0;
    break;
  }
  case ETHTOOL_SSG:{	/* Set ScatterGather info. Use "-K" w/ ethtool */
    struct ethtool_value edata;

    if (copy_from_user(&edata, rq->ifr_data, sizeof(edata)))
      return -EFAULT;

    if (edata.data) {
      /* Features for DMA, preserve for future */
      /*
	if (XLlTemac_IsDma(&lp->Emac)) {
	dev->features |=
	NETIF_F_SG | NETIF_F_FRAGLIST;
	}
      */
    }
    else {
      dev->features &=
	~(NETIF_F_SG | NETIF_F_FRAGLIST);
    }

    ret = 0;
    break;
  }
  case ETHTOOL_GCOALESCE:	/* Get coalescing info. Use "-c" w/ ethtool */
    /* For the moment, break since no DMA is supported */
    break;

  case ETHTOOL_SCOALESCE:	/* Set coalescing info. Use "-C" w/ ethtool */
    /* For the moment, break since no DMA is supported */
    break;

  case ETHTOOL_GDRVINFO:	/* Get driver information. Use "-i" w/ ethtool */
    edrv.cmd = edrv.cmd;
    ret = labx_ethtool_get_drvinfo(dev, &edrv);
    if (ret < 0) {
      return -EIO;
    }
    edrv.n_stats = XENET_STATS_LEN;
    if (copy_to_user
	(rq->ifr_data, &edrv, sizeof(struct ethtool_drvinfo))) {
      return -EFAULT;
    }
    ret = 0;
    break;
  case ETHTOOL_GREGS:	/* Get register values. Use "-d" with ethtool */
    regs.hd.cmd = edrv.cmd;
    labx_ethtool_get_regs(dev, &(regs.hd), &ret);
    if (ret < 0) {
      return ret;
    }
    if (copy_to_user
	(rq->ifr_data, &regs, sizeof(struct mac_regsDump))) {
      return -EFAULT;
    }
    ret = 0;
    break;
  case ETHTOOL_GRINGPARAM:	/* Get RX/TX ring parameters. Use "-g" w/ ethtool */
    /* For the moment, return error since no DMA is supported */
    return -EFAULT;
    break;

  case ETHTOOL_NWAY_RST:	/* Restart auto negotiation if enabled. Use "-r" w/ ethtool */
    return -EOPNOTSUPP;	/* TODO: To support in next version */
  case ETHTOOL_GSTRINGS:{
    struct ethtool_gstrings gstrings = { ETHTOOL_GSTRINGS };
    void *addr = rq->ifr_data;
    char *strings = NULL;

    if (copy_from_user(&gstrings, addr, sizeof(gstrings))) {
      return -EFAULT;
    }
    switch (gstrings.string_set) {
    case ETH_SS_STATS:
      gstrings.len = XENET_STATS_LEN;
      strings = *labx_ethtool_gstrings_stats;
      break;
    default:
      return -EOPNOTSUPP;
    }
    if (copy_to_user(addr, &gstrings, sizeof(gstrings))) {
      return -EFAULT;
    }
    addr += offsetof(struct ethtool_gstrings, data);
    if (copy_to_user
	(addr, strings, gstrings.len * ETH_GSTRING_LEN)) {
      return -EFAULT;
    }
    ret = 0;
    break;
  }
  case ETHTOOL_GSTATS:{
    struct {
      struct ethtool_stats cmd;
      uint64_t data[XENET_STATS_LEN];
    } stats = { {
	ETHTOOL_GSTATS, XENET_STATS_LEN}};

    stats.data[0] = lp->stats.tx_packets;
    stats.data[1] = lp->stats.tx_dropped;
    stats.data[2] = lp->stats.tx_errors;
    stats.data[3] = lp->stats.tx_fifo_errors;
    stats.data[4] = lp->stats.rx_packets;
    stats.data[5] = lp->stats.rx_dropped;
    stats.data[6] = lp->stats.rx_errors;
    stats.data[7] = lp->stats.rx_fifo_errors;
    stats.data[8] = lp->stats.rx_crc_errors;
    stats.data[9] = lp->max_frags_in_a_packet;
    stats.data[10] = lp->tx_hw_csums;
    stats.data[11] = lp->rx_hw_csums;

    if (copy_to_user(rq->ifr_data, &stats, sizeof(stats))) {
      return -EFAULT;
    }
    ret = 0;
    break;
  }

  case ETHTOOL_GLINK:{
    struct ethtool_value edata = { ETHTOOL_GLINK };

    edata.data = (netif_carrier_ok(dev) == 0) ? 0 : 1;

    if (copy_to_user(rq->ifr_data, &edata, sizeof(edata)))
      return -EFAULT;
    ret = 0;
    break;
  }

  default:
    return -EOPNOTSUPP;	/* All other operations not supported */
  }
  return ret;
}
#endif /* DEPRECATED ethtool ioctl() */

static int xenet_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
  struct net_local *lp = netdev_priv(dev);

  /* gmii_ioctl_data has 4 u16 fields: phy_id, reg_num, val_in & val_out */
  struct mii_ioctl_data *data = (struct mii_ioctl_data *) &rq->ifr_data;

  switch (cmd) {
  case SIOCGMIIPHY:	/* Get address of GMII PHY in use. */
  case SIOCGMIIREG:	/* Read GMII PHY register. */
  case SIOCSMIIREG:	/* Write GMII PHY register. */
    if (NULL == lp->phy_dev) {
      return -ENODEV;
    }

    return phy_mii_ioctl(lp->phy_dev, data, cmd);

  case SIOCDEVPRIVATE + 3:	/* set THRESHOLD */
  case SIOCDEVPRIVATE + 4:	/* set WAITBOUND */
  case SIOCDEVPRIVATE + 5:	/* get THRESHOLD */
  case SIOCDEVPRIVATE + 6:	/* get WAITBOUND */
    /* For the moment, return error since no DMA is supported */
    return -EFAULT;

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
	  XLlTemac_SetOperatingSpeed(&lp->Emac, lp->phy_dev->speed);
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
static int xtenet_setup(struct device *dev,
			struct resource *r_mem,
			struct resource *r_irq,
			struct labx_eth_platform_data *pdata) {
  u32 virt_baddr;		/* virtual base address of TEMAC */
  int i;

  XLlTemac_Config Temac_Config;

  struct net_device *ndev = NULL;
  struct net_local *lp = NULL;

  int rc = 0;
  u32 address_size;

  /* Create an ethernet device instance */
  ndev = alloc_etherdev(sizeof(struct net_local));
  if (!ndev) {
    dev_err(dev, "labx_ethernet: Could not allocate net device.\n");
    rc = -ENOMEM;
    goto error;
  }
  dev_set_drvdata(dev, ndev);

  /* Initialize the private data used by XEmac_LookupConfig().
   * The private data are zeroed out by alloc_etherdev() already.
   */
  lp = netdev_priv(ndev);
  lp->ndev = ndev;
  lp->fifo_irq = pdata->fifo_irq;
  strncpy(lp->phy_name,pdata->phy_name,64);

  /* Setup the Config structure for the XLlTemac_CfgInitialize() call. */
  Temac_Config.BaseAddress = r_mem->start;
  Temac_Config.TxCsum = pdata->tx_csum;
  Temac_Config.RxCsum = pdata->rx_csum;
  Temac_Config.PhyType = pdata->phy_type;
  Temac_Config.PhyAddr = pdata->phy_addr;

  /* Request the memory range for the device */
  address_size = (r_mem->end - r_mem->start + 1);
  if(request_mem_region(r_mem->start, address_size, "labx-ethernet") == NULL) {
    rc = -ENOMEM;
    goto error;
  }

  /* Get the virtual base address for the device */
  virt_baddr = (u32) ioremap_nocache(r_mem->start, address_size);
  if (0 == virt_baddr) {
    dev_err(dev, "labx_ethernet: Could not allocate iomem.\n");
    rc = -EIO;
    goto error;
  }

  if (XLlTemac_CfgInitialize(&lp->Emac, &Temac_Config, virt_baddr) !=
      XST_SUCCESS) {
    dev_err(dev, "labx_ethernet: Could not initialize device.\n");

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
    dev_err(dev, "labx_ethernet: could not set MAC address.\n");
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


  /* Assign the FIFO transmit callback */
  Write_Fifo32(lp->Emac, FIFO_TDFR_OFFSET, FIFO_RESET_MAGIC);
  Write_Fifo32(lp->Emac, FIFO_RDFR_OFFSET, FIFO_RESET_MAGIC);
  ndev->hard_start_xmit = xenet_FifoSend;

  /* initialize the netdev structure */
  ndev->open = xenet_open;
  ndev->stop = xenet_close;
  ndev->change_mtu = xenet_change_mtu;
  ndev->get_stats = xenet_get_stats;
  ndev->set_multicast_list = xenet_set_multicast_list;
  ndev->set_mac_address = xenet_set_mac_address;
  ndev->flags &= ~IFF_MULTICAST;
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
    (XLlTemac_GetOptions(&(lp->Emac)) & XTE_FCS_STRIP_OPTION) != 0;
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
  XLlTemac_WriteReg(lp->Emac.Config.BaseAddress, INT_MASK_REG, NO_IRQS);
  XLlTemac_WriteReg(lp->Emac.Config.BaseAddress, INT_FLAGS_REG, (PHY_IRQ_MASK | MDIO_IRQ_MASK));
  XLlTemac_WriteReg(lp->Emac.Config.BaseAddress, INT_MASK_REG, (PHY_IRQ_LOW | MDIO_IRQ_MASK));

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
  struct labx_eth_platform_data *pdata;
  struct platform_device *pdev = to_platform_device(dev);

  /* param check */
  if (!pdev) {
    dev_err(dev, "labx_ethernet: Internal error. Probe called with NULL param.\n");
    return -ENODEV;
  }

  pdata = (struct labx_eth_platform_data *) pdev->dev.platform_data;
  if (!pdata) {
    dev_err(dev, "labx_ethernet: Couldn't find platform data.\n");

    return -ENODEV;
  }

  /* Get iospace and an irq for the device */
  r_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if(!r_mem) {
    dev_err(dev, "labx_ethernet: IO resource MEM not found.\n");
    return -ENODEV;
  }
  r_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

  return(xtenet_setup(dev, r_mem, r_irq, pdata));
}


static int xtenet_probe(struct device *dev)
{
  kthread_run(xtenet_probe_thread, dev, "xtenet_probe_thread");
  return 0;
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

/* Note: This must be <= MII_BUS_ID_SIZE which is currently 17 (including trailing '\0') */
#define MDIO_OF_BUSNAME_FMT "labxeth%08x"

static int __devinit xtenet_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
  struct resource r_irq_struct;
  struct resource r_irq_fifo_struct;
  struct resource r_irq_phy_struct;
  struct resource r_mem_struct;
  struct resource r_connected_mdio_mem_struct;
  struct resource *r_irq      = &r_irq_struct;      /* Interrupt resources */
  struct resource *r_irq_fifo = &r_irq_fifo_struct; /* Interrupt resources */
  struct resource *r_irq_phy  = &r_irq_phy_struct;  /* Interrupt resources */
  struct resource *r_mem      = &r_mem_struct;      /* IO mem resources */

  struct labx_eth_platform_data pdata_struct = {};

  struct labx_eth_platform_data *pdata = &pdata_struct;
  const void *mac_address;
  int rc = 0;
  const phandle *mdio_controller_handle;
  struct device_node *mdio_controller_node;
  u32 phy_addr;
  int i;

  printk(KERN_INFO "Device Tree Probing \'%s\'\n",ofdev->node->name);

  /* Get iospace for the device */
  rc = of_address_to_resource(ofdev->node, 0, r_mem);
  if(rc) {
    dev_warn(&ofdev->dev, "invalid address\n");
    return rc;
  }

  /* Get IRQ for the device; this primary one handles MDIO and *can*
   * also multiplex the PHY IRQ if configured to
   */
  rc = of_irq_to_resource(ofdev->node, 0, r_irq);
  if(rc == NO_IRQ) {
    r_irq = NULL;
  }

  /* Get IRQ for the FIFO logic */
  rc = of_irq_to_resource(ofdev->node, 1, r_irq_fifo);
  if(rc == NO_IRQ) {
    dev_err(&ofdev->dev, "labx_ethernet: No FIFO IRQ found.\n");
    r_irq_fifo = NULL;
  }
  pdata_struct.fifo_irq = r_irq_fifo->start;

  /* Get IRQ of the attached phy (if any) */
  rc = of_irq_to_resource(ofdev->node, 2, r_irq_phy);
  if(rc == NO_IRQ) {
    r_irq_phy = NULL;
  }

  pdata_struct.tx_csum  = get_u32(ofdev, "xlnx,txcsum");
  pdata_struct.rx_csum  = get_u32(ofdev, "xlnx,rxcsum");

  /* Connected PHY information */
  pdata_struct.phy_type = get_u32(ofdev, "xlnx,phy-type");
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
      /* The MDIO controller node is itself the entity able to talk over MDIO;
       * it is not a compound device.
       */
      rc = of_address_to_resource(mdio_controller_node, 0, &r_connected_mdio_mem_struct);
      snprintf(pdata->phy_name, BUS_ID_SIZE, MDIO_OF_BUSNAME_FMT ":%02x", (u32)r_connected_mdio_mem_struct.start, phy_addr);
      printk("%s:phy_name: %s\n",__func__, pdata->phy_name);
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
  { .compatible = "xlnx,labx-ethernet-1.00.a", },
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
