#ifndef XLLTEMAC_COMMON_H
#define XLLTEMAC_COMMON_H

/* spinlock for hardware registers access */
extern spinlock_t XTE_spinlock;

#define XTE_AUTOSTRIPPING 1

/*
 * Our private per device data.  When a net_device is allocated we will
 * ask for enough extra space for this.
 */
struct xlltemac_net_local {
  struct list_head rcv;
  struct list_head xmit;
  
  struct net_device *ndev;	/* this device */
  struct net_device *next_dev;	/* The next device in dev_list */
  struct net_device_stats stats;	/* Statistics for this device */
  struct timer_list phy_timer;	/* PHY monitoring timer */
  
  u32 index;		/* Which interface is this */
#if 0
  XInterruptHandler Isr;	/* Pointer to the XLlTemac ISR routine */
#endif
  u8 gmii_addr;		/* The GMII address of the PHY */
  
  u8 led_blink;           /* slow blink state */
  
#ifdef PHY_USE_RESET_FLAG
  u8 reset_flag;          /* reset flag -- originally 0,
			     set to 1 to indicate that subsequent phy
			     resets will be necessary  */
#endif
  unsigned long poll_reset_time;   /* moment when PHY should be reset if it
				      still didn't get carrier */
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
};

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

static inline void _XLlTemac_SetOperatingSpeed(XLlTemac *InstancePtr,
					       u16 Speed)
{
  unsigned long flags;
  
  spin_lock_irqsave(&XTE_spinlock, flags);
  XLlTemac_SetOperatingSpeed(InstancePtr, Speed);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
  
  /*
    We need a delay after we set the speed. Otherwise the PHY
    will not be ready.
  */
  udelay(10000);
}

static inline int _XLlTemac_MulticastAdd(XLlTemac *InstancePtr,
					 void *AddressPtr, int Entry)
{
  int status;
  unsigned long flags;
  
  spin_lock_irqsave(&XTE_spinlock, flags);
  status = XLlTemac_MulticastAdd(InstancePtr, AddressPtr, Entry);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
  
  return status;
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

static inline int _XLlTemac_SetMacPauseAddress(XLlTemac *InstancePtr,
					       void *AddressPtr)
{
  int status;
  unsigned long flags;
  
  spin_lock_irqsave(&XTE_spinlock, flags);
  status = XLlTemac_SetMacPauseAddress(InstancePtr, AddressPtr);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
  
  return status;
}

static inline void _XLlTemac_GetMacPauseAddress(XLlTemac *InstancePtr,
						void *AddressPtr)
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
  status = XLlTemac_GetRgmiiStatus(InstancePtr, SpeedPtr, IsFullDuplexPtr,
				   IsLinkUpPtr);
  spin_unlock_irqrestore(&XTE_spinlock, flags);
  
  return status;
}

#endif
