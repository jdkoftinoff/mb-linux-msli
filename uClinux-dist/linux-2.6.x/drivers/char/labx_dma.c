/*
 *  linux/drivers/net/labx_avb/labx_dma.c
 *
 *  Lab X Technologies DMA coprocessor
 *
 *  Written by Eldridge Chris Wulff (chris.wulff@labxtechnologies.com)
 *
 *  Copyright (C) 2009 Lab X Technologies LLC, All Rights Reserved.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/labx_dma.h>
#include <linux/types.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <xio.h>
#include <linux/interrupt.h>
#include <linux/labx_dma_coprocessor_defs.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/sched.h>

#define DRIVER_VERSION_MIN  0x10
#define DRIVER_VERSION_MAX  0x12
#define CAPS_INDEX_VERSION  0x11 /* First version with # index counters in CAPS word */

/* Number of milliseconds we will wait before bailing out of a synced write */
#define SYNCED_WRITE_TIMEOUT_MSECS  (100)

/* Interrupt service routine for the instance */
static irqreturn_t labx_dma_interrupt(int irq, void *dev_id) {
  struct labx_dma *dma = (struct labx_dma*) dev_id;
  uint32_t maskedFlags;
  uint32_t irqMask;

  /* Read the interrupt flags and immediately clear them */
  maskedFlags = XIo_In32(DMA_REGISTER_ADDRESS(dma, DMA_IRQ_FLAGS_REG));
  irqMask = XIo_In32(DMA_REGISTER_ADDRESS(dma, DMA_IRQ_ENABLE_REG));
  maskedFlags &= irqMask;
  XIo_Out32(DMA_REGISTER_ADDRESS(dma, DMA_IRQ_FLAGS_REG), maskedFlags);

  /* Detect the synchronized write IRQ */
  if((maskedFlags & DMA_SYNC_IRQ) != 0) {
    /* Wake up all threads waiting for a synchronization event */
    wake_up_interruptible(&(dma->syncedWriteQueue));
  }

  /* Detect the status ready IRQ */
  if((maskedFlags & DMA_STAT_READY_IRQ) != 0) {
    /* TODO - Actually pull the data... */
    printk("DMA status ready!\n");
  }

  return(IRQ_HANDLED);
}

int32_t labx_dma_probe(struct labx_dma *dma, 
                       const char *name, 
                       int32_t microcodeWords, 
                       int32_t irq) {
  uint32_t capsWord;
  uint32_t versionWord;
  uint32_t versionMajor;
  uint32_t versionMinor;
  uint32_t versionCompare;
  uint32_t maxMicrocodeWords;
  int32_t returnValue = 0;

  /* Retain the name of the encapsulating device instance, as well as the IRQ */
  dma->name = name;
  dma->irq  = irq;
  
  /* Read the capabilities word to determine how many of the lowest
   * address bits are used to index into the microcode RAM, and therefore how
   * many bits an address sub-range field gets shifted up.  Each instruction is
   * 32 bits, and therefore inherently eats two lower address bits.
   */
  capsWord = XIo_In32(DMA_REGISTER_ADDRESS(dma, DMA_CAPABILITIES_REG));

  /* Inspect and check the version */
  versionWord = XIo_In32(DMA_REGISTER_ADDRESS(dma, DMA_REVISION_REG));
  versionMajor = ((versionWord >> DMA_REVISION_FIELD_BITS) & DMA_REVISION_FIELD_MASK);
  versionMinor = (versionWord & DMA_REVISION_FIELD_MASK);
  versionCompare = ((versionMajor << DMA_REVISION_FIELD_BITS) | versionMinor);
  if((versionCompare < DRIVER_VERSION_MIN) | 
     (versionCompare > DRIVER_VERSION_MAX)) {
    dma->regionShift = 0;

    printk(KERN_INFO "Found incompatible hardware version %d.%d at %p\n",
           versionMajor, versionMinor, dma->virtualAddress);
    return(-1);
  }

  /* Decode the various bits in the capabilities word */
  if (versionCompare < CAPS_INDEX_VERSION) {
    dma->capabilities.indexCounters = 4;
  } else {
    dma->capabilities.indexCounters = (capsWord >> DMA_CAPS_INDEX_SHIFT) & DMA_CAPS_INDEX_MASK;
  }
  dma->capabilities.alus = (capsWord >> DMA_CAPS_ALU_SHIFT) & DMA_CAPS_ALU_MASK;
  dma->capabilities.dmaChannels = (capsWord >> DMA_CAPS_CHANNELS_SHIFT) & DMA_CAPS_CHANNELS_MASK;
  dma->capabilities.parameterAddressBits = (capsWord >> DMA_CAPS_PARAM_ADDRESS_BITS_SHIFT) & DMA_CAPS_PARAM_ADDRESS_BITS_MASK;
  dma->capabilities.codeAddressBits = (capsWord >> DMA_CAPS_CODE_ADDRESS_BITS_SHIFT) & DMA_CAPS_CODE_ADDRESS_BITS_MASK;
  dma->regionShift = (dma->capabilities.codeAddressBits + 2);

  /* Either infer the number of microcode words available from the code address bits,
   * or sanity check the specified amount against the same.
   */
  maxMicrocodeWords = (0x01 << dma->capabilities.codeAddressBits);
  if(microcodeWords < 0) {
    /* Encapsulating device doesn't know the exact microcode size, assume that
     * the full microcode address space is available for use.
     */
    dma->capabilities.microcodeWords = maxMicrocodeWords;
  } else {
    /* Sanity-check the value we've been provided */
    if(microcodeWords > maxMicrocodeWords) {
      printk(KERN_INFO "(labx-dma, \"%s\") : Microcode size (%d) exceeds maximum of %d words\n",
             dma->name, microcodeWords, maxMicrocodeWords);
      return(-1);
    }
    dma->capabilities.microcodeWords = microcodeWords;
  }

  /* Check to see if the hardware has a status FIFO */
  dma->capabilities.hasStatusFifo = 
    ((capsWord & DMA_CAPS_STATUS_FIFO_BIT) ? DMA_HAS_STATUS_FIFO : DMA_NO_STATUS_FIFO);

  /* Initialize the waitqueue used for synchronized writes */
  init_waitqueue_head(&(dma->syncedWriteQueue));

  /* Initialize the data structures storing status information */
  spin_lock_init(&dma->statusMutex);
  dma->statusHead = &dma->statusPackets[0];
  dma->statusTail = &dma->statusPackets[0];

  /* Ensure that no interrupts are enabled */
  XIo_Out32(DMA_REGISTER_ADDRESS(dma, DMA_IRQ_ENABLE_REG), DMA_NO_IRQS);

  /* Request the IRQ if one was supplied */
  if(dma->irq != DMA_NO_IRQ_SUPPLIED) {
    uint32_t irqMask;

    returnValue = request_irq(dma->irq, 
                              &labx_dma_interrupt, 
                              IRQF_DISABLED, 
                              dma->name, 
                              dma);
    if (returnValue) {
      printk(KERN_ERR "%s : Could not allocate Lab X DMA interrupt (%d).\n",
             dma->name, dma->irq);
    }

    /* Clear all interrupt flags at the beginning and enable the IRQs:
     *
     * DMA_SYNC_IRQ - Used to detect successful synchronized writes
     * DMA_STAT_READY_IRQ - Used to detect status data arrival in the status FIFO
     *                      (if so equipped).
     */
    irqMask = DMA_SYNC_IRQ;
    if(dma->capabilities.hasStatusFifo == DMA_HAS_STATUS_FIFO) {
      irqMask |= DMA_STAT_READY_IRQ;
    }
    XIo_Out32(DMA_REGISTER_ADDRESS(dma, DMA_IRQ_FLAGS_REG), DMA_ALL_IRQS);
    XIo_Out32(DMA_REGISTER_ADDRESS(dma, DMA_IRQ_ENABLE_REG), irqMask);
  }

  /* Make a note if the instance is inferring its microcode size */
  printk(KERN_INFO "\nFound DMA unit %d.%d at %p: %d index counters, %d channels, %d alus\n", versionMajor, versionMinor,
    dma->virtualAddress, dma->capabilities.indexCounters, dma->capabilities.dmaChannels, dma->capabilities.alus);
  printk(KERN_INFO "  %d param bits, %d code bits, %d microcode words%s, %s status FIFO\n",
         dma->capabilities.parameterAddressBits,
         dma->capabilities.codeAddressBits, 
         dma->capabilities.microcodeWords,
         ((microcodeWords < 0) ? " (INFERRED)" : ""),
         ((dma->capabilities.hasStatusFifo == DMA_HAS_STATUS_FIFO) ? "has" : "no"));

  /* Make a note if the instance has a status FIFO but no IRQ was supplied */
  if(dma->irq != DMA_NO_IRQ_SUPPLIED) {
    printk(KERN_INFO "  IRQ %d\n\n", dma->irq);
  } else if(dma->capabilities.hasStatusFifo == DMA_HAS_STATUS_FIFO) {
    printk(KERN_WARNING "  Status FIFO services are unavailable due to lack of an IRQ\n\n");
  } else printk("\n");

  return(returnValue);
}

/* Export the probe function for encapsulating drivers to use */
EXPORT_SYMBOL(labx_dma_probe);

/* Waits for a synchronized write to commit, using either polling or
 * an interrupt-driven waitqueue.
 */
static int32_t await_synced_write(struct labx_dma *dma) {
  int32_t returnValue = 0;

  /* Determine whether to use an interrupt or polling. */
  if(dma->irq != DMA_NO_IRQ_SUPPLIED) {
    int32_t waitResult;

    /* Place ourselves onto a wait queue if the synced write is flagged as
     * pending by the hardware, as this indicates that the microengine is active,
     * and we need to wait for it to finish a pass through all the microcode 
     * before the hardware will commit the write to its destination (a register
     * or microcode RAM.)  If the engine is inactive or the write already 
     * committed, we will not actually enter the wait queue.
     */
    waitResult =
      wait_event_interruptible_timeout(dma->syncedWriteQueue,
                                       ((XIo_In32(DMA_REGISTER_ADDRESS(dma, DMA_SYNC_REG)) & DMA_SYNC_PENDING) == 0),
                                       msecs_to_jiffies(SYNCED_WRITE_TIMEOUT_MSECS));

    /* If the wait returns zero, then the timeout elapsed; if negative, a signal
     * interrupted the wait.
     */
    if(waitResult == 0) returnValue = -ETIMEDOUT;
    else if(waitResult < 0) returnValue = -EAGAIN;
  } else {
    /* No interrupt was supplied during the device probe, simply poll for the bit. */
    while(XIo_In32(DMA_REGISTER_ADDRESS(dma, DMA_SYNC_REG)) & DMA_SYNC_PENDING);
  }

  /* Return success or "timed out" */
  return(returnValue);
}

/* Loads the passed microcode descriptor into the instance */
static int32_t load_descriptor(struct labx_dma *dma, ConfigWords *descriptor) {
  uint32_t wordIndex;
  uintptr_t wordAddress;
  uint32_t lastIndex;
  int32_t returnValue = 0;

  if (dma->regionShift == 0)
  {
    printk(KERN_ERR "Trying to load a descriptor on invalid DMA hardware (vaddr = %p)\n", dma->virtualAddress);
    return(-1);
  }

  /* Handle the last write specially for interlocks */
  lastIndex = descriptor->numWords;
  if(descriptor->interlockedLoad) lastIndex--;

  wordAddress = (DMA_MICROCODE_BASE(dma) + (descriptor->offset * sizeof(uint32_t)));
  /*  printk(KERN_INFO "DMA (%p) Descriptor load at %p (%p + %08X)\n", dma, (void*)wordAddress, dma->virtualAddress, descriptor->offset); */
  for(wordIndex = 0; wordIndex < lastIndex; wordIndex++) {
    XIo_Out32(wordAddress, descriptor->configWords[wordIndex]);
    wordAddress += sizeof(uint32_t);
  }

  /* If an interlocked load is requested, issue a sync command on the last write */
  if(descriptor->interlockedLoad) {
    /* Request a synchronized write for the last word and wait for it */
    XIo_Out32(DMA_REGISTER_ADDRESS(dma, DMA_SYNC_REG), DMA_SYNC_NEXT_WRITE);
    XIo_Out32(wordAddress, descriptor->configWords[wordIndex]);
    returnValue = await_synced_write(dma);
  }

  return(returnValue);
}

/* Buffer for storing configuration words */
static uint32_t configWords[MAX_CONFIG_WORDS];

static int alloc_buffers(struct labx_dma* dma, DMAAlloc* alloc)
{
  // Reuse configWords space to hold buffer pointers
  void** pointers = (void**)configWords;
  int i,j;

  for (i=0; i<alloc->nBufs; i++)
  {
    if (alloc->size < 4096)
    {
      // Force alignment
      pointers[i] = (void*)(((uintptr_t)(kmalloc(alloc->size*2, GFP_DMA) + alloc->size-1)) & ~(alloc->size-1));
    }
    else
    {
      // Page size chunks or greater will always be aligned
      pointers[i] = kmalloc(alloc->size, GFP_DMA);
    }

    if (NULL == pointers[i])
    {
      for (j=0; j<i; j++)
      {
        kfree(pointers[i]);
      }
      return -ENOMEM;
    }
  }

  return 0;
}

static int free_buffers(struct labx_dma* dma, DMAAlloc* alloc)
{
  // Reuse configWords space to hold buffer pointers
  void** pointers = (void**)configWords;
  int i;

  for (i=0; i<alloc->nBufs; i++)
  {
    kfree(pointers[i]);
  }

  return 0;
}

static void copy_descriptor(struct labx_dma *dma,
                            ConfigWords *descriptor) {
  uint32_t wordIndex;
  uintptr_t wordAddress;

  wordAddress = (DMA_MICROCODE_BASE(dma) + (descriptor->offset * sizeof(uint32_t)));
  for(wordIndex = 0; wordIndex < descriptor->numWords; wordIndex++) {
    descriptor->configWords[wordIndex] = XIo_In32(wordAddress);
    wordAddress += sizeof(uint32_t);
  }
}

/* I/O control operations for the driver */
int labx_dma_ioctl(struct labx_dma* dma, unsigned int command, unsigned long arg)
{
  int returnValue = 0;

  /* Switch on the request */
  switch(command) {
  case DMA_IOC_START_ENGINE:
    XIo_Out32(DMA_REGISTER_ADDRESS(dma, DMA_CONTROL_REG), DMA_ENABLE);
    break;

  case DMA_IOC_STOP_ENGINE:
    XIo_Out32(DMA_REGISTER_ADDRESS(dma, DMA_CONTROL_REG), DMA_DISABLE);
    break;

  case DMA_IOC_LOAD_DESCRIPTOR:
    {
      ConfigWords descriptor;

      if(copy_from_user(&descriptor, (void __user*)arg, sizeof(descriptor)) != 0) {
        return(-EFAULT);
      }
      if(copy_from_user(configWords, (void __user*)descriptor.configWords, 
                        (descriptor.numWords * sizeof(uint32_t))) != 0) {
        return(-EFAULT);
      }
      descriptor.configWords = configWords;
      returnValue = load_descriptor(dma, &descriptor);
    }
    break;

  case DMA_IOC_COPY_DESCRIPTOR:
    {
      ConfigWords userDescriptor;
      ConfigWords localDescriptor;

      /* Copy into our local descriptor, then obtain buffer pointer from userland */
      if(copy_from_user(&userDescriptor, (void __user*)arg, sizeof(userDescriptor)) != 0) {
        return(-EFAULT);
      }
      localDescriptor.offset = userDescriptor.offset;
      localDescriptor.numWords = userDescriptor.numWords;
      localDescriptor.configWords = configWords;
      copy_descriptor(dma, &localDescriptor);
      if(copy_to_user((void __user*)userDescriptor.configWords, configWords, 
                      (userDescriptor.numWords * sizeof(uint32_t))) != 0) {
        return(-EFAULT);
      }
    }
    break;

  case DMA_IOC_START_CHANNEL:
    /* printk(KERN_INFO "DMA (%p) Start Channel %08X (%p)\n", dma, (int)arg, (void*)DMA_REGISTER_ADDRESS(dma, DMA_CHANNEL_ENABLE_REG)); */
    XIo_Out32(DMA_REGISTER_ADDRESS(dma, DMA_CHANNEL_ENABLE_REG), 
      XIo_In32(DMA_REGISTER_ADDRESS(dma, DMA_CHANNEL_ENABLE_REG)) | (1<<arg));
    break;

  case DMA_IOC_STOP_CHANNEL:
    /* printk(KERN_INFO "DMA (%p) Stop Channel %08X (%p)\n", dma, (int)arg, (void*)DMA_REGISTER_ADDRESS(dma, DMA_CHANNEL_ENABLE_REG)); */
    XIo_Out32(DMA_REGISTER_ADDRESS(dma, DMA_CHANNEL_ENABLE_REG), 
      XIo_In32(DMA_REGISTER_ADDRESS(dma, DMA_CHANNEL_ENABLE_REG)) & ~(1<<arg));
    break;

  case DMA_IOC_ALLOC_BUFFERS:
    {
      DMAAlloc alloc;

      if(copy_from_user(&alloc, (void __user*)arg, sizeof(alloc)) != 0) {
        return(-EFAULT);
      }

      if (alloc_buffers(dma, &alloc) < 0) return -ENOMEM;

      if(copy_to_user((void __user*)alloc.buffers, configWords,
                        (alloc.nBufs * sizeof(void*))) != 0) {
        return(-EFAULT);
      }
    }
    break;

  case DMA_IOC_FREE_BUFFERS:
    {
      DMAAlloc alloc;

      if(copy_from_user(&alloc, (void __user*)arg, sizeof(alloc)) != 0) {
        return(-EFAULT);
      }

      if(copy_from_user(&alloc, (void __user*)alloc.buffers, alloc.nBufs * sizeof(void*)) != 0) {
        return(-EFAULT);
      }

      if (free_buffers(dma, &alloc) < 0) return -EFAULT;
    }
    break;

  case DMA_IOC_SET_VECTOR:
    {
      DMAVector vector;
      if(copy_from_user(&vector, (void __user*)arg, sizeof(vector)) != 0) {
        return(-EFAULT);
      }

      XIo_Out32(DMA_REGISTER_ADDRESS(dma, DMA_VECTORS_BASE_ADDRESS + vector.channel), vector.address);
    }
    break;

  case DMA_IOC_GET_CAPS:
    {
      if(copy_to_user((void __user*)arg, &dma->capabilities, sizeof(DMACapabilities)) != 0) {
        return(-EFAULT);
      }
    }
    break;

  default:
    return(-EINVAL);
  }

  return(returnValue);
}
EXPORT_SYMBOL(labx_dma_ioctl);

MODULE_AUTHOR("Chris Wulff");
MODULE_LICENSE("GPL");
