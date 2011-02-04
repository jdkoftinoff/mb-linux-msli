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
#include <linux/labx_dma_coprocessor_defs.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/sched.h>

#define DRIVER_VERSION_MIN  0x10
#define DRIVER_VERSION_MAX  0x11
#define CAPS_INDEX_VERSION  0x11 /* First version with # index counters in CAPS word */

/* Number of milliseconds we will wait before bailing out of a synced write */
#define SYNCED_WRITE_TIMEOUT_MSECS  (100)

void labx_dma_probe(struct labx_dma *dma)
{
  uint32_t capsWord;
  uint32_t versionWord;
  uint32_t versionMajor;
  uint32_t versionMinor;
  uint32_t versionCompare;
  

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
    return;
  }

  /* Decode the various bits in the capabilities word */
  if (versionCompare < CAPS_INDEX_VERSION) {
    dma->capabilities.indexCounters = 4;
  }
  else {
    dma->capabilities.indexCounters = (capsWord >> DMA_CAPS_INDEX_SHIFT) & DMA_CAPS_INDEX_MASK;
  }
  dma->capabilities.alus = (capsWord >> DMA_CAPS_ALU_SHIFT) & DMA_CAPS_ALU_MASK;
  dma->capabilities.dmaChannels = (capsWord >> DMA_CAPS_CHANNELS_SHIFT) & DMA_CAPS_CHANNELS_MASK;
  dma->capabilities.parameterAddressBits = (capsWord >> DMA_CAPS_PARAM_ADDRESS_BITS_SHIFT) & DMA_CAPS_PARAM_ADDRESS_BITS_MASK;
  dma->capabilities.codeAddressBits = (capsWord >> DMA_CAPS_CODE_ADDRESS_BITS_SHIFT) & DMA_CAPS_CODE_ADDRESS_BITS_MASK;

  dma->regionShift = (dma->capabilities.codeAddressBits + 2);

  /* Initialize the waitqueue used for synchronized writes */
  init_waitqueue_head(&(dma->syncedWriteQueue));

  /* TEMPORARY - Don't indicate an IRQ assignment just yet; this needs to be
   *             added as a parameter to this function.
   *             Make sure we disable the IRQ, request, then re-enable (is this
   *             the appropriate place to enable?)
   */
  dma->irq = NO_IRQ_SUPPLIED;

  printk(KERN_INFO "Found DMA unit %d.%d at %p: %d index counters, %d channels, %d alus\n", versionMajor, versionMinor,
    dma->virtualAddress, dma->capabilities.indexCounters, dma->capabilities.dmaChannels, dma->capabilities.alus);
  printk(KERN_INFO "   %d param bits, %d code bits, %d shift\n",
    dma->capabilities.parameterAddressBits, dma->capabilities.codeAddressBits, dma->regionShift);
}

/* Waits for a synchronized write to commit, using either polling or
 * an interrupt-driven waitqueue.
 */
static int32_t await_synced_write(struct labx_dma *dma) {
  int32_t returnValue = 0;
  uint32_t tempValue;

  /* Determine whether to use an interrupt or polling */
  if(dma->irq != NO_IRQ_SUPPLIED) {
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
static int32_t load_descriptor(struct labx_dma *dma,
                               ConfigWords *descriptor) {
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
EXPORT_SYMBOL(labx_dma_probe);

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
      load_descriptor(dma, &descriptor);
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
