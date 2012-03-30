/*
 *  linux/drivers/net/labx_avb/labx_audio_packetizer.h
 *
 *  Lab X Technologies AVB flexible audio packetizer driver
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
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

#ifndef _LABX_AUDIO_PACKETIZER_H_
#define _LABX_AUDIO_PACKETIZER_H_

#include <linux/cdev.h>
#include <linux/highmem.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <net/labx_avb/packet_engine_defs.h>
#ifdef CONFIG_LABX_AUDIO_DEPACKETIZER_DMA
#include <linux/labx_dma.h>
#endif

/* Macros for determining sub-addresses for address ranges and individual registers.
 * These are affected by the amount of address space devoted to microcode storage, 
 * which is hardware-configurable.
 */
#define REGISTER_RANGE      0x0
#define CLOCK_DOMAIN_RANGE  0x1
#define MICROCODE_RANGE     0x2

/* Global control registers */
#define CONTROL_STATUS_REG   (0x000)
#  define ID_LOAD_ACTIVE        0x100
#  define ID_LOAD_LAST_WORD     0x200
#  define RTC_STABLE            0x000
#  define RTC_UNSTABLE          0x002
#  define DEPACKETIZER_DISABLE  0x000
#  define DEPACKETIZER_ENABLE   0x001

#define VECTOR_BAR_REG       (0x001)

#define ID_SELECT_0_REG      (0x002)
#define ID_SELECT_1_REG      (0x003)
#define ID_SELECT_2_REG      (0x004)
#define ID_SELECT_3_REG      (0x005)
#  define ID_SELECT_NONE  0x00000000
#  define ID_SELECT_ALL   0xFFFFFFFF

#define ID_CONFIG_DATA_REG   (0x006)

#define IRQ_MASK_REG         (0x008)
#define IRQ_FLAGS_REG        (0x009)
#  define NO_IRQS       (0x00000000)
#  define SYNC_IRQ      (0x00000001)
#  define STREAM_IRQ    (0x00000002)
#  define SEQ_ERROR_IRQ (0x00000004)

#define SYNC_REG             (0x00A)
#  define CANCEL_SYNC      (0x00000000)
#  define SYNC_NEXT_WRITE  (0x00000001)
#  define SYNC_PENDING     (0x80000000)

#define RELOCATE_REG         (0x00B)
#  define RELOCATION_INACTIVE       (0x00000000)
#  define RELOCATION_ACTIVE         (0x80000000)
#  define RELOCATION_MATCH_MASK     (0x0000007F)
#  define RELOCATION_ADDRESS_SHIFT  (7)

#define STREAM_STATUS_0_REG  (0x00C)
#define STREAM_STATUS_1_REG  (0x00D)
#define STREAM_STATUS_2_REG  (0x00E)
#define STREAM_STATUS_3_REG  (0x00F)

#define CAPABILITIES_REG_A   (0x0FD)
#  define MAX_STREAM_SLOTS_MASK  (0x7F)

#define CAPABILITIES_REG_B   (0x0FE)
#  define MATCH_ARCH_SHIFT          24
#  define MATCH_ARCH_MASK           0x0FF
#  define MAX_STREAMS_SHIFT         16
#  define MAX_STREAMS_MASK          0x0FF
#  define CLOCK_DOMAINS_SHIFT       8
#  define CLOCK_DOMAINS_MASK        0x0FF
#  define PARAM_ADDRESS_BITS_SHIFT  4
#  define PARAM_ADDRESS_BITS_MASK   0x0F
#  define CODE_ADDRESS_BITS_MASK    0x0F

#define REVISION_REG         (0x0FF)
#  define REVISION_FIELD_BITS  4
#  define REVISION_FIELD_MASK  0x0F

/* Per-clock-domain registers
 * Some of the recovery fields are based upon the maximum number of
 * streams supported by the instance, so macros are provided for them.
 */
#define REGS_PER_CLOCK_DOMAIN  16
#define RECOVERY_INDEX_REG  0x000
#  define STREAM_INDEX_MASK(device)   (device->streamIndexMask)
#  define RECOVERY_DISABLED           (0x00000000)
#  define RECOVERY_ENABLED(device)    (device->streamIndexMask + 1)
#  define PHASE_NUDGE_ENABLED(device) (RECOVERY_ENABLED(device) << 1)

#define MC_SYT_INTERVAL_REG  0x001
#define MC_CONTROL_REG       0x002
#  define MC_CONTROL_SYNC_EXTERNAL       0x00000008
#  define MC_CONTROL_SYNC_INTERNAL       0x00000000
#  define MC_CONTROL_SAMPLE_EDGE_RISING  0x00000001
#  define MC_CONTROL_SAMPLE_EDGE_FALLING 0x00000000

#define MC_HALF_PERIOD_REG   0x003
#define MC_REMAINDER_REG     0x004
#define MC_RTC_INCREMENT_REG 0x005

#define DAC_OFFSET_REG       0x008
#  define DAC_OFFSET_ZERO  0x00000000

#define DAC_P_COEFF_REG      0x009
/* DAC control coefficients are signed, so the upper bit is the sign bit */
#  define DAC_COEFF_MANTISSA_BITS  5
#  define DAC_COEFF_FRACTION_BITS  12
#  define DAC_COEFF_BITS           (DAC_COEFF_MANTISSA_BITS + DAC_COEFF_FRACTION_BITS + 1)
#  define DAC_COEFF_MIN            0x00020000
#  define DAC_COEFF_ZERO           0x00000000
#  define DAC_COEFF_MAX            0x0001FFFF
#  define DAC_COEFF(floatCoeff)  ((uint32_t)(floatCoeff * (float)(0x01 << DAC_COEFF_FRACTION_BITS)))

#define LOCK_COUNT_REG       0x00A
#  define VCO_UNLOCK_COUNT_MASK   0x0FFF
#  define VCO_UNLOCK_COUNT_SHIFT  12
#  define VCO_LOCK_COUNT_MASK     0x0FFF
#  define VCO_LOCK_COUNT_SHIFT    0

#define REGISTER_ADDRESS(device, offset) \
  ((uintptr_t)device->virtualAddress |                       \
   (REGISTER_RANGE << device->regionShift) | (offset << 2))

#define CLOCK_DOMAIN_REGISTER_ADDRESS(device, clockDomain, offset) \
  ((uintptr_t)device->virtualAddress |                              \
   (CLOCK_DOMAIN_RANGE << device->regionShift) |                    \
   (((clockDomain * REGS_PER_CLOCK_DOMAIN) + offset) << 2))

#define MICROCODE_RAM_BASE(device)              \
  ((uintptr_t)device->virtualAddress |           \
   (MICROCODE_RANGE << device->regionShift))

/* Maximum number of streams the depacketizer can architecturally handle */
#define MAX_CONCURRENT_STREAMS  128

/* 
 * Enumerated type used to identify the possible hardware schemes used for matching
 * stream IDs
 */
typedef enum {
  STREAM_MATCH_SRLC16E = 0x10,
  STREAM_MATCH_SRLC32E = 0x11,
  STREAM_MATCH_UNIFIED = 0xFF
} StreamMatchArchitecture;

/* Number of units and configuration words required for each architecture we support */
#define NUM_SRL16E_INSTANCES      16
#  define NUM_SRL16E_CONFIG_WORDS   (NUM_SRL16E_INSTANCES / 2)
#define NUM_SRLC16E_INSTANCES     16
#  define NUM_SRLC16E_CONFIG_WORDS  (NUM_SRLC16E_INSTANCES / 2)
#define NUM_SRLC32E_INSTANCES     13
#  define NUM_SRLC32E_CONFIG_WORDS  NUM_SRLC32E_INSTANCES
#define SRLCXXE_CLEARING_WORD     0x00000000

/* Constants related to match vectors */
#define MATCH_VECTORS_PER_WORD   2
#define MATCH_VECTOR_EVEN_SHIFT  0
#define MATCH_VECTOR_ODD_SHIFT   16
#define MATCH_VECTOR_MASK        0x0FFFF

#define INSTANCE_NO_DMA  0
#define INSTANCE_HAS_DMA 1

/* Driver structure to maintain state for each device instance */
#define NAME_MAX_SIZE  256
#define NO_IRQ_SUPPLIED   (-1)
struct depacketizer_presentation_channels {
  uint32_t channelBufferSizeBytes;
  struct audio_depacketizer *redundancyPartner; // Currently unused
  void *channelBuffers[1];
};

struct audio_depacketizer {
  /* Pointer back to the platform device */
  struct platform_device *pdev;

  /* Character device data */
  struct cdev cdev;
  dev_t       deviceNumber;
  uint32_t    instanceNumber;

  /* Name for use in identification */
  char name[NAME_MAX_SIZE];

  /* Physical and virtual base address */
  uintptr_t      physicalAddress;
  uintptr_t      addressRangeSize;
  void __iomem  *virtualAddress;

  /* Interrupt request number */
  int32_t irq;

  /* Bit shift for the address sub-range, related to max instructions */
  uint32_t regionShift;

  /* Architecture employed for stream matching */
  StreamMatchArchitecture matchArchitecture;

  /* Stream index mask appropriate for the instance */
  uint32_t streamIndexMask;

  /* Capabilities of the depacketizer hardware */
  DepacketizerCaps capabilities;

  /* DMA instance (if supported) */
#ifdef CONFIG_LABX_AUDIO_DEPACKETIZER_DMA
  uint32_t hasDma;
  struct labx_dma dma;
#endif

  /* Wait queue for putting threads to sleep */
  wait_queue_head_t syncedWriteQueue;

  /* Mutex for the device instance */
  spinlock_t mutex;
  bool opened;

  /* Netlink events */
  wait_queue_head_t streamStatusQueue;
  uint32_t streamStatusGeneration;
  uint32_t streamSeqError;
  uint32_t netlinkSequence;
  struct task_struct *netlinkTask;
  struct depacketizer_presentation_channels *presentationChannels[MAX_CONCURRENT_STREAMS];
};

/* From labx_audio_depacketizer_netlink.c */
extern int register_audio_depacketizer_netlink(void);
extern void unregister_audio_depacketizer_netlink(void);
extern int audio_depacketizer_stream_event(struct audio_depacketizer *depacketizer);

#endif
