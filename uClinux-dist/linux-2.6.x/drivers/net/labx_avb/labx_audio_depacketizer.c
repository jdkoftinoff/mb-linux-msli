/*
 *  linux/drivers/net/labx_avb/labx_audio_depacketizer.c
 *
 *  Lab X Technologies AVB flexible audio de-packetizer driver
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

#include "labx_audio_depacketizer.h"
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/version.h>
#include <xio.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF


/* Driver name and the revision of hardware expected (1.1 - 1.7) */
#define DRIVER_NAME "labx_audio_depacketizer"
#define DRIVER_VERSION_MIN  0x11
#define DRIVER_VERSION_MAX  0x18

/* "Breakpoint" revision numbers for certain features */
#define UNIFIED_MATCH_VERSION_MIN  0x12
#define EXTENDED_CAPS_VERSION_MIN  0x13

/* Instances before the extended capabilities version typically had
 * 32 stream slots maximum
 */
#define ASSUMED_MAX_STREAM_SLOTS  32

/* Major device number for the driver */
#define DRIVER_MAJOR 232

/* Interface type string prefix which indicates a Dma_Coprocessor is
 * in use for the instance.  This could be one of the following:
 * DMA_RAW, DMA_NPI, DMA_PLB, or DMA_AXI
 */
#define DMA_INTERFACE_PREFIX  "DMA_"

/* Maximum number of depacketizers and instance count */
#define MAX_INSTANCES 64
static uint32_t instanceCount;

/* Number of milliseconds we will wait before bailing out of a synced write */
#define SYNCED_WRITE_TIMEOUT_MSECS  (100)

#if 0
#define DBG(f, x...) printk(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

/* Number of milliseconds to wait before permitting consecutive events from
 * being propagated up to userspace
 */
#define EVENT_THROTTLE_MSECS (250)

#ifndef DMA_CALLBACKS
#define DMA_CALLBACKS NULL
#else
DMA_CALLBACKS_EXTERN
#endif

/* Disables the passed instance */
static void disable_depacketizer(struct audio_depacketizer *depacketizer) {
  uint32_t ctrlStatusReg;
  DBG("Disabling the depacketizer\n");

  /* Disable the micro-engine */
  ctrlStatusReg = XIo_In32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG));
  ctrlStatusReg &= ~DEPACKETIZER_ENABLE;
  XIo_Out32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG), ctrlStatusReg);
}

/* Enables the passed instance */
static void enable_depacketizer(struct audio_depacketizer *depacketizer) {
  uint32_t ctrlStatusReg;

  DBG("Enabling the depacketizer\n");

  /* Enable the micro-engine, with the "last load" flag for match unit configuration
   * disabled (the loading methods should assert this as needed.)
   */
  ctrlStatusReg = XIo_In32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG));
  ctrlStatusReg |= DEPACKETIZER_ENABLE;
  XIo_Out32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG), ctrlStatusReg);
}

/* Flags the RTC as stable for the passed instance */
static void set_rtc_stable(struct audio_depacketizer *depacketizer) {
  uint32_t ctrlStatusReg;

  DBG("Declaring RTC stable\n");

  /* Set the "RTC unstable" bit, which will coast any recovered media clock domains */
  ctrlStatusReg = XIo_In32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG));
  ctrlStatusReg &= ~RTC_UNSTABLE;
  XIo_Out32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG), ctrlStatusReg);
}

/* Flags the RTC as stable for the passed instance */
static void set_rtc_unstable(struct audio_depacketizer *depacketizer) {
  uint32_t ctrlStatusReg;

  DBG("Declaring RTC unstable\n");

  /* Set the "RTC unstable" bit, which will coast any recovered media clock domains */
  ctrlStatusReg = XIo_In32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG));
  ctrlStatusReg |= RTC_UNSTABLE;
  XIo_Out32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG), ctrlStatusReg);
}

/* Waits for a synchronized write to commit, using either polling or
 * an interrupt-driven waitqueue.
 */
static int32_t await_synced_write(struct audio_depacketizer *depacketizer) {
  int32_t returnValue = 0;

  /* Determine whether to use an interrupt or polling */
  if(depacketizer->irq != NO_IRQ_SUPPLIED) {
    int32_t waitResult;

    /* Place ourselves onto a wait queue if the synced write is flagged as
     * pending by the hardware, as this indicates that the microengine is active,
     * and we need to wait for it to finish a pass through all the microcode 
     * before the hardware will commit the write to its destination (a register
     * or microcode RAM.)  If the engine is inactive or the write already 
     * committed, we will not actually enter the wait queue.
     */
    waitResult =
      wait_event_interruptible_timeout(depacketizer->syncedWriteQueue,
                                       ((XIo_In32(REGISTER_ADDRESS(depacketizer, SYNC_REG)) & 
                                         SYNC_PENDING) == 0),
                                       msecs_to_jiffies(SYNCED_WRITE_TIMEOUT_MSECS));

    /* If the wait returns zero, then the timeout elapsed; if negative, a signal
     * interrupted the wait.
     */
    if(waitResult == 0) returnValue = -ETIMEDOUT;
    else if(waitResult < 0) returnValue = -EAGAIN;
  } else {
    /* No interrupt was supplied during the device probe, simply poll for the bit. */
    /* TODO: Need to introduce timeout semantics to this mode as well! */
    while(XIo_In32(REGISTER_ADDRESS(depacketizer, SYNC_REG)) & SYNC_PENDING);
  }

  /* Return success or "timed out" */
  return(returnValue);
}

/* Loads the passed microcode descriptor into the instance */
static int32_t load_descriptor(struct audio_depacketizer *depacketizer,
                               ConfigWords *descriptor) {
  
  uint32_t wordIndex;
  uintptr_t wordAddress;
  uint32_t lastIndex;
  int32_t returnValue = 0;

  /* Handle the last write specially for interlocks */
  lastIndex = descriptor->numWords;
  if(descriptor->interlockedLoad) lastIndex--;

  wordAddress = (MICROCODE_RAM_BASE(depacketizer) + (descriptor->offset * sizeof(uint32_t)));
  DBG("Loading descriptor @ %08X (%d), numWords %d\n", wordAddress, descriptor->offset, descriptor->numWords);
  for(wordIndex = 0; wordIndex < lastIndex; wordIndex++) {
    XIo_Out32(wordAddress, descriptor->configWords[wordIndex]);
    wordAddress += sizeof(uint32_t);
  }

  /* If an interlocked load is requested, issue a sync command on the last write */
  if(descriptor->interlockedLoad) {
    /* Request a synchronized write for the last word and wait for it */
    XIo_Out32(REGISTER_ADDRESS(depacketizer, SYNC_REG), SYNC_NEXT_WRITE);
    XIo_Out32(wordAddress, descriptor->configWords[wordIndex]);
    returnValue = await_synced_write(depacketizer);
  }

  return(returnValue);
}

/* Reads back and copies a descriptor from the packetizer into the passed 
 * structure, using the address and size it specifies.
 */
static void copy_descriptor(struct audio_depacketizer *depacketizer,
                            ConfigWords *descriptor) {
  uint32_t wordIndex;
  uintptr_t wordAddress;

  wordAddress = (MICROCODE_RAM_BASE(depacketizer) + (descriptor->offset * sizeof(uint32_t)));
  for(wordIndex = 0; wordIndex < descriptor->numWords; wordIndex++) {
    descriptor->configWords[wordIndex] = XIo_In32(wordAddress);
    wordAddress += sizeof(uint32_t);
  }
}

/* Busy loops until the match unit configuration logic is idle.  The hardware goes 
 * idle very quickly and deterministically after a configuration word is written, 
 * so this should not consume very much time at all.
 */
static void wait_match_config(struct audio_depacketizer *depacketizer) {
  uint32_t statusWord;
  uint32_t timeout = 10000;
  do {
    statusWord = XIo_In32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG));
    if (timeout-- == 0)
    {
      printk("depacketizer: wait_match_config timeout!\n");
      break;
    }
  } while(statusWord & ID_LOAD_ACTIVE);
}

/* Selects a set of match units for subsequent configuration loads */
typedef enum { SELECT_NONE, SELECT_SINGLE, SELECT_ALL } SelectionMode;
static void select_matchers(struct audio_depacketizer *depacketizer,
                            SelectionMode selectionMode,
                            uint32_t matchUnit) {
  uint32_t selectionWord;

  switch(selectionMode) {
  case SELECT_NONE:
    /* De-select all the match units */
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_0_REG), ID_SELECT_NONE);
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_1_REG), ID_SELECT_NONE);
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_2_REG), ID_SELECT_NONE);
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_3_REG), ID_SELECT_NONE);
    break;

  case SELECT_SINGLE:
    /* Select a single unit */
    selectionWord = ((matchUnit < 32) ? (0x01 << matchUnit) : ID_SELECT_NONE);
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_0_REG), selectionWord);
    selectionWord = (((matchUnit < 64) & (matchUnit >= 32)) ? (0x01 << (matchUnit - 32)) : ID_SELECT_NONE);
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_1_REG), selectionWord);
    selectionWord = (((matchUnit < 96) & (matchUnit >= 64)) ? (0x01 << (matchUnit - 64)) : ID_SELECT_NONE);
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_2_REG), selectionWord);
    selectionWord = ((matchUnit >= 96) ? (0x01 << (matchUnit - 96)) : ID_SELECT_NONE);
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_3_REG), selectionWord);
    break;

  default:
    /* Select all match units at once */
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_0_REG), ID_SELECT_ALL);
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_1_REG), ID_SELECT_ALL);
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_2_REG), ID_SELECT_ALL);
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_3_REG), ID_SELECT_ALL);
  }
}

/* Sets the loading mode for any selected match units.  This revolves around
 * automatically disabling the match units' outputs while they're being
 * configured so they don't fire false matches, and re-enabling them as their
 * last configuration word is loaded.
 */
typedef enum { LOADING_MORE_WORDS, LOADING_LAST_WORD } LoadingMode;
static void set_matcher_loading_mode(struct audio_depacketizer *depacketizer,
                                     LoadingMode loadingMode) {
  uint32_t controlWord = XIo_In32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG));

  if(loadingMode == LOADING_MORE_WORDS) {
    /* Clear the "last word" bit to suppress false matches while the units are
     * only partially cleared out
     */
    controlWord &= ~ID_LOAD_LAST_WORD;
  } else {
    /* Loading the final word, flag the match unit(s) to enable after the
     * next configuration word is loaded.
     */
    controlWord |= ID_LOAD_LAST_WORD;
  }
  XIo_Out32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG), controlWord);
}

/* Clears any selected match units, preventing them from matching any packets */
static void clear_selected_matchers(struct audio_depacketizer *depacketizer) {
  uint32_t numClearWords;
  uint32_t wordIndex;

  /* Ensure the unit(s) disable as the first word is load to prevent erronous
   * matches as the units become partially-cleared
   */
  set_matcher_loading_mode(depacketizer, LOADING_MORE_WORDS);

  /* Load the appropriate number of clearing words to the LUTs */
  switch(depacketizer->matchArchitecture) {
  case STREAM_MATCH_SRLC16E:
  case STREAM_MATCH_UNIFIED:
    /* The unified architecture actually makes use of SRLC16Es */
    numClearWords = NUM_SRLC16E_CONFIG_WORDS;
    break;

  case STREAM_MATCH_SRLC32E:
    numClearWords = NUM_SRLC32E_CONFIG_WORDS;
    break;

  default:
    numClearWords = 0;
  }
  for(wordIndex = 0; wordIndex < numClearWords; wordIndex++) {
    /* Assert the "last word" flag on the last word required to complete the clearing
     * of the selected unit(s).
     */
    if(wordIndex == (numClearWords - 1)) {
      set_matcher_loading_mode(depacketizer, LOADING_LAST_WORD);
    }
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_CONFIG_DATA_REG), SRLCXXE_CLEARING_WORD);
  }
}

/* Clears all of the match units within the passed instance */
static void clear_all_matchers(struct audio_depacketizer *depacketizer) {
  /* Ascertain that the configuration logic is ready, then clear all in one shot */
  wait_match_config(depacketizer);
  select_matchers(depacketizer, SELECT_ALL, 0);
  clear_selected_matchers(depacketizer);
  select_matchers(depacketizer, SELECT_NONE, 0);
}

/* Updates an entry in the match vector table, using the synchronized write
 * mechanism to make sure the operation is safe even if the engine is running
 * and the match unit is in use.
 */
static int32_t update_match_vector(struct audio_depacketizer *depacketizer,
                                   uint32_t matchUnit,
                                   uint32_t matchVector) {
  uintptr_t vectorAddress;
  uint32_t vectorWord;

  /* 
   * Locate the unit's vector in the relocatable table and write it.  Match vectors
   * are packed in sixteen-bit pairs, in little-endian order, so we must maintain the
   * "neighbor" value.
   */
  vectorAddress = XIo_In32(REGISTER_ADDRESS(depacketizer, VECTOR_BAR_REG));
  vectorAddress += (matchUnit / MATCH_VECTORS_PER_WORD);
  vectorAddress = (MICROCODE_RAM_BASE(depacketizer) + (vectorAddress * sizeof(uint32_t)));
  vectorWord = XIo_In32(vectorAddress);
  if(matchUnit % MATCH_VECTORS_PER_WORD) {
    vectorWord &= (MATCH_VECTOR_MASK << MATCH_VECTOR_EVEN_SHIFT);
    vectorWord |= (matchVector << MATCH_VECTOR_ODD_SHIFT);
  } else {
    vectorWord &= (MATCH_VECTOR_MASK << MATCH_VECTOR_ODD_SHIFT);
    vectorWord |= (matchVector << MATCH_VECTOR_EVEN_SHIFT);
  }
  
  /* Now modify the vector table itself.  An interlock is used here to ensure that
   * the engine isn't just about to access the vector table as we're modifying it
   * in what is potentially a different clock domain.
   */
  XIo_Out32(REGISTER_ADDRESS(depacketizer, SYNC_REG), SYNC_NEXT_WRITE);
  XIo_Out32(vectorAddress, vectorWord);
  return(await_synced_write(depacketizer));
}

/* Loads truth tables into a selected SRLC16E-based match unit */
static void load_srlc16e_matcher(struct audio_depacketizer *depacketizer,
                                 uint64_t matchStreamId) {
  int32_t lutIndex;
  uint32_t configWord = 0x00000000;
  uint32_t matchChunk;
  bool loadLutPair = false;
        
  for(lutIndex = (NUM_SRLC16E_INSTANCES - 1); lutIndex >= 0; lutIndex--) {
    configWord <<= 16;
    matchChunk = ((matchStreamId >> (lutIndex * 4)) & 0x0F);
    configWord |= (0x01 << matchChunk);
    if(loadLutPair) {
      /* Assert the "load last word" flag on the last word we're going to load
       * for the configuration; this will re-enable the match unit as soon as
       * the last bit of the word has been shifted into it.
       */
      if(lutIndex == 0) {
        set_matcher_loading_mode(depacketizer, LOADING_LAST_WORD);
      }
      XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_CONFIG_DATA_REG), configWord);
      wait_match_config(depacketizer);
    }
    loadLutPair = !loadLutPair;
  }
}

/* Loads truth tables into a selected SRLC32E-based match unit */
static void load_srlc32e_matcher(struct audio_depacketizer *depacketizer,
                                 uint64_t matchStreamId) {
  int32_t lutIndex;
  uint32_t configWord;
  uint32_t matchChunk;
  
  for(lutIndex = (NUM_SRLC32E_INSTANCES - 1); lutIndex >= 0; lutIndex--) {
    matchChunk = ((matchStreamId >> (lutIndex * 5)) & 0x01F);
    configWord = (0x01 << matchChunk);

    /* Assert the "load last word" flag as per the comment in SRLC16E above */
    if(lutIndex == 0) set_matcher_loading_mode(depacketizer, LOADING_LAST_WORD);
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_CONFIG_DATA_REG), configWord);
    wait_match_config(depacketizer);
  }
}

/* Loads truth tables into a match unit using the newest, "unified" match
 * architecture.  This is SRL16E based (not cascaded) due to the efficient
 * packing of these primitives into Xilinx LUT6-based architectures.
 */
static void load_unified_matcher(struct audio_depacketizer *depacketizer,
                                 uint64_t matchStreamId) {
  int32_t wordIndex;
  int32_t lutIndex;
  uint32_t configWord = 0x00000000;
  uint32_t matchChunk;
  
  /* In this architecture, all of the SRL16Es are loaded in parallel, with each
   * configuration word supplying two bits to each.  Only one of the two bits can
   * ever be set, so there is just an explicit check for one.
   */
  for(wordIndex = (NUM_SRL16E_CONFIG_WORDS - 1); wordIndex >= 0; wordIndex--) {
    for(lutIndex = (NUM_SRL16E_INSTANCES - 1); lutIndex >= 0; lutIndex--) {
      matchChunk = ((matchStreamId >> (lutIndex << 2)) & 0x0F);
      configWord <<= 2;
      if(matchChunk == (wordIndex << 1)) configWord |= 0x01;
      if(matchChunk == ((wordIndex << 1) + 1)) configWord |= 0x02;
    }

    /* Two bits of truth table have been determined for each SRL16E, load the
     * word and wait for the configuration to occur.  Be sure to flag the last
     * word to automatically re-enable the match unit(s) as the last word completes.
     */
      if(wordIndex == 0) set_matcher_loading_mode(depacketizer, LOADING_LAST_WORD);
      XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_CONFIG_DATA_REG), configWord);
      wait_match_config(depacketizer);
  }
}

/* Configures one of the passed instance's match units */
static int32_t configure_matcher(struct audio_depacketizer *depacketizer,
                                 MatcherConfig *matcherConfig) {
  int32_t returnValue = 0;

  /* Sanity-check the input structure */
  if(matcherConfig->matchUnit >= MAX_CONCURRENT_STREAMS) return(-EINVAL);

  /* If we are activating the match unit, we must first ensure that the match unit
   * is disabled, then update the matcher's entry in the vector table
   */
  switch(matcherConfig->configAction) {
  case MATCHER_ENABLE:
    /* Ascertain that the configuration logic is ready, then select the matcher */
    wait_match_config(depacketizer);
    select_matchers(depacketizer, SELECT_SINGLE, matcherConfig->matchUnit);

    /* First clear the match unit to disable any old configuration, then update the vector
     * table to point to the descriptor for the stream and configure the new match ID
     */
    clear_selected_matchers(depacketizer);
    returnValue = update_match_vector(depacketizer, matcherConfig->matchUnit, 
                                      matcherConfig->matchVector);

    /* Don't activate the match unit if there was a problem with the vector load */
    if(returnValue == 0) {
      /* Set the loading mode to disable as we load the first word */
      set_matcher_loading_mode(depacketizer, LOADING_MORE_WORDS);
      
      /* Calculate matching truth tables for the LUTs and load them */
      switch(depacketizer->matchArchitecture) {
      case STREAM_MATCH_SRLC16E:
        load_srlc16e_matcher(depacketizer, matcherConfig->matchStreamId);
        break;
      
      case STREAM_MATCH_SRLC32E:
        load_srlc32e_matcher(depacketizer, matcherConfig->matchStreamId);
        break;

      case STREAM_MATCH_UNIFIED:
        load_unified_matcher(depacketizer, matcherConfig->matchStreamId);
        break;
      
      default:
        /* Unrecognized architecture, do nothing */
        ;
      } /* switch(match architecture) */
    } /* if(vector relocation okay) */

    /* De-select the match unit */
    select_matchers(depacketizer, SELECT_NONE, 0);
    break;

  case MATCHER_DISABLE:
    /* Deactivate the match unit.  Ascertain that the configuration logic is ready, 
     * then select the matcher and clear it
     */
    wait_match_config(depacketizer);
    select_matchers(depacketizer, SELECT_SINGLE, matcherConfig->matchUnit);
    clear_selected_matchers(depacketizer);
    select_matchers(depacketizer, SELECT_NONE, 0);
    break;

  default:
    /* Bad parameter */
    returnValue = -EINVAL;
  }

  return(returnValue);
}

/* Relocates a match unit to point to a new descriptor.  This is done safely
 * and dynamically to permit shuffling of microcode around while the engine
 * is active and receiving AVBTP packets.
 */
static int32_t relocate_matcher(struct audio_depacketizer *depacketizer,
                                MatcherRelocation *matcherRelocation) {
  uint32_t relocationWord;
  int32_t returnValue = 0;

  /* Sanity-check the input structure */
  if(matcherRelocation->matchUnit >= MAX_CONCURRENT_STREAMS) return(-EINVAL);

  /* The match unit is being dynamically relocated to point to a new descriptor.
   * Some state information such as the audio cache ring offset cannot be captured,
   * copied to a new location in microcode RAM, and pointed to atomically.  The
   * engine may receive a packet on the stream while the host processor is copying
   * the descriptor, and when pointed to the new location, the stale state information
   * would be used again, dropping samples.
   *
   * To avoid this, we use the gateware's "relocation mode" facility; the hardware
   * is requested to host the state for the single stream into dedicated registers.
   */

  /* Place the match unit into relocation mode, having the engine grab the ring
   * buffer offset at the location where it presently exists.  This is performed
   * with a synchronized write to the relocation register to ensure the state
   * capture occurs when the engine is idle.
   */
  relocationWord  = (matcherRelocation->matchUnit & RELOCATION_MATCH_MASK(depacketizer));
  relocationWord |= ((matcherRelocation->oldVector + matcherRelocation->ringStateOffset) << 
                     RELOCATION_ADDRESS_SHIFT(depacketizer));
  relocationWord |= RELOCATION_ACTIVE;
  XIo_Out32(REGISTER_ADDRESS(depacketizer, SYNC_REG), SYNC_NEXT_WRITE);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, RELOCATE_REG), relocationWord);
  returnValue = await_synced_write(depacketizer);
  if(returnValue) goto relocate_fail;

  /* Now that the stream is in relocation mode, we can safely reconfigure the vector
   * table to jump to the new location, even if the code the application copied there
   * has a now-stale ring buffer offset stored in it - the engine won't use it.
   */
  returnValue = update_match_vector(depacketizer, matcherRelocation->matchUnit, 
                                    matcherRelocation->newVector);
  if(returnValue) goto relocate_fail;

  /* The match unit is now vectoring to the new location, but the stream is still
   * in relocation mode using a dedicated register.  Take it out of relocation mode,
   * causing the engine to commit the present value of the ring offset counter into
   * its new home, where it will be used from now on.
   */
  relocationWord  = (matcherRelocation->matchUnit & RELOCATION_MATCH_MASK(depacketizer));
  relocationWord |= ((matcherRelocation->newVector + matcherRelocation->ringStateOffset) << 
                     RELOCATION_ADDRESS_SHIFT(depacketizer));
  XIo_Out32(REGISTER_ADDRESS(depacketizer, SYNC_REG), SYNC_NEXT_WRITE);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, RELOCATE_REG), relocationWord);
  returnValue = await_synced_write(depacketizer);

 relocate_fail:
    return(returnValue);
}

/* Sets the base address of the match vector table; all subsequent calls to configure
 * match units will make use of this value.  All of the match units are cleared first 
 * as a side-effect of this call, since it is unsafe to move the vector table with enabled
 * match units.
 */
static void set_vector_base_address(struct audio_depacketizer *depacketizer,
                                    uint32_t vectorBaseAddress) {
  /* Clear the matchers, the set the BAR */
  clear_all_matchers(depacketizer);
  XIo_Out32(REGISTER_ADDRESS(depacketizer, VECTOR_BAR_REG), vectorBaseAddress);
}

/* Configures a clock recovery domain, including whether it is enabled */
static void configure_clock_recovery(struct audio_depacketizer *depacketizer, 
                                     ClockRecoverySettings *clockRecoverySettings) {
  ClockDomainSettings *clockDomainSettings;
  uint32_t clockDomain;
  uint32_t recoveryIndex;
  uint32_t controlValue = 0;
  uint32_t sampleRate = SINGLE_SAMPLE_RATE;

  /* Configure the timestamp interval for the domain first.  This informs the basic
   * reference clock recovery hardware of how many samples are being averaged each
   * time it receives a valid timestamp from the depacketizer.
   */
  clockDomainSettings = &clockRecoverySettings->clockDomainSettings;
  clockDomain = clockDomainSettings->clockDomain;
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, MC_SYT_INTERVAL_REG),
            clockDomainSettings->sytInterval);

  /* Configure the generated clock edge for the clock domain that corresponds to a
   * sample. This should match the gateware that is providing the "current" time
   * reference to the depacketizer.
   */
  controlValue |= (clockDomainSettings->sampleEdge == DOMAIN_SAMPLE_EDGE_RISING) ?
                   MC_CONTROL_SAMPLE_EDGE_RISING : MC_CONTROL_SAMPLE_EDGE_FALLING;
  controlValue |= (clockDomainSettings->enabled == DOMAIN_SYNC) ?
                   MC_CONTROL_SYNC_EXTERNAL : MC_CONTROL_SYNC_INTERNAL;
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, MC_CONTROL_REG),
            controlValue);

  /* Set the sample rate for the clock domain */
  switch(clockDomainSettings->sampleRate) {
  case ENGINE_SAMPLE_RATE_32_KHZ:
  case ENGINE_SAMPLE_RATE_44_1_KHZ:
  case ENGINE_SAMPLE_RATE_48_KHZ:
    sampleRate = SINGLE_SAMPLE_RATE;
    break;

  case ENGINE_SAMPLE_RATE_88_2_KHZ:
  case ENGINE_SAMPLE_RATE_96_KHZ:
    sampleRate = DOUBLE_SAMPLE_RATE;
    break;
  
  case ENGINE_SAMPLE_RATE_176_4_KHZ:
  case ENGINE_SAMPLE_RATE_192_KHZ:
    sampleRate = QUAD_SAMPLE_RATE;
    break;

  default:
    ;
  }
  XIo_Out32(REGISTER_ADDRESS(depacketizer, SAMPLE_RATE_REG), sampleRate);

  /* Configure the clock domain with which match unit it gets its temporal 
   * information from.  The match units, in turn, link a stream index to its AVBTP
   * stream ID, so a descriptor must be configured to receive a stream on this
   * index.
   */
  recoveryIndex = (clockRecoverySettings->matchUnit & STREAM_INDEX_MASK(depacketizer));
  if(clockDomainSettings->enabled == DOMAIN_ENABLED) {
    /* Enable both the recovery as well as the phase "nudge" logic */
    recoveryIndex |= RECOVERY_ENABLED(depacketizer);
    recoveryIndex |= PHASE_NUDGE_ENABLED(depacketizer);
  }
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, RECOVERY_INDEX_REG),
            recoveryIndex);

  /* Set an initial half-period for depacketizer >= 1.6 */
  if ((depacketizer->capabilities.versionMajor > 1) ||
      (depacketizer->capabilities.versionMinor >= 6)) {

    XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, MC_RTC_INCREMENT_REG),
              0x40000000 | MC_RTC_INCREMENT_FORCE); /* TODO - Where should we get the nominal increment from? */
    XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, MC_REMAINDER_REG),
              clockDomainSettings->remainder);
    XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, MC_HALF_PERIOD_REG),
              clockDomainSettings->halfPeriod);
  }

  /* Finally, configure the VCO control logic for the domain.  If the domain is enabled,
   * it is configured as a proportional controller to servo an external VCO via a DAC
   * to the audio clock frequency, based upon a feedback word clock.
   *
   * If the domain is disabled, the VCO control is set to output a constant zero value.
   * If a VCXO is being used, this will run it at the nominal frequency.
   *
   * For the moment, only an Analog Devices' AD5662 DAC and a Citizen CSX-750V VCXO are
   * being supported, so the parameters are hard-coded.  Once this is relaxed to permit
   * support for different clock recovery hardware, the parameters should be brought out
   * via the ioctl() interface.
   */
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, DAC_OFFSET_REG),
            DAC_OFFSET_ZERO);
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, DAC_P_COEFF_REG),
            (((clockDomainSettings->enabled == DOMAIN_ENABLED) ||
	      (clockDomainSettings->enabled == DOMAIN_SYNC)) 
	     ? DAC_COEFF_MAX : DAC_COEFF_ZERO));
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, LOCK_COUNT_REG),
            ((512 << VCO_LOCK_COUNT_SHIFT) | (8 << VCO_UNLOCK_COUNT_SHIFT)));

  /* TODO: Need to introduce some locked status and interrupt mask / flag bits in the hardware!
   *       Once they exist, a lock detection kernel event mechanism can be added to the driver
   *       and used when in slave mode.
   */
}

static void get_clock_recovery_info(struct audio_depacketizer *depacketizer, 
                                     ClockRecoveryInfo *clockRecoveryInfo) {
  ClockDomainSettings *clockDomainSettings;
  uint32_t clockDomain;
  uint32_t recoveryIndex;
  uint32_t controlValue;
  uint32_t sampleRate;

  /* Configure the timestamp interval for the domain first.  This informs the basic
   * reference clock recovery hardware of how many samples are being averaged each
   * time it receives a valid timestamp from the depacketizer.
   */
  clockDomainSettings = &clockRecoveryInfo->clockDomainSettings;
  clockDomain = clockDomainSettings->clockDomain;
  clockDomainSettings->sytInterval=XIo_In32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, MC_SYT_INTERVAL_REG));

  controlValue = XIo_In32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, MC_CONTROL_REG));
  /* Configure the generated clock edge for the clock domain that corresponds to a
   * sample. This should match the gateware that is providing the "current" time
   * reference to the depacketizer.
   */
  if(controlValue&MC_CONTROL_SAMPLE_EDGE_RISING) {
    clockDomainSettings->sampleEdge=DOMAIN_SAMPLE_EDGE_RISING;
  } else {
    clockDomainSettings->sampleEdge=DOMAIN_SAMPLE_EDGE_FALLING;
  }
  if(controlValue&MC_CONTROL_SELECTED_CLIENT) {
    clockRecoveryInfo->selected_client=true;
  } else {
    clockRecoveryInfo->selected_client=false;
  }
  if(controlValue&MC_CONTROL_SYNC_EXTERNAL) {
    clockDomainSettings->enabled=DOMAIN_SYNC;
    clockRecoveryInfo->external_sync=true;
  } else {
    clockDomainSettings->enabled=DOMAIN_DISABLED;
    clockRecoveryInfo->external_sync=false;
  }
  if(controlValue&MC_CONTROL_HAS_COAST_HOST_RTC) {
    clockRecoveryInfo->has_coast_host_rtc=true;
  } else {
    clockRecoveryInfo->has_coast_host_rtc=false;
  }
  if(controlValue&MC_CONTROL_IS_COASTING) {
    clockRecoveryInfo->coasting=true;
  } else {
    clockRecoveryInfo->coasting=false;
  }
  sampleRate=XIo_In32(REGISTER_ADDRESS(depacketizer, SAMPLE_RATE_REG));
  if(sampleRate==SINGLE_SAMPLE_RATE) {
    clockDomainSettings->sampleRate=ENGINE_SAMPLE_RATE_48_KHZ;
  } else if(sampleRate==DOUBLE_SAMPLE_RATE) {
    clockDomainSettings->sampleRate=ENGINE_SAMPLE_RATE_96_KHZ;
  } else {
    clockDomainSettings->sampleRate=ENGINE_SAMPLE_RATE_192_KHZ;
  }
  recoveryIndex=XIo_In32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, RECOVERY_INDEX_REG));
  clockRecoveryInfo->matchUnit=recoveryIndex & STREAM_INDEX_MASK(depacketizer);
  if(recoveryIndex&RECOVERY_ENABLED(depacketizer)) {
    clockDomainSettings->enabled=DOMAIN_ENABLED;
  }
  if(recoveryIndex&PHASE_NUDGE_ENABLED(depacketizer)) {
    clockRecoveryInfo->nudge_enabled=true;
  } else {
    clockRecoveryInfo->nudge_enabled=false;
  }
  clockRecoveryInfo->increment= XIo_In32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, MC_RTC_INCREMENT_REG));
  if(clockRecoveryInfo->increment&MC_RTC_INCREMENT_FORCE) {
    clockRecoveryInfo->rtc_increment_force=true;
    clockRecoveryInfo->increment&=~MC_RTC_INCREMENT_FORCE;
  } else {
    clockRecoveryInfo->rtc_increment_force=false;
  }
  clockDomainSettings->remainder=XIo_In32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, MC_REMAINDER_REG));
  clockDomainSettings->halfPeriod=XIo_In32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, MC_HALF_PERIOD_REG));

  clockRecoveryInfo->dac_offset=XIo_In32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, DAC_OFFSET_REG));
  clockRecoveryInfo->dac_p_coeff=XIo_In32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, DAC_P_COEFF_REG));
  clockRecoveryInfo->dac_lock_count=XIo_In32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, LOCK_COUNT_REG));
  clockRecoveryInfo->dac_control=XIo_In32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, DAC_CONTROL_REG));

  if(clockRecoveryInfo->dac_lock_count & (1<<24)) {
    clockRecoveryInfo->dac_locked=true;
  } else {
    clockRecoveryInfo->dac_locked=false;
  }
  clockRecoveryInfo->dac_unlock_count=(clockRecoveryInfo->dac_lock_count>>VCO_UNLOCK_COUNT_SHIFT)&VCO_UNLOCK_COUNT_MASK;
  clockRecoveryInfo->dac_lock_count=(clockRecoveryInfo->dac_lock_count>>VCO_LOCK_COUNT_SHIFT)&VCO_LOCK_COUNT_MASK;
}


/* Collects the stream status from the hardware */
static void get_stream_status(struct audio_depacketizer *depacketizer,
			      uint32_t *statusWords) {
  statusWords[0] = XIo_In32(REGISTER_ADDRESS(depacketizer, STREAM_STATUS_0_REG));
  statusWords[1] = XIo_In32(REGISTER_ADDRESS(depacketizer, STREAM_STATUS_1_REG));
  statusWords[2] = XIo_In32(REGISTER_ADDRESS(depacketizer, STREAM_STATUS_2_REG));
  statusWords[3] = XIo_In32(REGISTER_ADDRESS(depacketizer, STREAM_STATUS_3_REG));
}

/* Resets the state of the passed instance */
static void reset_depacketizer(struct audio_depacketizer *depacketizer) {
  /* Disable the instance and all of its match units */
  disable_depacketizer(depacketizer);
  set_rtc_unstable(depacketizer);
  clear_all_matchers(depacketizer);
  set_vector_base_address(depacketizer, 0x00000000);
}

/* Interrupt service routine for the instance */
static irqreturn_t labx_audio_depacketizer_interrupt(int irq, void *dev_id) {
  struct audio_depacketizer *depacketizer = (struct audio_depacketizer*) dev_id;
  uint32_t maskedFlags;
  uint32_t irqMask;
  uint32_t seqError;
  irqreturn_t returnValue = IRQ_NONE;

  /* Read the interrupt flags and immediately clear them */
  /* Grab the sequence error index prior to clearing the IRQ, it's 
     possible the index will change if cleared first */
  seqError = XIo_In32(REGISTER_ADDRESS(depacketizer, ERROR_REG));
  maskedFlags = XIo_In32(REGISTER_ADDRESS(depacketizer, IRQ_FLAGS_REG));
  irqMask = XIo_In32(REGISTER_ADDRESS(depacketizer, IRQ_MASK_REG));
  maskedFlags &= irqMask;
  XIo_Out32(REGISTER_ADDRESS(depacketizer, IRQ_FLAGS_REG), maskedFlags);

  /* Detect the timer IRQ */
  if((maskedFlags & SYNC_IRQ) != 0) {
    /* Wake up all threads waiting for a synchronization event */
    wake_up_interruptible(&(depacketizer->syncedWriteQueue));
    returnValue = IRQ_HANDLED;
  }
  
  /* Detect the stream change and stream reset IRQs; either one should
   * simply trigger the netlink thread.
   */
  if((maskedFlags & (STREAM_IRQ | SEQ_ERROR_IRQ)) != 0) {
    /* Increment the status count */
    depacketizer->streamStatusGeneration++;

    /* If this was a sequence error IRQ, leave a flag in place */
    if((maskedFlags & SEQ_ERROR_IRQ) != 0) {
      depacketizer->streamSeqError = 1;
      depacketizer->errorIndex = seqError;
    }

    /* Disarm both event interrupts while the status thread handles the present
     * event(s).  This permits the status thread to limit the rate at which events
     * are accepted and propagated up to userspace.
     */
    irqMask &= ~(STREAM_IRQ | SEQ_ERROR_IRQ);
    XIo_Out32(REGISTER_ADDRESS(depacketizer, IRQ_MASK_REG), irqMask);

    /* Wake up all threads waiting for a stream status event */
    wake_up_interruptible(&(depacketizer->streamStatusQueue));
    returnValue = IRQ_HANDLED;
  }

  /* Return whether this was an IRQ we handled or not */
  return(returnValue);
}

static int netlink_thread(void *data)
{
  struct audio_depacketizer *depacketizer = (struct audio_depacketizer*)data;
  uint32_t streamStatusGeneration = depacketizer->streamStatusGeneration;
  uint32_t irqMask;
  unsigned int channelIndex;
  unsigned int statusWordIndex;
  static const unsigned int statusWordOffset[STREAM_STATUS_WORDS] =
      {STREAM_STATUS_0_REG, STREAM_STATUS_1_REG, STREAM_STATUS_2_REG, STREAM_STATUS_3_REG};
  uint32_t streamStatus;
  uint32_t statusWordMask;
  uint32_t lastStreamStatus[STREAM_STATUS_WORDS];

  __set_current_state(TASK_RUNNING);
  lastStreamStatus[0] = XIo_In32(REGISTER_ADDRESS(depacketizer, STREAM_STATUS_0_REG));
  lastStreamStatus[1] = XIo_In32(REGISTER_ADDRESS(depacketizer, STREAM_STATUS_1_REG));
  lastStreamStatus[2] = XIo_In32(REGISTER_ADDRESS(depacketizer, STREAM_STATUS_2_REG));
  lastStreamStatus[3] = XIo_In32(REGISTER_ADDRESS(depacketizer, STREAM_STATUS_3_REG));

  do {
    set_current_state(TASK_INTERRUPTIBLE);
      
    wait_event_interruptible(depacketizer->streamStatusQueue,
                             ((depacketizer->streamStatusGeneration != streamStatusGeneration) || (kthread_should_stop())));

    if (kthread_should_stop()) break;

    __set_current_state(TASK_RUNNING);

    streamStatusGeneration = depacketizer->streamStatusGeneration;

    // Software based muting: If a stream's status has changed from Active to Inactive, then
    // zero the channel sample buffers associated with the stream.  For each of the 128 possible
    // streams, check for an Active to Inactive transition and clear the buffer if it has occurred.
    // Only buffers which have been registered for software-based muting will be cleared out.
    // TODO: This does not currently check for a redundancy partner; it should do so!
    for (channelIndex = 0, statusWordIndex = 0; statusWordIndex < STREAM_STATUS_WORDS; statusWordIndex++) {
      // There are four stream status registers we have to check; each has an Active bit for 32 streams
      streamStatus = XIo_In32(REGISTER_ADDRESS(depacketizer, statusWordOffset[statusWordIndex]));
      // Save time by skipping the bit checking if nothing has changed for this set of channels
      if (streamStatus != lastStreamStatus[statusWordIndex]) {
        // Start with a mask of 0x1, then shift it to 0x2 and so on until 0x40000000, 0x80000000, and 0
        for (statusWordMask = 0x1; statusWordMask != 0; statusWordMask <<= 1) {
          if (depacketizer->presentationChannels[channelIndex] != NULL &&
              (streamStatus & statusWordMask) == 0 &&
              (lastStreamStatus[statusWordIndex] & statusWordMask) != 0) {
        	void **channelBuffer;
            // Stream has become inactive
            // printk("Software based muting for %p: Stream %u inactive - mute buffer address", depacketizer, channelIndex);
            for (channelBuffer = depacketizer->presentationChannels[channelIndex]->channelBuffers;
                *channelBuffer != NULL; ++channelBuffer) {
              // printk(" %p", *channelBuffer);
              memset(*channelBuffer, 0, depacketizer->presentationChannels[channelIndex]->channelBufferSizeBytes);
            }
            // printk("\n");
          }
          ++channelIndex;
        }
        lastStreamStatus[statusWordIndex] = streamStatus;
      } else {
        channelIndex += 32;
      }
    }

    audio_depacketizer_stream_event(depacketizer);

    /* Before returning to waiting, optionally sleep a little bit and then
     * re-enable the IRQs which trigger us.  There should be no need to disable
     * interrupts to prevent a race condition since the only part of the ISR
     * which modifies the IRQ mask is the servicing of the IRQs which are at
     * present disabled.
     *
     * Inserting a delay here, in conjunction with the disabling of the event
     * IRQ flags by the ISR, effectively limits the rate at which events can
     * be generated and sent up to user space.  Using an ISR / waitqueue is,
     * however, more responsive to occasional events than straight-up polling.
     */
    msleep(EVENT_THROTTLE_MSECS);
    irqMask = XIo_In32(REGISTER_ADDRESS(depacketizer, IRQ_MASK_REG));
    irqMask |= (STREAM_IRQ | SEQ_ERROR_IRQ);
    XIo_Out32(REGISTER_ADDRESS(depacketizer, IRQ_MASK_REG), irqMask);
  } while (!kthread_should_stop());

  return 0;
}

/*
 * Character device hook functions
 */
static int open_count=0;

static int audio_depacketizer_open(struct inode *inode, struct file *filp)
{
  struct audio_depacketizer *depacketizer;
  unsigned long flags;
  int returnValue = 0;

  depacketizer = container_of(inode->i_cdev, struct audio_depacketizer, cdev);
  filp->private_data = depacketizer;

  if(open_count==0) {
    /* Lock the mutex and ensure there is only one owner */
      preempt_disable();
      spin_lock_irqsave(&depacketizer->mutex, flags);
      if(depacketizer->opened) {
        returnValue = -1;
      } else {
        depacketizer->opened = true;
      }
    spin_unlock_irqrestore(&depacketizer->mutex, flags);
    preempt_enable();

    /* Ensure the packet engine is reset */
    reset_depacketizer(depacketizer);
  
    /* Open the DMA, if we have one */
    if(depacketizer->hasDma == INSTANCE_HAS_DMA) {
      labx_dma_open(&depacketizer->dma);
    }
  }
  open_count++;
//  printk("depacketizer open %d\n", open_count);
  
  return(returnValue);
}

static int audio_depacketizer_release(struct inode *inode, struct file *filp)
{
  struct audio_depacketizer *depacketizer = (struct audio_depacketizer*)filp->private_data;
  unsigned long flags;

//  printk("depacketizer close %d\n", open_count);
  open_count--;
  if(open_count==0) {
    /* Release the DMA, if we have one */
    if(depacketizer->hasDma == INSTANCE_HAS_DMA) {
      labx_dma_release(&depacketizer->dma);
    }

    /* Ensure the packet engine is reset */
    reset_depacketizer(depacketizer);

    preempt_disable();
    spin_lock_irqsave(&depacketizer->mutex, flags);
    depacketizer->opened = false;
    spin_unlock_irqrestore(&depacketizer->mutex, flags);
    preempt_enable();
  }

  return(0);
}

/* Buffer for storing configuration words */
#define MAX_CONFIG_WORDS 1024
static uint32_t configWords[MAX_CONFIG_WORDS];

/* I/O control operations for the driver */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,38)
static long audio_depacketizer_ioctl(struct file *filp,
                                     unsigned int command, unsigned long arg)
{
  long returnValue = 0;
#else
static int audio_depacketizer_ioctl(struct inode *inode,
                                    struct file *filp,
                                    unsigned int command, unsigned long arg)
{
  int returnValue = 0;
#endif
  struct audio_depacketizer *depacketizer = (struct audio_depacketizer*)filp->private_data;

  // Switch on the request
  switch(command) {
  case IOC_START_ENGINE:
    enable_depacketizer(depacketizer);
    break;

  case IOC_STOP_ENGINE:
    disable_depacketizer(depacketizer);
    break;

  case IOC_LOAD_DESCRIPTOR:
    {
      ConfigWords userDescriptor;
      ConfigWords localDescriptor;

      if(copy_from_user(&userDescriptor, (void __user*)arg, sizeof(userDescriptor)) != 0) {
        return(-EFAULT);
      }

      /* Sanity-check the number of words against our maximum */
      if((userDescriptor.offset + userDescriptor.numWords) > 
         depacketizer->capabilities.maxInstructions) {
        return(-ERANGE);
      }

      localDescriptor.offset          = userDescriptor.offset;
      localDescriptor.interlockedLoad = userDescriptor.interlockedLoad;
      localDescriptor.loadFlags       = userDescriptor.loadFlags;
      localDescriptor.configWords     = configWords;
      while(userDescriptor.numWords > 0) {
        /* Load in chunks, never exceeding our local buffer size */
        localDescriptor.numWords = ((userDescriptor.numWords > MAX_CONFIG_WORDS) ?
                                    MAX_CONFIG_WORDS : userDescriptor.numWords);
        if(copy_from_user(configWords, (void __user*)userDescriptor.configWords, 
                          (localDescriptor.numWords * sizeof(uint32_t))) != 0) {
          return(-EFAULT);
        }
        returnValue                 = load_descriptor(depacketizer, &localDescriptor);
        userDescriptor.configWords += localDescriptor.numWords;
        localDescriptor.offset     += localDescriptor.numWords;
        userDescriptor.numWords    -= localDescriptor.numWords;
        if(returnValue < 0) break;
      }
    }
    break;

  case IOC_COPY_DESCRIPTOR:
    {
      ConfigWords userDescriptor;
      ConfigWords localDescriptor;

      /* Copy into our local descriptor, then obtain buffer pointer from userland */
      if(copy_from_user(&userDescriptor, (void __user*)arg, sizeof(userDescriptor)) != 0) {
        return(-EFAULT);
      }

      /* Sanity-check the number of words against our maximum */
      if((userDescriptor.offset + userDescriptor.numWords) > 
         depacketizer->capabilities.maxInstructions) {
        return(-ERANGE);
      }

      localDescriptor.offset      = userDescriptor.offset;
      localDescriptor.configWords = configWords;
      while(userDescriptor.numWords > 0) {
        /* Transfer in chunks, never exceeding our local buffer size */
        localDescriptor.numWords = ((userDescriptor.numWords > MAX_CONFIG_WORDS) ? 
                                    MAX_CONFIG_WORDS : userDescriptor.numWords);
        copy_descriptor(depacketizer, &localDescriptor);
        if(copy_to_user((void __user*)userDescriptor.configWords, configWords, 
                        (localDescriptor.numWords * sizeof(uint32_t))) != 0) {
          return(-EFAULT);
        }
        userDescriptor.configWords += localDescriptor.numWords;
        localDescriptor.offset     += localDescriptor.numWords;
        userDescriptor.numWords    -= localDescriptor.numWords;
      }
    }
    break;

  case IOC_CLEAR_MATCHERS:
    clear_all_matchers(depacketizer);
    break;

  case IOC_CONFIG_MATCHER:
    {
      MatcherConfig matcherConfig;
      if(copy_from_user(&matcherConfig, (void __user*)arg, sizeof(matcherConfig)) != 0) return(-EFAULT);
      returnValue = configure_matcher(depacketizer, &matcherConfig);
    }
    break;

  case IOC_RELOCATE_MATCHER:
    {
      MatcherRelocation matcherRelocation;
      if(copy_from_user(&matcherRelocation, (void __user*)arg, sizeof(matcherRelocation)) != 0) return(-EFAULT);
      returnValue = relocate_matcher(depacketizer, &matcherRelocation);
    }
    break;

  case IOC_LOCATE_VECTOR_TABLE:
    {
      uint32_t vectorBaseAddress;

      if(copy_from_user(&vectorBaseAddress, (void __user*)arg, sizeof(vectorBaseAddress)) != 0) {
        return(-EFAULT);
      }
      set_vector_base_address(depacketizer, vectorBaseAddress);
    }
    break;

  case IOC_CONFIG_CLOCK_RECOVERY:
    {
      ClockRecoverySettings clockRecoverySettings;
      if(copy_from_user(&clockRecoverySettings, (void __user*)arg, sizeof(clockRecoverySettings)) != 0) {
        return(-EFAULT);
      }
      configure_clock_recovery(depacketizer, &clockRecoverySettings);
    }
    break;

  case IOC_GET_DEPACKETIZER_CAPS:
    {
      if(copy_to_user((void __user*)arg, &depacketizer->capabilities, sizeof(DepacketizerCaps)) != 0) {
        return(-EFAULT);
      }
    }
    break;

  case IOC_GET_STREAM_STATUS:
    {
      uint32_t statusWords[STREAM_STATUS_WORDS];

      /* Get the stream status, then copy into the userspace pointer */
      get_stream_status(depacketizer, statusWords);
      if(copy_to_user((void __user*)arg, statusWords, 
                      (STREAM_STATUS_WORDS * sizeof(uint32_t))) != 0) {
        return(-EFAULT);
      }
    }
    break;
      
  case IOC_SET_RTC_STABLE:
    set_rtc_stable(depacketizer);
    break;

  case IOC_SET_RTC_UNSTABLE:
    set_rtc_unstable(depacketizer);
    break;

  case IOC_SET_MCR_RTC_INCREMENT:
    {
      ClockDomainIncrement cdi;

      if(copy_from_user(&cdi, (void __user*)arg, sizeof(cdi)) != 0) {
        return(-EFAULT);
      }

      XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, cdi.clockDomain, MC_RTC_INCREMENT_REG), cdi.increment);
    }
    break;

  case IOC_SET_AUTOMUTE_STREAM:
    {
      struct depacketizer_presentation_channels *channels;
      SetAutomuteStream setAutomuteStream;
      if(copy_from_user(&setAutomuteStream, (void __user*)arg, sizeof(SetAutomuteStream)) != 0 ||
    		  setAutomuteStream.streamIndex > DEPACKETIZER_MAX_STREAMS ||
    		  setAutomuteStream.nChannels > DEPACKETIZER_MAX_AUTOMUTE_STREAMS ) {
        return(-EFAULT);
      }
      if (depacketizer->presentationChannels[setAutomuteStream.streamIndex] != NULL) {
        kfree(depacketizer->presentationChannels[setAutomuteStream.streamIndex]);
      }
      if (setAutomuteStream.nChannels > 0) {
        void **channelBuffer;
        channels = kmalloc(sizeof(struct depacketizer_presentation_channels) +
      		  setAutomuteStream.nChannels*sizeof(void *), GFP_KERNEL);
        channels->channelBufferSizeBytes = setAutomuteStream.channelBufferSizeBytes;
        channels->redundancyPartner = NULL;
        // Caution: The buffer channel addresses reported to the userspace app are actually
        // in kernel memory space, having been (usually) obtained via kmalloc().  So, it
        // is safe to simply copy them here.  Beware, however, that we have no way to enforce
        // this and the addresses may in fact be virtual (user space) addresses, which would
        // be tragic in the general case where user and kernel address spaces are different.
        memcpy(channels->channelBuffers, setAutomuteStream.channelAddresses,
            setAutomuteStream.nChannels*sizeof(void *));
        channels->channelBuffers[setAutomuteStream.nChannels] = NULL;
        depacketizer->presentationChannels[setAutomuteStream.streamIndex] = channels;

        // Initial mute of stream channels
        // printk("Software based muting for %p: Initial mute stream %u size %u - mute buffer address", depacketizer, setAutomuteStream.streamIndex, channels->channelBufferSizeBytes);
        for (channelBuffer = channels->channelBuffers; *channelBuffer != NULL; ++channelBuffer) {
          // printk(" %p", *channelBuffer);
          memset(*channelBuffer, 0, channels->channelBufferSizeBytes);
        }
        // printk("\n");

      } else {
        depacketizer->presentationChannels[setAutomuteStream.streamIndex] = NULL;
      }
    }
    break;

    case IOC_GET_CLOCK_RECOVERY:
    {
      ClockRecoveryInfo clockRecoveryInfo;
      /* user specifies clockDomain to query */
      if(copy_from_user(&clockRecoveryInfo, (void __user*)arg, sizeof(clockRecoveryInfo)) != 0) {
        return(-EFAULT);
      }
      get_clock_recovery_info(depacketizer, &clockRecoveryInfo);
      if(copy_to_user((void __user*)arg,&clockRecoveryInfo, sizeof(clockRecoveryInfo)) != 0) {
        return(-EFAULT);
      }
    }
    break;

    case IOC_SET_DAC_OFFSET:
    {
      ClockDomainDacOffset cddo;

      if(copy_from_user(&cddo, (void __user*)arg, sizeof(cddo)) != 0) {
        return(-EFAULT);
      }

      XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, cddo.clockDomain, DAC_OFFSET_REG), cddo.offset);
    }
    break;

    case IOC_SET_DAC_COEFF:
    {
      ClockDomainDacCoeff cddc;

      if(copy_from_user(&cddc, (void __user*)arg, sizeof(cddc)) != 0) {
        return(-EFAULT);
      }

      XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, cddc.clockDomain, DAC_P_COEFF_REG), cddc.coeff);
    }
    break;

  default:
#ifdef CONFIG_LABX_AUDIO_DEPACKETIZER_DMA
    if(depacketizer->hasDma == INSTANCE_HAS_DMA) {
      return labx_dma_ioctl(&depacketizer->dma, command, arg);
    } else return(-EINVAL);
#else
    return(-EINVAL);
#endif
  }

  return(returnValue);
}

/* Character device file operations structure */
static struct file_operations audio_depacketizer_fops = {
  .open	          = audio_depacketizer_open,
  .release        = audio_depacketizer_release,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,38)
  .unlocked_ioctl = audio_depacketizer_ioctl,
#else
  .ioctl          = audio_depacketizer_ioctl,
#endif
  .owner          = THIS_MODULE,
};

/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 */
static int audio_depacketizer_probe(const char *name, 
                                    struct platform_device *pdev,
                                    struct resource *addressRange,
                                    struct resource *irq,
                                    const char *interfaceType,
                                    int is_mcr_host) {
  struct audio_depacketizer *depacketizer;
  uint32_t capsWord;
  uint32_t versionWord;
  uint32_t versionMajor;
  uint32_t versionMinor;
  uint32_t versionCompare;
  uint32_t maxStreamShifter;
  int32_t dmaIrqParam;
  int returnValue;
  
  /* Create and populate a device structure */
  depacketizer = (struct audio_depacketizer*) kmalloc(sizeof(struct audio_depacketizer), 
                                                      GFP_KERNEL);
  if(!depacketizer) return(-ENOMEM);
  memset(depacketizer, 0, sizeof(struct audio_depacketizer));

  /* Request and map the device's I/O memory region into uncacheable space */
  depacketizer->physicalAddress = addressRange->start;
  depacketizer->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
  snprintf(depacketizer->name, NAME_MAX_SIZE, "%s", name);
  depacketizer->name[NAME_MAX_SIZE - 1] = '\0';
  if(request_mem_region(depacketizer->physicalAddress, depacketizer->addressRangeSize,
                        depacketizer->name) == NULL) {
    returnValue = -ENOMEM;
    goto free;
  }

  depacketizer->virtualAddress = 
    (void*) ioremap_nocache(depacketizer->physicalAddress, depacketizer->addressRangeSize);
  if(!depacketizer->virtualAddress) {
    returnValue = -ENOMEM;
    goto release;
  }

  /* Ensure that the interrupts are disabled */
  XIo_Out32(REGISTER_ADDRESS(depacketizer, IRQ_MASK_REG), NO_IRQS);

  /* Retain the IRQ and register our handler, if an IRQ resource was supplied. */
  if(irq != NULL) {
    /* Request the IRQ as a shared line, since we may share it with a DMA */
    depacketizer->irq = irq->start;
    returnValue = request_irq(depacketizer->irq, 
                              &labx_audio_depacketizer_interrupt, 
                              IRQF_SHARED, 
                              depacketizer->name, 
                              depacketizer);
    if (returnValue) {
      printk(KERN_ERR "%s : Could not allocate Lab X Audio Depacketizer interrupt (%d).\n",
             depacketizer->name, depacketizer->irq);
      goto unmap;
    }
  } else depacketizer->irq = NO_IRQ_SUPPLIED;

  /* Read the capabilities word to determine how many of the lowest
   * address bits are used to index into the microcode RAM, and therefore how
   * many bits an address sub-range field gets shifted up.  Each instruction is
   * 32 bits, and therefore inherently eats two lower address bits.
   */
  capsWord = XIo_In32(REGISTER_ADDRESS(depacketizer, CAPABILITIES_REG_B));
  depacketizer->regionShift = ((capsWord & CODE_ADDRESS_BITS_MASK) + 2);

  /* Inspect and check the version */
  versionWord = XIo_In32(REGISTER_ADDRESS(depacketizer, REVISION_REG));
  versionMajor = ((versionWord >> REVISION_FIELD_BITS) & REVISION_FIELD_MASK);
  versionMinor = (versionWord & REVISION_FIELD_MASK);
  versionCompare = ((versionMajor << REVISION_FIELD_BITS) | versionMinor);
  if((versionCompare < DRIVER_VERSION_MIN) | 
     (versionCompare > DRIVER_VERSION_MAX)) {
    printk(KERN_INFO "%s: Found incompatible hardware version %d.%d at 0x%08X\n",
           depacketizer->name, versionMajor, versionMinor, (uint32_t)depacketizer->physicalAddress);
    returnValue = -ENXIO;
    goto unmap;
  }
  depacketizer->capabilities.versionMajor = versionMajor;
  depacketizer->capabilities.versionMinor = versionMinor;

  /* Test and sanity-check the stream-matching architecture for legacy
   * implementations; version 1.2 and up uses a more efficient and unified
   * architecture for all device families.
   */
  if(versionCompare < UNIFIED_MATCH_VERSION_MIN) {
    depacketizer->matchArchitecture = ((capsWord >> MATCH_ARCH_SHIFT) & MATCH_ARCH_MASK);
    switch(depacketizer->matchArchitecture) {
    case STREAM_MATCH_SRLC16E:
    case STREAM_MATCH_SRLC32E:
      break;
    default:
      printk(KERN_INFO "%s: Invalid match architecture 0x%02X\n", 
	     depacketizer->name, depacketizer->matchArchitecture);
      returnValue = -ENXIO;
      goto unmap;
    }
  } else {
    depacketizer->matchArchitecture = STREAM_MATCH_UNIFIED;
    depacketizer->capabilities.dynamicSampleRates = ((capsWord >> DYN_SAMPLE_RATES_SHIFT) & 0x1);
  }

  /* Capture more capabilities information */
  depacketizer->capabilities.maxInstructions = (0x01 << (capsWord & CODE_ADDRESS_BITS_MASK));
  depacketizer->capabilities.maxParameters   = (0x01 << ((capsWord >> PARAM_ADDRESS_BITS_SHIFT) & 
                                                         PARAM_ADDRESS_BITS_MASK));
  depacketizer->capabilities.maxClockDomains = ((capsWord >> CLOCK_DOMAINS_SHIFT) & CLOCK_DOMAINS_MASK);
  depacketizer->capabilities.maxStreams      = ((capsWord >> MAX_STREAMS_SHIFT) & MAX_STREAMS_MASK);

  /* Calculate an appropriate mask for stream indices, given the maximum
   * number of streams supported by the instance.
   */
  depacketizer->streamIndexShift = 0;
  depacketizer->streamIndexMask  = 0;
  maxStreamShifter = (depacketizer->capabilities.maxStreams - 1);
  while(maxStreamShifter != 0) {
    // Construct the mask by shifting in ones; nothing guarantees that the max
    // number of streams is an integer power of two.
    depacketizer->streamIndexShift++;
    depacketizer->streamIndexMask = ((depacketizer->streamIndexMask << 1) | 0x01);
    maxStreamShifter >>= 1;
  }

  /* Instances below a particular version lack the 'A' capabilities register */
  if(versionCompare < EXTENDED_CAPS_VERSION_MIN) {
    /* Use an assumed value for the maximum stream slots */
    depacketizer->capabilities.maxStreamSlots = ASSUMED_MAX_STREAM_SLOTS;
  } else {
    /* Fetch this capability from the 'A' capabilities register */
    capsWord = XIo_In32(REGISTER_ADDRESS(depacketizer, CAPABILITIES_REG_A));
    depacketizer->capabilities.maxStreamSlots = (capsWord & MAX_STREAM_SLOTS_MASK);
  }

  if (is_mcr_host) {
    depacketizer->capabilities.coastHostRtcIncrement =
      (XIo_In32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, 0, MC_CONTROL_REG))
       & MC_CONTROL_HAS_COAST_HOST_RTC) ? 1 : 0;
  } else {
    depacketizer->capabilities.coastHostRtcIncrement = 0;
  }

  /* Announce the device */
  printk(KERN_INFO "%s: Found Lab X depacketizer %d.%d at 0x%08X, ",
         depacketizer->name, versionMajor, versionMinor, 
         (uint32_t)depacketizer->physicalAddress);
  if(depacketizer->irq == NO_IRQ_SUPPLIED) {
    printk("polled interlocks");
  } else {
    printk("IRQ %d", depacketizer->irq);
  }
  printk(", %s interface\n", interfaceType);

  /* Initialize other resources */
  spin_lock_init(&depacketizer->mutex);
  depacketizer->opened = false;

  /* Assign an instance number to the depacketizer for use as a minor number */
  depacketizer->instanceNumber = instanceCount++;

  /* Test to see whether the depacketizer instance has a Dma_Coprocessor
   * module contained within for handling the back-end of the audio data
   */
  if(strncmp(interfaceType, DMA_INTERFACE_PREFIX, strlen(DMA_INTERFACE_PREFIX)) == 0) {
#ifdef CONFIG_LABX_AUDIO_DEPACKETIZER_DMA
    /* There are 4 address blocks in the depacketizer, each sized the same 
     * as the instruction RAM (which is 4 bytes wide).
     * DMA is selected with the next bit up in the address
     */
    depacketizer->hasDma = INSTANCE_HAS_DMA;
    depacketizer->dma.virtualAddress = depacketizer->virtualAddress + (depacketizer->capabilities.maxInstructions*4*4);

    /* Provide the encapsulated DMA with the shared interrupt line */
    if(depacketizer->irq == NO_IRQ_SUPPLIED) {
      dmaIrqParam = DMA_NO_IRQ_SUPPLIED;
    } else {
      dmaIrqParam = depacketizer->irq;
    }

    /* Allow the underlying DMA driver to infer its microcode size */
    labx_dma_probe(&depacketizer->dma, 
                   DRIVER_MAJOR,
                   depacketizer->instanceNumber,
                   depacketizer->name, 
                   DMA_UCODE_SIZE_UNKNOWN, 
                   dmaIrqParam,
                   DMA_CALLBACKS); 
#else
    /* The interface type specified by the platform involves a DMA instance,
     * but the driver for the Dma_Coprocessor hasn't been enabled in the
     * build configuration!
     */
    depacketizer->hasDma = INSTANCE_NO_DMA;
    printk("%s\n has a Dma_Coprocessor, but DMA driver is not built\n", depacketizer->name);
#endif
  } else depacketizer->hasDma = INSTANCE_NO_DMA;

  /* Provide navigation between the device structures */
  platform_set_drvdata(pdev, depacketizer);
  depacketizer->pdev = pdev;

  /* Reset the state of the depacketizer */
  reset_depacketizer(depacketizer);

  /* Add as a character device to make the instance available for use */
  cdev_init(&depacketizer->cdev, &audio_depacketizer_fops);
  depacketizer->cdev.owner = THIS_MODULE;
  kobject_set_name(&depacketizer->cdev.kobj, "%s.%d", depacketizer->name, depacketizer->instanceNumber);
  returnValue = cdev_add(&depacketizer->cdev, MKDEV(DRIVER_MAJOR, depacketizer->instanceNumber), 1);
  if (returnValue < 0)
  {
    printk("%s: Unable to add character device %d.%d (%d)\n", pdev->name, DRIVER_MAJOR, depacketizer->instanceNumber, returnValue);
    goto unmap;
  }

  /* Initialize the waitqueue used for synchronized writes */
  init_waitqueue_head(&(depacketizer->syncedWriteQueue));

  /* Initialize the waitqueue used for stream status events */
  init_waitqueue_head(&(depacketizer->streamStatusQueue));

  /* Initialize the netlink state and start the thread */
  depacketizer->netlinkSequence = 0;
  depacketizer->netlinkTask = kthread_run(netlink_thread, (void*)depacketizer, "%s:netlink", depacketizer->name);
  if (IS_ERR(depacketizer->netlinkTask)) {
    printk(KERN_ERR "Depacketizer netlink task creation failed.\n");
    returnValue = -EIO;
    goto kthread_fail;
  }

  /* Now that the device is configured, enable interrupts if they are to be used,
   * clearing IRQ state first
   */
  depacketizer->streamStatusGeneration = 0;
  depacketizer->streamSeqError         = 0;
  depacketizer->errorIndex             = 0;
  if(depacketizer->irq != NO_IRQ_SUPPLIED) {
    XIo_Out32(REGISTER_ADDRESS(depacketizer, IRQ_MASK_REG), (SYNC_IRQ | STREAM_IRQ | SEQ_ERROR_IRQ));
  }

  /* Return success */
  return(0);

 kthread_fail:
  cdev_del(&depacketizer->cdev);
 unmap:
  iounmap(depacketizer->virtualAddress);
 release:
  release_mem_region(depacketizer->physicalAddress, depacketizer->addressRangeSize);
 free:
  kfree(depacketizer);
  return(returnValue);
}

#ifdef CONFIG_OF
static int audio_depacketizer_platform_remove(struct platform_device *pdev);

static u32 get_u32(struct of_device *ofdev, const char *s) {
  u32 *p = (u32 *)of_get_property(ofdev->node, s, NULL);
  if(p) {
    return *p;
  } else {
    dev_warn(&ofdev->dev, "Parameter %s not found, defaulting to false.\n", s);
    return FALSE;
  }
}

static int __devinit audio_depacketizer_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
  struct resource r_mem_struct;
  struct resource r_irq_struct;
  struct resource *addressRange = &r_mem_struct;
  struct resource *irq          = &r_irq_struct;
  struct platform_device *pdev = to_platform_device(&ofdev->dev);
  const char *name = dev_name(&ofdev->dev);
  const char *interfaceType;
  int is_mcr_host = 0;
  int rc;

  /* Obtain the resources for this instance */
  rc = of_address_to_resource(ofdev->node,0,addressRange);
  if (rc) {
    dev_warn(&ofdev->dev,"invalid address\n");
    return rc;
  }

  rc = of_irq_to_resource(ofdev->node, 0, irq);
  if(rc == NO_IRQ) {
    /* No IRQ was defined; null the resource pointer to indicate polled mode */
    irq = NULL;
  }

  interfaceType = (char *) of_get_property(ofdev->node, "xlnx,interface-type", NULL);
  if(interfaceType == NULL) {
    dev_warn(&ofdev->dev, "No interface type specified in device tree\n");
    return(-EFAULT);
  }

  is_mcr_host = get_u32(ofdev, "xlnx,is-mcr-host");

  /* Dispatch to the generic function */
  return(audio_depacketizer_probe(name, pdev, addressRange, irq, interfaceType, is_mcr_host));
}

static int __devexit audio_depacketizer_of_remove(struct of_device *dev)
{
	struct platform_device *pdev = to_platform_device(&dev->dev);
	audio_depacketizer_platform_remove(pdev);
	return(0);
}

static struct of_device_id audio_depacketizer_of_match[] = {
	{ .compatible = "xlnx,labx-audio-depacketizer-1.00.a", },
	{ .compatible = "xlnx,labx-audio-depacketizer-1.01.a", },
	{ .compatible = "xlnx,labx-audio-depacketizer-1.02.a", },
	{ .compatible = "xlnx,labx-audio-depacketizer-1.03.a", },
	{ .compatible = "xlnx,labx-audio-depacketizer-1.04.a", },
	{ .compatible = "xlnx,labx-audio-depacketizer-1.05.a", },
	{ .compatible = "xlnx,labx-audio-depacketizer-1.06.a", },
	{ .compatible = "xlnx,labx-audio-depacketizer-1.07.a", },
	{ .compatible = "xlnx,labx-audio-depacketizer-1.08.a", },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, audio_depacketizer_of_match);

static struct of_platform_driver of_audio_depacketizer_driver = {
	.name		= DRIVER_NAME,
	.match_table	= audio_depacketizer_of_match,
	.probe		= audio_depacketizer_of_probe,
	.remove		= __devexit_p(audio_depacketizer_of_remove),
};
#endif

/*
 * Platform device hook functions
 */

/* Probe for registered devices */
static int audio_depacketizer_platform_probe(struct platform_device *pdev)
{
  struct resource *addressRange;
  struct resource *irq;
  char *interfaceType;
  int is_mcr_host;

  /* Obtain the resources for this instance */
  addressRange = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!addressRange) {
    printk(KERN_ERR "%s: IO resource address not found.\n", pdev->name);
    return(-ENXIO);
  }

  /* Attempt to obtain the IRQ; if none is specified, the resource pointer is
   * NULL, and polling will be used.
   */
  irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

  /* The only other platform data provided is a string specifying the
   * interface type for the instance
   */
  interfaceType = (char *) pdev->dev.platform_data;
  if(interfaceType == NULL) {
    printk(KERN_ERR "%s: No interface type string specified\n", pdev->name);
    return(-EFAULT);
  }

  is_mcr_host = 1; /* TODO */

  /* Dispatch to the generic function */
  return(audio_depacketizer_probe(pdev->name, pdev, addressRange, irq, interfaceType, is_mcr_host));
}

static int audio_depacketizer_platform_remove(struct platform_device *pdev)
{
  struct audio_depacketizer *depacketizer;

  /* Get a handle to the packetizer and begin shutting it down */
  depacketizer = platform_get_drvdata(pdev);
  if(!depacketizer) return(-1);

  if (depacketizer->hasDma == INSTANCE_HAS_DMA) {
    labx_dma_remove(&depacketizer->dma);
  }

  /* Release the IRQ */
  if (depacketizer->irq != NO_IRQ_SUPPLIED) {
    free_irq(depacketizer->irq, depacketizer);
  }

  kthread_stop(depacketizer->netlinkTask);
  cdev_del(&depacketizer->cdev);
  reset_depacketizer(depacketizer);
  iounmap(depacketizer->virtualAddress);
  release_mem_region(depacketizer->physicalAddress, depacketizer->addressRangeSize);
  kfree(depacketizer);
  return(0);
}

/* Platform device driver structure */
static struct platform_driver audio_depacketizer_driver = {
  .probe  = audio_depacketizer_platform_probe,
  .remove = audio_depacketizer_platform_remove,
  .driver = {
    .name = DRIVER_NAME,
  }
};

/* Driver initialization and exit */
static int __init audio_depacketizer_driver_init(void)
{
  int returnValue;
  printk(KERN_INFO DRIVER_NAME ": AVB Audio Depacketizer Driver\n");
  printk(KERN_INFO DRIVER_NAME ": Copyright (c) Lab X Technologies, LLC\n");

#ifdef CONFIG_OF
  returnValue = of_register_platform_driver(&of_audio_depacketizer_driver);
#endif

  /* Register as a platform device driver */
  if((returnValue = platform_driver_register(&audio_depacketizer_driver)) < 0) {
    printk(KERN_INFO DRIVER_NAME ": Failed to register platform driver\n");
    goto error;
  }

  /* Allocate a range of major / minor device numbers for use */
  instanceCount = 0;
  if((returnValue = register_chrdev_region(MKDEV(DRIVER_MAJOR, 0),MAX_INSTANCES, DRIVER_NAME)) < 0) { 
    printk(KERN_INFO DRIVER_NAME ": Failed to allocate character device range\n");
    goto device_unregister;
  }

  /* Initialize the Netlink layer for the driver */
  register_audio_depacketizer_netlink();

  return(0);

device_unregister:
  platform_driver_unregister(&audio_depacketizer_driver);
error:
  return returnValue;
}

static void __exit audio_depacketizer_driver_exit(void)
{
  unregister_chrdev_region(MKDEV(DRIVER_MAJOR, 0),MAX_INSTANCES);

  /* Unregister Generic Netlink family */
  unregister_audio_depacketizer_netlink();

  /* Unregister as a platform device driver */
  platform_driver_unregister(&audio_depacketizer_driver);
}

module_init(audio_depacketizer_driver_init);
module_exit(audio_depacketizer_driver_exit);

MODULE_AUTHOR("Eldridge M. Mount IV <eldridge.mount@labxtechnologies.com>");
MODULE_DESCRIPTION("Lab X Technologies AVB Audio Depacketizer driver");
MODULE_LICENSE("GPL");
