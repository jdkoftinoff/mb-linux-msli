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

#include <linux/autoconf.h>
#include "labx_audio_depacketizer.h"
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <xio.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF


/* Driver name and the revision of hardware expected (1.1) */
#define DRIVER_NAME "labx_audio_depacketizer"
#define HARDWARE_VERSION_MAJOR  1
#define HARDWARE_VERSION_MINOR  1

/* Major device number for the driver */
#define DRIVER_MAJOR 252

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

/* Syntonization callback */
static void depacketizer_syntonize(uint32_t rtcIncrement, void *callbackParam) {
  struct audio_depacketizer *depacketizer = (struct audio_depacketizer*)callbackParam;
  
  /* Write the new RTC increment to the depacketizer's local syntonization
   * counter so that it remains syntonized to 802.1AS network time.
   */
  XIo_Out32(REGISTER_ADDRESS(depacketizer, RTC_INCREMENT_REG), rtcIncrement);
}

/* Disables the passed instance */
static void disable_depacketizer(struct audio_depacketizer *depacketizer) {
  DBG("Disbling the depacketizer\n");

  /* Disable the micro-engine, and unregister its syntonization callback */
  // TEMPORARY
  XIo_Out32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG), DEPACKETIZER_DISABLE);
  //  XIo_Out32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG), (ID_LOAD_LAST_WORD | DEPACKETIZER_DISABLE));
  remove_syntonize_callback(&depacketizer_syntonize, (void*)depacketizer);
}

/* Enables the passed instance */
static void enable_depacketizer(struct audio_depacketizer *depacketizer) {
  DBG("Enabling the depacketizer\n");

  /* Initialize the local syntonized counter to start at a nominal rate, and
   * then register the instance with a callback for syntonization updates 
   */
  XIo_Out32(REGISTER_ADDRESS(depacketizer, RTC_INCREMENT_REG), NOMINAL_RTC_INCREMENT);
  add_syntonize_callback(&depacketizer_syntonize, (void*) depacketizer);

  /* Enable the micro-engine, with the "last load" flag for match unit configuration
   * disabled (the loading methods should assert this as needed.)
   */
  XIo_Out32(REGISTER_ADDRESS(depacketizer, CONTROL_STATUS_REG), DEPACKETIZER_ENABLE);
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
    selectionWord = (((matchUnit < 64) & (matchUnit > 32)) ? (0x01 << (matchUnit - 32)) : ID_SELECT_NONE);
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_1_REG), selectionWord);
    selectionWord = (((matchUnit < 96) & (matchUnit > 64)) ? (0x01 << (matchUnit - 64)) : ID_SELECT_NONE);
    XIo_Out32(REGISTER_ADDRESS(depacketizer, ID_SELECT_2_REG), selectionWord);
    selectionWord = ((matchUnit > 96) ? (0x01 << (matchUnit - 96)) : ID_SELECT_NONE);
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
  relocationWord = (matcherRelocation->matchUnit & RELOCATION_MATCH_MASK);
  relocationWord |= ((matcherRelocation->oldVector + matcherRelocation->ringStateOffset) << 
                     RELOCATION_ADDRESS_SHIFT);
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
  relocationWord = (matcherRelocation->matchUnit & RELOCATION_MATCH_MASK);
  relocationWord |= ((matcherRelocation->newVector + matcherRelocation->ringStateOffset) << 
                     RELOCATION_ADDRESS_SHIFT);
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

  /* Configure the timestamp interval for the domain first.  This informs the basic
   * reference clock recovery hardware of how many samples are being averaged each
   * time it receives a valid timestamp from the depacketizer.
   */
  clockDomainSettings = &clockRecoverySettings->clockDomainSettings;
  clockDomain = clockDomainSettings->clockDomain;
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, TS_INTERVAL_REG),
            clockDomainSettings->sytInterval);

  /* Configure the clock domain with which match unit it gets its temporal 
   * information from.  The match units, in turn, link a stream index to its AVBTP
   * stream ID, so a descriptor must be configured to receive a stream on this
   * index.
   */
  recoveryIndex = (clockRecoverySettings->matchUnit & STREAM_INDEX_MASK);
  if(clockDomainSettings->enabled == DOMAIN_ENABLED) {
    recoveryIndex |= RECOVERY_ENABLED;
  }
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, RECOVERY_INDEX_REG),
            recoveryIndex);

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
            ((clockDomainSettings->enabled == DOMAIN_ENABLED) ? DAC_COEFF_MAX : DAC_COEFF_ZERO));
  XIo_Out32(CLOCK_DOMAIN_REGISTER_ADDRESS(depacketizer, clockDomain, LOCK_COUNT_REG),
            ((512 << VCO_LOCK_COUNT_SHIFT) | 0));

  /* TODO: Need to introduce some locked status and interrupt mask / flag bits in the hardware!
   *       Once they exist, a lock detection kernel event mechanism can be added to the driver
   *       and used when in slave mode.
   */
}

/* Resets the state of the passed instance */
static void reset_depacketizer(struct audio_depacketizer *depacketizer) {
  /* Disable the instance and all of its match units */
  disable_depacketizer(depacketizer);
  clear_all_matchers(depacketizer);
  set_vector_base_address(depacketizer, 0x00000000);
}

/* Interrupt service routine for the instance */
static irqreturn_t labx_audio_depacketizer_interrupt(int irq, void *dev_id) {
  struct audio_depacketizer *depacketizer = (struct audio_depacketizer*) dev_id;
  uint32_t maskedFlags;

  /* Read the interrupt flags and immediately clear them */
  maskedFlags = XIo_In32(REGISTER_ADDRESS(depacketizer, IRQ_FLAGS_REG));
  maskedFlags &= XIo_In32(REGISTER_ADDRESS(depacketizer, IRQ_MASK_REG));
  XIo_Out32(REGISTER_ADDRESS(depacketizer, IRQ_FLAGS_REG), maskedFlags);

  /* Detect the timer IRQ */
  if((maskedFlags & SYNC_IRQ) != 0) {
    /* Wake up all threads waiting for a synchronization event */
    wake_up_interruptible(&(depacketizer->syncedWriteQueue));
  }
  
  return(IRQ_HANDLED);
}

/*
 * Character device hook functions
 */

static int audio_depacketizer_open(struct inode *inode, struct file *filp)
{
  struct audio_depacketizer *depacketizer;
  unsigned long flags;
  int returnValue = 0;

  depacketizer = container_of(inode->i_cdev, struct audio_depacketizer, cdev);
  filp->private_data = depacketizer;

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
  
  return(returnValue);
}

static int audio_depacketizer_release(struct inode *inode, struct file *filp)
{
  struct audio_depacketizer *depacketizer = (struct audio_depacketizer*)filp->private_data;
  unsigned long flags;

  preempt_disable();
  spin_lock_irqsave(&depacketizer->mutex, flags);
  depacketizer->opened = false;
  spin_unlock_irqrestore(&depacketizer->mutex, flags);
  preempt_enable();
  return(0);
}

/* Buffer for storing configuration words */
static uint32_t configWords[MAX_CONFIG_WORDS];

/* I/O control operations for the driver */
static int audio_depacketizer_ioctl(struct inode *inode, struct file *filp,
                                    unsigned int command, unsigned long arg)
{
  struct audio_depacketizer *depacketizer = (struct audio_depacketizer*)filp->private_data;
  int returnValue = 0;

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
      ConfigWords descriptor;

      if(copy_from_user(&descriptor, (void __user*)arg, sizeof(descriptor)) != 0) {
        return(-EFAULT);
      }
      if(copy_from_user(configWords, (void __user*)descriptor.configWords, 
                        (descriptor.numWords * sizeof(uint32_t))) != 0) {
        return(-EFAULT);
      }
      descriptor.configWords = configWords;
      returnValue = load_descriptor(depacketizer, &descriptor);
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
      localDescriptor.offset = userDescriptor.offset;
      localDescriptor.numWords = userDescriptor.numWords;
      localDescriptor.configWords = configWords;
      copy_descriptor(depacketizer, &localDescriptor);
      if(copy_to_user((void __user*)userDescriptor.configWords, configWords, 
                      (userDescriptor.numWords * sizeof(uint32_t))) != 0) {
        return(-EFAULT);
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
      
  default:
#ifdef CONFIG_LABX_AUDIO_DEPACKETIZER_DMA
    return labx_dma_ioctl(&depacketizer->dma, command, arg);
#else
    return(-EINVAL);
#endif
  }

  return(returnValue);
}

/* Character device file operations structure */
static struct file_operations audio_depacketizer_fops = {
  .open	   = audio_depacketizer_open,
  .release = audio_depacketizer_release,
  .ioctl   = audio_depacketizer_ioctl,
  .owner   = THIS_MODULE,
};

/* Function containing the "meat" of the probe mechanism - this is used by
 * the OpenFirmware probe as well as the standard platform device mechanism.
 */
static int audio_depacketizer_probe(const char *name, 
                                    struct platform_device *pdev,
                                    struct resource *addressRange,
                                    struct resource *irq) {
  struct audio_depacketizer *depacketizer;
  uint32_t capsWord;
  uint32_t versionWord;
  uint32_t versionMajor;
  uint32_t versionMinor;
  int returnValue;
  
  /* Create and populate a device structure */
  depacketizer = (struct audio_depacketizer*) kmalloc(sizeof(struct audio_depacketizer), 
                                                      GFP_KERNEL);
  if(!depacketizer) return(-ENOMEM);

  /* Request and map the device's I/O memory region into uncacheable space */
  depacketizer->physicalAddress = addressRange->start;
  depacketizer->addressRangeSize = ((addressRange->end - addressRange->start) + 1);
  snprintf(depacketizer->name, NAME_MAX_SIZE, "%s%d", name, pdev->id);
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
    depacketizer->irq = irq->start;
    returnValue = request_irq(depacketizer->irq, &labx_audio_depacketizer_interrupt, 
                              IRQF_DISABLED, depacketizer->name, depacketizer);
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
  capsWord = XIo_In32(REGISTER_ADDRESS(depacketizer, CAPABILITIES_REG));
  depacketizer->regionShift = ((capsWord & CODE_ADDRESS_BITS_MASK) + 2);

  /* Inspect and check the version */
  versionWord = XIo_In32(REGISTER_ADDRESS(depacketizer, REVISION_REG));
  versionMajor = ((versionWord >> REVISION_FIELD_BITS) & REVISION_FIELD_MASK);
  versionMinor = (versionWord & REVISION_FIELD_MASK);
  if((versionMajor != HARDWARE_VERSION_MAJOR) | 
     (versionMinor != HARDWARE_VERSION_MINOR)) {
    printk(KERN_INFO "%s: Found incompatible hardware version %d.%d at 0x%08X\n",
           depacketizer->name, versionMajor, versionMinor, (uint32_t)depacketizer->physicalAddress);
    returnValue = -ENXIO;
    goto unmap;
  }
  depacketizer->capabilities.versionMajor = versionMajor;
  depacketizer->capabilities.versionMinor = versionMinor;

  /* Test and sanity-check the stream-matching architecture */
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

  /* Capture more capabilities information */
  depacketizer->capabilities.maxInstructions = (0x01 << (capsWord & CODE_ADDRESS_BITS_MASK));
  depacketizer->capabilities.maxParameters   = (0x01 << ((capsWord >> PARAM_ADDRESS_BITS_SHIFT) & 
                                                         PARAM_ADDRESS_BITS_MASK));
  depacketizer->capabilities.maxClockDomains = ((capsWord >> CLOCK_DOMAINS_SHIFT) & CLOCK_DOMAINS_MASK);
  depacketizer->capabilities.maxStreams      = ((capsWord >> MAX_STREAMS_SHIFT) & MAX_STREAMS_MASK);

  /* Announce the device */
  printk(KERN_INFO "%s: Found Lab X depacketizer %d.%d at 0x%08X, ",
         depacketizer->name, versionMajor, versionMinor, 
         (uint32_t)depacketizer->physicalAddress);
  if(depacketizer->irq == NO_IRQ_SUPPLIED) {
    printk("polled interlocks\n");
  } else {
    printk("IRQ %d\n", depacketizer->irq);
  }

  /* Initialize other resources */
  spin_lock_init(&depacketizer->mutex);
  depacketizer->opened = false;

#ifdef CONFIG_LABX_AUDIO_DEPACKETIZER_DMA
  /* There are 4 address blocks in the depacketizer, each sized the same as the instruction RAM (which is 4 bytes wide).
   * DMA is selected with the next bit up in the address */
  depacketizer->dma.virtualAddress = depacketizer->virtualAddress + (depacketizer->capabilities.maxInstructions*4*4);
 
  labx_dma_probe(&depacketizer->dma); 
#endif

  /* Provide navigation between the device structures */
  platform_set_drvdata(pdev, depacketizer);
  depacketizer->pdev = pdev;

  /* Reset the state of the depacketizer */
  reset_depacketizer(depacketizer);

  /* Add as a character device to make the instance available for use */
  cdev_init(&depacketizer->cdev, &audio_depacketizer_fops);
  depacketizer->cdev.owner = THIS_MODULE;
  kobject_set_name(&depacketizer->cdev.kobj, "%s%d", pdev->name, pdev->id);
  depacketizer->instanceNumber = instanceCount++;
  returnValue = cdev_add(&depacketizer->cdev, MKDEV(DRIVER_MAJOR, depacketizer->instanceNumber), 1);
  if (returnValue < 0)
  {
    printk("%s: Unable to add character device %d.%d (%d)\n", pdev->name, DRIVER_MAJOR, depacketizer->instanceNumber, returnValue);
    goto unmap;
  }

  /* Initialize the waitqueue used for synchronized writes */
  init_waitqueue_head(&(depacketizer->syncedWriteQueue));

  /* Now that the device is configured, enable interrupts if they are to be used */
  if(depacketizer->irq != NO_IRQ_SUPPLIED) {
    XIo_Out32(REGISTER_ADDRESS(depacketizer, IRQ_MASK_REG), SYNC_IRQ);
  }

  /* Return success */
  return(0);

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

static int __devinit audio_depacketizer_of_probe(struct of_device *ofdev, const struct of_device_id *match)
{
  struct resource r_mem_struct;
  struct resource r_irq_struct;
  struct resource *addressRange = &r_mem_struct;
  struct resource *irq          = &r_irq_struct;
  struct platform_device *pdev = to_platform_device(&ofdev->dev);
  int rc;

  printk(KERN_INFO "Device Tree Probing \'%s\'\n", ofdev->node->name);

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
    return(rc);
  }

  /* Dispatch to the generic function */
  return(audio_depacketizer_probe(ofdev->node->name, pdev, addressRange, irq));
}

static int __devexit audio_depacketizer_of_remove(struct of_device *dev)
{
	struct platform_device *pdev = to_platform_device(&dev->dev);
	audio_depacketizer_platform_remove(pdev);
	return(0);
}

static struct of_device_id audio_depacketizer_of_match[] = {
	{ .compatible = "xlnx,labx-audio-depacketizer-1.00.a", },
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

  /* Dispatch to the generic function */
  return(audio_depacketizer_probe(pdev->name, pdev, addressRange, irq));
}

static int audio_depacketizer_platform_remove(struct platform_device *pdev)
{
  struct audio_depacketizer *depacketizer;

  /* Get a handle to the packetizer and begin shutting it down */
  depacketizer = platform_get_drvdata(pdev);
  if(!depacketizer) return(-1);

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
  printk(KERN_INFO DRIVER_NAME ": AVB Audio Depacketizer %d.%d driver\n",
         HARDWARE_VERSION_MAJOR, HARDWARE_VERSION_MINOR);
  printk(KERN_INFO DRIVER_NAME ": Copyright(c) Lab X Technologies, LLC\n");

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
    printk(KERN_INFO DRIVER_NAME "Failed to allocate character device range\n");
    goto device_unregister;
  }
  return(0);

device_unregister:
  platform_driver_unregister(&audio_depacketizer_driver);
error:
  return returnValue;
}

static void __exit audio_depacketizer_driver_exit(void)
{
  unregister_chrdev_region(MKDEV(DRIVER_MAJOR, 0),MAX_INSTANCES);

  /* Unregister as a platform device driver */
  platform_driver_unregister(&audio_depacketizer_driver);
}

module_init(audio_depacketizer_driver_init);
module_exit(audio_depacketizer_driver_exit);

MODULE_AUTHOR("Eldridge M. Mount IV <eldridge.mount@labxtechnologies.com>");
MODULE_DESCRIPTION("Lab X Technologies AVB Audio Depacketizer driver");
MODULE_LICENSE("GPL");
