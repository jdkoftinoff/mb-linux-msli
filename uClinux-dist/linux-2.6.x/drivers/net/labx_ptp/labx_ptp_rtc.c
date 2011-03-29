/*
 *  linux/drivers/net/labx_ptp_rtc.c
 *
 *  Lab X Technologies Precision Time Protocol (PTP) driver
 *  Real-Time Counter (RTC) interface methods
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

#include "labx_ptp.h"
#include <xio.h>

/* Uncomment to print debug messages for the slave offset */
/* #define SLAVE_OFFSET_DEBUG */

/* Threshold and purely-proportional coefficient to use when in phase
 * acquisition mode
 */
#define ACQUIRE_THRESHOLD (1000)
#define ACQUIRE_COEFF_P   ((int32_t)0xE0000000)

/* Saturation range limit for the integrator */
#define INTEGRAL_MAX_ABS  (1000LL)

/* Disables the RTC */
void disable_rtc(struct ptp_device *ptp) {
  XIo_Out32(REGISTER_ADDRESS(ptp, 0, PTP_RTC_INC_REG), PTP_RTC_DISABLE);
}

/* Sets the RTC increment, simultaneously enabling the RTC */
void set_rtc_increment(struct ptp_device *ptp, RtcIncrement *increment) {
  uint32_t incrementWord;

  /* Save the current increment if anyone needs it */
  ptp->currentIncrement = *increment;

  /* Assemble a single value from the increment components */
  incrementWord = ((increment->mantissa & RTC_MANTISSA_MASK) << RTC_MANTISSA_SHIFT);
  incrementWord |= (increment->fraction & RTC_FRACTION_MASK);
  incrementWord |= PTP_RTC_ENABLE;

  /* The actual write is already atomic, so no need to ensure mutual exclusion */
  XIo_Out32(REGISTER_ADDRESS(ptp, 0, PTP_RTC_INC_REG), incrementWord);
}

/* Return the current increment value */
void get_rtc_increment(struct ptp_device *ptp, RtcIncrement *increment) {
  *increment = ptp->currentIncrement;
}

/* Captures the present RTC time, returning it into the passed structure */
void get_rtc_time(struct ptp_device *ptp, PtpTime *time) {
  uint32_t timeWord;
  unsigned long flags;

  /* Write to the capture flag in the upper seconds word to initiate a capture,
   * then poll the same bit to determine when it has completed.  The capture only
   * takes a few RTC clocks, so this busy wait can only consume tens of nanoseconds.
   *
   * This will *not* modify the time, since we don't write the nanoseconds register.
   */
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  XIo_Out32(REGISTER_ADDRESS(ptp, 0, PTP_SECONDS_HIGH_REG), PTP_RTC_CAPTURE_FLAG);
  do {
    timeWord = XIo_In32(REGISTER_ADDRESS(ptp, 0, PTP_SECONDS_HIGH_REG));
  } while((timeWord & PTP_RTC_CAPTURE_FLAG) != 0);

  /* Now read the entire captured time and pack it into the structure.  The last
   * value read during polling is perfectly valid.
   */
  time->secondsUpper = (uint16_t) timeWord;
  time->secondsLower = XIo_In32(REGISTER_ADDRESS(ptp, 0, PTP_SECONDS_LOW_REG));
  time->nanoseconds = XIo_In32(REGISTER_ADDRESS(ptp, 0, PTP_NANOSECONDS_REG));
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();
}

void get_local_time(struct ptp_device *ptp, PtpTime *time) {
  uint32_t timeWord;
  unsigned long flags;

  /* Write to the capture flag in the upper seconds word to initiate a capture,
   * then poll the same bit to determine when it has completed.  The capture only
   * takes a few RTC clocks, so this busy wait can only consume tens of nanoseconds.
   *
   * This will *not* modify the time, since we don't write the nanoseconds register.
   */
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  XIo_Out32(REGISTER_ADDRESS(ptp, 0, PTP_LOCAL_SECONDS_HIGH_REG), PTP_RTC_LOCAL_CAPTURE_FLAG);
  do {
    timeWord = XIo_In32(REGISTER_ADDRESS(ptp, 0, PTP_LOCAL_SECONDS_HIGH_REG));
  } while((timeWord & PTP_RTC_LOCAL_CAPTURE_FLAG) != 0);

  /* Now read the entire captured time and pack it into the structure.  The last
   * value read during polling is perfectly valid.
   */
  time->secondsUpper = (uint16_t) timeWord;
  time->secondsLower = XIo_In32(REGISTER_ADDRESS(ptp, 0, PTP_LOCAL_SECONDS_LOW_REG));
  time->nanoseconds = XIo_In32(REGISTER_ADDRESS(ptp, 0, PTP_LOCAL_NANOSECONDS_REG));
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();
}


/* Sets a new RTC time from the passed structure */
void set_rtc_time(struct ptp_device *ptp, PtpTime *time) {
  unsigned long flags;

  /* Write to the time register, beginning with the seconds.  The write to the 
   * nanoseconds register is what actually effects the change to the RTC.
   */
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  XIo_Out32(REGISTER_ADDRESS(ptp, 0, PTP_SECONDS_HIGH_REG), time->secondsUpper);
  XIo_Out32(REGISTER_ADDRESS(ptp, 0, PTP_SECONDS_LOW_REG), time->secondsLower);
  XIo_Out32(REGISTER_ADDRESS(ptp, 0, PTP_NANOSECONDS_REG), time->nanoseconds);
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();
}

/* Updates lock detection metrics */
void update_rtc_lock_detect(struct ptp_device *ptp) {
  int32_t lockRangeSigned    = (int32_t) ptp->properties.lockRangeNsec;
  int32_t unlockThreshSigned = (int32_t) ptp->properties.unlockThreshNsec;

  /* Determine whether we are looking for transition to lock or unlock.
   * In either case, we use valid slave offsets to determine whether we're
   * satisfying our heuristics for lock or unlock.
   */
  if(ptp->rtcLockState == PTP_RTC_UNLOCKED) {
    /* RTC is unlocked, try to acquire lock.  If we don't have a valid RTC
     * offset, we can't move forward in attempting to lock, and should reset.
     */
    if(ptp->rtcLastOffsetValid == PTP_RTC_OFFSET_VALID) {

      /* See if the offset lies within our lock range and, if so, whether it has
       * been so for long enough.
       */
      if((ptp->rtcLastOffset > -lockRangeSigned) & (ptp->rtcLastOffset < lockRangeSigned)) {
        /* Within lock range, check counter */
        if(++ptp->rtcLockCounter >= ptp->rtcLockTicks) {
          /* Achieved lock! Change state and send a Netlink message. */
          ptp->rtcLockState   = PTP_RTC_LOCKED;
          ptp->rtcLockCounter = 0;
        }
      } else ptp->rtcLockCounter = 0;
    } else ptp->rtcLockCounter = 0;
  } else {
    /* RTC is already locked, see if we need to become unlocked.  If the last
     * offset is valid, we can proceed to examining its value.
     * TODO: Should we check to make sure we aren't getting an ongoing burst of
     *       invalid offsets?  This would indicate that the basic low-level message
     *       handshaking between master / slave or peer / peer is malfunctioning!
     */
    if(ptp->rtcLastOffsetValid == PTP_RTC_OFFSET_VALID) {
      /* The lock detection is hysteretic; once locked, it takes more than just
       * one out-of-range sample to cause us to declare loss of lock, as long as
       * it is not beyond a configurable threshold of sanity.
       */
      if((ptp->rtcLastOffset < -lockRangeSigned) | (ptp->rtcLastOffset > lockRangeSigned)) {
        /* Out of the lock range, see if it's past the "immediately unlock" threshold */
        if((ptp->rtcLastOffset < -unlockThreshSigned) | (ptp->rtcLastOffset > unlockThreshSigned)) {
          /* Way out of range, unlock immediately and send a Netlink message */
          ptp->rtcLockState   = PTP_RTC_UNLOCKED;
          ptp->rtcLockCounter = 0;
       } else {
          /* Out of the lock range, but not severely; check the counter */
          if(++ptp->rtcLockCounter >= ptp->rtcUnlockTicks) {
            /* We've become unlocked.  Change state and send a Netlink message. */
            ptp->rtcLockState   = PTP_RTC_UNLOCKED;
            ptp->rtcLockCounter = 0;
          }
        }
      } else ptp->rtcLockCounter = 0;
    } else ptp->rtcLockCounter = 0;
  }
}

/* RTC increment constants */
#define INCREMENT_ONE         ((int32_t) 0x08000000)
#define INCREMENT_NEG_ONE     ((int32_t) 0xF8000000)
#define INCREMENT_HALF        ((int32_t) 0x04000000)
#define INCREMENT_NEG_HALF    ((int32_t) 0xFC000000)
#define INCREMENT_QUARTER     ((int32_t) 0x02000000)
#define INCREMENT_NEG_QUARTER ((int32_t) 0xFE000000)
#define INCREMENT_EIGHTH      ((int32_t) 0x01000000)
#define INCREMENT_NEG_EIGHTH  ((int32_t) 0xFF000000)
#define INCREMENT_DELTA_MAX   (INCREMENT_EIGHTH)
#define INCREMENT_DELTA_MIN   (INCREMENT_NEG_EIGHTH)

/* Bits to shift to convert a coefficient product */
#define COEFF_PRODUCT_SHIFT  (28)

/* Updates the RTC servo when a slave */
#ifdef SLAVE_OFFSET_DEBUG
static uint32_t servoCount = 0;
#endif

void rtc_update_servo(struct ptp_device *ptp, uint32_t port) {
  int32_t slaveOffset       = 0;
  uint32_t slaveOffsetValid = PTP_RTC_OFFSET_INVALID;
  PtpTime difference;

  /* Update the servo using the appropriate delay mechanism */
  if(ptp->properties.delayMechanism == PTP_DELAY_MECHANISM_E2E) {
    /* Make certain there are both sets of valid timestamps available for a master->
     * slave offset calculation employing the end-to-end mechanism.
     */
    if(ptp->ports[port].syncTimestampsValid && ptp->ports[port].delayReqTimestampsValid) {
      PtpTime difference2;
      
      /* The core of the algorithm is the calculation of the slave's offset from the
       * master, eliminating the network delay from the equation:
       *
       * [SYNC Rx time - SYNC Tx time] = slave_error + link_delay
       * [DELAY_REQ Rx time - DELAY_REQ Tx time] = -slave_error + link_delay
       *
       * Rearranging terms to get link delay by itself and equate the two expressions
       * gives the following equation for the master-to-slave offset:
       *
       * Offset_m_s = [(SYNC_Rx - SYNC_Tx) + (DELAY_REQ_Tx - DELAY_REQ_Rx)] / 2
       */
      timestamp_difference(&ptp->ports[port].syncRxTimestamp, &ptp->ports[port].syncTxTimestamp, &difference);
      timestamp_difference(&ptp->ports[port].delayReqTxTimestamp, &ptp->ports[port].delayReqRxTimestamp, &difference2);
      
      /* The fact that this is called at all implies there's a < 1 sec slaveOffset; deal
       * strictly with nanoseconds now that the seconds have been normalized.
       */
      slaveOffset = (((int32_t) difference.nanoseconds) + ((int32_t) difference2.nanoseconds));
      slaveOffset >>= 1;
      slaveOffsetValid = PTP_RTC_OFFSET_VALID;

      /* Save the delay in the same spot as P2P mode does for consistency. */
      ptp->ports[port].neighborPropDelay = (-difference2.nanoseconds) - slaveOffset;
    }
  } else {
    /* The peer delay mechanism uses the SYNC->FUP messages, but relies upon the
     * messages having had their correction field updated by bridge residence time
     * logic along the way.  Since that is performed, the only remaining correction
     * to be made is to subtract out the path delay to our peer, which is periodically
     * calculated (and should be pretty small.)
     */
    timestamp_difference(&ptp->ports[port].syncRxTimestamp, &ptp->ports[port].syncTxTimestamp, &difference);
      
    /* The fact that this is called at all implies there's a < 1 sec slaveOffset; deal
     * strictly with nanoseconds now that the seconds have been normalized.
     */
    slaveOffset = (((int32_t) difference.nanoseconds) - ptp->ports[port].neighborPropDelay);
    slaveOffsetValid = PTP_RTC_OFFSET_VALID;
  }

  /* Perform the actual servo update if the slave offset is valid */
  if(slaveOffsetValid == PTP_RTC_OFFSET_VALID) {
    uint32_t newRtcIncrement;
    int64_t coefficient;
    int64_t slaveOffsetExtended;
    int64_t accumulator;
    int32_t adjustment;

    /* Update the servo with the present value; begin with the nominal increment */
    newRtcIncrement = (ptp->nominalIncrement.mantissa & RTC_MANTISSA_MASK);
    newRtcIncrement <<= RTC_MANTISSA_SHIFT;
    newRtcIncrement |= (ptp->nominalIncrement.fraction & RTC_FRACTION_MASK);
    
    /* Operate in two distinct modes; a high-gain, purely-proportional control loop
     * when we're far from the master, and a more complete set of controls once we've
     * narrowed in
     */
    if((slaveOffset > ACQUIRE_THRESHOLD) | (slaveOffset < -ACQUIRE_THRESHOLD)) {
      /* Accumulate the proportional coefficient's contribution */
      coefficient = (int64_t) ACQUIRE_COEFF_P;
      slaveOffsetExtended = (int64_t) slaveOffset;
      accumulator = ((coefficient * slaveOffsetExtended) >> COEFF_PRODUCT_SHIFT);

      /* Also dump the integrator for the integral term */
      ptp->integral = 0;
    } else {
      /* Accumulate the proportional coefficient's contribution */
      slaveOffsetExtended = (int64_t) slaveOffset;
      coefficient = (int64_t) ptp->coefficients.P;
      accumulator = ((coefficient * slaveOffsetExtended) >> COEFF_PRODUCT_SHIFT);

      /* Accumulate the integral coefficient's contribution, clamping the integrated
       * error to its bounds.
       */
      coefficient = (int64_t) ptp->coefficients.I;
      ptp->integral += slaveOffsetExtended;
      if(ptp->integral > INTEGRAL_MAX_ABS) {
        ptp->integral = INTEGRAL_MAX_ABS;
      } else if(ptp->integral < -INTEGRAL_MAX_ABS) {
        ptp->integral = -INTEGRAL_MAX_ABS;
      }
      accumulator += ((coefficient * ptp->integral) >> COEFF_PRODUCT_SHIFT);

      /* Accumulate the derivitave coefficient's contribution */
      coefficient = (int64_t) ptp->coefficients.D;
      ptp->derivative += (slaveOffset - ptp->previousOffset); /* TODO: Scale based on the time between syncs? */
      accumulator += ((coefficient * (int64_t)ptp->derivative) >> COEFF_PRODUCT_SHIFT);
      ptp->previousOffset = slaveOffset;
    }
    
    /* Clamp the new increment to within +/- one nanosecond of nominal */
    adjustment = (int32_t) accumulator;
    if(adjustment > INCREMENT_DELTA_MAX) {
      adjustment = INCREMENT_DELTA_MAX;
    } else if(adjustment < INCREMENT_DELTA_MIN) {
      adjustment = INCREMENT_DELTA_MIN;
    }
    newRtcIncrement += adjustment;
    
    /* Write the new increment out to the hardware, incorporating the enable bit.
     * Suppress the actual write to the RTC increment register if the userspace
     * control has not acknowledged a Grandmaster change.
     */
    newRtcIncrement |= PTP_RTC_ENABLE;
    if(ptp->rtcChangesAllowed) {
      /* Save the current increment if anyone needs it */
      ptp->currentIncrement.mantissa = (newRtcIncrement >> RTC_MANTISSA_SHIFT) & RTC_MANTISSA_MASK;
      ptp->currentIncrement.fraction = (newRtcIncrement & RTC_FRACTION_MASK);

      /* Update the increment register */
      XIo_Out32(REGISTER_ADDRESS(ptp, 0, PTP_RTC_INC_REG), newRtcIncrement); 
    }

#ifdef SLAVE_OFFSET_DEBUG
    if(++servoCount >= 10) {
      printk("Slave offset %d, Increment 0x%08X\n", slaveOffset, newRtcIncrement);
      printk("  syncRxNS %d, syncTxNS %d (%d), MeanPathNS %d\n", (int)ptp->ports[port].syncRxTimestamp.nanoseconds,
        (int)ptp->ports[port].syncTxTimestamp.nanoseconds, (int)difference.nanoseconds, (int)ptp->ports[port].neighborPropDelay);
      printk("RTC increment 0x%08X", newRtcIncrement);
      if(adjustment == INCREMENT_DELTA_MIN) {
        printk(" (MIN CLAMP)");
      } else if(adjustment == INCREMENT_DELTA_MAX) {
        printk(" (MAX CLAMP)");
      }
      printk("\n");
      servoCount = 0;
    }
#endif
  } /* if(slaveOffsetValid) */

  /* Store the offset and its validity to the device structure for use by
   * the lock detection state machine
   */
  ptp->rtcLastOffsetValid = slaveOffsetValid;
  ptp->rtcLastOffset      = slaveOffset;
}
