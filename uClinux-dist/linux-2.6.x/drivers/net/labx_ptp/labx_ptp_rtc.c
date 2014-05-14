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

/* Uncomment to print debug messages for the slave offset */
/* #define SLAVE_OFFSET_DEBUG */

/* Threshold and purely-proportional coefficient to use when in phase
 * acquisition mode
 */
#define ACQUIRE_THRESHOLD (10000)
#define ACQUIRE_COEFF_P   ((int32_t)0xE0000000)

/* Rate ratio limits that are considered to be reasonable */
#define RATE_RATIO_MAX ((uint32_t)0x80100000)
#define RATE_RATIO_MIN ((uint32_t)0x7FF00000)

/* Saturation range limit for the integrator */
#define INTEGRAL_MAX_ABS  (100000LL)


/* Sets a new RTC time from the passed structure */
void set_rtc_time_adjusted(struct ptp_device *ptp, PtpTime *time, PtpTime *entryTime) {
  unsigned long flags;
  PtpTime timeNow;
  PtpTime timeDifference;
  PtpTime adjustedTime;

  /* Get the current time and make the adjustment with as little jitter as possible */
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);

  get_rtc_time(ptp, &timeNow);
  timestamp_difference(&timeNow, entryTime, &timeDifference);
  timestamp_sum(time, &timeDifference, &adjustedTime);
  set_rtc_time(ptp, &adjustedTime);

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
      if((ptp->rtcLastOffset > -lockRangeSigned) & (ptp->rtcLastOffset < lockRangeSigned) &
          (ptp->rtcLastIncrementDelta > -0x1000) & (ptp->rtcLastIncrementDelta < 0x1000)) {
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

/* Calculate the rate ratio from the master. Note that we reuse the neighbor rate ratio
   fields from PDELAY but it is really the master we are talking to here. */
static void computeDelayRateRatio(struct ptp_device *ptp, uint32_t port)
{
  if (ptp->ports[port].initPdelayRespReceived == FALSE)
  {
    /* Capture the initial DELAY response */
    ptp->ports[port].initPdelayRespReceived = TRUE;
    ptp->ports[port].pdelayRespTxTimestampI = ptp->ports[port].delayReqTxLocalTimestamp;
    ptp->ports[port].pdelayRespRxTimestampI = ptp->ports[port].delayReqRxTimestamp;
  }
  else
  {
    PtpTime difference;
    PtpTime difference2;
    uint64_t nsResponder;
    uint64_t nsRequester;
    uint64_t rateRatio;
    int shift;

    timestamp_difference(&ptp->ports[port].delayReqTxLocalTimestamp, &ptp->ports[port].pdelayRespTxTimestampI, &difference2);
    timestamp_difference(&ptp->ports[port].delayReqRxTimestamp, &ptp->ports[port].pdelayRespRxTimestampI, &difference);

    /* The raw differences have been computed; sanity-check the peer delay timestamps; if the
     * initial Tx or Rx timestamp is later than the present one, the initial ones are bogus and
     * must be replaced.
     */
    if((difference.secondsUpper & 0x80000000) |
       (difference2.secondsUpper & 0x80000000)) {
      ptp->ports[port].initPdelayRespReceived = FALSE;
      ptp->ports[port].neighborRateRatioValid = FALSE;
      ptp->masterRateRatioValid = FALSE;
    } else {
      nsResponder = ((uint64_t)difference.secondsLower) * 1000000000ULL + (uint64_t)difference.nanoseconds;
      nsRequester = ((uint64_t)difference2.secondsLower) * 1000000000ULL + (uint64_t)difference2.nanoseconds;

      for (shift = 0; shift < 31; shift++)
        {
          if (nsResponder & (1ULL<<(63-shift))) break;
        }

      if ((nsRequester >> (31-shift)) != 0) {
        rateRatio = (nsResponder << shift) / (nsRequester >> (31-shift));
        if (((uint32_t)rateRatio < RATE_RATIO_MAX) && ((uint32_t)rateRatio > RATE_RATIO_MIN)) {
          ptp->ports[port].neighborRateRatio = (uint32_t)rateRatio;

          ptp->ports[port].neighborRateRatioValid = TRUE;

          /* Master rate is the same for E2E mode */
          ptp->masterRateRatio = (uint32_t)rateRatio;
          ptp->masterRateRatioValid = TRUE;
        } else {
          /* If we are outside the acceptable range, assume our initial values are bad and grab new ones */
          ptp->ports[port].initPdelayRespReceived = FALSE;
          ptp->ports[port].neighborRateRatioValid = FALSE;
          ptp->masterRateRatioValid = FALSE;
        }
      }

#ifdef PATH_DELAY_DEBUG
      printk("Responder delta: %08X%08X.%08X (%llu ns)\n", difference.secondsUpper,
             difference.secondsLower, difference.nanoseconds, nsResponder);
      printk("Requester delta: %08X%08X.%08X (%llu ns)\n", difference2.secondsUpper,
             difference2.secondsLower, difference2.nanoseconds, nsRequester);
      printk("Rate ratio: %08X (shift %d)\n", ptp->ports[port].neighborRateRatio, shift);
#endif
    } /* if(differences are sane) */
  }
}

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

      computeDelayRateRatio(ptp, port);

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
      ptp->ports[port].neighborPropDelay = (-difference2.nanoseconds) + slaveOffset;

      /* Mark the delay timestamps as invalid so we don't keep using them with their old offset */
      ptp->ports[port].delayReqTimestampsValid = 0;
    } else if (ptp->ports[port].syncTimestampsValid) {

      /* Cancel out the link delay with the last computed value.
       *
       * [SYNC Rx time - SYNC Tx time] = slave_error + link_delay
       * slaveOffset = slave_error + link_delay - link delay
       */
      timestamp_difference(&ptp->ports[port].syncRxTimestamp, &ptp->ports[port].syncTxTimestamp, &difference);
      slaveOffset = difference.nanoseconds - ptp->ports[port].neighborPropDelay;
      slaveOffsetValid = PTP_RTC_OFFSET_VALID;
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

    if (ptp->ports[port].neighborRateRatioValid) {
      uint64_t tempRate;
      // Convert from 2^-41 - (1.0) back to something in the 2^-31 range and add the 1.0 back in
      tempRate = (((int32_t)ptp->ports[port].cumulativeScaledRateOffset) >> 10) + 0x80000000;

      // Get the cumulative rate ratio, including our neighbor
      tempRate = ((ptp->ports[port].neighborRateRatio * tempRate) >> 31);

      if (!ptp->masterRateRatioValid) {
        ptp->integral = 0;
        ptp->zeroCrossingIntegral = 0;
      }

      ptp->masterRateRatio = (uint32_t)tempRate;
      ptp->masterRateRatioValid = TRUE;
    }
    else {
      ptp->masterRateRatioValid = FALSE;
    }
  }

  /* Perform the actual servo update if the slave offset is valid */
  if(slaveOffsetValid == PTP_RTC_OFFSET_VALID) {
    uint32_t newRtcIncrement;
    int64_t coefficient;
    int64_t slaveOffsetExtended;
    int64_t accumulator = 0;
    int32_t adjustment;

    /* Update the servo with the present value; begin with the master rate ratio
     * if it is available, otherwise start with the nominal increment */
    if (ptp->masterRateRatioValid) {
      uint32_t scaledMasterRateRatio = ptp->masterRateRatio >> 1;
      int32_t rateDiff = (int32_t)(scaledMasterRateRatio - ptp->prevBaseRtcIncrement);
      if (rateDiff < 1000 && rateDiff > -1000) {
        // Use the previous value as a base. Include a portion of the master rate ratio to speed up corrections.
        newRtcIncrement = ptp->prevBaseRtcIncrement + (rateDiff>>4);
      } else {
        // Use the master rate ratio as a base
        newRtcIncrement = scaledMasterRateRatio;
      }
      ptp->prevBaseRtcIncrement = newRtcIncrement;

      /* If we crossed the midpoint, damp the integral */
      if (((slaveOffset < 0) && (ptp->previousOffset > 0)) ||
          ((slaveOffset > 0) && (ptp->previousOffset < 0))) {
        uint64_t prevIntegral = ptp->zeroCrossingIntegral;
        ptp->zeroCrossingIntegral = ptp->integral;
        ptp->integral += prevIntegral;
        ptp->integral >>= 1;
      }
    } else {
      newRtcIncrement = (ptp->nominalIncrement.mantissa & RTC_MANTISSA_MASK);
      newRtcIncrement <<= RTC_MANTISSA_SHIFT;
      newRtcIncrement |= (ptp->nominalIncrement.fraction & RTC_FRACTION_MASK);
    }

    /* Operate in two distinct modes; a high-gain, purely-proportional control loop
     * when we're far from the master, and a more complete set of controls once we've
     * narrowed in
     */
    if(ptp->acquiring == PTP_RTC_ACQUIRING) {
      if((slaveOffset > ACQUIRE_THRESHOLD) || (slaveOffset < -ACQUIRE_THRESHOLD)) {
        /* Continue in acquiring mode; accumulate the proportional coefficient's contribution */
        coefficient = (int64_t) ACQUIRE_COEFF_P;
        slaveOffsetExtended = (int64_t) slaveOffset;
        accumulator = ((coefficient * slaveOffsetExtended) >> COEFF_PRODUCT_SHIFT);
#ifdef SLAVE_OFFSET_DEBUG
        if(servoCount >= 10) {
          uint32_t wordChunk;

          wordChunk = (uint32_t) (accumulator >> 32);
          printk("Acquiring, P contribution = 0x%08X", wordChunk);
          wordChunk = (uint32_t) accumulator;
          printk("%08X\n", wordChunk);
        }
#endif
      } else {
        /* Reached the acquisition band */
        ptp->acquiring = PTP_RTC_ACQUIRED;
      }

      /* Also dump the integrator for the integral term */
      ptp->integral = 0;
      ptp->zeroCrossingIntegral = 0;
    }

    /* Now check for "acquired" mode */
    if(ptp->acquiring == PTP_RTC_ACQUIRED) {
      /* We are in the acquisition band; see if we've wandered beyond it badly enough to
       * go back into acquiring mode, producing some hysteresis
       */
      if((slaveOffset > (8 * ACQUIRE_THRESHOLD)) || (slaveOffset < (8 * -ACQUIRE_THRESHOLD))) {
        ptp->acquiring = PTP_RTC_ACQUIRING;
      }

      /* Accumulate the proportional coefficient's contribution */
      slaveOffsetExtended = (int64_t) slaveOffset;
      coefficient = (int64_t) ptp->coefficients.P;
      accumulator = ((coefficient * slaveOffsetExtended) >> COEFF_PRODUCT_SHIFT);

#if 0
      /* Force proportional contribution of at least +- 128 to make sure small proportions still do something when in close */
      if (slaveOffset > 0) {
        accumulator = -128;
      } else {
        accumulator = +128;
      }
#endif

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
      accumulator += ((coefficient * ptp->derivative) >> COEFF_PRODUCT_SHIFT);
      ptp->previousOffset = slaveOffset;

    }

    /* Clamp the new increment to within +/- one nanosecond of nominal */
    if(accumulator > (int64_t)INCREMENT_DELTA_MAX) {
      adjustment = INCREMENT_DELTA_MAX;
    } else if(accumulator < (int64_t)INCREMENT_DELTA_MIN) {
      adjustment = INCREMENT_DELTA_MIN;
    } else {
      adjustment = (int32_t) accumulator;
    }
    newRtcIncrement += adjustment;

    /* Write the new increment out to the hardware, incorporating the enable bit.
     * Suppress the actual write to the RTC increment register if the userspace
     * control has not acknowledged a Grandmaster change.
     */

    if(ptp->rtcChangesAllowed) {
      /* Smooth increments when the changes are small */
      uint32_t tempRtcIncrement = newRtcIncrement;
      if ((ptp->prevAppliedRtcIncrement - newRtcIncrement) < 0x200 ||
          (newRtcIncrement - ptp->prevAppliedRtcIncrement) < 0x200) {
        newRtcIncrement = (newRtcIncrement >> 1) + (ptp->prevAppliedRtcIncrement >> 1);
      }
      ptp->rtcLastIncrementDelta = newRtcIncrement - ptp->prevAppliedRtcIncrement;
      ptp->prevAppliedRtcIncrement = tempRtcIncrement;

      RtcIncrement newIncrement;
      newIncrement.mantissa = (newRtcIncrement >> RTC_MANTISSA_SHIFT) & RTC_MANTISSA_MASK;
      newIncrement.fraction = (newRtcIncrement & RTC_FRACTION_MASK);
      set_rtc_increment(ptp,&newIncrement);
    }

#ifdef SLAVE_OFFSET_DEBUG
    if(servoCount++ >= 10) {
      printk("Slave offset %d\n", slaveOffset);
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
