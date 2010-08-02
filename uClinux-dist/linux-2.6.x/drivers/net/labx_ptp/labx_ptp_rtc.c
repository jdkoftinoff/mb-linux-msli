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

/* Disables the RTC */
void disable_rtc(struct ptp_device *ptp) {
  XIo_Out32(REGISTER_ADDRESS(ptp, PTP_RTC_INC_REG), PTP_RTC_DISABLE);
}

/* Sets the RTC increment, simultaneously enabling the RTC */
void set_rtc_increment(struct ptp_device *ptp, RtcIncrement *increment) {
  uint32_t incrementWord;

  /* Assemble a single value from the increment components */
  incrementWord = ((increment->mantissa & RTC_MANTISSA_MASK) << RTC_MANTISSA_SHIFT);
  incrementWord |= (increment->fraction & RTC_FRACTION_MASK);
  incrementWord |= PTP_RTC_ENABLE;

  /* The actual write is already atomic, so no need to ensure mutual exclusion */
  XIo_Out32(REGISTER_ADDRESS(ptp, PTP_RTC_INC_REG), incrementWord);
}

/* Captures the present RTC time, returning it into the passed structure */
void get_rtc_time(struct ptp_device *ptp, PtpTime *time) {
  uint32_t timeWord;
  uint32_t flags;

  /* Write to the capture flag in the upper seconds word to initiate a capture,
   * then poll the same bit to determine when it has completed.  The capture only
   * takes a few RTC clocks, so this busy wait can only consume tens of nanoseconds.
   *
   * This will *not* modify the time, since we don't write the nanoseconds register.
   */
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  XIo_Out32(REGISTER_ADDRESS(ptp, PTP_SECONDS_HIGH_REG), PTP_RTC_CAPTURE_FLAG);
  do {
    timeWord = XIo_In32(REGISTER_ADDRESS(ptp, PTP_SECONDS_HIGH_REG));
  } while((timeWord & PTP_RTC_CAPTURE_FLAG) != 0);

  /* Now read the entire captured time and pack it into the structure.  The last
   * value read during polling is perfectly valid.
   */
  time->secondsUpper = (uint16_t) timeWord;
  time->secondsLower = XIo_In32(REGISTER_ADDRESS(ptp, PTP_SECONDS_LOW_REG));
  time->nanoseconds = XIo_In32(REGISTER_ADDRESS(ptp, PTP_NANOSECONDS_REG));
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();
}

/* Sets a new RTC time from the passed structure */
void set_rtc_time(struct ptp_device *ptp, PtpTime *time) {
  uint32_t flags;

  /* Write to the time register, beginning with the seconds.  The write to the 
   * nanoseconds register is what actually effects the change to the RTC.
   */
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  XIo_Out32(REGISTER_ADDRESS(ptp, PTP_SECONDS_HIGH_REG), time->secondsUpper);
  XIo_Out32(REGISTER_ADDRESS(ptp, PTP_SECONDS_LOW_REG), time->secondsLower);
  XIo_Out32(REGISTER_ADDRESS(ptp, PTP_NANOSECONDS_REG), time->nanoseconds);
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();
}

/* RTC increment constants */
#define INCREMENT_ONE            ((int32_t) 0x08000000)
#define INCREMENT_NEG_ONE        ((int32_t) 0xF8000000)

/* Bits to shift to convert a coefficient product */
#define COEFF_PRODUCT_SHIFT  (28)

/* Updates the RTC servo when a slave */
#ifdef SLAVE_OFFSET_DEBUG
static uint32_t servoCount = 0;
#endif

void rtc_update_servo(struct ptp_device *ptp) {
  int32_t slaveOffset;
  uint32_t slaveOffsetValid = 0;
  PtpTime difference;

  /* Update the servo using the appropriate delay mechanism */
  if(ptp->properties.delayMechanism == PTP_DELAY_MECHANISM_E2E) {
    /* Make certain there are both sets of valid timestamps available for a master->
     * slave offset calculation employing the end-to-end mechanism.
     */
    if(ptp->syncTimestampsValid && ptp->delayReqTimestampsValid) {
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
      timestamp_difference(&ptp->syncRxTimestamp, &ptp->syncTxTimestamp, &difference);
      timestamp_difference(&ptp->delayReqTxTimestamp, &ptp->delayReqRxTimestamp, &difference2);
      
      /* The fact that this is called at all implies there's a < 1 sec slaveOffset; deal
       * strictly with nanoseconds now that the seconds have been normalized.
       */
      slaveOffset = (((int32_t) difference.nanoseconds) + ((int32_t) difference2.nanoseconds));
      slaveOffset >>= 1;
      slaveOffsetValid = 1;
    }
  } else {
    /* The peer delay mechanism uses the SYNC->FUP messages, but relies upon the
     * messages having had their correction field updated by bridge residence time
     * logic along the way.  Since that is performed, the only remaining correction
     * to be made is to subtract out the path delay to our peer, which is periodically
     * calculated (and should be pretty small.)
     */
    timestamp_difference(&ptp->syncRxTimestamp, &ptp->syncTxTimestamp, &difference);
      
    /* The fact that this is called at all implies there's a < 1 sec slaveOffset; deal
     * strictly with nanoseconds now that the seconds have been normalized.
     */
    slaveOffset = (((int32_t) difference.nanoseconds) - ptp->neighborPropDelay);
    slaveOffsetValid = 1;
  }

  /* Perform the actual servo update if the slave offset is valid */
  if(slaveOffsetValid) {
    uint32_t newRtcIncrement;
    int64_t coefficient;
    int64_t slaveOffsetExtended;
    int64_t accumulator;
    int32_t adjustment;

    /* Update the servo with the present value */
    newRtcIncrement = (ptp->nominalIncrement.mantissa & RTC_MANTISSA_MASK);
    newRtcIncrement <<= RTC_MANTISSA_SHIFT;
    newRtcIncrement |= (ptp->nominalIncrement.fraction & RTC_FRACTION_MASK);
    
    /* Accumulate the proportional coefficient's contribution */
    coefficient = (int64_t) ptp->coefficients.P;
    slaveOffsetExtended = (int64_t) slaveOffset;
    accumulator = ((coefficient * slaveOffsetExtended) >> COEFF_PRODUCT_SHIFT);

    /* Accumulate the integral coefficient's contribution */
    coefficient = (int64_t) ptp->coefficients.I;
    ptp->integral += slaveOffsetExtended; /* TODO: Scale based on the time between syncs? */
    accumulator += ((coefficient * ptp->integral) >> COEFF_PRODUCT_SHIFT);

    /* Accumulate the derivitave coefficient's contribution */
    coefficient = (int64_t) ptp->coefficients.D;
    ptp->derivative += (slaveOffset - ptp->previousOffset); /* TODO: Scale based on the time between syncs? */
    accumulator += ((coefficient * (int64_t)ptp->derivative) >> COEFF_PRODUCT_SHIFT);
    ptp->previousOffset = slaveOffset;

    adjustment = (int32_t) accumulator;
    
    /* Clamp the new increment to within +/- one nanosecond of nominal */
    if(adjustment > INCREMENT_ONE) {
      adjustment = INCREMENT_ONE;
    } else if(adjustment < INCREMENT_NEG_ONE) {
      adjustment = INCREMENT_NEG_ONE;
    }
    newRtcIncrement += adjustment;
    
    /* Write the new increment out to the hardware, incorporating the enable bit */
    newRtcIncrement |= PTP_RTC_ENABLE;
    XIo_Out32(REGISTER_ADDRESS(ptp, PTP_RTC_INC_REG), newRtcIncrement);

#ifdef SLAVE_OFFSET_DEBUG
    if(++servoCount >= 10) {
      printk("Slave offset %d, Increment 0x%08X\n", slaveOffset, newRtcIncrement);
      printk("  syncRxNS %d, syncTxNS %d (%d), MeanPathNS %d\n", (int)ptp->syncRxTimestamp.nanoseconds,
        (int)ptp->syncTxTimestamp.nanoseconds, (int)difference.nanoseconds, (int)ptp->neighborPropDelay);
      servoCount = 0;
    }
#endif
  } /* if(slaveOffsetValid) */
}
