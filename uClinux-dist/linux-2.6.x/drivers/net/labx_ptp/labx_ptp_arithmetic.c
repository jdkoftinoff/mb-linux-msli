/*
 *  linux/drivers/net/labx_ptp_arithmetic.c
 *
 *  Lab X Technologies Precision Time Protocol (PTP) driver
 *  PTP timestamp arithmetic
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *
 *  Copyright (C) 2010 Lab X Technologies LLC, All Rights Reserved.
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

/*
 * The 80-bit timestamps used by PTP are inherently unsigned.  However, the
 * upper 16 bits of the seconds are internally represented as a signed 32-bit
 * number.  These routines treat the entire structure as a single, 96-bit
 * two's-complement number.
 */

#define ONE_BILLION  (0x3B9ACA00)

/* Normalizes the sign of the two components of a timestamp */
static void normalize_timestamp(PtpTime *operand) {
  if(operand->secondsUpper < 0) {
    if(operand->nanoseconds > 0) {
      if(operand->secondsLower++ == 0xFFFFFFFF) operand->secondsUpper++;
      operand->nanoseconds -= ONE_BILLION;
    }
  } else if((operand->secondsUpper > 0) || (operand->secondsLower > 0)) {
    if(operand->nanoseconds < 0) {
      if(operand->secondsLower-- == 0x00000000) operand->secondsUpper--;
      operand->nanoseconds += ONE_BILLION;
    }
  }
}

/* Adds two timestamps */
void timestamp_sum(PtpTime *addend, PtpTime *augend, PtpTime *sum) {
  sum->secondsUpper = (addend->secondsUpper + augend->secondsUpper);
  sum->secondsLower = (addend->secondsLower + augend->secondsLower);
  if(sum->secondsLower < addend->secondsLower) sum->secondsUpper++;
  sum->nanoseconds = (addend->nanoseconds + augend->nanoseconds);
  if(sum->nanoseconds >= ONE_BILLION) {
    if(sum->secondsLower == 0xFFFFFFFF) sum->secondsUpper++;
    sum->secondsLower++;
    sum->nanoseconds -= ONE_BILLION;
  } else if(sum->nanoseconds <= -ONE_BILLION) {
    if(sum->secondsLower == 0x00000000) sum->secondsUpper--;
    sum->secondsLower--;
    sum->nanoseconds += ONE_BILLION;
  }
  normalize_timestamp(sum);
}

/* Subtracts two timestamps */
void timestamp_difference(PtpTime *minuend, PtpTime *subtrahend, PtpTime *difference) {
  difference->secondsUpper = (minuend->secondsUpper - subtrahend->secondsUpper);
  difference->secondsLower = (minuend->secondsLower - subtrahend->secondsLower);
  difference->nanoseconds = (minuend->nanoseconds - subtrahend->nanoseconds);
  if(subtrahend->secondsLower > minuend->secondsLower) difference->secondsUpper--;
  if(difference->nanoseconds >= ONE_BILLION) {
    if(difference->secondsLower == 0xFFFFFFFF) difference->secondsUpper++;
    difference->secondsLower++;
    difference->nanoseconds -= ONE_BILLION;
  } else if(difference->nanoseconds <= -ONE_BILLION) {
    if(difference->secondsLower == 0x00000000) difference->secondsUpper--;
    difference->secondsLower--;
    difference->nanoseconds += ONE_BILLION;
  }
  normalize_timestamp(difference);
}

/* Computes the absolute value of a timestamp */
void timestamp_abs(PtpTime *operand, PtpTime *result) {
  /* Copy to the result first */
  timestamp_copy(result, operand);

  /* The value has been normalized; simply do an absolute value on both fields */
  if(result->secondsUpper < 0) {
    /* Negative seconds */
    result->secondsUpper = ~result->secondsUpper;
    result->secondsLower = ~result->secondsLower;
    if(result->secondsLower++ == 0xFFFFFFFF) result->secondsUpper++;
  }
  if(result->nanoseconds < 0) result->nanoseconds = (0 - result->nanoseconds);
}

/* Copies one timestamp to another */
void timestamp_copy(PtpTime *destination, PtpTime *source) {
  destination->secondsUpper = source->secondsUpper;
  destination->secondsLower = source->secondsLower;
  destination->nanoseconds = source->nanoseconds;
}
