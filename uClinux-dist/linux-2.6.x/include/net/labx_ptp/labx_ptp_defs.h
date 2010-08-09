/*
 *  linux/include/net/labx_ptp/labx_ptp_defs.h
 *
 *  Lab X Technologies PTP driver definitions
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

#ifndef _LABX_PTP_DEFS_H_
#define _LABX_PTP_DEFS_H_

#include <linux/types.h>

/* Bytes per MAC address */
#define MAC_ADDRESS_BYTES  (6)

/* PTP data types and constant definitions */
typedef struct {
  uint8_t  clockClass;
  uint8_t  clockAccuracy;
  uint16_t offsetScaledLogVariance;
} PtpClockQuality;

#define PTP_CLOCK_IDENTITY_CHARS  (8)

/* timeSource enumeration */
#define PTP_SOURCE_ATOMIC_CLOCK         (0x10)
#define PTP_SOURCE_GPS                  (0x20)
#define PTP_SOURCE_TERRESTRIAL_RADIO    (0x30)
#define PTP_SOURCE_PTP                  (0x40)
#define PTP_SOURCE_NTP                  (0x50)
#define PTP_SOURCE_HAND_SET             (0x60)
#define PTP_SOURCE_OTHER                (0x90)
#define PTP_SOURCE_INTERNAL_OSCILLATOR  (0xA0)

/* delayMechanism enumeration */
#define PTP_DELAY_MECHANISM_E2E       (0x01)
#define PTP_DELAY_MECHANISM_P2P       (0x02)
#define PTP_DELAY_MECHANISM_DISABLED  (0xFE)

/* I/O control commands and structures for the PTP driver */
#define IOC_PTP_STOP_SERVICE    (0x10)
#define IOC_PTP_START_SERVICE   (0x11)
#define IOC_PTP_GET_PROPERTIES  (0x12)
#define IOC_PTP_SET_PROPERTIES  (0x13)

typedef struct {
  /* Source MAC address of the interface the PTP hardware uses */
  uint8_t sourceMacAddress[MAC_ADDRESS_BYTES];

  /* Various PTP-defined properties */
  uint8_t         domainNumber;
  int16_t         currentUtcOffset;
  uint8_t         grandmasterPriority1;
  PtpClockQuality grandmasterClockQuality;
  uint8_t         grandmasterPriority2;
  uint8_t         grandmasterIdentity[PTP_CLOCK_IDENTITY_CHARS];
  uint8_t         timeSource;
  uint8_t         delayMechanism;
} PtpProperties;

#define IOC_PTP_GET_TIME  (0x14)
#define IOC_PTP_SET_TIME  (0x15)
typedef struct {
  int32_t  secondsUpper;
  uint32_t secondsLower;
  int32_t nanoseconds;
} PtpTime;

#define IOC_PTP_GET_RTC_COEF (0x16)
#define IOC_PTP_SET_RTC_COEF (0x17)
typedef struct {
  int32_t P;
  int32_t I;
  int32_t D;
} PtpCoefficients;

/* Type used to represent RTC increments */
#define LABX_PTP_RTC_MANTISSA_BITS   (4)
#define LABX_PTP_RTC_FRACTION_BITS  (20)
typedef struct {
  uint32_t mantissa;
  uint32_t fraction;
} RtcIncrement;

/* Number of bits in the timer prescaler and divider, respectively */
#define LABX_PTP_TIMER_PRESCALER_BITS  (12)
#define LABX_PTP_TIMER_DIVIDER_BITS    (10)

/* Platform data structure for configuring an instance with board-specific
 * software parameters.  The RTC coefficients are signed, fully-fractional values;
 * that is, a full-scale -1.0 value is 0x80000000.
 * Proportional coefficients must be negative to converge.
 */
typedef struct {
  /* Parameters for the monotonic 10 msec event timer */
  uint32_t timerPrescaler;
  uint32_t timerDivider;

  /* Parameters for the RTC servo */
  RtcIncrement    nominalIncrement;
  PtpCoefficients coefficients;
} PtpPlatformData;

#endif
