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

/* Enumerated type identifying PTP roles. Defined to match 802.1AS Table 14-5 */
typedef enum {
  PTP_MASTER   = 6,
  PTP_SLAVE    = 9,
  PTP_PASSIVE  = 7,
  PTP_DISABLED = 3
    
} PtpRole;

#define PTP_CLOCK_IDENTITY_BYTES  (8)

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
#define IOC_PTP_STOP_SERVICE    _IO('p', 0x10)
#define IOC_PTP_START_SERVICE   _IO('p', 0x11)

/* Constants defining range and recommended values for RTC lock & unlock
 * detection.  Lock range is specified in nanoseconds and lock time in 
 * milliseconds.
 *
 * PTP_LOCK_RANGE_MAX        - Maximum permissible value for a lock range
 *
 * PTP_DEFAULT_LOCK_RANGE    - Default for lock range.  Slave offsets within
 *                             [-lockRangeNsec, lockRangeNsec] are declared
 *                             as "within lock range" by the RTC control loop.
 *
 * PTP_DEFAULT_LOCK_TIME     - Default for lock time.  The RTC control loop looks
 *                             for the slave offset to remain within the lock range
 *                             for the lock time before making the transition from
 *                             the unlocked to locked state.
 *
 * PTP_DEFAULT_UNLOCK_TIME   - Default for unlock time.  If the slave offset drifts
 *                             outside of the lock range for this amount of time,
 *                             the RTC control loop transitions from a locked to an
 *                             unlocked state.
 *
 * PTP_DEFAULT_UNLOCK_THRESH - Default unlock threshold.  If the slave offset hits
 *                             any value outside the range:
 *                             (-unlockThreshNsec, unlockThreshNsec), the RTC will
 *                             unlock instantly.
 */
#define PTP_LOCK_RANGE_MAX        (250000)
#define PTP_DEFAULT_LOCK_RANGE      (5000)
#define PTP_DEFAULT_LOCK_TIME       (2000)
#define PTP_DEFAULT_UNLOCK_TIME     ( 100)
#define PTP_DEFAULT_UNLOCK_THRESH   (8000)

/* E2E versions are larger to account for the packet jitter. Lock detection
 * uses the instantanious offset, not an average or filtered one. */
#define PTP_DEFAULT_LOCK_RANGE_E2E      (5000)
#define PTP_DEFAULT_LOCK_TIME_E2E       (2000)
#define PTP_DEFAULT_UNLOCK_TIME_E2E     ( 100)
#define PTP_DEFAULT_UNLOCK_THRESH_E2E  (20000)

typedef uint8_t PtpClockIdentity[PTP_CLOCK_IDENTITY_BYTES];

typedef struct {
  /* Various PTP-defined properties */
  uint8_t          domainNumber;
  int16_t          currentUtcOffset;
  uint8_t          grandmasterPriority1;
  PtpClockQuality  grandmasterClockQuality;
  uint8_t          grandmasterPriority2;
  PtpClockIdentity grandmasterIdentity;
  uint8_t          timeSource;
  uint8_t          delayMechanism;
  uint32_t         lockRangeNsec;
  uint32_t         lockTimeMsec;
  uint32_t         unlockThreshNsec;
  uint32_t         unlockTimeMsec;
} PtpProperties;
#define IOC_PTP_GET_PROPERTIES  _IOR('p', 0x12, PtpProperties)
#define IOC_PTP_SET_PROPERTIES  _IOW('p', 0x13, PtpProperties)

typedef struct {
  /* Port number to get/set */
  uint32_t portNumber;

  /* Source MAC address of the interface the PTP hardware uses */
  uint8_t sourceMacAddress[MAC_ADDRESS_BYTES];

  /* Steps removed from the master */
  uint16_t stepsRemoved;

} PtpPortProperties;
#define IOC_PTP_GET_PORT_PROPERTIES  _IOWR('p', 0x14, PtpPortProperties)
#define IOC_PTP_SET_PORT_PROPERTIES  _IOW('p', 0x15, PtpPortProperties)

typedef struct {
  int32_t  secondsUpper;
  uint32_t secondsLower;
  int32_t nanoseconds;
} PtpTime;
#define IOC_PTP_GET_TIME  _IOR('p', 0x16, PtpTime)
#define IOC_PTP_SET_TIME  _IOW('p', 0x17, PtpTime)

typedef struct {
  int32_t P;
  int32_t I;
  int32_t D;
} PtpCoefficients;
#define IOC_PTP_GET_RTC_COEF _IOR('p', 0x18, PtpCoefficients)
#define IOC_PTP_SET_RTC_COEF _IOW('p', 0x19, PtpCoefficients)

/* Type used to represent RTC increments as well as permissible bounds to the
 * nominal increment.  This limits the RTC clock range to [100, 250] MHz, which
 * is a reasonable limitation for good performance without excessive precision.
 */
#define LABX_PTP_RTC_MANTISSA_BITS   (4)
#  define LABX_PTP_RTC_INC_MIN ( 4)
#  define LABX_PTP_RTC_INC_MAX (10)
#define LABX_PTP_RTC_FRACTION_BITS  (27)
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
  /* Number of PTP ports attached to this instance */
  uint32_t numPorts;

  /* Width, in bits, of each of the PTP ports */
  uint32_t portWidth;

  /* Net interface associated with each PTP port */
  const char** interfaceName;

#ifdef CONFIG_OF
  /* OpenFirmware nodes associated with each PTP port */
  struct device_node **interfaceNode;
#endif

  /* Parameters for the monotonic 10 msec event timer */
  uint32_t timerPrescaler;
  uint32_t timerDivider;

  /* Parameters for the RTC servo */
  RtcIncrement    nominalIncrement;
  PtpCoefficients coefficients;
  
  /* Parameters for the MAC/PHY delay */
  PtpTime rxPhyMacDelay;
  PtpTime txPhyMacDelay;

} PtpPlatformData;

/* Default coefficient sets for the two distinct delay mechanisms */
#define DEFAULT_P2P_COEFF_P  (0xFFF00000)
#define DEFAULT_P2P_COEFF_I  (0x80000000)
#define DEFAULT_P2P_COEFF_D  (0x00000000)

#define DEFAULT_E2E_COEFF_P  (0xF8000000)
#define DEFAULT_E2E_COEFF_I  (0xFC000000)
#define DEFAULT_E2E_COEFF_D  (0x00000000)

/*
 * The following structures expose information for MIBs in 802.1AS (draft 7.2)
 */

/* Port status information: ieee802AsPortDataSet Port Parameter Data Set Table 14-6 */
typedef struct {
  uint32_t index; /* Port index to read */

  PtpClockIdentity clockIdentity;          /* 14.6.2 */
  uint16_t portNumber;                     /* 14.6.2 */
  uint32_t portRole;                       /* 14.6.3 - PtpRole */
  uint32_t pttPortEnabled;                 /* 14.6.4 - Boolean */
  uint32_t isMeasuringDelay;               /* 14.6.5 - Boolean */
  uint32_t asCapable;                      /* 14.6.6 - Boolean */
  uint64_t neighborPropDelay;              /* 14.6.7 - UScaledNs */
  uint64_t neighborPropDelayThresh;        /* 14.6.8 - UScaledNs */
  uint64_t delayAsymmetry;                 /* 14.6.9 - ScaledNs */
  uint64_t neighborRateRatio;              /* 14.6.10 */
  uint32_t initialLogAnnounceInterval;     /* 14.6.11 */
  uint32_t currentLogAnnounceInterval;     /* 14.6.12 */
  uint32_t announceReceiptTimeout;         /* 14.6.13 */
  uint32_t initialLogSyncInterval;         /* 14.6.14 */
  uint32_t currentLogSyncInterval;         /* 14.6.15 */
  uint32_t syncReceiptTimeout;             /* 14.6.16 */
  uint32_t syncReceiptTimeoutTimeInterval; /* 14.6.17 */
  uint32_t initialLogPdelayReqInterval;    /* 14.6.18 */
  uint32_t currentLogPdelayReqInterval;    /* 14.6.19 */
  uint32_t allowedLostResponses;           /* 14.6.20 */
  uint32_t versionNumber;                  /* 14.6.21 */
  uint32_t nup;                            /* 14.6.22 */
  uint32_t ndown;                          /* 14.6.23 */
  uint32_t acceptableMasterTableEnabled;   /* 14.6.24 - Boolean */

} PtpAsPortDataSet;
#define IOC_PTP_GET_AS_PORT_DATA_SET _IOWR('p', 0x1a, PtpAsPortDataSet)

/* Port statistics: ieee802AsPortStatistics Port Statistics Data Set Table 14-7 */
typedef struct {
  uint32_t index; /* Port index to read */

  uint32_t rxSyncCount;                             /* 14.7.2 */
  uint32_t rxFollowupCount;                         /* 14.7.3 */
  uint32_t rxPDelayRequestCount;                    /* 14.7.4 */
  uint32_t rxPDelayResponseCount;                   /* 14.7.5 */
  uint32_t rxPDelayResponseFollowupCount;           /* 14.7.6 */
  uint32_t rxAnnounceCount;                         /* 14.7.7 */
  uint32_t rxPTPPacketDiscardCount;                 /* 14.7.8 */
  uint32_t syncReceiptTimeoutCount;                 /* 14.7.9 */
  uint32_t announceReceiptTimeoutCount;             /* 14.7.10 */
  uint32_t pDelayAllowedLostResponsesExceededCount; /* 14.7.11 */
  uint32_t txSyncCount;                             /* 14.7.12 */
  uint32_t txFollowupCount;                         /* 14.7.13 */
  uint32_t txPDelayRequestCount;                    /* 14.7.14 */
  uint32_t txPDelayResponseCount;                   /* 14.7.15 */
  uint32_t txPDelayResponseFollowupCount;           /* 14.7.16 */
  uint32_t txAnnounceCount;                         /* 14.7.17 */

} PtpAsPortStatistics;
#define IOC_PTP_GET_AS_PORT_STATISTICS _IOWR('p', 0x1b, PtpAsPortStatistics)

/* I/O control operation to acknowledge Grandmaster changes */
#define IOC_PTP_ACK_GM_CHANGE _IO('p', 0x1c)

/* PTP events Generic Netlink family name, version, and multicast groups */
#define PTP_EVENTS_FAMILY_NAME     "PTP_EVENTS"
#define PTP_EVENTS_FAMILY_VERSION  1
#define PTP_EVENTS_RTC_GROUP       "RtcGroup"

/* Netlink family attributes */
enum {
  PTP_EVENTS_A_UNSPEC,
  PTP_EVENTS_A_DOMAIN,
  PTP_EVENTS_A_VALUEMAP,
  __PTP_EVENTS_A_MAX,
};
#define PTP_EVENTS_A_MAX (__PTP_EVENTS_A_MAX - 1)

/* Constant enumeration for Netlink event commands from the PTP driver */
enum {
  PTP_EVENTS_C_UNSPEC,
  PTP_EVENTS_C_HEARTBEAT,
  PTP_EVENTS_C_GM_CHANGE,
  PTP_EVENTS_C_RTC_LOCK,
  PTP_EVENTS_C_RTC_UNLOCK,
  __PTP_EVENTS_C_MAX,
};
#define PTP_EVENTS_C_MAX (__PTP_EVENTS_C_MAX - 1)

/* Constant enumeration defining the structure of a key / value map */
enum {
  PTP_VALUEMAP_A_LENGTH,
  PTP_VALUEMAP_A_PAIRS,
  __PTP_VALUEMAP_A_MAX
};
#define PTP_VALUEMAP_A_MAX (__PTP_VALUEMAP_A_MAX - 1)

/* Maximum number of key / value pair strings that can occur in a value map */
#define PTP_VALUEMAP_MAX_PAIRS (16)

#endif /* _LABX_PTP_DEFS_H_ */

