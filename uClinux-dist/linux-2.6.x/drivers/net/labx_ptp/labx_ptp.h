/*
 *  linux/drivers/net/labx_ptp.h
 *
 *  Lab X Technologies Precision Time Protocol (PTP) driver
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

#ifndef _LABX_PTP_H_
#define _LABX_PTP_H_

#include <linux/cdev.h>
#include <linux/highmem.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <net/labx_ptp/labx_ptp_defs.h>

/* Macros for determining sub-addresses for address ranges and individual registers.
 */
#define REGISTER_RANGE      (0x0)
#define TX_PACKET_RANGE     (0x2)
#define RX_PACKET_RANGE     (0x3)
#define ADDRESS_RANGE_SHIFT (11)

/* Bytes per packet buffer */
#define PTP_MAX_PACKET_BYTES     (256)
#define PTP_PACKET_BUFFER_SHIFT  (8)

/* Packets per ring (these must be a power of 2) */
#define PTP_TX_BUFFER_COUNT      (8)
#define PTP_RX_BUFFER_COUNT      (8)

/* Register file */
#define PTP_RX_REG            (0x000)
#  define PTP_RX_DISABLE      (0x00000000)
#  define PTP_RX_ENABLE       (0x80000000)
#  define PTP_RX_BUFFER_MASK  (PTP_RX_BUFFER_COUNT-1)

#define PTP_TX_REG            (0x001)
#  define PTP_TX_DISABLE          (0x00000000)
#  define PTP_TX_ENABLE           (0x80000000)
#  define PTP_TX_BUSY             (0x40000000)
#  define PTP_TX_BUFFER_MASK      ((1 << PTP_TX_BUFFER_COUNT) - 1)
#  define PTP_TX_BUFFER_NONE      (0x00000000)
#  define PTP_TX_BUFFER(txBuffer) ((0x01 << txBuffer) & PTP_TX_BUFFER_MASK)

#define PTP_IRQ_MASK_REG      (0x002)
#define PTP_IRQ_FLAGS_REG     (0x003)
#  define PTP_NO_IRQS          (0x00000)
#  define PTP_TX_IRQ_MASK      ((1 << PTP_TX_BUFFER_COUNT) - 1)
#  define PTP_TX_IRQ(txBuffer) ((0x01 << txBuffer) & PTP_TX_IRQ_MASK)
#  define PTP_RX_IRQ           (0x1 << PTP_TX_BUFFER_COUNT)
#  define PTP_TIMER_IRQ        (0x2 << PTP_TX_BUFFER_COUNT)

#define PTP_RTC_INC_REG       (0x004)
#  define PTP_RTC_DISABLE      (0x00000000)
#  define PTP_RTC_ENABLE       (0x80000000)
#  define RTC_MANTISSA_MASK   (0x0000000F)
#  define RTC_MANTISSA_SHIFT  (27)
#  define RTC_FRACTION_MASK   (0x07FFFFFF)

#define PTP_SECONDS_HIGH_REG  (0x005)
#  define PTP_RTC_CAPTURE_FLAG  (0x80000000)
#define PTP_SECONDS_LOW_REG   (0x006)
#define PTP_NANOSECONDS_REG   (0x007)

#define PTP_TIMER_REG         (0x008)
#  define PTP_PRESCALER_MASK  (0x00000FFF)
#  define PTP_DIVIDER_MASK    (0x000003FF)
#  define PTP_DIVIDER_SHIFT   (12)

#define PTP_REVISION_REG   (0x0FF)
#  define REVISION_FIELD_BITS  4
#  define REVISION_FIELD_MASK  (0x0F)

#define REGISTER_ADDRESS(device, offset) \
  ((uint32_t)device->virtualAddress |                       \
   (REGISTER_RANGE << ADDRESS_RANGE_SHIFT) | (offset << 2))

#define PTP_TX_PACKET_BUFFER(device, whichBuffer)                       \
  ((uint32_t)device->virtualAddress |                                   \
   (TX_PACKET_RANGE << ADDRESS_RANGE_SHIFT) |                           \
   ((whichBuffer & (PTP_TX_BUFFER_COUNT-1)) << PTP_PACKET_BUFFER_SHIFT))

#define PTP_RX_PACKET_BUFFER(device, whichBuffer)                       \
  ((uint32_t)device->virtualAddress |                                   \
   (RX_PACKET_RANGE << ADDRESS_RANGE_SHIFT) |                           \
   ((whichBuffer & PTP_RX_BUFFER_MASK) << PTP_PACKET_BUFFER_SHIFT))

/* Enumerated type identifying PTP roles */
typedef enum {
  PTP_MASTER  = 0x00,
  PTP_SLAVE   = 0x01,
  PTP_PASSIVE = 0x02
} PtpRole;

/* PTP message type enumeration */
#define MSG_TYPE_MASK        (0x0F)
#  define MSG_SYNC             (0x00)
#  define MSG_DELAY_REQ        (0x01)
#  define MSG_PDELAY_REQ       (0x02)
#  define MSG_PDELAY_RESP      (0x03)
#  define MSG_FUP              (0x08)
#  define MSG_DELAY_RESP       (0x09)
#  define MSG_PDELAY_RESP_FUP  (0x0A)
#  define MSG_ANNOUNCE         (0x0B)
#  define MSG_SIGNALING        (0x0C)
#  define MSG_MANAGEMENT       (0x0D)
#  define PACKET_NOT_PTP       (0xFF)

/* Transmit buffer assignments for each type of message we transmit */
#define PTP_TX_ANNOUNCE_BUFFER         (0)
#define PTP_TX_SYNC_BUFFER             (1)
#define PTP_TX_FUP_BUFFER              (2)
#define PTP_TX_DELAY_REQ_BUFFER        (3)
#define PTP_TX_DELAY_RESP_BUFFER       (4)
#define PTP_TX_PDELAY_REQ_BUFFER       (5)
#define PTP_TX_PDELAY_RESP_BUFFER      (6)
#define PTP_TX_PDELAY_RESP_FUP_BUFFER  (7)

/* Number of bytes in a PTP port ID */
#define PORT_ID_BYTES  (10)

/* Number of fractional nanosecond bits for correction field */
#define CORRECTION_FRACTION_BITS  (16)

/* 802.1AS MDPdelayReq state machine states */
typedef enum { MDPdelayReq_NOT_ENABLED, MDPdelayReq_INITIAL_SEND_PDELAY_REQ,
  MDPdelayReq_RESET, MDPdelayReq_SEND_PDELAY_REQ, MDPdelayReq_WAITING_FOR_PDELAY_RESP,
  MDPdelayReq_WAITING_FOR_PDELAY_RESP_FOLLOW_UP, MDPdelayReq_WAITING_FOR_PDELAY_INTERVAL_TIMER
} MDPdelayReq_State_t;

/* Driver structure to maintain state for each device instance */
#define NAME_MAX_SIZE  (256)
struct ptp_device {
  /* Pointer back to the platform device */
  struct platform_device *pdev;

  /* Character device data */
  struct cdev cdev;
  dev_t       deviceNumber;
  uint32_t    instanceNumber;

  /* Name for use in identification */
  char name[NAME_MAX_SIZE];

  /* Physical and virtual base address */
  uint32_t      physicalAddress;
  uint32_t      addressRangeSize;
  void __iomem  *virtualAddress;

  /* Interrupt request for the device and mask of pending transmit
   * interrupts to respond to
   */
  int32_t irq;
  uint32_t pendingTxFlags;

  /* Properties for the instance */
  PtpProperties properties;

  /* RTC control loop constants */
  RtcIncrement    nominalIncrement;
  PtpCoefficients coefficients;

  /* RTC control loop persistent values */
  int64_t integral;
  int32_t derivative;
  int32_t previousOffset;

  /* Present role and delay mechanism for the endpoint */
  PtpRole presentRole;

  /* Properties for the present grandmaster */
  PtpProperties presentMaster;

  /* Timer state space */
  struct tasklet_struct timerTasklet;
  uint32_t announceCounter;
  uint16_t announceSequenceId;
  uint32_t syncCounter;
  uint16_t syncSequenceId;
  uint32_t delayReqCounter;
  uint32_t delayReqSequenceId;

  /* Packet Rx state space */
  struct tasklet_struct rxTasklet;
  uint32_t lastRxBuffer;
  uint32_t announceTimeoutCounter;
  uint32_t syncSequenceIdValid;

  /* Packet Tx state space */
  struct tasklet_struct txTasklet;

  /* Timing parameters; these consist of raw timestamps for the slave as
   * well as the derived and filtered delay measurements.
   */

  /* End-to-end delay mechanism timing parameters */
  PtpTime syncRxTimestampTemp;
  PtpTime syncRxTimestamp;
  PtpTime syncTxTimestamp;
  uint32_t syncTimestampsValid;
  PtpTime delayReqTxTimestampTemp;
  PtpTime delayReqTxTimestamp;
  PtpTime delayReqRxTimestamp;
  uint32_t delayReqTimestampsValid;

  /* 802.1AS per-port variables (10.2.4) */
  uint32_t asCapable;
  uint32_t neighborRateRatio;
  uint32_t neighborPropDelay;
  uint32_t computeNeighborRateRatio;
  uint32_t computeNeighborPropDelay;
  uint32_t portEnabled;
  uint32_t pttPortEnabled;

  /* 802.1AS MD entity variables (11.2.12) */
  uint32_t pdelayReqInterval;
  uint32_t allowedLostResponses;
  uint32_t isMeasuringDelay;
  uint32_t neighborPropDelayThresh;

  /* 802.1AS Peer-to-peer delay mechanism variables (11.2.15.1) */
  MDPdelayReq_State_t mdPdelayReq_State;

  uint32_t pdelayIntervalTimer;
  uint32_t rcvdPdelayResp;
  uint32_t rcvdPdelayRespPtr;
  uint32_t rcvdPdelayRespFollowUp;
  uint32_t rcvdPdelayRespFollowUpPtr;
  uint32_t rcvdMDTimestampReceive;
  uint32_t pdelayReqSequenceId;
  uint32_t initPdelayRespReceived;
  uint32_t lostResponses;
  uint32_t neighborRateRatioValid;

  /* Current PDelay Request/Response timestamps */
  PtpTime pdelayReqTxTimestamp;  // pdelayReqEventEgressTimestamp (Treq1)
  PtpTime pdelayReqRxTimestamp;  // pdelayReqEventIngressTimestamp (Trsp2)
  PtpTime pdelayRespTxTimestamp; // pdelayRespEventEgressTimestamp (Trsp3)
  PtpTime pdelayRespRxTimestamp; // pdelayRespEventIngressTimestamp (Treq4)

  /* First PDelay Response timestamps (after the last enable/reset) */
  PtpTime pdelayRespTxTimestampI; // pdelayRespEventEgressTimestamp (Trsp3)
  PtpTime pdelayRespRxTimestampI; // pdelayRespEventIngressTimestamp (Treq4)

  /* pdelay response variables */
  uint8_t lastPeerRequestPortId[PORT_ID_BYTES];

  /* Mutex for the device instance */
  spinlock_t mutex;
  bool opened;

  /* Network device event notifier */
  struct notifier_block notifier;
};

/* Enumerated type identifying a packet buffer direction; outgoing or incoming, 
 * respectively.
 */
typedef enum {
  TRANSMITTED_PACKET,
  RECEIVED_PACKET
} PacketDirection;

/* Function prototypes for inter-module calls */

/* From labx_ptp_messages.c */
void init_tx_templates(struct ptp_device *ptp);
uint32_t get_message_type(struct ptp_device *ptp, uint32_t rxBuffer);
void get_rx_mac_address(struct ptp_device *ptp, uint32_t rxBuffer, uint8_t *macAddress);
void get_source_port_id(struct ptp_device *ptp, PacketDirection bufferDirection, uint32_t rxBuffer, uint8_t *sourcePortId);
void get_rx_requesting_port_id(struct ptp_device *ptp, uint32_t rxBuffer, uint8_t *requestingPortId);
void extract_announce(struct ptp_device *ptp, uint32_t rxBuffer, PtpProperties *properties);
void copy_ptp_properties(PtpProperties *to, PtpProperties *from);
int32_t compare_mac_addresses(const uint8_t *macAddressA, const uint8_t *macAddressB);
int32_t compare_clock_identity(const uint8_t *clockIdentityA, const uint8_t *clockIdentityB);
int32_t compare_port_ids(const uint8_t *portIdA, const uint8_t *portIdB);
void transmit_announce(struct ptp_device *ptp);
void transmit_sync(struct ptp_device *ptp);
void transmit_fup(struct ptp_device *ptp);
void transmit_delay_request(struct ptp_device *ptp);
void transmit_delay_response(struct ptp_device *ptp, uint32_t requestRxBuffer);
void transmit_pdelay_request(struct ptp_device *ptp);
void transmit_pdelay_response(struct ptp_device *ptp, uint32_t requestRxBuffer);
void transmit_pdelay_response_fup(struct ptp_device *ptp);
void print_packet_buffer(struct ptp_device *ptp, PacketDirection bufferDirection,
                         uint32_t packetBuffer);
uint16_t get_sequence_id(struct ptp_device *ptp, PacketDirection bufferDirection,
                         uint32_t packetBuffer);
void get_hardware_timestamp(struct ptp_device *ptp, PacketDirection bufferDirection,
                            uint32_t packetBuffer, PtpTime *timestamp);
void get_local_hardware_timestamp(struct ptp_device *ptp, PacketDirection bufferDirection,
                                  uint32_t packetBuffer, PtpTime *timestamp);
void get_timestamp(struct ptp_device *ptp, PacketDirection bufferDirection,
                   uint32_t packetBuffer, PtpTime *timestamp);
void get_correction_field(struct ptp_device *ptp, uint32_t txBuffer, PtpTime *correctionField);

/* From labx_ptp_state.c */
void init_state_machines(struct ptp_device *ptp);

/* From labx_ptp_pdelay_state.c */
void MDPdelayReq_StateMachine(struct ptp_device *ptp);

/* From labx_ptp_rtc.c */
void disable_rtc(struct ptp_device *ptp);
void set_rtc_increment(struct ptp_device *ptp, RtcIncrement *increment);
void get_rtc_time(struct ptp_device *ptp, PtpTime *time);
void set_rtc_time(struct ptp_device *ptp, PtpTime *time);
void rtc_update_servo(struct ptp_device *ptp);

/* From labx_ptp_arithmetic.c */
void timestamp_sum(PtpTime *addend, PtpTime *augend, PtpTime *sum);
void timestamp_difference(PtpTime *minuend, PtpTime *subtrahend, PtpTime *difference);
void timestamp_abs(PtpTime *operand, PtpTime *result);
void timestamp_copy(PtpTime *destination, PtpTime *source);

#endif
