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

#include <linux/if.h>
#include <linux/cdev.h>
#include <linux/highmem.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <net/labx_ptp/labx_ptp_defs.h>

/* Name of the driver for use by all the modules */
#define DRIVER_NAME "labx_ptp"

/* Macros for determining sub-addresses for address ranges and individual registers.
 */
#define REGISTER_RANGE      (0x0)
#define TX_PACKET_RANGE     (0x2)
#define RX_PACKET_RANGE     (0x3)
#define ADDRESS_RANGE_SHIFT (11)
#define PORT_RANGE_SHIFT    (13)

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

#define PTP_LOCAL_SECONDS_HIGH_REG  (0x009)
#  define PTP_RTC_LOCAL_CAPTURE_FLAG  (0x80000000)
#define PTP_LOCAL_SECONDS_LOW_REG   (0x00A)
#define PTP_LOCAL_NANOSECONDS_REG   (0x00B)

#define PTP_REVISION_REG   (0x0FF)
#  define REVISION_FIELD_BITS  4
#  define REVISION_FIELD_MASK  (0x0F)

#define REGISTER_ADDRESS(device, port, offset) \
  ((uintptr_t)device->virtualAddress | (port << PORT_RANGE_SHIFT) |      \
   (REGISTER_RANGE << ADDRESS_RANGE_SHIFT) | (offset << 2))

#define PTP_TX_PACKET_BUFFER(device, port, whichBuffer)                 \
  ((uintptr_t)device->virtualAddress | (port << PORT_RANGE_SHIFT) |      \
   (TX_PACKET_RANGE << ADDRESS_RANGE_SHIFT) |                           \
   ((whichBuffer & (PTP_TX_BUFFER_COUNT-1)) << PTP_PACKET_BUFFER_SHIFT))

#define PTP_RX_PACKET_BUFFER(device, port, whichBuffer)                 \
  ((uintptr_t)device->virtualAddress | (port << PORT_RANGE_SHIFT) |      \
   (RX_PACKET_RANGE << ADDRESS_RANGE_SHIFT) |                           \
   ((whichBuffer & PTP_RX_BUFFER_MASK) << PTP_PACKET_BUFFER_SHIFT))

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

/* Length, in bytes, of each packet type we transmit */
#define PTP_ANNOUNCE_LENGTH         (64)
#define PTP_SYNC_LENGTH             (44)
#define PTP_FUP_LENGTH              (44)
#define PTP_DELAY_REQ_LENGTH        (44)
#define PTP_DELAY_RESP_LENGTH       (54)
#define PTP_PDELAY_REQ_LENGTH       (54)
#define PTP_PDELAY_RESP_LENGTH      (54)
#define PTP_PDELAY_RESP_FUP_LENGTH  (54)

/* Number of bytes in a PTP port ID */
#define PORT_ID_BYTES  (10)

/* Number of fractional nanosecond bits for correction field */
#define CORRECTION_FRACTION_BITS  (16)

/* Enumeration for RTC lock state */
#define PTP_RTC_UNLOCKED (0)
#define PTP_RTC_LOCKED   (1)

/* Enumeration for RTC acquisition */
#define PTP_RTC_ACQUIRED  (0)
#define PTP_RTC_ACQUIRING (1)

/* Enumeration for flagging valid RTC offsets */
#define PTP_RTC_OFFSET_INVALID (0)
#define PTP_RTC_OFFSET_VALID   (1)

/* Period, in msec., of the hardware timer tick governing the PTP state machines */
#define PTP_TIMER_TICK_MS (10)

/* Ethernet header definitions */
#define LTF_MASK          (0x0FFFF)
#define ETH_HEADER_BYTES  (14)

/* PTP header definitions */
#define TS_SPEC_ETH_AVB    (0x01)
#define PTP_VERSION_2_0      (0x02)
#define MSG_LENGTH_MASK      (0x0FFFF)
#define DOMAIN_NUM_MASK      (0x0FF)
#define FLAGS_FIELD_MASK     (0x0FFFF)
#  define FLAG_SECURITY       (0x8000)
#  define FLAG_PROF_SPEC_2    (0x4000)
#  define FLAG_PROF_SPEC_1    (0x2000)
#  define FLAG_UNICAST        (0x0400)
#  define FLAG_TWO_STEP       (0x0200)
#  define FLAG_ALT_MASTER     (0x0100)
#  define FLAG_FREQ_TRACE     (0x0020)
#  define FLAG_TIME_TRACE     (0x0010)
#  define FLAG_PTP_TIMESCALE  (0x0008)
#  define FLAG_UTC_OFF_VALID  (0x0004)
#  define FLAG_LEAP_59        (0x0002)
#  define FLAG_LEAP_61        (0x0001)

/* Number of words comprising a hardware timestamp (transmit or receive) */
#define HW_TIMESTAMP_WORDS  (3)

/* Word-aligned offsets for fields which are dynamically changed or
 * inspected upon reception
 */
#define PACKET_BUFFER_WORDS                  (PTP_MAX_PACKET_BYTES / BYTES_PER_WORD)
#define SOURCE_MAC_OFFSET                    ( 1 * BYTES_PER_WORD)
#define MESSAGE_TYPE_OFFSET                  ( 3 * BYTES_PER_WORD)
#define DOMAIN_NUMBER_OFFSET                 ( 4 * BYTES_PER_WORD)
#define CORRECTION_FIELD_OFFSET              ( 5 * BYTES_PER_WORD)
#define SOURCE_PORT_ID_OFFSET                ( 8 * BYTES_PER_WORD)
#define SEQUENCE_ID_OFFSET                   (11 * BYTES_PER_WORD)
#define TIMESTAMP_OFFSET                     (12 * BYTES_PER_WORD)
#define UTC_OFFSET_OFFSET                    (14 * BYTES_PER_WORD)
#define REQ_PORT_ID_OFFSET                   (14 * BYTES_PER_WORD)
#define GM_PRIORITY1_OFFSET                  (15 * BYTES_PER_WORD)
#define CUMULATIVE_SCALED_RATE_OFFSET_OFFSET (17 * BYTES_PER_WORD)
#define LINK_DELAY_INTERVAL_OFFSET           (17 * BYTES_PER_WORD)

/* Port-width-specific offsets for timestamp words in the buffers;
 * the data alignment from the network side to the host interface
 * necessitates skipping words in the block RAMs.
 */
#define HW_TIMESTAMP_OFFSET_X8        ((PACKET_BUFFER_WORDS - HW_TIMESTAMP_WORDS) * BYTES_PER_WORD)
#define HW_LOCAL_TIMESTAMP_OFFSET_X8  (HW_TIMESTAMP_OFFSET_X8 - (HW_TIMESTAMP_WORDS * BYTES_PER_WORD))
#define HW_TIMESTAMP_OFFSET_X64       ((PACKET_BUFFER_WORDS - (2 * HW_TIMESTAMP_WORDS)) * BYTES_PER_WORD)
#define HW_LOCAL_TIMESTAMP_OFFSET_X64 (HW_TIMESTAMP_OFFSET_X64 - (2 * HW_TIMESTAMP_WORDS * BYTES_PER_WORD))

/* Additional offset applied for the transmit buffers, since the first
 * word holds the packet length (minus one.)  Additionally, the second
 * word must be skipped for 64-bit port instances.
 */
#define TX_LENGTH_OFFSET   (0 * BYTES_PER_WORD)
#define TX_DATA_OFFSET_X8  (1 * BYTES_PER_WORD)
#define TX_DATA_OFFSET_X64 (2 * BYTES_PER_WORD)

#define TX_DATA_OFFSET(ptp) ((ptp->portWidth == 8) ? TX_DATA_OFFSET_X8 : TX_DATA_OFFSET_X64)

/* Amount to be decremented from a packet length to form the length word
 * expected by the transmit hardware as the first word of each Tx buffer
 */
#define TX_LENGTH_SUB_X8  (1)
#define TX_LENGTH_SUB_X64 (8)

#define TX_LENGTH_SUB(ptp) ((ptp->portWidth == 8) ? TX_LENGTH_SUB_X8 : TX_LENGTH_SUB_X64)

/* TLV Types */
#define ORGANIZATION_EXTENSION_TLV_TYPE 0x0003
#define PATH_TRACE_TLV_TYPE             0x0008

/* TLV Sizes */
#define TLV_HEADER_LENGTH                4
#define PATH_TRACE_TLV_LENGTH(n)         (8*(n))
#define FOLLOW_UP_INFORMATION_TLV_LENGTH 28

/* 802.1AS MDPdelayReq state machine states */
typedef enum { MDPdelayReq_NOT_ENABLED, MDPdelayReq_INITIAL_SEND_PDELAY_REQ,
  MDPdelayReq_RESET, MDPdelayReq_SEND_PDELAY_REQ, MDPdelayReq_WAITING_FOR_PDELAY_RESP,
  MDPdelayReq_WAITING_FOR_PDELAY_RESP_FOLLOW_UP, MDPdelayReq_WAITING_FOR_PDELAY_INTERVAL_TIMER
} MDPdelayReq_State_t;

/* 802.1AS LinkDelaySyncIntervalSettings state machine states */
typedef enum { LinkDelaySyncIntervalSetting_NOT_ENABLED, LinkDelaySyncIntervalSetting_INITIALIZE,
  LinkDelaySyncIntervalSetting_SET_INTERVALS
} LinkDelaySyncIntervalSetting_State_t;

#define SIGNED_SHIFT(a, b) (((b) >= 0) ? ((a)<<(b)) : ((a)>>(-b)))
#define ANNOUNCE_INTERVAL_TICKS(ptp, port)   \
  SIGNED_SHIFT((1000/PTP_TIMER_TICK_MS), ((ptp)->ports[(port)].currentLogAnnounceInterval))
#define SYNC_INTERVAL_TICKS(ptp, port)       \
  SIGNED_SHIFT((1000/PTP_TIMER_TICK_MS), ((ptp)->ports[(port)].currentLogSyncInterval))
#define PDELAY_REQ_INTERVAL_TICKS(ptp, port) \
  SIGNED_SHIFT((1000/PTP_TIMER_TICK_MS), ((ptp)->ports[(port)].currentLogPdelayReqInterval))

struct ptp_port {

  /* Net interface name associated with this port */
  char interfaceName[IFNAMSIZ];
#ifdef CONFIG_OF
  struct device_node *interfaceNode;
#endif

  /* Port properties */
  PtpPortProperties portProperties;

  /* Timing parameters; these consist of raw timestamps for the slave as
   * well as the derived and filtered delay measurements.
   */

  /* End-to-end delay mechanism timing parameters */
  PtpTime syncRxTimestampTemp;
  PtpTime syncRxTimestamp;
  PtpTime syncTxTimestamp;
  uint32_t syncTimestampsValid;
  PtpTime delayReqTxTimestampTemp;
  PtpTime delayReqTxLocalTimestampTemp;
  PtpTime delayReqTxTimestamp;
  PtpTime delayReqTxLocalTimestamp;
  PtpTime delayReqRxTimestamp;
  uint32_t delayReqTimestampsValid;

  /* Configured delay for the PHY/MAC to where timestamping actually happens */
  PtpTime rxPhyMacDelay;
  PtpTime txPhyMacDelay;

  /* Mask of pending transmit interrupts to respond to */
  uint32_t pendingTxFlags;

  /* 802.1AS per-port variables (10.2.4) - Time sync state machines */
  uint32_t asCapable;
  int8_t   currentLogSyncInterval;
  int8_t   initialLogSyncInterval;
  uint32_t neighborRateRatio;
  uint32_t neighborPropDelay;
  uint32_t computeNeighborRateRatio;
  uint32_t computeNeighborPropDelay;
  uint32_t portEnabled;
  uint32_t pttPortEnabled;

  /* 802.1AS per-port variables (10.3.9) - BMCA/Announce state machines */
  int8_t currentLogAnnounceInterval;
  int8_t initialLogAnnounceInterval;

  /* 802.1AS timeouts (10.6.3) */
  uint8_t syncReceiptTimeout;
  uint8_t announceReceiptTimeout;

  /* 802.1AS MD entity variables (11.2.12) */
  int8_t currentLogPdelayReqInterval;
  int8_t initialLogPdelayReqInterval;
  uint32_t allowedLostResponses;
  uint32_t isMeasuringDelay;
  uint32_t neighborPropDelayThresh;

  /* 802.1AS Peer-to-peer delay mechanism variables (11.2.15.1) */
  MDPdelayReq_State_t mdPdelayReq_State;

  uint32_t pdelayIntervalTimer;
  uint32_t rcvdPdelayResp;
  uint8_t *rcvdPdelayRespPtr;
  uint32_t rcvdPdelayRespFollowUp;
  uint8_t *rcvdPdelayRespFollowUpPtr;
  uint32_t rcvdMDTimestampReceive;
  uint32_t pdelayReqSequenceId;
  uint32_t initPdelayRespReceived;
  uint32_t lostResponses;
  uint32_t neighborRateRatioValid;

  /* 802.1AS LinkDelaySyncIntervalSetting variables (11.2.17.1) */
  LinkDelaySyncIntervalSetting_State_t linkDelaySyncIntervalSetting_State;
  uint32_t rcvdSignalingMsg1;
  uint8_t *rcvdSignalingPtr;

  // 802.1AS Follow_Up information TLV variables (11.4.4.3)
  uint32_t cumulativeScaledRateOffset;

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

  /* Timer state space */
  uint32_t announceCounter;
  uint16_t announceSequenceId;
  uint32_t syncCounter;
  uint16_t syncSequenceId;
  uint32_t delayReqCounter;
  uint32_t delayReqSequenceId;

  /* Packet Rx state space */
  uint32_t lastRxBuffer;
  uint32_t syncSequenceIdValid;

  /* Packet statistics */
  PtpAsPortStatistics stats;
};

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

  /* Interrupt request for the device */
  int32_t irq;

  /* Number of ports attached to this instance */
  uint32_t numPorts;

  /* Width, in bits, of the instance's ports */
  uint32_t portWidth;

  /* Properties for the instance */
  PtpProperties properties;

  /* RTC control loop constants */
  RtcIncrement    nominalIncrement;
  PtpCoefficients coefficients;
  uint32_t masterRateRatio;
  uint32_t masterRateRatioValid;

  /* RTC control loop persistent values */
  int64_t  integral;
  int32_t  derivative;
  int32_t  previousOffset;
  uint32_t rtcChangesAllowed;
  int32_t  rtcLastOffset;
  uint32_t rtcLastOffsetValid;
  uint32_t rtcLastLockState;
  uint32_t acquiring;
  uint32_t rtcLockState;
  uint32_t rtcLockCounter;
  uint32_t rtcLockTicks;
  uint32_t rtcUnlockTicks;
  RtcIncrement currentIncrement;

  /* Present role and delay mechanism for the endpoint */
  PtpRole presentRole;

  /* Properties for the present grandmaster */
  PtpProperties presentMaster;
  PtpPortProperties presentMasterPort;
  uint32_t newMaster;

  /* Timer state space */
  struct tasklet_struct timerTasklet;
  uint32_t heartbeatCounter;
  uint32_t netlinkSequence;

  /* Packet Rx state space */
  struct tasklet_struct rxTasklet;
  uint32_t announceTimeoutCounter;
  uint32_t slaveDebugCounter;

  /* Packet Tx state space */
  struct tasklet_struct txTasklet;

  /* Per-port data */
  struct ptp_port *ports;

  /* Mutex for the device instance */
  spinlock_t mutex;
  uint32_t opened;

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
void init_tx_templates(struct ptp_device *ptp, uint32_t port);
uint32_t get_message_type(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer);
void get_rx_mac_address(struct ptp_device *ptp, uint32_t port, uint8_t * rxBuffer, uint8_t *macAddress);
void get_source_port_id(struct ptp_device *ptp, uint32_t port, PacketDirection bufferDirection, uint8_t * rxBuffer, uint8_t *sourcePortId);
void get_rx_requesting_port_id(struct ptp_device *ptp, uint32_t port, uint8_t * rxBuffer, uint8_t *requestingPortId);
void extract_announce(struct ptp_device *ptp, uint32_t port, uint8_t * rxBuffer, PtpProperties *properties, PtpPortProperties *portProperties);
void copy_ptp_properties(PtpProperties *to, PtpProperties *from);
void copy_ptp_port_properties(PtpPortProperties *to, PtpPortProperties *from);
int32_t compare_mac_addresses(const uint8_t *macAddressA, const uint8_t *macAddressB);
int32_t compare_clock_identity(const uint8_t *clockIdentityA, const uint8_t *clockIdentityB);
int32_t compare_port_ids(const uint8_t *portIdA, const uint8_t *portIdB);
void transmit_announce(struct ptp_device *ptp, uint32_t port);
void transmit_sync(struct ptp_device *ptp, uint32_t port);
void transmit_fup(struct ptp_device *ptp, uint32_t port);
void transmit_delay_request(struct ptp_device *ptp, uint32_t port);
void transmit_delay_response(struct ptp_device *ptp, uint32_t port, uint8_t * requestRxBuffer);
void transmit_pdelay_request(struct ptp_device *ptp, uint32_t port);
void transmit_pdelay_response(struct ptp_device *ptp, uint32_t port, uint8_t *requestRxBuffer);
void transmit_pdelay_response_fup(struct ptp_device *ptp, uint32_t port);
void print_packet_buffer(struct ptp_device *ptp, uint32_t port, PacketDirection bufferDirection,
                         uint8_t * packetBuffer, uint32_t packetWords);
uint16_t get_sequence_id(struct ptp_device *ptp, uint32_t port, PacketDirection bufferDirection,
                         uint8_t * packetBuffer);
void get_hardware_timestamp(struct ptp_device *ptp, uint32_t port, PacketDirection bufferDirection,
                            uint8_t * packetBuffer, PtpTime *timestamp);
void get_local_hardware_timestamp(struct ptp_device *ptp, uint32_t port, PacketDirection bufferDirection,
                                  uint8_t *packetBuffer, PtpTime *timestamp);
void get_timestamp(struct ptp_device *ptp, uint32_t port, PacketDirection bufferDirection,
                   uint8_t * packetBuffer, PtpTime *timestamp);
void get_correction_field(struct ptp_device *ptp, uint32_t port, uint8_t *txBuffer, PtpTime *correctionField);
uint32_t get_cumulative_scaled_rate_offset_field(uint8_t *rxBuffer);

/* From labx_ptp_state.c */
void ack_grandmaster_change(struct ptp_device *ptp);
void init_state_machines(struct ptp_device *ptp);
void process_rx_buffer(struct ptp_device *ptp, int port, uint8_t *buffer);

/* From labx_ptp_pdelay_state.c */
void MDPdelayReq_StateMachine(struct ptp_device *ptp, uint32_t port);
void LinkDelaySyncIntervalSetting_StateMachine(struct ptp_device *ptp, uint32_t port);

/* From labx_ptp_rtc.c */
void disable_rtc(struct ptp_device *ptp);
void set_rtc_increment(struct ptp_device *ptp, RtcIncrement *increment);
void get_rtc_increment(struct ptp_device *ptp, RtcIncrement *increment);
void get_rtc_time(struct ptp_device *ptp, PtpTime *time);
void get_local_time(struct ptp_device *ptp, PtpTime *time);
void set_rtc_time(struct ptp_device *ptp, PtpTime *time);
void set_rtc_time_adjusted(struct ptp_device *ptp, PtpTime *time, PtpTime *entryTime);
void rtc_update_servo(struct ptp_device *ptp, uint32_t port);
void update_rtc_lock_detect(struct ptp_device *ptp);

/* From labx_ptp_arithmetic.c */
void timestamp_sum(PtpTime *addend, PtpTime *augend, PtpTime *sum);
void timestamp_difference(PtpTime *minuend, PtpTime *subtrahend, PtpTime *difference);
void timestamp_abs(PtpTime *operand, PtpTime *result);
void timestamp_copy(PtpTime *destination, PtpTime *source);

/* From labx_ptp_netlink.c */
int register_ptp_netlink(void);
void unregister_ptp_netlink(void);
int ptp_events_tx_heartbeat(struct ptp_device *ptp);
int ptp_events_tx_gm_change(struct ptp_device *ptp);
int ptp_events_tx_rtc_change(struct ptp_device *ptp);

/* From Platform Specific Files */
void ptp_disable_irqs(struct ptp_device *ptp, int port);
void ptp_enable_irqs(struct ptp_device *ptp, int port);
void ptp_enable_port(struct ptp_device *ptp, int port);
void ptp_disable_port(struct ptp_device *ptp, int port);
void ptp_setup_event_timer(struct ptp_device *ptp, int port, PtpPlatformData *platformData);
uint32_t ptp_get_version(struct ptp_device *ptp);
uint32_t ptp_setup_interrupt(struct ptp_device *ptp);
void ptp_process_rx(struct ptp_device *ptp, int port);
uint8_t * get_output_buffer(struct ptp_device *ptp,uint32_t port,uint32_t bufType);
void write_packet(uint8_t *bufferBase, uint32_t *wordOffset, uint32_t writeWord);
uint32_t read_packet(uint8_t * bufferBase, uint32_t *wordOffset);
void transmit_packet(struct ptp_device *ptp, uint32_t port, uint8_t * txBuffer);


/* Bytes in a buffer word */
#define BYTES_PER_WORD  (4)
#define PTP_ETHERTYPE        (0x88F7u)

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif


#ifdef PATH_DELAY_DEBUG
#define PTP_CLOCK_IDENTITY_CHARS 8
#endif
#endif /* _LABX_PTP_H_ */

