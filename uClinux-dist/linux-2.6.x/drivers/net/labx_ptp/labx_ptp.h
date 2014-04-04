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

#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
#define POLL_MARVELL_TIMESTAMPS() switch_timestamp_poll()
#define WAIT_MARVELL_TIMESTAMPS() switch_timestamp_wait()
#else
#define POLL_MARVELL_TIMESTAMPS()
#define WAIT_MARVELL_TIMESTAMPS()
#endif
#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS

#define ETH_BASEADDR 0x82040000

enum TimeStampPair {
    TIMESTAMP_AVB2_INCOMMING_SYNC,
    TIMESTAMP_AVB2_OUTGOING_SYNC,
    TIMESTAMP_AVB2_INCOMMING_REQUEST,
    TIMESTAMP_AVB2_OUTGOING_REQUEST,
    TIMESTAMP_AVB2_INCOMMING_RESPONSE,
    TIMESTAMP_AVB2_OUTGOING_RESPONSE,
    TIMESTAMP_AVB1_INCOMMING_SYNC,
    TIMESTAMP_AVB1_OUTGOING_SYNC,
    TIMESTAMP_AVB1_INCOMMING_REQUEST,
    TIMESTAMP_AVB1_OUTGOING_REQUEST,
    TIMESTAMP_AVB1_INCOMMING_RESPONSE,
    TIMESTAMP_AVB1_OUTGOING_RESPONSE,
};

enum TimeStamp {
    TIMESTAMP_AVB2_OUTGOING_SYNC_T1,
    TIMESTAMP_AVB2_OUTGOING_DELAY_T1,
    TIMESTAMP_AVB2_OUTGOING_T2,
    TIMESTAMP_AVB2_INCOMMING_SYNC_T1,
    TIMESTAMP_AVB2_INCOMMING_DELAY_T1,
    TIMESTAMP_AVB2_INCOMMING_T2,
    TIMESTAMP_AVB1_OUTGOING_SYNC_T1,
    TIMESTAMP_AVB1_OUTGOING_DELAY_T1,
    TIMESTAMP_AVB1_OUTGOING_T2,
    TIMESTAMP_AVB1_INCOMMING_SYNC_T1,
    TIMESTAMP_AVB1_INCOMMING_DELAY_T1,
    TIMESTAMP_AVB1_INCOMMING_T2,
};

typedef struct {
    uint16_t flags;
    uint16_t low;
    uint16_t high;
    uint16_t sequence_id;
    enum TimeStamp type;
} switch_timestamp_t;

void switch_timestamp_init(void);
void switch_timestamp_poll(void);
void switch_timestamp_wait(void);
void switch_timestamp(enum TimeStampPair type,switch_timestamp_t ** t1, switch_timestamp_t ** t2,uint16_t sequence_id);

#define XPAR_XPS_GPIO_0_BASEADDR 0x820F0000
#define LABX_MDIO_ETH_BASEADDR 0x82050000
#define MDIO_CONTROL_REG      (0x00000000)
#  define PHY_MDIO_BUSY       (0x80000000)
#  define PHY_REG_ADDR_MASK   (0x01F)
#  define PHY_ADDR_MASK       (0x01F)
#  define PHY_ADDR_SHIFT      (5)
#  define PHY_MDIO_READ       (0x0400)
#  define PHY_MDIO_WRITE      (0x0000)
#define MDIO_DATA_REG         (0x00000004)

#define LABX_MAC_REGS_BASE    (0x00001000)
#define MAC_MDIO_CONFIG_REG   (LABX_MAC_REGS_BASE + 0x0014)
#define LABX_ETHERNET_MDIO_DIV  (0x28)
#  define MDIO_DIVISOR_MASK  (0x0000003F)
#  define MDIO_ENABLED       (0x00000040)


/* Performs a register write to a PHY */
static inline void mdio_write(int phy_addr, int reg_addr, int phy_data) {
  unsigned int addr;

  /* Write the data first, then the control register */
  addr = (LABX_MDIO_ETH_BASEADDR + MDIO_DATA_REG);
  *((volatile unsigned int *) addr) = phy_data;
  addr = (LABX_MDIO_ETH_BASEADDR + MDIO_CONTROL_REG);
  *((volatile unsigned int *) addr) = 
    (PHY_MDIO_WRITE | ((phy_addr & PHY_ADDR_MASK) << PHY_ADDR_SHIFT) |
     (reg_addr & PHY_REG_ADDR_MASK));
  while(*((volatile unsigned int *) addr) & PHY_MDIO_BUSY);
}

/* Performs a register read from a PHY */
static inline unsigned int mdio_read(int phy_addr, int reg_addr) {
  unsigned int addr;
  unsigned int readValue;

  /* Write to the MDIO control register to initiate the read */
  addr = (LABX_MDIO_ETH_BASEADDR + MDIO_CONTROL_REG);
  *((volatile unsigned int *) addr) = 
    (PHY_MDIO_READ | ((phy_addr & PHY_ADDR_MASK) << PHY_ADDR_SHIFT) |
     (reg_addr & PHY_REG_ADDR_MASK));
  while(*((volatile unsigned int *) addr) & PHY_MDIO_BUSY);
  addr = (LABX_MDIO_ETH_BASEADDR + MDIO_DATA_REG);
  readValue = *((volatile unsigned int *) addr);
  return(readValue);
}

/* CAL_ICS constants; ultimately these and the corresponding code
 * should migrate into board-specific init code.
 */
#define REG_PORT(p) (0x10 + (p))
#define REG_GLOBAL 0x1b
#define REG_GLOBAL2 0x1c

#define AVB_COMMAND_REG 0x16
#define AVB_DATA_REG 0x17
#define AVB_COMMAND_WRITE 0x3
#define AVB_COMMAND_READ 0x4
#define AVB_COMMAND_BLOCK_READ 0x6
#define AVB_BLOCK_PTP 0
#define AVB_BLOCK_POLICY 1
#define AVB_BLOCK_QAV 2
#define AVB_COMMAND_READ_PTP(PORT,ADDRESS) ((1<<15)|(AVB_COMMAND_READ<<12)|(PORT<<8)|(AVB_BLOCK_PTP<<5)|ADDRESS)
#define AVB_COMMAND_BLOCK_READ_PTP(PORT,ADDRESS) ((1<<15)|(AVB_COMMAND_BLOCK_READ<<12)|(PORT<<8)|(AVB_BLOCK_PTP<<5)|ADDRESS)
#define AVB_COMMAND_WRITE_PTP(PORT,ADDRESS) ((1<<15)|(AVB_COMMAND_WRITE<<12)|(PORT<<8)|(AVB_BLOCK_PTP<<5)|ADDRESS)
#define AVB_COMMAND_WRITE_POLICY(PORT,ADDRESS) ((1<<15)|(AVB_COMMAND_WRITE<<12)|(PORT<<8)|(AVB_BLOCK_POLICY<<5)|ADDRESS)
#define AVB_COMMAND_WRITE_QAV(PORT,ADDRESS) ((1<<15)|(AVB_COMMAND_WRITE<<12)|(PORT<<8)|(AVB_BLOCK_QAV<<5)|ADDRESS)

static inline uint16_t read_avb_ptp(int port,int reg) {
    while(mdio_read(REG_GLOBAL2,AVB_COMMAND_REG)&(1<<15));
    mdio_write(REG_GLOBAL2,AVB_COMMAND_REG,AVB_COMMAND_READ_PTP(port,reg));
    while(mdio_read(REG_GLOBAL2,AVB_COMMAND_REG)&(1<<15));
    return(mdio_read(REG_GLOBAL2,AVB_DATA_REG));
}

static inline void write_avb_ptp(int port,int reg,int value) {
    while(mdio_read(REG_GLOBAL2,AVB_COMMAND_REG)&(1<<15));
    mdio_write(REG_GLOBAL2,AVB_DATA_REG,value);
    while(mdio_read(REG_GLOBAL2,AVB_COMMAND_REG)&(1<<15));
    mdio_write(REG_GLOBAL2,AVB_COMMAND_REG,AVB_COMMAND_WRITE_PTP(port,reg));
}

static inline void write_avb_policy(int port,int reg,int value) {
    while(mdio_read(REG_GLOBAL2,AVB_COMMAND_REG)&(1<<15));
    mdio_write(REG_GLOBAL2,AVB_DATA_REG,value);
    while(mdio_read(REG_GLOBAL2,AVB_COMMAND_REG)&(1<<15));
    mdio_write(REG_GLOBAL2,AVB_COMMAND_REG,AVB_COMMAND_WRITE_POLICY(port,reg));
}

static inline void write_avb_qav(int port,int reg,int value) {
    while(mdio_read(REG_GLOBAL2,AVB_COMMAND_REG)&(1<<15));
    mdio_write(REG_GLOBAL2,AVB_DATA_REG,value);
    while(mdio_read(REG_GLOBAL2,AVB_COMMAND_REG)&(1<<15));
    mdio_write(REG_GLOBAL2,AVB_COMMAND_REG,AVB_COMMAND_WRITE_QAV(port,reg));
}

static inline void block_read_avb_ptp(switch_timestamp_t * time,int port,int reg) {
    while(mdio_read(REG_GLOBAL2,AVB_COMMAND_REG)&(1<<15));
    mdio_write(REG_GLOBAL2,AVB_COMMAND_REG,AVB_COMMAND_BLOCK_READ_PTP(port,reg));
    time->flags=mdio_read(REG_GLOBAL2,AVB_DATA_REG);
    time->low=mdio_read(REG_GLOBAL2,AVB_DATA_REG);
    time->high=mdio_read(REG_GLOBAL2,AVB_DATA_REG);
    time->sequence_id=mdio_read(REG_GLOBAL2,AVB_DATA_REG);
}

#define DEBUG_TIMESTAMPS
#define WARN_TIMESTAMPS
#define ERROR_TIMESTAMPS


#ifdef DEBUG_TIMESTAMPS
#define DEBUG_TIMESTAMP_PRINTF(FMT,...) printk(FMT, ## __VA_ARGS__)
#else
#define DEBUG_TIMESTAMP_PRINTF(FMT,...)
#endif

#ifdef WARN_TIMESTAMPS
#define WARN_TIMESTAMP_PRINTF(FMT,...) printk(FMT, ## __VA_ARGS__)
#else
#define WARN_TIMESTAMP_PRINTF(FMT,...)
#endif

#ifdef ERROR_TIMESTAMPS
#define ERROR_TIMESTAMP_PRINTF(FMT,...) printk(FMT, ## __VA_ARGS__)
#else
#define ERROR_TIMESTAMP_PRINTF(FMT,...)
#endif

#endif /* CONFIG_LABX_PTP_MARVELL_TIMESTAMPS */


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
#define STEPS_REMOVED_OFFSET                 (18 * BYTES_PER_WORD)
#define GM_TIME_BASE_INDICATOR_OFFSET        (18 * BYTES_PER_WORD)
#define PATH_TRACE_OFFSET                    (20 * BYTES_PER_WORD)

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

/* 802.1AS PortAnnounceInformation state machine states */
typedef enum { PortAnnounceInformation_BEGIN,
  PortAnnounceInformation_DISABLED, PortAnnounceInformation_AGED,
  PortAnnounceInformation_UPDATE, PortAnnounceInformation_SUPERIOR_MASTER_PORT,
  PortAnnounceInformation_REPEATED_MASTER_PORT, PortAnnounceInformation_INFERIOR_MASTER_OR_OTHER_PORT,
  PortAnnounceInformation_CURRENT, PortAnnounceInformation_RECEIVE
} PortAnnounceInformation_State_t;

typedef enum { InfoIs_Disabled, InfoIs_Received, InfoIs_Aged, InfoIs_Mine
} PtpInfoIs;

/* 802.1AS PortRoleSelection state machine states */
typedef enum { PortRoleSelection_INIT_BRIDGE, PortRoleSelection_ROLE_SELECTION
} PortRoleSelection_State_t;

#define SIGNED_SHIFT(a, b) (((b) >= 0) ? ((a)<<(b)) : ((a)>>(-b)))
#define ANNOUNCE_INTERVAL_TICKS(ptp, port)   \
  SIGNED_SHIFT((1000/PTP_TIMER_TICK_MS), ((ptp)->ports[(port)].currentLogAnnounceInterval))
#define SYNC_INTERVAL_TICKS(ptp, port)       \
  SIGNED_SHIFT((1000/PTP_TIMER_TICK_MS), ((ptp)->ports[(port)].currentLogSyncInterval))
#define SYNC_INTERVAL_TIMED_OUT(ptp, port)       \
  (((ptp)->ports[(port)].syncTimeoutCounter*PTP_TIMER_TICK_MS)>(SIGNED_SHIFT(1000*(ptp)->ports[(port)].syncReceiptTimeout,((ptp)->ports[(port)].currentLogSyncInterval))))
#define PDELAY_REQ_INTERVAL_TICKS(ptp, port) \
  SIGNED_SHIFT((1000/PTP_TIMER_TICK_MS), ((ptp)->ports[(port)].currentLogPdelayReqInterval))

/* Definitions and macros for manipulating port numbers */
#define PTP_PORT_NUMBER_BYTES sizeof(uint16_t)
typedef uint8_t PtpPortNumber[PTP_PORT_NUMBER_BYTES];

typedef struct __attribute__((packed)) {
  PtpClockIdentity clockIdentity;
  PtpPortNumber    portNumber;
} PtpPortIdentity; /* 8.5.2 */

/* Definitions and macros for manipulating offset scaled log variance */
#define PTP_OFFSET_VARIANCE_BYTES sizeof(uint16_t)
typedef uint8_t PtpOffsetVariance[PTP_OFFSET_VARIANCE_BYTES];

/* The PtpClockQuality structure used for userspace access is "flattened" here to
 * explicitly control the alignment of accesses and endian-ness
 */
typedef struct __attribute__((packed)) {
  uint8_t          priority1;

  /* Elements from PtpClockQuality flattened for packing and alignment */
  uint8_t           clockClass;
  uint8_t           clockAccuracy;
  PtpOffsetVariance offsetScaledLogVariance;

  uint8_t           priority2;
  PtpClockIdentity  clockIdentity;
} PtpSystemIdentity; /* 10.3.2 */

/* Definitions and macros for tracking steps removed */
#define PTP_STEPS_REMOVED_BYTES sizeof(uint16_t)
typedef uint8_t PtpStepsRemoved[PTP_STEPS_REMOVED_BYTES];

/* NOTE: All fields in the priority vector should be BIG ENDIAN for proper
   byte-wise comparison of the UInteger224 */
typedef struct __attribute__((packed)) {
  PtpSystemIdentity rootSystemIdentity;
  PtpStepsRemoved   stepsRemoved;
  PtpPortIdentity   sourcePortIdentity; /* Port identity of the transmitting port */
  PtpPortNumber     portNumber;         /* Port number of the receiving port */
} PtpPriorityVector; /* 10.3.4 */

typedef enum {
  SuperiorMasterInfo,
  RepeatedMasterInfo,
  InferiorMasterInfo,
  OtherInfo
} AnnounceReceiveInfo;

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

  /* 802.1AS per-port variables (10.2.3 - 10.2.4) - Time sync state machines */
  PtpRole  selectedRole; /* 10.2.3.20 */
  uint32_t asCapable;
  int8_t   currentLogSyncInterval;
  int8_t   initialLogSyncInterval;
  uint32_t neighborRateRatio;
  uint32_t neighborPropDelay;
  uint32_t computeNeighborRateRatio;
  uint32_t computeNeighborPropDelay;
  uint32_t portEnabled;
  uint32_t pttPortEnabled;

  /* 802.1AS PortSyncSyncSend state machine variables (10.2.11) */
  PtpTime  lastPreciseOriginTimestamp;  /* 10.2.11.1.4 */
  PtpTime  lastFollowUpCorrectionField; /* 10.2.11.1.5 */

  uint32_t fupPreciseOriginTimestampReceived;
  uint32_t syncTxLocalTimestampValid;

  /* 802.1AS per-port variables (10.3.8 - 10.3.9) - BMCA/Announce state machines */
  int8_t            reselect;                   /* 10.3.8.1 (bool) */
  int8_t            selected;                   /* 10.3.8.2 (bool) */
  PtpInfoIs         infoIs;                     /* 10.3.9.2 */
  PtpPriorityVector masterPriority;             /* 10.3.9.3 */
  int8_t            currentLogAnnounceInterval; /* 10.3.9.4 */
  int8_t            initialLogAnnounceInterval; /* 10.3.9.5 */
  int8_t            newInfo;                    /* 10.3.9.8 (bool) */
  PtpPriorityVector portPriority;               /* 10.3.9.9 */
  PtpPriorityVector gmPathPriority;
  uint16_t          portStepsRemoved;           /* 10.3.9.10 */
  uint8_t          *rcvdAnnouncePtr;            /* 10.3.9.11 */
  int8_t            rcvdMsg;                    /* 10.3.9.12 (bool) */
  int8_t            updtInfo;                   /* 10.3.9.13 (bool) */

  /* 802.1AS PortAnnounceInformation state machine variables */
  PortAnnounceInformation_State_t portAnnounceInformation_State;
  uint32_t            announceTimeoutCounter;   /* 10.3.11.1.1 equivalent (Ticks since last received announce) */
  PtpPriorityVector   messagePriority;          /* 10.3.11.1.2 */
  AnnounceReceiveInfo rcvdInfo;                 /* 10.3.11.1.3 */

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

  /* AVnu_PTP-5 PICS */
  uint32_t pdelayResponses;
  uint32_t multiplePdelayResponses;

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

#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
  int skippedResponseCount;
  int skippedFollowupCount;
  int recoveringA;
  int recoveringC;
  int recoveringE;
#endif

  /* pdelay response variables */
  uint8_t lastPeerRequestPortId[PORT_ID_BYTES];

  /* sync/fup response variables */
  uint8_t syncSourcePortId[PORT_ID_BYTES];

  /* Timer state space */
  uint32_t announceCounter;
  uint16_t announceSequenceId;
  uint32_t syncTimeoutCounter;
  uint32_t syncCounter;
  uint16_t syncSequenceId;
  uint32_t delayReqCounter;
  uint32_t delayReqSequenceId;

  /* Packet Rx state space */
  uint32_t lastRxBuffer;
  uint32_t syncSequenceIdValid;

  /* Packet statistics */
  PtpAsPortStatistics stats;

  /* Per port path trace data */
  uint32_t           pathTraceLength;               
  PtpClockIdentity   pathTrace[PTP_MAX_PATH_TRACE]; 
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
  uint32_t numPorts; /* (8.6.2.8) */

  /* Width, in bits, of the instance's ports (< 10G == 8, 10G == 64)*/
  uint32_t portWidth;

  /* 802.1AS Time aware system attributes (8.6.2) */
  PtpProperties properties;

  /* 802.1AS Time aware system global variables (10.2.3) */
  uint8_t            gmPresent; /* 10.2.3.13 (bool) */

  /* 802.1AS Time aware system global variables (10.3.8) */
  uint16_t           masterStepsRemoved; /* 10.3.8.3 */
  PtpPriorityVector  systemPriority; /* Priority vector for this system (10.3.8.18) */
  PtpPriorityVector *gmPriority;     /* Priority vector for the current grandmaster (10.3.8.19) */
  PtpPriorityVector  lastGmPriority; /* Previous grandmaster priority vector (10.3.8.20) */
  uint32_t           pathTraceLength;               /* Number of paths listed in the pathTrace array */
  PtpClockIdentity   pathTrace[PTP_MAX_PATH_TRACE]; /* 10.3.8.21 */

  uint16_t lastGmTimeBaseIndicator;

  /* RTC control loop constants */
  RtcIncrement    nominalIncrement;
  PtpCoefficients coefficients;
  uint32_t masterRateRatio;
  uint32_t masterRateRatioValid;

  /* RTC control loop persistent values */
  int64_t  integral;
  int64_t  zeroCrossingIntegral;
  int32_t  derivative;
  int32_t  previousOffset;
  uint32_t prevBaseRtcIncrement;
  uint32_t prevAppliedRtcIncrement;
  int32_t  rtcLastIncrementDelta;
  uint32_t rtcChangesAllowed;
  uint32_t rtcReset;
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
  PortRoleSelection_State_t       portRoleSelection_State;

  /* New master event needs to be transmitted */
  uint32_t newMaster;

  /* Timer state space */
#ifndef CONFIG_LABX_PTP_NO_TASKLET
  struct tasklet_struct timerTasklet;
#endif
  uint32_t heartbeatCounter;
  uint32_t netlinkSequence;

  /* Netlink workers to send messages */
  struct work_struct work_send_gm_change;
  struct work_struct work_send_rtc_change;
  struct work_struct work_send_heartbeat;
  struct work_struct work_send_rtc_increment_change;

  /* Packet Rx state space */
#ifndef CONFIG_LABX_PTP_NO_TASKLET
  struct tasklet_struct rxTasklet;
#endif
  uint32_t slaveDebugCounter;

  /* Packet Tx state space */
#ifndef CONFIG_LABX_PTP_NO_TASKLET
  struct tasklet_struct txTasklet;
#endif

  /* Per-port data */
  struct ptp_port *ports;

  /* Mutex for the device instance */
  spinlock_t mutex;
  uint32_t opened;

  /* Network device event notifier */
  struct notifier_block notifier;

  /* Number of timer ticks that have passed since the last time the tasklet ran */
  uint32_t timerTicks;
#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
  uint32_t t2_prev;
  PtpTime switchDelta;
#endif
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
void get_source_port_id(struct ptp_device *ptp, uint32_t port, PacketDirection bufferDirection, uint8_t *packetBuffer, uint8_t *sourcePortId);
void set_source_port_id(struct ptp_device *ptp, uint32_t port, PacketDirection bufferDirection, uint8_t *packetBuffer, uint8_t *sourcePortId);
void get_rx_requesting_port_id(struct ptp_device *ptp, uint32_t port, uint8_t * rxBuffer, uint8_t *requestingPortId);
uint16_t get_rx_announce_steps_removed(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer);
uint16_t get_rx_announce_path_trace(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer, PtpClockIdentity *pathTrace);
void extract_announce(struct ptp_device *ptp, uint32_t port, uint8_t * rxBuffer, PtpPriorityVector *pv);
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
uint16_t get_gm_time_base_indicator_field(uint8_t *rxBuffer);
uint32_t get_cumulative_scaled_rate_offset_field(uint8_t *rxBuffer);
uint16_t get_port_number(const uint8_t *portNumber);
void set_port_number(uint8_t *portNumber, uint16_t setValue);
uint16_t get_steps_removed(const uint8_t *stepsRemoved);
void set_steps_removed(uint8_t *stepsRemoved, uint16_t setValue);
uint16_t get_offset_scaled_log_variance(const uint8_t *offsetScaledLogVariance);
void set_offset_scaled_log_variance(uint8_t *offsetScaledLogVariance, uint16_t setValue);

/* From labx_ptp_state.c */
void ack_grandmaster_change(struct ptp_device *ptp);
void init_state_machines(struct ptp_device *ptp);
void process_rx_buffer(struct ptp_device *ptp, int port, uint8_t *buffer);
void labx_ptp_timer_state_task(unsigned long data);
void labx_ptp_rx_state_task(unsigned long data);
void labx_ptp_tx_state_task(unsigned long data);

/* From labx_ptp_pdelay_state.c */
void MDPdelayReq_StateMachine(struct ptp_device *ptp, uint32_t port);
void LinkDelaySyncIntervalSetting_StateMachine(struct ptp_device *ptp, uint32_t port);

/* From labx_ptp_bmca_announce_state.c */
void PortAnnounceReceive_StateMachine(struct ptp_device *ptp, uint32_t port);
void PortAnnounceInformation_StateMachine(struct ptp_device *ptp, uint32_t port);
void PortRoleSelection_StateMachine(struct ptp_device *ptp);

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
int ptp_events_tx_rtc_increment_change(struct ptp_device *ptp);
void ptp_work_send_heartbeat(struct work_struct *work);
void ptp_work_send_gm_change(struct work_struct *work);
void ptp_work_send_rtc_change(struct work_struct *work);
void ptp_work_send_rtc_increment_change(struct work_struct *work);

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


//#define PATH_DELAY_DEBUG

#ifdef PATH_DELAY_DEBUG
#define PTP_CLOCK_IDENTITY_CHARS 8
#endif

#endif /* _LABX_PTP_H_ */

