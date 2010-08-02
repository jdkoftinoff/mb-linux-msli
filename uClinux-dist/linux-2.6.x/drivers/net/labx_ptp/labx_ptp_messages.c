/*
 *  linux/drivers/net/labx_ptp_messages.c
 *
 *  Lab X Technologies Precision Time Protocol (PTP) driver
 *  PTP message building and parsing
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

/* Bytes in a buffer word */
#define BYTES_PER_WORD  (4)

/* Ethernet header definitions */
#define LTF_MASK          (0x0FFFF)
#define ETH_HEADER_BYTES  (14)

/* PTP header definitions */
#define PTP_ETHERTYPE        (0x88F7)
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
#define PACKET_BUFFER_WORDS     (PTP_MAX_PACKET_BYTES / BYTES_PER_WORD)
#define SOURCE_MAC_OFFSET       ( 1 * BYTES_PER_WORD)
#define MESSAGE_TYPE_OFFSET     ( 3 * BYTES_PER_WORD)
#define DOMAIN_NUMBER_OFFSET    ( 4 * BYTES_PER_WORD)
#define CORRECTION_FIELD_OFFSET ( 5 * BYTES_PER_WORD)
#define SOURCE_PORT_ID_OFFSET   ( 8 * BYTES_PER_WORD)
#define SEQUENCE_ID_OFFSET      (11 * BYTES_PER_WORD)
#define TIMESTAMP_OFFSET        (12 * BYTES_PER_WORD)
#define UTC_OFFSET_OFFSET       (14 * BYTES_PER_WORD)
#define REQ_PORT_ID_OFFSET      (14 * BYTES_PER_WORD)
#define GM_PRIORITY1_OFFSET     (15 * BYTES_PER_WORD)
#define HW_TIMESTAMP_OFFSET     ((PACKET_BUFFER_WORDS - HW_TIMESTAMP_WORDS) * \
                                  BYTES_PER_WORD)
#define HW_LOCAL_TIMESTAMP_OFFSET (HW_TIMESTAMP_OFFSET - (HW_TIMESTAMP_WORDS * BYTES_PER_WORD))

/* Additional offset applied for the transmit buffers, since the first
 * word holds the packet length (minus one.)
 */
#define TX_LENGTH_OFFSET  (0 * BYTES_PER_WORD)
#define TX_DATA_OFFSET    (1 * BYTES_PER_WORD)

/* Length, in bytes, of each packet type we transmit */
#define PTP_ANNOUNCE_LENGTH         (64)
#define PTP_SYNC_LENGTH             (44)
#define PTP_FUP_LENGTH              (44)
#define PTP_DELAY_REQ_LENGTH        (44)
#define PTP_DELAY_RESP_LENGTH       (54)
#define PTP_PDELAY_REQ_LENGTH       (54)
#define PTP_PDELAY_RESP_LENGTH      (54)
#define PTP_PDELAY_RESP_FUP_LENGTH  (54)

/**
 * Common helper methods
 */

/* Writes a word to a packet buffer at the passed offset.  The offset is
 * advanced to the next word.
 */
static void write_packet(uint32_t bufferBase, uint32_t *wordOffset, 
                         uint32_t writeWord) {
  XIo_Out32((bufferBase + *wordOffset), writeWord);
  *wordOffset += BYTES_PER_WORD;
}

/* Reads a word from a packet buffer at the passed offset.  The offset is
 * advanced to the next word.
 */
static uint32_t read_packet(uint32_t bufferBase, uint32_t *wordOffset) {
  uint32_t readWord = XIo_In32(bufferBase + *wordOffset);
  *wordOffset += BYTES_PER_WORD;
  return(readWord);
}

/**
 * Packet transmission methods
 */

/* Builds the common PTP header into the passed buffer using the passed word offset,
 * which should be set to zero prior to the call.
 */
static void init_ptp_header(struct ptp_device *ptp, uint32_t txBuffer, 
                            uint32_t *wordOffset, uint32_t messageType,
                            uint32_t messageLength, uint16_t headerFlags) {
  uint32_t bufferBase;
  uint32_t packetWord;

  /* Locate the requested buffer and begin with the packet's transmit length 
   * word (transmit length minus one.)
   */
  bufferBase = PTP_TX_PACKET_BUFFER(ptp, txBuffer);
  write_packet(bufferBase, wordOffset, (ETH_HEADER_BYTES + messageLength - 1));

  /* Begin with the destination and source MAC addresses.
   * The following multicast MAC addresses are used for PTP:
   * Peer delay mechanism messages : 01-80-C2-00-00-0E
   * End to end delay mechanism    : 01-1B-19-00-00-00
   *
   * The multicast destination address used for the peer delay
   * mechanism is borrowed from the 802.1D MAC Bridges Standard,
   * and is classified as a MAC bridge filtered address.  Ethernet
   * bridges will *not* relay these packets, making this only
   * functional for 802.1AS-aware bridges.
   */
  if(ptp->properties.delayMechanism == PTP_DELAY_MECHANISM_P2P) {
    /* Peer-to-peer (802.1AS) */
    write_packet(bufferBase, wordOffset, 0x0180C200);
    packetWord = 0x000E0000;
    packetWord |= (ptp->properties.sourceMacAddress[0] << 8);
    packetWord |= ptp->properties.sourceMacAddress[1];
    write_packet(bufferBase, wordOffset, packetWord);
  } else {
    /* End-to-end (legacy PTP 2.0) */
    write_packet(bufferBase, wordOffset, 0x011B1900);
    packetWord = 0x00000000;
    packetWord |= (ptp->properties.sourceMacAddress[0] << 8);
    packetWord |= ptp->properties.sourceMacAddress[1];
    write_packet(bufferBase, wordOffset, packetWord);
  }
  packetWord = (ptp->properties.sourceMacAddress[2] << 24);
  packetWord |= (ptp->properties.sourceMacAddress[3] << 16);
  packetWord |= (ptp->properties.sourceMacAddress[4] << 8);
  packetWord |= ptp->properties.sourceMacAddress[5];
  write_packet(bufferBase, wordOffset, packetWord);

  /* PTP EtherType and the first two PTP header bytes */
  packetWord = (PTP_ETHERTYPE << 16);
  packetWord |= (TS_SPEC_ETH_AVB << 12);
  packetWord |= ((messageType & MSG_TYPE_MASK) << 8);
  packetWord |= PTP_VERSION_2_0;
  write_packet(bufferBase, wordOffset, packetWord);

  /* Message length, domain number, and reserved field */
  packetWord = ((messageLength & MSG_LENGTH_MASK) << 16);
  packetWord |= ((ptp->properties.domainNumber & DOMAIN_NUM_MASK) << 8);
  write_packet(bufferBase, wordOffset, packetWord);

  /* Assign the passed flags into the flag field, clear the correction field */
  packetWord = (headerFlags << 16);
  write_packet(bufferBase, wordOffset, packetWord);

  /* Clear out the flag field, correction field, four reserved bytes,
   * and set the 80-bit source ID to be our clockIdentity, port zero.
   * clockIdentity is formed by our OUI, 0xFFFE, and the serial number.
   * OUI and serial number are the first three and last three bytes of
   * our MAC address, respectively.
   */
  write_packet(bufferBase, wordOffset, 0x00000000);
  write_packet(bufferBase, wordOffset, 0x00000000);
  packetWord = (ptp->properties.sourceMacAddress[0] << 8);
  packetWord |= ptp->properties.sourceMacAddress[1];
  write_packet(bufferBase, wordOffset, packetWord);
  packetWord = (ptp->properties.sourceMacAddress[2] << 24);
  packetWord |= 0x00FFFE00;
  packetWord |= ptp->properties.sourceMacAddress[3];
  write_packet(bufferBase, wordOffset, packetWord);
  packetWord = (ptp->properties.sourceMacAddress[4] << 24);
  packetWord |= (ptp->properties.sourceMacAddress[5] << 16);
  write_packet(bufferBase, wordOffset, packetWord);

  /* Clear the sequence ID and log message interval to zero, and set the
   * control field appropriately for the message type.
   */
  switch(messageType) {
  case MSG_SYNC:
    write_packet(bufferBase, wordOffset, 0x00000000);
    break;

  case MSG_DELAY_REQ:
    write_packet(bufferBase, wordOffset, 0x00000100);
    break;

  case MSG_FUP:
    write_packet(bufferBase, wordOffset, 0x00000200);
    break;

  case MSG_DELAY_RESP:
    write_packet(bufferBase, wordOffset, 0x00000300);
    break;

  case MSG_MANAGEMENT:
    write_packet(bufferBase, wordOffset, 0x00000400);
    break;

  default:
    write_packet(bufferBase, wordOffset, 0x00000500);
  }
}

/* Initializes the ANNOUNCE message transmit template */
static void init_announce_template(struct ptp_device *ptp) {
  uint32_t bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;
  PtpClockQuality *quality;

  /* Clear out all dynamic fields and populate static ones with properties from
   * the PTP device structure.
   */
  wordOffset = 0;
  init_ptp_header(ptp, PTP_TX_ANNOUNCE_BUFFER, &wordOffset, MSG_ANNOUNCE, 
                  PTP_ANNOUNCE_LENGTH, (uint16_t) (FLAG_TWO_STEP|FLAG_PTP_TIMESCALE));
  bufferBase = PTP_TX_PACKET_BUFFER(ptp, PTP_TX_ANNOUNCE_BUFFER);

  /* Clear originTimestamp and set currentUtcOffset */
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  packetWord = (ptp->properties.currentUtcOffset & 0x0000FFFF);
  write_packet(bufferBase, &wordOffset, packetWord);

  /* Clear reserved, and set the grandmaster properties.  Initialize the stepsRemoved
   * field to zero since the packet's source.
   */
  quality = &ptp->properties.grandmasterClockQuality;
  packetWord = (ptp->properties.grandmasterPriority1 << 16);
  packetWord |= (quality->clockClass << 8);
  packetWord |= quality->clockAccuracy;
  write_packet(bufferBase, &wordOffset, packetWord);
  packetWord = (quality->offsetScaledLogVariance << 16);
  packetWord |= (ptp->properties.grandmasterPriority2 << 8);
  packetWord |= ptp->properties.grandmasterIdentity[0];
  write_packet(bufferBase, &wordOffset, packetWord);
  packetWord = ((ptp->properties.grandmasterIdentity[1] << 24) | 
                (ptp->properties.grandmasterIdentity[2] << 16) | 
                (ptp->properties.grandmasterIdentity[3] << 8)  |
                ptp->properties.grandmasterIdentity[4]);
  write_packet(bufferBase, &wordOffset, packetWord);
  packetWord = ((ptp->properties.grandmasterIdentity[5] << 24) | 
                (ptp->properties.grandmasterIdentity[6] << 16) | 
                (ptp->properties.grandmasterIdentity[7] << 8));
  write_packet(bufferBase, &wordOffset, packetWord);
  packetWord = (ptp->properties.timeSource << 16);
  write_packet(bufferBase, &wordOffset, packetWord);
}

/* Initializes the SYNC message transmit template */
static void init_sync_template(struct ptp_device *ptp) {
  uint32_t bufferBase;
  uint32_t wordOffset;

  /* Initialize the header, and clear the originTimestamp for good measure. */
  wordOffset = 0;
  init_ptp_header(ptp, PTP_TX_SYNC_BUFFER, &wordOffset, MSG_SYNC, PTP_SYNC_LENGTH,
                  (uint16_t) FLAG_TWO_STEP);
  bufferBase = PTP_TX_PACKET_BUFFER(ptp, PTP_TX_SYNC_BUFFER);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
}

/* Initializes the FUP message transmit template */
static void init_fup_template(struct ptp_device *ptp) {
  uint32_t bufferBase;
  uint32_t wordOffset;

  /* Initialize the header, and clear the preciseOriginTimestamp for good measure */
  wordOffset = 0;
  init_ptp_header(ptp, PTP_TX_FUP_BUFFER, &wordOffset, MSG_FUP, PTP_FUP_LENGTH,
                  (uint16_t) FLAG_TWO_STEP);
  bufferBase = PTP_TX_PACKET_BUFFER(ptp, PTP_TX_FUP_BUFFER);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
}

/* Initializes the DELAY_REQ message transmit template */
static void init_delay_request_template(struct ptp_device *ptp) {
  uint32_t bufferBase;
  uint32_t wordOffset;

  /* Initialize the header, and clear the originTimestamp for good measure */
  wordOffset = 0;
  init_ptp_header(ptp, PTP_TX_DELAY_REQ_BUFFER, &wordOffset, MSG_DELAY_REQ, 
                  PTP_DELAY_REQ_LENGTH, (uint16_t) FLAG_TWO_STEP);
  bufferBase = PTP_TX_PACKET_BUFFER(ptp, PTP_TX_DELAY_REQ_BUFFER);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
}

/* Initializes the DELAY_RESP message transmit template */
static void init_delay_response_template(struct ptp_device *ptp) {
  uint32_t bufferBase;
  uint32_t wordOffset;

  /* Initialize the header, and clear the requestReceiptTimestamp and
   * requestingPortIdentity for good measure 
   */
  wordOffset = 0;
  init_ptp_header(ptp, PTP_TX_DELAY_RESP_BUFFER, &wordOffset, MSG_DELAY_RESP,
                  PTP_DELAY_RESP_LENGTH, (uint16_t) FLAG_TWO_STEP);
  bufferBase = PTP_TX_PACKET_BUFFER(ptp, PTP_TX_DELAY_RESP_BUFFER);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
}

/* Initializes the PDELAY_REQ message transmit template */
static void init_pdelay_request_template(struct ptp_device *ptp) {
  uint32_t bufferBase;
  uint32_t wordOffset;

  /* Initialize the header, and clear the originTimestamp for good measure.
   * Also clear out the reserved bytes at the end of the message; these are
   * set to zero and are sized to match that of the PDELAY_RESP message in
   * an attempt to minimize path asymmetry through bridges.
   */
  wordOffset = 0;
  init_ptp_header(ptp, PTP_TX_PDELAY_REQ_BUFFER, &wordOffset, MSG_PDELAY_REQ, 
                  PTP_PDELAY_REQ_LENGTH, (uint16_t) FLAG_TWO_STEP);
  bufferBase = PTP_TX_PACKET_BUFFER(ptp, PTP_TX_PDELAY_REQ_BUFFER);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
}

/* Initializes the PDELAY_RESP message transmit template */
static void init_pdelay_response_template(struct ptp_device *ptp) {
  uint32_t bufferBase;
  uint32_t wordOffset;

  /* Initialize the header, and clear the requestReceiptTimestamp and
   * requestingPortIdentity for good measure 
   */
  wordOffset = 0;
  init_ptp_header(ptp, PTP_TX_PDELAY_RESP_BUFFER, &wordOffset, MSG_PDELAY_RESP,
                  PTP_PDELAY_RESP_LENGTH, (uint16_t) FLAG_TWO_STEP);
  bufferBase = PTP_TX_PACKET_BUFFER(ptp, PTP_TX_PDELAY_RESP_BUFFER);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
}

/* Initializes the PDELAY_RESP_FUP message transmit template */
static void init_pdelay_response_fup_template(struct ptp_device *ptp) {
  uint32_t bufferBase;
  uint32_t wordOffset;

  /* Initialize the header, and clear the responseOriginTimestamp and
   * requestingPortIdentity for good measure 
   */
  wordOffset = 0;
  init_ptp_header(ptp, PTP_TX_PDELAY_RESP_FUP_BUFFER, &wordOffset, 
                  MSG_PDELAY_RESP_FUP, PTP_PDELAY_RESP_FUP_LENGTH, 
                  (uint16_t) FLAG_TWO_STEP);
  bufferBase = PTP_TX_PACKET_BUFFER(ptp, PTP_TX_PDELAY_RESP_FUP_BUFFER);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
  write_packet(bufferBase, &wordOffset, 0x00000000);
}

/* Initializes the transmit packet buffers with their packet templates */
void init_tx_templates(struct ptp_device *ptp) {
  /* Build the templates dynamically; while this could be done with initialized
   * constant data, this approach is more flexible and illustrates the packet
   * structure better.
   */
  init_announce_template(ptp);
  init_sync_template(ptp);
  init_fup_template(ptp);
  init_delay_request_template(ptp);
  init_delay_response_template(ptp);
  init_pdelay_request_template(ptp);
  init_pdelay_response_template(ptp);
  init_pdelay_response_fup_template(ptp);
}

/* Sets the sequence ID within the passed packet buffer */
static void set_sequence_id(struct ptp_device *ptp, uint32_t txBuffer, 
                            uint16_t sequenceId) {
  uint32_t bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  /* Read, modify, and write back the sequence ID */
  bufferBase = (PTP_TX_PACKET_BUFFER(ptp, txBuffer) + TX_DATA_OFFSET);
  wordOffset = SEQUENCE_ID_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  packetWord &= 0x0000FFFF;
  packetWord |= (sequenceId << 16);
  wordOffset = SEQUENCE_ID_OFFSET;
  write_packet(bufferBase, &wordOffset, packetWord);
}

/* Gets the sequence ID within the passed packet buffer */
uint16_t get_sequence_id(struct ptp_device *ptp, PacketDirection bufferDirection,
                         uint32_t packetBuffer) {
  uint32_t bufferBase;
  uint32_t wordOffset;

  /* Read and return the sequence ID */
  bufferBase = ((bufferDirection == TRANSMITTED_PACKET) ? 
                (PTP_TX_PACKET_BUFFER(ptp, packetBuffer) + TX_DATA_OFFSET) : 
                PTP_RX_PACKET_BUFFER(ptp, packetBuffer));
  wordOffset = SEQUENCE_ID_OFFSET;
  return((uint16_t) (read_packet(bufferBase, &wordOffset) >> 16));
}

/* Gets the message timestamp (e.g. originTimestamp) within the passed packet buffer */
void get_timestamp(struct ptp_device *ptp, PacketDirection bufferDirection,
                   uint32_t packetBuffer, PtpTime *timestamp) {
  uint32_t bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  bufferBase = ((bufferDirection == TRANSMITTED_PACKET) ? 
                (PTP_TX_PACKET_BUFFER(ptp, packetBuffer) + TX_DATA_OFFSET) : 
                PTP_RX_PACKET_BUFFER(ptp, packetBuffer));
  wordOffset = TIMESTAMP_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  timestamp->secondsUpper = (packetWord >> 16);
  timestamp->secondsLower = (packetWord << 16);
  packetWord = read_packet(bufferBase, &wordOffset);
  timestamp->secondsLower |= ((packetWord >> 16) & 0x0000FFFF);
  timestamp->nanoseconds = (packetWord << 16);
  packetWord = read_packet(bufferBase, &wordOffset);
  timestamp->nanoseconds |= ((packetWord >> 16) & 0x0000FFFF);
}

/* Gets the correction field from the passed Rx packet buffer */
void get_correction_field(struct ptp_device *ptp, uint32_t rxBuffer, PtpTime *correctionField) {
  uint32_t bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;
  int64_t rawField;

  /* Presume we are dealing with less than a second of correction */
  correctionField->secondsUpper = 0;
  correctionField->secondsLower = 0;
  bufferBase = PTP_RX_PACKET_BUFFER(ptp, rxBuffer);
  wordOffset = CORRECTION_FIELD_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  rawField = ((int64_t) (packetWord & 0x0000FFFF) << 48);
  packetWord = read_packet(bufferBase, &wordOffset);
  rawField |= (((int64_t) packetWord) << 16);
  packetWord = read_packet(bufferBase, &wordOffset);
  rawField |= (int64_t) ((packetWord & 0xFFFF0000) >> 16);
  correctionField->nanoseconds = (uint32_t) (rawField >> CORRECTION_FRACTION_BITS);
}

/* Sets the message timestamp (e.g. originTimestamp) within the passed packet buffer */
static void set_timestamp(struct ptp_device *ptp, uint32_t txBuffer, PtpTime *timestamp) {
  uint32_t bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  bufferBase = (PTP_TX_PACKET_BUFFER(ptp, txBuffer) + TX_DATA_OFFSET);
  wordOffset = TIMESTAMP_OFFSET;
  packetWord = ((((uint32_t) timestamp->secondsUpper) << 16) | 
                (timestamp->secondsLower >> 16));
  write_packet(bufferBase, &wordOffset, packetWord);
  packetWord = ((timestamp->secondsLower << 16) | (timestamp->nanoseconds >> 16));
  write_packet(bufferBase, &wordOffset, packetWord);
  packetWord = read_packet(bufferBase, &wordOffset);
  packetWord &= 0x0000FFFF;
  packetWord |= (timestamp->nanoseconds << 16);
  wordOffset -= BYTES_PER_WORD;
  write_packet(bufferBase, &wordOffset, packetWord);
}

/* Updates the correctionField for an outgoing packet */
static void update_correction_field(struct ptp_device *ptp, uint32_t txBuffer,
                                    int64_t correctionField) {
  uint32_t bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  bufferBase = (PTP_TX_PACKET_BUFFER(ptp, txBuffer) + TX_DATA_OFFSET);
  wordOffset = CORRECTION_FIELD_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  packetWord &= 0xFFFF0000;
  packetWord |= (uint32_t) (correctionField >> 48);
  wordOffset -= BYTES_PER_WORD;
  write_packet(bufferBase, &wordOffset, packetWord);
  packetWord = (uint32_t) (correctionField >> 16);
  write_packet(bufferBase, &wordOffset, packetWord);
  packetWord = read_packet(bufferBase, &wordOffset);
  packetWord &= 0x0000FFFF;
  packetWord |= (uint32_t) (correctionField << 16);
  wordOffset -= BYTES_PER_WORD;
  write_packet(bufferBase, &wordOffset, packetWord);
}

  /* Sets the requestingPortIdentity field of a delay response packet */
static void set_requesting_port_id(struct ptp_device *ptp, uint32_t txBuffer,
                                   uint8_t *requestPortId) {
  uint32_t bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  bufferBase = (PTP_TX_PACKET_BUFFER(ptp, txBuffer) + TX_DATA_OFFSET);
  wordOffset = REQ_PORT_ID_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  packetWord &= 0xFFFF0000;
  packetWord |= ((requestPortId[0] << 8) | requestPortId[1]);
  wordOffset -= BYTES_PER_WORD;
  write_packet(bufferBase, &wordOffset, packetWord);
  packetWord = ((requestPortId[2] << 24) | (requestPortId[3] << 16) |
                (requestPortId[4] << 8) | requestPortId[5]);
  write_packet(bufferBase, &wordOffset, packetWord);
  packetWord = ((requestPortId[6] << 24) | (requestPortId[7] << 16) |
                (requestPortId[8] << 8) | requestPortId[9]);
  write_packet(bufferBase, &wordOffset, packetWord);
}

/* Gets the hardware timestamp located within the passed packet buffer.
 * It is up to the client code to ascertain that a valid timestamp exists; that is,
 * a packet has either been transmitted from or received into the buffer.
 */
void get_hardware_timestamp(struct ptp_device *ptp, PacketDirection bufferDirection,
                            uint32_t packetBuffer, PtpTime *timestamp) {
  uint32_t bufferBase;
  uint32_t wordOffset;

  /* Fetch the hardware timestamp from the end of the specified packet buffer and pack
   * it into the passed timestamp structure.  Don't offset the Tx packet buffer by
   * its data offset in this case, since we're not going to be reading relative to
   * the start of data, but relative to the end of the buffer instead!
   */
  bufferBase = ((bufferDirection == TRANSMITTED_PACKET) ? 
                PTP_TX_PACKET_BUFFER(ptp, packetBuffer) :
                PTP_RX_PACKET_BUFFER(ptp, packetBuffer));
  wordOffset = HW_TIMESTAMP_OFFSET;
  timestamp->secondsUpper = (int32_t)(read_packet(bufferBase, &wordOffset) & 0x0FFFF);
  timestamp->secondsLower = read_packet(bufferBase, &wordOffset);
  timestamp->nanoseconds  = read_packet(bufferBase, &wordOffset);
}

/* Gets the local hardware timestamp located within the passed packet buffer.
 * This is the same as get_hardware_timestamp except it is from a local clock
 * that is running at a fixed rate unmodified by PTP.
 */
void get_local_hardware_timestamp(struct ptp_device *ptp, PacketDirection bufferDirection,
                            uint32_t packetBuffer, PtpTime *timestamp) {
  uint32_t bufferBase;
  uint32_t wordOffset;

  /* Fetch the hardware timestamp from the end of the specified packet buffer and pack
   * it into the passed timestamp structure.  Don't offset the Tx packet buffer by
   * its data offset in this case, since we're not going to be reading relative to
   * the start of data, but relative to the end of the buffer instead!
   */
  bufferBase = ((bufferDirection == TRANSMITTED_PACKET) ? 
                PTP_TX_PACKET_BUFFER(ptp, packetBuffer) :
                PTP_RX_PACKET_BUFFER(ptp, packetBuffer));
  wordOffset = HW_LOCAL_TIMESTAMP_OFFSET;
  timestamp->secondsUpper = (int32_t)(read_packet(bufferBase, &wordOffset) & 0x0FFFF);
  timestamp->secondsLower = read_packet(bufferBase, &wordOffset);
  timestamp->nanoseconds  = read_packet(bufferBase, &wordOffset);
}

void print_packet_buffer(struct ptp_device *ptp, PacketDirection bufferDirection,
                         uint32_t packetBuffer) {
  uint32_t bufferBase;
  uint32_t wordOffset;
  uint32_t wordIndex;

  /* Fetch the hardware timestamp from the end of the specified packet buffer and pack
   * it into the passed timestamp structure.
   */
  bufferBase = ((bufferDirection == TRANSMITTED_PACKET) ? 
                (PTP_TX_PACKET_BUFFER(ptp, packetBuffer) + TX_DATA_OFFSET) : 
                PTP_RX_PACKET_BUFFER(ptp, packetBuffer));
  wordOffset = 0;
  for(wordIndex = 0; wordIndex < (PTP_MAX_PACKET_BYTES / BYTES_PER_WORD); wordIndex++) {
    printk("0x%08X\n", read_packet(bufferBase, &wordOffset));
  }
}

/* Transmits the packet within the specified buffer.  The first word of the
 * buffer must contain the packet length minus one, in bytes.
 */
static void transmit_packet(struct ptp_device *ptp, uint32_t txBuffer) {
  /* Don't bother checking the busy flag; it only exists to ensure that we know the
   * pending flags are valid.  We're not using them anyways - the only hazard that
   * exists is if we attempt to send two packets from the same buffer simultaneously.
   */
  XIo_Out32(REGISTER_ADDRESS(ptp, PTP_TX_REG), 
            (PTP_TX_BUFFER(txBuffer) | PTP_TX_ENABLE));
}

/* Transmits the next ANNOUNCE message in a sequence */
void transmit_announce(struct ptp_device *ptp) {
  PtpTime presentTime;

  /* Update the sequence ID */
  set_sequence_id(ptp, PTP_TX_ANNOUNCE_BUFFER, ptp->announceSequenceId++);

  /* Update the origin timestamp with the present state of the RTC */
  get_rtc_time(ptp, &presentTime);
  set_timestamp(ptp, PTP_TX_ANNOUNCE_BUFFER, &presentTime);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, PTP_TX_ANNOUNCE_BUFFER);
}

/* Transmits the next SYNC message in a sequence */
void transmit_sync(struct ptp_device *ptp) {
  PtpTime presentTime;
  int64_t correctionField; 

  /* Update the sequence ID */
  set_sequence_id(ptp, PTP_TX_SYNC_BUFFER, ptp->syncSequenceId++);

  /* Update the origin timestamp with the present state of the RTC */
  get_rtc_time(ptp, &presentTime);
  set_timestamp(ptp, PTP_TX_SYNC_BUFFER, &presentTime);

  /* Update the correction field.
   * TODO: This should always be zero except if we are acting as a transparent clock 
   */
  correctionField = (int64_t) 0;
  correctionField <<= CORRECTION_FRACTION_BITS;
  update_correction_field(ptp, PTP_TX_SYNC_BUFFER, correctionField);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, PTP_TX_SYNC_BUFFER);
}

/* Transmits a FUP message related to the last SYNC message that was sent */
void transmit_fup(struct ptp_device *ptp) {
  PtpTime syncTxTimestamp;
  int64_t correctionField; 

  /* Copy the sequence ID from the last SYNC message we sent */
  set_sequence_id(ptp, PTP_TX_FUP_BUFFER, 
                  get_sequence_id(ptp, TRANSMITTED_PACKET, PTP_TX_SYNC_BUFFER));

  /* Update the precise origin timestamp with the hardware timestamp from when
   * the preceding SYNC was accepted into the MAC, augmented by the MAC's TX
   * latency.
   */
  get_hardware_timestamp(ptp, TRANSMITTED_PACKET, PTP_TX_SYNC_BUFFER, &syncTxTimestamp);

  /* TODO: Need to add in the MAC latency and the PHY latency! */
  set_timestamp(ptp, PTP_TX_FUP_BUFFER, &syncTxTimestamp);

  /* Update the correction field.
   * TODO: This should always be zero except if we are acting as a transparent clock 
   */
  correctionField = (int64_t) 0;
  correctionField <<= CORRECTION_FRACTION_BITS;
  update_correction_field(ptp, PTP_TX_SYNC_BUFFER, correctionField);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, PTP_TX_FUP_BUFFER);
}

/* Transmits the next DELAY_REQ message in a sequence */
void transmit_delay_request(struct ptp_device *ptp) {
  PtpTime presentTime;

  /* Update the sequence ID */
  set_sequence_id(ptp, PTP_TX_DELAY_REQ_BUFFER, ptp->delayReqSequenceId++);

  /* Update the origin timestamp with the present state of the RTC */
  get_rtc_time(ptp, &presentTime);
  set_timestamp(ptp, PTP_TX_DELAY_REQ_BUFFER, &presentTime);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, PTP_TX_DELAY_REQ_BUFFER);
}

/* Transmits a DELAY_RESP message in response to the DELAY_REQ message received
 * into the passed Rx packet buffer
 */
void transmit_delay_response(struct ptp_device *ptp, uint32_t requestRxBuffer) {
  PtpTime delayReqRxTimestamp;
  uint8_t requestPortId[PORT_ID_BYTES];
  uint16_t delayReqSequenceId;
  
  /* Get the source MAC address and sequence ID of the delay request */
  get_source_port_id(ptp, RECEIVED_PACKET, requestRxBuffer, requestPortId);
  delayReqSequenceId = get_sequence_id(ptp, RECEIVED_PACKET, requestRxBuffer);

  /* Update the sequence ID and requesting port identity */
  set_sequence_id(ptp, PTP_TX_DELAY_RESP_BUFFER, delayReqSequenceId);
  set_requesting_port_id(ptp, PTP_TX_DELAY_RESP_BUFFER, requestPortId);

  /* Update the requestReceiptTimestamp with the timestamp captured in the delay
   * request's Rx buffer
   */
  get_hardware_timestamp(ptp, RECEIVED_PACKET, requestRxBuffer, &delayReqRxTimestamp);
  set_timestamp(ptp, PTP_TX_DELAY_RESP_BUFFER, &delayReqRxTimestamp);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, PTP_TX_DELAY_RESP_BUFFER);
}

/* Transmits the next PDELAY_REQ message in a sequence */
void transmit_pdelay_request(struct ptp_device *ptp) {
  PtpTime presentTime;

  /* Update the sequence ID */
  set_sequence_id(ptp, PTP_TX_PDELAY_REQ_BUFFER, ptp->pdelayReqSequenceId++);

  /* Update the origin timestamp with the present state of the RTC */
  get_rtc_time(ptp, &presentTime);
  set_timestamp(ptp, PTP_TX_PDELAY_REQ_BUFFER, &presentTime);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, PTP_TX_PDELAY_REQ_BUFFER);
}

/* Transmits a PDELAY_RESP message in response to the PDELAY_REQ message received
 * into the passed Rx packet buffer
 */
void transmit_pdelay_response(struct ptp_device *ptp, uint32_t requestRxBuffer) {
  PtpTime pdelayReqRxTimestamp;
  uint16_t pdelayReqSequenceId;
  
  /* Get the source port identity address and sequence ID of the peer delay request */
  get_source_port_id(ptp, RECEIVED_PACKET, requestRxBuffer, ptp->lastPeerRequestPortId);
  pdelayReqSequenceId = get_sequence_id(ptp, RECEIVED_PACKET, requestRxBuffer);

  /* Update the sequence ID and requesting port identity */
  set_sequence_id(ptp, PTP_TX_PDELAY_RESP_BUFFER, pdelayReqSequenceId);
  set_requesting_port_id(ptp, PTP_TX_PDELAY_RESP_BUFFER, ptp->lastPeerRequestPortId);

  /* Update the requestReceiptTimestamp with the timestamp captured in the peer delay
   * request's Rx buffer
   */
  get_hardware_timestamp(ptp, RECEIVED_PACKET, requestRxBuffer, &pdelayReqRxTimestamp);
  set_timestamp(ptp, PTP_TX_PDELAY_RESP_BUFFER, &pdelayReqRxTimestamp);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, PTP_TX_PDELAY_RESP_BUFFER);
}

/* Transmits a PDELAY_RESP_FUP message related to the last PDELAY_RESP message that was
 * sent
 */
void transmit_pdelay_response_fup(struct ptp_device *ptp) {
  PtpTime pdelayRespTxTimestamp;

  /* Copy the sequence ID from the last PDELAY_RESP message we sent */
  set_sequence_id(ptp, PTP_TX_PDELAY_RESP_FUP_BUFFER, 
                  get_sequence_id(ptp, TRANSMITTED_PACKET, PTP_TX_PDELAY_RESP_BUFFER));

  /* Update the requesting port identity from the last peer delay request */
  set_requesting_port_id(ptp, PTP_TX_PDELAY_RESP_FUP_BUFFER, ptp->lastPeerRequestPortId);

  /* Update the precise origin timestamp with the hardware timestamp from when
   * the preceding PDELAY_RESP was accepted into the MAC, augmented by the MAC's TX
   * latency.
   */
  get_hardware_timestamp(ptp, TRANSMITTED_PACKET, PTP_TX_PDELAY_RESP_BUFFER, 
                         &pdelayRespTxTimestamp);

  /* TODO: Need to add in the MAC latency and the PHY latency! */
  set_timestamp(ptp, PTP_TX_PDELAY_RESP_FUP_BUFFER, &pdelayRespTxTimestamp);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, PTP_TX_PDELAY_RESP_FUP_BUFFER);
}

/**
 * Packet reception methods
 */

/* Returns the type of PTP message contained within the passed receive buffer, or 
 * PACKET_NOT_PTP if the buffer does not contain a valid PTP datagram.
 */
uint32_t get_message_type(struct ptp_device *ptp, uint32_t rxBuffer) {
  uint32_t bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;
  uint32_t messageType = PACKET_NOT_PTP;

  /* Fetch the word containing the LTF and the message type */
  bufferBase = PTP_RX_PACKET_BUFFER(ptp, rxBuffer);
  wordOffset = MESSAGE_TYPE_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);

  /* Check the LTF for the PTP Ethertype; if we are seriously falling behind in 
   * responding to received packets, it's possible we could be reading from the
   * present write buffer; which most likely is processing a non-PTP packet.
   */
  if(((packetWord >> 16) & LTF_MASK) == PTP_ETHERTYPE) {
    /* Good PTP packet, return its message type */
    messageType = ((packetWord >> 8) & MSG_TYPE_MASK);
  }

  return(messageType);
}

/* Extracts the contents of an ANNOUNCE message into the passed properties structure.
 * The message should already be ascertained to be an ANNOUNCE, this method will not
 * re-check.
 */
void extract_announce(struct ptp_device *ptp, uint32_t rxBuffer, PtpProperties *properties) {
  uint32_t bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  /* Extract the source address of the packet */
  bufferBase = PTP_RX_PACKET_BUFFER(ptp, rxBuffer);
  wordOffset = SOURCE_MAC_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  properties->sourceMacAddress[0] = ((packetWord >> 8) & 0x0FF);
  properties->sourceMacAddress[1] = (packetWord & 0x0FF);
  packetWord = read_packet(bufferBase, &wordOffset);
  properties->sourceMacAddress[2] = ((packetWord >> 24) & 0x0FF);
  properties->sourceMacAddress[3] = ((packetWord >> 16) & 0x0FF);
  properties->sourceMacAddress[4] = ((packetWord >> 8) & 0x0FF);
  properties->sourceMacAddress[5] = (packetWord & 0x0FF);

  /* Exract the domain number */
  wordOffset = DOMAIN_NUMBER_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  properties->domainNumber = ((packetWord >> 8) & 0x0FF);

  /* Extract the current UTC offset */
  wordOffset = UTC_OFFSET_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  properties->currentUtcOffset = (packetWord & 0x0FFFF);

  /* Extract the grandmaster priorities and clock quality */
  wordOffset = GM_PRIORITY1_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  properties->grandmasterPriority1 = ((packetWord >> 16) & 0x0FF);
  properties->grandmasterClockQuality.clockClass = ((packetWord >> 8) & 0x0FF);
  properties->grandmasterClockQuality.clockAccuracy = (packetWord & 0x0FF);
  packetWord = read_packet(bufferBase, &wordOffset);
  properties->grandmasterClockQuality.offsetScaledLogVariance = ((packetWord >> 16) & 0x0FFFF);
  properties->grandmasterPriority2 = ((packetWord >> 8) & 0x0FF);
  properties->grandmasterIdentity[0] = (packetWord & 0x0FF);
  packetWord = read_packet(bufferBase, &wordOffset);
  properties->grandmasterIdentity[1] = ((packetWord >> 24) & 0x0FF);
  properties->grandmasterIdentity[2] = ((packetWord >> 16) & 0x0FF);
  properties->grandmasterIdentity[3] = ((packetWord >> 8) & 0x0FF);
  properties->grandmasterIdentity[4] = (packetWord & 0x0FF);
  packetWord = read_packet(bufferBase, &wordOffset);
  properties->grandmasterIdentity[5] = ((packetWord >> 24) & 0x0FF);
  properties->grandmasterIdentity[6] = ((packetWord >> 16) & 0x0FF);
  properties->grandmasterIdentity[7] = ((packetWord >> 8) & 0x0FF);
  packetWord = read_packet(bufferBase, &wordOffset);
  properties->timeSource = ((packetWord >> 16) & 0x0FF);
}

/* Gets the MAC address from a received packet */
void get_rx_mac_address(struct ptp_device *ptp, uint32_t rxBuffer, uint8_t *macAddress) {
  uint32_t bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  /* Extract the source address of the packet */
  bufferBase = PTP_RX_PACKET_BUFFER(ptp, rxBuffer);
  wordOffset = SOURCE_MAC_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  macAddress[0] = ((packetWord >> 8) & 0x0FF);
  macAddress[1] = (packetWord & 0x0FF);
  packetWord = read_packet(bufferBase, &wordOffset);
  macAddress[2] = ((packetWord >> 24) & 0x0FF);
  macAddress[3] = ((packetWord >> 16) & 0x0FF);
  macAddress[4] = ((packetWord >> 8) & 0x0FF);
  macAddress[5] = (packetWord & 0x0FF);
}

/* Gets the source port identity from a received packet */
void get_source_port_id(struct ptp_device *ptp, PacketDirection bufferDirection,
                        uint32_t packetBuffer, uint8_t *sourcePortId) {
  uint32_t bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  /* Extract the source address of the packet */
  bufferBase = ((bufferDirection == TRANSMITTED_PACKET) ? 
                (PTP_TX_PACKET_BUFFER(ptp, packetBuffer) + TX_DATA_OFFSET) : 
                PTP_RX_PACKET_BUFFER(ptp, packetBuffer));
  wordOffset = SOURCE_PORT_ID_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  sourcePortId[0] = ((packetWord >> 8) & 0x0FF);
  sourcePortId[1] = (packetWord & 0x0FF);
  packetWord = read_packet(bufferBase, &wordOffset);
  sourcePortId[2] = ((packetWord >> 24) & 0x0FF);
  sourcePortId[3] = ((packetWord >> 16) & 0x0FF);
  sourcePortId[4] = ((packetWord >> 8) & 0x0FF);
  sourcePortId[5] = (packetWord & 0x0FF);
  packetWord = read_packet(bufferBase, &wordOffset);
  sourcePortId[6] = ((packetWord >> 24) & 0x0FF);
  sourcePortId[7] = ((packetWord >> 16) & 0x0FF);
  sourcePortId[8] = ((packetWord >> 8) & 0x0FF);
  sourcePortId[9] = (packetWord & 0x0FF);
}

/* Gets the requesting port identity from a received response packet
 * (either a DELAY_RESP, PDELAY_RESP, or PDELAY_RESP_FUP)
 */
void get_rx_requesting_port_id(struct ptp_device *ptp, uint32_t rxBuffer, 
                               uint8_t *requestingPortId) {
  uint32_t bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  /* Extract the source address of the packet */
  bufferBase = PTP_RX_PACKET_BUFFER(ptp, rxBuffer);
  wordOffset = REQ_PORT_ID_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  requestingPortId[0] = ((packetWord >> 8) & 0x0FF);
  requestingPortId[1] = (packetWord & 0x0FF);
  packetWord = read_packet(bufferBase, &wordOffset);
  requestingPortId[2] = ((packetWord >> 24) & 0x0FF);
  requestingPortId[3] = ((packetWord >> 16) & 0x0FF);
  requestingPortId[4] = ((packetWord >> 8) & 0x0FF);
  requestingPortId[5] = (packetWord & 0x0FF);
  packetWord = read_packet(bufferBase, &wordOffset);
  requestingPortId[6] = ((packetWord >> 24) & 0x0FF);
  requestingPortId[7] = ((packetWord >> 16) & 0x0FF);
  requestingPortId[8] = ((packetWord >> 8) & 0x0FF);
  requestingPortId[9] = (packetWord & 0x0FF);
}

/* Copies the set of properties */
void copy_ptp_properties(PtpProperties *to, PtpProperties *from) {
  uint32_t byteIndex;

  for(byteIndex = 0; byteIndex < MAC_ADDRESS_BYTES; byteIndex++) {
    to->sourceMacAddress[byteIndex] = from->sourceMacAddress[byteIndex];
  }
  to->domainNumber = from->domainNumber;
  to->currentUtcOffset = from->currentUtcOffset;
  to->grandmasterPriority1 = from->grandmasterPriority1;
  to->grandmasterClockQuality.clockClass = from->grandmasterClockQuality.clockClass;
  to->grandmasterClockQuality.clockAccuracy = from->grandmasterClockQuality.clockAccuracy;
  to->grandmasterClockQuality.offsetScaledLogVariance = from->grandmasterClockQuality.offsetScaledLogVariance;
  to->grandmasterPriority2 = from->grandmasterPriority2;
  for(byteIndex = 0; byteIndex < PTP_CLOCK_IDENTITY_CHARS; byteIndex++) {
    to->grandmasterIdentity[byteIndex] = from->grandmasterIdentity[byteIndex];
  }
  to->timeSource = from->timeSource;
}

int32_t compare_clock_identity(const uint8_t *clockIdentityA, const uint8_t *clockIdentityB)
{
  uint32_t byteIndex;
  int32_t comparisonResult = 0;

  for(byteIndex = 0; byteIndex < PTP_CLOCK_IDENTITY_CHARS; byteIndex++) {
    if(clockIdentityA[byteIndex] < clockIdentityB[byteIndex]) {
      comparisonResult = -1;
      break;
    } else if(clockIdentityA[byteIndex] > clockIdentityB[byteIndex]) {
      comparisonResult = 1;
      break;
    }
  }
  return(comparisonResult);
}

/* Compares the two passed MAC addresses; returns less than zero if the first MAC 
 * address is less than the second, positive if the converse is true, and zero if 
 * they are equal.
 */
int32_t compare_mac_addresses(const uint8_t *macAddressA, const uint8_t *macAddressB) {
  uint32_t byteIndex;
  int32_t comparisonResult = 0;

  for(byteIndex = 0; byteIndex < MAC_ADDRESS_BYTES; byteIndex++) {
    if(macAddressA[byteIndex] < macAddressB[byteIndex]) {
      comparisonResult = -1;
      break;
    } else if(macAddressA[byteIndex] > macAddressB[byteIndex]) {
      comparisonResult = 1;
      break;
    }
  }
  return(comparisonResult);
}

/* Compares the two passed port IDs; returns less than zero if the first MAC address
 * is less than the second, positive if the converse is true, and zero if they are 
 * equal.
 */
int32_t compare_port_ids(const uint8_t *portIdA, const uint8_t *portIdB) {
  uint32_t byteIndex;
  int32_t comparisonResult = 0;

  for(byteIndex = 0; byteIndex < PORT_ID_BYTES; byteIndex++) {
    if(portIdA[byteIndex] < portIdB[byteIndex]) {
      comparisonResult = -1;
      break;
    } else if(portIdA[byteIndex] > portIdB[byteIndex]) {
      comparisonResult = 1;
      break;
    }
  }
  return(comparisonResult);
}
