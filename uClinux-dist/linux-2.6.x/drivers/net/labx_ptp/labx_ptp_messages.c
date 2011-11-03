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

/**
 * Common helper methods
 */



uint8_t * get_output_buffer(struct ptp_device *ptp,uint32_t port,uint32_t bufType);
void write_packet(uint8_t *bufferBase, uint32_t *wordOffset, uint32_t writeWord);
uint32_t read_packet(uint8_t * bufferBase, uint32_t *wordOffset);
void transmit_packet(struct ptp_device *ptp, uint32_t port, uint8_t * txBuffer);

/**
 * Packet transmission methods
 */

/* Builds the common PTP header into the passed buffer using the passed word offset,
 * which should be set to zero prior to the call.
 */
static void init_ptp_header(struct ptp_device *ptp, uint32_t port, uint8_t *txBuffer, 
                            uint32_t *wordOffset, uint32_t messageType,
                            uint32_t messageLength, uint16_t headerFlags) {
  uint32_t packetWord;
  PtpPortProperties *portProperties = &ptp->ports[port].portProperties;

  /* Locate the requested buffer and begin with the packet's transmit length 
   * word.  This length word is always the length of the packet, in bytes,
   * minus one full port word, in bytes (which differs between 100M/1G and 10G).
   */
  *wordOffset = 0; 
  write_packet(txBuffer, wordOffset, (ETH_HEADER_BYTES + messageLength - TX_LENGTH_SUB(ptp)));

  /* Now begin at the Tx data base, which differs based upon port width */
  *wordOffset = TX_DATA_OFFSET(ptp);

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
    write_packet(txBuffer, wordOffset, 0x0180C200);
    packetWord = 0x000E0000;
    packetWord |= (portProperties->sourceMacAddress[0] << 8);
    packetWord |= portProperties->sourceMacAddress[1];
    write_packet(txBuffer, wordOffset, packetWord);
  } else {
    /* End-to-end (legacy PTP 2.0) */
    write_packet(txBuffer, wordOffset, 0x011B1900);
    packetWord = 0x00000000;
    packetWord |= (portProperties->sourceMacAddress[0] << 8);
    packetWord |= portProperties->sourceMacAddress[1];
    write_packet(txBuffer, wordOffset, packetWord);
  }
  packetWord = (portProperties->sourceMacAddress[2] << 24);
  packetWord |= (portProperties->sourceMacAddress[3] << 16);
  packetWord |= (portProperties->sourceMacAddress[4] << 8);
  packetWord |= portProperties->sourceMacAddress[5];
  write_packet(txBuffer, wordOffset, packetWord);

  /* PTP EtherType and the first two PTP header bytes */
  packetWord = (PTP_ETHERTYPE << 16);
  packetWord |= (TS_SPEC_ETH_AVB << 12);
  packetWord |= ((messageType & MSG_TYPE_MASK) << 8);
  packetWord |= PTP_VERSION_2_0;
  write_packet(txBuffer, wordOffset, packetWord);

  /* Message length, domain number, and reserved field */
  packetWord = ((messageLength & MSG_LENGTH_MASK) << 16);
  packetWord |= ((ptp->properties.domainNumber & DOMAIN_NUM_MASK) << 8);
  write_packet(txBuffer, wordOffset, packetWord);

  /* Assign the passed flags into the flag field, clear the correction field */
  packetWord = (headerFlags << 16);
  write_packet(txBuffer, wordOffset, packetWord);

  /* Clear out the flag field, correction field, four reserved bytes,
   * and set the 80-bit source ID to be our clockIdentity, port zero.
   * clockIdentity is formed by our OUI, 0xFFFE, and the serial number.
   * OUI and serial number are the first three and last three bytes of
   * our MAC address, respectively.
   */
  write_packet(txBuffer, wordOffset, 0x00000000);
  write_packet(txBuffer, wordOffset, 0x00000000);
  packetWord = (ptp->properties.grandmasterIdentity[0] << 8);
  packetWord |= ptp->properties.grandmasterIdentity[1];
  write_packet(txBuffer, wordOffset, packetWord);
  packetWord = (ptp->properties.grandmasterIdentity[2] << 24);
  packetWord |= (ptp->properties.grandmasterIdentity[3] << 16);
  packetWord |= (ptp->properties.grandmasterIdentity[4] << 8);
  packetWord |= ptp->properties.grandmasterIdentity[5];
  write_packet(txBuffer, wordOffset, packetWord);
  packetWord = (ptp->properties.grandmasterIdentity[6] << 24);
  packetWord |= (ptp->properties.grandmasterIdentity[7] << 16);
  write_packet(txBuffer, wordOffset, packetWord);

  /* Clear the sequence ID and log message interval to zero, and set the
   * control field appropriately for the message type.
   */
  switch(messageType) {
  case MSG_SYNC:
    write_packet(txBuffer, wordOffset, 0x00000000);
    break;

  case MSG_DELAY_REQ:
    write_packet(txBuffer, wordOffset, 0x00000100);
    break;

  case MSG_FUP:
    write_packet(txBuffer, wordOffset, 0x00000200);
    break;

  case MSG_DELAY_RESP:
    write_packet(txBuffer, wordOffset, 0x00000300);
    break;

  case MSG_MANAGEMENT:
    write_packet(txBuffer, wordOffset, 0x00000400);
    break;

  default:
    write_packet(txBuffer, wordOffset, 0x00000500);
  }
}

/* Initializes the ANNOUNCE message transmit template */
static void init_announce_template(struct ptp_device *ptp, uint32_t port) {
  uint8_t *txBuffer;
  uint32_t wordOffset;
  uint32_t packetWord;
  PtpClockQuality *quality;

  /* Clear out all dynamic fields and populate static ones with properties from
   * the PTP device structure.
   */
  txBuffer = get_output_buffer(ptp,port,PTP_TX_ANNOUNCE_BUFFER);
  init_ptp_header(ptp, port, txBuffer, &wordOffset, MSG_ANNOUNCE, 
                  PTP_ANNOUNCE_LENGTH + TLV_HEADER_LENGTH + PATH_TRACE_TLV_LENGTH(1),
                  (uint16_t) (FLAG_TWO_STEP|FLAG_PTP_TIMESCALE));

  /* Clear originTimestamp and set currentUtcOffset */
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  packetWord = (ptp->properties.currentUtcOffset & 0x0000FFFF);
  write_packet(txBuffer, &wordOffset, packetWord);

  /* Clear reserved, and set the grandmaster properties.  Initialize the stepsRemoved
   * field to zero since the packet's source.
   */
  quality = &ptp->properties.grandmasterClockQuality;
  packetWord = (ptp->properties.grandmasterPriority1 << 16);
  packetWord |= (quality->clockClass << 8);
  packetWord |= quality->clockAccuracy;
  write_packet(txBuffer, &wordOffset, packetWord);
  packetWord = (quality->offsetScaledLogVariance << 16);
  packetWord |= (ptp->properties.grandmasterPriority2 << 8);
  packetWord |= ptp->properties.grandmasterIdentity[0];
  write_packet(txBuffer, &wordOffset, packetWord);
  packetWord = ((ptp->properties.grandmasterIdentity[1] << 24) | 
                (ptp->properties.grandmasterIdentity[2] << 16) | 
                (ptp->properties.grandmasterIdentity[3] << 8)  |
                ptp->properties.grandmasterIdentity[4]);
  write_packet(txBuffer, &wordOffset, packetWord);
  packetWord = ((ptp->properties.grandmasterIdentity[5] << 24) | 
                (ptp->properties.grandmasterIdentity[6] << 16) | 
                (ptp->properties.grandmasterIdentity[7] << 8));
  write_packet(txBuffer, &wordOffset, packetWord);
  packetWord = (ptp->properties.timeSource << 16) | PATH_TRACE_TLV_TYPE;
  write_packet(txBuffer, &wordOffset, packetWord);

  /* Add the path trace TLV info. TODO: If we are ever forwarding
     announce packets then this needs to be modified to include
     incoming path entries. As a master we just need to have ours. */
  packetWord = ((1/*entries*/*8) << 16) |
               (ptp->properties.grandmasterIdentity[0] << 8) |
               ptp->properties.grandmasterIdentity[1];
  write_packet(txBuffer, &wordOffset, packetWord);
  packetWord = (ptp->properties.grandmasterIdentity[2] << 24) |
               (ptp->properties.grandmasterIdentity[3] << 16) |
               (ptp->properties.grandmasterIdentity[4] << 8) |
               ptp->properties.grandmasterIdentity[5];
  write_packet(txBuffer, &wordOffset, packetWord);
  packetWord = (ptp->properties.grandmasterIdentity[6] << 24) |
               (ptp->properties.grandmasterIdentity[7] << 16);
  write_packet(txBuffer, &wordOffset, packetWord);
}

/* Initializes the SYNC message transmit template */
static void init_sync_template(struct ptp_device *ptp, uint32_t port) {
  uint8_t * txBuffer;
  uint32_t wordOffset;

  /* Initialize the header, and clear the originTimestamp for good measure. */
  txBuffer = get_output_buffer(ptp,port,PTP_TX_SYNC_BUFFER);
  init_ptp_header(ptp, port, txBuffer, &wordOffset, MSG_SYNC, PTP_SYNC_LENGTH,
                  (uint16_t) FLAG_TWO_STEP);
  txBuffer = get_output_buffer(ptp, port, PTP_TX_SYNC_BUFFER);
  write_packet(txBuffer , &wordOffset, 0x00000000);
  write_packet(txBuffer , &wordOffset, 0x00000000);
  write_packet(txBuffer , &wordOffset, 0x00000000);
}

/* Initializes the FUP message transmit template */
static void init_fup_template(struct ptp_device *ptp, uint32_t port) {
  uint8_t * txBuffer;
  uint32_t wordOffset;
  uint32_t packetWord;

  /* Initialize the header, and clear the preciseOriginTimestamp for good measure */
  txBuffer = get_output_buffer(ptp, port, PTP_TX_FUP_BUFFER);
  init_ptp_header(ptp, port, txBuffer, &wordOffset, MSG_FUP,
                  PTP_FUP_LENGTH + TLV_HEADER_LENGTH + FOLLOW_UP_INFORMATION_TLV_LENGTH,
                  (uint16_t) FLAG_TWO_STEP);

  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  packetWord = ORGANIZATION_EXTENSION_TLV_TYPE;
  write_packet(txBuffer, &wordOffset, packetWord);

  /* Add the follow-up information TLV */
  packetWord = (FOLLOW_UP_INFORMATION_TLV_LENGTH << 16) | 0x0080;
  write_packet(txBuffer, &wordOffset, packetWord);
  write_packet(txBuffer, &wordOffset, 0xC2000001);
  write_packet(txBuffer, &wordOffset, 0x00000000); /* TODO: rate ratio is 1.0 unless we can forward */
  write_packet(txBuffer, &wordOffset, 0x00000000); /* TODO: gmTimeBaseIndicator goes in 0xFFFF0000 */
  write_packet(txBuffer, &wordOffset, 0x00000000); /* TODO: lastGmPhaseChange*/
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000); /* TODO: scaledLastGmFreqChange */
}

/* Initializes the DELAY_REQ message transmit template */
static void init_delay_request_template(struct ptp_device *ptp, uint32_t port) {
  uint8_t * txBuffer;
  uint32_t wordOffset;

  /* Initialize the header, and clear the originTimestamp for good measure */
  txBuffer = get_output_buffer(ptp,port,PTP_TX_DELAY_REQ_BUFFER);
  init_ptp_header(ptp, port, txBuffer, &wordOffset, MSG_DELAY_REQ, 
                  PTP_DELAY_REQ_LENGTH, (uint16_t) FLAG_TWO_STEP);

  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
}

/* Initializes the DELAY_RESP message transmit template */
static void init_delay_response_template(struct ptp_device *ptp, uint32_t port) {
  uint8_t *txBuffer;
  uint32_t wordOffset;

  /* Initialize the header, and clear the requestReceiptTimestamp and
   * requestingPortIdentity for good measure 
   */
  txBuffer = get_output_buffer(ptp,port,PTP_TX_DELAY_RESP_BUFFER);
  init_ptp_header(ptp, port, txBuffer, &wordOffset, MSG_DELAY_RESP,
                  PTP_DELAY_RESP_LENGTH, (uint16_t) FLAG_TWO_STEP);

  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
}

/* Initializes the PDELAY_REQ message transmit template */
static void init_pdelay_request_template(struct ptp_device *ptp, uint32_t port) {
  uint8_t *txBuffer;
  uint32_t wordOffset;

  /* Initialize the header, and clear the originTimestamp for good measure.
   * Also clear out the reserved bytes at the end of the message; these are
   * set to zero and are sized to match that of the PDELAY_RESP message in
   * an attempt to minimize path asymmetry through bridges.
   */
  txBuffer = get_output_buffer(ptp,port,PTP_TX_PDELAY_REQ_BUFFER);
  init_ptp_header(ptp, port, txBuffer, &wordOffset, MSG_PDELAY_REQ, 
                  PTP_PDELAY_REQ_LENGTH, (uint16_t) FLAG_TWO_STEP);

  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
}

/* Initializes the PDELAY_RESP message transmit template */
static void init_pdelay_response_template(struct ptp_device *ptp, uint32_t port) {
  uint8_t * txBuffer ;
  uint32_t wordOffset;

  /* Initialize the header, and clear the requestReceiptTimestamp and
   * requestingPortIdentity for good measure 
   */
  txBuffer = get_output_buffer(ptp,port,PTP_TX_PDELAY_RESP_BUFFER);
  init_ptp_header(ptp, port, txBuffer, &wordOffset, MSG_PDELAY_RESP,
                  PTP_PDELAY_RESP_LENGTH, (uint16_t) FLAG_TWO_STEP);

  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
}

/* Initializes the PDELAY_RESP_FUP message transmit template */
static void init_pdelay_response_fup_template(struct ptp_device *ptp, uint32_t port) {
  uint8_t *txBuffer;
  uint32_t wordOffset;

  /* Initialize the header, and clear the responseOriginTimestamp and
   * requestingPortIdentity for good measure 
   */
  txBuffer = get_output_buffer(ptp,port,PTP_TX_PDELAY_RESP_FUP_BUFFER);
  init_ptp_header(ptp, port, txBuffer, &wordOffset, 
                  MSG_PDELAY_RESP_FUP, PTP_PDELAY_RESP_FUP_LENGTH, 
                  (uint16_t) FLAG_TWO_STEP);

  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
}

/* Initializes the transmit packet buffers with their packet templates */
void init_tx_templates(struct ptp_device *ptp, uint32_t port) {
  /* Build the templates dynamically; while this could be done with initialized
   * constant data, this approach is more flexible and illustrates the packet
   * structure better.
   */
  init_announce_template(ptp, port);
  init_sync_template(ptp, port);
  init_fup_template(ptp, port);
  init_delay_request_template(ptp, port);
  init_delay_response_template(ptp, port);
  init_pdelay_request_template(ptp, port);
  init_pdelay_response_template(ptp, port);
  init_pdelay_response_fup_template(ptp, port);
}

/* Sets the sequence ID within the passed packet buffer */
static void set_sequence_id(struct ptp_device *ptp, uint32_t port, uint8_t * txBuffer, 
                            uint16_t sequenceId) {
  uint8_t * bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  /* Read, modify, and write back the sequence ID */
  bufferBase = txBuffer + TX_DATA_OFFSET(ptp);

  /* Locate the sequence ID in the packet */
  wordOffset = SEQUENCE_ID_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  packetWord &= 0x0000FFFF;
  packetWord |= (sequenceId << 16);
  wordOffset = SEQUENCE_ID_OFFSET;
  write_packet(bufferBase, &wordOffset, packetWord);
}

/* Gets the sequence ID within the passed packet buffer */
uint16_t get_sequence_id(struct ptp_device *ptp, uint32_t port, PacketDirection bufferDirection,
                         uint8_t * packetBuffer) {
  uint8_t *bufferBase;
  uint32_t wordOffset;

  /* Read and return the sequence ID */
  bufferBase = (bufferDirection == TRANSMITTED_PACKET) ? packetBuffer + TX_DATA_OFFSET(ptp) : packetBuffer; 
                

  /* Locate the sequence ID in the packet */
  wordOffset = SEQUENCE_ID_OFFSET;
  return((uint16_t) (read_packet(bufferBase, &wordOffset) >> 16));
}

/* Gets the message timestamp (e.g. originTimestamp) within the passed packet buffer */
void get_timestamp(struct ptp_device *ptp, uint32_t port, PacketDirection bufferDirection,
                   uint8_t * packetBuffer, PtpTime *timestamp) {
  uint8_t * bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  bufferBase = (bufferDirection == TRANSMITTED_PACKET) ? packetBuffer + TX_DATA_OFFSET(ptp) : packetBuffer;

  /* Locate the timestamp in the packet */
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
void get_correction_field(struct ptp_device *ptp, uint32_t port, uint8_t * rxBuffer, PtpTime *correctionField) {
  uint32_t wordOffset;
  uint32_t packetWord;
  int64_t rawField;

  /* Presume we are dealing with less than a second of correction */
  correctionField->secondsUpper = 0;
  correctionField->secondsLower = 0;
  wordOffset = CORRECTION_FIELD_OFFSET;
  packetWord = read_packet(rxBuffer, &wordOffset);
  rawField = ((int64_t) (packetWord & 0x0000FFFF) << 48);
  packetWord = read_packet(rxBuffer, &wordOffset);
  rawField |= (((int64_t) packetWord) << 16);
  packetWord = read_packet(rxBuffer, &wordOffset);
  rawField |= (int64_t) ((packetWord & 0xFFFF0000) >> 16);
  correctionField->nanoseconds = (uint32_t) (rawField >> CORRECTION_FRACTION_BITS);
}

/* Get the cumulative scaled rate offset from the follow-up TLV */
uint32_t get_cumulative_scaled_rate_offset_field(uint8_t *rxBuffer) {
  uint32_t wordOffset = CUMULATIVE_SCALED_RATE_OFFSET_OFFSET;
  return read_packet(rxBuffer, &wordOffset);
}

/* Sets the message timestamp (e.g. originTimestamp) within the passed packet buffer */
static void set_timestamp(struct ptp_device *ptp, uint32_t port, uint8_t * txBuffer, PtpTime *timestamp) {
  uint8_t * bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  bufferBase = txBuffer + TX_DATA_OFFSET(ptp);

  /* Locate the timestamp in the packet */
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
static void update_correction_field(struct ptp_device *ptp, 
                                    uint32_t port, 
                                    uint8_t *txBuffer,
                                    int64_t correctionField) {
  uint8_t *bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  bufferBase =  txBuffer + TX_DATA_OFFSET(ptp);

  /* Locate the correction field in the packet */
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
static void set_requesting_port_id(struct ptp_device *ptp, 
                                   uint32_t port, 
                                   uint8_t *txBuffer,
                                   uint8_t *requestPortId) {
  uint8_t * bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  bufferBase = txBuffer + TX_DATA_OFFSET(ptp);

  /* Locate the requesting port ID in the packet */
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

void print_packet_buffer(struct ptp_device *ptp,
                         uint32_t port, 
                         PacketDirection bufferDirection,
                         uint8_t *packetBuffer,
                         uint32_t packetWords) {
  uint8_t * bufferBase;
  uint32_t wordOffset;
  uint32_t wordIndex;

  /* Print the entire contents of the packet buffer, skipping the length for Tx
   * buffers
   */
  bufferBase = (bufferDirection == TRANSMITTED_PACKET) ? packetBuffer + TX_DATA_OFFSET(ptp):packetBuffer;
                
  wordOffset = 0;
  for(wordIndex = 0; wordIndex < packetWords; wordIndex++) {
    printk("0x%08X\n", read_packet(bufferBase, &wordOffset));
  }
}

/* Transmits the next ANNOUNCE message in a sequence */
void transmit_announce(struct ptp_device *ptp, uint32_t port) {
  PtpTime presentTime;
  uint8_t *txBuffer;

  /* Update the sequence ID */
  txBuffer = get_output_buffer(ptp,port,PTP_TX_ANNOUNCE_BUFFER);
  set_sequence_id(ptp, port, txBuffer, ptp->ports[port].announceSequenceId++);

  /* Update the origin timestamp with the present state of the RTC */
  get_rtc_time(ptp, &presentTime);
  set_timestamp(ptp, port, txBuffer, &presentTime);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, port, txBuffer);
  ptp->ports[port].stats.txAnnounceCount++;
}

/* Transmits the next SYNC message in a sequence */
void transmit_sync(struct ptp_device *ptp, uint32_t port) {
  PtpTime presentTime;
  int64_t correctionField;
  uint8_t *txBuffer; 

  /* Update the sequence ID */
  txBuffer = get_output_buffer(ptp,port,PTP_TX_SYNC_BUFFER);
  set_sequence_id(ptp, port, txBuffer, ptp->ports[port].syncSequenceId++);

  /* Update the origin timestamp with the present state of the RTC */
  get_rtc_time(ptp, &presentTime);
  set_timestamp(ptp, port, txBuffer, &presentTime);

  /* Update the correction field.
   * TODO: This should always be zero except if we are acting as a transparent clock 
   */
  correctionField = (int64_t) 0;
  correctionField <<= CORRECTION_FRACTION_BITS;
  update_correction_field(ptp, port, txBuffer , correctionField);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, port, txBuffer);
  ptp->ports[port].stats.txSyncCount++;
}

/* Transmits a FUP message related to the last SYNC message that was sent */
void transmit_fup(struct ptp_device *ptp, uint32_t port) {
  uint8_t *txFupBuffer;
  uint8_t *txSyncBuffer;
  PtpTime syncTxTimestamp;
  int64_t correctionField; 

  txFupBuffer = get_output_buffer(ptp,port,PTP_TX_FUP_BUFFER);
  txSyncBuffer= get_output_buffer(ptp,port,PTP_TX_SYNC_BUFFER);
  /* Copy the sequence ID from the last SYNC message we sent */
  set_sequence_id(ptp, port, txFupBuffer, 
                  get_sequence_id(ptp, port, TRANSMITTED_PACKET, txSyncBuffer));

  /* Update the precise origin timestamp with the hardware timestamp from when
   * the preceding SYNC was accepted into the MAC, augmented by the MAC's TX
   * latency.
   */
  get_hardware_timestamp(ptp, port, TRANSMITTED_PACKET, txSyncBuffer, &syncTxTimestamp);

  set_timestamp(ptp, port, txFupBuffer, &syncTxTimestamp);

  /* Update the correction field.
   * TODO: This should always be zero except if we are acting as a transparent clock 
   */
  correctionField = (int64_t) 0;
  correctionField <<= CORRECTION_FRACTION_BITS;
  update_correction_field(ptp, port, txSyncBuffer, correctionField);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, port, txFupBuffer);
  ptp->ports[port].stats.txFollowupCount++;
}

/* Transmits the next DELAY_REQ message in a sequence */
void transmit_delay_request(struct ptp_device *ptp, uint32_t port) {
  PtpTime presentTime;
  uint8_t *txBuffer;

  /* Update the sequence ID */
  txBuffer = get_output_buffer(ptp,port,PTP_TX_DELAY_REQ_BUFFER);
  set_sequence_id(ptp, port, txBuffer, ptp->ports[port].delayReqSequenceId++);

  /* Update the origin timestamp with the present state of the RTC */
  get_rtc_time(ptp, &presentTime);
  set_timestamp(ptp, port, txBuffer, &presentTime);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, port, txBuffer);
}

/* Transmits a DELAY_RESP message in response to the DELAY_REQ message received
 * into the passed Rx packet buffer
 */
void transmit_delay_response(struct ptp_device *ptp, uint32_t port, uint8_t * requestRxBuffer) {
  PtpTime delayReqRxTimestamp;
  uint8_t requestPortId[PORT_ID_BYTES];
  uint16_t delayReqSequenceId;
  uint8_t *txBuffer;
  
  /* Get the source MAC address and sequence ID of the delay request */
  get_source_port_id(ptp, port, RECEIVED_PACKET, requestRxBuffer, requestPortId);
  delayReqSequenceId = get_sequence_id(ptp, port, RECEIVED_PACKET, requestRxBuffer);

  /* Update the sequence ID and requesting port identity */
  txBuffer = get_output_buffer(ptp,port,PTP_TX_DELAY_RESP_BUFFER);
  set_sequence_id(ptp, port, txBuffer, delayReqSequenceId);
  set_requesting_port_id(ptp, port, txBuffer, requestPortId);

  /* Update the requestReceiptTimestamp with the timestamp captured in the delay
   * request's Rx buffer
   */
  get_hardware_timestamp(ptp, port, RECEIVED_PACKET, requestRxBuffer, &delayReqRxTimestamp);

  set_timestamp(ptp, port, txBuffer, &delayReqRxTimestamp);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, port, txBuffer);
}

/* Transmits the next PDELAY_REQ message in a sequence */
void transmit_pdelay_request(struct ptp_device *ptp, uint32_t port) {
  PtpTime presentTime;
  uint8_t *txBuffer;

  txBuffer = get_output_buffer(ptp,port,PTP_TX_PDELAY_REQ_BUFFER);
  /* Update the sequence ID */
  set_sequence_id(ptp, port, txBuffer, ptp->ports[port].pdelayReqSequenceId++);

  /* Update the origin timestamp with the present state of the RTC */
  get_rtc_time(ptp, &presentTime);
  set_timestamp(ptp, port, txBuffer, &presentTime);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, port, txBuffer);
  ptp->ports[port].stats.txPDelayRequestCount++;
}

/* Transmits a PDELAY_RESP message in response to the PDELAY_REQ message received
 * into the passed Rx packet buffer
 */
void transmit_pdelay_response(struct ptp_device *ptp, uint32_t port, uint8_t * requestRxBuffer) {
  PtpTime pdelayReqRxTimestamp;
  uint16_t pdelayReqSequenceId;
  uint8_t *txBuffer;
  
  /* Get the source port identity address and sequence ID of the peer delay request */
  get_source_port_id(ptp, port, RECEIVED_PACKET, requestRxBuffer, ptp->ports[port].lastPeerRequestPortId);
  pdelayReqSequenceId = get_sequence_id(ptp, port, RECEIVED_PACKET, requestRxBuffer);

  /* Update the sequence ID and requesting port identity */
  txBuffer = get_output_buffer(ptp, port, PTP_TX_PDELAY_RESP_BUFFER);
  set_sequence_id(ptp, port, txBuffer, pdelayReqSequenceId);
  set_requesting_port_id(ptp, port, txBuffer, ptp->ports[port].lastPeerRequestPortId);

  /* Update the requestReceiptTimestamp with the timestamp captured in the peer delay
   * request's Rx buffer
   */
  get_hardware_timestamp(ptp, port, RECEIVED_PACKET, requestRxBuffer, &pdelayReqRxTimestamp);

  set_timestamp(ptp, port, txBuffer, &pdelayReqRxTimestamp);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, port, txBuffer);
  ptp->ports[port].stats.txPDelayResponseCount++;
}

/* Transmits a PDELAY_RESP_FUP message related to the last PDELAY_RESP message that was
 * sent
 */
void transmit_pdelay_response_fup(struct ptp_device *ptp, uint32_t port) {
  PtpTime pdelayRespTxTimestamp;
  uint8_t *txFupBuffer;
  uint8_t *txRespBuffer;

  txFupBuffer = get_output_buffer(ptp,port,PTP_TX_PDELAY_RESP_FUP_BUFFER);
  txRespBuffer = get_output_buffer(ptp,port,PTP_TX_PDELAY_RESP_BUFFER);

  /* Copy the sequence ID from the last PDELAY_RESP message we sent */
  set_sequence_id(ptp, port, txFupBuffer, 
                  get_sequence_id(ptp, port, TRANSMITTED_PACKET, txRespBuffer));

  /* Update the requesting port identity from the last peer delay request */
  set_requesting_port_id(ptp, port, txFupBuffer, ptp->ports[port].lastPeerRequestPortId);

  /* Update the precise origin timestamp with the hardware timestamp from when
   * the preceding PDELAY_RESP was accepted into the MAC, augmented by the MAC's TX
   * latency.
   */
  get_hardware_timestamp(ptp, port, TRANSMITTED_PACKET, txRespBuffer, 
                         &pdelayRespTxTimestamp);

  set_timestamp(ptp, port, txFupBuffer, &pdelayRespTxTimestamp);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, port, txFupBuffer);
  ptp->ports[port].stats.txPDelayResponseFollowupCount++;
}

/**
 * Packet reception methods
 */

/* Returns the type of PTP message contained within the passed receive buffer, or 
 * PACKET_NOT_PTP if the buffer does not contain a valid PTP datagram.
 */
uint32_t get_message_type(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer) {
  uint32_t wordOffset;
  uint32_t packetWord;
  uint32_t messageType = PACKET_NOT_PTP;

  /* Fetch the word containing the LTF and the message type */
  wordOffset = MESSAGE_TYPE_OFFSET;
  packetWord = read_packet(rxBuffer, &wordOffset);

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
void extract_announce(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer,
  PtpProperties *properties, PtpPortProperties *portProperties) {

  uint32_t wordOffset;
  uint32_t packetWord;

  /* Extract the source address of the packet */
  wordOffset = SOURCE_MAC_OFFSET;
  packetWord = read_packet(rxBuffer, &wordOffset);
  portProperties->sourceMacAddress[0] = ((packetWord >> 8) & 0x0FF);
  portProperties->sourceMacAddress[1] = (packetWord & 0x0FF);
  packetWord = read_packet(rxBuffer, &wordOffset);
  portProperties->sourceMacAddress[2] = ((packetWord >> 24) & 0x0FF);
  portProperties->sourceMacAddress[3] = ((packetWord >> 16) & 0x0FF);
  portProperties->sourceMacAddress[4] = ((packetWord >> 8) & 0x0FF);
  portProperties->sourceMacAddress[5] = (packetWord & 0x0FF);

  /* Exract the domain number */
  wordOffset = DOMAIN_NUMBER_OFFSET;
  packetWord = read_packet(rxBuffer, &wordOffset);
  properties->domainNumber = ((packetWord >> 8) & 0x0FF);

  /* Extract the current UTC offset */
  wordOffset = UTC_OFFSET_OFFSET;
  packetWord = read_packet(rxBuffer, &wordOffset);
  properties->currentUtcOffset = (packetWord & 0x0FFFF);

  /* Extract the grandmaster priorities and clock quality */
  wordOffset = GM_PRIORITY1_OFFSET;
  packetWord = read_packet(rxBuffer, &wordOffset);
  properties->grandmasterPriority1 = ((packetWord >> 16) & 0x0FF);
  properties->grandmasterClockQuality.clockClass = ((packetWord >> 8) & 0x0FF);
  properties->grandmasterClockQuality.clockAccuracy = (packetWord & 0x0FF);
  packetWord = read_packet(rxBuffer, &wordOffset);
  properties->grandmasterClockQuality.offsetScaledLogVariance = ((packetWord >> 16) & 0x0FFFF);
  properties->grandmasterPriority2 = ((packetWord >> 8) & 0x0FF);
  properties->grandmasterIdentity[0] = (packetWord & 0x0FF);
  packetWord = read_packet(rxBuffer, &wordOffset);
  properties->grandmasterIdentity[1] = ((packetWord >> 24) & 0x0FF);
  properties->grandmasterIdentity[2] = ((packetWord >> 16) & 0x0FF);
  properties->grandmasterIdentity[3] = ((packetWord >> 8) & 0x0FF);
  properties->grandmasterIdentity[4] = (packetWord & 0x0FF);
  packetWord = read_packet(rxBuffer, &wordOffset);
  properties->grandmasterIdentity[5] = ((packetWord >> 24) & 0x0FF);
  properties->grandmasterIdentity[6] = ((packetWord >> 16) & 0x0FF);
  properties->grandmasterIdentity[7] = ((packetWord >> 8) & 0x0FF);
  packetWord = read_packet(rxBuffer, &wordOffset);
  properties->timeSource = ((packetWord >> 16) & 0x0FF);

  /* Port number is 1 based and is the port it came in on */
  portProperties->portNumber = port + 1;
  portProperties->stepsRemoved = 0; /* TODO - where do we get this from? */
}

/* Gets the MAC address from a received packet */
void get_rx_mac_address(struct ptp_device *ptp, uint32_t port, uint8_t * rxBuffer, uint8_t *macAddress) {
  uint32_t wordOffset;
  uint32_t packetWord;

  /* Extract the source address of the packet */
  wordOffset = SOURCE_MAC_OFFSET;
  packetWord = read_packet(rxBuffer, &wordOffset);
  macAddress[0] = ((packetWord >> 8) & 0x0FF);
  macAddress[1] = (packetWord & 0x0FF);
  packetWord = read_packet(rxBuffer, &wordOffset);
  macAddress[2] = ((packetWord >> 24) & 0x0FF);
  macAddress[3] = ((packetWord >> 16) & 0x0FF);
  macAddress[4] = ((packetWord >> 8) & 0x0FF);
  macAddress[5] = (packetWord & 0x0FF);
}

/* Gets the source port identity from a received packet */
void get_source_port_id(struct ptp_device *ptp, uint32_t port, PacketDirection bufferDirection,
                        uint8_t * packetBuffer, uint8_t *sourcePortId) {
  uint8_t *bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  /* Extract the source address of the packet */
  bufferBase = (bufferDirection == TRANSMITTED_PACKET) ? packetBuffer + TX_DATA_OFFSET(ptp) : packetBuffer;

  /* Locate the source port ID in the packet */
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
void get_rx_requesting_port_id(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer, 
                               uint8_t *requestingPortId) {
  uint8_t * bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  /* Extract the source address of the packet */
  bufferBase = rxBuffer;
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

  to->domainNumber = from->domainNumber;
  to->currentUtcOffset = from->currentUtcOffset;
  to->grandmasterPriority1 = from->grandmasterPriority1;
  to->grandmasterClockQuality.clockClass = from->grandmasterClockQuality.clockClass;
  to->grandmasterClockQuality.clockAccuracy = from->grandmasterClockQuality.clockAccuracy;
  to->grandmasterClockQuality.offsetScaledLogVariance = from->grandmasterClockQuality.offsetScaledLogVariance;
  to->grandmasterPriority2 = from->grandmasterPriority2;
  for(byteIndex = 0; byteIndex < PTP_CLOCK_IDENTITY_BYTES; byteIndex++) {
    to->grandmasterIdentity[byteIndex] = from->grandmasterIdentity[byteIndex];
  }
  to->timeSource = from->timeSource;
}

void copy_ptp_port_properties(PtpPortProperties *to, PtpPortProperties *from) {
  uint32_t byteIndex;

  to->portNumber = from->portNumber;
  
  for(byteIndex = 0; byteIndex < MAC_ADDRESS_BYTES; byteIndex++) {
    to->sourceMacAddress[byteIndex] = from->sourceMacAddress[byteIndex];
  }

  to->stepsRemoved = from->stepsRemoved;
}

int32_t compare_clock_identity(const uint8_t *clockIdentityA, const uint8_t *clockIdentityB)
{
  uint32_t byteIndex;
  int32_t comparisonResult = 0;

  for(byteIndex = 0; byteIndex < PTP_CLOCK_IDENTITY_BYTES; byteIndex++) {
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
