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
   * and set the 80-bit source ID to be our clockIdentity, and port number.
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
  packetWord |= port + 1;
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

  case MSG_PDELAY_RESP:
  case MSG_PDELAY_RESP_FUP:
    write_packet(txBuffer, wordOffset, 0x0000057F);
    break;

  default:
    write_packet(txBuffer, wordOffset, 0x00000500);
  }
}

/* Initializes the ANNOUNCE message transmit template */
static void init_announce_template(struct ptp_device *ptp, uint32_t port, PtpPriorityVector *pv) {
  uint8_t *txBuffer;
  uint32_t wordOffset;
  uint32_t packetWord;
  uint32_t i;
  PtpSystemIdentity *identity = &pv->rootSystemIdentity;

  /* Clear out all dynamic fields and populate static ones with properties from
   * the PTP device structure.
   */
  txBuffer = get_output_buffer(ptp,port,PTP_TX_ANNOUNCE_BUFFER);
  init_ptp_header(ptp, port, txBuffer, &wordOffset, MSG_ANNOUNCE,
                  PTP_ANNOUNCE_LENGTH + TLV_HEADER_LENGTH + PATH_TRACE_TLV_LENGTH(ptp->pathTraceLength),
                  (uint16_t) (FLAG_PTP_TIMESCALE));

  /* Clear originTimestamp and set currentUtcOffset */
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  packetWord = (ptp->properties.currentUtcOffset & 0x0000FFFF);
  write_packet(txBuffer, &wordOffset, packetWord);

  /* Clear reserved, and set the grandmaster properties. */
  packetWord = (identity->priority1 << 16);
  packetWord |= (identity->clockClass << 8);
  packetWord |= identity->clockAccuracy;
  write_packet(txBuffer, &wordOffset, packetWord);
  packetWord =  (identity->offsetScaledLogVariance[0] << 24);
  packetWord |= (identity->offsetScaledLogVariance[1] << 16);
  packetWord |= (identity->priority2 << 8);
  packetWord |= identity->clockIdentity[0];
  write_packet(txBuffer, &wordOffset, packetWord);
  packetWord = ((identity->clockIdentity[1] << 24) |
                (identity->clockIdentity[2] << 16) |
                (identity->clockIdentity[3] << 8)  |
                 identity->clockIdentity[4]);
  write_packet(txBuffer, &wordOffset, packetWord);
  packetWord = ((identity->clockIdentity[5] << 24) |
                (identity->clockIdentity[6] << 16) |
                (identity->clockIdentity[7] << 8) |
                (ptp->masterStepsRemoved >> 8));
  write_packet(txBuffer, &wordOffset, packetWord);
  packetWord = (ptp->masterStepsRemoved << 24) |
               (ptp->properties.timeSource << 16) |
               PATH_TRACE_TLV_TYPE;
  write_packet(txBuffer, &wordOffset, packetWord);

  /* Add the path trace TLV info. */
  packetWord = (PATH_TRACE_TLV_LENGTH(ptp->pathTraceLength) << 16);
  for (i=0; i<ptp->pathTraceLength; i++) {
    packetWord |= (ptp->pathTrace[i][0] << 8) |
                   ptp->pathTrace[i][1];
    write_packet(txBuffer, &wordOffset, packetWord);
    packetWord = (ptp->pathTrace[i][2] << 24) |
                 (ptp->pathTrace[i][3] << 16) |
                 (ptp->pathTrace[i][4] << 8) |
                  ptp->pathTrace[i][5];
    write_packet(txBuffer, &wordOffset, packetWord);
    packetWord = (ptp->pathTrace[i][6] << 24) |
                 (ptp->pathTrace[i][7] << 16);
  }
  write_packet(txBuffer, &wordOffset, packetWord);
}

/* Initializes the SYNC message transmit template */
static void init_sync_template(struct ptp_device *ptp, uint32_t port) {
  uint8_t * txBuffer;
  uint32_t wordOffset;

  /* Initialize the header, and clear the originTimestamp for good measure. */
  txBuffer = get_output_buffer(ptp,port,PTP_TX_SYNC_BUFFER);
  init_ptp_header(ptp, port, txBuffer, &wordOffset, MSG_SYNC, PTP_SYNC_LENGTH,
                  (uint16_t) FLAG_NONE);
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
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
  write_packet(txBuffer, &wordOffset, 0x00000000);
}

/* Initializes the DELAY_REQ message transmit template */
static void init_delay_request_template(struct ptp_device *ptp, uint32_t port) {
  uint8_t * txBuffer;
  uint32_t wordOffset;

  /* Initialize the header, and clear the originTimestamp for good measure */
  txBuffer = get_output_buffer(ptp,port,PTP_TX_DELAY_REQ_BUFFER);
  init_ptp_header(ptp, port, txBuffer, &wordOffset, MSG_DELAY_REQ,
                  PTP_DELAY_REQ_LENGTH, (uint16_t) FLAG_NONE);

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
                  PTP_DELAY_RESP_LENGTH, (uint16_t) FLAG_NONE);

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
                  PTP_PDELAY_REQ_LENGTH, (uint16_t) FLAG_NONE);

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
                  PTP_PDELAY_RESP_LENGTH, (uint16_t) FLAG_NONE);

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
                  (uint16_t) FLAG_NONE);

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
  init_announce_template(ptp, port, &ptp->systemPriority);
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

/* Sets the logMessageInterval within the passed packet buffer */
static void set_log_message_interval(struct ptp_device *ptp, uint32_t port, uint8_t * txBuffer,
                            uint8_t logMsgInterval) {
  uint8_t * bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  /* Read, modify, and write back the log message interval */
  bufferBase = txBuffer + TX_DATA_OFFSET(ptp);

  /* Locate the sequence ID in the packet */
  wordOffset = LOG_MSG_INTERVAL_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  packetWord &= 0xFFFFFF00;
  packetWord |= logMsgInterval;
  wordOffset = LOG_MSG_INTERVAL_OFFSET;
  write_packet(bufferBase, &wordOffset, packetWord);
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


/* Get the gmTimeBaseIndicator from the follow-up TLV */
uint16_t get_gm_time_base_indicator_field(uint8_t *rxBuffer) {
  uint32_t wordOffset = GM_TIME_BASE_INDICATOR_OFFSET;
  return read_packet(rxBuffer, &wordOffset) >> 16;
}

/* Sets the message GM time base indicator within the passed packet buffer */
static void set_gm_time_base_indicator(struct ptp_device *ptp, uint8_t * txBuffer) {
  uint8_t * bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  bufferBase = txBuffer + TX_DATA_OFFSET(ptp);

  /* Locate the time base in the packet */
  wordOffset = GM_TIME_BASE_INDICATOR_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  packetWord &= 0x0000FFFF;
  packetWord |= ((uint32_t) ptp->lastGmTimeBaseIndicator << 16);
  wordOffset -= BYTES_PER_WORD;
  write_packet(bufferBase, &wordOffset, packetWord);
}

/* Get the lastGmPhaseChange from the follow-up TLV */
void get_gm_phase_change_field(uint8_t *rxBuffer, Integer96 *lastGmPhaseChange) {
  uint32_t packetWord;
  uint32_t wordOffset = GM_PHASE_CHANGE_OFFSET;

  lastGmPhaseChange->upper = ((read_packet(rxBuffer, &wordOffset) & 0x0000FFFF) << 16);
  packetWord = read_packet(rxBuffer, &wordOffset);
  lastGmPhaseChange->upper |= ((packetWord & 0xFFFF0000) >> 16);

  lastGmPhaseChange->middle = ((packetWord & 0x0000FFFF) << 16);
  packetWord = read_packet(rxBuffer, &wordOffset);
  lastGmPhaseChange->middle |= ((packetWord & 0xFFFF0000) >> 16);

  lastGmPhaseChange->lower = ((packetWord & 0x0000FFFF) << 16);
  lastGmPhaseChange->lower |= ((read_packet(rxBuffer, &wordOffset) & 0xFFFF0000) >> 16);
}

/* Sets the message GM phase change within the passed packet buffer */
static void set_gm_phase_change(struct ptp_device *ptp, uint8_t * txBuffer) {
  uint8_t * bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  bufferBase = txBuffer + TX_DATA_OFFSET(ptp);

  /* Locate the phase change in the packet */
  wordOffset = GM_PHASE_CHANGE_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  packetWord &= 0xFFFF0000;
  packetWord |= (ptp->lastGmPhaseChange.upper >> 16);
  wordOffset -= BYTES_PER_WORD;
  write_packet(bufferBase, &wordOffset, packetWord);

  packetWord = (((ptp->lastGmPhaseChange.upper) << 16) |
                (ptp->lastGmPhaseChange.middle >> 16));
  write_packet(bufferBase, &wordOffset, packetWord);
  packetWord = (((ptp->lastGmPhaseChange.middle) << 16) |
                (ptp->lastGmPhaseChange.lower >> 16));
  write_packet(bufferBase, &wordOffset, packetWord);

  packetWord = read_packet(bufferBase, &wordOffset);
  packetWord &= 0x0000FFFF;
  packetWord |= (ptp->lastGmPhaseChange.lower << 16);
  wordOffset -= BYTES_PER_WORD;
  write_packet(bufferBase, &wordOffset, packetWord);
}

/* Get the scaledLastGmFreqChange from the follow-up TLV */
uint16_t get_gm_freq_change_field(uint8_t *rxBuffer) {
  uint32_t packetWord;
  uint32_t wordOffset = GM_FREQ_CHANGE_OFFSET;

  packetWord = ((read_packet(rxBuffer, &wordOffset) & 0x0000FFFF) << 16);
  packetWord |= ((read_packet(rxBuffer, &wordOffset) & 0xFFFF0000) >> 16);

  return packetWord;
}

/* Sets the message GM frequency change within the passed packet buffer */
static void set_gm_freq_change(struct ptp_device *ptp, uint8_t * txBuffer) {
  uint8_t * bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  bufferBase = txBuffer + TX_DATA_OFFSET(ptp);

  /* Locate the time base in the packet */
  wordOffset = GM_FREQ_CHANGE_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  packetWord &= 0xFFFF0000;
  packetWord |= (ptp->lastGmFreqChange >> 16);
  wordOffset -= BYTES_PER_WORD;
  write_packet(bufferBase, &wordOffset, packetWord);

  packetWord = read_packet(bufferBase, &wordOffset);
  packetWord &= 0x0000FFFF;
  packetWord |= (ptp->lastGmFreqChange << 16);
  wordOffset -= BYTES_PER_WORD;
  write_packet(bufferBase, &wordOffset, packetWord);
}


/* Get the cumulative scaled rate offset from the follow-up TLV */
uint32_t get_cumulative_scaled_rate_offset_field(uint8_t *rxBuffer) {
  uint32_t wordOffset = CUMULATIVE_SCALED_RATE_OFFSET_OFFSET;
  return read_packet(rxBuffer, &wordOffset);
}

/* Set the cumulative scaled rate offset in the follow-up TLV */
void set_cumulative_scaled_rate_offset_field(struct ptp_device *ptp, uint8_t *txBuffer, int32_t scaledRateOffset) {
  uint32_t wordOffset = CUMULATIVE_SCALED_RATE_OFFSET_OFFSET + TX_DATA_OFFSET(ptp);
  return write_packet(txBuffer, &wordOffset, scaledRateOffset);
}

uint16_t get_port_number(const uint8_t *portNumber) {
  /* Fetch the big-endian packed value */
  return((uint16_t) ((portNumber[0] << 8) | portNumber[1]));
}

void set_port_number(uint8_t *portNumber, uint16_t setValue) {
  portNumber[0] = (uint8_t) (setValue >> 8);
  portNumber[1] = (uint8_t) setValue;
}

uint16_t get_steps_removed(const uint8_t *stepsRemoved) {
  /* Fetch the big-endian packed value */
  return((uint16_t) ((stepsRemoved[0] << 8) | stepsRemoved[1]));
}

void set_steps_removed(uint8_t *stepsRemoved, uint16_t setValue) {
  stepsRemoved[0] = (uint8_t) (setValue >> 8);
  stepsRemoved[1] = (uint8_t) setValue;
}

uint16_t get_offset_scaled_log_variance(const uint8_t *offsetScaledLogVariance) {
  /* Fetch the big-endian packed value */
  return((uint16_t) ((offsetScaledLogVariance[0] << 8) | offsetScaledLogVariance[1]));
}

void set_offset_scaled_log_variance(uint8_t *offsetScaledLogVariance, uint16_t setValue) {
  offsetScaledLogVariance[0] = (uint8_t) (setValue >> 8);
  offsetScaledLogVariance[1] = (uint8_t) setValue;
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

  /* Update with the current GM info and path vector */
  init_announce_template(ptp, port, ptp->gmPriority);

  /* Update the sequence ID */
  txBuffer = get_output_buffer(ptp, port, PTP_TX_ANNOUNCE_BUFFER);
  set_sequence_id(ptp, port, txBuffer, ptp->ports[port].announceSequenceId++);

  /* Update the origin timestamp with the present state of the RTC */
  get_rtc_time(ptp, &presentTime);
  set_timestamp(ptp, port, txBuffer, &presentTime);
  set_log_message_interval(ptp, port, txBuffer, ptp->ports[port].currentLogAnnounceInterval);

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
  set_log_message_interval(ptp, port, txBuffer, ptp->ports[port].currentLogSyncInterval);

  /* Update the correction field. This is always zero. */
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
  int64_t correctionField;
  int32_t rateRatio;

  txFupBuffer = get_output_buffer(ptp,port,PTP_TX_FUP_BUFFER);
  txSyncBuffer= get_output_buffer(ptp,port,PTP_TX_SYNC_BUFFER);
  /* Copy the sequence ID from the last SYNC message we sent */
  set_sequence_id(ptp, port, txFupBuffer,
                  get_sequence_id(ptp, port, TRANSMITTED_PACKET, txSyncBuffer));
  set_log_message_interval(ptp, port, txFupBuffer, ptp->ports[port].currentLogSyncInterval);

  if (ptp->ports[port].syncTxLocalTimestampValid) {
    PtpTime residency;
    PtpTime correction;
    timestamp_difference(&ptp->ports[port].syncTxTimestamp, &ptp->ports[port].syncRxTimestamp, &residency);
    timestamp_sum(&residency, &ptp->ports[port].lastFollowUpCorrectionField, &correction);
    correctionField = correction.nanoseconds;

    // Convert to 2^-41 - (1.0) from something in the 2^-31 range and remove the 1.0 back out
    rateRatio = (int32_t)((ptp->masterRateRatio - 0x80000000) << 10);

    set_timestamp(ptp, port, txFupBuffer, &ptp->ports[port].lastPreciseOriginTimestamp);

  } else {
    /* Update the precise origin timestamp with the hardware timestamp from when
     * the preceding SYNC was accepted into the MAC, augmented by the MAC's TX
     * latency. In the case where we are not the GM, the syncTxTimestamp member
     * will contain the value from the received follow-up, NOT the time we sent
     * the sync.
     */
    set_timestamp(ptp, port, txFupBuffer, &ptp->ports[port].syncTxTimestamp);

    correctionField = 0;
    rateRatio = 0;
  }

  /* Update the correction field.
   * This should always be zero except if we are acting as a transparent clock
   */
  correctionField <<= CORRECTION_FRACTION_BITS;
  update_correction_field(ptp, port, txFupBuffer, correctionField);

  /* Set the scaled rate offset */
  set_cumulative_scaled_rate_offset_field(ptp, txFupBuffer, rateRatio);
  set_gm_time_base_indicator(ptp, txFupBuffer);
  set_gm_phase_change(ptp, txFupBuffer);
  set_gm_freq_change(ptp, txFupBuffer);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, port, txFupBuffer);
  ptp->ports[port].stats.txFollowupCount++;

  /* No valid timestamps for this set now */
  ptp->ports[port].syncTxLocalTimestampValid = FALSE;
  ptp->ports[port].fupPreciseOriginTimestampReceived = FALSE;
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
  set_log_message_interval(ptp, port, txBuffer, ptp->ports[port].currentLogPdelayReqInterval);

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
  /* Update the sequence ID (incremented in the pdelay state machine) */
  set_sequence_id(ptp, port, txBuffer, ptp->ports[port].pdelayReqSequenceId);

  /* All dynamic fields have been updated, transmit the packet */
  transmit_packet(ptp, port, txBuffer);
  ptp->ports[port].stats.txPDelayRequestCount++;
}

/* Transmits a PDELAY_RESP message in response to the PDELAY_REQ message received
 * into the passed Rx packet buffer
 */
void transmit_pdelay_response(struct ptp_device *ptp, uint32_t port, uint8_t * requestRxBuffer) {
#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
  PtpTime diff;
  PtpTime timeTest;
  switch_timestamp_t *switch_t1;
  switch_timestamp_t *switch_t2;
  switch_timestamp_t switch_t1a,switch_t2a;
  switch_timestamp_t switch_t1b,switch_t2b;
  int32_t t1,t2;
  uint16_t sequence_id;
  int cnt1,cnt2;
#endif

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
   * request's Rx buffer.  Use the monotonic local counter, per 802.1AS section
   * 10.1.1: "All timestamps are taken relative to the LocalClock entity".
   */
  get_local_hardware_timestamp(ptp, port, RECEIVED_PACKET, requestRxBuffer, &pdelayReqRxTimestamp);

#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
      {
        sequence_id=pdelayReqSequenceId;
        cnt1=0;
        cnt2=0;
        if(port==0) {
          switch_timestamp(TIMESTAMP_AVB2_INCOMMING_REQUEST,&switch_t1, &switch_t2,sequence_id);
          if(switch_t1==NULL) {
              do {
                block_read_avb_ptp(&switch_t1a,0,0x08);
                block_read_avb_ptp(&switch_t1b,0,0x08);
                cnt1++;
              } while(
                    (switch_t1a.sequence_id!=switch_t1b.sequence_id)||
                    (switch_t1a.low!=switch_t1b.low)||
                    (switch_t1a.high!=switch_t1b.high));
          } else {
              memcpy(&switch_t1a,switch_t1,sizeof(switch_timestamp_t));
          }
          if(switch_t2==NULL) {
              do {
                block_read_avb_ptp(&switch_t2a,5,0x10);
                block_read_avb_ptp(&switch_t2b,5,0x10);
                cnt2++;
              } while(
                    (switch_t2a.sequence_id!=switch_t2b.sequence_id)||
                    (switch_t2a.low!=switch_t2b.low)||
                    (switch_t2a.high!=switch_t2b.high));
          } else {
              memcpy(&switch_t2a,switch_t2,sizeof(switch_timestamp_t));
          }
        } else {
          switch_timestamp(TIMESTAMP_AVB1_INCOMMING_REQUEST,&switch_t1, &switch_t2,sequence_id);
          if(switch_t1==NULL) {
              do {
                block_read_avb_ptp(&switch_t1a,1,0x08);
                block_read_avb_ptp(&switch_t1b,1,0x08);
                cnt1++;
              } while(
                    (switch_t1a.sequence_id!=switch_t1b.sequence_id)||
                    (switch_t1a.low!=switch_t1b.low)||
                    (switch_t1a.high!=switch_t1b.high));
          } else {
              memcpy(&switch_t1a,switch_t1,sizeof(switch_timestamp_t));
          }
          if(switch_t2==NULL) {
              do {
                block_read_avb_ptp(&switch_t2a,6,0x10);
                block_read_avb_ptp(&switch_t2b,6,0x10);
                cnt2++;
              } while(
                    (switch_t2a.sequence_id!=switch_t2b.sequence_id)||
                    (switch_t2a.low!=switch_t2b.low)||
                    (switch_t2a.high!=switch_t2b.high));
          } else {
              memcpy(&switch_t2a,switch_t2,sizeof(switch_timestamp_t));
          }
        }
        if((cnt1!=0)||(cnt2!=0)) {
            DEBUG_TIMESTAMP_PRINTF("C retry%d,%d\r\n",cnt1,cnt2);
        }
        if((sequence_id==switch_t1a.sequence_id) &&
           (sequence_id==switch_t2a.sequence_id)) {
          t1=(switch_t1a.high<<16)|switch_t1a.low;
          t2=(switch_t2a.high<<16)|switch_t2a.low;
          diff.secondsUpper=0;
          diff.secondsLower=((uint32_t)t2*8ULL)/1000000000ULL;
          diff.nanoseconds=((uint32_t)t2*8ULL)%1000000000ULL;
          timestamp_difference(&pdelayReqRxTimestamp,&diff,&ptp->switchDelta);
          ptp->t2_prev=t2;

          diff.secondsUpper=0;
          diff.secondsLower=0;
          diff.nanoseconds=(t2-t1)*8;
          if((uint32_t)t2<(uint32_t)t1) {
            WARN_TIMESTAMP_PRINTF("C t2 wrapped %04x\r\n",diff.nanoseconds);
          }
          timestamp_difference(&pdelayReqRxTimestamp,&diff,&timeTest);
          if((uint32_t)t2<(uint32_t)t1) {
            WARN_TIMESTAMP_PRINTF("C t1 wrapped around\r\n");
            diff.secondsUpper=0;
            diff.secondsLower=34;
            diff.nanoseconds=359738368;
            timestamp_difference(&ptp->switchDelta,&diff,&ptp->switchDelta);
            ptp->t2_prev=t1;
          }
          diff.secondsUpper=0;
          diff.secondsLower=((uint32_t)t1*8ULL)/1000000000ULL;
          diff.nanoseconds=((uint32_t)t1*8ULL)%1000000000ULL;
          timestamp_sum(&ptp->switchDelta,&diff,&pdelayReqRxTimestamp);

          timestamp_difference(&pdelayReqRxTimestamp,&timeTest,&diff);
          if((diff.secondsUpper!=0x00000000) ||
             (diff.secondsLower!=0x00000000) ||
             ((diff.nanoseconds&0xffffff00)!=0x00000000)) {
                if((diff.secondsUpper!=0xffffffff) ||
                   (diff.secondsLower!=0xffffffff) ||
                   ((diff.nanoseconds&0xffffff00)!=0xffffff00)) {
                        ERROR_TIMESTAMP_PRINTF("C    !!!     bad math %08x%08x.%08x\r\n",diff.secondsUpper,diff.secondsLower,diff.nanoseconds);
                }
          }
          if(ptp->ports[port].recoveringC==1) {
                DEBUG_TIMESTAMP_PRINTF("C recovered rx request, ts:%08x%08x.%08x\r\n",pdelayReqRxTimestamp.secondsUpper,pdelayReqRxTimestamp.secondsLower,pdelayReqRxTimestamp.nanoseconds);
                ptp->ports[port].recoveringC=0;
          }
#endif
          set_timestamp(ptp, port, txBuffer, &pdelayReqRxTimestamp);
          /* All dynamic fields have been updated, transmit the packet */
          transmit_packet(ptp, port, txBuffer);
          ptp->ports[port].stats.txPDelayResponseCount++;
#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
          if(ptp->ports[port].skippedResponseCount>=2) {
            ERROR_TIMESTAMP_PRINTF("C skipped %d tx responses in a row\r\n",ptp->ports[port].skippedResponseCount);
          }
          ptp->ports[port].skippedResponseCount=0;
        } else if(sequence_id==switch_t1a.sequence_id) {
          t1=(switch_t1a.high<<16)|switch_t1a.low;
          if(((uint32_t)t1<0x70000000) &&
             ((uint32_t)ptp->t2_prev>0x90000000)) {
            WARN_TIMESTAMP_PRINTF("C t1 wrapped around\r\n");
            diff.secondsUpper=0;
            diff.secondsLower=34;
            diff.nanoseconds=359738368;
            timestamp_sum(&ptp->switchDelta,&diff,&ptp->switchDelta);
          }
          if(((uint32_t)ptp->t2_prev<0x70000000) &&
             ((uint32_t)t1>0x90000000)) {
            WARN_TIMESTAMP_PRINTF("C t2 wrapped around\r\n");
            diff.secondsUpper=0;
            diff.secondsLower=34;
            diff.nanoseconds=359738368;
            timestamp_difference(&ptp->switchDelta,&diff,&ptp->switchDelta);
          }
          diff.secondsUpper=0;
          diff.secondsLower=((uint32_t)t1*8ULL)/1000000000ULL;
          diff.nanoseconds=((uint32_t)t1*8ULL)%1000000000ULL;
          timestamp_sum(&ptp->switchDelta,&diff,&pdelayReqRxTimestamp);
          DEBUG_TIMESTAMP_PRINTF("C recovered rx request %d t1:%08x, t2:%08x, delta:%08x%08x.%08x\r\n",port,(uint32_t)t1,(uint32_t)ptp->t2_prev,ptp->switchDelta.secondsUpper,ptp->switchDelta.secondsLower,ptp->switchDelta.nanoseconds);
          ptp->ports[port].recoveringC=1;

          set_timestamp(ptp, port, txBuffer, &pdelayReqRxTimestamp);
          /* All dynamic fields have been updated, transmit the packet */
          transmit_packet(ptp, port, txBuffer);
          ptp->ports[port].stats.txPDelayResponseCount++;

          if(ptp->ports[port].skippedResponseCount>=2) {
            ERROR_TIMESTAMP_PRINTF("C skipped %d tx responses in a row\r\n",ptp->ports[port].skippedResponseCount);
          }
          ptp->ports[port].skippedResponseCount=0;
        } else {
          ptp->ports[port].recoveringC=1;
          ptp->ports[port].skippedResponseCount++;
          DEBUG_TIMESTAMP_PRINTF("C missed rx request %04x:%04x instead of %04x\r\n",switch_t1a.sequence_id,switch_t2a.sequence_id,sequence_id);
          ptp->ports[port].pdelayIntervalTimer += PDELAY_REQ_INTERVAL_TICKS(ptp, port)/4;
        }
      }
#endif
}

/* Transmits a PDELAY_RESP_FUP message related to the last PDELAY_RESP message that was
 * sent
 */
void transmit_pdelay_response_fup(struct ptp_device *ptp, uint32_t port) {
#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
  PtpTime diff;
  switch_timestamp_t *switch_t1;
  switch_timestamp_t *switch_t2;
  switch_timestamp_t switch_t1a,switch_t2a;
  switch_timestamp_t switch_t1b,switch_t2b;
  int32_t t1,t2;
  uint16_t sequence_id;
  int cnt1,cnt2;
#endif

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
   * latency.  Use the monotonic local counter, per 802.1AS section
   * 10.1.1: "All timestamps are taken relative to the LocalClock entity".
   */
  get_local_hardware_timestamp(ptp, port, TRANSMITTED_PACKET, txRespBuffer,
                               &pdelayRespTxTimestamp);

#ifdef CONFIG_LABX_PTP_MARVELL_TIMESTAMPS
  sequence_id=get_sequence_id(ptp, port, TRANSMITTED_PACKET, txRespBuffer);
  cnt1=0;
  cnt2=0;
  if(port==0) {
    switch_timestamp(TIMESTAMP_AVB2_OUTGOING_RESPONSE,&switch_t1, &switch_t2,sequence_id);
    if(switch_t1==NULL) {
        do {
          block_read_avb_ptp(&switch_t1a,5,0x08);
          block_read_avb_ptp(&switch_t1b,5,0x08);
          cnt1++;
        } while(
              (switch_t1a.sequence_id!=switch_t1b.sequence_id)||
              (switch_t1a.low!=switch_t1b.low)||
              (switch_t1a.high!=switch_t1b.high));
    } else {
        memcpy(&switch_t1a,switch_t1,sizeof(switch_timestamp_t));
    }
    if(switch_t2==NULL) {
        do {
          block_read_avb_ptp(&switch_t2a,0,0x10);
          block_read_avb_ptp(&switch_t2b,0,0x10);
          cnt2++;
        } while(
              (switch_t2a.sequence_id!=switch_t2b.sequence_id)||
              (switch_t2a.low!=switch_t2b.low)||
              (switch_t2a.high!=switch_t2b.high));
    } else {
        memcpy(&switch_t2a,switch_t2,sizeof(switch_timestamp_t));
    }
  } else {
    switch_timestamp(TIMESTAMP_AVB1_OUTGOING_RESPONSE,&switch_t1, &switch_t2,sequence_id);
    if(switch_t1==NULL) {
        do {
          block_read_avb_ptp(&switch_t1a,6,0x08);
          block_read_avb_ptp(&switch_t1b,6,0x08);
          cnt1++;
        } while(
              (switch_t1a.sequence_id!=switch_t1b.sequence_id)||
              (switch_t1a.low!=switch_t1b.low)||
              (switch_t1a.high!=switch_t1b.high));
    } else {
        memcpy(&switch_t1a,switch_t1,sizeof(switch_timestamp_t));
    }
    if(switch_t2==NULL) {
        do {
          block_read_avb_ptp(&switch_t2a,1,0x10);
          block_read_avb_ptp(&switch_t2b,1,0x10);
          cnt2++;
        } while(
              (switch_t2a.sequence_id!=switch_t2b.sequence_id)||
              (switch_t2a.low!=switch_t2b.low)||
              (switch_t2a.high!=switch_t2b.high));
    } else {
        memcpy(&switch_t2a,switch_t2,sizeof(switch_timestamp_t));
    }
  }
  if((cnt1!=0)||(cnt2!=0)) {
      DEBUG_TIMESTAMP_PRINTF("H retry%d,%d\r\n",cnt1,cnt2);
  }
  if((sequence_id==switch_t1a.sequence_id) &&
     (sequence_id==switch_t2a.sequence_id)) {
    t1=(switch_t1a.high<<16)|switch_t1a.low;
    t2=(switch_t2a.high<<16)|switch_t2a.low;
    diff.secondsUpper=0;
    diff.secondsLower=0;
    diff.nanoseconds=(t2-t1)*8;
    if((uint32_t)t2<(uint32_t)t1) {
      WARN_TIMESTAMP_PRINTF("H t2 wrapped %04x\r\n",diff.nanoseconds);
    }
    if(ptp->ports[port].skippedFollowupCount>=2) {
      ERROR_TIMESTAMP_PRINTF("H skipped %d response followups in a row\r\n",ptp->ports[port].skippedFollowupCount);
    }
    ptp->ports[port].skippedFollowupCount=0;
  } else {
    ptp->ports[port].skippedFollowupCount++;
    DEBUG_TIMESTAMP_PRINTF("H missed tx response %04x:%04x instead of %04x\r\n",switch_t1a.sequence_id,switch_t2a.sequence_id,sequence_id);
    return;
  }
  timestamp_sum(&pdelayRespTxTimestamp,&diff,&pdelayRespTxTimestamp);
#endif

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

/* Returns the type of PTP message contained within the passed receive buffer, or
 * PACKET_NOT_PTP if the buffer does not contain a valid PTP datagram.
 */
uint32_t get_transport_specific(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer) {
  uint32_t wordOffset;
  uint32_t packetWord;
  uint32_t transportSpecific = TRANSPORT_NOT_PTP;

  /* Fetch the word containing the LTF and the message type */
  wordOffset = MESSAGE_TYPE_OFFSET;
  packetWord = read_packet(rxBuffer, &wordOffset);

  transportSpecific = ((packetWord >> 8) & MSG_TRANSPORT_MASK);

  return(transportSpecific);
}


uint16_t get_rx_announce_steps_removed(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer) {
  uint16_t stepsRemoved = 0;
  uint32_t wordOffset = STEPS_REMOVED_OFFSET;
  uint32_t packetWord;
  packetWord = read_packet(rxBuffer, &wordOffset);
  stepsRemoved = ((packetWord & 0xFF) << 8);
  packetWord = read_packet(rxBuffer, &wordOffset);
  stepsRemoved |= ((packetWord >> 24) & 0xFF);
  return(stepsRemoved);
}

uint16_t get_rx_announce_path_trace(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer, PtpClockIdentity *pathTrace) {
  uint16_t pathTraceLength = 0;
  uint32_t wordOffset = PATH_TRACE_OFFSET;
  uint32_t packetWord;
  uint32_t i;

  /* Get the path trace TLV info. */
  packetWord = read_packet(rxBuffer, &wordOffset);
  pathTraceLength = (packetWord >> 16) / 8;
  for (i=0; i<pathTraceLength && i<PTP_MAX_PATH_TRACE; i++) {
    pathTrace[i][0] = (packetWord >> 8) & 0xFF;
    pathTrace[i][1] = packetWord & 0xFF;
    packetWord = read_packet(rxBuffer, &wordOffset);
    pathTrace[i][2] = (packetWord >> 24) & 0xFF;
    pathTrace[i][3] = (packetWord >> 16) & 0xFF;
    pathTrace[i][4] = (packetWord >> 8) & 0xFF;
    pathTrace[i][5] =  packetWord & 0xFF;
    packetWord = read_packet(rxBuffer, &wordOffset);
    pathTrace[i][6] = (packetWord >> 24) & 0xFF;
    pathTrace[i][7] = (packetWord >> 16) & 0xFF;
  }

  return pathTraceLength;
}

/* Extracts the contents of an ANNOUNCE message into the passed properties structure.
 * The message should already be ascertained to be an ANNOUNCE, this method will not
 * re-check.
 */
void extract_announce(struct ptp_device *ptp, uint32_t port, uint8_t *rxBuffer,
  PtpPriorityVector *pv) {

  uint32_t wordOffset;
  uint32_t packetWord;

  /* Get the source port identity*/
  get_source_port_id(ptp, port, RECEIVED_PACKET, rxBuffer, (uint8_t*)&pv->sourcePortIdentity);

#if 0
  /* Exract the domain number */
  wordOffset = DOMAIN_NUMBER_OFFSET;
  packetWord = read_packet(rxBuffer, &wordOffset);
  properties->domainNumber = ((packetWord >> 8) & 0x0FF);

  /* Extract the current UTC offset */
  wordOffset = UTC_OFFSET_OFFSET;
  packetWord = read_packet(rxBuffer, &wordOffset);
  properties->currentUtcOffset = (packetWord & 0x0FFFF);
#endif

  /* Extract the grandmaster priorities and clock quality */
  wordOffset = GM_PRIORITY1_OFFSET;
  packetWord = read_packet(rxBuffer, &wordOffset);
  pv->rootSystemIdentity.priority1 = ((packetWord >> 16) & 0xFF);
  pv->rootSystemIdentity.clockClass = ((packetWord >> 8) & 0xFF);
  pv->rootSystemIdentity.clockAccuracy = (packetWord & 0xFF);
  packetWord = read_packet(rxBuffer, &wordOffset);
  pv->rootSystemIdentity.offsetScaledLogVariance[0] = (uint8_t) (packetWord >> 24);
  pv->rootSystemIdentity.offsetScaledLogVariance[1] = (uint8_t) (packetWord >> 16);
  pv->rootSystemIdentity.priority2 = ((packetWord >> 8) & 0xFF);
  pv->rootSystemIdentity.clockIdentity[0] = (packetWord & 0xFF);
  packetWord = read_packet(rxBuffer, &wordOffset);
  pv->rootSystemIdentity.clockIdentity[1] = ((packetWord >> 24) & 0xFF);
  pv->rootSystemIdentity.clockIdentity[2] = ((packetWord >> 16) & 0xFF);
  pv->rootSystemIdentity.clockIdentity[3] = ((packetWord >> 8) & 0xFF);
  pv->rootSystemIdentity.clockIdentity[4] = (packetWord & 0xFF);
  packetWord = read_packet(rxBuffer, &wordOffset);
  pv->rootSystemIdentity.clockIdentity[5] = ((packetWord >> 24) & 0xFF);
  pv->rootSystemIdentity.clockIdentity[6] = ((packetWord >> 16) & 0xFF);
  pv->rootSystemIdentity.clockIdentity[7] = ((packetWord >> 8) & 0xFF);
  pv->stepsRemoved[0] = (uint8_t) packetWord;
  packetWord = read_packet(rxBuffer, &wordOffset);
  pv->stepsRemoved[1] = (uint8_t) (packetWord >> 24);
  //properties->timeSource = ((packetWord >> 16) & 0xFF);

  /* Port number is 1 based and is the port it came in on */
  set_port_number(pv->portNumber, (port + 1));
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
                        uint8_t *packetBuffer, uint8_t *sourcePortId) {
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

/* Gets the source port identity from a received packet */
void set_source_port_id(struct ptp_device *ptp, uint32_t port, PacketDirection bufferDirection,
                        uint8_t *packetBuffer, uint8_t *sourcePortId) {
  uint8_t *bufferBase;
  uint32_t wordOffset;
  uint32_t packetWord;

  /* Extract the source address of the packet */
  bufferBase = (bufferDirection == TRANSMITTED_PACKET) ? packetBuffer + TX_DATA_OFFSET(ptp) : packetBuffer;

  /* Update the source port id */
  wordOffset = SOURCE_PORT_ID_OFFSET;
  packetWord = read_packet(bufferBase, &wordOffset);
  wordOffset = SOURCE_PORT_ID_OFFSET;
  packetWord &= 0xFFFF0000;
  packetWord |= (sourcePortId[0] << 8);
  packetWord |= sourcePortId[1];
  write_packet(bufferBase, &wordOffset, packetWord);
  packetWord = (sourcePortId[2] << 24);
  packetWord |= (sourcePortId[3] << 16);
  packetWord |= (sourcePortId[4] << 8);
  packetWord |= sourcePortId[5];
  write_packet(bufferBase, &wordOffset, packetWord);
  packetWord = (sourcePortId[6] << 24);
  packetWord |= (sourcePortId[7] << 16);
  packetWord |= (sourcePortId[8] << 8);
  packetWord |= sourcePortId[9];
  write_packet(bufferBase, &wordOffset, packetWord);
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
