/*
 *  linux/drivers/net/labx_ptp_main_fpga.c
 *
 *  Lab X Technologies Precision Time Protocol (PTP) driver
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *
 *  Copyright (C) 2011 Lab X Technologies LLC, All Rights Reserved.
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
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>

/* Interrupt service routine for the instance */
static irqreturn_t labx_ptp_interrupt(int irq, void *dev_id)
{
  struct ptp_device *ptp = dev_id;
  uint32_t maskedFlags;
  uint32_t txCompletedFlags;
  unsigned long flags;
  int i;

  for (i=0; i<ptp->numPorts; i++) {
    /* Read the interrupt flags and immediately clear them */
    maskedFlags = ioread32(REGISTER_ADDRESS(ptp, i, PTP_IRQ_FLAGS_REG));
    maskedFlags &= ioread32(REGISTER_ADDRESS(ptp, i, PTP_IRQ_MASK_REG));
    iowrite32(maskedFlags, REGISTER_ADDRESS(ptp, i, PTP_IRQ_FLAGS_REG));

    /* Detect the timer IRQ */
    if((maskedFlags & PTP_TIMER_IRQ) != 0) {

      preempt_disable();
      spin_lock_irqsave(&ptp->mutex, flags);
      ptp->timerTicks++;
      spin_unlock_irqrestore(&ptp->mutex, flags);
      preempt_enable();

#ifdef CONFIG_LABX_PTP_NO_TASKLET
      labx_ptp_timer_state_task((uintptr_t)ptp);
#else
      /* Kick off the timer tasklet */
      tasklet_hi_schedule(&ptp->timerTasklet);
#endif
    }

    /* Detect the Tx IRQ from any enabled buffer bits */
    txCompletedFlags = (maskedFlags & PTP_TX_IRQ_MASK);
    if(txCompletedFlags != PTP_TX_BUFFER_NONE) {
      /* Add the new pending Tx buffer IRQ flags to the mask in the device
       * structure for the tasklet to whittle away at.  Lock the mutex so we
       * avoid a race condition with the Tx tasklet.
       */
      preempt_disable();
      spin_lock_irqsave(&ptp->mutex, flags);
      ptp->ports[i].pendingTxFlags |= txCompletedFlags;
      spin_unlock_irqrestore(&ptp->mutex, flags);
      preempt_enable();

#ifdef CONFIG_LABX_PTP_NO_TASKLET
      labx_ptp_tx_state_task((uintptr_t)ptp);
#else
      /* Now kick off the Tx tasklet */
      tasklet_hi_schedule(&ptp->txTasklet);
#endif
    }

    /* Detect the Rx IRQ */
    if((maskedFlags & PTP_RX_IRQ) != 0) {
#ifdef CONFIG_LABX_PTP_NO_TASKLET
      labx_ptp_rx_state_task((uintptr_t)ptp);
#else
      /* Kick off the Rx tasklet */
      tasklet_hi_schedule(&ptp->rxTasklet);
#endif
    }
  
  }
  return(IRQ_HANDLED);
}

void ptp_enable_port(struct ptp_device *ptp,int port)
{
    iowrite32(ioread32(REGISTER_ADDRESS(ptp, port, PTP_TX_REG)) | PTP_TX_ENABLE, REGISTER_ADDRESS(ptp, port, PTP_TX_REG));
    iowrite32(ioread32(REGISTER_ADDRESS(ptp, port, PTP_RX_REG)) | PTP_RX_ENABLE, REGISTER_ADDRESS(ptp, port, PTP_RX_REG));
}

void ptp_disable_port(struct ptp_device *ptp,int port)
{
    iowrite32(ioread32(REGISTER_ADDRESS(ptp, port, PTP_TX_REG)) & ~PTP_TX_ENABLE, REGISTER_ADDRESS(ptp, port, PTP_TX_REG));
    iowrite32(ioread32(REGISTER_ADDRESS(ptp, port, PTP_RX_REG)) & ~PTP_RX_ENABLE, REGISTER_ADDRESS(ptp, port, PTP_RX_REG));
}

void ptp_disable_irqs(struct ptp_device *ptp, int port)
{
    iowrite32(PTP_NO_IRQS, REGISTER_ADDRESS(ptp, port, PTP_IRQ_MASK_REG));
}

void ptp_enable_irqs(struct ptp_device *ptp, int port)
{
    uint32_t irqMask;

    irqMask = (PTP_TIMER_IRQ | PTP_RX_IRQ | 
               PTP_TX_IRQ(PTP_TX_SYNC_BUFFER) |
               PTP_TX_IRQ(PTP_TX_DELAY_REQ_BUFFER) |
               PTP_TX_IRQ(PTP_TX_PDELAY_REQ_BUFFER) |
               PTP_TX_IRQ(PTP_TX_PDELAY_RESP_BUFFER));
    iowrite32(irqMask, REGISTER_ADDRESS(ptp, port, PTP_IRQ_FLAGS_REG));
    iowrite32(irqMask, REGISTER_ADDRESS(ptp, port, PTP_IRQ_MASK_REG));
}

uint32_t ptp_get_version(struct ptp_device *ptp)
{
    return(ioread32(REGISTER_ADDRESS(ptp, 0, PTP_REVISION_REG)));
}

/* Configure the prescaler and divider used to generate a 10 msec event timer.
 * The register values are terminal counts, so are one less than the count value.
 */
void ptp_setup_event_timer(struct ptp_device *ptp, int port, PtpPlatformData *platformData)
{
    iowrite32( (((platformData->timerPrescaler - 1) & PTP_PRESCALER_MASK) |
               (((platformData->timerDivider - 1) & PTP_DIVIDER_MASK) << PTP_DIVIDER_SHIFT)), 
                                                    REGISTER_ADDRESS(ptp, 0, PTP_TIMER_REG));
}

uint32_t ptp_setup_interrupt(struct ptp_device *ptp)
{
  uint32_t returnValue;
  returnValue = request_irq(ptp->irq, &labx_ptp_interrupt, IRQF_DISABLED, "Lab X PTP", ptp);
  if (returnValue) {
    printk(KERN_ERR "%s: : Could not allocate Lab X PTP interrupt (%d).\n",
           ptp->name, ptp->irq);
  }
  return returnValue;
}

/* Writes a word to a packet buffer at the passed offset.  The offset is
 * advanced to the next word.
 */
void write_packet(uint8_t * bufferBase, uint32_t *wordOffset, 
                         uint32_t writeWord) {
  iowrite32(writeWord, ((uint32_t)bufferBase + *wordOffset));
  *wordOffset += BYTES_PER_WORD;
}

/* Reads a word from a packet buffer at the passed offset.  The offset is
 * advanced to the next word.
 */
uint32_t read_packet(uint8_t * bufferBase, uint32_t *wordOffset) {
  uint32_t readWord = ioread32((uint32_t)bufferBase + *wordOffset);
  *wordOffset += BYTES_PER_WORD;
  return(readWord);
}

void ptp_process_rx(struct ptp_device *ptp, int port)
{
    uint32_t newRxBuffer;
    uint32_t bufferBase;

    /* Process all messages received since the last time we ran */
    newRxBuffer = (ioread32(REGISTER_ADDRESS(ptp, port, PTP_RX_REG)) & PTP_RX_BUFFER_MASK);
    while(ptp->ports[port].lastRxBuffer != newRxBuffer) {
      /* Advance the last buffer circularly around the available Rx buffers */
      ptp->ports[port].lastRxBuffer = ((ptp->ports[port].lastRxBuffer + 1) & PTP_RX_BUFFER_MASK);

      /* Fetch the word containing the LTF and the message type */
      bufferBase = PTP_RX_PACKET_BUFFER(ptp, port, ptp->ports[port].lastRxBuffer);
      process_rx_buffer(ptp,port,(uint8_t *)bufferBase);
    }
}

void ptp_platform_init(struct ptp_device *ptp, int port)
{
    ptp->ports[port].lastRxBuffer = (ioread32(REGISTER_ADDRESS(ptp, port, PTP_RX_REG)) & PTP_RX_BUFFER_MASK);
}

/* Gets the hardware timestamp located within the passed packet buffer.
 * It is up to the client code to ascertain that a valid timestamp exists; that is,
 * a packet has either been transmitted from or received into the buffer.
 */
void get_hardware_timestamp(struct ptp_device *ptp, 
                            uint32_t port, 
                            PacketDirection bufferDirection,
                            uint8_t * packetBuffer, 
                            PtpTime *timestamp) {
  uint32_t wordOffset;
  uint32_t skipIncrement;
  PtpTime tempTimestamp;

  /* Fetch the hardware timestamp from the end of the specified packet buffer and pack
   * it into the passed timestamp structure.  Don't offset the Tx packet buffer by
   * its data offset in this case, since we're not going to be reading relative to
   * the start of data, but relative to the end of the buffer instead!
   */
  /* If the port is 64 bits wide, the Tx or Rx hardware must write full 64-bit
   * words into the buffer, even for the timestamp data.  This affects where the
   * timestamp data must begin, as well as the stride between two values.
   */
  wordOffset = (ptp->portWidth == 8) ? HW_TIMESTAMP_OFFSET_X8 : HW_TIMESTAMP_OFFSET_X64;
  skipIncrement = (ptp->portWidth = 8) ? 0 : BYTES_PER_WORD;
  tempTimestamp.secondsUpper = (int32_t)(read_packet(packetBuffer, &wordOffset) & 0x0FFFF);
  wordOffset += skipIncrement;
  tempTimestamp.secondsLower = read_packet(packetBuffer, &wordOffset);
  wordOffset += skipIncrement;
  tempTimestamp.nanoseconds  = read_packet(packetBuffer, &wordOffset);

  if (bufferDirection == TRANSMITTED_PACKET) {
    /* Add the MAC latency and the PHY latency */
    timestamp_sum(&tempTimestamp, &ptp->ports[port].txPhyMacDelay, timestamp);
  } else {
    /* Subtract the MAC latency and the PHY latency */
    timestamp_difference(&tempTimestamp, &ptp->ports[port].rxPhyMacDelay, timestamp);
  }
}

/* Gets the local hardware timestamp located within the passed packet buffer.
 * This is the same as get_hardware_timestamp except it is from a local clock
 * that is running at a fixed rate unmodified by PTP.
 */
void get_local_hardware_timestamp(struct ptp_device *ptp, 
                                  uint32_t port, 
                                  PacketDirection bufferDirection,
                                  uint8_t *packetBuffer, 
                                  PtpTime *timestamp) {
  uint32_t wordOffset;
  uint32_t skipIncrement;
  PtpTime tempTimestamp;

  /* Fetch the hardware timestamp from the end of the specified packet buffer and pack
   * it into the passed timestamp structure.  Don't offset the Tx packet buffer by
   * its data offset in this case, since we're not going to be reading relative to
   * the start of data, but relative to the end of the buffer instead!
   */

  wordOffset = (ptp->portWidth == 8) ? HW_LOCAL_TIMESTAMP_OFFSET_X8 : HW_LOCAL_TIMESTAMP_OFFSET_X64;
  skipIncrement = (ptp->portWidth = 8) ? 0 : BYTES_PER_WORD;
  tempTimestamp.secondsUpper = (int32_t)(read_packet(packetBuffer, &wordOffset) & 0x0FFFF);
  wordOffset += skipIncrement;
  tempTimestamp.secondsLower = read_packet(packetBuffer, &wordOffset);
  wordOffset += skipIncrement;
  tempTimestamp.nanoseconds  = read_packet(packetBuffer, &wordOffset);

  if (bufferDirection == TRANSMITTED_PACKET) {
    /* Add the MAC latency and the PHY latency */
    timestamp_sum(&tempTimestamp, &ptp->ports[port].txPhyMacDelay, timestamp);
  } else {
    /* Subtract the MAC latency and the PHY latency */
    timestamp_difference(&tempTimestamp, &ptp->ports[port].rxPhyMacDelay, timestamp);
  }
}

/* Transmits the packet within the specified buffer.  The first word of the
 * buffer must contain the packet length minus one, in bytes.
 */
void transmit_packet(struct ptp_device *ptp, uint32_t port, uint8_t * txBuffer) {
  int i;
  for (i=0; i<PTP_TX_BUFFER_COUNT; i++) {
    if ((uintptr_t)txBuffer == PTP_TX_PACKET_BUFFER(ptp, port, i)) break;
  }

  /* Don't bother checking the busy flag; it only exists to ensure that we know the
   * pending flags are valid.  We're not using them anyways - the only hazard that
   * exists is if we attempt to send two packets from the same buffer simultaneously.
   */
  iowrite32(((1<<i) | PTP_TX_ENABLE), REGISTER_ADDRESS(ptp, port, PTP_TX_REG));
}


uint8_t * get_output_buffer(struct ptp_device *ptp,uint32_t port,uint32_t bufType)
{
  return (uint8_t*)PTP_TX_PACKET_BUFFER(ptp, port, bufType);
}

/* Disables the RTC */
void disable_rtc(struct ptp_device *ptp) {
  iowrite32(PTP_RTC_DISABLE, REGISTER_ADDRESS(ptp, 0, PTP_RTC_INC_REG));
}

/* Sets the RTC increment, simultaneously enabling the RTC */
void set_rtc_increment(struct ptp_device *ptp, RtcIncrement *increment) {
  uint32_t incrementWord;

  /* Save the current increment if anyone needs it */
  ptp->currentIncrement = *increment;

  /* Assemble a single value from the increment components */
  incrementWord = ((increment->mantissa & RTC_MANTISSA_MASK) << RTC_MANTISSA_SHIFT);
  incrementWord |= (increment->fraction & RTC_FRACTION_MASK);
  incrementWord |= PTP_RTC_ENABLE;

  /* The actual write is already atomic, so no need to ensure mutual exclusion */
  iowrite32(incrementWord, REGISTER_ADDRESS(ptp, 0, PTP_RTC_INC_REG));
}

/* Return the current increment value */
void get_rtc_increment(struct ptp_device *ptp, RtcIncrement *increment) {
  *increment = ptp->currentIncrement;
}

/* Captures the present RTC time, returning it into the passed structure */
void get_rtc_time(struct ptp_device *ptp, PtpTime *time) {
  uint32_t timeWord;
  unsigned long flags;

  /* Write to the capture flag in the upper seconds word to initiate a capture,
   * then poll the same bit to determine when it has completed.  The capture only
   * takes a few RTC clocks, so this busy wait can only consume tens of nanoseconds.
   *
   * This will *not* modify the time, since we don't write the nanoseconds register.
   */
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  iowrite32(PTP_RTC_CAPTURE_FLAG, REGISTER_ADDRESS(ptp, 0, PTP_SECONDS_HIGH_REG));
  do {
    timeWord = ioread32(REGISTER_ADDRESS(ptp, 0, PTP_SECONDS_HIGH_REG));
  } while((timeWord & PTP_RTC_CAPTURE_FLAG) != 0);

  /* Now read the entire captured time and pack it into the structure.  The last
   * value read during polling is perfectly valid.
   */
  time->secondsUpper = (uint16_t) timeWord;
  time->secondsLower = ioread32(REGISTER_ADDRESS(ptp, 0, PTP_SECONDS_LOW_REG));
  time->nanoseconds = ioread32(REGISTER_ADDRESS(ptp, 0, PTP_NANOSECONDS_REG));
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();
}

void get_local_time(struct ptp_device *ptp, PtpTime *time) {
  uint32_t timeWord;
  unsigned long flags;

  /* Write to the capture flag in the upper seconds word to initiate a capture,
   * then poll the same bit to determine when it has completed.  The capture only
   * takes a few RTC clocks, so this busy wait can only consume tens of nanoseconds.
   *
   * This will *not* modify the time, since we don't write the nanoseconds register.
   */
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  iowrite32(PTP_RTC_LOCAL_CAPTURE_FLAG, REGISTER_ADDRESS(ptp, 0, PTP_LOCAL_SECONDS_HIGH_REG));
  do {
    timeWord = ioread32(REGISTER_ADDRESS(ptp, 0, PTP_LOCAL_SECONDS_HIGH_REG));
  } while((timeWord & PTP_RTC_LOCAL_CAPTURE_FLAG) != 0);

  /* Now read the entire captured time and pack it into the structure.  The last
   * value read during polling is perfectly valid.
   */
  time->secondsUpper = (uint16_t) timeWord;
  time->secondsLower = ioread32(REGISTER_ADDRESS(ptp, 0, PTP_LOCAL_SECONDS_LOW_REG));
  time->nanoseconds = ioread32(REGISTER_ADDRESS(ptp, 0, PTP_LOCAL_NANOSECONDS_REG));
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();
}


/* Sets a new RTC time from the passed structure */
void set_rtc_time(struct ptp_device *ptp, PtpTime *time) {
  unsigned long flags;

  /* Write to the time register, beginning with the seconds.  The write to the 
   * nanoseconds register is what actually effects the change to the RTC.
   */
  preempt_disable();
  spin_lock_irqsave(&ptp->mutex, flags);
  iowrite32(time->secondsUpper, REGISTER_ADDRESS(ptp, 0, PTP_SECONDS_HIGH_REG));
  iowrite32(time->secondsLower, REGISTER_ADDRESS(ptp, 0, PTP_SECONDS_LOW_REG));
  iowrite32(time->nanoseconds, REGISTER_ADDRESS(ptp, 0, PTP_NANOSECONDS_REG));
  spin_unlock_irqrestore(&ptp->mutex, flags);
  preempt_enable();
}

