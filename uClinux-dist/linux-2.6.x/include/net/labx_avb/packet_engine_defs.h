/*
 *  linux/include/net/labx_avb/packet_engine_defs.h
 *
 *  Lab X Technologies AVB packet engine definitions
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

#ifndef _PACKET_ENGINE_DEFS_H_
#define _PACKET_ENGINE_DEFS_H_

#include <linux/types.h>
#include <linux/ioctl.h>

/*
 * Common definitions
 */

#define AUDIO_MAP_CHANNEL_BITS  (16)
#define AUDIO_MAP_CHANNEL_MASK  (0x0FFFF)

/* Converts the passed value to an word containing a pointer to an audio channel map
 *
 * @param descriptorBase - Base address of the containing descriptor
 * @param mapOffset      - Relative offset of the audio channel map to be encoded
 */
#define AUDIO_CHANNEL_MAP_POINTER(descriptorBase, mapOffset) ((uint32_t) (descriptorBase + mapOffset))

/* Indices for identifying memory resources */
#define PACKET_ENGINE_ADDRESS_RANGE_RESOURCE  (0)
#define PACKET_ENGINE_IRQ_RESOURCE            (1)
#define PACKET_ENGINE_NUM_RESOURCES           (2)

/* Size of AVBTP stream IDs */
#define AVBTP_STREAM_ID_BYTES  (8)

/* Timestamp intervals for supported audio sampling rates */
#define SYT_INTERVAL_32_KHZ     (8)
#define SYT_INTERVAL_44_1_KHZ   (8)
#define SYT_INTERVAL_48_KHZ     (8)
#define SYT_INTERVAL_88_2_KHZ   (16)
#define SYT_INTERVAL_96_KHZ     (16)
#define SYT_INTERVAL_176_4_KHZ  (32)
#define SYT_INTERVAL_192_KHZ    (32)

/* Number of bits in an AVBTP packet sequence number */
#define AVBTP_SEQUENCE_NUMBER_BITS  (8)

/* I/O control commands and structures common to all packet engines */
#define IOC_START_ENGINE           _IO('d', 0x01)
#define IOC_STOP_ENGINE            _IO('d', 0x02)

#define MAX_CONFIG_WORDS 1024
typedef struct {
  uint32_t  offset;
  uint32_t  numWords;
  uint32_t *configWords;
  uint32_t  interlockedLoad;
} ConfigWords;

#define IOC_LOAD_DESCRIPTOR        _IOW('d', 0x03, ConfigWords)
#define IOC_COPY_DESCRIPTOR        _IOWR('d', 0x04, ConfigWords)

/* Definitions for the interlockedLoad member.  An "interlocked" load makes
 * use of hardware interlocks to ensure the final word of a descriptor load
 * is written in a manner guaranteed not to disrupt the running microengine.
 */
#define LOAD_NORMAL       (0)
#define LOAD_INTERLOCKED  (1)

/*
 * Packetizer definitions
 */

/* I/O control commands and structures specific to the packetizer */
#define IOC_LOAD_PACKET_TEMPLATE     _IOW('d', 0x10, ConfigWords)
#define IOC_COPY_PACKET_TEMPLATE     _IOW('d', 0x11, ConfigWords)
#define IOC_SET_START_VECTOR         _IOW('d', 0x12, uint32_t)

typedef struct {
  uint32_t clockDomain;
  uint32_t sytInterval;
  uint32_t enabled;
} ClockDomainSettings;
#  define DOMAIN_DISABLED  (0x00)
#  define DOMAIN_ENABLED   (0x01)

#define IOC_CONFIG_CLOCK_DOMAIN      _IOW('d', 0x13, ClockDomainSettings)

typedef struct {
  uint32_t versionMajor;
  uint32_t versionMinor;
  uint32_t maxInstructions;
  uint32_t maxTemplateBytes;
  uint32_t maxClockDomains;
  uint32_t shaperFractionBits;
} PacketizerCaps;

#define IOC_GET_PACKETIZER_CAPS      _IOR('d', 0x14, PacketizerCaps)

#define IOC_SET_PRESENTATION_OFFSET  _IOW('d', 0x15, uint32_t)
#  define PRESENTATION_OFFSET_MASK  (0x001FFFFF)

typedef struct {
  int32_t  idleSlope;
  int32_t  sendSlope;
  uint32_t enabled;
} CreditShaperSettings;
#  define CREDIT_SHAPER_DISABLED (0x00)
#  define CREDIT_SHAPER_ENABLED  (0x01)

#define IOC_CONFIG_CREDIT_SHAPER     _IOW('d', 0x16, CreditShaperSettings)

/* Type definitions and macros for packetizer microcode */

/* Opcode definitions */
#define PACKETIZER_OPCODE_TEMPLATE               (0x00)
#define PACKETIZER_OPCODE_LOAD_ACCUMULATOR       (0x01)
#define PACKETIZER_OPCODE_STORE_ACCUMULATOR      (0x02)
#define PACKETIZER_OPCODE_INCREMENT_ACCUMULATOR  (0x03)
#define PACKETIZER_OPCODE_INSERT_ACCUMULATOR     (0x04)
#define PACKETIZER_OPCODE_COND_LOAD_ACCUMULATOR  (0x05)
#define PACKETIZER_OPCODE_INSERT_TIMESTAMP       (0x06)
#define PACKETIZER_OPCODE_INSERT_PAYLOAD_SIZE    (0x07)
#define PACKETIZER_OPCODE_INSERT_BLOCK_COUNT     (0x08)
#define PACKETIZER_OPCODE_AUDIO_SAMPLES          (0x09)
#define PACKETIZER_OPCODE_LOAD_RING_OFFSET       (0x0A)
#define PACKETIZER_OPCODE_STORE_RING_OFFSET      (0x0B)
#define PACKETIZER_OPCODE_LINK                   (0x0C)
#define PACKETIZER_OPCODE_JUMP                   (0x0D)
#define PACKETIZER_OPCODE_PUSH_PARAM             (0x0E)

/* Instruction field constant definitions */
#define PACKETIZER_OPCODE_SHIFT          (24)
#define PACKETIZER_USE_STACK_BIT         (0x00800000)
#define PACKETIZER_TEMPLATE_COUNT_MASK   (0x01F)
#define PACKETIZER_TEMPLATE_COUNT_SHIFT  (18)

#define PACKETIZER_ACCUMULATOR_ZERO   (0x00000000)
#define PACKETIZER_ACCUMULATOR_ONE    (0x00400000)
#define PACKETIZER_ACCUMULATOR_TWO    (0x00800000)
#define PACKETIZER_ACCUMULATOR_THREE  (0x00C00000)

#define PACKETIZER_ACCUMULATOR_MASK  (0x000000FF)
#define PACKETIZER_ACCUMULATOR_BITS  (8)

#define PACKETIZER_STATUS_NONE       (0x00000000)
#define PACKETIZER_STATUS_TIMESTAMP  (0x00200000)

#define PACKETIZER_PARAM_STACK_MASK  (0x00FFFFFF)
#define PACKETIZER_PARAM_STACK_BITS  (24)

/* NOTE - These constants are related to the PACKETIZER_MAX_SAMPLE_BYTES and 
 *        PACKETIZER_MAX_STREAM_SLOTS constants within Audio_Packetizer_Params.vhd.  
 *        Should the VHDL constants ever be changed, these must be revisited.
 */
#define PACKETIZER_MAX_STREAM_SLOTS      (32)
#define PACKETIZER_SAMPLE_SIZE_BITS      (3)
#define PACKETIZER_SAMPLE_SIZE_MASK      (0x07)
#define PACKETIZER_PACKET_SLOT_BITS      (5)
#define PACKETIZER_DATA_BLOCK_SIZE_BITS  (PACKETIZER_SAMPLE_SIZE_BITS + PACKETIZER_PACKET_SLOT_BITS)
#define PACKETIZER_DATA_BLOCK_SIZE_MASK  (0x0FF)
#define PACKETIZER_CHANNEL_COUNT_SHIFT   (PACKETIZER_PARAM_STACK_BITS - PACKETIZER_PACKET_SLOT_BITS)

/* Returns an instruction word containing a clock domain specifier
 * @param clockDomain - Zero-based clock domain the stream descriptor belongs to
 */
#define PACKETIZER_DOMAIN_FIELD(clockDomain)  ((uint32_t) (0x01 << clockDomain))

/* Returns an instruction word containing a link address and validity
 * @param linkValid   - True if the link address points to a valid descriptor, 
 *                      false if this is the last
 * @param linkAddress - Address of the next stream descriptor, if linkValid is
 *                      true.  Ignored if linkValid is false.
 */
#define PACKETIZER_LINK_ADDRESS(linkValid, linkAddress) \
  ((uint32_t) (linkValid | linkAddress))
#  define PACKETIZER_LINK_INVALID  (0x00000000)
#  define PACKETIZER_LINK_VALID    (0x80000000)

/* Symbolic definitions for the PACKETIZER_TEMPLATE macro */
#define USE_CODED_ADDRESS       (0x00)
#define USE_STACK_ADDRESS       (0x01)
#define DUMMY_TEMPLATE_ADDRESS  (0)

/* Returns a TEMPLATE instruction
 * @param useStackAddress - If true, the address parameter is popped from the 
 *                          parameter stack.  If false, the address encoded in
 *                          the instruction is used instead.
 * @param templateAddress - Address of the first byte in the template RAM to emit
 * @param byteCount       - Number of template bytes to emit
 */
#define PACKETIZER_TEMPLATE(useStackAddress, templateAddress, byteCount) \
  ((uint32_t) ((PACKETIZER_OPCODE_TEMPLATE << PACKETIZER_OPCODE_SHIFT) | \
               ((useStackAddress != 0) ? PACKETIZER_USE_STACK_BIT : 0) | \
               (((byteCount - 1) & PACKETIZER_TEMPLATE_COUNT_MASK) <<    \
                PACKETIZER_TEMPLATE_COUNT_SHIFT)                       | \
               templateAddress))

/* Returns a LOAD_ACCUMULATOR instruction.  The accumulator is loaded a cycle
 * after the next instruction begins; so the instruction immediately following
 * must not expect the value to be present already.  The low byte of memory
 * is loaded.
 * @param whichAccumulator - Accumulator to load: PACKETIZER_ACCUMULATOR_ZERO, etc.
 * @param loadAddress      - Address of the value to load into the accumulator
 */
#define PACKETIZER_LOAD_ACCUMULATOR(whichAccumulator, loadAddress)   \
  ((uint32_t) ((PACKETIZER_OPCODE_LOAD_ACCUMULATOR << PACKETIZER_OPCODE_SHIFT) | \
               whichAccumulator | loadAddress))

/* Returns a STORE_ACCUMULATOR instruction
 * @param whichAccumulator - Accumulator to store: PACKETIZER_ACCUMULATOR_ZERO, etc.
 * @param storeAddress     - Address at which to store the accumulator value
 */
#define PACKETIZER_STORE_ACCUMULATOR(whichAccumulator, storeAddress) \
  ((uint32_t) ((PACKETIZER_OPCODE_STORE_ACCUMULATOR << PACKETIZER_OPCODE_SHIFT) | \
               whichAccumulator | storeAddress))
  
/* Returns an INCREMENT_ACCUMULATOR instruction
 * @param whichAccumulator  - Accumulator to increment: PACKETIZER_ACCUMULATOR_ZERO, etc.
 * @param accumulatorAddend - Addend to increment the accumulator by
 */
#define PACKETIZER_INCREMENT_ACCUMULATOR(whichAccumulator, accumulatorAddend)         \
  ((uint32_t) ((PACKETIZER_OPCODE_INCREMENT_ACCUMULATOR << PACKETIZER_OPCODE_SHIFT) | \
               whichAccumulator | (accumulatorAddend & PACKETIZER_ACCUMULATOR_MASK)))

/* Returns an INSERT_ACCUMULATOR instruction
 * @param whichAccumulator - Accumulator to insert: PACKETIZER_ACCUMULATOR_ZERO, etc.
 */
#define PACKETIZER_INSERT_ACCUMULATOR(whichAccumulator)                            \
  ((uint32_t) ((PACKETIZER_OPCODE_INSERT_ACCUMULATOR << PACKETIZER_OPCODE_SHIFT) | \
               whichAccumulator))

/* Returns a COND_LOAD_ACCUMULATOR instruction
 * @param whichAccumulator - Index of the accumulator to conditionally modify
 * @param statusMask       - Status mask to conditionally test
 * @param loadTrue         - Value to load if all the bits in the statusMask
 *                           are set within the packetizer's state
 * @param loadFalse        - Value to load if not all the status bits are set
 */
#define PACKETIZER_COND_LOAD_ACCUMULATOR(whichAccumulator, statusMask, loadTrue, loadFalse) \
  ((uint32_t) ((PACKETIZER_OPCODE_COND_LOAD_ACCUMULATOR << PACKETIZER_OPCODE_SHIFT) |       \
               whichAccumulator | statusMask |                                              \
               ((loadFalse & PACKETIZER_ACCUMULATOR_MASK) << PACKETIZER_ACCUMULATOR_BITS) | \
               (loadTrue & PACKETIZER_ACCUMULATOR_MASK)))

/* Returns an INSERT_TIMESTAMP instruction, which implicitly makes use of the present
 * packet's clock domain for selection of the timestamp.
 */
#define PACKETIZER_INSERT_TIMESTAMP \
  ((uint32_t) (PACKETIZER_OPCODE_INSERT_TIMESTAMP << PACKETIZER_OPCODE_SHIFT))

/* Returns a parameter word properly formatted with the paramters required by an upcoming
 * INSERT_PAYLOAD opcode.  This may be used with the PUSH_PARAM opcode.
 * @param dataBlockSize - Size of each data block (bytes per sample * number of channels)
 * @param overheadBytes - Number of bytes of overhead preceding the audio data (e.g. CIP 
 *                        header)
 */
#define PACKETIZER_PAYLOAD_SIZE_PARAMS(dataBlockSize, overheadBytes) \
  ((uint32_t) ((overheadBytes << PACKETIZER_DATA_BLOCK_SIZE_BITS) | \
               (dataBlockSize & PACKETIZER_DATA_BLOCK_SIZE_MASK)))

/* Returns an INSERT_PAYLOAD_SIZE instruction, which implicitly makes use of the present
 * number of samples captured within the packet's clock domain.  The data block size and 
 * overhead bytes must already be pushed onto the parameter stack, which will be popped as 
 * a side-effect.
 */
#define PACKETIZER_INSERT_PAYLOAD_SIZE \
  ((uint32_t) (PACKETIZER_OPCODE_INSERT_PAYLOAD_SIZE << PACKETIZER_OPCODE_SHIFT))

/* Returns an INSERT_BLOCK_COUNT instruction, which implicitly makes use of the present
 * data block counter for the packet's clock domain.
 */
#define PACKETIZER_INSERT_BLOCK_COUNT \
  ((uint32_t) (PACKETIZER_OPCODE_INSERT_BLOCK_COUNT << PACKETIZER_OPCODE_SHIFT))
  
/* Returns a LOAD_RING_OFFSET instruction
 * @param loadAddress - Address of the value to load into the ring offset
 */
#define PACKETIZER_LOAD_RING_OFFSET(loadAddress) \
  ((uint32_t) ((PACKETIZER_OPCODE_LOAD_RING_OFFSET << PACKETIZER_OPCODE_SHIFT) | loadAddress))

/* Returns a STORE_RING_OFFSET instruction
 * @param storeAddress - Address at which to store the ring offset
 */
#define PACKETIZER_STORE_RING_OFFSET(storeAddress)                                \
  ((uint32_t) ((PACKETIZER_OPCODE_STORE_RING_OFFSET << PACKETIZER_OPCODE_SHIFT) | \
               storeAddress))

/* Returns a parameter word propertly formatted to push for an upcoming AUDIO_SAMPLES 
 * instruction.
 * @param numChannels - Number of channels in the stream
 * @param mapAddress  - Address of the audio channel map to be encoded
 */
#define PACKETIZER_AUDIO_PARAMS(numChannels, mapAddress)               \
  ((uint32_t) (((numChannels - 1) << PACKETIZER_CHANNEL_COUNT_SHIFT) | \
               mapAddress))
                                    
/* Returns an AUDIO_SAMPLES instruction, which implicitly makes use of the present 
 * number of samples captured within the packet's clock domain.  The number of channels
 * in each packet and the address of the audio channel map must already be pushed onto
 * the parameter stack, which is popped as a side-effect.
 * @param sampleSize  - Size, in bytes, of each sample
 * @param numChannels - Number of channels in the stream
 */
#define PACKETIZER_AUDIO_SAMPLES(sampleSize)                                  \
  ((uint32_t) ((PACKETIZER_OPCODE_AUDIO_SAMPLES << PACKETIZER_OPCODE_SHIFT) | \
               ((sampleSize - 1) & PACKETIZER_SAMPLE_SIZE_MASK)));
                                    
/* Returns a LINK instruction */
#define PACKETIZER_LINK ((uint32_t) (PACKETIZER_OPCODE_LINK << PACKETIZER_OPCODE_SHIFT))

/* Returns a JUMP instruction
 * @param jumpAddress - Address to jump to for execution
 */
#define PACKETIZER_JUMP(jumpAddress)                                 \
  ((uint32_t) ((PACKETIZER_OPCODE_JUMP << PACKETIZER_OPCODE_SHIFT) | \
               jumpAddress))

/* Returns a PUSH_PARAM instruction with the passed data
 * @param paramData - Parameter data to be pushed onto the stack
 */
#define PACKETIZER_PUSH_PARAM(paramData) \
  ((uint32_t) ((PACKETIZER_OPCODE_PUSH_PARAM << PACKETIZER_OPCODE_SHIFT) | \
               (paramData & PACKETIZER_PARAM_STACK_MASK)))
  
/*
 * Depacketizer definitions
 */

/* I/O control commands specific to the depacketizer */
#define IOC_CLEAR_MATCHERS         _IO('d', 0x20)

typedef struct {
  uint32_t matchUnit;
  uint32_t configAction;
  uint32_t matchVector;
  uint64_t matchStreamId;
} MatcherConfig;

/* Valid configuration actions to be performed on a match unit:
 * MATCHER_DISABLE - Disables the match unit
 * MATCHER_ENABLE  - Enables the match unit with a new ID
 */
#  define MATCHER_DISABLE  0x00000000
#  define MATCHER_ENABLE   0x00000001

#define IOC_CONFIG_MATCHER         _IOW('d', 0x21, MatcherConfig)

typedef struct {
  uint32_t matchUnit;
  uint32_t oldVector;
  uint32_t newVector;
  uint32_t ringStateOffset;
} MatcherRelocation;

#define IOC_RELOCATE_MATCHER       _IOW('d', 0x22, MatcherRelocation)

#define IOC_LOCATE_VECTOR_TABLE    _IOW('d', 0x23, uint32_t)

typedef struct {
  uint32_t            matchUnit;
  ClockDomainSettings clockDomainSettings;
} ClockRecoverySettings;

#define IOC_CONFIG_CLOCK_RECOVERY  _IOW('d', 0x23, ClockRecoverySettings)

typedef struct {
  uint32_t versionMajor;
  uint32_t versionMinor;
  uint32_t maxInstructions;
  uint32_t maxParameters;
  uint32_t maxClockDomains;
  uint32_t maxStreams;
} DepacketizerCaps;

#define IOC_GET_DEPACKETIZER_CAPS  _IOR('d', 0x24, DepacketizerCaps)

/* Type definitions and macros for depacketizer microcode */


/* Parameter maxima
 * NOTE - The first constant is related to the DEPACKETIZER_MAX_STREAM_SLOTS
 *        constant within Audio_Depacketizer_Params.vhd.  
 *        Should the VHDL constant ever be changed, these must be revisited.
 */
#define DEPACKETIZER_MAX_STREAM_SLOTS   (32)
#define DEPACKETIZER_MAX_STREAMS       (128)

/* Opcode definitions */
#define DEPACKETIZER_OPCODE_NOP                (0x00)
#define DEPACKETIZER_OPCODE_AUDIO_SAMPLES      (0x01)
#define DEPACKETIZER_OPCODE_BRANCH_BAD_PACKET  (0x02)
#define DEPACKETIZER_OPCODE_STORE_RING_OFFSET  (0x03)
#define DEPACKETIZER_OPCODE_CHECK_SEQUENCE     (0x04)
#define DEPACKETIZER_OPCODE_BRANCH_CHECK       (0x05)
#define DEPACKETIZER_OPCODE_WRITE_PARAM        (0x06)
#define DEPACKETIZER_OPCODE_STOP               (0x0F)

/* Instruction field constant definitions */
#define DEPACKETIZER_OPCODE_SHIFT       (24)
#define DEPACKETIZER_SAMPLE_SIZE_MASK   (0x07)
#define DEPACKETIZER_PACKET_SLOT_MASK   (0x03F)
#define DEPACKETIZER_PACKET_SLOT_SHIFT  (3)

#define DEPACKETIZER_PARAM_SOURCE_FALSE        (0x00)
#define DEPACKETIZER_PARAM_SOURCE_TRUE         (0x01)
#define DEPACKETIZER_PARAM_SOURCE_TIMESTAMP    (0x02)
#define DEPACKETIZER_PARAM_SOURCE_TS_VALID     (0x03)
#define DEPACKETIZER_PARAM_SOURCE_TS_MODULUS   (0x04)
#define DEPACKETIZER_PARAM_SOURCE_RING_OFFSET  (0x05)

/* Number of bits in a timestamp interval mask */
#define DEPACKETIZER_SYT_MASK_BITS   (5)
#define DEPACKETIZER_SYT_MASK_SHIFT  (32 - DEPACKETIZER_SYT_MASK_BITS)

/* Returns a NOP instruction
 * This instruction, not surprisingly, simply consumes a cycle with no side-effects.
 */
#define DEPACKETIZER_NOP \
  ((uint32_t) (DEPACKETIZER_OPCODE_NOP << DEPACKETIZER_OPCODE_SHIFT))

/* Returns an AUDIO_SAMPLES instruction, looping through the number of available 
 * samples in the packet.
 *
 * @param sampleSize  - Size, in bytes, of each sample
 * @param numChannels - Number of channels in the stream
 */
#define DEPACKETIZER_AUDIO_SAMPLES(sampleSize, numChannels)                                              \
  ((uint32_t) ((DEPACKETIZER_OPCODE_AUDIO_SAMPLES << DEPACKETIZER_OPCODE_SHIFT)                        | \
               (((numChannels - 1) & DEPACKETIZER_PACKET_SLOT_MASK) << DEPACKETIZER_PACKET_SLOT_SHIFT) | \
               ((sampleSize - 1) & DEPACKETIZER_SAMPLE_SIZE_MASK)))

/* Returns a CHECK_SEQUENCE instruction
 * This checks the received AVBTP sequence number against the expected value.  This instruction
 * sets the "check" status bit accordingly for use by a future BRANCH_CHECK; the status bit
 * is not valid until two instructions later, so a delay slot must be inserted between this
 * instruction and a BRANCH_CHECK.
 *
 * @param descriptorBase - Base address of the containing descriptor
 * @param sequenceOffset - Relative offset at which to fetch and store the sequence number
 */
#define DEPACKETIZER_CHECK_SEQUENCE(descriptorBase, sequenceOffset)                \
  ((uint32_t) ((DEPACKETIZER_OPCODE_CHECK_SEQUENCE << DEPACKETIZER_OPCODE_SHIFT) | \
               (descriptorBase + sequenceOffset)))

/* Returns a BRANCH_BAD_PACKET instruction
 * This instruction will stall the processor until the packet is complete.  If the packet is good,
 * execution continues at the next address.  If it is bad, the branch path is taken.
 *
 * @param descriptorBase - Base address of the containing descriptor
 * @param branchTarget   - Branch target taken if the packet is bad
 */
#define DEPACKETIZER_BRANCH_BAD_PACKET(descriptorBase, branchTarget)                  \
  ((uint32_t) ((DEPACKETIZER_OPCODE_BRANCH_BAD_PACKET << DEPACKETIZER_OPCODE_SHIFT) | \
               (descriptorBase + branchTarget)))

/* Returns a BRANCH_CHECK instruction
 * This will branch if a prior "check" instruction resulted in a true condition.
 *
 * @param descriptorBase - Base address of the containing descriptor
 * @param branchTarget   - Branch target taken if the prior check instruction evaluated to true
 */
#define DEPACKETIZER_BRANCH_CHECK(descriptorBase, branchTarget)                  \
  ((uint32_t) ((DEPACKETIZER_OPCODE_BRANCH_CHECK << DEPACKETIZER_OPCODE_SHIFT) | \
               (descriptorBase + branchTarget)))

/* Returns a WRITE_PARAM instruction
 * This writes a parameter to an address over the parameter memory interface.  Parameters are
 * derived from stream status information (timestamps, buffer write pointers, etc.) and are used
 * by downstream audio processing logic to maintain proper presentation time, mute during stream
 * errors, etc.
 *
 * @param paramAddress      - Address at which to write the parameter
 * @param paramAddressWidth - Width, in bits, of the parameter address.  This is dependent upon the
 *                            DEPACKETIZER_MAX_PARAMS VHDL constant which the depacketizer was built
 *                            against.
 * @param paramSource       - Which of the enumerated sources to write
 */
#define DEPACKETIZER_WRITE_PARAM(paramAddress, paramAddressWidth, paramSource)  \
  ((uint32_t) ((DEPACKETIZER_OPCODE_WRITE_PARAM << DEPACKETIZER_OPCODE_SHIFT) | \
               (paramSource << paramAddressWidth)                             | \
               paramAddress))

/* Returns a STORE_RING_OFFSET instruction
 * The ring offset is the buffer pointer into cache memory for the stream, and forms the lower
 * portion of the cache memory write address.  It advances automatically with each byte of audio
 * data written during execution of an AUDIO_SAMPLES opcode, and must be stored back if the packet
 * is determined to be valid and error-free.
 *
 * @param descriptorBase - Base address of the containing descriptor
 * @param storeOffset    - Relative offset at which to store the ring offset
 */
#define DEPACKETIZER_STORE_RING_OFFSET(descriptorBase, storeAddress)                  \
  ((uint32_t) ((DEPACKETIZER_OPCODE_STORE_RING_OFFSET << DEPACKETIZER_OPCODE_SHIFT) | \
               (descriptorBase + storeAddress)))
  
/* Returns a STOP instruction
 * This stops the microengine's activity for the present packet.  It will return to an
 * idle state until another packet is matched by an activated match unit.
 */
#define DEPACKETIZER_STOP \
  ((uint32_t) (DEPACKETIZER_OPCODE_STOP << DEPACKETIZER_OPCODE_SHIFT))

/* Returns an instruction word containing an initial sequence number for a stream.
 * This special encoded value suppresses a sequence error from being generated when the
 * first packet of a newly-initialized stream is received, since there is no way for
 * the engine to know what sequence number will come first.
 */
#define DEPACKETIZER_INITIALIZE_SEQUENCE  ((uint32_t) (0x01 << AVBTP_SEQUENCE_NUMBER_BITS))

/* Returns an instruction word containing a ring buffer offset, useful for allocating a location
 * for storing the present offset into a clock domain's ring buffers.  This also encodes
 * the SYT interval mask used for calculating the sample modulus for packets with a valid
 * timestamp.
 *
 * @param ringOffset  - Starting offset for the stream
 * @param sytInterval - Timestamp interval used for the clock domain
 */
#define DEPACKETIZER_RING_OFFSET_WORD(ringOffset, sytInterval) \
  (((sytInterval - 1) << DEPACKETIZER_SYT_MASK_SHIFT) | ((uint32_t) ringOffset))

#endif
