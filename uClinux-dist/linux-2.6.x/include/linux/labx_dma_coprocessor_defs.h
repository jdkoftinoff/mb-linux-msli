/*
 *  linux/include/linux/labx_dma_coprocessor_defs.h
 *
 *  Lab X Technologies DMA coprocessor definitions
 *
 *  Written by Chris Wulff (chris.wulff@labxtechnologies.com)
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

#ifndef _LABX_DMA_COPROCESSOR_DEFS_H_
#define _LABX_DMA_COPROCESSOR_DEFS_H_

#include <linux/types.h>

/* Type definition for the instructions used by the coprocessor */
#define DMA_INSTRUCTION_WIDTH  32
typedef uint32_t DMAInstruction;
  
/* Types dependent upon the parameters package */
#define DMA_OPCODE_BITS           8
#define DMA_BRANCH_CONDITION_BITS 3
#define DMA_ALU_OPCODE_BITS       3
#define DMA_INDEX_VALUE_BITS      12
#define DMA_CACHE_ADDRESS_BITS    16
#define DMA_LOGICAL_OP_BITS       2

typedef struct
{
	int indexSelectBits;
	int sourceSelectBits;
	int aluSelectBits;

} DMAConfiguration;

typedef enum
{
	eDMAIndex0 = 0,
	eDMAIndex1 = 1,
	eDMAIndex2 = 2,
	eDMAIndex3 = 3,

} EDMAIndex;

typedef enum
{
	eDMAALU0 = 0,

} EDMAALU;

typedef enum
{
	eDMASourceCode  = 0,
	eDMASourceParam = 1,

} EDMASource;

typedef enum
{
	eDMANoIncB = 0,
	eDMAIncB   = 1,

} EDMAIncB;

typedef enum
{
	eDMABlockCountNone = 0,
	eDMABlockCountAlu  = 1

} EDMABlockCount;

typedef enum
{
	eDMAStoreFull = 0,
	eDMAStoreMasked = 1

} EDMAStoreMask;
  
/* Definitions for instruction opcodes */
#define DMA_OPCODE_NOP                    0x00
#define DMA_OPCODE_LOAD_INDEX             0x01
#define DMA_OPCODE_ADD_INDEX              0x02
#define DMA_OPCODE_BRANCH                 0x03
#define DMA_OPCODE_BRANCH_ALU             0x04
#define DMA_OPCODE_LOAD_ALU               0x05
#define DMA_OPCODE_LOAD_CACHE_ADDRESS     0x06
#define DMA_OPCODE_STORE_CACHE_ADDRESS    0x07
#define DMA_OPCODE_INC_CACHE_ADDRESS      0x08
#define DMA_OPCODE_LOAD_CACHE_INCREMENT   0x09
#define DMA_OPCODE_LOAD_BUFFER_ADDRESS    0x0A
#define DMA_OPCODE_STORE_BUFFER_ADDRESS   0x0B
#define DMA_OPCODE_INC_BUFFER_ADDRESS     0x0C
#define DMA_OPCODE_LOAD_BUFFER_INCREMENT  0x0D
#define DMA_OPCODE_BUFFER_READ            0x0E
#define DMA_OPCODE_BUFFER_WRITE           0x0F
#define DMA_OPCODE_JOIN_TRANSFER          0x10
#define DMA_OPCODE_STORE_ALU              0x11
#define DMA_OPCODE_LOAD_INDEX_INDIRECT    0x12
#define DMA_OPCODE_STORE_INDEX_INDIRECT   0x13
#define DMA_OPCODE_ADDRESS_ALIGN          0x14
#define DMA_OPCODE_INDEX_LOGICAL          0x15
#define DMA_OPCODE_SET_BYTE_ORDER         0x16
#define DMA_OPCODE_STOP                   0xFF

/* Constants identifying different branch conditions */
typedef enum
{
	DMA_BRANCH_ALWAYS = 0,
	DMA_BRANCH_EQ     = 1,
	DMA_BRANCH_NEQ    = 2,
	DMA_BRANCH_LT     = 3,
	DMA_BRANCH_LT_EQ  = 4,
	DMA_BRANCH_GT     = 5,
	DMA_BRANCH_GT_EQ  = 6,

} EDMABranchCondition;
 
/* Constants identifying the logical operations */
typedef enum
{
	DMA_LOGICAL_OR = 0,
	DMA_LOGICAL_AND = 1,
	DMA_LOGICAL_XOR = 2,

} EDMALogicalOperation;
 
/* Constants identifying address generator increments */
typedef enum
{
	DMA_INCREMENT_A = 0,
	DMA_INCREMENT_B = 1,

} EDMAIncrement;
  
/* Constants for opcodes supported by the arithmetic unit internal to the */
/* Dma_Coprocessor module. */
typedef enum
{
	DMA_ALU_OPCODE_LOAD_ACCUM = 0, /* Load a 32-bit value into the accumulator */
	DMA_ALU_OPCODE_ADD        = 1, /* Add a 32-bit value to the accumulator */
	DMA_ALU_OPCODE_SUB        = 2, /* Subtract a 32-bit value from the accumulator */
	DMA_ALU_OPCODE_MUL        = 3, /* Integer multiply (18x18 signed on DSP48A) */
	DMA_ALU_OPCODE_MULFIX     = 4, /* Fixed point multiply (32 bit signed fraction operand * 35 bits from accumulator) */

} EDMAALUOpcode;

/* Constants for the bit flags that can be passed into the address generator opcodes */
typedef enum
{
	DMA_ADDRESS_LOAD_BASE        = 0x10, /* Load the base address */
	DMA_ADDRESS_LOAD_OFFSET      = 0x08, /* Load the offset (no base address) */
	DMA_ADDRESS_LOAD_INDEX       = 0x04, /* Load the index */
	DMA_ADDRESS_LOAD_SIZE        = 0x02, /* Load the block size */
	DMA_ADDRESS_ADD_BASE         = 0x01, /* Add the existing base/index */
	DMA_ADDRESS_LOAD_OFFSET_BASE = 0x09, /* Load the offset and add the existing base/index */

} EDMAAddressFlags;

/* Constants for the interlock flag */
typedef enum
{
	eNoInterlock = 0,
	eInterlock = 1,

} EInterlock;

/* Field ranges for opcode bits */
#define DMA_OPCODE_SHIFT(config)             (DMA_INSTRUCTION_WIDTH - DMA_OPCODE_BITS)
#define  DMA_INCREMENT_B_SHIFT(config)       (DMA_OPCODE_SHIFT(config) - 1)
#define   DMA_TRANSFER_ALU_BC_SHIFT(config)  (DMA_INCREMENT_B_SHIFT(config) - 1)
#define  DMA_WHICH_INCREMENT_SHIFT(config)   (DMA_OPCODE_SHIFT(config) - 1)
#define  DMA_INDEX_SELECT_SHIFT(config)      (DMA_OPCODE_SHIFT(config) - config.indexSelectBits)
#define   DMA_BRANCH_CONDITION_SHIFT(config) (DMA_INDEX_SELECT_SHIFT(config) - DMA_BRANCH_CONDITION_BITS)
#define   DMA_LOGICAL_OP_SHIFT(config)       (DMA_INDEX_SELECT_SHIFT(config) - DMA_LOGICAL_OP_BITS)
#define   DMA_INDEX_VALUE_SHIFT(config)      (0)
#define   DMA_INDEX_VALUE_MASK(config)       (((1<<DMA_INDEX_VALUE_BITS)-1)<<DMA_INDEX_VALUE_SHIFT(config))
#define   DMA_SOURCE_SELECT_SHIFT(config)    (DMA_INDEX_SELECT_SHIFT(config) - config.sourceSelectBits)
#define    DMA_STORE_MASKED_SHIFT(config)    (DMA_SOURCE_SELECT_SHIFT(config)-1)
#define    DMA_LOAD_BASE_SHIFT(config)       (DMA_SOURCE_SELECT_SHIFT(config)-1)
#define     DMA_LOAD_OFFSET_SHIFT(config)    (DMA_LOAD_BASE_SHIFT(config)-1)
#define      DMA_LOAD_INDEX_SHIFT(config)    (DMA_LOAD_OFFSET_SHIFT(config)-1)
#define       DMA_LOAD_SIZE_SHIFT(config)    (DMA_LOAD_INDEX_SHIFT(config)-1)
#define        DMA_ADD_BASE_SHIFT(config)    (DMA_LOAD_SIZE_SHIFT(config)-1)
#define    DMA_INDEX_SELECT_2_SHIFT(config)  (DMA_SOURCE_SELECT_SHIFT(config) - config.indexSelectBits)
#define    DMA_ALU_OPCODE_SHIFT(config)      (DMA_SOURCE_SELECT_SHIFT(config) - DMA_ALU_OPCODE_BITS)
#define     DMA_ALU_SELECT_SHIFT(config)     (DMA_ALU_OPCODE_SHIFT(config) - config.aluSelectBits)
#define      DMA_INTERLOCK_SHIFT(config)     (DMA_ALU_SELECT_SHIFT(config) - 1)
#define DMA_CODE_ADDRESS_SHIFT(config)       (0)
#define DMA_CACHE_ADDRESS_SHIFT(config)      (0)
#define DMA_CACHE_ADDRESS_MASK(config)       (((1<<DMA_CACHE_ADDRESS_BITS)-1)<<DMA_CACHE_ADDRESS_SHIFT(config))
#define DMA_BUFFER_ADDRESS_SHIFT(config)     (0)
#define DMA_TRANSFER_LENGTH_SHIFT(config)    (0)

/* Returns a NOP instruction */
static inline DMAInstruction DMA_NOP(DMAConfiguration config)
{
	return (DMA_OPCODE_NOP << DMA_OPCODE_SHIFT(config));
}
  
/* Returns a LOAD_INDEX instruction */
/* @param index_Select - Selects which of the index counters to load */
/* @param index_Value  - Value to load into the index counter */
static inline DMAInstruction DMA_LOAD_INDEX(DMAConfiguration config, EDMAIndex index_Select, uint32_t index_Value)
{
	return ((DMA_OPCODE_LOAD_INDEX << DMA_OPCODE_SHIFT(config)) |
		(index_Select << DMA_INDEX_SELECT_SHIFT(config))    |
		((index_Value << DMA_INDEX_VALUE_SHIFT(config))&DMA_INDEX_VALUE_MASK(config)));
}
  
/* Returns a LOAD_INDEX_INDIRECT instruction */
/* @param index_Select   - Index counter to use as an offset to the load address */
/* @param source_Select  - Which RAM block to load from */
/* @param index_Select_2 - Selects which of the index counters to load */
/* @param load_Address   - Base location to address the RAM block */
/* @param interlock      - Hold the parameter interlock or not */
static inline DMAInstruction DMA_LOAD_INDEX_INDIRECT(DMAConfiguration config, EDMAIndex index_Select,
	EDMASource source_Select, EDMAIndex index_Select_2, uint32_t load_Address, EInterlock interlock)
{
	return ((DMA_OPCODE_LOAD_INDEX_INDIRECT << DMA_OPCODE_SHIFT(config)) |
		(index_Select << DMA_INDEX_SELECT_SHIFT(config))             |
		(source_Select << DMA_SOURCE_SELECT_SHIFT(config))           |
		(index_Select_2 << DMA_INDEX_SELECT_2_SHIFT(config))         |
		(interlock << DMA_INTERLOCK_SHIFT(config))                   |
        	(load_Address << DMA_CODE_ADDRESS_SHIFT(config)));
}

/* Returns a STORE_INDEX_INDIRECT instruction */
/* @param index_Select   - Index counter to use as an offset to the store address */
/* @param source_Select - Which RAM block to load from */
/* @param index_Select_2 - Selects which of the index counters to store */
/* @param store_Address  - Base location to address the RAM block */
/* @param interlock      - Hold the parameter interlock or not */
static inline DMAInstruction DMA_STORE_INDEX_INDIRECT(DMAConfiguration config, EDMAIndex index_Select,
	EDMASource source_Select, EDMAIndex index_Select_2, uint32_t store_Address, EInterlock interlock)
{
	return ((DMA_OPCODE_STORE_INDEX_INDIRECT << DMA_OPCODE_SHIFT(config)) |
		(index_Select << DMA_INDEX_SELECT_SHIFT(config))              |
		(source_Select << DMA_SOURCE_SELECT_SHIFT(config))            |
		(index_Select_2 << DMA_INDEX_SELECT_2_SHIFT(config))          |
		(interlock << DMA_INTERLOCK_SHIFT(config))                    |
        	(store_Address << DMA_CODE_ADDRESS_SHIFT(config)));
}

/* Returns an ADD_INDEX instruction */
/* @param index_Select - Selects which of the index counters to add to */
/* @param index_Addend - Value to add to the index counter */
static inline DMAInstruction DMA_ADD_INDEX(DMAConfiguration config, EDMAIndex index_Select, int32_t index_Addend)
{
	return ((DMA_OPCODE_ADD_INDEX << DMA_OPCODE_SHIFT(config)) |
		(index_Select << DMA_INDEX_SELECT_SHIFT(config))   |
		((index_Addend << DMA_INDEX_VALUE_SHIFT(config))&DMA_INDEX_VALUE_MASK(config)));
}
  
/* Returns a BRANCH instruction; every BRANCH instruction must be followed by a  */
/* branch target, see inline DMAInstruction Branch_Target(). */
/* @param index_Select     - Selects which of the index counters to load */
/* @param branch_Condition - Condition for which the branch is taken */
/* @param branch_Value     - Value used in evaluating the branch condition */
static inline DMAInstruction DMA_BRANCH(DMAConfiguration config, EDMAIndex index_Select, EDMABranchCondition branch_Condition, uint32_t branch_Value)
{
	return ((DMA_OPCODE_BRANCH << DMA_OPCODE_SHIFT(config))          |
		(index_Select << DMA_INDEX_SELECT_SHIFT(config))         |
		(branch_Condition << DMA_BRANCH_CONDITION_SHIFT(config)) |
		((branch_Value << DMA_INDEX_VALUE_SHIFT(config))&DMA_INDEX_VALUE_MASK(config)));
}
  
/* Returns a BRANCH instruction; every BRANCH instruction must be followed by a  */
/* branch target, see inline DMAInstruction Branch_Target(). */
/* @param alu_Select       - Selects which ALU flags to load */
/* @param branch_Condition - Condition for which the branch is taken */
static inline DMAInstruction DMA_BRANCH_ALU(DMAConfiguration config, EDMAALU alu_Select, EDMABranchCondition branch_Condition)
{
	return ((DMA_OPCODE_BRANCH_ALU << DMA_OPCODE_SHIFT(config)) | 
		(alu_Select << DMA_ALU_SELECT_SHIFT(config))        |
		(branch_Condition << DMA_BRANCH_CONDITION_SHIFT(config)));
}

/* Returns an instruction word containing a branch target. */
static inline DMAInstruction DMA_BRANCH_TARGET(DMAConfiguration config, uint32_t target_Address)
{
	return (target_Address);
}
  
/* Returns a LOAD_ALU instruction */
/* @param alu_Select    - Which ALU unit to load */
/* @param source_Select - Which RAM block to load from */
/* @param load_Address  - Base location to address the RAM block */
/* @param index_Select  - Index counter to use as an offset to the load address */
/* @param alu_Opcode    - ALU operation to perform with the parameter and the accumulator */
/* @param interlock     - Hold the parameter interlock or not */
static inline DMAInstruction DMA_LOAD_ALU(DMAConfiguration config, EDMAALU alu_Select, EDMASource source_Select,
	uint32_t load_Address, EDMAIndex index_Select, EDMAALUOpcode alu_Opcode, EInterlock interlock)
{
	return ((DMA_OPCODE_LOAD_ALU << DMA_OPCODE_SHIFT(config))  |
		(alu_Select << DMA_ALU_SELECT_SHIFT(config))       |
		(source_Select << DMA_SOURCE_SELECT_SHIFT(config)) |
		(load_Address << DMA_CODE_ADDRESS_SHIFT(config))   |
		(index_Select << DMA_INDEX_SELECT_SHIFT(config))   |
		(interlock << DMA_INTERLOCK_SHIFT(config))         |
		(alu_Opcode << DMA_ALU_OPCODE_SHIFT(config)));
}
  
/* Returns a STORE_ALU instruction */
/* @param alu_Select    - Which ALU unit to load */
/* @param source_Select - Which RAM block to load from */
/* @param store_Address - Base location to address the RAM block */
/* @param index_Select  - Index counter to use as an offset to the store address */
/* @param interlock     - Hold the parameter interlock or not */
static inline DMAInstruction DMA_STORE_ALU(DMAConfiguration config, EDMAALU alu_Select, EDMASource source_Select,
	uint32_t store_Address, EDMAIndex index_Select, EInterlock interlock)
{
	return ((DMA_OPCODE_STORE_ALU << DMA_OPCODE_SHIFT(config)) |
		(alu_Select << DMA_ALU_SELECT_SHIFT(config))       |
		(source_Select << DMA_SOURCE_SELECT_SHIFT(config)) |
		(store_Address << DMA_CODE_ADDRESS_SHIFT(config))  |
		(interlock << DMA_INTERLOCK_SHIFT(config))         |
		(index_Select << DMA_INDEX_SELECT_SHIFT(config)));
}

/* Returns a LOAD_CACHE_ADDRESS instruction, used for configuring cache addressing */
/* for subsequent data transfers. */
/* @param load_Address - Base location of the address and modulus mask to be loaded from microcode RAM */
/* @param source_Select - Which RAM block to load from */
/* @param address_Flags - Which address part is being loaded */
/* @param index_Select - Index counter to use as an offset to the base address */
static inline DMAInstruction DMA_LOAD_CACHE_ADDRESS(DMAConfiguration config, uint32_t load_Address,
	EDMASource source_Select, EDMAAddressFlags address_Flags, EDMAIndex index_Select)
{
	return ((DMA_OPCODE_LOAD_CACHE_ADDRESS << DMA_OPCODE_SHIFT(config)) |
		(load_Address << DMA_CODE_ADDRESS_SHIFT(config))            |
		(source_Select << DMA_SOURCE_SELECT_SHIFT(config))          |
		( ((uint32_t)address_Flags) << DMA_ADD_BASE_SHIFT(config))  |
		(index_Select << DMA_INDEX_SELECT_SHIFT(config)));
}
  
/* Returns a LOAD_CACHE_ADDRESS instruction, used for configuring cache addressing */
/* for subsequent data transfers. */
/* @param load_Address - Base location of the address and modulus mask to be loaded from microcode RAM */
/* @param source_Select - Which RAM block to load from */
/* @param index_Select - Index counter to use as an offset to the base address */
static inline DMAInstruction DMA_LOAD_CACHE_BASE(DMAConfiguration config, uint32_t load_Address,
	EDMASource source_Select, EDMAIndex index_Select)
{
	return DMA_LOAD_CACHE_ADDRESS(config, load_Address, source_Select, DMA_ADDRESS_LOAD_BASE, index_Select);
}

/* Returns a LOAD_CACHE_ADDRESS instruction, used for configuring cache addressing */
/* for subsequent data transfers. */
/* @param load_Address - Base location of the address and modulus mask to be loaded from microcode RAM */
/* @param source_Select - Which RAM block to load from */
/* @param index_Select - Index counter to use as an offset to the base address */
static inline DMAInstruction DMA_LOAD_CACHE_OFFSET(DMAConfiguration config, uint32_t load_Address,
	EDMASource source_Select, int add_Base, EDMAIndex index_Select)
{
	return DMA_LOAD_CACHE_ADDRESS(config, load_Address, source_Select,
		(add_Base) ? DMA_ADDRESS_LOAD_OFFSET_BASE : DMA_ADDRESS_LOAD_OFFSET, index_Select);
}

/* Returns a LOAD_CACHE_ADDRESS instruction, used for configuring cache addressing */
/* for subsequent data transfers. */
/* @param load_Address - Base location of the address and modulus mask to be loaded from microcode RAM */
/* @param source_Select - Which RAM block to load from */
/* @param index_Select - Index counter to use as an offset to the base address */
static inline DMAInstruction DMA_LOAD_CACHE_INDEX(DMAConfiguration config, uint32_t load_Address,
	EDMASource source_Select, EDMAIndex index_Select)
{
	return DMA_LOAD_CACHE_ADDRESS(config, load_Address, source_Select, DMA_ADDRESS_LOAD_INDEX, index_Select);
}

/* Returns a LOAD_CACHE_ADDRESS instruction, used for configuring cache addressing */
/* for subsequent data transfers. */
/* @param load_Address - Base location of the address and modulus mask to be loaded from microcode RAM */
/* @param source_Select - Which RAM block to load from */
/* @param index_Select - Index counter to use as an offset to the base address */
static inline DMAInstruction DMA_LOAD_CACHE_SIZE(DMAConfiguration config, uint32_t load_Address,
	EDMASource source_Select, EDMAIndex index_Select)
{
	return DMA_LOAD_CACHE_ADDRESS(config, load_Address, source_Select, DMA_ADDRESS_LOAD_SIZE, index_Select);
}

/* Returns a STORE_CACHE_ADDRESS instruction, used for committing the present cache address */
/* to a location in microcode RAM. */
/* @param store_Address - Base location at which to store the address into microcode RAM */
/* @param index_Select  - Index counter to use as an offset to the base address */
static inline DMAInstruction DMA_STORE_CACHE_ADDRESS(DMAConfiguration config, uint32_t store_Address,
	EDMASource source_Select, EDMAStoreMask mask, EDMAIndex index_Select)
{
	return ((DMA_OPCODE_STORE_CACHE_ADDRESS << DMA_OPCODE_SHIFT(config)) |
		(store_Address << DMA_CODE_ADDRESS_SHIFT(config))            |
		(source_Select << DMA_SOURCE_SELECT_SHIFT(config))           |
		(mask          << DMA_STORE_MASKED_SHIFT(config))            |
		(index_Select << DMA_INDEX_SELECT_SHIFT(config)));
}
                                   
/* Returns a INC_CACHE_ADDRESS instruction, which manually increments the cache address */
/* using one of two previously-loaded increments. */
/* @param which_Increment - Selection of which increment value (A or B) to apply */
static inline DMAInstruction DMA_INC_CACHE_ADDRESS(DMAConfiguration config, EDMAIncrement which_Increment)
{
	return ((DMA_OPCODE_INC_CACHE_ADDRESS << DMA_OPCODE_SHIFT(config)) |
		(which_Increment << DMA_WHICH_INCREMENT_SHIFT(config)));
}
  
/* Returns a LOAD_CACHE_INCREMENT instruction, with an increment selection and value encoded */
/* within the instruction word. */
/* @param which_Increment - Selection of which increment value (A or B) to load */
/* @param increment_Value - Signed value for the selected increment */
static inline DMAInstruction DMA_LOAD_CACHE_INCREMENT(DMAConfiguration config, EDMAIncrement which_Increment, int32_t increment_Value)
{
	return ((DMA_OPCODE_LOAD_CACHE_INCREMENT << DMA_OPCODE_SHIFT(config))   |
		(which_Increment << DMA_WHICH_INCREMENT_SHIFT(config))          |
		((increment_Value << DMA_CACHE_ADDRESS_SHIFT(config))&DMA_CACHE_ADDRESS_MASK(config)));
}
  
/* Returns a LOAD_BUFFER_ADDRESS instruction, used for configuring cache addressing */
/* for subsequent data transfers. */
/* @param load_Address - Base location of the address and modulus mask to be loaded from microcode RAM */
/* @param source_Select - Which RAM block to load from */
/* @param address_Flags - Which address part is being loaded */
/* @param index_Select - Index counter to use as an offset to the base address */
static inline DMAInstruction DMA_LOAD_BUFFER_ADDRESS(DMAConfiguration config, uint32_t load_Address,
	EDMASource source_Select, EDMAAddressFlags address_Flags, EDMAIndex index_Select)
{
	return ((DMA_OPCODE_LOAD_BUFFER_ADDRESS << DMA_OPCODE_SHIFT(config)) |
		(load_Address << DMA_CODE_ADDRESS_SHIFT(config))            |
		(source_Select << DMA_SOURCE_SELECT_SHIFT(config))          |
		( ((uint32_t)address_Flags) << DMA_ADD_BASE_SHIFT(config))  |
		(index_Select << DMA_INDEX_SELECT_SHIFT(config)));
}
  
/* Returns a LOAD_BUFFER_ADDRESS instruction, used for configuring cache addressing */
/* for subsequent data transfers. */
/* @param load_Address - Base location of the address and modulus mask to be loaded from microcode RAM */
/* @param source_Select - Which RAM block to load from */
/* @param index_Select - Index counter to use as an offset to the base address */
static inline DMAInstruction DMA_LOAD_BUFFER_BASE(DMAConfiguration config, uint32_t load_Address,
	EDMASource source_Select, EDMAIndex index_Select)
{
	return DMA_LOAD_BUFFER_ADDRESS(config, load_Address, source_Select, DMA_ADDRESS_LOAD_BASE, index_Select);
}

/* Returns a LOAD_BUFFER_ADDRESS instruction, used for configuring cache addressing */
/* for subsequent data transfers. */
/* @param load_Address - Base location of the address and modulus mask to be loaded from microcode RAM */
/* @param source_Select - Which RAM block to load from */
/* @param index_Select - Index counter to use as an offset to the base address */
static inline DMAInstruction DMA_LOAD_BUFFER_OFFSET(DMAConfiguration config, uint32_t load_Address,
	EDMASource source_Select, int add_Base, EDMAIndex index_Select)
{
	return DMA_LOAD_BUFFER_ADDRESS(config, load_Address, source_Select,
		(add_Base) ? DMA_ADDRESS_LOAD_OFFSET_BASE : DMA_ADDRESS_LOAD_OFFSET, index_Select);
}

/* Returns a LOAD_BUFFER_ADDRESS instruction, used for configuring cache addressing */
/* for subsequent data transfers. */
/* @param load_Address - Base location of the address and modulus mask to be loaded from microcode RAM */
/* @param source_Select - Which RAM block to load from */
/* @param index_Select - Index counter to use as an offset to the base address */
static inline DMAInstruction DMA_LOAD_BUFFER_INDEX(DMAConfiguration config, uint32_t load_Address,
	EDMASource source_Select, EDMAIndex index_Select)
{
	return DMA_LOAD_BUFFER_ADDRESS(config, load_Address, source_Select, DMA_ADDRESS_LOAD_INDEX, index_Select);
}

/* Returns a LOAD_BUFFER_ADDRESS instruction, used for configuring cache addressing */
/* for subsequent data transfers. */
/* @param load_Address - Base location of the address and modulus mask to be loaded from microcode RAM */
/* @param source_Select - Which RAM block to load from */
/* @param index_Select - Index counter to use as an offset to the base address */
static inline DMAInstruction DMA_LOAD_BUFFER_SIZE(DMAConfiguration config, uint32_t load_Address,
	EDMASource source_Select, EDMAIndex index_Select)
{
	return DMA_LOAD_BUFFER_ADDRESS(config, load_Address, source_Select, DMA_ADDRESS_LOAD_SIZE, index_Select);
}

/* Returns a STORE_BUFFER_ADDRESS instruction, used for committing the present buffer address */
/* to a location in microcode RAM. */
/* @param store_Address - Base location at which to store the address into microcode RAM */
/* @param index_Select  - Index counter to use as an offset to the base address */
static inline DMAInstruction DMA_STORE_BUFFER_ADDRESS(DMAConfiguration config, uint32_t store_Address,
	EDMASource source_Select, EDMAStoreMask mask, EDMAIndex index_Select)
{
	return ((DMA_OPCODE_STORE_BUFFER_ADDRESS << DMA_OPCODE_SHIFT(config)) |
		(store_Address << DMA_CODE_ADDRESS_SHIFT(config))             |
		(source_Select << DMA_SOURCE_SELECT_SHIFT(config))            |
		(mask          << DMA_STORE_MASKED_SHIFT(config))             |
		(index_Select << DMA_INDEX_SELECT_SHIFT(config)));
}
  
/* Returns a INC_BUFFER_ADDRESS instruction, which manually increments the buffer address */
/* using one of two previously-loaded increments. */
/* @param which_Increment - Selection of which increment value (A or B) to apply */
static inline DMAInstruction DMA_INC_BUFFER_ADDRESS(DMAConfiguration config, EDMAIncrement which_Increment)
{
	return ((DMA_OPCODE_INC_BUFFER_ADDRESS << DMA_OPCODE_SHIFT(config)) |
		(which_Increment << DMA_WHICH_INCREMENT_SHIFT(config)));
}
  
/* Returns a BUFFER_READ instruction configured for the passed transfer length. */
/* This instruction will not issue a request while another one is active, and will return  */
/* as soon as the request has been acknowledged; at that point, the transfer is in progress.   */
/* */
/* During the transfer, the cache address is incremented by its 'A' increment on every data word  */
/* transferred, and should not be tampered with until the transfer has been "joined".  The buffer */
/* address, however, is incremented by its 'A' increment as soon as the transfer has been acknowledged, */
/* and does not undergo any additional auto-increments.  As such, it may be manipulated, stored, etc. */
/* while the transfer is in progress. */
static inline DMAInstruction DMA_BUFFER_READ(DMAConfiguration config, EDMAIncB incB, uint32_t transfer_Length)
{
	return ((DMA_OPCODE_BUFFER_READ << DMA_OPCODE_SHIFT(config)) |
		(incB << DMA_INCREMENT_B_SHIFT(config)) |
		(transfer_Length << DMA_CODE_ADDRESS_SHIFT(config)));
}
  
/* Returns a BUFFER_WRITE instruction configured for the passed transfer length. */
/* This instruction will not issue a request while another one is active, and will return  */
/* as soon as the request has been acknowledged; at that point, the transfer is in progress.   */
/* */
/* During the transfer, the cache address is incremented by its 'A' increment on every data word  */
/* transferred, and should not be tampered with until the transfer has been "joined".  The buffer */
/* address, however, is incremented by its 'A' increment as soon as the transfer has been acknowledged, */
/* and does not undergo any additional auto-increments.  As such, it may be manipulated, stored, etc. */
/* while the transfer is in progress. */
static inline DMAInstruction DMA_BUFFER_WRITE(DMAConfiguration config, EDMAIncB incB,
                                              EDMABlockCount bc, uint32_t transfer_Length)
{
	return ((DMA_OPCODE_BUFFER_WRITE << DMA_OPCODE_SHIFT(config)) |
		(incB << DMA_INCREMENT_B_SHIFT(config)) |
                (bc << DMA_TRANSFER_ALU_BC_SHIFT(config)) |
		(transfer_Length << DMA_CODE_ADDRESS_SHIFT(config)));
}

/* Returns a LOAD_BUFFER_INCREMENT instruction, with an increment selection encoded */
/* within the instruction word.  This instruction must be followed by a word containing */
/* the increment to be used. */
/* @param which_Increment - Selection of which increment value (A or B) to load */
static inline DMAInstruction DMA_LOAD_BUFFER_INCREMENT(DMAConfiguration config, EDMAIncrement which_Increment)
{
	return ((DMA_OPCODE_LOAD_BUFFER_INCREMENT << DMA_OPCODE_SHIFT(config)) |
		(which_Increment << DMA_WHICH_INCREMENT_SHIFT(config)));
}

/* Returns an instruction word containing an increment value for buffer addresses.   */
/* @param increment_Value - Signed value for the selected increment */
static inline DMAInstruction DMA_BUFFER_INCREMENT(DMAConfiguration config, int32_t increment_Value)
{
	return (increment_Value);
}
  
/* Returns a JOIN_TRANSFER instruction */
static inline DMAInstruction DMA_JOIN_TRANSFER(DMAConfiguration config)
{
	return (DMA_OPCODE_JOIN_TRANSFER << DMA_OPCODE_SHIFT(config));
}
  
/* Returns a STOP instruction */
static inline DMAInstruction DMA_STOP(DMAConfiguration config)
{
	return (DMA_OPCODE_STOP << DMA_OPCODE_SHIFT(config));
}

/* Returns an ADDRESS_ALIGN instruction, used for aligning cache and buffer pointers */
/* @param address_Modulus - Modulus which will be applied to check for alignment */
static inline DMAInstruction DMA_ADDRESS_ALIGN(DMAConfiguration config, uint32_t address_Modulus)
{
	return ((DMA_OPCODE_ADDRESS_ALIGN << DMA_OPCODE_SHIFT(config)) |
		((address_Modulus-1) << DMA_CODE_ADDRESS_SHIFT(config)));
}

/* Returns an INDEX_LOGICAL instruction used to perform a logical operation on an index register */
/* @param index_Select - Index register to use */
/* @param index_Op     - Logical operation to perform */
/* @param logic_Value  - Constant to be used as the second input to the logical operation */
static inline DMAInstruction DMA_INDEX_LOGICAL(DMAConfiguration config, EDMAIndex index_Select,
	EDMALogicalOperation index_Op, uint32_t logic_Value)
{
        return ((DMA_OPCODE_INDEX_LOGICAL << DMA_OPCODE_SHIFT(config)) |
                (index_Select << DMA_INDEX_SELECT_SHIFT(config))       |
		(index_Op     << DMA_LOGICAL_OP_SHIFT(config))         |
                ((logic_Value << DMA_INDEX_VALUE_SHIFT(config))&DMA_INDEX_VALUE_MASK(config)));
}

/* Returns an SET_BYTE_ORDER instruction used to set byte ordering from the cache */
/* @param byte_Order - Byte order (each nybble defines a byte; ex. 0x0123 for standard order) */
static inline DMAInstruction DMA_SET_BYTE_ORDER(DMAConfiguration config, uint32_t byte_Order)
{
        return ((DMA_OPCODE_SET_BYTE_ORDER << DMA_OPCODE_SHIFT(config)) | byte_Order);
}

/* DMA ioctls */
#define DMA_IOC_LOAD_DESCRIPTOR        (0x81)
#define DMA_IOC_COPY_DESCRIPTOR        (0x82)
#define DMA_MAX_CONFIG_WORDS 1024
typedef struct {
  uint32_t  offset;
  uint32_t  numWords;
  uint32_t *configWords;
} DMAConfigWords;

#define DMA_IOC_START_CHANNEL          (0x83)
#define DMA_IOC_STOP_CHANNEL           (0x84)

#define DMA_IOC_ALLOC_BUFFERS          (0x85)
#define DMA_IOC_FREE_BUFFERS           (0x86)
typedef struct {
  uint32_t nBufs;
  uint32_t size;
  void**   buffers;
} DMAAlloc;

#define DMA_IOC_START_DMA              (0x87)
#define DMA_IOC_STOP_DMA               (0x88)

typedef struct {
  uint32_t channel;
  uint32_t address;
} DMAVector;
#define DMA_IOC_SET_VECTOR             (0x89)

/* Indices for identifying memory resources */
#define LABX_DMA_ADDRESS_RANGE_RESOURCE    (0)
#define LABX_DMA_NUM_RESOURCES             (1)
  
#endif /* _LABX_DMA_COPROCESSOR_DEFS_H_ */

