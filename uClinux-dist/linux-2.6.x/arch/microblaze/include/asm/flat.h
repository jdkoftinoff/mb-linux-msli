/*
 * uClinux flat-format executables
 *
 * Copyright (C) 2005 John Williams <jwilliams@itee.uq.edu.au>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file COPYING in the main directory of this
 * archive for more details.
 */

#ifndef _ASM_MICROBLAZE_FLAT_H
#define _ASM_MICROBLAZE_FLAT_H

#include <asm/unaligned.h>

#define	flat_stack_align(sp) /* nothing needed */
#define	flat_argvp_envp_on_stack()	0
#define	flat_old_ram_flag(flags)	(flags)
#define	flat_reloc_valid(reloc, size)	((reloc) <= (size))

/*
 * Microblaze works a little differently from other arches, because
 * of the MICROBLAZE_64 reloc type. Here, a 32 bit address is split
 * over two instructions, an 'imm' instruction which provides the top
 * 16 bits, then the instruction "proper" which provides the low 16
 * bits.
 */

#define MICROBLAZE_FLAT_RELVAL_ADDR    0x1fffffff
#define MICROBLAZE_FLAT_RELVAL_PERSIST 0x60000000
#define MICROBLAZE_FLAT_RELVAL_GOTREL  0x20000000
#define MICROBLAZE_FLAT_RELVAL_PCREL   0x40000000
#define MICROBLAZE_FLAT_RELVAL_64      0x80000000

#define	flat_get_relocate_addr(rel)	(rel & MICROBLAZE_FLAT_RELVAL_ADDR)

static inline int flat_set_persistent(unsigned long relval,
				      unsigned long *persistent)
{
	if ((relval & MICROBLAZE_FLAT_RELVAL_PERSIST) == MICROBLAZE_FLAT_RELVAL_PERSIST) {
		*persistent = flat_get_relocate_addr(relval);
		return 1;
	}
	return 0;
}

/* Get a value from an instruction or memory location. This only
   looks at the 64/32 flag. */
static inline unsigned long
flat_get_value(unsigned long *rp, unsigned long relval)
{
	unsigned long addr;

	/* Is it a split 64/32 reference? */
	if (relval & MICROBLAZE_FLAT_RELVAL_64) {
		/* Grab the two halves of the reference */
		unsigned long val_hi, val_lo;

		val_hi = get_unaligned(rp);
		val_lo = get_unaligned(rp+1);

		/* Crack the address out */
		addr = ((val_hi & 0xffff) << 16) + (val_lo & 0xffff);
	} else {
		/* Get the address straight out */
		addr = get_unaligned(rp);
	}

	return addr;
}

/* Set a value into an instruction or memory location. This only
   looks at the 64/32 flag. */
static inline void
flat_set_value(unsigned long *rp, unsigned long addr, unsigned long relval)
{	
	/* Is this a split 64/32 reloc? */
	if (relval & MICROBLAZE_FLAT_RELVAL_64) {
		/* Get the two "halves" */
		unsigned long val_hi = get_unaligned(rp);
		unsigned long val_lo = get_unaligned(rp + 1);

		/* insert the address */
		val_hi = (val_hi & 0xffff0000) | addr >> 16;
		val_lo = (val_lo & 0xffff0000) | (addr & 0xffff);

		/* store the two halves back into memory */
		put_unaligned(val_hi, rp);
		put_unaligned(val_lo, rp+1);
	} else {
		/* Put it straight in, no messing around */
		put_unaligned(addr, rp);
	}
}

/*
 * Crack open a symbol reference and extract the address to be
 * relocated. rp is a potentially unaligned pointer to the
 * reference
 */

static inline unsigned long
flat_get_addr_from_rp(unsigned long *rp, unsigned long relval,
			unsigned long flags, unsigned long *persistent)
{
	unsigned long addr = flat_get_value(rp, relval);
	(void)flags;

	/* Is it a PC relative reference (to another segment)? */
	if (relval & MICROBLAZE_FLAT_RELVAL_PCREL) {
		/* Add the address. It will be removed again when storing. */
		addr += flat_get_relocate_addr(relval);
	}

	/* Is it a GOT relative reference (to another segment)? */
	if (relval & MICROBLAZE_FLAT_RELVAL_GOTREL) {
		/* Add the GOT address (held in persistent). It will be removed again when storing. */
		addr += *persistent;

		/* Save the difference between relval and the GOT for use when storing. */
		flat_set_value(rp, flat_get_relocate_addr(relval) - *persistent, relval);
	}

	return addr;
}

/*
 * Insert an address into the symbol reference at rp. rp is potentially
 * unaligned.
 */

static inline void
flat_put_addr_at_rp(unsigned long *rp, unsigned long addr, unsigned long relval)
{
	/* Is it a PC relative reference (to another segment)? */
	if (relval & MICROBLAZE_FLAT_RELVAL_PCREL) {
		/* Turn the address back into PC relative. */
		addr -= (unsigned long)rp;
	}

	/* Is it a GOT relative reference (to another segment)? */
	if (relval & MICROBLAZE_FLAT_RELVAL_GOTREL) {
		/* Turn the address back into a relative one and */
		/* add the GOT delta (saved in *rp on load). */
		addr -= (unsigned long)rp;
		addr += flat_get_value(rp, relval);
	}

	flat_set_value(rp, addr, relval);
}

#endif /* _ASM_MICROBLAZE_FLAT_H */
