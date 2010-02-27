/*
 * libc/sysdeps/linux/microblaze/crt1.S -- Initial program entry point for linux/microblaze
 *
 *  Copyright (C) 2009       Meyer Sound Laboratories
 *  Copyright (C) 2003       John Williams <jwilliams@itee.uq.edu.au>
 *  Copyright (C) 2001,2002  NEC Corporation
 *  Copyright (C) 2001,2002  Miles Bader <miles@gnu.org>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License.  See the file COPYING.LIB in the main
 * directory of this archive for more details.
 *
 * Written by Miles Bader <miles@gnu.org>
 */

#include <asm/clinkage.h>

/* Upon entry, the stack contains the following data:
	argc, argv[0], ..., argv[argc-1], 0, envp[0], ..., 0
*/

	.text
C_ENTRY(_start):
	
	/* Load SDAs */
	la	r2, r0, C_SYMBOL_NAME(_SDA2_BASE_) /* in the original source r2 was SDA, and r13 was SDA2, no idea why */
	la	r13, r0, C_SYMBOL_NAME(_SDA_BASE_)

	/*
	Preparing arguments for uClibc's startup routine.
	The routine has 6 arguments, so 5 of them are placed
	into registers, one on the stack
	*/
	
	la      r5, r0, C_SYMBOL_NAME(main) /* Arg 0: main() */
	lw	r6, r0, r1		    /* Arg 1: argc   */
	addi	r7, r1, 4		    /* Arg 2: argv   */
	la	r8, r0, _init               /* Arg 3: init   */
	la	r9, r0, _fini               /* Arg 4: fini   */
	/*
	To properly follow rgister/stack calling convention on MicroBlaze,
	the stack offsets were taken from compiled C source:

void __uClibc_mini_main(int (*main)(int, char **, char **), int argc, char **argv, void (*app_init)(void), void (*app_fini)(void), void *stack_end)
{
    __uClibc_main(main,argc,argv,app_init,app_fini,0,stack_end);
}

	*/
	
	addik	r1,r1,-56                   /* stack frame */
	addk	r19,r1,r0
	
	swi	r1,r1,28                    /* Arg 6: stack_end */
	addk	r10,r0,r0                   /* Arg 5: rtld_fini = NULL */

	/* tail-call uClibc's startup routine */
	brid	C_SYMBOL_NAME(__uClibc_main)
	nop                                 /* Unfilled delay slot */
	
/* Define a symbol for the first piece of initialized data.  */
	.data
	.globl __data_start
__data_start:
	.long 0
	.weak data_start
	data_start = __data_start
