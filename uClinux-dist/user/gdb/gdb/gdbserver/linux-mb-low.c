/* linux-mb-low.c -- uClinux/mb target code for gdbserver

   Written by John Williams <john.williams@petalogix.com>
   based on patches by Miles Bader <miles@gnu.org>

   Copyright 2003 Free Software Foundation, Inc.

   This file is part of GDB.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.  */

#include <errno.h>

#include <sys/ptrace.h>
#include <asm/ptrace.h>

#include "server.h"
#include "linux-low.h"

/* ptrace.h in 2.6 doesn't define these the same way 2.4 did.
   Detect it, and make backward compatible definitions */
#ifndef PT_SIZE
#include <asm/asm-offsets.h>
#define PT_GPR(n) (n)*sizeof(unsigned int)
#define PT_PSW PT_MSR
#endif

/* This matches the layout of `mbe_reg_names' in mb-tdep.c (not sure if
   it really has too [e.g., perhaps the registers get remapped by name
   using ../regformats/reg-mb.dat or something], but why not), and maps
   register numbers into struct pt_regs offsets.  */
static int mb_regmap[] = {
  -1,    	PT_GPR(1),	PT_GPR(2),	PT_GPR(3),
  PT_GPR(4),	PT_GPR(5),	PT_GPR(6),	PT_GPR(7),
  PT_GPR(8),	PT_GPR(9),	PT_GPR(10),	PT_GPR(11),
  PT_GPR(12),	PT_GPR(13),	PT_GPR(14),	PT_GPR(15),
  PT_GPR(16),	PT_GPR(17),	PT_GPR(18),	PT_GPR(19),
  PT_GPR(20),	PT_GPR(21),	PT_GPR(22),	PT_GPR(23),
  PT_GPR(24),	PT_GPR(25),	PT_GPR(26),	PT_GPR(27),
  PT_GPR(28),	PT_GPR(29),	PT_GPR(30),	PT_GPR(31),
  PT_PC, 	PT_PSW,		PT_EAR,		PT_ESR,
  PT_FSR
};
#define mb_num_regs (sizeof mb_regmap / sizeof mb_regmap[0])

static int
mb_cannot_store_register (int regno)
{
  return regno < 0 || regno >= mb_num_regs || mb_regmap[regno] < 0;
}

static int
mb_cannot_fetch_register (int regno)
{
  return regno < 0 || regno >= mb_num_regs || mb_regmap[regno] < 0;
}

static CORE_ADDR
mb_get_pc (void)
{
  unsigned long pc;
  collect_register_by_name ("pc", &pc);
  return (CORE_ADDR) pc;
}

static void
mb_set_pc (CORE_ADDR pc)
{
  unsigned long newpc = pc;
  supply_register_by_name ("pc", &newpc);
}

/* dbtrap insn */
/* brki r14, 0x60; */
typedef unsigned long mb_breakpoint_t;
static const mb_breakpoint_t mb_breakpoint = 0xb9cc0060;
#define mb_breakpoint_len 4

static int
mb_breakpoint_at (CORE_ADDR where)
{
  mb_breakpoint_t insn;

  (*the_target->read_memory) (where, (char *) &insn, mb_breakpoint_len);
  if (insn == mb_breakpoint)
    return 1;
  /* If necessary, recognize more trap instructions here.  GDB only uses the
     one.  */
  return 0;
}

/* We only place breakpoints in empty marker functions, and thread locking
   is outside of the function.  So rather than importing software single-step,
   we can just run until exit.
   
   r15 wis the subroutine link register, however we must also add offset of 8
   this is where the temp reinsert breakpoint should be inserted */
static CORE_ADDR
mb_reinsert_addr ()
{
  unsigned long pc;
  collect_register_by_name ("r15", &pc);
  return pc+8;
}

struct linux_target_ops the_low_target = {
  num_regs: 			mb_num_regs,
  regmap: 			mb_regmap,
  cannot_fetch_register:	mb_cannot_fetch_register,
  cannot_store_register:	mb_cannot_store_register,
  get_pc:			mb_get_pc,
  set_pc:			mb_set_pc,
  breakpoint:			(const char *) &mb_breakpoint,
  breakpoint_len:		mb_breakpoint_len,
  breakpoint_reinsert_addr:	mb_reinsert_addr,
  decr_pc_after_break:		0, //mb_breakpoint_len,
  breakpoint_at:		mb_breakpoint_at
};
