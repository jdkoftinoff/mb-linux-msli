/*
 * Copyright (C) 2006 Atmark Techno, Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#ifndef _ASM_MICROBLAZE_TIMEX_H
#define _ASM_MICROBLAZE_TIMEX_H

#define CLOCK_TICK_RATE 1000 /* Timer input freq. */

typedef unsigned long cycles_t;

#ifdef CONFIG_SELFMOD_TIMER
extern cycles_t get_cycles(void);
#else
extern unsigned int microblaze_timer_baseaddr;
static inline cycles_t get_cycles(void)
{
  return *(volatile unsigned int __force *)(microblaze_timer_baseaddr + 0x18);
}
#endif

#endif /* _ASM_TIMEX_H */
