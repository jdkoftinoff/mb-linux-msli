/*
 * clock_nanosleep() for uClibc
 *
 * Copyright (C) 2010 Nathan Sidwell <nathan@codesourcery.com>
 *
 * Licensed under the LGPL v2.1, see the file COPYING.LIB in this tarball.
 */

#include <sys/syscall.h>
#include <time.h>

#ifdef __NR_clock_nanosleep
_syscall4(int, clock_nanosleep, clockid_t, clock_id, int, flags, __const struct timespec*, req, struct timespec*, rem)
#else
int clock_nanosleep(clockid_t clock_id, int flags, __const struct timespec* req, struct timespec* rem)
{
        __set_errno (ENOSYS);
	return -1;
}
#endif
