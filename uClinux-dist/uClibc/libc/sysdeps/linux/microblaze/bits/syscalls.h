#ifndef _BITS_SYSCALLS_H
#define _BITS_SYSCALLS_H
#ifndef _SYSCALL_H
# error "Never use <bits/syscalls.h> directly; include <sys/syscall.h> instead."
#endif

#include <features.h>

#undef INTERNAL_SYSCALL_ERROR_P
#define INTERNAL_SYSCALL_ERROR_P(val, err) \
  ((unsigned long int)(val) >= (unsigned long int)(-4095))

#undef INTERNAL_SYSCALL_ERRNO
#define INTERNAL_SYSCALL_ERRNO(val, err)   (-(val))

#undef INTERNAL_SYSCALL_DECL
#define INTERNAL_SYSCALL_DECL(err) long int err

#undef INLINE_SYSCALL
#define INLINE_SYSCALL(name, nr, args...)				\
  ({									\
    INTERNAL_SYSCALL_DECL(err);						\
    long int result_var = INTERNAL_SYSCALL(name, err, nr, args);	\
    if (INTERNAL_SYSCALL_ERROR_P(result_var, err)) {			\
      errno = INTERNAL_SYSCALL_ERRNO(result_var, err);			\
      result_var = -1L;							\
    }									\
    result_var;								\
  })

#define INTERNAL_SYSCALL(name, err, nr, args...)	\
  INTERNAL_SYSCALL_##nr(name, err, args)

#define INTERNAL_SYSCALL_0(name, err, args)				\
  ({									\
    long __ret;								\
    asm volatile ("addik	r12, r0, %1	\n\t"			\
		  "brki	r14, 0x8	\n\t"				\
		  "addk	%0, r3, r0	\n\t"				\
		  : "=r" (__ret)					\
		  : "i" (__NR_##name)					\
		  : "r3", "r4", "r5", "r6", "r7", "r8", "r9",		\
		    "r10", "r11", "r12", "r14", "cc", "memory");	\
    err = __ret;							\
    __ret;								\
  })

#define INTERNAL_SYSCALL_1(name, err, arg1)				\
  ({									\
    long __ret;								\
    asm volatile ("addk	r5, r0, %2	\n\t"				\
		  "addik	r12, r0, %1	\n\t"			\
		  "brki	r14, 0x8	\n\t"				\
		  "addk	%0, r3, r0	\n\t"				\
		  : "=r" (__ret)					\
		  : "i" (__NR_##name),					\
		    "r" ((long)arg1)					\
		  : "r3", "r4", "r5", "r6", "r7", "r8", "r9",		\
		    "r10", "r11", "r12", "r14", "cc", "memory");	\
    err = __ret;							\
    __ret;								\
  })

#define INTERNAL_SYSCALL_2(name, err, arg1, arg2)			\
  ({									\
    long __ret;								\
    asm volatile ("addk	r5, r0, %2	\n\t"				\
		  "addk	r6, r0, %3	\n\t"				\
		  "addik	r12, r0, %1	\n\t"			\
		  "brki	r14, 0x8	\n\t"				\
		  "addk	%0, r3, r0	\n\t"				\
		  : "=r" (__ret)					\
		  : "i" (__NR_##name),					\
		    "r" ((long)arg1),					\
		    "r" ((long)arg2)					\
		  : "r3", "r4", "r5", "r6", "r7", "r8", "r9",		\
		    "r10", "r11", "r12", "r14", "cc", "memory");	\
    err = __ret;							\
    __ret;								\
  })

#define INTERNAL_SYSCALL_3(name, err, arg1, arg2, arg3)			\
  ({									\
    long __ret;								\
    asm volatile ("addk	r5, r0, %2	\n\t"				\
		  "addk	r6, r0, %3	\n\t"				\
		  "addk	r7, r0, %4	\n\t"				\
		  "addik	r12, r0, %1	\n\t"			\
		  "brki	r14, 0x8	\n\t"				\
		  "addk	%0, r3, r0	\n\t"				\
		  : "=r" (__ret)					\
		  : "i" (__NR_##name),					\
		    "r" ((long)arg1),					\
		    "r" ((long)arg2),					\
		    "r" ((long)arg3)					\
		  : "r3", "r4", "r5", "r6", "r7", "r8", "r9",		\
		    "r10", "r11", "r12", "r14", "cc", "memory");	\
    err = __ret;							\
    __ret;								\
  })

#define INTERNAL_SYSCALL_4(name, err, arg1, arg2, arg3, arg4)		\
  ({									\
    long __ret;								\
    asm volatile ("addk	r5, r0, %2	\n\t"				\
		  "addk	r6, r0, %3	\n\t"				\
		  "addk	r7, r0, %4	\n\t"				\
		  "addk	r8, r0, %5	\n\t"				\
		  "addik	r12, r0, %1	\n\t"			\
		  "brki	r14, 0x8	\n\t"				\
		  "addk	%0, r3, r0	\n\t"				\
		  : "=r" (__ret)					\
		  : "i" (__NR_##name),					\
		    "r" ((long)arg1),					\
		    "r" ((long)arg2),					\
		    "r" ((long)arg3),					\
		    "r" ((long)arg4)					\
		  : "r3", "r4", "r5", "r6", "r7", "r8", "r9",		\
		    "r10", "r11","r12", "r14", "cc", "memory");		\
    err = __ret;							\
    __ret;								\
  })

#define INTERNAL_SYSCALL_5(name, err, arg1, arg2, arg3, arg4, arg5)	\
  ({									\
    long __ret;								\
    asm volatile ("addk	r5, r0, %2	\n\t"				\
		  "addk	r6, r0, %3	\n\t"				\
		  "addk	r7, r0, %4	\n\t"				\
		  "addk	r8, r0, %5	\n\t"				\
		  "addk	r9, r0, %6	\n\t"				\
		  "addik	r12, r0, %1	\n\t"			\
		  "brki	r14, 0x8	\n\t"				\
		  "addk	%0, r3, r0	\n\t"				\
		  : "=r" (__ret)					\
		  : "i" (__NR_##name),					\
		    "r" ((long)arg1),					\
		    "r" ((long)arg2),					\
		    "r" ((long)arg3),					\
		    "r" ((long)arg4),					\
		    "r" ((long)arg5)					\
		  : "r3", "r4", "r5", "r6", "r7", "r8", "r9",		\
		    "r10", "r11", "r12", "r14", "cc", "memory");	\
    err = __ret;							\
    __ret;								\
  })

#define INTERNAL_SYSCALL_6(name, err, arg1, arg2, arg3, arg4, arg5, arg6) \
  ({									\
    long __ret;								\
      asm volatile ("addk	r5, r0, %2	\n\t"			\
		    "addk	r6, r0, %3	\n\t"			\
		    "addk	r7, r0, %4	\n\t"			\
		    "addk	r8, r0, %5	\n\t"			\
		    "addk	r9, r0, %6	\n\t"			\
		    "addk	r10, r0, %7	\n\t"			\
		    "addik	r12, r0, %1	\n\t"			\
		    "brki	r14, 0x8	\n\t"			\
		    "addk	%0, r3, r0	\n\t"			\
		    : "=r" (__ret)					\
		    : "i" (__NR_##name),				\
		    "r" ((long)arg1),					\
		    "r" ((long)arg2),					\
		    "r" ((long)arg3),					\
		    "r" ((long)arg4),					\
		    "r" ((long)arg5),					\
		    "r" ((long)arg6)					\
		    : "r3", "r4", "r5", "r6", "r7", "r8", "r9",		\
		    "r10", "r11","r12", "r14", "cc", "memory");		\
    err = __ret;							\
    __ret;								\
  })

#define __syscall_return(type, res)					\
  do {									\
    /* user-visible error numbers are in the range -1 - -4095:		\
       see <asm-microblaze/errno.h> */					\
    if ((unsigned long int)(res) >= (unsigned long int)(-4095)) {	\
      errno = -(res);							\
      res = -1;								\
    }									\
    return (type) (res);						\
  } while (0)

#define _syscall0(type, name)						\
type name(void)								\
{									\
	long __ret;							\
	asm volatile ("addik	r12, r0, %1	\n\t"			\
			"brki	r14, 0x8	\n\t"			\
			"addk	%0, r3, r0	\n\t"			\
			: "=r" (__ret)					\
			: "i" (__NR_##name)				\
			: "r3", "r4", "r5", "r6", "r7", "r8", "r9",	\
			  "r10", "r11", "r12", "r14", "cc", "memory");	\
	__syscall_return(type, __ret);					\
}

#define _syscall1(type, name, type1, arg1)				\
type name(type1 arg1)							\
{									\
	long __ret;							\
	asm volatile ("addk	r5, r0, %2	\n\t"			\
			"addik	r12, r0, %1	\n\t"			\
			"brki	r14, 0x8	\n\t"			\
			"addk	%0, r3, r0	\n\t"			\
			: "=r" (__ret)					\
			: "i" (__NR_##name),				\
			"r" ((long)arg1)				\
			: "r3", "r4", "r5", "r6", "r7", "r8", "r9",	\
			  "r10", "r11", "r12", "r14", "cc", "memory");	\
	__syscall_return(type, __ret);					\
}

#define _syscall2(type, name, type1, arg1, type2, arg2)			\
type name(type1 arg1, type2 arg2)					\
{									\
	long __ret;							\
	asm volatile ("addk	r5, r0, %2	\n\t"			\
			"addk	r6, r0, %3	\n\t"			\
			"addik	r12, r0, %1	\n\t"			\
			"brki	r14, 0x8	\n\t"			\
			"addk	%0, r3, r0	\n\t"			\
			: "=r" (__ret)					\
			: "i" (__NR_##name),				\
			"r" ((long)arg1),				\
			"r" ((long)arg2)				\
			: "r3", "r4", "r5", "r6", "r7", "r8", "r9",	\
			  "r10", "r11", "r12", "r14", "cc", "memory");	\
	__syscall_return(type, __ret);					\
}

#define _syscall3(type, name, type1, arg1, type2, arg2, type3, arg3)	\
type name(type1 arg1, type2 arg2, type3 arg3)				\
{									\
	long __ret;							\
	asm volatile ("addk	r5, r0, %2	\n\t"			\
			"addk	r6, r0, %3	\n\t"			\
			"addk	r7, r0, %4	\n\t"			\
			"addik	r12, r0, %1	\n\t"			\
			"brki	r14, 0x8	\n\t"			\
			"addk	%0, r3, r0	\n\t"			\
			: "=r" (__ret)					\
			: "i" (__NR_##name),				\
			"r" ((long)arg1),				\
			"r" ((long)arg2),				\
			"r" ((long)arg3)				\
			: "r3", "r4", "r5", "r6", "r7", "r8", "r9",	\
			  "r10", "r11", "r12", "r14", "cc", "memory");	\
	__syscall_return(type, __ret);					\
}

#define _syscall4(type, name, type1, arg1, type2, arg2, type3, arg3,	\
			type4, arg4)					\
type name(type1 arg1, type2 arg2, type3 arg3, type4 arg4)		\
{									\
	long __ret;							\
	asm volatile ("addk	r5, r0, %2	\n\t"			\
			"addk	r6, r0, %3	\n\t"			\
			"addk	r7, r0, %4	\n\t"			\
			"addk	r8, r0, %5	\n\t"			\
			"addik	r12, r0, %1	\n\t"			\
			"brki	r14, 0x8	\n\t"			\
			"addk	%0, r3, r0	\n\t"			\
			: "=r" (__ret)					\
			: "i" (__NR_##name),				\
			"r" ((long)arg1),				\
			"r" ((long)arg2),				\
			"r" ((long)arg3),				\
			"r" ((long)arg4)				\
			: "r3", "r4", "r5", "r6", "r7", "r8", "r9",	\
			  "r10", "r11","r12", "r14", "cc", "memory");	\
	__syscall_return(type, __ret);					\
}

#define _syscall5(type, name, type1, arg1, type2, arg2, type3, arg3,	\
			type4, arg4, type5, arg5)			\
type name(type1 arg1, type2 arg2, type3 arg3, type4 arg4, type5 arg5)	\
{									\
	long __ret;							\
	asm volatile ("addk	r5, r0, %2	\n\t"			\
			"addk	r6, r0, %3	\n\t"			\
			"addk	r7, r0, %4	\n\t"			\
			"addk	r8, r0, %5	\n\t"			\
			"addk	r9, r0, %6	\n\t"			\
			"addik	r12, r0, %1	\n\t"			\
			"brki	r14, 0x8	\n\t"			\
			"addk	%0, r3, r0	\n\t"			\
			: "=r" (__ret)					\
			: "i" (__NR_##name),				\
			"r" ((long)arg1),				\
			"r" ((long)arg2),				\
			"r" ((long)arg3),				\
			"r" ((long)arg4),				\
			"r" ((long)arg5)				\
			: "r3", "r4", "r5", "r6", "r7", "r8", "r9",	\
			  "r10", "r11", "r12", "r14", "cc", "memory");	\
	__syscall_return(type, __ret);					\
}

#define _syscall6(type, name, type1, arg1, type2, arg2, type3, arg3,	\
			type4, arg4, type5, arg5, type6, arg6)		\
type name(type1 arg1, type2 arg2, type3 arg3, type4 arg4, type5 arg5,	\
			type6 arg6)					\
{									\
	long __ret;							\
	asm volatile ("addk	r5, r0, %2	\n\t"			\
			"addk	r6, r0, %3	\n\t"			\
			"addk	r7, r0, %4	\n\t"			\
			"addk	r8, r0, %5	\n\t"			\
			"addk	r9, r0, %6	\n\t"			\
			"addk	r10, r0, %7	\n\t"			\
			"addik	r12, r0, %1	\n\t"			\
			"brki	r14, 0x8	\n\t"			\
			"addk	%0, r3, r0	\n\t"			\
			: "=r" (__ret)					\
			: "i" (__NR_##name),				\
			"r" ((long)arg1),				\
			"r" ((long)arg2),				\
			"r" ((long)arg3),				\
			"r" ((long)arg4),				\
			"r" ((long)arg5),				\
			"r" ((long)arg6)				\
			: "r3", "r4", "r5", "r6", "r7", "r8", "r9",	\
			  "r10", "r11","r12", "r14", "cc", "memory");	\
	__syscall_return(type, __ret);					\
}
#endif /* _BITS_SYSCALLS_H */
