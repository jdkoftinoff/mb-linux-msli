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
    register long int r3  __asm__ ("r3");				\
    register long int r12 __asm__ ("r12");				\
    r12=__NR_##name;							\
    __asm__ __volatile__						\
      ("brki	r14, 0x08	\n\t"					\
       : "=r" (r3)							\
       : "r" (r12)							\
       : "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r14",	\
	 "cc", "memory");						\
    err = r3;								\
    (long int) r3;							\
  })

#define INTERNAL_SYSCALL_1(name, err, a1)				\
  ({									\
    register long int r3  __asm__ ("r3");				\
    register long int r5  __asm__ ("r5");				\
    register long int r12 __asm__ ("r12");				\
    r12=__NR_##name;							\
    r5= (long int)a1;							\
    __asm__ __volatile__						\
      ("brki	r14, 0x08	\n\t"					\
       : "=r" (r3)							\
       : "r" (r12), "r" (r5)						\
       : "r4", "r6", "r7", "r8", "r9", "r10", "r11", "r14",		\
       "cc", "memory");							\
    err = r3;								\
    (long int) r3;							\
  })

#define INTERNAL_SYSCALL_2(name, err, a1, a2)				\
  ({									\
    register long int r3  __asm__ ("r3");				\
    register long int r5  __asm__ ("r5");				\
    register long int r6  __asm__ ("r6");				\
    register long int r12 __asm__ ("r12");				\
    r12=__NR_##name;							\
    r5= (long int)a1;							\
    r6= (long int)a2;							\
    __asm__ __volatile__						\
      ("brki	r14, 0x08	\n\t"					\
       : "=r" (r3)							\
       : "r" (r12), "r" (r5), "r" (r6)					\
       : "r4", "r7", "r8", "r9", "r10", "r11", "r14", "cc", "memory");	\
    err = r3;								\
    (long int) r3;							\
  })

#define INTERNAL_SYSCALL_3(name, err, a1, a2, a3)			\
  ({									\
    register long int r3  __asm__ ("r3");				\
    register long int r5  __asm__ ("r5");				\
    register long int r6  __asm__ ("r6");				\
    register long int r7  __asm__ ("r7");				\
    register long int r12 __asm__ ("r12");				\
    r12=__NR_##name;							\
    r5= (long int)a1;							\
    r6= (long int)a2;							\
    r7= (long int)a3;							\
    __asm__ __volatile__						\
      ("brki	r14, 0x08	\n\t"					\
       : "=r" (r3)							\
       : "r" (r12), "r" (r5), "r" (r6), "r" (r7)			\
       : "r4", "r8", "r9", "r10", "r11", "r14", "cc", "memory");	\
    err = r3;								\
    (long int) r3;							\
  })

#define INTERNAL_SYSCALL_4(name, err, a1, a2, a3, a4)			\
  ({									\
    register long int r3  __asm__ ("r3");				\
    register long int r5  __asm__ ("r5");				\
    register long int r6  __asm__ ("r6");				\
    register long int r7  __asm__ ("r7");				\
    register long int r8  __asm__ ("r8");				\
    register long int r12 __asm__ ("r12");				\
    r12=__NR_##name;							\
    r5= (long int)a1;							\
    r6= (long int)a2;							\
    r7= (long int)a3;							\
    r8= (long int)a4;							\
    __asm__ __volatile__						\
      ("brki	r14, 0x08	\n\t"					\
	 : "=r" (r3)							\
       : "r" (r12), "r" (r5), "r" (r6), "r" (r7), "r" (r8)		\
       : "r4", "r9", "r10", "r11", "r14", "cc", "memory");		\
    err = r3;								\
    (long int) r3;							\
  })

#define INTERNAL_SYSCALL_5(name, err, a1, a2, a3, a4, a5)		\
  ({									\
    register long int r3  __asm__ ("r3");				\
    register long int r5  __asm__ ("r5");				\
    register long int r6  __asm__ ("r6");				\
    register long int r7  __asm__ ("r7");				\
    register long int r8  __asm__ ("r8");				\
    register long int r9  __asm__ ("r9");				\
    register long int r12 __asm__ ("r12");				\
    r12=__NR_##name;							\
    r5= (long int)a1;							\
    r6= (long int)a2;							\
    r7= (long int)a3;							\
    r8= (long int)a4;							\
    r9= (long int)a5;							\
    __asm__ __volatile__						\
      ("brki	r14, 0x08	\n\t"					\
       : "=r" (r3)							\
       : "r" (r12), "r" (r5), "r" (r6), "r" (r7), "r" (r8), "r" (r9)	\
       : "r4", "r10", "r11", "r14", "cc", "memory");			\
    err = r3;								\
    (long int) r3;							\
  })

#define INTERNAL_SYSCALL_6(name, err, a1, a2, a3, a4, a5, a6)		\
  ({									\
    register long int r3  __asm__ ("r3");				\
    register long int r5  __asm__ ("r5");				\
    register long int r6  __asm__ ("r6");				\
    register long int r7  __asm__ ("r7");				\
    register long int r8  __asm__ ("r8");				\
    register long int r9  __asm__ ("r9");				\
    register long int r10 __asm__ ("r10");				\
    register long int r12 __asm__ ("r12");				\
    r12=__NR_##name;							\
    r5= (long int)a1;							\
    r6= (long int)a2;							\
    r7= (long int)a3;							\
    r8= (long int)a4;							\
    r9= (long int)a5;							\
    r10= (long int)a6;							\
    __asm__ __volatile__						\
      ("brki	r14, 0x08	\n\t"					\
       : "=r" (r3)							\
       : "r" (r12), "r" (r5), "r" (r6), "r" (r7), "r" (r8), "r" (r9),	\
	 "r" (r10)							\
       : "r4", "r11", "r14", "cc", "memory");				\
    err = r3;								\
    (long int) r3;							\
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
  type name(void)							\
  {									\
    long int __ret;							\
    register long int r3  __asm__ ("r3");				\
    register long int r12 __asm__ ("r12");				\
    r12=__NR_##name;							\
    __asm__ __volatile__						\
      ("brki	r14, 0x8	\n\t"					\
       : "=r" (r3)							\
       : "r" (r12)							\
       : "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r14",	\
	 "cc", "memory");						\
    __ret = r3;								\
    __syscall_return(type, __ret);					\
  }

#define _syscall1(type, name, type1, arg1)				\
  type name(type1 arg1)							\
  {									\
    long int __ret;							\
    register long int r3  __asm__ ("r3");				\
    register long int r5  __asm__ ("r5");				\
    register long int r12 __asm__ ("r12");				\
    r5= (long int)arg1;							\
    r12=__NR_##name;							\
    __asm__ __volatile__						\
      ("brki	r14, 0x8	\n\t"					\
       : "=r" (r3)							\
       : "r" (r12), "r" (r5)						\
       : "r4", "r6", "r7", "r8", "r9", "r10", "r11", "r14",		\
	 "cc", "memory");						\
    __ret = r3;								\
    __syscall_return(type, __ret);					\
  }

#define _syscall2(type, name, type1, arg1, type2, arg2)			\
  type name(type1 arg1, type2 arg2)					\
  {									\
    long int __ret;							\
    register long int r3  __asm__ ("r3");				\
    register long int r5  __asm__ ("r5");				\
    register long int r6  __asm__ ("r6");				\
    register long int r12 __asm__ ("r12");				\
    r5= (long int)arg1;							\
    r6= (long int)arg2;							\
    r12=__NR_##name;							\
    __asm__ __volatile__						\
      ("brki	r14, 0x8	\n\t"					\
       : "=r" (r3)							\
       : "r" (r12), "r" (r5), "r" (r6)					\
       : "r4", "r7", "r8", "r9", "r10", "r11", "r14", "cc", "memory");  \
    __ret = r3;								\
    __syscall_return(type, __ret);					\
  }

#define _syscall3(type, name, type1, arg1, type2, arg2, type3, arg3)	\
  type name(type1 arg1, type2 arg2, type3 arg3)				\
  {									\
    long int __ret;							\
    register long int r3  __asm__ ("r3");				\
    register long int r5  __asm__ ("r5");				\
    register long int r6  __asm__ ("r6");				\
    register long int r7  __asm__ ("r7");				\
    register long int r12 __asm__ ("r12");				\
    r5= (long int)arg1;							\
    r6= (long int)arg2;							\
    r7= (long int)arg3;							\
    r12=__NR_##name;							\
    __asm__ __volatile__						\
      ("brki	r14, 0x8	\n\t"					\
       : "=r" (r3)							\
       : "r" (r12), "r" (r5), "r" (r6), "r" (r7)			\
       : "r4", "r8", "r9", "r10", "r11", "r14", "cc", "memory");	\
    __ret = r3;								\
    __syscall_return(type, __ret);					\
  }

#define _syscall4(type, name, type1, arg1, type2, arg2, type3, arg3,	\
		  type4, arg4)						\
  type name(type1 arg1, type2 arg2, type3 arg3, type4 arg4)		\
  {									\
    long int __ret;							\
    register long int r3  __asm__ ("r3");				\
    register long int r5  __asm__ ("r5");				\
    register long int r6  __asm__ ("r6");				\
    register long int r7  __asm__ ("r7");				\
    register long int r8  __asm__ ("r8");				\
    register long int r12 __asm__ ("r12");				\
    r5= (long int)arg1;							\
    r6= (long int)arg2;							\
    r7= (long int)arg3;							\
    r8= (long int)arg4;							\
    r12=__NR_##name;							\
    __asm__ __volatile__						\
      ("brki	r14, 0x8	\n\t"					\
       : "=r" (r3)							\
       : "r" (r12), "r" (r5), "r" (r6), "r" (r7), "r" (r8)		\
       : "r4", "r9", "r10", "r11", "r14", "cc", "memory");		\
    __ret = r3;								\
    __syscall_return(type, __ret);					\
  }

#define _syscall5(type, name, type1, arg1, type2, arg2, type3, arg3,	\
		  type4, arg4, type5, arg5)				\
  type name(type1 arg1, type2 arg2, type3 arg3, type4 arg4, type5 arg5)	\
  {									\
    long int __ret;							\
    register long int r3  __asm__ ("r3");				\
    register long int r5  __asm__ ("r5");				\
    register long int r6  __asm__ ("r6");				\
    register long int r7  __asm__ ("r7");				\
    register long int r8  __asm__ ("r8");				\
    register long int r9  __asm__ ("r9");				\
    register long int r12 __asm__ ("r12");				\
    r5= (long int)arg1;							\
    r6= (long int)arg2;							\
    r7= (long int)arg3;							\
    r8= (long int)arg4;							\
    r9= (long int)arg5;							\
    r12=__NR_##name;							\
    __asm__ __volatile__						\
      ("addk	r0, r0, r0	\n\t"				        \
       "brki	r14, 0x8	\n\t"					\
       "addk	r0, r0, r0	\n\t"					\
       : "=r" (r3)							\
       : "r" (r12), "r" (r5), "r" (r6), "r" (r7), "r" (r8), "r" (r9)	\
       : "r4", "r10", "r11", "r14", "cc", "memory");			\
    __ret = r3;								\
    __syscall_return(type, __ret);					\
  }

#define _syscall6(type, name, type1, arg1, type2, arg2, type3, arg3,	\
		  type4, arg4, type5, arg5, type6, arg6)		\
  type name(type1 arg1, type2 arg2, type3 arg3, type4 arg4, type5 arg5,	\
	    type6 arg6)							\
  {									\
    long int __ret;							\
    register long int r3  __asm__ ("r3");				\
    register long int r5  __asm__ ("r5");				\
    register long int r6  __asm__ ("r6");				\
    register long int r7  __asm__ ("r7");				\
    register long int r8  __asm__ ("r8");				\
    register long int r9  __asm__ ("r9");				\
    register long int r10 __asm__ ("r10");				\
    register long int r12 __asm__ ("r12");				\
    r5= (long int)arg1;							\
    r6= (long int)arg2;							\
    r7= (long int)arg3;							\
    r8= (long int)arg4;							\
    r9= (long int)arg5;							\
    r10= (long int)arg6;						\
    r12=__NR_##name;							\
    __asm__ __volatile__						\
      ("addk	r0, r0, r0	\n\t"				        \
       "brki	r14, 0x8	\n\t"					\
       "addk	r0, r0, r0	\n\t"					\
       : "=r" (r3)							\
       : "r" (r12), "r" (r5), "r" (r6), "r" (r7), "r" (r8), "r" (r9),	\
	 "r" (r10)							\
       : "r4", "r11", "r14", "cc", "memory");				\
    __ret = r3;								\
    __syscall_return(type, __ret);					\
  }
/* the following macros are for testing purposes only */

#define _syscall5_noexregs(type, name, type1, arg1, type2, arg2, type3, arg3, \
			   type4, arg4, type5, arg5)			\
  type name(type1 arg1, type2 arg2, type3 arg3, type4 arg4, type5 arg5)	\
  {									\
    long __ret;								\
    __asm__ __volatile__						\
      ("addk	r5, r0, %2	\n\t"					\
       "addk	r6, r0, %3	\n\t"					\
       "addk	r7, r0, %4	\n\t"					\
       "addk	r8, r0, %5	\n\t"					\
       "addk	r9, r0, %6	\n\t"					\
       "addik	r12, r0, %1	\n\t"					\
       "brki	r14, 0x8	\n\t"					\
       "addk	%0, r3, r0	\n\t"					\
       : "=r" (__ret)							\
       : "i" (__NR_##name),						\
	 "r" ((long)arg1),						\
	 "r" ((long)arg2),						\
	 "r" ((long)arg3),						\
	 "r" ((long)arg4),						\
	 "r" ((long)arg5)						\
       : "r3", "r4", "r5", "r6", "r7", "r8", "r9",			\
       "r10", "r11", "r12", "r14", "cc", "memory");			\
    __syscall_return(type, __ret);					\
  }

#define _syscall6_noexregs(type, name, type1, arg1, type2, arg2, type3, arg3, \
			   type4, arg4, type5, arg5, type6, arg6)	\
  type name(type1 arg1, type2 arg2, type3 arg3, type4 arg4, type5 arg5,	\
	    type6 arg6)							\
  {									\
    long __ret;								\
    __asm__ __volatile__						\
      ("addk	r5, r0, %2	\n\t"					\
       "addk	r6, r0, %3	\n\t"					\
       "addk	r7, r0, %4	\n\t"					\
       "addk	r8, r0, %5	\n\t"					\
       "addk	r9, r0, %6	\n\t"					\
       "addk	r10, r0, %7	\n\t"					\
       "addik	r12, r0, %1	\n\t"					\
       "brki	r14, 0x8	\n\t"					\
       "addk	%0, r3, r0	\n\t"					\
       : "=r" (__ret)							\
       : "i" (__NR_##name),						\
	 "r" ((long)arg1),						\
	 "r" ((long)arg2),						\
	 "r" ((long)arg3),						\
	 "r" ((long)arg4),						\
	 "r" ((long)arg5),						\
	 "r" ((long)arg6)						\
       : "r3", "r4", "r5", "r6", "r7", "r8", "r9",			\
       "r10", "r11","r12", "r14", "cc", "memory");			\
    __syscall_return(type, __ret);					\
  }

#endif /* _BITS_SYSCALLS_H */
