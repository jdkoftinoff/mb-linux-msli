#ifndef _ASM_MICROBLAZE_CLINKAGE_H
#define _ASM_MICROBLAZE_CLINKAGE_H

#include <linux/linkage.h>

#define C_SYMBOL_NAME(name)	name
#define C_ENTRY(name)		.globl name; .align 4; name
#define C_END(name)

#endif /* _ASM_MICROBLAZE_CLINKAGE_H */
