/*
 * Cache control for MicroBlaze cache memories
 *
 * Copyright (C) 2007-2009 Michal Simek <monstr@monstr.eu>
 * Copyright (C) 2007-2009 PetaLogix
 * Copyright (C) 2007 John Williams <john.williams@petalogix.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file COPYING in the main directory of this
 * archive for more details.
 */

#include <asm/cacheflush.h>
#include <linux/cache.h>
#include <asm/cpuinfo.h>
#include <asm/pvr.h>

static inline void __invalidate_flush_icache(unsigned int addr)
{
	__asm__ __volatile__ ("wic	%0, r0;"	\
					: : "r" (addr));
}

static inline void __flush_dcache(unsigned int addr)
{
	__asm__ __volatile__ ("wdc.flush	%0, r0;"	\
					: : "r" (addr));
}

static inline void __invalidate_dcache(unsigned int baseaddr,
						unsigned int offset)
{
	__asm__ __volatile__ ("wdc.clear	%0, %1;"	\
					: : "r" (baseaddr), "r" (offset));
}

static inline void __enable_icache_msr(void)
{
	__asm__ __volatile__ ("	msrset	r0, %0;				\
				nop; "					\
			: : "i" (MSR_ICE) : "memory");
}

static inline void __disable_icache_msr(void)
{
	__asm__ __volatile__ ("	msrclr	r0, %0;				\
				nop; "					\
			: : "i" (MSR_ICE) : "memory");
}

static inline void __enable_dcache_msr(void)
{
		__asm__ __volatile__ ("					\
				msrset	r0, %0;				\
				nop; "					\
				:					\
				: "i" (MSR_DCE)				\
				: "memory");
}

static inline void __disable_dcache_msr(void)
{
		__asm__ __volatile__ ("					\
				msrclr r0, %0;				\
				nop; "					\
				:					\
				: "i" (MSR_DCE)			\
				: "memory");
}

static inline void __enable_icache_nomsr(void)
{
		__asm__ __volatile__ ("					\
				mfs	r12, rmsr;			\
				nop;					\
			ori	r12, r12, %0;			\
				mts	rmsr, r12;			\
				nop; "					\
				:					\
				: "i" (MSR_ICE)				\
				: "memory", "r12");
}

static inline void __disable_icache_nomsr(void)
{
		__asm__ __volatile__ ("					\
				mfs	r12, rmsr;			\
				nop;					\
				andi	r12, r12, ~%0;			\
				mts	rmsr, r12;			\
				nop; "					\
				:					\
				: "i" (MSR_ICE)				\
				: "memory", "r12");
}

static inline void __enable_dcache_nomsr(void)
{
		__asm__ __volatile__ ("					\
				mfs	r12, rmsr;			\
				nop;					\
				ori	r12, r12, %0;			\
				mts	rmsr, r12;			\
				nop; "					\
				:					\
				: "i" (MSR_DCE)			\
				: "memory", "r12");
}

static inline void __disable_dcache_nomsr(void)
{
		__asm__ __volatile__ ("					\
				mfs	r12, rmsr;			\
				nop;					\
				andi	r12, r12, ~%0;			\
				mts	rmsr, r12;			\
				nop; "					\
				:					\
				: "i" (MSR_DCE)			\
				: "memory", "r12");
}

static void __flush_icache_range_msr_irq(unsigned long start, unsigned long end)
{
	unsigned int i;
	unsigned long flags;
	unsigned int align;

	pr_debug("%s: start 0x%x, end 0x%x\n", __func__,
				(unsigned int)start, (unsigned int) end);

	/*
	 * No need to cover entire cache range,
	 * just cover cache footprint
	 */
	end = min(start + cpuinfo.icache_size, end);
	align = ~(cpuinfo.icache_line_length - 1);
	start &= align; /* Make sure we are aligned */
	/* Push end up to the next cache line */
	end = (end & align) + cpuinfo.icache_line_length;

	local_irq_save(flags);
	__disable_icache_msr();

	for (i = start; i < end; i += cpuinfo.icache_line_length)
		__asm__ __volatile__ ("wic	%0, r0;" : : "r" (i));

	__enable_icache_msr();
	local_irq_restore(flags);
}

static void __flush_icache_range_msr_noirq(unsigned long start,
							unsigned long end)
{
	unsigned int i;
	unsigned int align;

	pr_debug("%s: start 0x%x, end 0x%x\n", __func__,
				(unsigned int)start, (unsigned int) end);

	/*
	 * No need to cover entire cache range,
	 * just cover cache footprint
	 */
	end = min(start + cpuinfo.icache_size, end);
	align = ~(cpuinfo.icache_line_length - 1);
	start &= align; /* Make sure we are aligned */
	/* Push end up to the next cache line */
	end = (end & align) + cpuinfo.icache_line_length;

	for (i = start; i < end; i += cpuinfo.icache_line_length)
		__asm__ __volatile__ ("wic	%0, r0;" : : "r" (i));
}

static void __flush_icache_range_nomsr_irq(unsigned long start,
				unsigned long end)
{
	unsigned int i;
	unsigned long flags;
	unsigned int align;

	pr_debug("%s: start 0x%x, end 0x%x\n", __func__,
				(unsigned int)start, (unsigned int) end);

	/*
	 * No need to cover entire cache range,
	 * just cover cache footprint
	 */
	end = min(start + cpuinfo.icache_size, end);
	align = ~(cpuinfo.icache_line_length - 1);
	start &= align; /* Make sure we are aligned */
	/* Push end up to the next cache line */
	end = ((end & align) + cpuinfo.icache_line_length);

		local_irq_save(flags);
	__disable_icache_nomsr();

	for (i = start; i < end; i += cpuinfo.icache_line_length)
		__asm__ __volatile__ ("wic	%0, r0;" : : "r" (i));

	__enable_icache_nomsr();
		local_irq_restore(flags);
}

static void __flush_icache_range_nomsr_noirq(unsigned long start,
				unsigned long end)
{
	unsigned int i;
	unsigned int align;

	pr_debug("%s: start 0x%x, end 0x%x\n", __func__,
				(unsigned int)start, (unsigned int) end);

		/*
		 * No need to cover entire cache range,
		 * just cover cache footprint
		 */
		end = min(start + cpuinfo.icache_size, end);
	align = ~(cpuinfo.icache_line_length - 1);
		start &= align; /* Make sure we are aligned */
		/* Push end up to the next cache line */
	end = ((end & align) + cpuinfo.icache_line_length);

	for (i = start; i < end; i += cpuinfo.icache_line_length)
		__asm__ __volatile__ ("wic	%0, r0;" : : "r" (i));
}

static void __flush_icache_all_msr_irq(void)
{
	int step;
	unsigned int len;
	unsigned long flags;

	pr_debug("%s\n", __func__);

		local_irq_save(flags);
	__disable_icache_msr();

	/* Just loop through cache size and invalidate it */
	len = cpuinfo.icache_size;
	step = -cpuinfo.icache_line_length;
	__asm__ __volatile__ (" 1:	wic	%0, r0;			\
					bgtid	%0, 1b;			\
					addk	%0, %0, %1;		\
					" : : "r" (len), "r" (step));

	__enable_icache_msr();
		local_irq_restore(flags);
}

static void __flush_icache_all_msr_noirq(void)
{
	int step;
	unsigned int len;

	pr_debug("%s\n", __func__);

	/* Just loop through cache size and invalidate it */
	len = cpuinfo.icache_size;
	step = -cpuinfo.icache_line_length;
	__asm__ __volatile__ (" 1:	wic	%0, r0;			\
					bgtid	%0, 1b;			\
					addk	%0, %0, %1;		\
					" : : "r" (len), "r" (step));
}


static void __flush_icache_all_nomsr_irq(void)
{
	int step;
	unsigned int len;
	unsigned long flags;

	pr_debug("%s\n", __func__);

	local_irq_save(flags);
	__disable_icache_nomsr();

	/* Just loop through cache size and invalidate it */
	len = cpuinfo.icache_size;
	step = -cpuinfo.icache_line_length;
	__asm__ __volatile__ (" 1:	wic	%0, r0;			\
					bgtid	%0, 1b;			\
					addk	%0, %0, %1;		\
					" : : "r" (len), "r" (step));

	__enable_icache_nomsr();
	local_irq_restore(flags);
}

static void __flush_icache_all_nomsr_noirq(void)
{
	int step;
	unsigned int len;

	pr_debug("%s\n", __func__);

	/* Just loop through cache size and invalidate it */
	len = cpuinfo.icache_size;
	step = -cpuinfo.icache_line_length;
	__asm__ __volatile__ (" 1:	wic	%0, r0;			\
					bgtid	%0, 1b;			\
					addk	%0, %0, %1;		\
					" : : "r" (len), "r" (step));
}

static void __invalidate_dcache_all_msr_irq(void)
{
	int step;
	unsigned int len;
	unsigned long flags;

	pr_debug("%s\n", __func__);

	local_irq_save(flags);
	__disable_dcache_msr();

	len = cpuinfo.dcache_size;
	step = -cpuinfo.dcache_line_length;
	__asm__ __volatile__ (" 1:	wdc	%0, r0;		\
					bgtid	%0, 1b;			\
					addk	%0, %0, %1;		\
					" : : "r" (len), "r" (step));

	__enable_dcache_msr();
	local_irq_restore(flags);
}

static void __invalidate_dcache_all_nomsr_irq(void)
{
	int step;
	unsigned int len;
	unsigned long flags;

	pr_debug("%s\n", __func__);

	local_irq_save(flags);
	__disable_dcache_nomsr();

	len = cpuinfo.dcache_size;
	step = -cpuinfo.dcache_line_length;
	/* FIXME this is ok because it is used only for WT caches */
	__asm__ __volatile__ (" 1:	wdc	%0, r0;		\
					bgtid	%0, 1b;			\
					addk	%0, %0, %1;		\
					" : : "r" (len), "r" (step));

	__enable_dcache_nomsr();
	local_irq_restore(flags);
}


static void __invalidate_dcache_all_noirq(void)
{
	int step;
	unsigned int len;

	pr_debug("%s\n", __func__);

	len = cpuinfo.dcache_size;
	step = -cpuinfo.dcache_line_length;

	__asm__ __volatile__ (" 1:	wdc	%0, r0;		\
					bgtid	%0, 1b;			\
					addk	%0, %0, %1;		\
					" : : "r" (len), "r" (step));
}


static void __invalidate_dcache_range_noirq(unsigned long start,
						unsigned long end)
{
	unsigned int i;
	unsigned int align;

	pr_debug("%s: start 0x%x, end 0x%x\n", __func__,
				(unsigned int)start, (unsigned int) end);

	/*
	 * No need to cover entire cache range,
	 * just cover cache footprint
	 */
	end = min(start + cpuinfo.dcache_size, end);
	align = ~(cpuinfo.dcache_line_length - 1);
	start &= align; /* Make sure we are aligned */
	/* Push end up to the next cache line */
	end = ((end & align) + cpuinfo.dcache_line_length);

	/* FIXME maybe should be rewrite in asm as is used with wdc.clear */
	for (i = start; i < end; i += cpuinfo.dcache_line_length)
		__asm__ __volatile__ ("wdc.clear	%0, r0;"	\
					: : "r" (i));
}

static void __invalidate_dcache_range_msr_irq(unsigned long start,
							unsigned long end)
{
	unsigned int i;
	unsigned long flags;
	unsigned int align;

	pr_debug("%s: start 0x%x, end 0x%x\n", __func__,
				(unsigned int)start, (unsigned int) end);

	/*
	 * No need to cover entire cache range,
	 * just cover cache footprint
	 */
	end = min(start + cpuinfo.dcache_size, end);
	align = ~(cpuinfo.dcache_line_length - 1);
	start &= align; /* Make sure we are aligned */
	/* Push end up to the next cache line */
	end = ((end & align) + cpuinfo.dcache_line_length);
	local_irq_save(flags);
	__disable_dcache_msr();

	for (i = start; i < end; i += cpuinfo.dcache_line_length)
		__asm__ __volatile__ ("wdc.clear	%0, r0;"	\
					: : "r" (i));


	__enable_dcache_msr();
	local_irq_restore(flags);
}

static void __invalidate_dcache_range_nomsr_irq(unsigned long start,
							unsigned long end)
{
	unsigned int i;
	unsigned long flags;
	unsigned int align;

	pr_debug("%s: start 0x%x, end 0x%x\n", __func__,
				(unsigned int)start, (unsigned int) end);
	/*
	 * No need to cover entire cache range,
	 * just cover cache footprint
	 */
	end = min(start + cpuinfo.dcache_size, end);
	align = ~(cpuinfo.dcache_line_length - 1);
	start &= align; /* Make sure we are aligned */
	/* Push end up to the next cache line */
	end = ((end & align) + cpuinfo.dcache_line_length);
	local_irq_save(flags);
	__disable_dcache_nomsr();

	for (i = start; i < end; i += cpuinfo.dcache_line_length)
		__asm__ __volatile__ ("wdc.clear	%0, r0;"	\
				: : "r" (i));

	__enable_dcache_nomsr();
	local_irq_restore(flags);
}

static void __flush_dcache_all_noirq(void)
{
	int step;
	unsigned int len;

	pr_debug("%s\n", __func__);

	len = cpuinfo.dcache_size;
	step = -cpuinfo.dcache_line_length;

	__asm__ __volatile__ (" 1:	wdc.flush	%0, r0;		\
					bgtid	%0, 1b;			\
					addk	%0, %0, %1;		\
					" : : "r" (len), "r" (step));
}

#if 0
static void __flush_dcache_all_msr_irq(void)
{
	unsigned int i;
	unsigned long flags;

	pr_debug("%s\n", __func__);

		local_irq_save(flags);
	__disable_icache_msr();

		/*
		 * Just loop through cache size and invalidate,
		 * no need to add CACHE_BASE address
		 */
	for (i = 0; i < cpuinfo.dcache_size; i += cpuinfo.dcache_line_length)
			__flush_dcache(i);

	__enable_icache_msr();
		local_irq_restore(flags);
}
#endif

static void __flush_dcache_all_nomsr_irq(void)
{
	int step;
	unsigned int len;
	unsigned long flags;

	pr_debug("%s\n", __func__);

	local_irq_save(flags);
	__disable_icache_nomsr();

	len = cpuinfo.dcache_size;
	step = -cpuinfo.dcache_line_length;

	__asm__ __volatile__ (" 1:	wdc.flush	%0, r0;		\
					bgtid	%0, 1b;			\
					addk	%0, %0, %1;		\
					" : : "r" (len), "r" (step));

	__enable_icache_nomsr();
	local_irq_restore(flags);
}



static void __flush_dcache_range_noirq(unsigned long start, unsigned long end)
{
	unsigned int i;
	unsigned int align;

	pr_debug("%s: start 0x%x, end 0x%x\n", __func__,
				(unsigned int)start, (unsigned int) end);

		/*
		 * No need to cover entire cache range,
		 * just cover cache footprint
		 */
		end = min(start + cpuinfo.dcache_size, end);
	align = ~(cpuinfo.dcache_line_length - 1);
		start &= align; /* Make sure we are aligned */
		/* Push end up to the next cache line */
	end = ((end & align) + cpuinfo.dcache_line_length);

	for (i = start; i < end; i += cpuinfo.dcache_line_length)
		__asm__ __volatile__ ("wdc.flush	%0, r0;"	\
				: : "r" (i));

}

#if 0
static void __flush_dcache_range_msr_irq(unsigned long start, unsigned long end)
{
	unsigned int i;
	unsigned int align;
	unsigned long flags;

	pr_debug("%s: start 0x%x, end 0x%x\n", __func__,
				(unsigned int)start, (unsigned int) end);

	/*
	 * No need to cover entire cache range,
	 * just cover cache footprint
	 */
	end = min(start + cpuinfo.dcache_size, end);
	align = ~(cpuinfo.dcache_line_length - 1);
	start &= align; /* Make sure we are aligned */
	/* Push end up to the next cache line */
	end = ((end & align) + cpuinfo.dcache_line_length);

		local_irq_save(flags);
	__disable_icache_msr();

	for (i = start; i < end; i += cpuinfo.dcache_line_length)
		__flush_dcache(i);

	__enable_icache_msr();
		local_irq_restore(flags);
}
#endif


static void __flush_dcache_range_nomsr_irq(unsigned long start,
						unsigned long end)
{
	unsigned int i;
	unsigned int align;
	unsigned long flags;

	pr_debug("%s: start 0x%x, end 0x%x\n", __func__,
				(unsigned int)start, (unsigned int) end);

	/*
	 * No need to cover entire cache range,
	 * just cover cache footprint
	 */
	end = min(start + cpuinfo.dcache_size, end);
	align = ~(cpuinfo.dcache_line_length - 1);
	start &= align; /* Make sure we are aligned */
	/* Push end up to the next cache line */
	end = ((end & align) + cpuinfo.dcache_line_length);

	local_irq_save(flags);
	__disable_icache_nomsr();

	for (i = start; i < end; i += cpuinfo.dcache_line_length)
		__asm__ __volatile__ ("wdc.flush	%0, r0;"	\
				: : "r" (i));

	__enable_icache_nomsr();
	local_irq_restore(flags);
}

/* struct for wb caches and for wt caches */
struct scache *mbc;

const struct scache wb_msr = {
	.ie = __enable_icache_msr,
	.id = __disable_icache_msr,
	.ifl = __flush_icache_all_msr_noirq,
	.iflr = __flush_icache_range_msr_noirq,
	.iin = __flush_icache_all_msr_noirq,
	.iinr = __flush_icache_range_msr_noirq,
	.de = __enable_dcache_msr,
	.dd = __disable_dcache_msr,
	.dfl = __flush_dcache_all_noirq,
	.dflr = __flush_dcache_range_noirq,
	.din = __invalidate_dcache_all_noirq,
	.dinr = __invalidate_dcache_range_noirq,
};

const struct scache wb_nomsr = {
	.ie = __enable_icache_nomsr,
	.id = __disable_icache_nomsr,
	.ifl = __flush_icache_all_nomsr_noirq,
	.iflr = __flush_icache_range_nomsr_noirq,
	.iin = __flush_icache_all_nomsr_noirq,
	.iinr = __flush_icache_range_nomsr_noirq,
	.de = __enable_dcache_nomsr,
	.dd = __disable_dcache_nomsr,
	.dfl = __flush_dcache_all_noirq,
	.dflr = __flush_dcache_range_noirq,
	.din = __invalidate_dcache_all_noirq,
	.dinr = __invalidate_dcache_range_noirq,
};

const struct scache wt_msr = {
	.ie = __enable_icache_msr,
	.id = __disable_icache_msr,
	.ifl = __flush_icache_all_msr_irq,
	.iflr = __flush_icache_range_msr_irq,
	.iin = __flush_icache_all_msr_irq,
	.iinr = __flush_icache_range_msr_irq,
	.de = __enable_dcache_msr,
	.dd = __disable_dcache_msr,
	.dfl = __invalidate_dcache_all_msr_irq,
	.dflr = __invalidate_dcache_range_msr_irq,
	.din = __invalidate_dcache_all_msr_irq,
	.dinr = __invalidate_dcache_range_msr_irq,
};

const struct scache wt_nomsr = {
	.ie = __enable_icache_nomsr,
	.id = __disable_icache_nomsr,
	.ifl = __flush_icache_all_nomsr_irq,
	.iflr = __flush_icache_range_nomsr_irq,
	.iin = __flush_icache_all_nomsr_irq,
	.iinr = __flush_icache_range_nomsr_irq,
	.de = __enable_dcache_nomsr,
	.dd = __disable_dcache_nomsr,
	.dfl = __flush_dcache_all_nomsr_irq,
	.dflr = __flush_dcache_range_nomsr_irq,
	.din = __invalidate_dcache_all_nomsr_irq,
	.dinr = __invalidate_dcache_range_nomsr_irq,
};

void microblaze_cache_init(void)
{
/* FIXME there is important to cover icache/dcache is OFF, etc */
/* FIXME I will have to check all function and especially implementations */

	if (cpuinfo.use_instr & PVR2_USE_MSR_INSTR) {
		if (cpuinfo.dcache_wb)
			mbc = (struct scache *)&wb_msr;
		else
			mbc = (struct scache *)&wt_msr;
	} else {
		if (cpuinfo.dcache_wb)
			mbc = (struct scache *)&wb_nomsr;
		else
			mbc = (struct scache *)&wt_nomsr;
	}
}
