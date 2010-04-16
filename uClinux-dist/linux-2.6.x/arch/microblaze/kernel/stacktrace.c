#include <linux/sched.h>
#include <linux/stacktrace.h>
#include <linux/thread_info.h>
#include <linux/module.h>
#include <asm/ptrace.h>
//#include <asm/stacktrace.h>

void save_stack_trace(struct stack_trace *trace)
{
	//printk("%s\n", __func__);
	//__save_stack_trace(current_thread_info(), trace, false);
}
EXPORT_SYMBOL_GPL(save_stack_trace);
