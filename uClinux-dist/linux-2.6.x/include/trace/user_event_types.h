/* use <trace/user.h> instead */
#ifndef TRACE_EVENT
# error Do not include this file directly.
# error Unless you know what you are doing.
#endif

#undef TRACE_SYSTEM
#define TRACE_SYSTEM user

#define MAX_USER_TRACE_SIZE 127

TRACE_EVENT(user,

	TP_PROTO(const char *message),

	TP_ARGS(message),

	TP_STRUCT__entry(
		__array(char, message, MAX_USER_TRACE_SIZE+1)
	),

	TP_fast_assign(
		memcpy(__entry->message, message, MAX_USER_TRACE_SIZE+1);
	),

	TP_printk("user %s", __entry->message)
);

#undef TRACE_SYSTEM

