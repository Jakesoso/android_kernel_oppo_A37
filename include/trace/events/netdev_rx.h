/*
 * If TRACE_SYSTEM is defined, that will be the directory created
 * in the ftrace directory under /sys/kernel/debug/tracing/events/<system>
 *
 * The define_trace.h below will also look for a file name of
 * TRACE_SYSTEM.h where TRACE_SYSTEM is what is defined here.
 * In this case, it would look for sample.h
 *
 * If the header name will be different than the system name
 * (as in this case), then you can override the header name that
 * define_trace.h will look up by defining TRACE_INCLUDE_FILE
 *
 * This file is called trace-events-sample.h but we want the system
 * to be called "sample". Therefore we must define the name of this
 * file:
 *
 * #define TRACE_INCLUDE_FILE trace-events-sample
 *
 * As we do an the bottom of this file.
 *
 * Notice that TRACE_SYSTEM should be defined outside of #if
 * protection, just like TRACE_INCLUDE_FILE.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM netdev_rx
//#define NETDEV_TRACE 1
//#define NETDEV_DL_TRACE 1
//#define NETDEV_UL_TRACE 1

/*
 * Notice that this file is not protected like a normal header.
 * We also must allow for rereading of this file. The
 *
 *  || defined(TRACE_HEADER_MULTI_READ)
 *
 * serves this purpose.
 */
#if !defined(_TRACE_NETDEV_RX_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_NETDEV_RX_H

/*
 * All trace headers should include tracepoint.h, until we finally
 * make it into a standard header.
 */
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/ip.h>
#include <linux/tracepoint.h>
/*
 * The TRACE_EVENT macro is broken up into 5 parts.
 *
 * name: name of the trace point. This is also how to enable the tracepoint.
 *   A function called trace_foo_bar() will be created.
 *
 * proto: the prototype of the function trace_foo_bar()
 *   Here it is trace_foo_bar(char *foo, int bar).
 *
 * args:  must match the arguments in the prototype.
 *    Here it is simply "foo, bar".
 *
 * struct:  This defines the way the data will be stored in the ring buffer.
 *    There are currently two types of elements. __field and __array.
 *    a __field is broken up into (type, name). Where type can be any
 *    type but an array.
 *    For an array. there are three fields. (type, name, size). The
 *    type of elements in the array, the name of the field and the size
 *    of the array.
 *
 *    __array( char, foo, 10) is the same as saying   char foo[10].
 *
 * fast_assign: This is a C like function that is used to store the items
 *    into the ring buffer.
 *
 * printk: This is a way to print out the data in pretty print. This is
 *    useful if the system crashes and you are logging via a serial line,
 *    the data can be printed to the console using this "printk" method.
 *
 * Note, that for both the assign and the printk, __entry is the handler
 * to the data structure in the ring buffer, and is defined by the
 * TP_STRUCT__entry.
 */
TRACE_EVENT(netd_skb_rx,

	TP_PROTO(unsigned long long *net_deta),

	TP_ARGS(net_deta),

	TP_STRUCT__entry(
		__array( unsigned long long,	net_deta, 8 )
	),

	TP_fast_assign(
		memcpy(__entry->net_deta, net_deta, 8*sizeof(unsigned long long));
	),

	TP_printk("	%llu	%llu	%llu	%llu	%llu	%llu	%llu	%llu", 
										   __entry->net_deta[0], __entry->net_deta[1], __entry->net_deta[2],__entry->net_deta[3],
										   __entry->net_deta[4], __entry->net_deta[5],__entry->net_deta[6], __entry->net_deta[7])
);


TRACE_EVENT(rpsd_skb_rx,

	TP_PROTO(unsigned long long *dl_delay),

	TP_ARGS(dl_delay),

	TP_STRUCT__entry(
		__array(	unsigned long long,	dl_delay, 8)
	),

	TP_fast_assign(
		memcpy(__entry->dl_delay, dl_delay, 8*sizeof(unsigned long long));
	),

	TP_printk("	%llu	%llu	%llu	%llu	%llu	%llu	%llu	%llu", 
										__entry->dl_delay[0], __entry->dl_delay[1], __entry->dl_delay[2], __entry->dl_delay[3],
										__entry->dl_delay[4], __entry->dl_delay[5], __entry->dl_delay[6], __entry->dl_delay[7])
);
#endif

/***** NOTICE! The #if protection ends here. *****/


/*
 * There are several ways I could have done this. If I left out the
 * TRACE_INCLUDE_PATH, then it would default to the kernel source
 * include/trace/events directory.
 *
 * I could specify a path from the define_trace.h file back to this
 * file.
 *
 * #define TRACE_INCLUDE_PATH ../../samples/trace_events
 *
 * But the safest and easiest way to simply make it use the directory
 * that the file is in is to add in the Makefile:
 *
 * CFLAGS_trace-events-sample.o := -I$(src)
 *
 * This will make sure the current path is part of the include
 * structure for our file so that define_trace.h can find it.
 *
 * I could have made only the top level directory the include:
 *
 * CFLAGS_trace-events-sample.o := -I$(PWD)
 *
 * And then let the path to this directory be the TRACE_INCLUDE_PATH:
 *
 * #define TRACE_INCLUDE_PATH samples/trace_events
 *
 * But then if something defines "samples" or "trace_events" as a macro
 * then we could risk that being converted too, and give us an unexpected
 * result.
 */

/*
 * TRACE_INCLUDE_FILE is not needed if the filename and TRACE_SYSTEM are equal
 */

#include <trace/define_trace.h>
