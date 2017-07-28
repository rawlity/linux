/*
 * Copyright (C) 2015 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#if !defined(_VC4_TRACE_H_) || defined(TRACE_HEADER_MULTI_READ)
#define _VC4_TRACE_H_

#include <linux/stringify.h>
#include <linux/types.h>
#include <linux/tracepoint.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM vc4
#define TRACE_INCLUDE_FILE vc4_trace

TRACE_EVENT(vc4_wait_for_seqno_begin,
	    TP_PROTO(struct drm_device *dev, uint64_t seqno, uint64_t timeout),
	    TP_ARGS(dev, seqno, timeout),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u64, seqno)
			     __field(u64, timeout)
			     ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			   __entry->seqno = seqno;
			   __entry->timeout = timeout;
			   ),

	    TP_printk("dev=%u, seqno=%llu, timeout=%llu",
		      __entry->dev, __entry->seqno, __entry->timeout)
);

TRACE_EVENT(vc4_wait_for_seqno_end,
	    TP_PROTO(struct drm_device *dev, uint64_t seqno),
	    TP_ARGS(dev, seqno),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u64, seqno)
			     ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			   __entry->seqno = seqno;
			   ),

	    TP_printk("dev=%u, seqno=%llu",
		      __entry->dev, __entry->seqno)
);

TRACE_EVENT(vc4_submit_cl_begin,
	    TP_PROTO(struct drm_device *dev),
	    TP_ARGS(dev),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			   ),

	    TP_printk("dev=%u",
		      __entry->dev)
);

TRACE_EVENT(vc4_submit_cl,
	    TP_PROTO(struct drm_device *dev, uint64_t seqno, int ring),
	    TP_ARGS(dev, seqno, ring),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u64, seqno)
			     __field(bool, ring)
			     ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			   __entry->seqno = seqno;
			   __entry->ring = ring;
			   ),

	    TP_printk("dev=%u, seqno=%llu %s",
		      __entry->dev, __entry->seqno,
		      __entry->ring ? "RCL" : "BCL")
);

TRACE_EVENT(vc4_finish_cl,
	    TP_PROTO(struct drm_device *dev, uint64_t seqno, int ring),
	    TP_ARGS(dev, seqno, ring),

	    TP_STRUCT__entry(
			     __field(u32, dev)
			     __field(u64, seqno)
			     __field(bool, ring)
			     ),

	    TP_fast_assign(
			   __entry->dev = dev->primary->index;
			   __entry->seqno = seqno;
			   __entry->ring = ring;
			   ),

	    TP_printk("dev=%u, seqno=%llu %s",
		      __entry->dev, __entry->seqno,
		      __entry->ring ? "RCL" : "BCL")
);

#endif /* _VC4_TRACE_H_ */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../drivers/gpu/drm/vc4
#include <trace/define_trace.h>
