/*
 *
 * (C) COPYRIGHT 2012-2013 ARM Limited. All rights reserved.
 *
 *
 * Parts of this file were based on sources as follows:
 *
 * Copyright (c) 2006-2008 Intel Corporation
 * Copyright (c) 2007 Dave Airlie <airlied@linux.ie>
 * Copyright (C) 2011 Texas Instruments
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms of
 * such GNU licence.
 *
 */

#ifndef _PL111_DRM_H_
#define _PL111_DRM_H_

#include <drm/drm_gem.h>

#define DRIVER_AUTHOR    "ARM Ltd."
#define DRIVER_NAME      "pl111_drm"
#define DRIVER_DESC      "DRM module for PL111"
#define DRIVER_LICENCE   "GPL"
#define DRIVER_ALIAS     "platform:pl111_drm"
#define DRIVER_DATE      "20101111"
#define DRIVER_VERSION   "0.2"
#define DRIVER_MAJOR      2
#define DRIVER_MINOR      1
#define DRIVER_PATCHLEVEL 1

/*
 * Number of flips allowed in flight at any one time. Any more flips requested
 * beyond this value will cause the caller to block until earlier flips have
 * completed.
 *
 * For performance reasons, this must be greater than the number of buffers
 * used in the rendering pipeline. Note that the rendering pipeline can contain
 * different types of buffer, e.g.:
 * - 2 final framebuffers
 * - >2 geometry buffers for GPU use-cases
 * - >2 vertex buffers for GPU use-cases
 *
 * For example, a system using 5 geometry buffers could have 5 flips in flight,
 * and so NR_FLIPS_IN_FLIGHT_THRESHOLD must be 5 or greater.
 *
 * Whilst there may be more intermediate buffers (such as vertex/geometry) than
 * final framebuffers, KDS is used to ensure that GPU rendering waits for the
 * next off-screen buffer, so it doesn't overwrite an on-screen buffer and
 * produce tearing.
 */

/*
 * Here, we choose a conservative value. A lower value is most likely
 * suitable for GPU use-cases.
 */
#define NR_FLIPS_IN_FLIGHT_THRESHOLD 16

#define CLCD_IRQ_NEXTBASE_UPDATE (1u<<2)

struct pl111_drm_crtc {
	struct drm_crtc crtc;
	int crtc_index;

	spinlock_t current_displaying_lock;
	spinlock_t base_update_lock;
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct kds_resource_set *old_kds_res_set;
#endif
	struct drm_framebuffer *displaying_fb;

	/*
	 * The resource that caused a base address update. Only one can be
	 * pending, hence it's != NULL if there's a pending update
	 */
	struct pl111_drm_flip_resource *current_update_res;
	/* Queue of things waiting to update the base address */
	struct list_head update_queue;

	struct workqueue_struct *vsync_wq;

	void (*show_framebuffer_cb)(struct pl111_drm_flip_resource *flip_res,
				struct drm_framebuffer *fb);
};

struct pl111_drm_connector {
	struct drm_connector connector;
};

struct pl111_drm_dev_private {
	struct drm_device *drm;
	struct pl111_drm_crtc *pl111_crtc;

	struct drm_encoder encoder;
	struct drm_plane primary;
	struct drm_fbdev_cma *fbdev;

	struct amba_device *amba_dev;
	unsigned long mmio_start;
	__u32 mmio_len;
	void *regs;
	struct clk *clk;
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct kds_callback kds_cb;
	struct kds_callback kds_obtain_current_cb;
#endif
	/*
	 * Number of flips that were started in show_framebuffer_on_crtc(),
	 * but haven't completed yet - because we do deferred flipping
	 */
	atomic_t nr_flips_in_flight;
	wait_queue_head_t wait_for_flips;

	/*
	 * Used to prevent race between pl111_dma_buf_release and
	 * drm_gem_prime_handle_to_fd
	 */
	struct mutex export_dma_buf_lock;

	uint32_t number_crtcs;
};

#define to_pl111_crtc(x) container_of(x, struct pl111_drm_crtc, crtc)

#define PL111_CONNECTOR_FROM_CONNECTOR(x) \
	container_of(x, struct pl111_drm_connector, connector)

#include "pl111_drm_funcs.h"

#endif /* _PL111_DRM_H_ */
