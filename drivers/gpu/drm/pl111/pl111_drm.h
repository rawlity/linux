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

struct pl111_drm_connector {
	struct drm_connector connector;
};

struct pl111_drm_dev_private {
	struct drm_device *drm;

	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_plane primary;
	struct drm_fbdev_cma *fbdev;

	struct amba_device *amba_dev;
	void *regs;
	struct clk *clk;
};

#define to_pl111_crtc(x) container_of(x, struct pl111_drm_crtc, crtc)

#define PL111_CONNECTOR_FROM_CONNECTOR(x) \
	container_of(x, struct pl111_drm_connector, connector)

/* Platform Initialisation */
int pl111_drm_init(struct platform_device *dev);
void pl111_drm_exit(struct platform_device *dev);

/* CRTC Functions */
int pl111_crtc_create(struct drm_device *dev);
void pl111_crtc_destroy(struct drm_crtc *crtc);
irqreturn_t pl111_irq(int irq, void *data);

int pl111_primary_plane_init(struct drm_device *dev);

/* Connector Functions */
struct pl111_drm_connector *pl111_connector_create(struct drm_device *dev);
void pl111_connector_destroy(struct drm_connector *connector);

/* Encoder Functions */
int pl111_encoder_init(struct drm_device *dev);

/* Frame Buffer Functions */
struct drm_framebuffer *pl111_fb_create(struct drm_device *dev,
					struct drm_file *file_priv,
					const struct drm_mode_fb_cmd2 *mode_cmd);

/* GEM Functions */
int pl111_dumb_create(struct drm_file *file_priv,
		      struct drm_device *dev,
		      struct drm_mode_create_dumb *args);

/* Pl111 Functions */
int pl111_device_init(struct drm_device *dev);
void pl111_device_fini(struct drm_device *dev);

#endif /* _PL111_DRM_H_ */
