/*
 * (C) COPYRIGHT 2012-2013 ARM Limited. All rights reserved.
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

/**
 * pl111_drm_funcs.h
 * Function prototypes for PL111 DRM
 */

#ifndef PL111_DRM_FUNCS_H_
#define PL111_DRM_FUNCS_H_

/* Platform Initialisation */
int pl111_drm_init(struct platform_device *dev);
void pl111_drm_exit(struct platform_device *dev);

/* CRTC Functions */
struct pl111_drm_crtc *pl111_crtc_create(struct drm_device *dev);
struct pl111_drm_crtc *pl111_crtc_dummy_create(struct drm_device *dev);
void pl111_crtc_destroy(struct drm_crtc *crtc);

/* Common IRQ handler */
void pl111_common_irq(struct pl111_drm_crtc *pl111_crtc);

int pl111_primary_plane_init(struct drm_device *dev);

/* Connector Functions */
struct pl111_drm_connector *pl111_connector_create(struct drm_device *dev);
void pl111_connector_destroy(struct drm_connector *connector);
struct pl111_drm_connector *pl111_connector_dummy_create(struct drm_device
								*dev);

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

#endif /* PL111_DRM_FUNCS_H_ */
