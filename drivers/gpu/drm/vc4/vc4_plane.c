/*
 * Copyright (C) 2015 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/**
 * DOC: VC4 plane module
 *
 * Each DRM plane is a layer of pixels being scanned out by the HVS.
 *
 * At atomic modeset check time, we compute the HVS display element
 * state that would be necessary for displaying the plane (giving us a
 * chance to figure out if a plane configuration is invalid), then at
 * atomic flush time the CRTC will ask us to write our element state
 * into the region of the HVS that it has allocated for us.
 */

#include "vc4_drv.h"
#include "vc4_regs.h"
#include "drm_atomic_helper.h"
#include "drm_fb_cma_helper.h"
#include "drm_plane_helper.h"

enum vc4_scaling_mode {
	VC4_SCALING_NONE,
	VC4_SCALING_TPZ,
	VC4_SCALING_PPF,
};

struct vc4_plane_state {
	struct drm_plane_state base;
	/* System memory copy of the display list for this element, computed
	 * at atomic_check time.
	 */
	u32 *dlist;
	u32 dlist_size; /* Number of dwords allocated for the display list */
	u32 dlist_count; /* Number of used dwords in the display list. */

	/* Offset in the dlist to pointer word 0. */
	u32 pw0_offset;

	/* Offset where the plane's dlist was last stored in the
	 * hardware at vc4_crtc_atomic_flush() time.
	 */
	u32 __iomem *hw_dlist;

	/* Clipped coordinates of the plane on the display. */
	int crtc_x, crtc_y, crtc_w, crtc_h;
	/* Clipped area being scanned from in the FB. */
	u32 src_x, src_y, src_w, src_h;

	enum vc4_scaling_mode x_scaling, y_scaling;
	bool is_unity;

	/* Offset to start scanning out from the start of the plane's
	 * BO.
	 */
	u32 offset;

	/* Our allocation in LBM for temporary storage during scaling. */
	struct drm_mm_node lbm;
};

static inline struct vc4_plane_state *
to_vc4_plane_state(struct drm_plane_state *state)
{
	return (struct vc4_plane_state *)state;
}

static const struct hvs_format {
	u32 drm; /* DRM_FORMAT_* */
	u32 hvs; /* HVS_FORMAT_* */
	u32 pixel_order;
	bool has_alpha;
} hvs_formats[] = {
	{
		.drm = DRM_FORMAT_XRGB8888, .hvs = HVS_PIXEL_FORMAT_RGBA8888,
		.pixel_order = HVS_PIXEL_ORDER_ABGR, .has_alpha = false,
	},
	{
		.drm = DRM_FORMAT_ARGB8888, .hvs = HVS_PIXEL_FORMAT_RGBA8888,
		.pixel_order = HVS_PIXEL_ORDER_ABGR, .has_alpha = true,
	},
	{
		.drm = DRM_FORMAT_RGB565, .hvs = HVS_PIXEL_FORMAT_RGB565,
		.pixel_order = HVS_PIXEL_ORDER_XRGB, .has_alpha = false,
	},
	{
		.drm = DRM_FORMAT_BGR565, .hvs = HVS_PIXEL_FORMAT_RGB565,
		.pixel_order = HVS_PIXEL_ORDER_XBGR, .has_alpha = false,
	},
	{
		.drm = DRM_FORMAT_ARGB1555, .hvs = HVS_PIXEL_FORMAT_RGBA5551,
		.pixel_order = HVS_PIXEL_ORDER_ABGR, .has_alpha = true,
	},
	{
		.drm = DRM_FORMAT_XRGB1555, .hvs = HVS_PIXEL_FORMAT_RGBA5551,
		.pixel_order = HVS_PIXEL_ORDER_ABGR, .has_alpha = false,
	},
};

static const struct hvs_format *vc4_get_hvs_format(u32 drm_format)
{
	unsigned i;

	for (i = 0; i < ARRAY_SIZE(hvs_formats); i++) {
		if (hvs_formats[i].drm == drm_format)
			return &hvs_formats[i];
	}

	return NULL;
}

static enum vc4_scaling_mode vc4_get_scaling_mode(u32 src, u32 dst)
{
	if (dst > src)
		return VC4_SCALING_PPF;
	else if (dst < src)
		return VC4_SCALING_TPZ;
	else
		return VC4_SCALING_NONE;
}

static bool plane_enabled(struct drm_plane_state *state)
{
	return state->fb && state->crtc;
}

static struct drm_plane_state *vc4_plane_duplicate_state(struct drm_plane *plane)
{
	struct vc4_plane_state *vc4_state;

	if (WARN_ON(!plane->state))
		return NULL;

	vc4_state = kmemdup(plane->state, sizeof(*vc4_state), GFP_KERNEL);
	if (!vc4_state)
		return NULL;

	memset(&vc4_state->lbm, 0, sizeof(vc4_state->lbm));

	__drm_atomic_helper_plane_duplicate_state(plane, &vc4_state->base);

	if (vc4_state->dlist) {
		vc4_state->dlist = kmemdup(vc4_state->dlist,
					   vc4_state->dlist_count * 4,
					   GFP_KERNEL);
		if (!vc4_state->dlist) {
			kfree(vc4_state);
			return NULL;
		}
		vc4_state->dlist_size = vc4_state->dlist_count;
	}

	return &vc4_state->base;
}

static void vc4_plane_destroy_state(struct drm_plane *plane,
				    struct drm_plane_state *state)
{
	struct vc4_dev *vc4 = to_vc4_dev(plane->dev);
	struct vc4_plane_state *vc4_state = to_vc4_plane_state(state);

	if (vc4_state->lbm.allocated) {
		unsigned long irqflags;

		spin_lock_irqsave(&vc4->hvs->mm_lock, irqflags);
		drm_mm_remove_node(&vc4_state->lbm);
		spin_unlock_irqrestore(&vc4->hvs->mm_lock, irqflags);
	}

	kfree(vc4_state->dlist);
	__drm_atomic_helper_plane_destroy_state(plane, &vc4_state->base);
	kfree(state);
}

/* Called during init to allocate the plane's atomic state. */
static void vc4_plane_reset(struct drm_plane *plane)
{
	struct vc4_plane_state *vc4_state;

	WARN_ON(plane->state);

	vc4_state = kzalloc(sizeof(*vc4_state), GFP_KERNEL);
	if (!vc4_state)
		return;

	plane->state = &vc4_state->base;
	vc4_state->base.plane = plane;
}

static void vc4_dlist_write(struct vc4_plane_state *vc4_state, u32 val)
{
	if (vc4_state->dlist_count == vc4_state->dlist_size) {
		u32 new_size = max(4u, vc4_state->dlist_count * 2);
		u32 *new_dlist = kmalloc(new_size * 4, GFP_KERNEL);

		if (!new_dlist)
			return;
		memcpy(new_dlist, vc4_state->dlist, vc4_state->dlist_count * 4);

		kfree(vc4_state->dlist);
		vc4_state->dlist = new_dlist;
		vc4_state->dlist_size = new_size;
	}

	vc4_state->dlist[vc4_state->dlist_count++] = val;
}

/* Returns the scl0/scl1 field based on whether the dimensions need to
 * be up/down/non-scaled.
 *
 * This is a replication of a table from the spec.
 */
static u32 vc4_get_scl_field(struct drm_plane_state *state)
{
	struct vc4_plane_state *vc4_state = to_vc4_plane_state(state);

	switch (vc4_state->x_scaling << 2 | vc4_state->y_scaling) {
	case VC4_SCALING_PPF << 2 | VC4_SCALING_PPF:
		return SCALER_CTL0_SCL_H_PPF_V_PPF;
	case VC4_SCALING_TPZ << 2 | VC4_SCALING_PPF:
		return SCALER_CTL0_SCL_H_TPZ_V_PPF;
	case VC4_SCALING_PPF << 2 | VC4_SCALING_TPZ:
		return SCALER_CTL0_SCL_H_PPF_V_TPZ;
	case VC4_SCALING_TPZ << 2 | VC4_SCALING_TPZ:
		return SCALER_CTL0_SCL_H_TPZ_V_TPZ;
	case VC4_SCALING_PPF << 2 | VC4_SCALING_NONE:
		return SCALER_CTL0_SCL_H_PPF_V_NONE;
	case VC4_SCALING_NONE << 2 | VC4_SCALING_PPF:
		return SCALER_CTL0_SCL_H_NONE_V_PPF;
	case VC4_SCALING_NONE << 2 | VC4_SCALING_TPZ:
		return SCALER_CTL0_SCL_H_NONE_V_TPZ;
	case VC4_SCALING_TPZ << 2 | VC4_SCALING_NONE:
		return SCALER_CTL0_SCL_H_TPZ_V_NONE;
	default:
	case VC4_SCALING_NONE << 2 | VC4_SCALING_NONE:
		/* The unity case is independently handled by
		 * SCALER_CTL0_UNITY.
		 */
		return 0;
	}
}

static int vc4_plane_setup_clipping_and_scaling(struct drm_plane_state *state)
{
	struct vc4_plane_state *vc4_state = to_vc4_plane_state(state);
	struct drm_framebuffer *fb = state->fb;
	u32 subpixel_src_mask = (1 << 16) - 1;

	vc4_state->offset = fb->offsets[0];

	/* We don't support subpixel source positioning for scaling. */
	if ((state->src_x & subpixel_src_mask) ||
	    (state->src_y & subpixel_src_mask) ||
	    (state->src_w & subpixel_src_mask) ||
	    (state->src_h & subpixel_src_mask)) {
		return -EINVAL;
	}

	vc4_state->src_x = state->src_x >> 16;
	vc4_state->src_y = state->src_y >> 16;
	vc4_state->src_w = state->src_w >> 16;
	vc4_state->src_h = state->src_h >> 16;

	vc4_state->crtc_x = state->crtc_x;
	vc4_state->crtc_y = state->crtc_y;
	vc4_state->crtc_w = state->crtc_w;
	vc4_state->crtc_h = state->crtc_h;

	vc4_state->x_scaling = vc4_get_scaling_mode(vc4_state->src_w,
						    vc4_state->crtc_w);
	vc4_state->y_scaling = vc4_get_scaling_mode(vc4_state->src_h,
						    vc4_state->crtc_h);
	vc4_state->is_unity = (vc4_state->x_scaling == VC4_SCALING_NONE &&
			       vc4_state->y_scaling == VC4_SCALING_NONE);

	/* Clamp the on-screen start x/y to 0.  The hardware doesn't
	 * support negative y, and negative x wastes bandwidth.
	 */
	if (vc4_state->crtc_x < 0) {
		vc4_state->offset += (drm_format_plane_cpp(fb->pixel_format,
							   0) *
				      -vc4_state->crtc_x);
		vc4_state->src_w += vc4_state->crtc_x;
		vc4_state->crtc_x = 0;
	}

	if (vc4_state->crtc_y < 0) {
		vc4_state->offset += fb->pitches[0] * -vc4_state->crtc_y;
		vc4_state->src_h += vc4_state->crtc_y;
		vc4_state->crtc_y = 0;
	}

	return 0;
}

static void vc4_write_tpz(struct vc4_plane_state *vc4_state, u32 src, u32 dst)
{
	u32 scale, recip;

	scale = (1 << 16) * src;
	do_div(scale, dst);

	/* The specs note that while the reciprocal would be defined
	 * as (1<<32)/scale, ~0 is close enough.
	 */
	recip = ~0;
	do_div(recip, scale);

	vc4_dlist_write(vc4_state,
			VC4_SET_FIELD(scale, SCALER_TPZ0_SCALE) |
			VC4_SET_FIELD(0, SCALER_TPZ0_IPHASE));
	vc4_dlist_write(vc4_state,
			VC4_SET_FIELD(recip, SCALER_TPZ1_RECIP));
}

static void vc4_write_ppf(struct vc4_plane_state *vc4_state, u32 src, u32 dst)
{
	u32 scale = (1 << 16) * src;

	do_div(scale, dst);

	vc4_dlist_write(vc4_state,
			SCALER_PPF_AGC |
			VC4_SET_FIELD(scale, SCALER_PPF_SCALE) |
			VC4_SET_FIELD(0, SCALER_PPF_IPHASE));
}

static u32 vc4_lbm_size(struct drm_plane_state *state)
{
	struct vc4_plane_state *vc4_state = to_vc4_plane_state(state);
	u32 dwords = 0;

	if (vc4_state->y_scaling != VC4_SCALING_NONE) {
		u32 pix_per_line;

		if (vc4_state->x_scaling == VC4_SCALING_NONE ||
		    (vc4_state->x_scaling == VC4_SCALING_PPF &&
		     vc4_state->crtc_w > vc4_state->src_w)) {
			pix_per_line = vc4_state->src_w;
		} else {
			pix_per_line = vc4_state->crtc_w;
		}

		/* NOTE: The non-TPZ mode may be smaller for src data
		 * without alpha.
		 */
		if (vc4_state->y_scaling == VC4_SCALING_TPZ)
			dwords = pix_per_line * 8;
		else
			dwords = pix_per_line * 16;
	}

	return dwords * 4;
}

static void vc4_write_scaling_parameters(struct drm_plane_state *state)
{
	struct vc4_plane_state *vc4_state = to_vc4_plane_state(state);

	/* Ch0 H-PPF Word 0: Scaling Parameters */
	if (vc4_state->x_scaling == VC4_SCALING_PPF) {
		vc4_write_ppf(vc4_state,
			      vc4_state->src_w, vc4_state->crtc_w);
	}

	/* Ch0 V-PPF Words 0-1: Scaling Parameters, Context */
	if (vc4_state->y_scaling == VC4_SCALING_PPF) {
		vc4_write_ppf(vc4_state,
			      vc4_state->src_h, vc4_state->crtc_h);
		vc4_dlist_write(vc4_state, 0xc0c0c0c0);
	}

	/* Ch0 H-TPZ Words 0-1: Scaling Parameters, Recip */
	if (vc4_state->x_scaling == VC4_SCALING_TPZ) {
		vc4_write_tpz(vc4_state,
			      vc4_state->src_w, vc4_state->crtc_w);
	}

	/* Ch0 V-TPZ Words 0-2: Scaling Parameters, Recip, Context */
	if (vc4_state->y_scaling == VC4_SCALING_TPZ) {
		vc4_write_tpz(vc4_state,
			      vc4_state->src_h, vc4_state->crtc_h);
		vc4_dlist_write(vc4_state, 0xc0c0c0c0);
	}
}

/* Writes out a full display list for an active plane to the plane's
 * private dlist state.
 */
static int vc4_plane_mode_set(struct drm_plane *plane,
			      struct drm_plane_state *state)
{
	struct vc4_dev *vc4 = to_vc4_dev(plane->dev);
	struct vc4_plane_state *vc4_state = to_vc4_plane_state(state);
	struct drm_framebuffer *fb = state->fb;
	struct drm_gem_cma_object *bo = drm_fb_cma_get_gem_obj(fb, 0);
	u32 ctl0_offset = vc4_state->dlist_count;
	const struct hvs_format *format = vc4_get_hvs_format(fb->pixel_format);
	u32 scl;
	u32 lbm_size;
	unsigned long irqflags;
	int ret;

	ret = vc4_plane_setup_clipping_and_scaling(state);
	if (ret)
		return ret;

	/* Allocate the LBM memory that the HVS will use for temporary
	 * storage due to our scaling/format conversion.
	 */
	lbm_size = vc4_lbm_size(state);
	if (lbm_size) {
		spin_lock_irqsave(&vc4->hvs->mm_lock, irqflags);
		ret = drm_mm_insert_node(&vc4->hvs->lbm_mm, &vc4_state->lbm,
					 lbm_size * 4, 32, 0);
		spin_unlock_irqrestore(&vc4->hvs->mm_lock, irqflags);
	}

	scl = vc4_get_scl_field(state);

	/* Control word */
	vc4_dlist_write(vc4_state,
			SCALER_CTL0_VALID |
			(format->pixel_order << SCALER_CTL0_ORDER_SHIFT) |
			(format->hvs << SCALER_CTL0_PIXEL_FORMAT_SHIFT) |
			(vc4_state->is_unity ? SCALER_CTL0_UNITY : 0) |
			VC4_SET_FIELD(scl, SCALER_CTL0_SCL0) |
			VC4_SET_FIELD(scl, SCALER_CTL0_SCL1));

	/* Position Word 0: Image Positions and Alpha Value */
	vc4_dlist_write(vc4_state,
			VC4_SET_FIELD(0xff, SCALER_POS0_FIXED_ALPHA) |
			VC4_SET_FIELD(vc4_state->crtc_x, SCALER_POS0_START_X) |
			VC4_SET_FIELD(vc4_state->crtc_y, SCALER_POS0_START_Y));

	/* Position Word 1: Scaled Image Dimensions. */
	if (!vc4_state->is_unity) {
		vc4_dlist_write(vc4_state,
				VC4_SET_FIELD(vc4_state->crtc_w,
					      SCALER_POS1_SCL_WIDTH) |
				VC4_SET_FIELD(vc4_state->crtc_h,
					      SCALER_POS1_SCL_HEIGHT));
	}

	/* Position Word 2: Source Image Size, Alpha Mode */
	vc4_dlist_write(vc4_state,
			VC4_SET_FIELD(format->has_alpha ?
				      SCALER_POS2_ALPHA_MODE_PIPELINE :
				      SCALER_POS2_ALPHA_MODE_FIXED,
				      SCALER_POS2_ALPHA_MODE) |
			VC4_SET_FIELD(vc4_state->src_w, SCALER_POS2_WIDTH) |
			VC4_SET_FIELD(vc4_state->src_h, SCALER_POS2_HEIGHT));

	/* Position Word 3: Context.  Written by the HVS. */
	vc4_dlist_write(vc4_state, 0xc0c0c0c0);

	vc4_state->pw0_offset = vc4_state->dlist_count;

	/* Pointer Word 0: RGB / Y Pointer */
	vc4_dlist_write(vc4_state, bo->paddr + vc4_state->offset);

	/* Pointer Context Word 0: Written by the HVS */
	vc4_dlist_write(vc4_state, 0xc0c0c0c0);

	/* Pitch word 0: Pointer 0 Pitch */
	vc4_dlist_write(vc4_state,
			VC4_SET_FIELD(fb->pitches[0], SCALER_SRC_PITCH));

	if (!vc4_state->is_unity) {
		/* LBM Base Address. */
		if (vc4_state->y_scaling != VC4_SCALING_NONE)
			vc4_dlist_write(vc4_state, vc4_state->lbm.start);

		vc4_write_scaling_parameters(state);

		/* If any PPF setup was done, then all the kernel
		 * pointers get uploaded.
		 */
		if (vc4_state->x_scaling == VC4_SCALING_PPF ||
		    vc4_state->y_scaling == VC4_SCALING_PPF) {
			u32 kernel = VC4_SET_FIELD(vc4->hvs->mitchell_netravali_filter.start,
						   SCALER_PPF_KERNEL_OFFSET);

			/* HPPF plane 0 */
			vc4_dlist_write(vc4_state, kernel);
			/* VPPF plane 0 */
			vc4_dlist_write(vc4_state, kernel);
			/* HPPF plane 1 */
			vc4_dlist_write(vc4_state, kernel);
			/* VPPF plane 1 */
			vc4_dlist_write(vc4_state, kernel);
		}
	}

	vc4_state->dlist[ctl0_offset] |=
		VC4_SET_FIELD(vc4_state->dlist_count, SCALER_CTL0_SIZE);

	return 0;
}

/* If a modeset involves changing the setup of a plane, the atomic
 * infrastructure will call this to validate a proposed plane setup.
 * However, if a plane isn't getting updated, this (and the
 * corresponding vc4_plane_atomic_update) won't get called.  Thus, we
 * compute the dlist here and have all active plane dlists get updated
 * in the CRTC's flush.
 */
static int vc4_plane_atomic_check(struct drm_plane *plane,
				  struct drm_plane_state *state)
{
	struct vc4_plane_state *vc4_state = to_vc4_plane_state(state);

	vc4_state->dlist_count = 0;

	if (plane_enabled(state))
		return vc4_plane_mode_set(plane, state);
	else
		return 0;
}

static void vc4_plane_atomic_update(struct drm_plane *plane,
				    struct drm_plane_state *old_state)
{
	/* No contents here.  Since we don't know where in the CRTC's
	 * dlist we should be stored, our dlist is uploaded to the
	 * hardware with vc4_plane_write_dlist() at CRTC atomic_flush
	 * time.
	 */
}

u32 vc4_plane_write_dlist(struct drm_plane *plane, u32 __iomem *dlist)
{
	struct vc4_plane_state *vc4_state = to_vc4_plane_state(plane->state);
	int i;

	vc4_state->hw_dlist = dlist;

	/* Can't memcpy_toio() because it needs to be 32-bit writes. */
	for (i = 0; i < vc4_state->dlist_count; i++)
		writel(vc4_state->dlist[i], &dlist[i]);

	return vc4_state->dlist_count;
}

u32 vc4_plane_dlist_size(struct drm_plane_state *state)
{
	struct vc4_plane_state *vc4_state = to_vc4_plane_state(state);

	return vc4_state->dlist_count;
}

/* Updates the plane to immediately (well, once the FIFO needs
 * refilling) scan out from at a new framebuffer.
 */
void vc4_plane_async_set_fb(struct drm_plane *plane, struct drm_framebuffer *fb)
{
	struct vc4_plane_state *vc4_state = to_vc4_plane_state(plane->state);
	struct drm_gem_cma_object *bo = drm_fb_cma_get_gem_obj(fb, 0);
	uint32_t addr;

	/* We're skipping the address adjustment for negative origin,
	 * because this is only called on the primary plane.
	 */
	WARN_ON_ONCE(plane->state->crtc_x < 0 || plane->state->crtc_y < 0);
	addr = bo->paddr + fb->offsets[0];

	/* Write the new address into the hardware immediately.  The
	 * scanout will start from this address as soon as the FIFO
	 * needs to refill with pixels.
	 */
	writel(addr, &vc4_state->hw_dlist[vc4_state->pw0_offset]);

	/* Also update the CPU-side dlist copy, so that any later
	 * atomic updates that don't do a new modeset on our plane
	 * also use our updated address.
	 */
	vc4_state->dlist[vc4_state->pw0_offset] = addr;
}

static const struct drm_plane_helper_funcs vc4_plane_helper_funcs = {
	.prepare_fb = NULL,
	.cleanup_fb = NULL,
	.atomic_check = vc4_plane_atomic_check,
	.atomic_update = vc4_plane_atomic_update,
};

static void vc4_plane_destroy(struct drm_plane *plane)
{
	drm_plane_helper_disable(plane);
	drm_plane_cleanup(plane);
}

static const struct drm_plane_funcs vc4_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = vc4_plane_destroy,
	.set_property = NULL,
	.reset = vc4_plane_reset,
	.atomic_duplicate_state = vc4_plane_duplicate_state,
	.atomic_destroy_state = vc4_plane_destroy_state,
};

struct drm_plane *vc4_plane_init(struct drm_device *dev,
				 enum drm_plane_type type)
{
	struct drm_plane *plane = NULL;
	struct vc4_plane *vc4_plane;
	u32 formats[ARRAY_SIZE(hvs_formats)];
	int ret = 0;
	unsigned i;

	vc4_plane = devm_kzalloc(dev->dev, sizeof(*vc4_plane),
				 GFP_KERNEL);
	if (!vc4_plane) {
		ret = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < ARRAY_SIZE(hvs_formats); i++)
		formats[i] = hvs_formats[i].drm;
	plane = &vc4_plane->base;
	ret = drm_universal_plane_init(dev, plane, 0xff,
				       &vc4_plane_funcs,
				       formats, ARRAY_SIZE(formats),
				       type);

	drm_plane_helper_add(plane, &vc4_plane_helper_funcs);

	return plane;
fail:
	if (plane)
		vc4_plane_destroy(plane);

	return ERR_PTR(ret);
}
