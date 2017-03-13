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
 * pl111_drm_crtc.c
 * Implementation of the CRTC functions for PL111 DRM
 */
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/version.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>
#include <linux/module.h>

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>

#include "pl111_drm.h"

static int pl111_crtc_num;

static void pl111_convert_drm_mode_to_timing(const struct drm_display_mode *mode,
					     struct clcd_regs *timing)
{
	unsigned int ppl, hsw, hfp, hbp;
	unsigned int lpp, vsw, vfp, vbp;
	unsigned int cpl;

	memset(timing, 0, sizeof(struct clcd_regs));

	ppl = (mode->hdisplay / 16) - 1;
	hsw = mode->hsync_end - mode->hsync_start - 1;
	hfp = mode->hsync_start - mode->hdisplay - 1;
	hbp = mode->htotal - mode->hsync_end - 1;

	lpp = mode->vdisplay - 1;
	vsw = mode->vsync_end - mode->vsync_start - 1;
	vfp = mode->vsync_start - mode->vdisplay;
	vbp = mode->vtotal - mode->vsync_end;

	cpl = mode->hdisplay - 1;

	timing->tim0 = (ppl << 2) | (hsw << 8) | (hfp << 16) | (hbp << 24);
	timing->tim1 = lpp | (vsw << 10) | (vfp << 16) | (vbp << 24);
	timing->tim2 = TIM2_IVS | TIM2_IHS | TIM2_IPC | TIM2_BCD | (cpl << 16);
	timing->tim3 = 0;

	timing->pixclock = mode->clock * 1000;
}

void pl111_common_irq(struct pl111_drm_crtc *pl111_crtc)
{
	drm_handle_vblank(pl111_crtc->crtc.dev, pl111_crtc->crtc_index);
}

void pl111_crtc_helper_mode_set_nofb(struct drm_crtc *crtc)
{
	struct clcd_regs timing;

	pl111_convert_drm_mode_to_timing(&crtc->state->mode, &timing);

	clk_set_rate(priv.clk, timing.pixclock);

	writel(timing.tim0, priv.regs + CLCD_TIM0);
	writel(timing.tim1, priv.regs + CLCD_TIM1);
	writel(timing.tim2, priv.regs + CLCD_TIM2);
	writel(timing.tim3, priv.regs + CLCD_TIM3);
}

void pl111_crtc_helper_prepare(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("DRM %s on crtc=%p\n", __func__, crtc);
}

void pl111_crtc_helper_commit(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("DRM %s on crtc=%p\n", __func__, crtc);
}

bool pl111_crtc_helper_mode_fixup(struct drm_crtc *crtc,
				  const struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	DRM_DEBUG_KMS("DRM %s on crtc=%p\n", __func__, crtc);

#ifdef CONFIG_ARCH_VEXPRESS
	/*
	 * 1024x768 with more than 16 bits per pixel does not work correctly
	 * on Versatile Express
	 */
	if (mode->hdisplay == 1024 && mode->vdisplay == 768 &&
			crtc->fb->bits_per_pixel > 16) {
		return false;
	}
#endif

	return true;
}

void pl111_crtc_helper_enable(struct drm_crtc *crtc)
{
	__u32 cntl;
	struct clcd_board *board;

	DRM_DEBUG_KMS("DRM %s on crtc=%p\n", __func__, crtc);

	clk_prepare_enable(priv.clk);

	/* Enable and Power Up */
	cntl = CNTL_LCDEN | CNTL_LCDTFT | CNTL_LCDPWR | CNTL_LCDVCOMP(1);
	/* XXX: Choose format correctly by propagating it from the primary plane's atomic state.
	 */
	/*
	if (crtc->state->fb->format->format == DRM_FORMAT_RGB565)
		cntl |= CNTL_LCDBPP16_565;
	else if (crtc->state->fb->format->format == DRM_FORMAT_XRGB8888)
		cntl |= CNTL_LCDBPP24;
	else
		BUG_ON(1);
	*/
	cntl |= CNTL_LCDBPP24;

	cntl |= CNTL_BGR;

	writel(cntl, priv.regs + CLCD_PL111_CNTL);

	board = priv.amba_dev->dev.platform_data;

	if (board->enable)
		board->enable(NULL);

	/* Enable Interrupts */
	writel(CLCD_IRQ_NEXTBASE_UPDATE, priv.regs + CLCD_PL111_IENB);
}

void pl111_crtc_helper_disable(struct drm_crtc *crtc)
{
	struct clcd_board *board;

	DRM_DEBUG_KMS("DRM %s on crtc=%p\n", __func__, crtc);

	/* Disable Interrupts */
	writel(0x00000000, priv.regs + CLCD_PL111_IENB);

	board = priv.amba_dev->dev.platform_data;

	if (board->disable)
		board->disable(NULL);

	/* Disable and Power Down */
	writel(0, priv.regs + CLCD_PL111_CNTL);

	/* Disable clock */
	clk_disable_unprepare(priv.clk);
}

void pl111_crtc_destroy(struct drm_crtc *crtc)
{
	struct pl111_drm_crtc *pl111_crtc = to_pl111_crtc(crtc);

	DRM_DEBUG_KMS("DRM %s on crtc=%p\n", __func__, crtc);

	destroy_workqueue(pl111_crtc->vsync_wq);
	drm_crtc_cleanup(crtc);
	kfree(pl111_crtc);
}

/*
 * pl111 does not have a proper HW counter for vblank IRQs so enable_vblank
 * and disable_vblank are just no op callbacks.
 */
static int pl111_enable_vblank(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("%s: dev=%p, crtc=%d", __func__, crtc->dev,
		      drm_crtc_index(crtc));
	return 0;
}

static void pl111_disable_vblank(struct drm_crtc *crtc)
{
	DRM_DEBUG_KMS("%s: dev=%p, crtc=%d", __func__, crtc->dev,
		      drm_crtc_index(crtc));
}

const struct drm_crtc_funcs crtc_funcs = {
	.set_config = drm_crtc_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.destroy = pl111_crtc_destroy,
	.enable_vblank = pl111_enable_vblank,
	.disable_vblank = pl111_disable_vblank,
};

const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	.mode_set_nofb = pl111_crtc_helper_mode_set_nofb,
	.prepare = pl111_crtc_helper_prepare,
	.commit = pl111_crtc_helper_commit,
	.mode_fixup = pl111_crtc_helper_mode_fixup,
	.disable = pl111_crtc_helper_disable,
};

struct pl111_drm_crtc *pl111_crtc_create(struct drm_device *dev)
{
	struct pl111_drm_crtc *pl111_crtc;

	pl111_crtc = kzalloc(sizeof(struct pl111_drm_crtc), GFP_KERNEL);
	if (pl111_crtc == NULL) {
		pr_err("Failed to allocated pl111_drm_crtc\n");
		return NULL;
	}

	drm_crtc_init(dev, &pl111_crtc->crtc, &crtc_funcs);
	drm_crtc_helper_add(&pl111_crtc->crtc, &crtc_helper_funcs);

	pl111_crtc->crtc_index = pl111_crtc_num;
	pl111_crtc_num++;

	INIT_LIST_HEAD(&pl111_crtc->update_queue);
	spin_lock_init(&pl111_crtc->current_displaying_lock);
	spin_lock_init(&pl111_crtc->base_update_lock);

	return pl111_crtc;
}

