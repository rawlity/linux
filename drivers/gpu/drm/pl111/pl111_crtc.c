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

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>

#include "pl111_drm.h"

irqreturn_t pl111_irq(int irq, void *data)
{
	struct pl111_drm_dev_private *priv = data;
	u32 irq_stat;
	irqreturn_t status = IRQ_NONE;

	irq_stat = readl(priv->regs + CLCD_PL111_MIS);

	if (!irq_stat)
		return IRQ_NONE;

	if (irq_stat & CLCD_IRQ_NEXTBASE_UPDATE) {
		drm_crtc_handle_vblank(&priv->crtc);

		status = IRQ_HANDLED;
	}

	/* Clear the interrupt once done */
	writel(irq_stat, priv->regs + CLCD_PL111_ICR);

	return status;
}

static void pl111_crtc_helper_mode_set_nofb(struct drm_crtc *crtc)
{
	struct pl111_drm_dev_private *priv = crtc->dev->dev_private;
	const struct drm_display_mode *mode = &crtc->state->mode;
	struct drm_display_mode local_mode = {
		.hdisplay = 640,
		.hsync_start = 640 + 60,
		.hsync_end = 640 + 60 + 70,
		.htotal = 640 + 60 + 70 + 140,

		.vdisplay = 480,
		.vsync_start = 480 + 5,
		.vsync_end = 480 + 5 + 3,
		.vtotal = 480 + 5 + 3 + 33,
		.clock = 27000,
	};
	unsigned int ppl, hsw, hfp, hbp;
	unsigned int lpp, vsw, vfp, vbp;
	unsigned int cpl;
	int ret;

	/* XXX: Hack in 911360's display for the moment. */
	mode = &local_mode;

	ret = clk_set_rate(priv->clk, mode->clock * 1000);
	if (ret) {
		dev_err(&priv->amba_dev->dev,
			"Failed to set pixel clock rate: %d\n", ret);
	}

	ppl = (mode->hdisplay / 16) - 1;
	hsw = mode->hsync_end - mode->hsync_start - 1;
	hfp = mode->hsync_start - mode->hdisplay - 1;
	hbp = mode->htotal - mode->hsync_end - 1;

	lpp = mode->vdisplay - 1;
	vsw = mode->vsync_end - mode->vsync_start - 1;
	vfp = mode->vsync_start - mode->vdisplay;
	vbp = mode->vtotal - mode->vsync_end;

	cpl = mode->hdisplay - 1;

	writel((ppl << 2) | (hsw << 8) | (hfp << 16) | (hbp << 24),
	       priv->regs + CLCD_TIM0);
	writel(lpp | (vsw << 10) | (vfp << 16) | (vbp << 24),
	       priv->regs + CLCD_TIM1);
	writel(TIM2_IVS | TIM2_IHS | TIM2_IPC | TIM2_BCD | (cpl << 16) |
	       TIM2_CLKSEL,
	       priv->regs + CLCD_TIM2);
	writel(0, priv->regs + CLCD_TIM3);
}

static void pl111_crtc_helper_enable(struct drm_crtc *crtc)
{
	struct pl111_drm_dev_private *priv = crtc->dev->dev_private;
	u32 cntl;

	DRM_DEBUG_KMS("DRM %s on crtc=%p\n", __func__, crtc);

	clk_prepare_enable(priv->clk);

	/* Enable and Power Up */
	cntl = CNTL_LCDEN | CNTL_LCDTFT | CNTL_LCDPWR | CNTL_LCDVCOMP(1);
	/* XXX: Choose format correctly by propagating it from the
	 * primary plane's atomic state.
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

	writel(cntl, priv->regs + CLCD_PL111_CNTL);
}

void pl111_crtc_helper_disable(struct drm_crtc *crtc)
{
	struct pl111_drm_dev_private *priv = crtc->dev->dev_private;

	DRM_DEBUG_KMS("DRM %s on crtc=%p\n", __func__, crtc);

	/* Disable and Power Down */
	writel(0, priv->regs + CLCD_PL111_CNTL);

	/* Disable clock */
	clk_disable_unprepare(priv->clk);
}

static void pl111_crtc_helper_atomic_flush(struct drm_crtc *crtc,
					   struct drm_crtc_state *old_state)
{
	struct drm_pending_vblank_event *event = crtc->state->event;

	if (event) {
		crtc->state->event = NULL;

		spin_lock_irq(&crtc->dev->event_lock);
		if (drm_crtc_vblank_get(crtc) == 0)
			drm_crtc_arm_vblank_event(crtc, event);
		else
			drm_crtc_send_vblank_event(crtc, event);
		spin_unlock_irq(&crtc->dev->event_lock);
	}
}

static int pl111_enable_vblank(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct pl111_drm_dev_private *priv = dev->dev_private;

	writel(CLCD_IRQ_NEXTBASE_UPDATE, priv->regs + CLCD_PL111_IENB);

	return 0;
}

static void pl111_disable_vblank(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct pl111_drm_dev_private *priv = dev->dev_private;

	writel(0, priv->regs + CLCD_PL111_IENB);
}

const struct drm_crtc_funcs crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.enable_vblank = pl111_enable_vblank,
	.disable_vblank = pl111_disable_vblank,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	.mode_set_nofb = pl111_crtc_helper_mode_set_nofb,
	.atomic_flush = pl111_crtc_helper_atomic_flush,
	.disable = pl111_crtc_helper_disable,
	.enable = pl111_crtc_helper_enable,
};

int pl111_crtc_create(struct drm_device *dev)
{
	struct pl111_drm_dev_private *priv = dev->dev_private;
	struct drm_crtc *crtc = &priv->crtc;

	drm_crtc_init_with_planes(dev, crtc,
				  &priv->primary, NULL,
				  &crtc_funcs, "primary");
	drm_crtc_helper_add(crtc, &crtc_helper_funcs);

	return 0;
}
