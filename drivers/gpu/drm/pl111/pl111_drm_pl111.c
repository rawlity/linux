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
 * pl111_drm_pl111.c
 * PL111 specific functions for PL111 DRM
 */
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
#include <linux/version.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>
#include <linux/module.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include "pl111_clcd_ext.h"

#include "pl111_drm.h"

void do_flip_to_res(struct pl111_drm_flip_resource *flip_res)
{
	struct pl111_drm_crtc *pl111_crtc = to_pl111_crtc(flip_res->crtc);
	struct drm_framebuffer *fb;
	struct pl111_gem_bo *bo;

	fb = flip_res->fb;
	bo = PL111_BO_FROM_FRAMEBUFFER(fb);

	/* Don't even attempt PL111_BOT_SHM, it's not contiguous */
	BUG_ON(bo->type != PL111_BOT_DMA);

	/*
	 * Note the buffer for releasing after IRQ, and don't allow any more
	 * updates until then.
	 *
	 * This clcd controller latches the new address on next vsync. Address
	 * latching is indicated by CLCD_IRQ_NEXTBASE_UPDATE, and so we must
	 * wait for that before releasing the previous buffer's kds
	 * resources. Otherwise, we'll allow writers to write to the old buffer
	 * whilst it is still being displayed
	 */
	pl111_crtc->current_update_res = flip_res;

	DRM_DEBUG_KMS("Displaying fb 0x%p, dumb_bo 0x%p, physaddr %.8x\n",
			fb, bo, bo->backing_data.dma.fb_dev_addr);

	if (drm_crtc_vblank_get(&pl111_crtc->crtc) < 0)
		DRM_ERROR("Could not get vblank reference for crtc %d\n",
				pl111_crtc->crtc_index);

	/* Set the scanout buffer */
}

void
show_framebuffer_on_crtc_cb_internal(struct pl111_drm_flip_resource *flip_res,
					struct drm_framebuffer *fb)
{
	unsigned long irq_flags;
	struct pl111_drm_crtc *pl111_crtc = to_pl111_crtc(flip_res->crtc);

	spin_lock_irqsave(&pl111_crtc->base_update_lock, irq_flags);
	if (list_empty(&pl111_crtc->update_queue) &&
			!pl111_crtc->current_update_res) {
		do_flip_to_res(flip_res);

		/*
		 * Catch a potential race with the IRQ handler:
		 * - We may've updated the Base Address just after it was
		 *   latched, but before the OS ran our IRQ handler
		 * - Hence, the CLCD controller is now scanning out the
		 *   previous buffer, not our new buffer.
		 * - However, as soon as the IRQ handler runs, it'll inspect
		 *   pl111_crtc->current_update_res, and use that to cause the
		 *   previous buffer to be released on the workqueue (even
		 *   though the CLCD controller is still scanning it out)
		 * Instead, we must wait until the *next* IRQ to allow
		 * releasing of the previous buffer:
		 */
		if (readl(priv.regs + CLCD_PL111_MIS) &
				CLCD_IRQ_NEXTBASE_UPDATE) {
			DRM_DEBUG_KMS("Redoing flip to fb %p on next IRQ\n",
					fb);
			pl111_crtc->current_update_res = NULL;
			list_add_tail(&flip_res->link,
					&pl111_crtc->update_queue);
		}
	} else {
		/*
		 * Enqueue the update to occur on a future IRQ
		 * This only happens on triple-or-greater buffering
		 */
		DRM_DEBUG_KMS("Deferring 3+ buffered flip to fb %p to IRQ\n",
				fb);
		list_add_tail(&flip_res->link, &pl111_crtc->update_queue);
	}

	spin_unlock_irqrestore(&pl111_crtc->base_update_lock, irq_flags);

	if (!flip_res->page_flip) {
		drm_mode_destroy(flip_res->crtc->dev, pl111_crtc->current_mode);
		pl111_crtc->current_mode = pl111_crtc->new_mode;
		pl111_crtc->new_mode = NULL;
	}

	BUG_ON(pl111_crtc->new_mode);
	BUG_ON(!pl111_crtc->current_mode);

	/*
	 * If IRQs weren't enabled before, they are now. This will eventually
	 * cause flip_res to be released via vsync_worker(), which updates
	 * every time the Base Address is latched (i.e. every frame, regardless
	 * of whether we update the base address or not)
	 */
}

irqreturn_t pl111_irq(int irq, void *data)
{
	u32 irq_stat;
	struct pl111_drm_crtc *pl111_crtc = priv.pl111_crtc;

	irq_stat = readl(priv.regs + CLCD_PL111_MIS);

	if (!irq_stat)
		return IRQ_NONE;

	if (irq_stat & CLCD_IRQ_NEXTBASE_UPDATE) {
		if (pl111_crtc->current_update_res ||
				!list_empty(&pl111_crtc->update_queue))
			DRM_DEBUG_KMS("DRM irq %x after base update\n",
					irq_stat);

		/*
		 * We don't need to lock here as we don't do any flip-specific
		 * processing in this function. All these, including locks, is
		 * done in common_irq handler
		 */
		pl111_common_irq(pl111_crtc);
	}

	/* Clear the interrupt once done */
	writel(irq_stat, priv.regs + CLCD_PL111_ICR);

	return IRQ_HANDLED;
}

int pl111_device_init(struct drm_device *dev)
{
	struct pl111_drm_dev_private *priv = dev->dev_private;
	int ret;

	if (priv == NULL || priv->amba_dev == NULL)
		return -EINVAL;

	/* set up MMIO for register access */
	priv->mmio_start = priv->amba_dev->res.start;
	priv->mmio_len = resource_size(&priv->amba_dev->res);

	DRM_DEBUG_KMS("mmio_start=%lu, mmio_len=%u\n", priv->mmio_start,
			priv->mmio_len);

	priv->regs = ioremap(priv->mmio_start, priv->mmio_len);
	if (priv->regs == NULL) {
		pr_err("%s failed mmio\n", __func__);
		return -EINVAL;
	}

	/* turn off interrupts */
	writel(0, priv->regs + CLCD_PL111_IENB);

	ret = request_irq(priv->amba_dev->irq[0], pl111_irq, 0,
				"pl111_irq_handler", NULL);
	if (ret != 0) {
		pr_err("%s failed %d\n", __func__, ret);
		goto out_mmio;
	}

	goto finish;

out_mmio:
	iounmap(priv->regs);
finish:
	DRM_DEBUG_KMS("pl111_device_init returned %d\n", ret);
	return ret;
}

void pl111_device_fini(struct drm_device *dev)
{
	struct pl111_drm_dev_private *priv = dev->dev_private;
	u32 cntl;

	if (priv == NULL || priv->regs == NULL)
		return;

	free_irq(priv->amba_dev->irq[0], NULL);

	cntl = readl(priv->regs + CLCD_PL111_CNTL);

	cntl &= ~CNTL_LCDEN;
	writel(cntl, priv->regs + CLCD_PL111_CNTL);

	cntl &= ~CNTL_LCDPWR;
	writel(cntl, priv->regs + CLCD_PL111_CNTL);

	iounmap(priv->regs);
}

int pl111_amba_probe(struct amba_device *dev, const struct amba_id *id)
{
	struct clcd_board *board = dev->dev.platform_data;
	int ret;
	pr_info("DRM %s\n", __func__);

	if (board == NULL)
		return -EINVAL;

	ret = amba_request_regions(dev, NULL);
	if (ret != 0) {
		DRM_ERROR("CLCD: unable to reserve regs region\n");
		goto out;
	}

	priv.amba_dev = dev;

	priv.clk = clk_get(&priv.amba_dev->dev, NULL);
	if (IS_ERR(priv.clk)) {
		DRM_ERROR("CLCD: unable to get clk.\n");
		ret = PTR_ERR(priv.clk);
		goto clk_err;
	}

	return 0;

clk_err:
	amba_release_regions(dev);
out:
	return ret;
}

int pl111_amba_remove(struct amba_device *dev)
{
	DRM_DEBUG_KMS("DRM %s\n", __func__);

	clk_put(priv.clk);

	amba_release_regions(dev);

	priv.amba_dev = NULL;

	return 0;
}

void pl111_set_cursor_size(enum pl111_cursor_size size)
{
	u32 reg_data = readl(priv.regs + CLCD_CRSR_CONFIG);

	if (size == CURSOR_64X64)
		reg_data |= CRSR_CONFIG_CRSR_SIZE;
	else
		reg_data &= ~CRSR_CONFIG_CRSR_SIZE;

	writel(reg_data, priv.regs + CLCD_CRSR_CONFIG);
}

void pl111_set_cursor_sync(enum pl111_cursor_sync sync)
{
	u32 reg_data = readl(priv.regs + CLCD_CRSR_CONFIG);

	if (sync == CURSOR_SYNC_VSYNC)
		reg_data |= CRSR_CONFIG_CRSR_FRAME_SYNC;
	else
		reg_data &= ~CRSR_CONFIG_CRSR_FRAME_SYNC;

	writel(reg_data, priv.regs + CLCD_CRSR_CONFIG);
}

void pl111_set_cursor(u32 cursor)
{
	u32 reg_data = readl(priv.regs + CLCD_CRSR_CTRL);

	reg_data &= ~(CRSR_CTRL_CRSR_MAX << CRSR_CTRL_CRSR_NUM_SHIFT);
	reg_data |= (cursor & CRSR_CTRL_CRSR_MAX) << CRSR_CTRL_CRSR_NUM_SHIFT;

	writel(reg_data, priv.regs + CLCD_CRSR_CTRL);
}

void pl111_set_cursor_enable(bool enable)
{
	u32 reg_data = readl(priv.regs + CLCD_CRSR_CTRL);

	if (enable)
		reg_data |= CRSR_CTRL_CRSR_ON;
	else
		reg_data &= ~CRSR_CTRL_CRSR_ON;

	writel(reg_data, priv.regs + CLCD_CRSR_CTRL);
}

void pl111_set_cursor_position(u32 x, u32 y)
{
	u32 reg_data = (x & CRSR_XY_MASK) |
			((y & CRSR_XY_MASK) << CRSR_XY_Y_SHIFT);
	/* could optimise out if same values */
	writel(reg_data, priv.regs + CLCD_CRSR_XY);
}

void pl111_set_cursor_clipping(u32 x, u32 y)
{
	u32 reg_data = (x & CRSR_CLIP_MASK) |
			((y & CRSR_CLIP_MASK) << CRSR_CLIP_Y_SHIFT);
	/* could optimise out if same values */
	writel(reg_data, priv.regs + CLCD_CRSR_CLIP);
}

void pl111_set_cursor_palette(u32 color0, u32 color1)
{
	writel(color0 & CRSR_PALETTE_MASK, priv.regs + CLCD_CRSR_PALETTE_0);
	writel(color1 & CRSR_PALETTE_MASK, priv.regs + CLCD_CRSR_PALETTE_1);
}

void pl111_cursor_enable(void)
{
	pl111_set_cursor_sync(CURSOR_SYNC_VSYNC);
	pl111_set_cursor_size(CURSOR_64X64);
	pl111_set_cursor_palette(0x0, 0x00ffffff);
	pl111_set_cursor_enable(true);
}

void pl111_cursor_disable(void)
{
	pl111_set_cursor_enable(false);
}

void pl111_set_cursor_image(u32 *data)
{
	u32 *cursor_ram = priv.regs + CLCD_CRSR_IMAGE;
	int i;

	for (i = 0; i < CLCD_CRSR_IMAGE_MAX_WORDS; i++, data++, cursor_ram++)
		writel(*data, cursor_ram);
}
