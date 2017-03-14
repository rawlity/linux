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

irqreturn_t pl111_irq(int irq, void *data)
{
	struct pl111_drm_dev_private *priv = data;
	u32 irq_stat;
	struct pl111_drm_crtc *pl111_crtc = priv->pl111_crtc;

	irq_stat = readl(priv->regs + CLCD_PL111_MIS);

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
	writel(irq_stat, priv->regs + CLCD_PL111_ICR);

	return IRQ_HANDLED;
}

int pl111_device_init(struct drm_device *drm)
{
	struct pl111_drm_dev_private *priv = drm->dev_private;
	struct device *dev = &priv->amba_dev->dev;
	int ret;

	/* set up MMIO for register access */
	priv->mmio_start = priv->amba_dev->res.start;
	priv->mmio_len = resource_size(&priv->amba_dev->res);

	DRM_DEBUG_KMS("mmio_start=%lu, mmio_len=%u\n", priv->mmio_start,
			priv->mmio_len);

	priv->regs = devm_ioremap(dev, priv->mmio_start, priv->mmio_len);
	if (priv->regs == NULL) {
		pr_err("%s failed mmio\n", __func__);
		return -EINVAL;
	}

	/* turn off interrupts */
	writel(0, priv->regs + CLCD_PL111_IENB);

	ret = devm_request_irq(dev, priv->amba_dev->irq[0], pl111_irq, 0,
			       "pl111_irq_handler", priv);
	if (ret != 0) {
		pr_err("%s failed irq %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

void pl111_device_fini(struct drm_device *dev)
{
	struct pl111_drm_dev_private *priv = dev->dev_private;
	u32 cntl;

	cntl = readl(priv->regs + CLCD_PL111_CNTL);

	cntl &= ~CNTL_LCDEN;
	writel(cntl, priv->regs + CLCD_PL111_CNTL);

	cntl &= ~CNTL_LCDPWR;
	writel(cntl, priv->regs + CLCD_PL111_CNTL);
}
