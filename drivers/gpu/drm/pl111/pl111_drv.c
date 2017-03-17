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
 * DOC: ARM PrimeCell PL111 CLCD Driver
 *
 * The PL111 is a simple LCD controller that can support TFT and STN
 * displays.  This driver exposes a standard KMS interface for them.
 *
 * This driver uses the same Device Tree binding as the fbdev CLCD
 * driver.  While the fbdev driver supports panels that may be
 * connected to the CLCD internally to the CLCD driver, in DRM the
 * panels get split out to drivers/gpu/drm/panels/.  This means that,
 * in converting from using fbdev to using DRM, you also need to write
 * a panel driver (which may be as simple as an entry in
 * panel-simple.c).
 *
 * The driver currently doesn't expose the cursor.  The DRM API for
 * cursors requires support for 64x64 ARGB8888 cursor images, while
 * the hardware can only support 64x64 monochrome with masking
 * cursors.  While one could imagine trying to hack something together
 * to look at the ARGB8888 and program reasonable in monochrome, we
 * just don't expose the cursor at all instead, and leave cursor
 * support to the X11 software cursor layer.
 *
 * TODO:
 *
 * - Fix race between setting plane base address and getting IRQ for
 *   vsync firing the pageflip completion.
 *
 * - Expose the correct set of formats we can support based on the
 *   "arm,pl11x,tft-r0g0b0-pads" DT property.
 *
 * - Use the "max-memory-bandwidth" DT property to filter the
 *   supported formats.
 *
 * - Read back hardware state at boot to skip reprogramming the
 *   hardware when doing a no-op modeset.
 *
 * - Use the internal clock divisor to reduce power consumption by
 *   using HCLK (apb_pclk) when appropriate.
 */

#include <linux/amba/bus.h>
#include <linux/amba/clcd-regs.h>
#include <linux/version.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>

#include "pl111_drm.h"

#define DRIVER_DESC      "DRM module for PL111"

struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int pl111_modeset_init(struct drm_device *dev)
{
	struct drm_mode_config *mode_config;
	struct pl111_drm_dev_private *priv = dev->dev_private;
	int ret = 0;

	if (priv == NULL)
		return -EINVAL;

	drm_mode_config_init(dev);
	mode_config = &dev->mode_config;
	mode_config->funcs = &mode_config_funcs;
	mode_config->min_width = 1;
	mode_config->max_width = 1024;
	mode_config->min_height = 1;
	mode_config->max_height = 768;

	ret = pl111_primary_plane_init(dev);
	if (ret != 0) {
		dev_err(dev->dev, "Failed to init primary plane\n");
		goto out_config;
	}

	ret = pl111_crtc_create(dev);
	if (ret) {
		dev_err(dev->dev, "Failed to create crtc\n");
		goto out_config;
	}

	ret = pl111_connector_create(dev);
	if (ret) {
		dev_err(dev->dev, "Failed to create pl111_drm_connector\n");
		goto out_config;
	}

	ret = pl111_encoder_init(dev);
	if (ret) {
		dev_err(dev->dev, "Failed to create pl111_drm_encoder\n");
		goto out_config;
	}

	ret = drm_mode_connector_attach_encoder(&priv->connector.connector,
						&priv->encoder);
	if (ret != 0) {
		dev_err(dev->dev, "Failed to attach encoder\n");
		goto out_config;
	}

	priv->connector.connector.encoder = &priv->encoder;

	ret = drm_vblank_init(dev, 1);
	if (ret != 0) {
		dev_err(dev->dev, "Failed to init vblank\n");
		goto out_config;
	}

	drm_mode_config_reset(dev);

	priv->fbdev = drm_fbdev_cma_init(dev, 32,
					 dev->mode_config.num_connector);

	drm_kms_helper_poll_init(dev);

	goto finish;

out_config:
	drm_mode_config_cleanup(dev);
finish:
	return ret;
}

static const struct file_operations drm_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = drm_gem_cma_mmap,
	.poll = drm_poll,
	.read = drm_read,
};

static void pl111_lastclose(struct drm_device *dev)
{
	struct pl111_drm_dev_private *priv = dev->dev_private;

	drm_fbdev_cma_restore_mode(priv->fbdev);
}

static struct drm_driver pl111_drm_driver = {
	.driver_features =
		DRIVER_MODESET | DRIVER_GEM | DRIVER_PRIME | DRIVER_ATOMIC,
	.lastclose = pl111_lastclose,
	.ioctls = NULL,
	.fops = &drm_fops,
	.name = "pl111_drm",
	.desc = DRIVER_DESC,
	.date = "20170317",
	.major = 1,
	.minor = 0,
	.patchlevel = 0,
	.dumb_create = pl111_dumb_create,
	.dumb_destroy = drm_gem_dumb_destroy,
	.dumb_map_offset = drm_gem_cma_dumb_map_offset,
	.gem_free_object = drm_gem_cma_free_object,
	.gem_vm_ops = &drm_gem_cma_vm_ops,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_export = drm_gem_prime_export,
	.gem_prime_get_sg_table	= drm_gem_cma_prime_get_sg_table,
};

#ifdef CONFIG_ARM_AMBA
static int pl111_amba_probe(struct amba_device *amba_dev,
			    const struct amba_id *id)
{
	struct device *dev = &amba_dev->dev;
	struct pl111_drm_dev_private *priv;
	struct drm_device *drm;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	drm = drm_dev_alloc(&pl111_drm_driver, dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);
	amba_set_drvdata(amba_dev, drm);
	priv->drm = drm;
	drm->dev_private = priv;

	priv->clk = devm_clk_get(dev, "clcdclk");
	if (IS_ERR(priv->clk)) {
		dev_err(dev, "CLCD: unable to get clk.\n");
		ret = PTR_ERR(priv->clk);
		goto dev_unref;
	}

	priv->regs = devm_ioremap_resource(dev, &amba_dev->res);
	if (priv->regs == NULL) {
		dev_err(dev, "%s failed mmio\n", __func__);
		return -EINVAL;
	}

	/* turn off interrupts before requesting the irq */
	writel(0, priv->regs + CLCD_PL111_IENB);

	ret = devm_request_irq(dev, amba_dev->irq[0], pl111_irq, 0,
			       "pl111", priv);
	if (ret != 0) {
		dev_err(dev, "%s failed irq %d\n", __func__, ret);
		return ret;
	}

	ret = pl111_modeset_init(drm);
	if (ret != 0) {
		dev_err(dev, "Failed to init modeset\n");
		goto dev_unref;
	}

	ret = drm_dev_register(drm, 0);
	if (ret < 0)
		goto dev_unref;

	return 0;

dev_unref:
	drm_dev_unref(drm);
	return ret;
}

static int pl111_amba_remove(struct amba_device *amba_dev)
{
	struct drm_device *drm = amba_get_drvdata(amba_dev);
	struct pl111_drm_dev_private *priv = drm->dev_private;

	drm_dev_unregister(drm);
	if (priv->fbdev)
		drm_fbdev_cma_fini(priv->fbdev);
	drm_mode_config_cleanup(drm);
	drm_dev_unref(drm);

	return 0;
}

static struct amba_id pl111_id_table[] = {
	{
		.id = 0x00041110,
		.mask = 0x000ffffe,
	},
	{0, 0},
};

static struct amba_driver pl111_amba_driver = {
	.drv = {
		.name = "clcd-pl11x",
	},
	.probe = pl111_amba_probe,
	.remove = pl111_amba_remove,
	.id_table = pl111_id_table,
};
#endif /* CONFIG_ARM_AMBA */

module_amba_driver(pl111_amba_driver);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("ARM Ltd.");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pl111_drm");
