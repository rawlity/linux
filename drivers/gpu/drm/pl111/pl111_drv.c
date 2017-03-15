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
 * pl111_drm_device.c
 * Implementation of the Linux device driver entrypoints for PL111 DRM
 */
#include <linux/amba/bus.h>
#include <linux/amba/clcd.h>
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

struct pl111_drm_dev_private priv;

struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int pl111_modeset_init(struct drm_device *dev)
{
	struct drm_mode_config *mode_config;
	struct pl111_drm_dev_private *priv = dev->dev_private;
	struct pl111_drm_connector *pl111_connector;
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
		pr_err("Failed to init primary plane\n");
		goto out_config;
	}

	ret = pl111_crtc_create(dev);
	if (ret) {
		pr_err("Failed to create crtc\n");
		goto out_config;
	}

	pl111_connector = pl111_connector_create(dev);
	if (pl111_connector == NULL) {
		pr_err("Failed to create pl111_drm_connector\n");
		ret = -ENOMEM;
		goto out_config;
	}

	ret = pl111_encoder_init(dev);
	if (ret) {
		pr_err("Failed to create pl111_drm_encoder\n");
		goto out_config;
	}

	ret = drm_mode_connector_attach_encoder(&pl111_connector->connector,
						&priv->encoder);
	if (ret != 0) {
		DRM_ERROR("Failed to attach encoder\n");
		goto out_config;
	}

	pl111_connector->connector.encoder = &priv->encoder;

	ret = drm_vblank_init(dev, 1);
	if (ret != 0) {
		DRM_ERROR("Failed to init vblank\n");
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
	DRM_DEBUG("%s returned %d\n", __func__, ret);
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
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
	.dumb_create = pl111_dumb_create,
	.dumb_destroy = drm_gem_dumb_destroy,
	.dumb_map_offset = drm_gem_cma_dumb_map_offset,
	.gem_free_object = drm_gem_cma_free_object,
	.gem_vm_ops = &drm_gem_cma_vm_ops,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_export = drm_gem_prime_export,
};

static int pl111_amba_probe(struct amba_device *amba_dev,
			    const struct amba_id *id)
{
	struct device *dev = &amba_dev->dev;
	struct pl111_drm_dev_private *priv;
	struct drm_device *drm;
	int ret;
	pr_info("DRM %s\n", __func__);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	drm = drm_dev_alloc(&pl111_drm_driver, dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);
	amba_set_drvdata(amba_dev, drm);
	priv->drm = drm;
	drm->dev_private = priv;

	priv->amba_dev = amba_dev;

	priv->clk = devm_clk_get(dev, "clcdclk");
	if (IS_ERR(priv->clk)) {
		DRM_ERROR("CLCD: unable to get clk.\n");
		ret = PTR_ERR(priv->clk);
		goto dev_unref;
	}

	priv->regs = devm_ioremap_resource(dev, &priv->amba_dev->res);
	if (priv->regs == NULL) {
		pr_err("%s failed mmio\n", __func__);
		return -EINVAL;
	}

	/* turn off interrupts before requesting the irq */
	writel(0, priv->regs + CLCD_PL111_IENB);

	ret = devm_request_irq(dev, priv->amba_dev->irq[0], pl111_irq, 0,
			       "pl111_irq_handler", priv);
	if (ret != 0) {
		pr_err("%s failed irq %d\n", __func__, ret);
		return ret;
	}

	ret = pl111_modeset_init(drm);
	if (ret != 0) {
		pr_err("Failed to init modeset\n");
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

module_amba_driver(pl111_amba_driver);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE(DRIVER_LICENCE);
MODULE_ALIAS(DRIVER_ALIAS);
