/*
 * Copyright © 2016 Broadcom Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Portions of this file (derived from panel-simple.c) are:
 *
 * Copyright (C) 2013, NVIDIA Corporation.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

/**
 * DOC: Raspberry Pi 7" touchscreen panel driver.
 *
 * The 7" touchscreen consists of a DPI LCD panel, a Toshiba
 * TC358762XBG DSI-DPI bridge, and an I2C-connected Atmel ATTINY88-MUR
 * controlling power management, the LCD PWM, and the touchscreen.
 *
 * This driver presents this device as a MIPI DSI panel to the DRM
 * driver, and should expose the touchscreen as a HID device.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm.h>

#include <drm/drm_panel.h>
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

/* I2C registers of the Atmel microcontroller. */
enum REG_ADDR {
	REG_ID = 0x80,
	REG_PORTA,
	REG_PORTB,
	REG_PORTC,
	REG_PORTD,
	REG_POWERON,
	REG_PWM,
	REG_DDRA,
	REG_DDRB,
	REG_DDRC,
	REG_DDRD,
	REG_TEST,
	REG_WR_ADDRL,
	REG_WR_ADDRH,
	REG_READH,
	REG_READL,
	REG_WRITEH,
	REG_WRITEL,
	REG_ID2,
};

/* We only turn the PWM on or off, without varying values. */
#define RPI_TOUCHSCREEN_MAX_BRIGHTNESS 1

/* DSI D-PHY Layer Registers */
#define D0W_DPHYCONTTX		0x0004
#define CLW_DPHYCONTRX		0x0020
#define D0W_DPHYCONTRX		0x0024
#define D1W_DPHYCONTRX		0x0028
#define COM_DPHYCONTRX		0x0038
#define CLW_CNTRL		0x0040
#define D0W_CNTRL		0x0044
#define D1W_CNTRL		0x0048
#define DFTMODE_CNTRL		0x0054

/* DSI PPI Layer Registers */
#define PPI_STARTPPI		0x0104
#define PPI_BUSYPPI		0x0108
#define PPI_LINEINITCNT		0x0110
#define PPI_LPTXTIMECNT		0x0114
//#define PPI_LANEENABLE		0x0134
//#define PPI_TX_RX_TA		0x013C
#define PPI_CLS_ATMR		0x0140
#define PPI_D0S_ATMR		0x0144
#define PPI_D1S_ATMR		0x0148
#define PPI_D0S_CLRSIPOCOUNT	0x0164
#define PPI_D1S_CLRSIPOCOUNT	0x0168
#define CLS_PRE			0x0180
#define D0S_PRE			0x0184
#define D1S_PRE			0x0188
#define CLS_PREP		0x01A0
#define D0S_PREP		0x01A4
#define D1S_PREP		0x01A8
#define CLS_ZERO		0x01C0
#define D0S_ZERO		0x01C4
#define D1S_ZERO		0x01C8
#define PPI_CLRFLG		0x01E0
#define PPI_CLRSIPO		0x01E4
#define HSTIMEOUT		0x01F0
#define HSTIMEOUTENABLE		0x01F4

/* DSI Protocol Layer Registers */
#define DSI_STARTDSI		0x0204
#define DSI_BUSYDSI		0x0208
#define DSI_LANEENABLE		0x0210
# define DSI_LANEENABLE_CLOCK		BIT(0)
# define DSI_LANEENABLE_D0		BIT(1)
# define DSI_LANEENABLE_D1		BIT(2)

#define DSI_LANESTATUS0		0x0214
#define DSI_LANESTATUS1		0x0218
#define DSI_INTSTATUS		0x0220
#define DSI_INTMASK		0x0224
#define DSI_INTCLR		0x0228
#define DSI_LPTXTO		0x0230
#define DSI_MODE		0x0260
#define DSI_PAYLOAD0		0x0268
#define DSI_PAYLOAD1		0x026C
#define DSI_SHORTPKTDAT		0x0270
#define DSI_SHORTPKTREQ		0x0274
#define DSI_BTASTA		0x0278
#define DSI_BTACLR		0x027C

/* DSI General Registers */
#define DSIERRCNT		0x0300
#define DSISIGMOD		0x0304

/* DSI Application Layer Registers */
#define APLCTRL			0x0400
#define APLSTAT			0x0404
#define APLERR			0x0408
#define PWRMOD			0x040C
#define RDPKTLN			0x0410
#define PXLFMT			0x0414
#define MEMWRCMD		0x0418

/* LCDC/DPI Host Registers */
#define LCDCTRL			0x0420
#define HSR			0x0424
#define HDISPR			0x0428
#define VSR			0x042C
#define VDISPR			0x0430
#define VFUEN			0x0434

/* DBI-B Host Registers */
#define DBIBCTRL		0x0440

/* SPI Master Registers */
#define SPICMR			0x0450
#define SPITCR			0x0454

/* System Controller Registers */
#define SYSSTAT			0x0460
#define SYSCTRL			0x0464
#define SYSPLL1			0x0468
#define SYSPLL2			0x046C
#define SYSPLL3			0x0470
#define SYSPMCTRL		0x047C

/* GPIO Registers */
#define GPIOC			0x0480
#define GPIOO			0x0484
#define GPIOI			0x0488

/* I2C Registers */
#define I2CCLKCTRL		0x0490

/* Chip/Rev Registers */
#define IDREG			0x04A0

/* Debug Registers */
#define WCMDQUEUE		0x0500
#define RCMDQUEUE		0x0504

struct rpi_touchscreen {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;
	struct i2c_client *bridge_i2c;
	struct backlight_device *backlight;

	bool prepared;
	bool enabled;

	/* Version of the firmware on the bridge chip */
	int atmel_ver;
};

static const struct drm_display_mode rpi_touchscreen_modes[] = {
	{
		/* This is assuming that we'll be running the DSI PLL
		 * at 2Ghz / 3 (since we only get integer dividers),
		 * so a pixel clock of 2Ghz / 3 / 8.
		 */
		.clock = 83333,
		.hdisplay = 800,
		.hsync_start = 800 + 61,
		.hsync_end = 800 + 61 + 2,
		.htotal = 800 + 61 + 2 + 44,
		.vdisplay = 480,
		.vsync_start = 480 + 7,
		.vsync_end = 480 + 7 + 2,
		.vtotal = 480 + 7 + 2 + 21,
		.vrefresh = 60,
	},
};

static struct rpi_touchscreen *panel_to_ts(struct drm_panel *panel)
{
	return container_of(panel, struct rpi_touchscreen, base);
}

static u8 rpi_touchscreen_i2c_read(struct rpi_touchscreen *ts, u8 reg)
{
	dev_info(ts->base.dev, "R 0x%02x\n", reg);

	return i2c_smbus_read_byte_data(ts->bridge_i2c, reg);
}

static void rpi_touchscreen_i2c_write(struct rpi_touchscreen *ts, u8 reg, u8 val)
{
	int ret;

	dev_info(ts->base.dev, "W 0x%02x -> 0x%02x\n", reg, val);

	ret = i2c_smbus_write_byte_data(ts->bridge_i2c, reg, val);
	if (ret)
		dev_err(&ts->dsi->dev, "I2C write failed: %d\n", ret);
}

static int rpi_touchscreen_write(struct rpi_touchscreen *ts, u16 reg, u32 val)
{
	u8 msg[] = {
		reg,
		reg >> 8,
		val,
		val >> 8,
		val >> 16,
		val >> 24,
	};

	dev_info(ts->base.dev, "W 0x%04x -> 0x%08x\n", reg, val);

	mipi_dsi_dcs_write_buffer(ts->dsi, msg, sizeof(msg));

	return 0;
}

static int rpi_touchscreen_disable(struct drm_panel *panel)
{
	struct rpi_touchscreen *ts = panel_to_ts(panel);
	pr_err("disable\n");

	rpi_touchscreen_i2c_write(ts, REG_POWERON, 0);
	udelay(1);

	if (!ts->enabled)
		return 0;

	if (ts->backlight) {
		ts->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ts->backlight);
	}

	ts->enabled = false;
	pr_err("disable done\n");

	return 0;
}

static int rpi_touchscreen_unprepare(struct drm_panel *panel)
{
	struct rpi_touchscreen *ts = panel_to_ts(panel);
	pr_err("unprep\n");

	if (!ts->prepared)
		return 0;

	ts->prepared = false;

	return 0;
}

static int rpi_touchscreen_prepare(struct drm_panel *panel)
{
	struct rpi_touchscreen *ts = panel_to_ts(panel);

	pr_err("prep\n");
	if (ts->prepared)
		return 0;

	ts->prepared = true;

	return 0;
}

static int rpi_touchscreen_enable(struct drm_panel *panel)
{
	struct rpi_touchscreen *ts = panel_to_ts(panel);
	int i;

	pr_err("enable\n");

	if (ts->enabled)
		return 0;

	/* Turn on the backklight. */
	rpi_touchscreen_i2c_write(ts, REG_PWM, 255);

	rpi_touchscreen_i2c_write(ts, REG_PORTA, 4); /* rotation state */

	rpi_touchscreen_i2c_write(ts, REG_POWERON, 1);
	/* Wait for nPWRDWN to go low to indicate poweron is done. */
	for (i = 0; i < 100; i++) {
		if (rpi_touchscreen_i2c_read(ts, REG_PORTB) & 1)
			break;
	}

	rpi_touchscreen_write(ts, DSI_LANEENABLE,
			      DSI_LANEENABLE_CLOCK |
			      DSI_LANEENABLE_D0 |
			      (ts->dsi->lanes > 1 ? DSI_LANEENABLE_D1 : 0));
	rpi_touchscreen_write(ts, PPI_D0S_CLRSIPOCOUNT, 0x05);
	rpi_touchscreen_write(ts, PPI_D1S_CLRSIPOCOUNT, 0x05);
	rpi_touchscreen_write(ts, PPI_D0S_ATMR, 0x00);
	rpi_touchscreen_write(ts, PPI_D1S_ATMR, 0x00);
	rpi_touchscreen_write(ts, PPI_LPTXTIMECNT, 0x03);

	rpi_touchscreen_write(ts, SPICMR, 0x00);
	rpi_touchscreen_write(ts, LCDCTRL, 0x00100150);
	rpi_touchscreen_write(ts, SYSCTRL, 0x040f);

	rpi_touchscreen_write(ts, PPI_STARTPPI, 0x01);
	rpi_touchscreen_write(ts, DSI_STARTDSI, 0x01);
	msleep(100);

	if (ts->backlight) {
		ts->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ts->backlight);
	}

	ts->enabled = true;
	pr_err("enable done\n");

	return 0;
}

static int rpi_touchscreen_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct drm_device *drm = panel->drm;
	unsigned int i, num = 0;

	pr_err("get modes\n");

	for (i = 0; i < ARRAY_SIZE(rpi_touchscreen_modes); i++) {
		const struct drm_display_mode *m = &rpi_touchscreen_modes[i];
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(drm, m);
		if (!mode) {
			dev_err(drm->dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay, m->vrefresh);
			continue;
		}

		mode->type |= DRM_MODE_TYPE_DRIVER;

		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);

		drm_mode_probed_add(connector, mode);
		num++;
	}

	connector->display_info.bpc = 8;
	connector->display_info.width_mm = 217; /* XXX */
	connector->display_info.height_mm = 136; /* XXX */

	pr_err("get %d modes done\n", num);

	return num;
}

static int rpi_touchscreen_backlight_update(struct backlight_device *bl)
{
	struct rpi_touchscreen *ts = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK))
		brightness = 0;

	rpi_touchscreen_i2c_write(ts, REG_PWM, brightness);

	return 0;
}

static const struct backlight_ops rpi_touchscreen_backlight_ops = {
	.update_status	= rpi_touchscreen_backlight_update,
};

static const struct drm_panel_funcs rpi_touchscreen_funcs = {
	.disable = rpi_touchscreen_disable,
	.unprepare = rpi_touchscreen_unprepare,
	.prepare = rpi_touchscreen_prepare,
	.enable = rpi_touchscreen_enable,
	.get_modes = rpi_touchscreen_get_modes,
};

static struct i2c_client *rpi_touchscreen_get_i2c(struct device *dev,
						  const char *name)
{
	struct device_node *node;
	struct i2c_client *client;

	node = of_parse_phandle(dev->of_node, name, 0);
	if (!node)
		return ERR_PTR(-ENODEV);

	client = of_find_i2c_device_by_node(node);

	of_node_put(node);

	return client;
}

static int rpi_touchscreen_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct rpi_touchscreen *ts;
	int ret, ver;

	pr_err("panel probing\n");

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	dev_set_drvdata(dev, ts);

	ts->dsi = dsi;
	dsi->mode_flags = (MIPI_DSI_MODE_VIDEO |
			   MIPI_DSI_MODE_VIDEO_SYNC_PULSE);
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->lanes = 1;

	ts->bridge_i2c =
		rpi_touchscreen_get_i2c(dev, "raspberrypi,touchscreen-bridge");
	if (!ts->bridge_i2c) {
		ret = -EPROBE_DEFER;
		return ret;
	}

	ver = rpi_touchscreen_i2c_read(ts, REG_ID);
	switch (ver) {
	case 0xde:
		ts->atmel_ver = 1;
		break;
	case 0xc3:
		ts->atmel_ver = 2;
		break;
	default:
		dev_err(dev, "Unknown Atmel firmware revision: 0x%02x\n", ver);
		ts->atmel_ver = 2;
		break;
	}

#if 0
	ts->backlight =
		devm_backlight_device_register(dev,
					       "raspberrypi-touchscreen-backlight",
					       dev, ts,
					       &rpi_touchscreen_backlight_ops,
					       NULL);
	if (IS_ERR(ts->backlight)) {
		DRM_ERROR("failed to register backlight\n");
		return PTR_ERR(ts->backlight);
	}
	ts->backlight->props.max_brightness = RPI_TOUCHSCREEN_MAX_BRIGHTNESS;
	ts->backlight->props.brightness = RPI_TOUCHSCREEN_MAX_BRIGHTNESS;
#endif

	pr_err("panel initing\n");
	drm_panel_init(&ts->base);
	ts->base.dev = dev;
	ts->base.funcs = &rpi_touchscreen_funcs;

	ret = drm_panel_add(&ts->base);
	if (ret < 0)
		goto err_release_bridge;

	pr_err("panel attaching\n");

	return mipi_dsi_attach(dsi);

err_release_bridge:
	put_device(&ts->bridge_i2c->dev);
	return ret;
}

static int rpi_touchscreen_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct rpi_touchscreen *ts = dev_get_drvdata(dev);
	int ret;

	pr_err("panel removing\n");

	ret = mipi_dsi_detach(dsi);
	if (ret < 0) {
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);
		return ret;
	}

	drm_panel_detach(&ts->base);
	drm_panel_remove(&ts->base);

	put_device(&ts->bridge_i2c->dev);

	return 0;
}

static void rpi_touchscreen_dsi_shutdown(struct mipi_dsi_device *dsi)
{
	/* XXX: poweroff */
}

static const struct of_device_id rpi_touchscreen_of_match[] = {
	{ .compatible = "raspberrypi,touchscreen" },
	{ } /* sentinel */
};
MODULE_DEVICE_TABLE(of, rpi_touchscreen_of_match);

static struct mipi_dsi_driver rpi_touchscreen_driver = {
	.driver = {
		.name = "raspberrypi-touchscreen",
		.of_match_table = rpi_touchscreen_of_match,
	},
	.probe = rpi_touchscreen_dsi_probe,
	.remove = rpi_touchscreen_dsi_remove,
	.shutdown = rpi_touchscreen_dsi_shutdown,
};
module_mipi_dsi_driver(rpi_touchscreen_driver);

MODULE_AUTHOR("Eric Anholt <eric@anholt.net>");
MODULE_DESCRIPTION("Raspberry Pi 7-inch touchscreen driver");
MODULE_LICENSE("GPL v2");
