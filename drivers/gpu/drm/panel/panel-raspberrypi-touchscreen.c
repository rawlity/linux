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
#include <linux/regulator/consumer.h>

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
#define D2W_DPHYCONTRX		0x002C
#define D3W_DPHYCONTRX		0x0030
#define COM_DPHYCONTRX		0x0038
#define CLW_CNTRL		0x0040
#define D0W_CNTRL		0x0044
#define D1W_CNTRL		0x0048
#define D2W_CNTRL		0x004C
#define D3W_CNTRL		0x0050
#define DFTMODE_CNTRL		0x0054

/* DSI PPI Layer Registers */
#define PPI_STARTPPI		0x0104
#define PPI_BUSYPPI		0x0108
#define PPI_LINEINITCNT		0x0110
#define PPI_LPTXTIMECNT		0x0114
#define PPI_LANEENABLE		0x0134
#define PPI_TX_RX_TA		0x013C
#define PPI_CLS_ATMR		0x0140
#define PPI_D0S_ATMR		0x0144
#define PPI_D1S_ATMR		0x0148
#define PPI_D2S_ATMR		0x014C
#define PPI_D3S_ATMR		0x0150
#define PPI_D0S_CLRSIPOCOUNT	0x0164
#define PPI_D1S_CLRSIPOCOUNT	0x0168
#define PPI_D2S_CLRSIPOCOUNT	0x016C
#define PPI_D3S_CLRSIPOCOUNT	0x0170
#define CLS_PRE			0x0180
#define D0S_PRE			0x0184
#define D1S_PRE			0x0188
#define D2S_PRE			0x018C
#define D3S_PRE			0x0190
#define CLS_PREP		0x01A0
#define D0S_PREP		0x01A4
#define D1S_PREP		0x01A8
#define D2S_PREP		0x01AC
#define D3S_PREP		0x01B0
#define CLS_ZERO		0x01C0
#define D0S_ZERO		0x01C4
#define D1S_ZERO		0x01C8
#define D2S_ZERO		0x01CC
#define D3S_ZERO		0x01D0
#define PPI_CLRFLG		0x01E0
#define PPI_CLRSIPO		0x01E4
#define HSTIMEOUT		0x01F0
#define HSTIMEOUTENABLE		0x01F4

/* DSI Protocol Layer Registers */
#define DSI_STARTDSI		0x0204
#define DSI_BUSYDSI		0x0208
#define DSI_LANEENABLE		0x0210
#define DSI_LANESTATUS0		0x0214
#define DSI_LANESTATUS1		0x0218
#define DSI_INTSTATUS		0x0220
#define DSI_INTMASK		0x0224
#define DSI_INTCLR		0x0228
#define DSI_LPTXTO		0x0230

/* DSI General Registers */
#define DSIERRCNT		0x0300

/* DSI Application Layer Registers */
#define APLCTRL			0x0400
#define RDPKTLN			0x0404

/* Video Path Registers */
#define VPCTRL			0x0450
#define HTIM1			0x0454
#define HTIM2			0x0458
#define VTIM1			0x045C
#define VTIM2			0x0460
#define VFUEN			0x0464

/* from anholt */
#define LCDCTRL			0x0420
#define SYSCTRL			0x0464
#define SPICMR			0x0450

/* LVDS Registers */
#define LVMX0003		0x0480
#define LVMX0407		0x0484
#define LVMX0811		0x0488
#define LVMX1215		0x048C
#define LVMX1619		0x0490
#define LVMX2023		0x0494
#define LVMX2427		0x0498
#define LVCFG			0x049C
#define LVPHY0			0x04A0
#define LVPHY1			0x04A4

/* System Registers */
#define SYSSTAT			0x0500
#define SYSRST			0x0504

/* GPIO Registers */
/*#define GPIOC			0x0520*/
#define GPIOO			0x0524
#define GPIOI			0x0528

/* I2C Registers */
#define I2CTIMCTRL		0x0540
#define I2CMADDR		0x0544
#define WDATAQ			0x0548
#define RDATAQ			0x054C

/* Chip/Rev Registers */
#define IDREG			0x0580

/* Debug Registers */
#define DEBUG00			0x05A0
#define DEBUG01			0x05A4

struct rpi_touchscreen {
	struct drm_panel base;
	struct i2c_client *client;
	struct drm_bridge bridge;
	struct backlight_device *bl;

	bool prepared;
	bool enabled;
};

static const struct drm_display_mode rpi_touchscreen_modes[] = {
	{
		.clock = 157200,
		.hdisplay = 800,
		.hsync_start = 800 + 61,
		.hsync_end = 800 + 61 + 2,
		.htotal = 800 + 61 + 2 + 44,
		.vdisplay = 480,
		.vsync_start = 480 + 7,
		.vsync_end = 480 + 7 + 2,
		.vtotal = 480 + 7 + 2 + 21,
		.vrefresh = 60,
	};
};

#if 0
static const struct panel_desc_dsi raspberrypi_touchscreen = {
       .flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
                MIPI_DSI_CLOCK_NON_CONTINUOUS, /* XXX */
       .format = MIPI_DSI_FMT_RGB888,
       .lanes = 1,
};
#endif

static struct rpi_touchscreen *
panel_to_tc358762(struct drm_panel *panel)
{
	return container_of(panel, struct rpi_touchscreen, base);
}

static u8 rpi_touchscreen_i2c_read(struct rpi_touchscreen *ts, u8 reg)
{
	return i2c_smbus_read_byte_data(ts->client, reg);
}

static int tc358762_i2c_write(struct rpi_touchscreen *ts, u8 reg, u8 val)
{
	dev_info(&ts->client->dev, "W 0x%02x -> 0x%02x\n", reg, val);

	return i2c_smbus_write_byte_data(ts->client, reg, val);
}

static int tc358762_write(struct rpi_touchscreen *ts, u16 reg, u32 val)
{
	/* XXX */
	return 0;
}

static int rpi_touchscreen_disable(struct drm_panel *panel)
{
	struct rpi_touchscreen *ts = panel_to_ts(panel);

	tc358762_i2c_write(tc358762, REG_POWERON, 0);
	udelay(1);

	if (!p->enabled)
		return 0;

	if (p->backlight) {
		p->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(p->backlight);
	}

	if (p->desc->delay.disable)
		msleep(p->desc->delay.disable);

	p->enabled = false;

	return 0;
}

static int rpi_touchscreen_unprepare(struct drm_panel *panel)
{
	struct rpi_touchscreen *ts = panel_to_ts(panel);

	if (!p->prepared)
		return 0;

	if (p->desc->delay.unprepare)
		msleep(p->desc->delay.unprepare);

	p->prepared = false;

	return 0;
}

static int panel_simple_prepare(struct drm_panel *panel)
{
	struct rpi_touchscreen *ts = panel_to_ts(panel);
	int err;

	if (p->prepared)
		return 0;

	err = regulator_enable(p->supply);
	if (err < 0) {
		dev_err(panel->dev, "failed to enable supply: %d\n", err);
		return err;
	}

	if (p->enable_gpio)
		gpiod_set_value_cansleep(p->enable_gpio, 1);

	if (p->desc->delay.prepare)
		msleep(p->desc->delay.prepare);

	p->prepared = true;

	return 0;
}

static int panel_simple_enable(struct drm_panel *panel)
{
	struct rpi_touchscreen *ts = panel_to_ts(panel);

	if (p->enabled)
		return 0;

	if (p->desc->delay.enable)
		msleep(p->desc->delay.enable);

	if (p->backlight) {
		p->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(p->backlight);
	}

	p->enabled = true;

	return 0;
}

static int panel_simple_get_modes(struct drm_panel *panel)
{
	struct rpi_touchscreen *ts = panel_to_ts(panel);
	struct drm_connector *connector = panel->base.connector;
	struct drm_device *drm = panel->base.drm;
	struct drm_display_mode *mode;
	unsigned int i, num = 0;

	for (i = 0; i < ARRAY_SIZE(rpi_touchscreen_modes); i++) {
		const struct drm_display_mode *m = &rpi_touchscreen_modes[i];

		mode = drm_mode_duplicate(drm, m);
		if (!mode) {
			dev_err(drm->dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay, m->vrefresh);
			continue;
		}

		mode->type |= DRM_MODE_TYPE_DRIVER;

		if (panel->desc->num_modes == 1)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);

		drm_mode_probed_add(connector, mode);
		num++;
	}

	connector->display_info.bpc = 8;
	connector->display_info.width = 217; /* XXX */
	connector->display_info.height = 136; /* XXX */

	/* XXX
	if (panel->desc->bus_format)
		drm_display_info_set_bus_formats(&connector->display_info,
						 &panel->desc->bus_format, 1);
	*/

	return num;
}

static int panel_simple_get_timings(struct drm_panel *panel,
				    unsigned int num_timings,
				    struct display_timing *timings)
{
	struct rpi_touchscreen *ts = panel_to_ts(panel);
	unsigned int i;

	if (p->desc->num_timings < num_timings)
		num_timings = p->desc->num_timings;

	if (timings)
		for (i = 0; i < num_timings; i++)
			timings[i] = p->desc->timings[i];

	return p->desc->num_timings;
}

static int rpi_touchscreen_backlight_update(struct backlight_device *bl)
{
	struct rpi_touchscreen *ts = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK))
		brightness = 0;

	return tc358762_i2c_write(tc358762, REG_PWM, brightness);
}

static const struct backlight_ops rpi_touchscreen_backlight_ops = {
	.update_status	= rpi_touchscreen_backlight_update,
};

static void tc358762_pre_enable(struct drm_bridge *bridge)
{
	struct rpi_touchscreen *ts = bridge_to_tc358762(bridge);

	if (ts->enabled)
		return;

	if (drm_panel_prepare(ts->panel)) {
		DRM_ERROR("failed to prepare panel\n");
		return;
	}

	ts->enabled = true;
}

static void tc358762_enable(struct drm_bridge *bridge)
{
	struct rpi_touchscreen *ts = bridge_to_tc358762(bridge);
	int i;
	int lanes = 1; /* XXX */

	/* Turn on the backklight. */
	tc358762_i2c_write(tc358762, REG_PWM, 255);

	tc358762_i2c_write(tc358762, REG_PORTA, 4); /* rotation state */

	tc358762_i2c_write(tc358762, REG_POWERON, 1);
	/* Wait for nPWRDWN to go low to indicate poweron is done. */
	for (i = 0; i < 100; i++) {
		if (tc358762_i2c_read(tc358762, REG_PORTB) & 1)
			break;
	}

	tc358762_write(tc358762, PPI_LANEENABLE, lanes == 2 ? 0x07 : 0x03);
	tc358762_write(tc358762, PPI_D0S_CLRSIPOCOUNT, 0x05);
	tc358762_write(tc358762, PPI_D1S_CLRSIPOCOUNT, 0x05);
	tc358762_write(tc358762, PPI_D0S_ATMR, 0x00);
	tc358762_write(tc358762, PPI_D1S_ATMR, 0x00);
	tc358762_write(tc358762, PPI_LPTXTIMECNT, 0x03);

	tc358762_write(tc358762, SPICMR, 0x00);
	tc358762_write(tc358762, LCDCTRL, 0x00100150);
	tc358762_write(tc358762, SYSCTRL, 0x040f);

	tc358762_write(tc358762, PPI_STARTPPI, 0x01);
	tc358762_write(tc358762, DSI_STARTDSI, 0x01);
	msleep(100);

	if (drm_panel_enable(ts->panel)) {
		DRM_ERROR("failed to enable panel\n");
		return;
	}
}

static const struct drm_panel_funcs rpi_touchscreen_funcs = {
	.disable = panel_simple_disable,
	.unprepare = panel_simple_unprepare,
	.prepare = panel_simple_prepare,
	.enable = panel_simple_enable,
	.get_modes = panel_simple_get_modes,
	.get_timings = panel_simple_get_timings,
};

static const struct of_device_id tc358762_devices[] = {
	{.compatible = "toshiba,tc358762",},
	{}
};
MODULE_DEVICE_TABLE(of, tc358762_devices);

static int tc358762_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *panel_node;
	struct rpi_touchscreen *ts;
	int ret;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;

	ts->bl = devm_backlight_device_register(dev,
						"raspberrypi-touchscreen-backlight",
						dev, ts,
						&ts_backlight_ops,
						NULL);
	if (IS_ERR(ts->bl)) {
		DRM_ERROR("failed to register backlight\n");
		return PTR_ERR(ts->bl);
	}
	ts->bl->props.max_brightness = RPI_TOUCHSCREEN_MAX_BRIGHTNESS;
	ts->bl->props.brightness = RPI_TOUCHSCREEN_MAX_BRIGHTNESS;

	ts->bridge.funcs = &rpi_touchscreen_funcs;
	ts->bridge.of_node = dev->of_node;

	drm_panel_init(&ts->base);
	ts->base.dev = dev;
	ts->base.funcs = &rpi_touschreen_funcs;
	ret = drm_panel_add(&ts->base);
	if (ret < 0)
		return ret;

	i2c_set_clientdata(client, ts);

	return 0;
}

static int rpi_touchscreen_remove(struct i2c_client *client)
{
	struct rpi_touchscreen *ts = i2c_get_clientdata(client);

	drm_panel_detach(&panel->base);
	drm_panel_remove(&panel->base);

	return 0;
}

static const struct i2c_device_id tc358762_i2c_table[] = {
	{"touchscreen"},
	{},
};
MODULE_DEVICE_TABLE(i2c, tc358762_i2c_table);

MODULE_AUTHOR("Eric Anholt <eric@anholt.net>");
MODULE_DESCRIPTION("Raspberry Pi 7-inch touchscreen driver");
MODULE_LICENSE("GPL v2");
