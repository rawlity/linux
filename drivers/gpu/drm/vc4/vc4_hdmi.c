/*
 * Copyright (C) 2015 Broadcom
 * Copyright (c) 2014 The Linux Foundation. All rights reserved.
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * DOC: VC4 Falcon HDMI module
 *
 * The HDMI core has a state machine and a PHY.  Most of the unit
 * operates off of the HSM clock from CPRMAN.  It also internally uses
 * the PLLH_PIX clock for the PHY.
 */

#include "drm_atomic_helper.h"
#include "drm_crtc_helper.h"
#include "drm_edid.h"
#include "linux/clk.h"
#include "linux/component.h"
#include "linux/i2c.h"
#include "linux/of_gpio.h"
#include "linux/of_platform.h"
#include "linux/rational.h"
#include "sound/hdmi-codec.h"
#include "sound/bcm2835-hdmi.h"
#include "vc4_drv.h"
#include "vc4_regs.h"

/* General HDMI hardware state. */
struct vc4_hdmi {
	struct platform_device *pdev;

	struct drm_encoder *encoder;
	struct drm_connector *connector;

	struct hdmi_codec_params audio_params;

	struct i2c_adapter *ddc;
	void __iomem *hdmicore_regs;
	void __iomem *hd_regs;
	int hpd_gpio;
	bool hpd_active_low;

	struct clk *pixel_clock;
	struct clk *hsm_clock;
};

#define HDMI_READ(offset) readl(vc4->hdmi->hdmicore_regs + offset)
#define HDMI_WRITE(offset, val) writel(val, vc4->hdmi->hdmicore_regs + offset)
#define HD_READ(offset) readl(vc4->hdmi->hd_regs + offset)
#define HD_WRITE(offset, val) writel(val, vc4->hdmi->hd_regs + offset)

/* VC4 HDMI encoder KMS struct */
struct vc4_hdmi_encoder {
	struct vc4_encoder base;
	bool hdmi_monitor;
	bool limited_rgb_range;
	bool rgb_range_selectable;
};

static inline struct vc4_hdmi_encoder *
to_vc4_hdmi_encoder(struct drm_encoder *encoder)
{
	return container_of(encoder, struct vc4_hdmi_encoder, base.base);
}

/* VC4 HDMI connector KMS struct */
struct vc4_hdmi_connector {
	struct drm_connector base;

	/* Since the connector is attached to just the one encoder,
	 * this is the reference to it so we can do the best_encoder()
	 * hook.
	 */
	struct drm_encoder *encoder;
};

static inline struct vc4_hdmi_connector *
to_vc4_hdmi_connector(struct drm_connector *connector)
{
	return container_of(connector, struct vc4_hdmi_connector, base);
}

#define HDMI_REG(reg) { reg, #reg }
static const struct {
	u32 reg;
	const char *name;
} hdmi_regs[] = {
	HDMI_REG(VC4_HDMI_CORE_REV),
	HDMI_REG(VC4_HDMI_SW_RESET_CONTROL),
	HDMI_REG(VC4_HDMI_HOTPLUG_INT),
	HDMI_REG(VC4_HDMI_HOTPLUG),
	HDMI_REG(VC4_HDMI_MAI_CHANNEL_MAP),
	HDMI_REG(VC4_HDMI_MAI_CONFIG),
	HDMI_REG(VC4_HDMI_MAI_FORMAT),
	HDMI_REG(VC4_HDMI_AUDIO_PACKET_CONFIG),
	HDMI_REG(VC4_HDMI_RAM_PACKET_CONFIG),
	HDMI_REG(VC4_HDMI_HORZA),
	HDMI_REG(VC4_HDMI_HORZB),
	HDMI_REG(VC4_HDMI_FIFO_CTL),
	HDMI_REG(VC4_HDMI_SCHEDULER_CONTROL),
	HDMI_REG(VC4_HDMI_VERTA0),
	HDMI_REG(VC4_HDMI_VERTA1),
	HDMI_REG(VC4_HDMI_VERTB0),
	HDMI_REG(VC4_HDMI_VERTB1),
	HDMI_REG(VC4_HDMI_TX_PHY_RESET_CTL),
};

static const struct {
	u32 reg;
	const char *name;
} hd_regs[] = {
	HDMI_REG(VC4_HD_M_CTL),
	HDMI_REG(VC4_HD_MAI_CTL),
	HDMI_REG(VC4_HD_MAI_THR),
	HDMI_REG(VC4_HD_MAI_FMT),
	HDMI_REG(VC4_HD_MAI_SMP),
	HDMI_REG(VC4_HD_VID_CTL),
	HDMI_REG(VC4_HD_CSC_CTL),
	HDMI_REG(VC4_HD_FRAME_COUNT),
};

#ifdef CONFIG_DEBUG_FS
int vc4_hdmi_debugfs_regs(struct seq_file *m, void *unused)
{
	struct drm_info_node *node = (struct drm_info_node *)m->private;
	struct drm_device *dev = node->minor->dev;
	struct vc4_dev *vc4 = to_vc4_dev(dev);
	int i;

	for (i = 0; i < ARRAY_SIZE(hdmi_regs); i++) {
		seq_printf(m, "%s (0x%04x): 0x%08x\n",
			   hdmi_regs[i].name, hdmi_regs[i].reg,
			   HDMI_READ(hdmi_regs[i].reg));
	}

	for (i = 0; i < ARRAY_SIZE(hd_regs); i++) {
		seq_printf(m, "%s (0x%04x): 0x%08x\n",
			   hd_regs[i].name, hd_regs[i].reg,
			   HD_READ(hd_regs[i].reg));
	}

	return 0;
}
#endif /* CONFIG_DEBUG_FS */

static void vc4_hdmi_dump_regs(struct drm_device *dev)
{
	struct vc4_dev *vc4 = to_vc4_dev(dev);
	int i;

	for (i = 0; i < ARRAY_SIZE(hdmi_regs); i++) {
		DRM_INFO("0x%04x (%s): 0x%08x\n",
			 hdmi_regs[i].reg, hdmi_regs[i].name,
			 HDMI_READ(hdmi_regs[i].reg));
	}
	for (i = 0; i < ARRAY_SIZE(hd_regs); i++) {
		DRM_INFO("0x%04x (%s): 0x%08x\n",
			 hd_regs[i].reg, hd_regs[i].name,
			 HD_READ(hd_regs[i].reg));
	}
}

static enum drm_connector_status
vc4_hdmi_connector_detect(struct drm_connector *connector, bool force)
{
	struct drm_device *dev = connector->dev;
	struct vc4_dev *vc4 = to_vc4_dev(dev);

	if (vc4->hdmi->hpd_gpio) {
		if (gpio_get_value_cansleep(vc4->hdmi->hpd_gpio) ^
		    vc4->hdmi->hpd_active_low)
			return connector_status_connected;
		else
			return connector_status_disconnected;
	}

	if (drm_probe_ddc(vc4->hdmi->ddc))
		return connector_status_connected;

	if (HDMI_READ(VC4_HDMI_HOTPLUG) & VC4_HDMI_HOTPLUG_CONNECTED)
		return connector_status_connected;
	else
		return connector_status_disconnected;
}

static void vc4_hdmi_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static int vc4_hdmi_connector_get_modes(struct drm_connector *connector)
{
	struct vc4_hdmi_connector *vc4_connector =
		to_vc4_hdmi_connector(connector);
	struct drm_encoder *encoder = vc4_connector->encoder;
	struct vc4_hdmi_encoder *vc4_encoder = to_vc4_hdmi_encoder(encoder);
	struct drm_device *dev = connector->dev;
	struct vc4_dev *vc4 = to_vc4_dev(dev);
	int ret = 0;
	struct edid *edid;

	edid = drm_get_edid(connector, vc4->hdmi->ddc);
	if (!edid)
		return -ENODEV;

	vc4_encoder->hdmi_monitor = drm_detect_hdmi_monitor(edid);

	if (edid && edid->input & DRM_EDID_INPUT_DIGITAL) {
		vc4_encoder->rgb_range_selectable =
			drm_rgb_quant_range_selectable(edid);
	}

	drm_mode_connector_update_edid_property(connector, edid);
	ret = drm_add_edid_modes(connector, edid);
	drm_edid_to_eld(connector, edid);

	return ret;
}

static const struct drm_connector_funcs vc4_hdmi_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.detect = vc4_hdmi_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = vc4_hdmi_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs vc4_hdmi_connector_helper_funcs = {
	.get_modes = vc4_hdmi_connector_get_modes,
};

static struct drm_connector *vc4_hdmi_connector_init(struct drm_device *dev,
						     struct drm_encoder *encoder)
{
	struct drm_connector *connector = NULL;
	struct vc4_hdmi_connector *hdmi_connector;
	int ret = 0;

	hdmi_connector = devm_kzalloc(dev->dev, sizeof(*hdmi_connector),
				      GFP_KERNEL);
	if (!hdmi_connector) {
		ret = -ENOMEM;
		goto fail;
	}
	connector = &hdmi_connector->base;

	hdmi_connector->encoder = encoder;

	drm_connector_init(dev, connector, &vc4_hdmi_connector_funcs,
			   DRM_MODE_CONNECTOR_HDMIA);
	drm_connector_helper_add(connector, &vc4_hdmi_connector_helper_funcs);

	connector->polled = (DRM_CONNECTOR_POLL_CONNECT |
			     DRM_CONNECTOR_POLL_DISCONNECT);

	connector->interlace_allowed = 1;
	connector->doublescan_allowed = 0;

	drm_mode_connector_attach_encoder(connector, encoder);

	return connector;

 fail:
	if (connector)
		vc4_hdmi_connector_destroy(connector);

	return ERR_PTR(ret);
}

static void vc4_hdmi_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs vc4_hdmi_encoder_funcs = {
	.destroy = vc4_hdmi_encoder_destroy,
};

static int vc4_hdmi_stop_packet(struct drm_encoder *encoder,
				enum hdmi_infoframe_type type)
{
	struct drm_device *dev = encoder->dev;
	struct vc4_dev *vc4 = to_vc4_dev(dev);
	u32 packet_id = type - 0x80;

	HDMI_WRITE(VC4_HDMI_RAM_PACKET_CONFIG,
		   HDMI_READ(VC4_HDMI_RAM_PACKET_CONFIG) & ~BIT(packet_id));

	return wait_for(!(HDMI_READ(VC4_HDMI_RAM_PACKET_STATUS) &
			  BIT(packet_id)), 100);
}

static void vc4_hdmi_write_infoframe(struct drm_encoder *encoder,
				     union hdmi_infoframe *frame)
{
	struct drm_device *dev = encoder->dev;
	struct vc4_dev *vc4 = to_vc4_dev(dev);
	u32 packet_id = frame->any.type - 0x80;
	u32 packet_reg = VC4_HDMI_GCP_0 + VC4_HDMI_PACKET_STRIDE * packet_id;
	uint8_t buffer[VC4_HDMI_PACKET_STRIDE];
	ssize_t len, i;
	int ret;

	WARN_ONCE(!(HDMI_READ(VC4_HDMI_RAM_PACKET_CONFIG) &
		    VC4_HDMI_RAM_PACKET_ENABLE),
		  "Packet RAM has to be on to store the packet.");

	len = hdmi_infoframe_pack(frame, buffer, sizeof(buffer));
	if (len < 0)
		return;

	ret = vc4_hdmi_stop_packet(encoder, frame->any.type);
	if (ret) {
		DRM_ERROR("Failed to wait for infoframe to go idle: %d\n", ret);
		return;
	}

	for (i = 0; i < len; i += 7) {
		HDMI_WRITE(packet_reg,
			   buffer[i + 0] << 0 |
			   buffer[i + 1] << 8 |
			   buffer[i + 2] << 16);
		packet_reg += 4;

		HDMI_WRITE(packet_reg,
			   buffer[i + 3] << 0 |
			   buffer[i + 4] << 8 |
			   buffer[i + 5] << 16 |
			   buffer[i + 6] << 24);
		packet_reg += 4;
	}

	HDMI_WRITE(VC4_HDMI_RAM_PACKET_CONFIG,
		   HDMI_READ(VC4_HDMI_RAM_PACKET_CONFIG) | BIT(packet_id));
	ret = wait_for((HDMI_READ(VC4_HDMI_RAM_PACKET_STATUS) &
			BIT(packet_id)), 100);
	if (ret)
		DRM_ERROR("Failed to wait for infoframe to start: %d\n", ret);
}

static void vc4_hdmi_set_avi_infoframe(struct drm_encoder *encoder)
{
	struct vc4_hdmi_encoder *vc4_encoder = to_vc4_hdmi_encoder(encoder);
	struct drm_crtc *crtc = encoder->crtc;
	const struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	union hdmi_infoframe frame;
	int ret;

	ret = drm_hdmi_avi_infoframe_from_display_mode(&frame.avi, mode);
	if (ret < 0) {
		DRM_ERROR("couldn't fill AVI infoframe\n");
		return;
	}

	if (vc4_encoder->rgb_range_selectable) {
		if (vc4_encoder->limited_rgb_range) {
			frame.avi.quantization_range =
				HDMI_QUANTIZATION_RANGE_LIMITED;
		} else {
			frame.avi.quantization_range =
				HDMI_QUANTIZATION_RANGE_FULL;
		}
	}

	vc4_hdmi_write_infoframe(encoder, &frame);
}

static void vc4_hdmi_set_spd_infoframe(struct drm_encoder *encoder)
{
	union hdmi_infoframe frame;
	int ret;

	ret = hdmi_spd_infoframe_init(&frame.spd, "Broadcom", "Videocore");
	if (ret < 0) {
		DRM_ERROR("couldn't fill SPD infoframe\n");
		return;
	}

	frame.spd.sdi = HDMI_SPD_SDI_PC;

	vc4_hdmi_write_infoframe(encoder, &frame);
}

static void vc4_hdmi_set_audio_infoframe(struct drm_encoder *encoder)
{
	struct drm_device *drm = encoder->dev;
	struct vc4_dev *vc4 = drm->dev_private;
	struct vc4_hdmi *hdmi = vc4->hdmi;
	union hdmi_infoframe frame;
	int ret;

	ret = hdmi_audio_infoframe_init(&frame.audio);

	frame.audio.coding_type = HDMI_AUDIO_CODING_TYPE_STREAM;
	frame.audio.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_STREAM;
	frame.audio.sample_size = HDMI_AUDIO_SAMPLE_SIZE_STREAM;
	frame.audio.channels = hdmi->audio_params.cea.channels;

	vc4_hdmi_write_infoframe(encoder, &frame);
}

static void vc4_hdmi_set_infoframes(struct drm_encoder *encoder)
{
	vc4_hdmi_set_avi_infoframe(encoder);
	vc4_hdmi_set_spd_infoframe(encoder);
}

static void vc4_hdmi_encoder_mode_set(struct drm_encoder *encoder,
				      struct drm_display_mode *unadjusted_mode,
				      struct drm_display_mode *mode)
{
	struct vc4_hdmi_encoder *vc4_encoder = to_vc4_hdmi_encoder(encoder);
	struct drm_device *dev = encoder->dev;
	struct vc4_dev *vc4 = to_vc4_dev(dev);
	bool debug_dump_regs = false;
	bool hsync_pos = mode->flags & DRM_MODE_FLAG_PHSYNC;
	bool vsync_pos = mode->flags & DRM_MODE_FLAG_PVSYNC;
	bool interlaced = mode->flags & DRM_MODE_FLAG_INTERLACE;
	u32 pixel_rep = (mode->flags & DRM_MODE_FLAG_DBLCLK) ? 2 : 1;
	u32 verta = (VC4_SET_FIELD(mode->crtc_vsync_end - mode->crtc_vsync_start,
				   VC4_HDMI_VERTA_VSP) |
		     VC4_SET_FIELD(mode->crtc_vsync_start - mode->crtc_vdisplay,
				   VC4_HDMI_VERTA_VFP) |
		     VC4_SET_FIELD(mode->crtc_vdisplay, VC4_HDMI_VERTA_VAL));
	u32 vertb = (VC4_SET_FIELD(0, VC4_HDMI_VERTB_VSPO) |
		     VC4_SET_FIELD(mode->crtc_vtotal - mode->crtc_vsync_end,
				   VC4_HDMI_VERTB_VBP));
	u32 vertb_even = (VC4_SET_FIELD(0, VC4_HDMI_VERTB_VSPO) |
			  VC4_SET_FIELD(mode->crtc_vtotal -
					mode->crtc_vsync_end -
					interlaced,
					VC4_HDMI_VERTB_VBP));
	u32 csc_ctl;

	if (debug_dump_regs) {
		DRM_INFO("HDMI regs before:\n");
		vc4_hdmi_dump_regs(dev);
	}

	HD_WRITE(VC4_HD_VID_CTL, 0);

	clk_set_rate(vc4->hdmi->pixel_clock, mode->clock * 1000 *
		     ((mode->flags & DRM_MODE_FLAG_DBLCLK) ? 2 : 1));

	HDMI_WRITE(VC4_HDMI_SCHEDULER_CONTROL,
		   HDMI_READ(VC4_HDMI_SCHEDULER_CONTROL) |
		   VC4_HDMI_SCHEDULER_CONTROL_MANUAL_FORMAT |
		   VC4_HDMI_SCHEDULER_CONTROL_IGNORE_VSYNC_PREDICTS);

	HDMI_WRITE(VC4_HDMI_HORZA,
		   (vsync_pos ? VC4_HDMI_HORZA_VPOS : 0) |
		   (hsync_pos ? VC4_HDMI_HORZA_HPOS : 0) |
		   VC4_SET_FIELD(mode->hdisplay * pixel_rep,
				 VC4_HDMI_HORZA_HAP));

	HDMI_WRITE(VC4_HDMI_HORZB,
		   VC4_SET_FIELD((mode->htotal -
				  mode->hsync_end) * pixel_rep,
				 VC4_HDMI_HORZB_HBP) |
		   VC4_SET_FIELD((mode->hsync_end -
				  mode->hsync_start) * pixel_rep,
				 VC4_HDMI_HORZB_HSP) |
		   VC4_SET_FIELD((mode->hsync_start -
				  mode->hdisplay) * pixel_rep,
				 VC4_HDMI_HORZB_HFP));

	HDMI_WRITE(VC4_HDMI_VERTA0, verta);
	HDMI_WRITE(VC4_HDMI_VERTA1, verta);

	HDMI_WRITE(VC4_HDMI_VERTB0, vertb_even);
	HDMI_WRITE(VC4_HDMI_VERTB1, vertb);

	HD_WRITE(VC4_HD_VID_CTL,
		 (vsync_pos ? 0 : VC4_HD_VID_CTL_VSYNC_LOW) |
		 (hsync_pos ? 0 : VC4_HD_VID_CTL_HSYNC_LOW));

	csc_ctl = VC4_SET_FIELD(VC4_HD_CSC_CTL_ORDER_BGR,
				VC4_HD_CSC_CTL_ORDER);

	if (vc4_encoder->hdmi_monitor && drm_match_cea_mode(mode) > 1) {
		/* CEA VICs other than #1 requre limited range RGB
		 * output unless overridden by an AVI infoframe.
		 * Apply a colorspace conversion to squash 0-255 down
		 * to 16-235.  The matrix here is:
		 *
		 * [ 0      0      0.8594 16]
		 * [ 0      0.8594 0      16]
		 * [ 0.8594 0      0      16]
		 * [ 0      0      0       1]
		 */
		csc_ctl |= VC4_HD_CSC_CTL_ENABLE;
		csc_ctl |= VC4_HD_CSC_CTL_RGB2YCC;
		csc_ctl |= VC4_SET_FIELD(VC4_HD_CSC_CTL_MODE_CUSTOM,
					 VC4_HD_CSC_CTL_MODE);

		HD_WRITE(VC4_HD_CSC_12_11, (0x000 << 16) | 0x000);
		HD_WRITE(VC4_HD_CSC_14_13, (0x100 << 16) | 0x6e0);
		HD_WRITE(VC4_HD_CSC_22_21, (0x6e0 << 16) | 0x000);
		HD_WRITE(VC4_HD_CSC_24_23, (0x100 << 16) | 0x000);
		HD_WRITE(VC4_HD_CSC_32_31, (0x000 << 16) | 0x6e0);
		HD_WRITE(VC4_HD_CSC_34_33, (0x100 << 16) | 0x000);
		vc4_encoder->limited_rgb_range = true;
	} else {
		vc4_encoder->limited_rgb_range = false;
	}

	/* The RGB order applies even when CSC is disabled. */
	HD_WRITE(VC4_HD_CSC_CTL, csc_ctl);

	HDMI_WRITE(VC4_HDMI_FIFO_CTL, VC4_HDMI_FIFO_CTL_MASTER_SLAVE_N);

	if (debug_dump_regs) {
		DRM_INFO("HDMI regs after:\n");
		vc4_hdmi_dump_regs(dev);
	}
}

static void vc4_hdmi_encoder_disable(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct vc4_dev *vc4 = to_vc4_dev(dev);

	HDMI_WRITE(VC4_HDMI_RAM_PACKET_CONFIG, 0);

	HDMI_WRITE(VC4_HDMI_TX_PHY_RESET_CTL, 0xf << 16);
	HD_WRITE(VC4_HD_VID_CTL,
		 HD_READ(VC4_HD_VID_CTL) & ~VC4_HD_VID_CTL_ENABLE);
}

static void vc4_hdmi_encoder_enable(struct drm_encoder *encoder)
{
	struct vc4_hdmi_encoder *vc4_encoder = to_vc4_hdmi_encoder(encoder);
	struct drm_device *dev = encoder->dev;
	struct vc4_dev *vc4 = to_vc4_dev(dev);
	int ret;

	HDMI_WRITE(VC4_HDMI_TX_PHY_RESET_CTL, 0);

	HD_WRITE(VC4_HD_VID_CTL,
		 HD_READ(VC4_HD_VID_CTL) |
		 VC4_HD_VID_CTL_ENABLE |
		 VC4_HD_VID_CTL_UNDERFLOW_ENABLE |
		 VC4_HD_VID_CTL_FRAME_COUNTER_RESET);

	if (vc4_encoder->hdmi_monitor) {
		HDMI_WRITE(VC4_HDMI_SCHEDULER_CONTROL,
			   HDMI_READ(VC4_HDMI_SCHEDULER_CONTROL) |
			   VC4_HDMI_SCHEDULER_CONTROL_MODE_HDMI);

		ret = wait_for(HDMI_READ(VC4_HDMI_SCHEDULER_CONTROL) &
			       VC4_HDMI_SCHEDULER_CONTROL_HDMI_ACTIVE, 1000);
		WARN_ONCE(ret, "Timeout waiting for "
			  "VC4_HDMI_SCHEDULER_CONTROL_HDMI_ACTIVE\n");
	} else {
		HDMI_WRITE(VC4_HDMI_RAM_PACKET_CONFIG,
			   HDMI_READ(VC4_HDMI_RAM_PACKET_CONFIG) &
			   ~(VC4_HDMI_RAM_PACKET_ENABLE));
		HDMI_WRITE(VC4_HDMI_SCHEDULER_CONTROL,
			   HDMI_READ(VC4_HDMI_SCHEDULER_CONTROL) &
			   ~VC4_HDMI_SCHEDULER_CONTROL_MODE_HDMI);

		ret = wait_for(!(HDMI_READ(VC4_HDMI_SCHEDULER_CONTROL) &
				 VC4_HDMI_SCHEDULER_CONTROL_HDMI_ACTIVE), 1000);
		WARN_ONCE(ret, "Timeout waiting for "
			  "!VC4_HDMI_SCHEDULER_CONTROL_HDMI_ACTIVE\n");
	}

	if (vc4_encoder->hdmi_monitor) {
		u32 drift;

		WARN_ON(!(HDMI_READ(VC4_HDMI_SCHEDULER_CONTROL) &
			  VC4_HDMI_SCHEDULER_CONTROL_HDMI_ACTIVE));
		HDMI_WRITE(VC4_HDMI_SCHEDULER_CONTROL,
			   HDMI_READ(VC4_HDMI_SCHEDULER_CONTROL) |
			   VC4_HDMI_SCHEDULER_CONTROL_VERT_ALWAYS_KEEPOUT);

		HDMI_WRITE(VC4_HDMI_RAM_PACKET_CONFIG,
			   VC4_HDMI_RAM_PACKET_ENABLE);

		vc4_hdmi_set_infoframes(encoder);

		drift = HDMI_READ(VC4_HDMI_FIFO_CTL);
		drift &= VC4_HDMI_FIFO_VALID_WRITE_MASK;

		HDMI_WRITE(VC4_HDMI_FIFO_CTL,
			   drift & ~VC4_HDMI_FIFO_CTL_RECENTER);
		HDMI_WRITE(VC4_HDMI_FIFO_CTL,
			   drift | VC4_HDMI_FIFO_CTL_RECENTER);
		udelay(1000);
		HDMI_WRITE(VC4_HDMI_FIFO_CTL,
			   drift & ~VC4_HDMI_FIFO_CTL_RECENTER);
		HDMI_WRITE(VC4_HDMI_FIFO_CTL,
			   drift | VC4_HDMI_FIFO_CTL_RECENTER);

		ret = wait_for(HDMI_READ(VC4_HDMI_FIFO_CTL) &
			       VC4_HDMI_FIFO_CTL_RECENTER_DONE, 1);
		WARN_ONCE(ret, "Timeout waiting for "
			  "VC4_HDMI_FIFO_CTL_RECENTER_DONE");
	}
}

static const struct drm_encoder_helper_funcs vc4_hdmi_encoder_helper_funcs = {
	.mode_set = vc4_hdmi_encoder_mode_set,
	.disable = vc4_hdmi_encoder_disable,
	.enable = vc4_hdmi_encoder_enable,
};

/* HDMI audio codec callbacks */

static int vc4_hdmi_audio_hw_params(struct device *dev, void *data,
				    struct hdmi_codec_daifmt *daifmt,
				    struct hdmi_codec_params *params)
{
	struct vc4_hdmi *hdmi = dev_get_drvdata(dev);

	dev_info(dev, "%s: %u Hz, %d bit, %d channels\n", __func__,
		 params->sample_rate, params->sample_width,
		 params->cea.channels);

	if (daifmt->fmt != HDMI_SPDIF) {
		dev_err(dev, "%s: Invalid DAI format %d\n", __func__,
			daifmt->fmt);
		return -EINVAL;
	}

	memcpy(&hdmi->audio_params, params, sizeof(*params));

	/* XXX: Restart? */

	return 0;
}

static void vc4_hdmi_audio_set_mai_clock(struct vc4_hdmi *hdmi)
{
	struct drm_device *drm = hdmi->encoder->dev;
	struct vc4_dev *vc4 = to_vc4_dev(drm);
	u32 hsm_clock = clk_get_rate(hdmi->hsm_clock);
	u32 samplerate = 0 /* XXX */;
	unsigned long n, m;

	/* XXX: n/m right way around? */
	rational_best_approximation(hsm_clock, samplerate,
				    VC4_HD_MAI_SMP_N_MASK >>
				    VC4_HD_MAI_SMP_N_SHIFT,
				    (VC4_HD_MAI_SMP_M_MASK >>
				     VC4_HD_MAI_SMP_M_SHIFT) + 1,
				    &n, &m);

	HD_WRITE(VC4_HD_MAI_SMP,
		 VC4_SET_FIELD(n, VC4_HD_MAI_SMP_N) |
		 VC4_SET_FIELD(m + 1, VC4_HD_MAI_SMP_M));
}

static void vc4_hdmi_set_n_cts(struct vc4_hdmi *hdmi)
{
	struct drm_encoder *encoder = hdmi->encoder;
	struct drm_crtc *crtc = encoder->crtc;
	struct drm_device *drm = encoder->dev;
	struct vc4_dev *vc4 = to_vc4_dev(drm);
	const struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	u32 samplerate = hdmi->audio_params.sample_rate;
	u32 n, cts;
	u64 tmp;

	n = 128 * samplerate / 1000;
	tmp = (u64)(mode->clock * 1000) * n;
	do_div(tmp, 128 * samplerate);
	cts = tmp;

	DRM_INFO("Setting N/CTS to %d/%d\n", n, cts);

	HDMI_WRITE(VC4_HDMI_CRP_CFG,
		   VC4_HDMI_CRP_CFG_EXTERNAL_CTS_EN |
		   VC4_SET_FIELD(n, VC4_HDMI_CRP_CFG_N));

	/* We could get slightly more accurate clocks in some cases by
	 * providing a CTS_1 value.  The two CTS values are alternated
	 * between based on the period fields
	 */
	HDMI_WRITE(VC4_HDMI_CTS_0, cts);
	HDMI_WRITE(VC4_HDMI_CTS_1, cts);
}

static int vc4_hdmi_audio_startup(struct device *dev, void *data)
{
	struct vc4_hdmi *hdmi = dev_get_drvdata(dev);
	struct drm_encoder *encoder = hdmi->encoder;
	struct drm_device *drm = encoder->dev;
	struct vc4_dev *vc4 = to_vc4_dev(drm);
	u32 audio_packet_config, channel_mask;
	u32 channel_map, channel_map_next, i;

	dev_info(dev, "%s\n", __func__);

	HD_WRITE(VC4_HD_MAI_CTL,
		 VC4_HD_MAI_CTL_RESET |
		 VC4_HD_MAI_CTL_FLUSH);

	vc4_hdmi_audio_set_mai_clock(hdmi);

	audio_packet_config =
		VC4_HDMI_AUDIO_PACKET_ZERO_DATA_ON_SAMPLE_FLAT |
		VC4_HDMI_AUDIO_PACKET_ZERO_DATA_ON_INACTIVE_CHANNELS |
		VC4_SET_FIELD(0xf, VC4_HDMI_AUDIO_PACKET_B_FRAME_IDENTIFIER);

	/* XXX: Better CEA channel mask handling */
	channel_mask = BIT(0) | BIT(1);
	audio_packet_config |= VC4_SET_FIELD(channel_mask,
					     VC4_HDMI_AUDIO_PACKET_CEA_MASK);

	/* Set the MAI threshold.  This logic mimics the firmware's. */
	if (hdmi->audio_params.sample_rate > 96000) {
		HD_WRITE(VC4_HD_MAI_THR,
			 VC4_SET_FIELD(0x12, VC4_HD_MAI_THR_DREQHIGH) |
			 VC4_SET_FIELD(0x12, VC4_HD_MAI_THR_DREQLOW));
	} else if (hdmi->audio_params.sample_rate > 48000) {
		HD_WRITE(VC4_HD_MAI_THR,
			 VC4_SET_FIELD(0x14, VC4_HD_MAI_THR_DREQHIGH) |
			 VC4_SET_FIELD(0x12, VC4_HD_MAI_THR_DREQLOW));
	} else {
		HD_WRITE(VC4_HD_MAI_THR,
			 VC4_SET_FIELD(0x1, VC4_HD_MAI_THR_PANICHIGH) |
			 VC4_SET_FIELD(0x1, VC4_HD_MAI_THR_PANICLOW) |
			 VC4_SET_FIELD(0x1, VC4_HD_MAI_THR_DREQHIGH) |
			 VC4_SET_FIELD(0x1, VC4_HD_MAI_THR_DREQLOW));
	}

	HDMI_WRITE(VC4_HDMI_MAI_CONFIG, VC4_HDMI_MAI_CONFIG_BIT_REVERSE);

	channel_map = 0;
	channel_map_next = 0;
	for (i = 0; i < 8; i++) {
		if (channel_mask & BIT(0))
			channel_map |= i << (3 * channel_map_next++);
	}
	HDMI_WRITE(VC4_HDMI_MAI_CHANNEL_MAP, channel_map);

	HDMI_WRITE(VC4_HDMI_MAI_CONFIG, VC4_HDMI_MAI_CONFIG_BIT_REVERSE);
	HDMI_WRITE(VC4_HDMI_AUDIO_PACKET_CONFIG, audio_packet_config);

	vc4_hdmi_set_n_cts(hdmi);

	HD_WRITE(VC4_HD_MAI_CTL,
		 VC4_SET_FIELD(2 /* XXX */, VC4_HD_MAI_CTL_CHNUM) |
		 VC4_HD_MAI_CTL_ENABLE);

	vc4_hdmi_set_audio_infoframe(encoder);

	return 0;
}

static void vc4_hdmi_audio_shutdown(struct device *dev, void *data)
{
	struct vc4_hdmi *hdmi = dev_get_drvdata(dev);
	struct drm_encoder *encoder = hdmi->encoder;
	struct drm_device *drm = encoder->dev;
	struct vc4_dev *vc4 = to_vc4_dev(drm);
	int ret;

	dev_info(dev, "%s\n", __func__);

	ret = vc4_hdmi_stop_packet(encoder, HDMI_INFOFRAME_TYPE_AUDIO);
	if (ret)
		dev_err(dev, "Failed to stop audio infoframe: %d\n", ret);

	HD_WRITE(VC4_HD_MAI_CTL, VC4_HD_MAI_CTL_FLUSH | VC4_HD_MAI_CTL_ENABLE);
	HD_WRITE(VC4_HD_MAI_CTL, VC4_HD_MAI_CTL_RESET);
	HD_WRITE(VC4_HD_MAI_CTL, VC4_HD_MAI_CTL_ERRORF);
}

static int
vc4_hdmi_audio_digital_mute(struct device *dev, void *data, bool enable)
{
	dev_dbg(dev, "%s(%d)\n", __func__, enable);

	/* XXX: What to do here? */

	return 0;
}

static int vc4_hdmi_audio_get_eld(struct device *dev, void *data,
				  uint8_t *buf, size_t len)
{
	struct vc4_hdmi *hdmi = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	memcpy(buf, hdmi->connector->eld,
	       min(sizeof(hdmi->connector->eld), len));

	return 0;
}

static const struct hdmi_codec_ops vc4_hdmi_audio_codec_ops = {
	.hw_params = vc4_hdmi_audio_hw_params,
	.audio_startup = vc4_hdmi_audio_startup,
	.audio_shutdown = vc4_hdmi_audio_shutdown,
	.digital_mute = vc4_hdmi_audio_digital_mute,
	.get_eld = vc4_hdmi_audio_get_eld,
};

/*
 * Registers the HDMI audio codec driver as a child of our device.
 */
static void vc4_hdmi_register_audio_driver(struct device *dev)
{
	struct platform_device *pdev;
	const struct hdmi_codec_pdata codec_data = {
		.ops = &vc4_hdmi_audio_codec_ops,
		.spdif = 1,
	};

	pdev = platform_device_register_data(dev, HDMI_CODEC_DRV_NAME,
					     PLATFORM_DEVID_AUTO, &codec_data,
					     sizeof(codec_data));
	if (IS_ERR(pdev))
		return;

	DRM_INFO("%s driver bound to HDMI\n", HDMI_CODEC_DRV_NAME);
}

static int vc4_hdmi_bind(struct device *dev, struct device *master, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *drm = dev_get_drvdata(master);
	struct vc4_dev *vc4 = drm->dev_private;
	struct vc4_hdmi *hdmi;
	struct vc4_hdmi_encoder *vc4_hdmi_encoder;
	struct device_node *ddc_node;
	u32 value;
	int ret;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;
	dev_set_drvdata(dev, hdmi);

	vc4_hdmi_encoder = devm_kzalloc(dev, sizeof(*vc4_hdmi_encoder),
					GFP_KERNEL);
	if (!vc4_hdmi_encoder)
		return -ENOMEM;
	vc4_hdmi_encoder->base.type = VC4_ENCODER_TYPE_HDMI;
	hdmi->encoder = &vc4_hdmi_encoder->base.base;

	hdmi->pdev = pdev;
	hdmi->hdmicore_regs = vc4_ioremap_regs(pdev, 0);
	if (IS_ERR(hdmi->hdmicore_regs))
		return PTR_ERR(hdmi->hdmicore_regs);

	hdmi->hd_regs = vc4_ioremap_regs(pdev, 1);
	if (IS_ERR(hdmi->hd_regs))
		return PTR_ERR(hdmi->hd_regs);

	hdmi->pixel_clock = devm_clk_get(dev, "pixel");
	if (IS_ERR(hdmi->pixel_clock)) {
		DRM_ERROR("Failed to get pixel clock\n");
		return PTR_ERR(hdmi->pixel_clock);
	}
	hdmi->hsm_clock = devm_clk_get(dev, "hdmi");
	if (IS_ERR(hdmi->hsm_clock)) {
		DRM_ERROR("Failed to get HDMI state machine clock\n");
		return PTR_ERR(hdmi->hsm_clock);
	}

	ddc_node = of_parse_phandle(dev->of_node, "ddc", 0);
	if (!ddc_node) {
		DRM_ERROR("Failed to find ddc node in device tree\n");
		return -ENODEV;
	}

	hdmi->ddc = of_find_i2c_adapter_by_node(ddc_node);
	of_node_put(ddc_node);
	if (!hdmi->ddc) {
		DRM_DEBUG("Failed to get ddc i2c adapter by node\n");
		return -EPROBE_DEFER;
	}

	/* Enable the clocks at startup.  We can't quite recover from
	 * turning off the pixel clock during disable/enables yet, so
	 * it's always running.
	 */
	ret = clk_prepare_enable(hdmi->pixel_clock);
	if (ret) {
		DRM_ERROR("Failed to turn on pixel clock: %d\n", ret);
		goto err_put_i2c;
	}

	/* This is the rate that is set by the firmware.  The number
	 * needs to be a bit higher than the pixel clock rate
	 * (generally 148.5Mhz).
	 */
	ret = clk_set_rate(hdmi->hsm_clock, 163682864);
	if (ret) {
		DRM_ERROR("Failed to set HSM clock rate: %d\n", ret);
		goto err_unprepare_pix;
	}

	ret = clk_prepare_enable(hdmi->hsm_clock);
	if (ret) {
		DRM_ERROR("Failed to turn on HDMI state machine clock: %d\n",
			  ret);
		goto err_unprepare_pix;
	}

	/* Only use the GPIO HPD pin if present in the DT, otherwise
	 * we'll use the HDMI core's register.
	 */
	if (of_find_property(dev->of_node, "hpd-gpios", &value)) {
		enum of_gpio_flags hpd_gpio_flags;

		hdmi->hpd_gpio = of_get_named_gpio_flags(dev->of_node,
							 "hpd-gpios", 0,
							 &hpd_gpio_flags);
		if (hdmi->hpd_gpio < 0) {
			ret = hdmi->hpd_gpio;
			goto err_unprepare_hsm;
		}

		hdmi->hpd_active_low = hpd_gpio_flags & OF_GPIO_ACTIVE_LOW;
	}

	vc4->hdmi = hdmi;

	/* HDMI core must be enabled. */
	if (!(HD_READ(VC4_HD_M_CTL) & VC4_HD_M_ENABLE)) {
		HD_WRITE(VC4_HD_M_CTL, VC4_HD_M_SW_RST);
		udelay(1);
		HD_WRITE(VC4_HD_M_CTL, 0);

		HD_WRITE(VC4_HD_M_CTL, VC4_HD_M_ENABLE);

		HDMI_WRITE(VC4_HDMI_SW_RESET_CONTROL,
			   VC4_HDMI_SW_RESET_HDMI |
			   VC4_HDMI_SW_RESET_FORMAT_DETECT);

		HDMI_WRITE(VC4_HDMI_SW_RESET_CONTROL, 0);

		/* PHY should be in reset, like
		 * vc4_hdmi_encoder_disable() does.
		 */
		HDMI_WRITE(VC4_HDMI_TX_PHY_RESET_CTL, 0xf << 16);
	}

	drm_encoder_init(drm, hdmi->encoder, &vc4_hdmi_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, NULL);
	drm_encoder_helper_add(hdmi->encoder, &vc4_hdmi_encoder_helper_funcs);

	hdmi->connector = vc4_hdmi_connector_init(drm, hdmi->encoder);
	if (IS_ERR(hdmi->connector)) {
		ret = PTR_ERR(hdmi->connector);
		goto err_destroy_encoder;
	}

	vc4_hdmi_register_audio_driver(dev);

	return 0;

err_destroy_encoder:
	vc4_hdmi_encoder_destroy(hdmi->encoder);
err_unprepare_hsm:
	clk_disable_unprepare(hdmi->hsm_clock);
err_unprepare_pix:
	clk_disable_unprepare(hdmi->pixel_clock);
err_put_i2c:
	put_device(&hdmi->ddc->dev);

	return ret;
}

static void vc4_hdmi_unbind(struct device *dev, struct device *master,
			    void *data)
{
	struct drm_device *drm = dev_get_drvdata(master);
	struct vc4_dev *vc4 = drm->dev_private;
	struct vc4_hdmi *hdmi = vc4->hdmi;

	vc4_hdmi_connector_destroy(hdmi->connector);
	vc4_hdmi_encoder_destroy(hdmi->encoder);

	clk_disable_unprepare(hdmi->pixel_clock);
	clk_disable_unprepare(hdmi->hsm_clock);
	put_device(&hdmi->ddc->dev);

	vc4->hdmi = NULL;
}

static const struct component_ops vc4_hdmi_ops = {
	.bind   = vc4_hdmi_bind,
	.unbind = vc4_hdmi_unbind,
};

static int vc4_hdmi_dev_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &vc4_hdmi_ops);
}

static int vc4_hdmi_dev_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &vc4_hdmi_ops);
	return 0;
}

static const struct of_device_id vc4_hdmi_dt_match[] = {
	{ .compatible = "brcm,bcm2835-hdmi" },
	{}
};

struct platform_driver vc4_hdmi_driver = {
	.probe = vc4_hdmi_dev_probe,
	.remove = vc4_hdmi_dev_remove,
	.driver = {
		.name = "vc4_hdmi",
		.of_match_table = vc4_hdmi_dt_match,
	},
};
