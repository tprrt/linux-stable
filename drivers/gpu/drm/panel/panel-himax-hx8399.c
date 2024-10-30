// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for panels based on Himax HX8399 controller, such as:
 *
 * - YOURITech ET055WU01-TT 5.5" MIPI-DSI panel
 *
 * Copyright (C) 2024 Thomas Perrot
 *
 * Based on drivers/gpu/drm/panel/panel-himax-hx8394.c
 * Copyright (C) 2021 Kamil Trzciński
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/media-bus-format.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#define DRV_NAME "panel-himax-hx8399"

/* Manufacturer specific commands sent via DSI, listed in HX8399-C datasheet */
#define HX8399_CMD_SETPOWER	  0xb1
#define HX8399_CMD_SETDISP	  0xb2
#define HX8399_CMD_SETCYC	  0xb4
#define HX8399_CMD_SETVCOM	  0xb6
#define HX8399_CMD_SETTE	  0xb7
#define HX8399_CMD_SETEXTC	  0xb9
#define HX8399_CMD_SETMIPI	  0xba
#define HX8399_CMD_SETOTP	  0xbb
#define HX8399_CMD_SETREGBANK	  0xbd
#define HX8399_CMD_SETDGCLUT	  0xc1
#define HX8399_CMD_SETDISMO	  0xc2
#define HX8399_CMD_SETID	  0xc3
#define HX8399_CMD_SETDDB	  0xc4
#define HX8399_CMD_SETCABC	  0xc9
#define HX8399_CMD_SETCLOCK	  0xcb
#define HX8399_CMD_SETPANEL	  0xcc
#define HX8399_CMD_SETOFFSET	  0xd2
#define HX8399_CMD_SETGIP0	  0xd3
#define HX8399_CMD_SETGIP1	  0xd5
#define HX8399_CMD_SETGIP2	  0xd6
#define HX8399_CMD_SETGIP3	  0xd8
#define HX8399_CMD_SETGPO	  0xd9
#define HX8399_CMD_SETSCALING	  0xdd
#define HX8399_CMD_SETDGCLUT_N	  0xde
#define HX8399_CMD_SETIDLE	  0xdf
#define HX8399_CMD_SETGAMMA	  0xe0
#define HX8399_CMD_SETCHEMODE_DYN 0xe4
#define HX8399_CMD_SET_I2C_SA	  0xe8
#define HX8399_CMD_SET_SP_CMD	  0xe9
#define HX8399_CMD_SETCNCD	  0xfd
#define HX8399_CMD_GETCNCD	  0xfd
#define HX8399_CMD_SETREADINDEX	  0xfe
#define HX8399_CMD_GETSPIREAD	  0xff

struct hx8399 {
	struct device *dev;
	struct drm_panel panel;
	struct gpio_desc *reset_gpio;
	struct regulator *vcc;
	bool prepared;

	const struct hx8399_panel_desc *desc;
};

struct hx8399_panel_desc {
	const struct drm_display_mode *mode;
	unsigned int lanes;
	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;
	int (*init_sequence)(struct hx8399 *ctx);
};

static inline struct hx8399 *panel_to_hx8399(struct drm_panel *panel)
{
	return container_of(panel, struct hx8399, panel);
}

static int et055wu01_init_sequence(struct hx8399 *ctx)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);

	/* 6.3.6 SETEXTC: Set extension command (B9h) */
	mipi_dsi_dcs_write_seq(dsi, HX8399_CMD_SETEXTC,
			       0xff, 0x83, 0x99);

	/* 6.3.17 SETOFFSET (D2h) */
	mipi_dsi_dcs_write_seq(dsi, HX8399_CMD_SETOFFSET,
			       0x77);

	/* 6.3.1 SETPOWER: Set power (B1h) */
	mipi_dsi_dcs_write_seq(dsi, HX8399_CMD_SETPOWER,
			       0x02, 0x04, 0x54, 0x94, 0x07, 0x32, 0x33, 0x12,
			       0x12, 0xAB, 0x4D, 0x56, 0x73, 0x02, 0x02);

	/* 6.3.1 SETDISP: Set display related register (B2h) */
	mipi_dsi_dcs_write_seq(dsi, HX8399_CMD_SETDISP,
			       0x42, 0x80, 0x80, 0xAE, 0x05, 0x07, 0x5A, 0x11,
			       0x00, 0x00, 0x10, 0x1e, 0x70, 0x03, 0xD4);

	/* 6.3.3 SETCYC: Set display waveform cycles (B4h) */
	mipi_dsi_dcs_write_seq(dsi, HX8399_CMD_SETCYC,
			       0x00, 0xFF, 0x02, 0xC0, 0x02, 0xC0, 0x00, 0x00,
			       0x08, 0x00, 0x04, 0x06, 0x00, 0x32, 0x04, 0x0A,
			       0x08, 0x21, 0x03, 0x01, 0x00, 0x0F, 0xB8, 0x8B,
			       0x02, 0xC0, 0x02, 0xC0, 0x00, 0x00, 0x08, 0x00,
			       0x04, 0x06, 0x00, 0x32, 0x04, 0x0A, 0x08, 0x01,
			       0x00, 0x0F, 0xB8, 0x01);

	/* 6.13.18 SETGIP0: Set GIP Option0 (D3h) */
	mipi_dsi_dcs_write_seq(dsi, HX8399_CMD_SETGIP0,
			       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00,
			       0x00, 0x10, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00,
			       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
			       0x00, 0x05, 0x05, 0x07, 0x00, 0x00, 0x00, 0x05,
			       0x40);


	/* 6.13.19 Set GIP Option1 (D5h) */
	mipi_dsi_dcs_write_seq(dsi, HX8399_CMD_SETGIP1,
			       0x18, 0x18, 0x19, 0x19, 0x18, 0x18, 0x21, 0x20,
			       0x01, 0x00, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02,
			       0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x2F, 0x2F,
			       0x30, 0x30, 0x31, 0x31, 0x18, 0x18, 0x18, 0x18);


	/* 6.13.20 Set GIP Option2 (D6h) */
	mipi_dsi_dcs_write_seq(dsi, HX8399_CMD_SETGIP2,
			       0x18, 0x18, 0x19, 0x19, 0x40, 0x40, 0x20, 0x21,
			       0x06, 0x07, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
			       0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x2F, 0x2F,
			       0x30, 0x30, 0x31, 0x31, 0x40, 0x40, 0x40,0x40);

	/* 6.13.21 Set GIP Option3 (D8h) */
	mipi_dsi_dcs_write_seq(dsi, HX8399_CMD_SETGIP3,
			       0xA2, 0xAA, 0x02, 0xA0, 0xA2, 0xA8, 0x02, 0xA0,
			       0xB0, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x00, 0x00);

	/* 6.13.21 Set GIP Option3 (D8h) */
	mipi_dsi_dcs_write_seq(dsi, HX8399_CMD_SETGIP3,
			       0xB0, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x00, 0x00,
			       0xE2, 0xAA, 0x03, 0xF0, 0xE2, 0xAA, 0x03, 0xF0);

	/* 6.13.21 Set GIP Option3 (D8h) */
	mipi_dsi_dcs_write_seq(dsi, HX8399_CMD_SETGIP3,
			       0xE2, 0xAA, 0x03, 0xF0, 0xE2, 0xAA, 0x03, 0xF0);

	/* 6.3.9 Set register bank (BDh) */
	mipi_dsi_dcs_write_seq(dsi, HX8399_CMD_SETREGBANK,
			       0x00);

	/* 6.3.4 SETVCOM: Set VCOM voltage (B6h) */
	mipi_dsi_dcs_write_seq(dsi, HX8399_CMD_SETVCOM,
			       0x85, 0x85);

	/* 6.3.26 SETGAMMA: Set gamma curve related setting (E0h) */
	mipi_dsi_dcs_write_seq(dsi, HX8399_CMD_SETGAMMA,
			       0x00, 0x17, 0x26, 0x23, 0x55, 0x61, 0x71, 0x6F,
			       0x78, 0x82, 0x89, 0x8F, 0x94, 0x9D, 0xA5, 0xA9,
			       0xAE, 0xB7, 0xBA, 0xC2, 0xB6, 0xC2, 0xC3, 0x64,
			       0x5F, 0x69, 0x77, 0x00, 0x17, 0x26, 0x23, 0x55,
			       0x61, 0x71, 0x6F, 0x78, 0x82, 0x89, 0x8F, 0x94,
			       0x9D, 0xA5, 0xA9, 0xAE, 0xB7, 0xBA, 0xC2, 0xB6,
			       0xC2, 0xC4, 0x64, 0x5F, 0x69, 0x77);

	/* 6.3.16 SETPANEL (CCh) */
	mipi_dsi_dcs_write_seq(dsi, HX8399_CMD_SETPANEL,
			       0x08);

	return 0;
}

static const struct drm_display_mode et055wu01_mode = {
	.hdisplay    = 1080,
	.hsync_start = 1080 + 20,
	.hsync_end   = 1080 + 20 + 10,
	.htotal	     = 1080 + 20 + 10 + 30,
	.vdisplay    = 1980,
	.vsync_start = 1980 + 15,
	.vsync_end   = 1980 + 15 + 4,
	.vtotal	     = 1980 + 15 + 4 + 8,
	.clock	     = 133330,
	.flags	     = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
	.width_mm    = 68,
	.height_mm   = 120,
};

static const struct hx8399_panel_desc et055wu01_desc = {
	.mode = &et055wu01_mode,
	.lanes = 4,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST,
	.format = MIPI_DSI_FMT_RGB888,
	.init_sequence = et055wu01_init_sequence,
};

static int hx8399_enable(struct drm_panel *panel)
{
	struct hx8399 *ctx = panel_to_hx8399(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	ret = ctx->desc->init_sequence(ctx);
	if (ret) {
		dev_err(ctx->dev, "Panel init sequence failed: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret) {
		dev_err(ctx->dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}

	/* Panel is operational 120 msec after reset */
	msleep(120);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret) {
		dev_err(ctx->dev, "Failed to turn on the display: %d\n", ret);
		goto sleep_in;
	}

	return 0;

sleep_in:
	/* This will probably fail, but let's try orderly power off anyway. */
	if (!mipi_dsi_dcs_enter_sleep_mode(dsi))
		msleep(50);

	return ret;
}

static int hx8399_disable(struct drm_panel *panel)
{
	struct hx8399 *ctx = panel_to_hx8399(panel);
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret) {
		dev_err(ctx->dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}

	msleep(50); /* about 3 frames */

	return 0;
}

static int hx8399_unprepare(struct drm_panel *panel)
{
	struct hx8399 *ctx = panel_to_hx8399(panel);

	if (!ctx->prepared)
		return 0;

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	regulator_disable(ctx->vcc);

	ctx->prepared = false;

	return 0;
}

static int hx8399_prepare(struct drm_panel *panel)
{
	struct hx8399 *ctx = panel_to_hx8399(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	ret = regulator_enable(ctx->vcc);
	if (ret) {
		dev_err(ctx->dev, "Failed to enable vcc supply: %d\n", ret);
		return ret;
	}

	msleep(3); // > 1ms + 1ms + 10µs
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);

	msleep(180);

	ctx->prepared = true;

	return 0;
}

static int hx8399_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	struct hx8399 *ctx = panel_to_hx8399(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, ctx->desc->mode);
	if (!mode) {
		dev_err(ctx->dev, "Failed to add mode %ux%u@%u\n",
			ctx->desc->mode->hdisplay, ctx->desc->mode->vdisplay,
			drm_mode_vrefresh(ctx->desc->mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs hx8399_drm_funcs = {
	.disable   = hx8399_disable,
	.unprepare = hx8399_unprepare,
	.prepare   = hx8399_prepare,
	.enable	   = hx8399_enable,
	.get_modes = hx8399_get_modes,
};

static int hx8399_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct hx8399 *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset gpio\n");

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	ctx->desc = of_device_get_match_data(dev);

	dsi->mode_flags = ctx->desc->mode_flags;
	dsi->format = ctx->desc->format;
	dsi->lanes = ctx->desc->lanes;

	ctx->vcc = devm_regulator_get(dev, "vcc");
	if (IS_ERR(ctx->vcc))
		return dev_err_probe(dev, PTR_ERR(ctx->vcc),
				     "Failed to request vcc regulator\n");

	drm_panel_init(&ctx->panel, dev, &hx8399_drm_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err_probe(dev, ret, "mipi_dsi_attach failed\n");
		drm_panel_remove(&ctx->panel);
		return ret;
	}

	dev_dbg(dev, "%ux%u@%u %ubpp dsi %udl - ready\n",
		ctx->desc->mode->hdisplay, ctx->desc->mode->vdisplay,
		drm_mode_vrefresh(ctx->desc->mode),
		mipi_dsi_pixel_format_to_bpp(dsi->format), dsi->lanes);

	return 0;
}

static void hx8399_shutdown(struct mipi_dsi_device *dsi)
{
	struct hx8399 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = drm_panel_disable(&ctx->panel);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to disable panel: %d\n", ret);

	ret = drm_panel_unprepare(&ctx->panel);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to unprepare panel: %d\n", ret);
}

static void hx8399_remove(struct mipi_dsi_device *dsi)
{
	struct hx8399 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	hx8399_shutdown(dsi);

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id hx8399_of_match[] = {
	{ .compatible = "youritech,et055wu01-tt", .data = &et055wu01_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hx8399_of_match);

static struct mipi_dsi_driver hx8399_driver = {
	.probe	= hx8399_probe,
	.remove = hx8399_remove,
	.shutdown = hx8399_shutdown,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = hx8399_of_match,
	},
};
module_mipi_dsi_driver(hx8399_driver);

MODULE_AUTHOR("Thomas Perrot <thomas.perrot@bootlin.com>");
MODULE_DESCRIPTION("DRM driver for Himax HX8399 based MIPI DSI panels");
MODULE_LICENSE("GPL");
