/*
 * Driver for memory based ft5406 touchscreen
 *
 * Copyright (C) 2015 Raspberry Pi
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/input/mt.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <soc/bcm2835/raspberrypi-firmware.h>

struct ft5406_touchpoint {
	uint8_t xh;
	uint8_t xl;
	uint8_t yh;
	uint8_t yl;
	uint8_t res1;
	uint8_t res2;
};

#define MAXIMUM_SUPPORTED_POINTS 10
struct ft5406_regs {
	uint8_t device_mode;
	uint8_t gesture_id;
	uint8_t num_points;
	struct ft5406_touchpoint point[MAXIMUM_SUPPORTED_POINTS];
};

#define SCREEN_WIDTH  800
#define SCREEN_HEIGHT 480

struct ft5406 {
	struct platform_device *pdev;
	struct i2c_client *i2c;
	struct input_dev *input_dev;
	struct task_struct *thread;
	int known_ids;
};

static void ft5406_poll_input(struct ft5406 *ts)
{
	int ret;
	int i;
	u8 num_points;
	int modified_ids = 0, released_ids;

	ret = i2c_smbus_read_i2c_block_data(ts->i2c,
					    offsetof(struct ft5406_regs, num_points),
					    sizeof(num_points),
					    &num_points);
	if (ret)
		return;

	/* If there's nothing going on, return immediately. */
	if (ts->known_ids == 0 && num_points == 0)
		return;

	for(i = 0; i < num_points; i++) {
		struct ft5406_touchpoint point;
		int x, y, touchid;

		ret = i2c_smbus_read_i2c_block_data(ts->i2c,
						    offsetof(struct ft5406_regs,
							     point[i]),
						    sizeof(point),
						    (u8 *)&point);
		if (ret)
			return;

		x = (((int) point.xh & 0xf) << 8) + point.xl;
		y = (((int) point.yh & 0xf) << 8) + point.yl;
		touchid = (point.yh >> 4) & 0xf;

		modified_ids |= 1 << touchid;

		if (!(known_ids & BIT(touchid))) {
			dev_dbg(&ts->pdev->dev,
				"x = %d, y = %d, touchid = %d\n", x, y, touchid);
		}

		input_mt_slot(ts->input_dev, touchid);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);

		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);

	}

	released_ids = ts->known_ids & ~modified_ids;
	for(i = 0; released_ids && i < MAXIMUM_SUPPORTED_POINTS; i++) {
		if (released_ids & BIT(i)) {
			dev_dbg(&ts->pdev->dev, "Released %d, known = %x modified = %x\n",
				i, ts->known_ids, modified_ids);
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
			modified_ids &= ~(1 << i);
		}
	}
	ts->known_ids = modified_ids;

	input_mt_report_pointer_emulation(ts->input_dev, true);
	input_sync(ts->input_dev);
}

/* kernel thread to poll the I2C device for touchscreen events */
static int ft5406_thread(void *arg)
{
	struct ft5406 *ts = (struct ft5406 *) arg;
	struct ft5406_regs regs;

	while(!kthread_should_stop()) {
		ft5406_poll_input(ts);
		// 60fps polling
		msleep_interruptible(17);
	}

	return 0;
}

static int ft5406_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct input_dev *input_dev;
	struct ft5406 *ts;
	int ret;

	dev_info(&pdev->dev, "Probing device\n");

	ts = devm_kzalloc(dev, sizeof(struct ft5406), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	input_dev = input_allocate_device();
	if (!input_dev)
		return -ENOMEM;

	ts->input_dev = input_dev;
	platform_set_drvdata(pdev, ts);
	ts->pdev = pdev;

	input_dev->name = "FT5406 I2C-based driver";

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
			     SCREEN_WIDTH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
			     SCREEN_HEIGHT, 0, 0);

	input_mt_init_slots(input_dev, MAXIMUM_SUPPORTED_POINTS, INPUT_MT_DIRECT);

	input_set_drvdata(input_dev, ts);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&pdev->dev, "could not register input device, %d\n",
			ret);
		return ret;
	}

	// create thread to poll the touch events
	ts->thread = kthread_run(ft5406_thread, ts, "ft5406");
	if (ts->thread == NULL) {
		dev_err(&pdev->dev, "Failed to create kernel thread");
		input_unregister_device(input_dev);
		kzfree(ts);
	}

	return 0;
}

static int ft5406_remove(struct platform_device *pdev)
{
	struct ft5406 *ts = (struct ft5406 *) platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "Removing rpi-ft5406\n");

	kthread_stop(ts->thread);
	input_unregister_device(ts->input_dev);
	kzfree(ts);

	return 0;
}

static const struct of_device_id ft5406_match[] = {
	{ .compatible = "raspberrypi,touchscreen-ft5406", },
	{},
};
MODULE_DEVICE_TABLE(of, ft5406_match);

static struct platform_driver ft5406_driver = {
	.driver = {
		.name   = "rpi-ft5406",
		.of_match_table = ft5406_match,
	},
	.probe          = ft5406_probe,
	.remove         = ft5406_remove,
};

module_platform_driver(ft5406_driver);

MODULE_AUTHOR("Eric Anholt");
MODULE_DESCRIPTION("Touchscreen driver for I2C-based FT5406");
MODULE_LICENSE("GPL");
