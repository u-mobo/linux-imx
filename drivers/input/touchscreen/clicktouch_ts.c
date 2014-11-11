/* Source for:
 * Clicktouch Prototype touchscreen driver.
 * drivers/input/touchscreen/clicktouch_ts.c
 *
 * Copyright (C) 2012, Clicktouch NV. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Clicktouch reserves the right to make changes without further notice
 * to the materials described herein. Clicktouch does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Clicktouch Semiconductor at www.Clicktouch.eu
 *
 */
//#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
//#include <linux/input/clicktouch_ts.h>
#include <linux/pm.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define CLICKTOUCH_TS_MAX_FINGERS	(10)
#define CLICKTOUCH_TS_SCANTIME_MS	(20)
#define CLICKTOUCH_TS_PRESSURE_MIN	(0)

#define CLICKTOUCH_TS_TOUCHES_MASK	(0x1f)
#define CLICKTOUCH_TS_TOUCHID_MASK	(0x0f)
#define CLICKTOUCH_TS_INVALID_DATA	(0xff)

#define CLICKTOUCH_TS_SWAP_XY	(1 << 0)
#define CLICKTOUCH_TS_INVERT_X	(1 << 1)
#define CLICKTOUCH_TS_INVERT_Y	(1 << 2)

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define CLICKTOUCH_TS_SUSPEND_LEVEL 1
#endif

#define BB2W(x) (x[0] << 8 | x[1])

#undef USE_SMBUS

static int multitouch = 1;
module_param(multitouch, int, 0644);
MODULE_PARM_DESC(multitouch, "enable multitouch events");

struct clicktouch_ts_static_data {
	u8 hst_mode;
	u8 reserved1;
	u8 command[9];
	u8 firmware_revision;
	u8 device_type;
	u8 reserved2[2];
	u8 x_count;
	u8 y_count;
	u8 max_x[2];
	u8 max_y[2];
	u8 max_fingers;
	u8 max_buttons;
	u8 button_offset;
	u8 xy_offset;
	u8 report_length;
	u8 report_stat;
};

struct clicktouch_ts_touch_record {
	u8 x[2];
	u8 y[2];
	u8 z;
	u8 stat;
};

struct clicktouch_ts_touch_data {
	u8 stat;	
	struct clicktouch_ts_touch_record record[CLICKTOUCH_TS_MAX_FINGERS];
};

struct clicktouch_ts {
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	struct workqueue_struct *wq;
	struct clicktouch_ts_platform_data *platform_data;
	struct clicktouch_ts_static_data static_data;
	struct clicktouch_ts_touch_data touch_data;
	u16 touch_data_length;
	u8 previous_touches;
	u16 max_x;
	u16 max_y;
	u8 invert_x;
	u8 invert_y;
	u8 swap_xy;
	u32 scantime_jiffies;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend		early_suspend;
#endif
};

#ifdef NEEDED
static s32 clicktouch_ts_write_reg_u8(struct i2c_client *client, u8 reg, u8 val)
{
#ifdef USE_SMBUS
	s32 data;

	data = i2c_smbus_write_byte_data(client, reg, val);
	if (data < 0)
		dev_err(&client->dev, "error %d in writing reg 0x%x\n",
						 data, reg);

	return data;
#else
	struct i2c_msg msg;
	u8 buf[2];

	buf[0] = reg;
	buf[1] = val;

	msg.addr = client->addr;
	msg.len = 2;
	msg.flags = 0;
	msg.buf = buf;

	return i2c_transfer(client->adapter, &msg, 1);

#endif
}

static s32 clicktouch_ts_read_reg_u8(struct i2c_client *client, u8 reg)
{
	s32 data;

	data = i2c_smbus_read_byte_data(client, reg);
	if (data < 0)
		dev_err(&client->dev, "error %d in reading reg 0x%x\n",
						 data, reg);

	return data;
}
#endif /* NEEDED */

static int clicktouch_ts_read(struct i2c_client *client, u8 reg, u8 *buf, int len)
{
	struct i2c_msg xfer_msg[2];

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = 1;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = &reg;

	xfer_msg[1].addr = client->addr;
	xfer_msg[1].len = len;
	xfer_msg[1].flags = I2C_M_RD;
	xfer_msg[1].buf = buf;

	return i2c_transfer(client->adapter, xfer_msg, 2);
}

static void report_data(struct clicktouch_ts *ts, u16 x, u16 y, u8 z, u8 id, u8 counter)
{
	/* handle inverting coordinates */
	if (ts->swap_xy)
		swap(x, y);

	if (ts->invert_x)
		x = ts->max_x - x;
	if (ts->invert_y)
		y = ts->max_y - y;

	if (multitouch) {
		input_report_abs(ts->input, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input, ABS_MT_POSITION_Y, y);
		input_report_abs(ts->input, ABS_MT_PRESSURE, z);
		input_report_abs(ts->input, ABS_MT_TRACKING_ID, id);
		input_mt_sync(ts->input);
	} else if (counter == 0) {
		input_report_abs(ts->input, ABS_X, x);
		input_report_abs(ts->input, ABS_Y, y);
		input_report_abs(ts->input, ABS_PRESSURE, z);
	}
}

static void process_data(struct clicktouch_ts *ts)
{
	u8 id, z, touches, invalid_touches, i;
	u16 x, y;

	touches = ts->touch_data.stat & CLICKTOUCH_TS_TOUCHES_MASK;
	invalid_touches = 0;

	for (i = 0; i < touches; i++) {
		x = BB2W(ts->touch_data.record[i].x);
		y = BB2W(ts->touch_data.record[i].y);
		z = ts->touch_data.record[i].z;
		id = ts->touch_data.record[i].stat & CLICKTOUCH_TS_TOUCHID_MASK;

		if (z > CLICKTOUCH_TS_PRESSURE_MIN)
			report_data(ts, x, y, z, id, i);
		else
			invalid_touches++;
	}

	touches -= invalid_touches;

	if (touches && !(ts->previous_touches))
		input_report_key(ts->input, BTN_TOUCH, 1);
	else if (!touches && (ts->previous_touches))
		input_report_key(ts->input, BTN_TOUCH, 0);

	input_sync(ts->input);

	ts->previous_touches = touches;
}

static void clicktouch_ts_xy_worker(struct work_struct *work)
{
	int rc;
	struct clicktouch_ts *ts = container_of(work, struct clicktouch_ts,
				 work.work);

	/* read touch data */
	rc = clicktouch_ts_read(ts->client, ts->static_data.xy_offset,
			(u8*)&(ts->touch_data), ts->touch_data_length);
	if (rc < 0) {
		dev_err(&ts->client->dev, "touch data read failed\n");
		goto schedule;
	}

	if (ts->touch_data.stat == CLICKTOUCH_TS_INVALID_DATA)
		goto schedule;

	process_data(ts);

schedule:
	if (ts->client->irq)
		enable_irq(ts->client->irq);
	else
		queue_delayed_work(ts->wq, &ts->work, ts->scantime_jiffies);
}

static irqreturn_t clicktouch_ts_irq(int irq, void *dev_id)
{
	struct clicktouch_ts *ts = dev_id;

	disable_irq_nosync(irq);

	queue_delayed_work(ts->wq, &ts->work, 0);

	return IRQ_HANDLED;
}

static int clicktouch_ts_open(struct input_dev *dev)
{
	int rc = 0;
	struct clicktouch_ts *ts = input_get_drvdata(dev);

	if (ts->client->irq) {
		rc = request_irq(ts->client->irq, clicktouch_ts_irq,
					IRQF_TRIGGER_FALLING,
					ts->client->dev.driver->name, ts);
		if (rc)
			dev_err(&ts->client->dev, "could not request irq\n");
	} else {
		queue_delayed_work(ts->wq, &ts->work, ts->scantime_jiffies);
	}

	return rc;
}

static void clicktouch_ts_close(struct input_dev *dev)
{
	int rc;
	struct clicktouch_ts *ts = input_get_drvdata(dev);

	rc = cancel_delayed_work_sync(&ts->work);

	if (ts->client->irq)
		free_irq(ts->client->irq, ts);
}

static int clicktouch_ts_init(struct i2c_client *client, struct clicktouch_ts *ts)
{
	struct device_node *of_node = client->dev.of_node;
	struct input_dev *input_device;
	int rc;
	u32 flags;

	/* read touch data */
	rc = clicktouch_ts_read(ts->client, 0, (u8*)&(ts->static_data), sizeof(ts->static_data));
	if (rc < 0) {
		dev_err(&ts->client->dev, "static data read failed\n");
		return rc;
	}
	ts->touch_data_length = 1 + 6 * ts->static_data.max_fingers;

	ts->max_x = BB2W(ts->static_data.max_x);
	ts->max_y = BB2W(ts->static_data.max_y);

	if (of_property_read_u32(of_node, "flags", &flags) == 0) {
		dev_dbg(&ts->client->dev, "device tree data available\n");
		ts->invert_x = flags & CLICKTOUCH_TS_INVERT_X;
		ts->invert_y = flags & CLICKTOUCH_TS_INVERT_Y;
		ts->swap_xy = flags & CLICKTOUCH_TS_SWAP_XY;
	} else if (ts->platform_data) {
		flags = (u32)ts->platform_data;
		dev_dbg(&ts->client->dev, "platform data available\n");
		ts->invert_x = flags & CLICKTOUCH_TS_INVERT_X;
		ts->invert_y = flags & CLICKTOUCH_TS_INVERT_Y;
		ts->swap_xy = flags & CLICKTOUCH_TS_SWAP_XY;
	} else {
		dev_dbg(&ts->client->dev, "platform data unavailable\n");
		ts->invert_x = 1;
		ts->invert_y = 0;
		ts->swap_xy = 1;
	}

	if (ts->swap_xy)
		swap(ts->max_x, ts->max_y);

	dev_dbg(&ts->client->dev, "swap_xy %d\n", ts->swap_xy);
	dev_dbg(&ts->client->dev, "invert_x %d\n", ts->invert_x);
	dev_dbg(&ts->client->dev, "invert_y %d\n", ts->invert_y);

	dev_dbg(&ts->client->dev, "hst_mode %d\n", ts->static_data.hst_mode);
	dev_dbg(&ts->client->dev, "firmware_revision %d\n", ts->static_data.firmware_revision);
	dev_dbg(&ts->client->dev, "device_type %d\n", ts->static_data.device_type);
	dev_dbg(&ts->client->dev, "x_count %d\n", ts->static_data.x_count);
	dev_dbg(&ts->client->dev, "y_count %d\n", ts->static_data.y_count);
	dev_dbg(&ts->client->dev, "max_x %d\n", ts->max_x);
	dev_dbg(&ts->client->dev, "max_y %d\n", ts->max_y);
	dev_dbg(&ts->client->dev, "max_fingers %d\n", ts->static_data.max_fingers);
	dev_dbg(&ts->client->dev, "max_buttons %d\n", ts->static_data.max_buttons);
	dev_dbg(&ts->client->dev, "button_offset %d\n", ts->static_data.button_offset);
	dev_dbg(&ts->client->dev, "xy_offset %d\n", ts->static_data.xy_offset);
	dev_dbg(&ts->client->dev, "report_length %d\n", ts->static_data.report_length);
	dev_dbg(&ts->client->dev, "report_stat %d\n", ts->static_data.report_stat);

	ts->scantime_jiffies = msecs_to_jiffies(CLICKTOUCH_TS_SCANTIME_MS);

	ts->previous_touches = 0;

	input_device = input_allocate_device();
	if (!input_device) {
		rc = -ENOMEM;
		goto error_alloc_dev;
	}

	ts->input = input_device;
	input_device->name = "ClickTouch Touchscreen";
	input_device->id.bustype = BUS_I2C;
	input_device->dev.parent = &client->dev;
	input_set_drvdata(input_device, ts);

	__set_bit(EV_ABS, input_device->evbit);

	/* multi touch capability */
	input_set_abs_params(input_device, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);
	input_set_abs_params(input_device, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_device, ABS_MT_TRACKING_ID, 0,
			CLICKTOUCH_TS_MAX_FINGERS - 1, 0, 0);

	/* single touch capability */
	input_set_abs_params(input_device, ABS_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(input_device, ABS_Y, 0, ts->max_y, 0, 0);

	input_set_abs_params(input_device, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_capability(input_device, EV_KEY, BTN_TOUCH);

	ts->wq = create_singlethread_workqueue("clicktouch_kworkqueue");
	if (!ts->wq) {
		dev_err(&client->dev, "Could not create workqueue\n");
		goto error_wq_create;
	}

	INIT_DELAYED_WORK(&ts->work, clicktouch_ts_xy_worker);

	input_device->open = clicktouch_ts_open;
	input_device->close = clicktouch_ts_close;

	rc = input_register_device(input_device);
	if (rc)
		goto error_unreg_device;

	return 0;

error_unreg_device:
	destroy_workqueue(ts->wq);
error_wq_create:
	input_free_device(input_device);
error_alloc_dev:
	return rc;
}

#ifdef CONFIG_PM
static int clicktouch_ts_suspend(struct device *dev)
{
	struct clicktouch_ts *ts = dev_get_drvdata(dev);
	int rc = 0;

	if (ts->client->irq)
		disable_irq_nosync(ts->client->irq);

	rc = cancel_delayed_work_sync(&ts->work);

	return 0;
}

static int clicktouch_ts_resume(struct device *dev)
{
	struct clicktouch_ts *ts = dev_get_drvdata(dev);

	if (ts->client->irq)
		enable_irq(ts->client->irq);
	else
		queue_delayed_work(ts->wq, &ts->work, ts->scantime_jiffies);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void clicktouch_ts_early_suspend(struct early_suspend *h)
{
	struct clicktouch_ts *ts = container_of(h, struct clicktouch_ts, early_suspend);

	clicktouch_ts_suspend(&ts->client->dev);
}

static void clicktouch_ts_late_resume(struct early_suspend *h)
{
	struct clicktouch_ts *ts = container_of(h, struct clicktouch_ts, early_suspend);

	clicktouch_ts_resume(&ts->client->dev);
}
#endif

static struct dev_pm_ops clicktouch_ts_pm_ops = {
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= clicktouch_ts_suspend,
	.resume		= clicktouch_ts_resume,
#endif
};
#endif

static int clicktouch_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct clicktouch_ts *ts;
	struct clicktouch_ts_platform_data *pdata = client->dev.platform_data;
	int rc;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA)) {
		dev_err(&client->dev, "I2C functionality not supported\n");
		return -EIO;
	}


	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	ts->platform_data = pdata;
	i2c_set_clientdata(client, ts);

	rc = clicktouch_ts_init(client, ts);
	if (rc < 0) {
		dev_err(&client->dev, "Clicktouch init failed\n");
		goto error_touch_data_alloc;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						CLICKTOUCH_TS_SUSPEND_LEVEL;
	ts->early_suspend.suspend = clicktouch_ts_early_suspend;
	ts->early_suspend.resume = clicktouch_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	return 0;

error_touch_data_alloc:
	kfree(ts);
	return rc;
}

static int clicktouch_ts_remove(struct i2c_client *client)
{
	struct clicktouch_ts *ts = i2c_get_clientdata(client);
	int rc;

	rc = cancel_delayed_work_sync(&ts->work);

	if (client->irq)
		free_irq(client->irq, ts);

	destroy_workqueue(ts->wq);

	input_unregister_device(ts->input);

	kfree(ts);

	return 0;
}

static const struct i2c_device_id clicktouch_ts_id[] = {
	{"clicktouch_ts", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, clicktouch_ts_id);

static const struct of_device_id clicktouch_ts_dt_ids[] = {
	{ .compatible = "clicktouch_ts", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, clicktouch_ts_ts_dt_ids);


static struct i2c_driver clicktouch_ts_driver = {
	.driver = {
		.name = "clicktouch_ts",
		.owner = THIS_MODULE,
		.of_match_table = clicktouch_ts_dt_ids,
#ifdef CONFIG_PM
		.pm = &clicktouch_ts_pm_ops,
#endif
	},
	.probe		= clicktouch_ts_probe,
	.remove		= clicktouch_ts_remove,
	.id_table	= clicktouch_ts_id,
};

static int __init clicktouch_ts_module_init(void)
{
	return i2c_add_driver(&clicktouch_ts_driver);
}
/* Making this as late init to avoid power fluctuations
 * during LCD initialization.
 */
late_initcall(clicktouch_ts_module_init);

static void __exit clicktouch_ts_module_exit(void)
{
	return i2c_del_driver(&clicktouch_ts_driver);
}
module_exit(clicktouch_ts_module_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Clicktouch touchscreen controller driver");
MODULE_AUTHOR("Pierluigi Passaro <info@phoenixsoftware.it>");
MODULE_ALIAS("platform:clicktouch_ts");
