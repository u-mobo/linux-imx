/*
 * drivers/input/touchscreen/sx865x.c
 *
 * Copyright (c) 2013 U-MoBo Srl
 *      Pierluigi Passaro <p.passaro@u-mobo.com>
 *
 * Using code from:
 *  - sx8650.c
 *      Copyright (c) 2009 Wayne Roberts
 *  - tsc2007.c
 *      Copyright (c) 2008 Kwangwoo Lee
 *  - ads7846.c
 *      Copyright (c) 2005 David Brownell
 *      Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *      Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *      Copyright (C) 2002 MontaVista Software
 *      Copyright (C) 2004 Texas Instruments
 *      Copyright (C) 2005 Dirk Behme
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input/sx865x.h>

/* timeout expires after pen is lifted, no more PENIRQs comming */
#define TS_TIMEOUT		(8 * 1000000)	/* adjust with POWDLY setting */

/* analog channels */
#define CH_X			0
#define CH_Y			1
#define CH_Z1			2
#define CH_Z2			3
#define CH_AUX			4
#define CH_RX			5
#define CH_RY			6
#define CH_SEQ			7

/* commands */
#define SX865X_WRITE_REGISTER	0x00
#define SX865X_READ_REGISTER	0x40
#define SX865X_SELECT_CH(ch)	(0x80 | ch)
#define SX865X_CONVERT_CH(ch)	(0x90 | ch)
#define SX865X_POWDWN		0xb0	// power down, ignore pen
#define SX865X_PENDET		0xc0	// " " with pen sensitivity
#define SX865X_PENTRG		0xe0	// " " " " and sample channels

/* register addresses */
#define I2C_REG_CTRL0		0x00
#define I2C_REG_CTRL1		0x01
#define I2C_REG_CTRL2		0x02
#define I2C_REG_CTRL3		0x03
#define I2C_REG_CHANMASK	0x04
#define I2C_REG_STAT		0x05
#define I2C_REG_SOFTRESET	0x1f

#define I2C_EXTENDED_REG_STAT		0x24
#define I2C_EXTENDED_REG_SOFTRESET	0x3f

#define SOFTRESET_VALUE		0xde

/* bits for I2C_REG_STAT */
#define STATUS_CONVIRQ		0x80	// I2C_REG_STAT: end of conversion flag
#define STATUS_PENIRQ		0x40	// I2C_REG_STAT: pen detected

/* bits for I2C_EXTENDED_REG_STAT */
#define EXTENDED_STATUS_CONVIRQ	0x08	// I2C_EXTENDED_REG_STAT: end of conversion flag
#define EXTENDED_STATUS_PENIRQ	0x04	// I2C_EXTENDED_REG_STAT: pen detected

/* sx865x bits for RegCtrl1 */
#define CONDIRQ			0x20
#define FILT_NONE		0x00	/* no averaging */
#define FILT_3SA		0x01	/* 3 sample averaging */
#define FILT_5SA		0x02	/* 5 sample averaging */
#define FILT_7SA		0x03	/* 7 samples, sort, then average of 3 middle samples */

/* bits for register 2, I2CRegChanMsk */
#define CONV_X			0x80
#define CONV_Y			0x40
#define CONV_Z1			0x20
#define CONV_Z2			0x10
#define CONV_AUX		0x08
#define CONV_RX			0x04
#define CONV_RY			0x02

/* cordinantes rate: higher nibble of CTRL0 register */
#define RATE_MANUAL		0x00
#define RATE_10CPS		0x01
#define RATE_20CPS		0x02
#define RATE_40CPS		0x03
#define RATE_60CPS		0x04
#define RATE_80CPS		0x05
#define RATE_100CPS		0x06
#define RATE_200CPS		0x07
#define RATE_300CPS		0x08
#define RATE_400CPS		0x09
#define RATE_500CPS		0x0a
#define RATE_1000CPS		0x0b
#define RATE_2000CPS		0x0c
#define RATE_3000CPS		0x0d
#define RATE_4000CPS		0x0e
#define RATE_5000CPS		0x0f

/* power delay: lower nibble of CTRL0 register */
#define POWDLY_IMMEDIATE	0x00
#define POWDLY_1_1US		0x01
#define POWDLY_2_2US		0x02
#define POWDLY_4_4US		0x03
#define POWDLY_8_9US		0x04
#define POWDLY_17_8US		0x05
#define POWDLY_35_5US		0x06
#define POWDLY_71US		0x07
#define POWDLY_140US		0x08
#define POWDLY_280US		0x09
#define POWDLY_570US		0x0a
#define POWDLY_1_1MS		0x0b
#define POWDLY_2_3MS		0x0c
#define POWDLY_4_6MS		0x0d
#define POWDLY_9MS		0x0e
#define POWDLY_18MS		0x0f

#define MAX_11BIT		((1 << 11) - 1)
#define MAX_12BIT		((1 << 12) - 1)

/* when changing the channel mask, also change the read length appropriately */
#define CHAN_SINGLETOUCH_MASK	(CONV_X | CONV_Y | CONV_Z1 | CONV_Z2)
#define CHAN_MULTITOUCH_MASK	(CHAN_SINGLETOUCH_MASK | CONV_RX | CONV_RY)
#define NUM_SINGLETOUCH_CHANS	4
#define NUM_MULTITOUCH_CHANS	6
#define NUM_MAX_CHANS		NUM_MULTITOUCH_CHANS
#define CHAN_READ_LENGTH(chans)	(chans << 1)

#define SX_MULTITOUCH		0x01
#define SX_PROXIMITY_SENSING	0x02
#define SX_HAPTICS_GENERIC	0x04
#define SX_HAPTICS_IMMERSION	0x08
#define SX_EXTENDED_REGS	(SX_PROXIMITY_SENSING | SX_HAPTICS_GENERIC | SX_HAPTICS_IMMERSION)

#define SX865X_UP_SCANTIME_MS	(100)
#define SX865X_DOWN_SCANTIME_MS	(20)

struct ts_event {
	u16 x, y;
	u16 z1, z2;
	u16 rx, ry;
};

struct sx865x {
	struct input_dev *input;
	struct ts_event tc;

	struct i2c_client *client;

	struct workqueue_struct *ts_workq;
	struct delayed_work pen_event_work;

	u16 features;
	u16 invert_x;
	u16 invert_y;
	u16 swap_xy;
	u16 x_plate_ohms;
	u16 y_plate_ohms;
	u16 max_dx, min_rx;
	u16 max_dy, min_ry;
	u16 scantime_jiffies;

	unsigned pendown;
	int irq;
};

#define AVERAGE_SHIFT	2
#define AVERAGE_LEN	4	/* 1 << AVERAGE_SHIFT */
#define AVERAGE_MASK	3	/* AVERAGE_LEN - 1 */

static void sx865x_send_event(struct sx865x *ts)
{
	u32 pressure;
	u16 x, y, z1, z2;
	u16 rx, ry;
	s16 dx = 0, dy = 0;

	static u16 second_touch = 0;
	static u16 average_index = 0;
	static struct ts_event average_array[AVERAGE_LEN + 1];

	x = ts->tc.x;
	y = ts->tc.y;
	z1 = ts->tc.z1;
	z2 = ts->tc.z2;

	if (ts->features & SX_MULTITOUCH) {
		rx = ts->tc.rx;
		ry = ts->tc.ry;

		if (ts->tc.rx < ts->min_rx)
			ts->min_rx = ts->tc.rx;
		if (ts->tc.ry < ts->min_ry)
			ts->min_ry = ts->tc.ry;
		dx = rx - ts->min_rx;
		dy = ry - ts->min_ry;
		if ((dx > 8) || (dy > 8)) {
			if (second_touch) {
				average_index &= AVERAGE_MASK;
				average_array[AVERAGE_LEN].x -= average_array[average_index].x;
				average_array[AVERAGE_LEN].y -= average_array[average_index].y;
				average_array[AVERAGE_LEN].rx -= average_array[average_index].rx;
				average_array[AVERAGE_LEN].ry -= average_array[average_index].ry;
				average_array[average_index].x = x;
				average_array[average_index].y = y;
				average_array[average_index].rx = rx;
				average_array[average_index].ry = ry;
				average_array[AVERAGE_LEN].x += x;
				average_array[AVERAGE_LEN].y += y;
				average_array[AVERAGE_LEN].rx += rx;
				average_array[AVERAGE_LEN].ry += ry;
				x = average_array[AVERAGE_LEN].x >> AVERAGE_SHIFT;
				y = average_array[AVERAGE_LEN].y >> AVERAGE_SHIFT;
				rx = average_array[AVERAGE_LEN].rx >> AVERAGE_SHIFT;
				ry = average_array[AVERAGE_LEN].ry >> AVERAGE_SHIFT;
				dx = rx - ts->min_rx;
				dy = ry - ts->min_ry;
				average_index++;
			} else {
				/* no previous second touch */
				for (average_index = 0; average_index < AVERAGE_LEN; average_index++) {
					average_array[average_index].x = x;
					average_array[average_index].y = y;
					average_array[average_index].rx = rx;
					average_array[average_index].ry = ry;
				}
				average_array[AVERAGE_LEN].x = x << AVERAGE_SHIFT;
				average_array[AVERAGE_LEN].y = y << AVERAGE_SHIFT;
				average_array[AVERAGE_LEN].rx = rx << AVERAGE_SHIFT;
				average_array[AVERAGE_LEN].ry = ry << AVERAGE_SHIFT;
				second_touch = 1;
			}

			dev_dbg(&ts->client->dev, "minr(%4d,%4d), d(%4d,%4d), maxd(%4d,%4d)\n", ts->min_rx, ts->min_ry, dx, dy, ts->max_dx, ts->max_dy);

			if (dx > ts->max_dx)
				ts->max_dx = dx;
			if (dy > ts->max_dy)
				ts->max_dy = dy;
			dx = (dx << 11) / ts->max_dx;
			dy = (dy << 11) / ts->max_dy;

			if (dx >> 11)
				dx = MAX_11BIT;
			if (dx > x)
				dx = x;
			if (x + dx > MAX_12BIT)
				dx = MAX_12BIT - x;
			if (dy >> 11)
				dy = MAX_11BIT;
			if (dy > y)
				dy = y;
			if (y + dy > MAX_12BIT)
				dy = MAX_12BIT - y;
		} else {
			dx = dy = 0;
			second_touch = 0;
		}
	}

	/* compute touch pressure */
	if ((y != 0) && (z1 != z2)) {
		if (z2 > z1) {
			pressure = (z1 << 20) / (y * (z2 - z1));
		} else {
			pressure = (z1 << 20) / (y * (z1 - z2));
			/* FIXME?
			 * This technology provide a middle point and a delta
			 * between points, but this delta is unsigned.
			 * This means that the first point is assumed to have
			 * lowest x and y and second point higher x and y.
			 * When the real points mix higher and lower x and y
			 * this condition lead to false estimated points
			 * detection.
			 * However pinch and zoom keep on working due to the
			 * coherent variation of the estimated position.
			 * Strange behaviour: z2 is higher than z1 almost
			 * always. Somtimes z2 became lower, but only when
			 * real points mix higher and lower x and y.
			 * In this specific case, the following line leads to
			 * correct estimation, but this depends on the applyed
			 * pressure level, so at present we don't use it to
			 * avoid inconsistent behaviours
			 * dx = -dx;
			 */
		}
		if (pressure > MAX_12BIT)
			pressure = MAX_12BIT;
	} else
		pressure = MAX_12BIT;

	/* NOTE: We can't rely on the pressure to determine the pen down
	 * state, even this controller has a pressure sensor. The pressure
	 * value can fluctuate for quite a while after lifting the pen and
	 * in some cases may not even settle at the expected value.
	 *
	 * The only safe way to check for the pen up condition is in the
	 * timer by reading the pen signal state (it's a GPIO _and_ IRQ).
	 */
	if (pressure) {
		struct input_dev *input = ts->input;

		if (ts->invert_x){
			x = (~x) & MAX_12BIT;
		}
		if (ts->invert_y) {
			y = (~y) & MAX_12BIT;
		}
		if (ts->swap_xy) {
			swap(x, y);
		}

		if (!ts->pendown) {
			dev_dbg(&ts->client->dev, "DOWN\n");
			ts->pendown = 1;
			ts->scantime_jiffies = msecs_to_jiffies(SX865X_DOWN_SCANTIME_MS);
			input_report_key(input, BTN_TOUCH, 1);
		}


		if (ts->features & SX_MULTITOUCH) {
			input_report_abs(ts->input, ABS_MT_POSITION_X, x - dx);
			input_report_abs(ts->input, ABS_MT_POSITION_Y, y - dy);
			input_report_abs(ts->input, ABS_MT_PRESSURE, pressure);
			input_report_abs(ts->input, ABS_MT_TRACKING_ID, 0);
			input_mt_sync(ts->input);
			if (second_touch) {
				input_report_abs(ts->input, ABS_MT_POSITION_X, x + dx);
				input_report_abs(ts->input, ABS_MT_POSITION_Y, y + dy);
				input_report_abs(ts->input, ABS_MT_PRESSURE, pressure);
				input_report_abs(ts->input, ABS_MT_TRACKING_ID, 1);
				input_mt_sync(ts->input);
			}
			dev_dbg(&ts->client->dev, "midpoint(%4d,%4d), deltap(%4d,%4d), pressure (%4u)\n", x, y, dx, dy, pressure);
		} else {
			input_report_abs(input, ABS_X, x);
			input_report_abs(input, ABS_Y, y);
			input_report_abs(input, ABS_PRESSURE, pressure);
			dev_dbg(&ts->client->dev, "point(%4d,%4d), pressure (%4u)\n", x, y, pressure);
		}

		input_sync(input);
	}
}

static int sx865x_read_values(struct sx865x *ts)
{
	s32 data;
	u16 vals[NUM_MAX_CHANS+1];	// +1 for last dummy read
	int length, chan_read_length;
	int i;

	memset(&(ts->tc), 0, sizeof(ts->tc));
	/* The protocol and raw data format from i2c interface:
	 * S Addr R A [DataLow] A [DataHigh] A (repeat) NA P
	 * Where DataLow has (channel | [D11-D8]), DataHigh has [D7-D0].
	 */
	if (ts->features & SX_MULTITOUCH)
		chan_read_length = CHAN_READ_LENGTH(NUM_MULTITOUCH_CHANS);
	else
		chan_read_length = CHAN_READ_LENGTH(NUM_SINGLETOUCH_CHANS);

	length = i2c_master_recv(ts->client, (char *)vals, chan_read_length);

	if (likely(length == chan_read_length)) {
		length >>= 1;
		for (i = 0; i < length; i++) {
			u16 ch;
			data = swab16(vals[i]);
			if (unlikely(data & 0x8000)) {
				dev_err(&ts->client->dev, "hibit @ %d [0x%04x]\n", i, data);
				return -EAGAIN;
			}
			ch = data >> 12;
			if (ch == CH_X) {
				ts->tc.x = data & 0xfff;
			} else if (ch == CH_Y) {
				ts->tc.y = data & 0xfff;
			} else if (ch == CH_Z1) {
				ts->tc.z1 = data & 0xfff;
			} else if (ch == CH_Z2) {
				ts->tc.z2 = data & 0xfff;
			} else if (ch == CH_RX) {
				ts->tc.rx = data & 0xfff;
			} else if (ch == CH_RY) {
				ts->tc.ry = data & 0xfff;
			} else {
				dev_err(&ts->client->dev, "? CH%d %x\n", ch, data & 0xfff);
			}
		}
	} else {
		dev_err(&ts->client->dev, "%d = recv()\n", length);
	}

	dev_dbg(&ts->client->dev, "X:%d Y:%d Z1:%d Z2:%d RX:%d RY:%d\n",
		ts->tc.x, ts->tc.y, ts->tc.z1, ts->tc.z2, ts->tc.rx, ts->tc.ry);

	return !ts->tc.z1;	/* return 0 only if pressure not 0 */
}

static void sx865x_pen_up(struct sx865x *ts)
{
	struct input_dev *input = ts->input;

	/* This timer expires after PENIRQs havent been coming in for some time.
	 * It means that the pen is now UP. */
	input_report_key(input, BTN_TOUCH, 0);
	input_report_abs(input, ABS_PRESSURE, 0);
	input_sync(input);

	ts->pendown = 0;
	ts->scantime_jiffies = msecs_to_jiffies(SX865X_UP_SCANTIME_MS);

	dev_dbg(&ts->client->dev, "UP\n");
}

static int sx865x_data_available(struct sx865x *ts)
{
	u16 reg_stat, status_bit;
	u8 status;

	if (ts->features & SX_EXTENDED_REGS) {
		reg_stat = I2C_EXTENDED_REG_STAT;
		status_bit = EXTENDED_STATUS_CONVIRQ;
	} else {
		reg_stat = I2C_REG_STAT;
		status_bit = STATUS_CONVIRQ;
	}
	status = i2c_smbus_read_byte_data(ts->client, SX865X_READ_REGISTER | reg_stat);
	return status & status_bit;
}

static void sx865x_pen_irq_worker(struct work_struct *work)
{
	struct sx865x *ts = container_of(work, struct sx865x, pen_event_work.work);

	if (sx865x_data_available(ts)) {
		/* the pen is down */
		if (likely(sx865x_read_values(ts) == 0)) {
			/* valid data was read in */
			sx865x_send_event(ts);
		} else
			dev_err(&ts->client->dev, "fail\n");
	} else {
		/* the pen is up */
		if (ts->pendown)
			sx865x_pen_up(ts);
	}

	if ((ts->irq == 0) || (ts->pendown))
		queue_delayed_work(ts->ts_workq, &ts->pen_event_work, ts->scantime_jiffies);
}

static irqreturn_t sx865x_irq(int irq, void *handle)
{
	struct sx865x *ts = handle;

	/* the reading of the samples can be time-consuming if using
	 * a slow i2c, so the work is done in a queue */
	queue_delayed_work(ts->ts_workq, &ts->pen_event_work, 0);

	return IRQ_HANDLED;
}

static int get_rmsel(u16 plate_ohms)
{
	int rmsel;

	if (plate_ohms < 187)
		rmsel = 0;
	else if (plate_ohms < 312)
		rmsel = 1;
	else if (plate_ohms < 938)
		rmsel = 2;
	else if (plate_ohms < 1875)
		rmsel = 3;
	else if (plate_ohms < 4375)
		rmsel = 4;
	else if (plate_ohms < 9375)
		rmsel = 5;
	else if (plate_ohms < 18780)
		rmsel = 6;
	else
		rmsel = 7;

	return rmsel;
}

static int sx865x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct sx865x *ts;
	struct sx865x_platform_data *pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;
	u16 reset_reg, rmselx, rmsely;

	dev_info(&client->dev, "sx865x_probe()\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	ts = kzalloc(sizeof(struct sx865x), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts->input = input_dev;

	ts->features = id->driver_data;

	if (pdata) {
		ts->x_plate_ohms = pdata->x_plate_ohms;
		ts->y_plate_ohms = pdata->y_plate_ohms;
		ts->max_dx = pdata->max_dx;
		ts->max_dy = pdata->max_dy;
		ts->invert_x = pdata->flags & SX865X_INVERT_X;
		ts->invert_y = pdata->flags & SX865X_INVERT_Y;
		ts->swap_xy = pdata->flags & SX865X_SWAP_XY;
	} else {
		/* U-MoBo defaults */
		ts->x_plate_ohms = 600;
		ts->y_plate_ohms = 300;
		ts->invert_x = 0;
		ts->invert_y = 1;
		ts->swap_xy = 0;
	}
	if (!ts->max_dx)
		ts->max_dx = MAX_12BIT >> 3;
	if (!ts->max_dy)
		ts->max_dy = MAX_12BIT >> 4;
	ts->min_rx = ts->min_ry = MAX_12BIT;

	ts->scantime_jiffies = msecs_to_jiffies(SX865X_UP_SCANTIME_MS);

	input_dev->name = "SX865X Touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_set_drvdata(input_dev, ts);

	__set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 1, 0, 0);


	if (ts->features & SX_EXTENDED_REGS)
		reset_reg = I2C_EXTENDED_REG_SOFTRESET;
	else
		reset_reg = I2C_REG_SOFTRESET;
	err = i2c_smbus_write_byte_data(client, reset_reg, SOFTRESET_VALUE);
	/* soft reset: SX8650 fails to nak at the end, ignore return value */


	if (ts->features & SX_MULTITOUCH) {
		/* set mask to convert X, Y, Z1, Z2, RX, RY for CH_SEQ */
		err = i2c_smbus_write_byte_data(client, I2C_REG_CHANMASK, CHAN_MULTITOUCH_MASK);
		if (err != 0) {
			dev_err(&client->dev, "write mask fail");
			goto err_free_mem;
		}
		if (ts->x_plate_ohms < 100)
			ts->x_plate_ohms = 100;
		if (ts->y_plate_ohms < 100)
			ts->y_plate_ohms = 100;
		rmselx = get_rmsel(ts->x_plate_ohms);
		rmsely = get_rmsel(ts->y_plate_ohms);
		err = i2c_smbus_write_byte_data(client, I2C_REG_CTRL3, rmselx | (rmsely << 3));
		if (err != 0) {
			dev_err(&client->dev, "writereg3 fail");
			goto err_free_mem;
		}
	} else {
		/* set mask to convert X, Y, Z1, Z2 for CH_SEQ */
		err = i2c_smbus_write_byte_data(client, I2C_REG_CHANMASK, CHAN_SINGLETOUCH_MASK);
		if (err != 0) {
			dev_err(&client->dev, "write mask fail");
			goto err_free_mem;
		}
	}

	err = i2c_smbus_write_byte_data(client, I2C_REG_CTRL1, CONDIRQ | FILT_7SA);
	if (err != 0) {
		dev_err(&client->dev, "writereg1 fail");
		goto err_free_mem;
	}

	/* set POWDLY settling time -- adjust TS_TIMEOUT accordingly */
	err = i2c_smbus_write_byte_data(client, I2C_REG_CTRL0, POWDLY_1_1MS | (ts->irq ? RATE_100CPS : 0));
	if (err != 0) {
		dev_err(&client->dev, "writereg0 fail");
		goto err_free_mem;
	}

	ts->ts_workq = create_singlethread_workqueue("sx865x");
	if (ts->ts_workq == NULL) {
		dev_err(&client->dev, "failed to create workqueue\n");
		goto err_free_mem;
	}

	INIT_DELAYED_WORK(&ts->pen_event_work, sx865x_pen_irq_worker);

	ts->irq = client->irq;

	if (ts->irq) {
		err = request_threaded_irq(ts->irq, NULL, sx865x_irq, IRQF_TRIGGER_FALLING,
				client->dev.driver->name, ts);
		if (err < 0) {
			dev_err(&client->dev, "irq %d busy?\n", ts->irq);
			goto err_free_mem;
		}
	} else {
		queue_delayed_work(ts->ts_workq, &ts->pen_event_work, ts->scantime_jiffies);
	}

	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	dev_info(&client->dev, "registered with irq (%d)\n", ts->irq);

	/* enter pen-trigger mode */
	err = i2c_smbus_write_byte(client, SX865X_PENTRG);
	if (err != 0) {
		dev_err(&client->dev, "enter fail");
		goto err_free_mem;
	}

	return 0;

 err_free_irq:
	if (ts->irq)
		free_irq(ts->irq, ts);
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int sx865x_remove(struct i2c_client *client)
{
	struct sx865x *ts = i2c_get_clientdata(client);
	struct sx865x_platform_data *pdata;

	pdata = client->dev.platform_data;

	cancel_delayed_work_sync(&ts->pen_event_work);
	destroy_workqueue(ts->ts_workq);

	if (ts->irq)
		free_irq(ts->irq, ts);
	input_unregister_device(ts->input);
	kfree(ts);

	return 0;
}

static struct i2c_device_id sx865x_idtable[] = {
	{ "sx8650", 0 },
	{ "sx8651", SX_MULTITOUCH },
	{ "sx8654", SX_PROXIMITY_SENSING | SX_HAPTICS_GENERIC },
	{ "sx8655", SX_HAPTICS_GENERIC },
	{ "sx8656", SX_PROXIMITY_SENSING },
	{ "sx8657", SX_PROXIMITY_SENSING | SX_HAPTICS_IMMERSION },
	{ "sx8658", SX_HAPTICS_IMMERSION },
	{ "sx8674", SX_MULTITOUCH | SX_PROXIMITY_SENSING | SX_HAPTICS_GENERIC },
	{ "sx8675", SX_MULTITOUCH | SX_HAPTICS_GENERIC },
	{ "sx8676", SX_MULTITOUCH | SX_PROXIMITY_SENSING },
	{ "sx8677", SX_MULTITOUCH | SX_PROXIMITY_SENSING | SX_HAPTICS_IMMERSION },
	{ "sx8678", SX_MULTITOUCH | SX_HAPTICS_IMMERSION },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sx865x_idtable);

static struct i2c_driver sx865x_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "sx865x"
	},
	.id_table = sx865x_idtable,
	.probe = sx865x_probe,
	.remove	= sx865x_remove,
};

static int __init sx865x_init(void)
{
	return i2c_add_driver(&sx865x_driver);
}

static void __exit sx865x_exit(void)
{
	i2c_del_driver(&sx865x_driver);
}

module_init(sx865x_init);
module_exit(sx865x_exit);

MODULE_AUTHOR("Pierluigi Passaro <p.passaro@u-mobo.com>");
MODULE_DESCRIPTION("SX865X TouchScreen Driver");
MODULE_LICENSE("GPL");

