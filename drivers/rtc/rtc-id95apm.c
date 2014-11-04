/*
 * Real Time Clock driver for IDT ID95APM
 *
 * Copyright 2009-2010 Stefan Roese <sr@denx.de>, DENX Software Engineering
 *
 * This code is based on rtc-wm8350.c:
 * Copyright (C) 2007, 2008 Wolfson Microelectronics PLC.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mfd/id95apm.h>

/*
 * Read current time and date in RTC
 */
static int id95apm_rtc_readtime(struct device *dev, struct rtc_time *tm)
{
	struct id95apm *id95apm = dev_get_drvdata(dev);
	u8 regs[7];
	int ret;

	/* read the RTC date and time registers all at once */
	ret = id95apm_block_read(id95apm, ID95APM_RTC_SECOND_BASE, 7, regs);
	if (ret) {
		dev_err(dev, "%s error %d\n", "read", ret);
		return -EIO;
	}

	dev_dbg(dev, "%s: %02x %02x %02x %02x %02x %02x %02x\n", "read",
		regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6]);

	tm->tm_sec = bcd2bin(regs[ID95APM_RTC_SECOND] & 0x7f);
	tm->tm_min = bcd2bin(regs[ID95APM_RTC_MINUTE] & 0x7f);
	tm->tm_hour = bcd2bin(regs[ID95APM_RTC_HOUR] & 0x3f);
	tm->tm_wday = bcd2bin(regs[ID95APM_RTC_DAY] & 0x07) - 1;
	tm->tm_mday = bcd2bin(regs[ID95APM_RTC_DATE] & 0x3f);
	tm->tm_mon = bcd2bin(regs[ID95APM_RTC_MONTH] & 0x1f) - 1;

	/* assume 20YY not 19YY, and ignore century bit */
	tm->tm_year = bcd2bin(regs[ID95APM_RTC_YEAR]) + 100;

	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
		"read", tm->tm_sec, tm->tm_min,
		tm->tm_hour, tm->tm_mday,
		tm->tm_mon, tm->tm_year, tm->tm_wday);

	/* initial clock setting can be undefined */
	return rtc_valid_tm(tm);
}

/*
 * Set current time and date in RTC
 */
static int id95apm_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	struct id95apm *id95apm = dev_get_drvdata(dev);
	int ret;
	u8 buf[7];

	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
		"write", tm->tm_sec, tm->tm_min,
		tm->tm_hour, tm->tm_mday,
		tm->tm_mon, tm->tm_year, tm->tm_wday);

	buf[ID95APM_RTC_SECOND] = bin2bcd(tm->tm_sec);
	buf[ID95APM_RTC_MINUTE] = bin2bcd(tm->tm_min);
	buf[ID95APM_RTC_HOUR] = bin2bcd(tm->tm_hour);
	buf[ID95APM_RTC_DAY] = bin2bcd(tm->tm_wday + 1);
	buf[ID95APM_RTC_DATE] = bin2bcd(tm->tm_mday);
	buf[ID95APM_RTC_MONTH] = bin2bcd(tm->tm_mon + 1);

	/* assume 20YY not 19YY */
	buf[ID95APM_RTC_YEAR] = bin2bcd(tm->tm_year - 100);
	buf[ID95APM_RTC_MONTH] |= ID95APM_RTC_MONTH_CENTURY;

	dev_dbg(dev, "%s: %02x %02x %02x %02x %02x %02x %02x\n",
		"write", buf[0], buf[1], buf[2], buf[3],
		buf[4], buf[5], buf[6]);

	ret = id95apm_block_write(id95apm, ID95APM_RTC_SECOND_BASE, 7, buf);
	if (ret) {
		dev_err(dev, "%s error %d\n", "write", ret);
		return -EIO;
	}

	return ret;
}

/*
 * Read alarm time and date in RTC
 */
static int id95apm_rtc_readalarm(struct device *dev, struct rtc_wkalrm *tm)
{
	struct id95apm *id95apm = dev_get_drvdata(dev);
	u8 regs[10];
	int ret;

	/* read all ALARM1, ALARM2, and status registers at once */
	ret = id95apm_block_read(id95apm, ID95APM_RTC_ALARM1_SECOND, 10, regs);
	if (ret) {
		dev_err(dev, "%s error %d\n", "alarm read", ret);
		return -EIO;
	}

	dev_dbg(dev, "%s: %02x %02x %02x %02x, %02x %02x %02x, %02x %02x %02x\n",
		"alarm read", regs[0], regs[1], regs[2], regs[3],
		regs[4], regs[5], regs[6], regs[7], regs[8], regs[9]);

	/*
	 * report alarm time (ALARM1); assume 24 hour and day-of-month modes,
	 * and that all four fields are checked matches
	 */
	tm->time.tm_sec = bcd2bin(regs[0] & 0x7f);
	tm->time.tm_min = bcd2bin(regs[1] & 0x7f);
	tm->time.tm_hour = bcd2bin(regs[2] & 0x3f);
	tm->time.tm_mday = bcd2bin(regs[3] & 0x3f);
	tm->time.tm_mon = -1;
	tm->time.tm_year = -1;
	tm->time.tm_wday = -1;
	tm->time.tm_yday = -1;
	tm->time.tm_isdst = -1;

	/* ... and status */
	tm->enabled = !!(regs[8] & ID95APM_RTC_IRQ_A1_EN);
	tm->pending = !!(regs[9] & ID95APM_RTC_IRQ_A1_EN);

	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, enabled=%d, pending=%d\n",
		"alarm read", tm->time.tm_sec, tm->time.tm_min,
		tm->time.tm_hour, tm->time.tm_mday,
		tm->enabled, tm->pending);

	return 0;
}

static int id95apm_rtc_setalarm(struct device *dev, struct rtc_wkalrm *tm)
{
	struct id95apm *id95apm = dev_get_drvdata(dev);
	u8 buf[10];
	int ret;

	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, enabled=%d, pending=%d\n",
		"alarm set", tm->time.tm_sec, tm->time.tm_min,
		tm->time.tm_hour, tm->time.tm_mday,
		tm->enabled, tm->pending);

	/* set ALARM1, using 24 hour and day-of-month modes */
	buf[0] = bin2bcd(tm->time.tm_sec);
	buf[1] = bin2bcd(tm->time.tm_min);
	buf[2] = bin2bcd(tm->time.tm_hour);
	buf[3] = bin2bcd(tm->time.tm_mday);

	/* set ALARM2 to non-garbage */
	buf[4] = 0;
	buf[5] = 0;
	buf[6] = 0;
	buf[7] = 0;

	/* optionally enable ALARM1 */
	buf[8] = 0;
	if (tm->enabled) {
		dev_dbg(dev, "alarm IRQ armed\n");
		/* only ALARM1 is used */
		buf[8] |= ID95APM_RTC_IRQ_A1_EN;
	}

	/* clear pending interrupt */
	buf[9] = ID95APM_RTC_IRQ_A1_EN;

	ret = id95apm_block_write(id95apm, ID95APM_RTC_ALARM1_SECOND, 10, buf);
	if (ret) {
		dev_err(dev, "can't set alarm time\n");
		return ret;
	}

	return 0;
}

/*
 * Handle commands from user-space
 */
static int id95apm_rtc_ioctl(struct device *dev, unsigned int cmd,
			     unsigned long arg)
{
	struct id95apm *id95apm = dev_get_drvdata(dev);

	switch (cmd) {
	case RTC_AIE_OFF:
		/* disable interrupt */
		id95apm_clrset_bits(id95apm, ID95APM_RTC_IRQ_EN,
				    ID95APM_RTC_IRQ_A1_EN, 0);
		break;

	case RTC_AIE_ON:
		/* enable interrupt */
		id95apm_clrset_bits(id95apm, ID95APM_RTC_IRQ_EN,
				    0, ID95APM_RTC_IRQ_A1_EN);
		break;

	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}

static void id95apm_rtc_alarm_handler(struct id95apm *id95apm, int irq,
				      void *data)
{
	struct rtc_device *rtc = data;

	rtc_update_irq(rtc, 1, RTC_IRQF | RTC_AF);

	id95apm_clrset_bits(id95apm, ID95APM_RTC_IRQ_STAT,
			    0, ID95APM_RTC_IRQ_A1_EN);
}

static const struct rtc_class_ops id95apm_rtc_ops = {
	.read_time = id95apm_rtc_readtime,
	.set_time = id95apm_rtc_settime,
	.read_alarm = id95apm_rtc_readalarm,
	.set_alarm = id95apm_rtc_setalarm,
	.ioctl = id95apm_rtc_ioctl,
};

static int id95apm_rtc_probe(struct platform_device *pdev)
{
	struct id95apm *id95apm = platform_get_drvdata(pdev);
	int ret;

	id95apm->rtc = rtc_device_register("id95apm", &pdev->dev,
					   &id95apm_rtc_ops, THIS_MODULE);
	if (IS_ERR(id95apm->rtc)) {
		ret = PTR_ERR(id95apm->rtc);
		dev_err(&pdev->dev, "failed to register RTC: %d\n", ret);
		return ret;
	}

	id95apm_register_irq(id95apm, ID95APM_IRQ_RTC_ALARM1,
			     id95apm_rtc_alarm_handler, id95apm->rtc);

	return 0;
}

static int id95apm_rtc_remove(struct platform_device *pdev)
{
	struct id95apm *id95apm = platform_get_drvdata(pdev);

	id95apm_free_irq(id95apm, ID95APM_IRQ_RTC_ALARM1);

	rtc_device_unregister(id95apm->rtc);

	return 0;
}

static struct platform_driver id95apm_rtc_driver = {
	.probe = id95apm_rtc_probe,
	.remove = id95apm_rtc_remove,
	.driver = {
		.name = "id95apm-rtc",
	},
};

static int __init id95apm_rtc_init(void)
{
	return platform_driver_register(&id95apm_rtc_driver);
}
module_init(id95apm_rtc_init);

static void __exit id95apm_rtc_exit(void)
{
	platform_driver_unregister(&id95apm_rtc_driver);
}
module_exit(id95apm_rtc_exit);

MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_DESCRIPTION("RTC driver for the ID95APM");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:id95apm-rtc");
