/*
 * Watchdog driver for the ID95APM PMIC
 *
 * Copyright 2009-2010 Stefan Roese <sr@denx.de>, DENX Software Engineering
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/uaccess.h>
#include <linux/mfd/id95apm.h>

static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static unsigned long id95apm_wdt_users;
static struct miscdevice id95apm_wdt_miscdev;
static int id95apm_wdt_expect_close;

/*
 * ID95APM supports 4 timebases for the watchdog timer:
 * 8Hz, 1Hz, 0.5Hz, 1/60Hz (1 min)
 *
 * With 8bit timer values, this gives the following possible
 * timeouts:
 *
 * 8Hz timebase:
 * -------------
 * 1: 0.125s
 * 2: 0.250s
 * 3: 0.375s
 * ...
 *
 * 1Hz timebase:
 * -------------
 * 1: 1s
 * 2: 2s
 * 3: 3s
 * ...
 *
 * 0.5Hz timebase:
 * -------------
 * 1: 2s
 * 2: 4s
 * 3: 6s
 * ...
 *
 * 0.016Hz (1/60) timebase:
 * -------------
 * 1: 60s
 * 2: 120s
 * 3: 180s
 * ...
 *
 */

#define MAX_COUNT	256
#define FACTOR		10000

/* Watchdog timebase * FACTOR */
static unsigned int wdt_timebase[] = { 8 * FACTOR, 1 * FACTOR, FACTOR / 2,
				       FACTOR / 60 };
static int id95apm_timeout;

static struct id95apm *get_id95apm(void)
{
	return dev_get_drvdata(id95apm_wdt_miscdev.parent);
}

static int id95apm_wdt_set_timeout(struct id95apm *id95apm, int timeout)
{
	int i;
	int ret;
	int count;

	/*
	 * Seach best fitting timebase for requested timeout value
	 */
	for (i = 0; i < ARRAY_SIZE(wdt_timebase); i++) {
		if (wdt_timebase[i] < ((FACTOR * MAX_COUNT) / timeout))
			break;
	}

	if (i == ARRAY_SIZE(wdt_timebase)) {
		printk(KERN_ERR "%s: timeout %d not supported!\n",
		       __func__, timeout);
		return -EINVAL;
	}

	/* First set timebase */
	id95apm_clrset_bits(id95apm, ID95APM_GPTIMER_TBASE,
			    ID95APM_GPTIMER_TBASE_WDT_MASK, i << 4);

	/* Set timer count */
	count = ((wdt_timebase[i] * timeout) / FACTOR);
	if (count > 0xff)
		count = 0xff;
	ret = id95apm_reg_write(id95apm, ID95APM_GPTIMER_WDT_COUNT, count);

	/* Re-calculate the timeout value */
	id95apm_timeout = (FACTOR * count) / wdt_timebase[i];

	return ret;
}

static int id95apm_wdt_start(struct id95apm *id95apm)
{
	/* Generally enable GPT module */
	id95apm_clrset_bits(id95apm, ID95APM_PCON_GP_TIMER_CTRL,
			    0, ID95APM_PCON_GP_TIMER_CTRL_EN);

	/* Enable watchdog timer */
	return id95apm_clrset_bits(id95apm, ID95APM_GPTIMER_WDT_EN,
				   0, ID95APM_GPTIMER_WDT_EN_EN);
}

static int id95apm_wdt_stop(struct id95apm *id95apm)
{
	/* Disable watchdog timer */
	return id95apm_clrset_bits(id95apm, ID95APM_GPTIMER_WDT_EN,
				   ID95APM_GPTIMER_WDT_EN_EN, 0);
}

static int id95apm_wdt_kick(struct id95apm *id95apm)
{
	return id95apm_clrset_bits(id95apm, ID95APM_GPTIMER_WDT_EN,
				   0, ID95APM_GPTIMER_WDT_EN_RST);
}

static int id95apm_wdt_open(struct inode *inode, struct file *file)
{
	struct id95apm *id95apm = get_id95apm();
	int ret;

	if (!id95apm)
		return -ENODEV;

	if (test_and_set_bit(0, &id95apm_wdt_users))
		return -EBUSY;

	ret = id95apm_wdt_start(id95apm);
	if (ret != 0)
		return ret;

	return nonseekable_open(inode, file);
}

static int id95apm_wdt_release(struct inode *inode, struct file *file)
{
	struct id95apm *id95apm = get_id95apm();

	if (id95apm_wdt_expect_close)
		id95apm_wdt_stop(id95apm);
	else {
		dev_warn(id95apm->dev, "Watchdog device closed uncleanly\n");
		id95apm_wdt_kick(id95apm);
	}

	clear_bit(0, &id95apm_wdt_users);

	return 0;
}

static ssize_t id95apm_wdt_write(struct file *file,
				 const char __user *data, size_t count,
				 loff_t *ppos)
{
	struct id95apm *id95apm = get_id95apm();
	size_t i;

	if (count) {
		id95apm_wdt_kick(id95apm);

		if (!nowayout) {
			/* In case it was set long ago */
			id95apm_wdt_expect_close = 0;

			/* scan to see whether or not we got the magic
			   character */
			for (i = 0; i != count; i++) {
				char c;
				if (get_user(c, data + i))
					return -EFAULT;
				if (c == 'V')
					id95apm_wdt_expect_close = 42;
			}
		}
	}
	return count;
}

static struct watchdog_info ident = {
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
	.identity = "ID95APM Watchdog",
};

static long id95apm_wdt_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	struct id95apm *id95apm = get_id95apm();
	int ret = -ENOTTY, time;
	void __user *argp = (void __user *)arg;
	int __user *p = argp;

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		ret = copy_to_user(argp, &ident, sizeof(ident)) ? -EFAULT : 0;
		break;

	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		ret = put_user(0, p);
		break;

	case WDIOC_SETOPTIONS:
	{
		int options;

		if (get_user(options, p))
			return -EFAULT;

		ret = -EINVAL;

		/* Setting both simultaneously means at least one must fail */
		if (options == WDIOS_DISABLECARD)
			ret = id95apm_wdt_start(id95apm);

		if (options == WDIOS_ENABLECARD)
			ret = id95apm_wdt_stop(id95apm);
		break;
	}

	case WDIOC_KEEPALIVE:
		ret = id95apm_wdt_kick(id95apm);
		break;

	case WDIOC_SETTIMEOUT:
		ret = get_user(time, p);
		if (ret)
			break;

		if (time == 0) {
			if (nowayout)
				ret = -EINVAL;
			else
				id95apm_wdt_stop(id95apm);
			break;
		}

		ret = id95apm_wdt_set_timeout(id95apm, time);
		break;

	case WDIOC_GETTIMEOUT:
		ret = put_user(id95apm_timeout, p);

	}

	return ret;
}

static const struct file_operations id95apm_wdt_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.write = id95apm_wdt_write,
	.unlocked_ioctl = id95apm_wdt_ioctl,
	.open = id95apm_wdt_open,
	.release = id95apm_wdt_release,
};

static struct miscdevice id95apm_wdt_miscdev = {
	.minor = WATCHDOG_MINOR,
	.name = "watchdog",
	.fops = &id95apm_wdt_fops,
};

static void id95apm_wdt_handler(struct id95apm *id95apm, int irq,
				void *data)
{
	dev_err(id95apm->dev, "watchdog timer expired (irq)\n");

	id95apm_wdt_kick(id95apm);

	/* Acknowledge timeout */
	id95apm_clrset_bits(id95apm, ID95APM_GPTIMER_IRQ_STAT,
			    0, ID95APM_GPTIMER_IRQ_STAT_WDT);
}

static int id95apm_wdt_probe(struct platform_device *pdev)
{
	struct id95apm *id95apm = platform_get_drvdata(pdev);
	int ret;

	if (!id95apm) {
		dev_err(id95apm->dev, "No driver data supplied\n");
		return -ENODEV;
	}

	/* Default to 4s timeout */
	id95apm_timeout = 4;
	id95apm_wdt_set_timeout(id95apm, id95apm_timeout);

	id95apm_wdt_miscdev.parent = &pdev->dev;

	ret = misc_register(&id95apm_wdt_miscdev);
	if (ret) {
		dev_err(&pdev->dev, "cannot register misc device\n");
		return ret;
	}

	id95apm_register_irq(id95apm, ID95APM_IRQ_WDT,
			     id95apm_wdt_handler, NULL);
	/* Enable watchdog interrupt */
	id95apm_clrset_bits(id95apm, ID95APM_GPTIMER_IRQ_EN,
			    0, ID95APM_GPTIMER_IRQ_EN_WDT);

	dev_info(&pdev->dev, "watchdog timer initialized\n");

	return 0;
}

static int id95apm_wdt_remove(struct platform_device *pdev)
{
	struct id95apm *id95apm = platform_get_drvdata(pdev);

	/* Disable watchdog interrupt */
	id95apm_clrset_bits(id95apm, ID95APM_GPTIMER_IRQ_EN,
			    ID95APM_GPTIMER_IRQ_EN_WDT, 0);
	id95apm_free_irq(id95apm, ID95APM_IRQ_WDT);

	misc_deregister(&id95apm_wdt_miscdev);

	return 0;
}

static struct platform_driver id95apm_wdt_driver = {
	.probe = id95apm_wdt_probe,
	.remove = id95apm_wdt_remove,
	.driver = {
		.name = "id95apm-wdt",
	},
};

static int __init id95apm_wdt_init(void)
{
	return platform_driver_register(&id95apm_wdt_driver);
}
/* Init early because of potentionally already pending irq's */
subsys_initcall(id95apm_wdt_init);

static void __exit id95apm_wdt_exit(void)
{
	platform_driver_unregister(&id95apm_wdt_driver);
}
module_exit(id95apm_wdt_exit);

MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_DESCRIPTION("ID95APM Watchdog");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:id95apm-wdt");
