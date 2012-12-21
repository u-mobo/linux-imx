/*
 * Backlight driver for IDT ID95APM audio & PMIC
 *
 * Copyright 2010 Stefan Roese <sr@denx.de>, DENX Software Engineering
 *
 * This code is based on wm831x_bl.c:
 * Copyright 2009 Wolfson Microelectonics plc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>

#include <linux/mfd/id95apm.h>

struct id95apm_backlight_data {
	struct id95apm *id95apm;
	int current_brightness;
};

static int id95apm_backlight_set(struct backlight_device *bl, int brightness)
{
	struct id95apm_backlight_data *data = bl_get_data(bl);
	struct id95apm *id95apm = data->id95apm;
	int val = 0;

	/*
	 * brightness = 0  -> off
	 *		1  -> min brightness, 0.39mA, half scale
	 *		2  -> 0.78mA, half scale (step 0.39mA)
	 *		...
	 *		32 -> 12.5mA, half scale
	 *		33 -> 13.28mA, full scale (step 0.78mA)
	 *		...
	 *		47 -> 24.22mA, full scale
	 *		48 -> max brightness, 25.00 mA, full scale
	 */
	if (brightness > 32) {
		val |= ID95APM_LED_BOOST_SCALE_FULL;
		brightness -= 16;
	}

	/* Use brightness == 0 to switch off LEB boost converter */
	if (brightness != 0) {
		val |= ID95APM_LED_BOOST_EN;
		val |= brightness - 1;
	}

	/* Set the new brightness */
	id95apm_reg_write(id95apm, ID95APM_DCDC_LED_BOOST, val);
	data->current_brightness = brightness;

	return 0;
}

static int id95apm_backlight_update_status(struct backlight_device *bl)
{
	int brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31))
	if (bl->props.state & BL_CORE_SUSPENDED)
		brightness = 0;
#endif

	return id95apm_backlight_set(bl, brightness);
}

static int id95apm_backlight_get_brightness(struct backlight_device *bl)
{
	struct id95apm_backlight_data *data = bl_get_data(bl);
	return data->current_brightness;
}

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31))
static const struct backlight_ops id95apm_backlight_ops = {
#else
static struct backlight_ops id95apm_backlight_ops = {
#endif
	.update_status	= id95apm_backlight_update_status,
	.get_brightness	= id95apm_backlight_get_brightness,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31))
	.options = BL_CORE_SUSPENDRESUME,
#endif
};

static int id95apm_backlight_probe(struct platform_device *pdev)
{
	struct id95apm *id95apm = dev_get_drvdata(pdev->dev.parent);
	struct id95apm_backlight_data *data;
	struct backlight_device *bl;
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31))
	struct backlight_properties props;
#endif

	/* Enable LED boost converter */
	id95apm_clrset_bits(id95apm, ID95APM_GLOB_DCDC_ENABLE, 0, 0x10);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	data->id95apm = id95apm;
	data->current_brightness = 0;

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31))
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = 48;
	bl = backlight_device_register("id95apm", &pdev->dev,
				       data, &id95apm_backlight_ops, &props);
#else
	bl = backlight_device_register("id95apm", &pdev->dev,
				       data, &id95apm_backlight_ops);
#endif
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		kfree(data);
		return PTR_ERR(bl);
	}

	bl->props.max_brightness = 48;
	bl->props.brightness = 0;

	platform_set_drvdata(pdev, bl);

	backlight_update_status(bl);

	return 0;
}

static int id95apm_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct id95apm_backlight_data *data = bl_get_data(bl);

	backlight_device_unregister(bl);
	kfree(data);
	return 0;
}

static struct platform_driver id95apm_backlight_driver = {
	.driver		= {
		.name	= "id95apm-backlight",
		.owner	= THIS_MODULE,
	},
	.probe		= id95apm_backlight_probe,
	.remove		= id95apm_backlight_remove,
};

static int __init id95apm_backlight_init(void)
{
	return platform_driver_register(&id95apm_backlight_driver);
}
module_init(id95apm_backlight_init);

static void __exit id95apm_backlight_exit(void)
{
	platform_driver_unregister(&id95apm_backlight_driver);
}
module_exit(id95apm_backlight_exit);

MODULE_DESCRIPTION("Backlight Driver for ID95APM audio & PMIC");
MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:id95apm-backlight");
