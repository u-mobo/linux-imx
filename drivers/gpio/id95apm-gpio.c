/*
 * gpiolib support for IDT ID95APM PMIC
 *
 * Copyright 2010 Stefan Roese <sr@denx.de>, DENX Software Engineering
 *
 * This code is based on wm831x-gpio.c:
 * Copyright 2009 Wolfson Microelectronics PLC.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/mfd/core.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include <linux/mfd/id95apm.h>

/* ID95APM has GPIO0 - GPIO10 */
#define ID95APM_GPIO_MAX 11

struct id95apm_gpio {
	struct id95apm *id95apm;
	struct gpio_chip gpio_chip;
};

static inline struct id95apm_gpio *to_id95apm_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct id95apm_gpio, gpio_chip);
}

static int id95apm_gpio_set_gpio(struct id95apm *id95apm, unsigned offset)
{
	int ret;

	/* Clear OFF bit to select GPIO functionality */
	ret = id95apm_clrset_bits16(id95apm, ID95APM_PCON_GPIO_OFF,
				    1 << offset, 0);

	/* Clear SPECIAL bit to select GPIO functionality */
	if ((offset >= 0) && (offset <= 5)) {
		ret = id95apm_clrset_bits(id95apm, ID95APM_PCON_GPIO_SPECIAL,
					  1 << offset, 0);
	}

	return ret;
}

static int id95apm_gpio_direction_in(struct gpio_chip *chip, unsigned offset)
{
	struct id95apm_gpio *id95apm_gpio = to_id95apm_gpio(chip);
	struct id95apm *id95apm = id95apm_gpio->id95apm;

	/* Set this pin to GPIO and not ADC or special function */
	id95apm_gpio_set_gpio(id95apm, offset);

	/* Clear bit for input */
	return id95apm_clrset_bits16(id95apm, ID95APM_PCON_GPIO_DIR,
				     1 << offset, 0);
}

static int id95apm_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct id95apm_gpio *id95apm_gpio = to_id95apm_gpio(chip);
	struct id95apm *id95apm = id95apm_gpio->id95apm;
	u16 val;

	val = id95apm_reg16_read(id95apm, ID95APM_PCON_GPIO_DATA);

	if (val & (1 << offset))
		return 1;
	else
		return 0;
}

static int id95apm_gpio_direction_out(struct gpio_chip *chip,
				     unsigned offset, int value)
{
	struct id95apm_gpio *id95apm_gpio = to_id95apm_gpio(chip);
	struct id95apm *id95apm = id95apm_gpio->id95apm;

	/* Set this pin to GPIO and not ADC or special function */
	id95apm_gpio_set_gpio(id95apm, offset);

	/* Set bit for output */
	return id95apm_clrset_bits16(id95apm, ID95APM_PCON_GPIO_DIR,
				     0, 1 << offset);
}

static void id95apm_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct id95apm_gpio *id95apm_gpio = to_id95apm_gpio(chip);
	struct id95apm *id95apm = id95apm_gpio->id95apm;

	id95apm_clrset_bits16(id95apm, ID95APM_PCON_GPIO_DATA, 1 << offset,
			      value << offset);
}

static struct gpio_chip template_chip = {
	.label			= "id95apm",
	.owner			= THIS_MODULE,
	.direction_input	= id95apm_gpio_direction_in,
	.get			= id95apm_gpio_get,
	.direction_output	= id95apm_gpio_direction_out,
	.set			= id95apm_gpio_set,
	.can_sleep		= 1,
};

static int __devinit id95apm_gpio_probe(struct platform_device *pdev)
{
	struct id95apm *id95apm = dev_get_drvdata(pdev->dev.parent);
	struct id95apm_gpio_init *pdata = pdev->dev.platform_data;
	struct id95apm_gpio *id95apm_gpio;
	int ret;

	id95apm_gpio = kzalloc(sizeof(*id95apm_gpio), GFP_KERNEL);
	if (id95apm_gpio == NULL)
		return -ENOMEM;

	id95apm_gpio->id95apm = id95apm;
	id95apm_gpio->gpio_chip = template_chip;
	id95apm_gpio->gpio_chip.ngpio = ID95APM_GPIO_MAX;
	id95apm_gpio->gpio_chip.dev = &pdev->dev;
	if (pdata && pdata->gpio_base)
		id95apm_gpio->gpio_chip.base = pdata->gpio_base;
	else
		id95apm_gpio->gpio_chip.base = -1;

	ret = gpiochip_add(&id95apm_gpio->gpio_chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n",
			ret);
		goto err;
	}

	platform_set_drvdata(pdev, id95apm_gpio);

	return ret;

err:
	kfree(id95apm_gpio);
	return ret;
}

static int __devexit id95apm_gpio_remove(struct platform_device *pdev)
{
	struct id95apm_gpio *id95apm_gpio = platform_get_drvdata(pdev);
	int ret;

	ret = gpiochip_remove(&id95apm_gpio->gpio_chip);
	if (ret == 0)
		kfree(id95apm_gpio);

	return ret;
}

static struct platform_driver id95apm_gpio_driver = {
	.driver.name	= "id95apm-gpio",
	.driver.owner	= THIS_MODULE,
	.probe		= id95apm_gpio_probe,
	.remove		= __devexit_p(id95apm_gpio_remove),
};

static int __init id95apm_gpio_init(void)
{
	return platform_driver_register(&id95apm_gpio_driver);
}
subsys_initcall(id95apm_gpio_init);

static void __exit id95apm_gpio_exit(void)
{
	platform_driver_unregister(&id95apm_gpio_driver);
}
module_exit(id95apm_gpio_exit);

MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_DESCRIPTION("GPIO interface for ID95APM PMIC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:id95apm-gpio");
