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

static int id95apm_gpio_is_valid(struct id95apm *id95apm, unsigned offset)
{
	if ((offset == 0)  || (offset >= ID95APM_GPIO_MAX)) {
		dev_err(id95apm->dev, "gpio pin %d not available\n", offset);
		return 0;
	}
	if ((id95apm->chip_irq) && (offset == 5)) {
		dev_err(id95apm->dev, "gpio pin 5 reserved for irq out\n");
		return 0;
	}
#if defined(CONFIG_TOUCHSCREEN_ID95APM) || defined(CONFIG_SENSORS_ID95APM)
	if ((offset >= 6) && (offset <= 9)) {
		dev_err(id95apm->dev, "gpio pin %d reserved for ADC\n", offset);
		return 0;
	}
#endif
	return 1;
}

static int id95apm_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct id95apm_gpio *id95apm_gpio = container_of(chip, struct id95apm_gpio, chip);
	struct id95apm *id95apm =  container_of(id95apm_gpio, struct id95apm, gpio);
	u16 mask;

	if (!id95apm_gpio_is_valid(id95apm, offset))
		return -EINVAL;

	mask = 1 << offset;
	if (id95apm_gpio->requested & mask)
		return -EBUSY;

	id95apm_gpio->requested |= mask;
	return 0;
}

static void id95apm_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct id95apm_gpio *id95apm_gpio = container_of(chip, struct id95apm_gpio, chip);
	struct id95apm *id95apm =  container_of(id95apm_gpio, struct id95apm, gpio);
	u16 mask;

	if (!id95apm_gpio_is_valid(id95apm, offset))
		return;

	mask = 1 << offset;
	if (id95apm_gpio->requested & mask)
		id95apm_gpio->requested &= ~mask;
}

static int id95apm_gpio_direction_in(struct gpio_chip *chip, unsigned offset)
{
	struct id95apm_gpio *id95apm_gpio = container_of(chip, struct id95apm_gpio, chip);
	struct id95apm *id95apm =  container_of(id95apm_gpio, struct id95apm, gpio);
	u16 mask;

	if (!id95apm_gpio_is_valid(id95apm, offset))
		return -EINVAL;

	mask = 1 << offset;

	/* Clear bit for input */
	return id95apm_clrset_bits16(id95apm, ID95APM_PCON_GPIO_DIR, mask, 0);
}

static int id95apm_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct id95apm_gpio *id95apm_gpio = container_of(chip, struct id95apm_gpio, chip);
	struct id95apm *id95apm =  container_of(id95apm_gpio, struct id95apm, gpio);
	u16 mask, val;

	if (!id95apm_gpio_is_valid(id95apm, offset))
		return -EINVAL;

	mask = 1 << offset;
	val = id95apm_reg16_read(id95apm, ID95APM_PCON_GPIO_DATA);

	if (val & (mask))
		return 1;
	else
		return 0;
}

static int id95apm_gpio_direction_out(struct gpio_chip *chip,
				     unsigned offset, int value)
{
	struct id95apm_gpio *id95apm_gpio = container_of(chip, struct id95apm_gpio, chip);
	struct id95apm *id95apm =  container_of(id95apm_gpio, struct id95apm, gpio);
	int mask, ret;

	if (!id95apm_gpio_is_valid(id95apm, offset))
		return -EINVAL;

	mask = 1 << offset;

	/* Set bit for output */
	ret = id95apm_clrset_bits16(id95apm, ID95APM_PCON_GPIO_DIR, mask, mask);

	/* Set bit value */
	ret = id95apm_clrset_bits16(id95apm, ID95APM_PCON_GPIO_DATA,
				    mask, value << offset);

	return ret;
}

static void id95apm_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct id95apm_gpio *id95apm_gpio = container_of(chip, struct id95apm_gpio, chip);
	struct id95apm *id95apm =  container_of(id95apm_gpio, struct id95apm, gpio);

	if (id95apm_gpio_is_valid(id95apm, offset))
		id95apm_clrset_bits16(id95apm, ID95APM_PCON_GPIO_DATA,
				      1 << offset, value << offset);
}

static struct gpio_chip template_chip = {
	.base			= -1,
	.ngpio			= ID95APM_GPIO_MAX,
	.label			= "id95apm",
	.owner			= THIS_MODULE,
	.request		= id95apm_gpio_request,
	.free			= id95apm_gpio_free,
	.direction_input	= id95apm_gpio_direction_in,
	.get			= id95apm_gpio_get,
	.direction_output	= id95apm_gpio_direction_out,
	.set			= id95apm_gpio_set,
	.can_sleep		= 1,
};

static int id95apm_gpio_probe(struct platform_device *pdev)
{
	struct id95apm *id95apm = platform_get_drvdata(pdev);
	struct id95apm_gpio *id95apm_gpio = &id95apm->gpio;
	int ret;
	u16 mask;

	id95apm_gpio->chip = template_chip;
	id95apm_gpio->chip.dev = &pdev->dev;
#ifdef CONFIG_OF_GPIO
	id95apm_gpio->chip.of_node = id95apm->dev->of_node;
#endif

	mask = 0x001e;	// gpio1 -> gpio4
	if (!id95apm->chip_irq)
		mask |= 0x020;	// shared as irq out

	/* Clear SPECIAL bits to select GPIO functionality */
	id95apm_clrset_bits(id95apm, ID95APM_PCON_GPIO_SPECIAL, mask, 0);

#if !defined(CONFIG_TOUCHSCREEN_ID95APM) && !defined(CONFIG_SENSORS_ID95APM)
	mask |= 0x03c0;	// shared with adc
#endif
	mask |= 0x0400;	// gpio10

	/* Clear OFF bits to select GPIO functionality */
	id95apm_clrset_bits16(id95apm, ID95APM_PCON_GPIO_OFF, mask, 0);

	ret = gpiochip_add(&id95apm_gpio->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Could not register gpiochip, %d\n",
			ret);
	} else {
		platform_set_drvdata(pdev, id95apm);
	}

	return ret;
}

static int id95apm_gpio_remove(struct platform_device *pdev)
{
	struct id95apm *id95apm = platform_get_drvdata(pdev);
	struct id95apm_gpio *id95apm_gpio = &id95apm->gpio;

	return gpiochip_remove(&id95apm_gpio->chip);
}

static struct platform_driver id95apm_gpio_driver = {
	.driver.name	= "id95apm-gpio",
	.driver.owner	= THIS_MODULE,
	.probe		= id95apm_gpio_probe,
	.remove		= id95apm_gpio_remove,
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
