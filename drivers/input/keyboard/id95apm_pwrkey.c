/*
 * Power Switch Detector support for IDT ID95APM PMIC
 *
 * Copyright 2012 Pierluigi Passaro <info@phoenixsoftware.it>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
//#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/mfd/id95apm.h>

#define ID95APM_PWRKEY_NAME "id95apm-pwrkey"

static void id95apm_pwrkey_irq_handler(struct id95apm *id95apm, int irq, void *data)
{
	if (irq == ID95APM_IRQ_SHORT_SW) {
		input_report_key(id95apm->pwrkey.input, id95apm->pwrkey.codes[0], 1);
		id95apm_clrset_bits(id95apm, ID95APM_PCON_SW_STAT, 0xff, 0x01);
		input_report_key(id95apm->pwrkey.input, id95apm->pwrkey.codes[0], 0);
		dev_dbg(id95apm->dev, "ID95APM_IRQ_SHORT_SW on %d\n", id95apm->pwrkey.codes[0]);
	} else if (irq == ID95APM_IRQ_MID_SW) {
		input_report_key(id95apm->pwrkey.input, id95apm->pwrkey.codes[1], 1);
		id95apm_clrset_bits(id95apm, ID95APM_PCON_SW_STAT, 0xff, 0x04);
		input_report_key(id95apm->pwrkey.input, id95apm->pwrkey.codes[1], 0);
		dev_dbg(id95apm->dev, "ID95APM_IRQ_MID_SW on %d\n", id95apm->pwrkey.codes[1]);
	}
}

static int id95apm_pwrkey_probe(struct platform_device *pdev)
{
	struct id95apm *id95apm = platform_get_drvdata(pdev);
	struct id95apm_pwrkey_init *pdata = pdev->dev.platform_data;
	int ret;

	id95apm->pwrkey.input = input_allocate_device();
	if (!id95apm->pwrkey.input) {
		dev_err(&pdev->dev, "allocate input device failed\n");
		ret = -ENOMEM;
		goto err_allocate;
	}

	id95apm->pwrkey.input->name = ID95APM_PWRKEY_NAME;
	id95apm->pwrkey.input->dev.parent = &pdev->dev;

	id95apm->pwrkey.input->id.bustype = BUS_HOST;
	id95apm->pwrkey.input->evbit[0] = BIT_MASK(EV_KEY);

	id95apm->pwrkey.codes[0] = pdata->codes[0];
	id95apm->pwrkey.codes[1] = pdata->codes[1];

	if (id95apm->pwrkey.codes[0]) {
		input_set_capability(id95apm->pwrkey.input, EV_KEY, id95apm->pwrkey.codes[0]);
	}
	if (id95apm->pwrkey.codes[1]) {
		input_set_capability(id95apm->pwrkey.input, EV_KEY, id95apm->pwrkey.codes[1]);
	}

	input_set_drvdata(id95apm->pwrkey.input, id95apm);

	ret = input_register_device(id95apm->pwrkey.input);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto err_register;
	}

	/* Register pwrkey interrupts */
	if (id95apm->pwrkey.codes[0]) {
		id95apm_register_irq(id95apm, ID95APM_IRQ_SHORT_SW, id95apm_pwrkey_irq_handler, NULL);
		dev_info(&pdev->dev, "Registered key %d on ID95APM_IRQ_SHORT_SW\n", id95apm->pwrkey.codes[0]);
	}
	if (id95apm->pwrkey.codes[1]) {
		id95apm_register_irq(id95apm, ID95APM_IRQ_MID_SW, id95apm_pwrkey_irq_handler, NULL);
		dev_info(&pdev->dev, "Registered key %d on ID95APM_IRQ_MID_SW\n", id95apm->pwrkey.codes[1]);
	}
	
	//id95apm_clrset_bits(id95apm, ID95APM_PCON_SW_STAT, 0x05, 0x05);
	id95apm_clrset_bits(id95apm, ID95APM_PCON_SW_IRQ, 0x05, 0x05);

	dev_info(&pdev->dev, "ID95APM powerkey probe\n");

	return 0;

err_register:
	input_free_device(id95apm->tsc.input);
err_allocate:
	return ret;
}

static int id95apm_pwrkey_remove(struct platform_device *pdev)
{
	struct id95apm *id95apm = platform_get_drvdata(pdev);

	id95apm_clrset_bits(id95apm, ID95APM_PCON_SW_IRQ, 0x05, 0x00);

	/* Free touch interrupt */
	id95apm_free_irq(id95apm, ID95APM_IRQ_SHORT_SW);
	id95apm_free_irq(id95apm, ID95APM_IRQ_MID_SW);

	input_unregister_device(id95apm->pwrkey.input);
	input_free_device(id95apm->pwrkey.input);

	return 0;
}

static struct platform_driver id95apm_pwrkey_driver = {
	.driver = {
		.name = ID95APM_PWRKEY_NAME,
	},
	.probe = id95apm_pwrkey_probe,
	.remove = id95apm_pwrkey_remove,
};

static int __init id95apm_pwrkey_init(void)
{
	return platform_driver_register(&id95apm_pwrkey_driver);
}

static void __exit id95apm_pwrkey_exit(void)
{
	platform_driver_unregister(&id95apm_pwrkey_driver);
}

module_init(id95apm_pwrkey_init);
module_exit(id95apm_pwrkey_exit);


MODULE_AUTHOR("Pierluigi Passaro <info@phoenixsoftware.it>");
MODULE_DESCRIPTION("ID95APM power key Driver");
MODULE_LICENSE("GPL");
