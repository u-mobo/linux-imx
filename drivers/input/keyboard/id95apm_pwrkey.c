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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/mfd/id95apm.h>
#include <linux/of.h>

#define ID95APM_PWRKEY_NAME "id95apm-pwrkey"

static void id95apm_shortkey_monitor(struct work_struct *work)
{
	struct id95apm_pwrkey *pwrkey = container_of(work, struct id95apm_pwrkey, shortkey_monitor.work);
	struct id95apm *id95apm = container_of(pwrkey, struct id95apm, pwrkey);

	mutex_lock(&pwrkey->shortkey_mutex);
	input_report_key(pwrkey->input, pwrkey->codes[pwrkey->shortkey_counter], 1);
	input_report_key(pwrkey->input, pwrkey->codes[pwrkey->shortkey_counter], 0);
	input_sync(pwrkey->input);
	dev_dbg(id95apm->dev, "id95apm_shortkey_monitor on key[%d] %d\n",
		pwrkey->shortkey_counter,
		pwrkey->codes[pwrkey->shortkey_counter]);
	pwrkey->shortkey_counter = 0;
	mutex_unlock(&pwrkey->shortkey_mutex);
}

static void id95apm_pwrkey_irq_handler(struct id95apm *id95apm, int irq, void *data)
{
	if (irq == ID95APM_IRQ_SHORT_SW) {
		mutex_lock(&id95apm->pwrkey.shortkey_mutex);
		if (!id95apm->pwrkey.shortkey_counter)
			queue_delayed_work(id95apm->pwrkey.shortkey_workq,
				&id95apm->pwrkey.shortkey_monitor,
				id95apm->pwrkey.shortkey_monitor_intervall);
		id95apm->pwrkey.shortkey_counter++;
		mutex_unlock(&id95apm->pwrkey.shortkey_mutex);

		id95apm_clrset_bits(id95apm, ID95APM_PCON_SW_STAT, 0xff, 0x01);
		dev_dbg(id95apm->dev, "ID95APM_IRQ_SHORT_SW on key[%d] %d\n",
			id95apm->pwrkey.shortkey_counter,
			id95apm->pwrkey.codes[id95apm->pwrkey.shortkey_counter]);
	} else if (irq == ID95APM_IRQ_MID_SW) {
		input_report_key(id95apm->pwrkey.input, id95apm->pwrkey.codes[0], 1);
		id95apm_clrset_bits(id95apm, ID95APM_PCON_SW_STAT, 0xff, 0x04);
		input_report_key(id95apm->pwrkey.input, id95apm->pwrkey.codes[0], 0);
		input_sync(id95apm->pwrkey.input);
		dev_dbg(id95apm->dev, "ID95APM_IRQ_MID_SW on key %d\n", id95apm->pwrkey.codes[0]);
	}
}

#ifdef CONFIG_OF
static int id95apm_pwrkey_set_pdata_from_of(struct id95apm *id95apm)
{
	struct device_node *node = id95apm->dev->of_node;
	struct id95apm_platform_data *pdata = &id95apm->pdata;
	int count;

	if (!node) {
		dev_err(id95apm->dev, "main node not found\n");
		return -EINVAL;
	}

	/* determine the number of brightness levels */
	if (of_find_property(node, "pwrkey-codes", &count)) {
		count /= sizeof(u32);
		if (count > ID95APM_PWRKEY_NUM_CODES) {
			count = ID95APM_PWRKEY_NUM_CODES;
			dev_err(id95apm->dev, "truncating key codes\n");
		}
		of_property_read_u32_array(node, "pwrkey-codes", pdata->pwrkey_init.codes, count);
	}

	return 0;
}
#else
static void id95apm_pwrkey_set_pdata_from_of(struct id95apm *id95apm)
{
	return -1;
}
#endif

static int id95apm_pwrkey_probe(struct platform_device *pdev)
{
	struct id95apm *id95apm = platform_get_drvdata(pdev);
	struct id95apm_pwrkey_init *pdata = pdev->dev.platform_data;
	int i, ret;

	mutex_init(&id95apm->pwrkey.shortkey_mutex);
	id95apm->pwrkey.shortkey_workq = create_singlethread_workqueue("kid95apmshortkey");
	if (id95apm->pwrkey.shortkey_workq == NULL) {
		dev_err(&pdev->dev, "create singlethread workqueue failed\n");
		return -EINVAL;
	}
	INIT_DELAYED_WORK(&id95apm->pwrkey.shortkey_monitor, id95apm_shortkey_monitor);
	id95apm->pwrkey.shortkey_counter = 0;

	id95apm->pwrkey.input = input_allocate_device();
	if (!id95apm->pwrkey.input) {
		dev_err(&pdev->dev, "allocate input device failed\n");
		ret = -ENOMEM;
		goto err_allocate;
	}

	id95apm->pwrkey.input->name = ID95APM_PWRKEY_NAME;
	id95apm->pwrkey.input->dev.parent = &pdev->dev;
	id95apm->pwrkey.input->id.bustype = BUS_HOST;

	/* setup input device */
	__set_bit(EV_KEY, id95apm->pwrkey.input->evbit);

	id95apm_pwrkey_set_pdata_from_of(id95apm);
	memcpy(id95apm->pwrkey.codes, pdata->codes, sizeof(pdata->codes));
	if (pdata->shortkey_monitor_intervall)
		id95apm->pwrkey.shortkey_monitor_intervall = pdata->shortkey_monitor_intervall;
	else
		id95apm->pwrkey.shortkey_monitor_intervall = msecs_to_jiffies(500);

	for (i = 0; i < ID95APM_PWRKEY_NUM_CODES; i++)
		if (id95apm->pwrkey.codes[i])
			input_set_capability(id95apm->pwrkey.input, EV_KEY, id95apm->pwrkey.codes[i]);

	input_set_drvdata(id95apm->pwrkey.input, id95apm);

	ret = input_register_device(id95apm->pwrkey.input);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto err_register;
	}

	id95apm_register_irq(id95apm, ID95APM_IRQ_SHORT_SW, id95apm_pwrkey_irq_handler, NULL);
	dev_info(&pdev->dev, "Registered keys %d, %d on ID95APM_IRQ_SHORT_SW\n", id95apm->pwrkey.codes[1], id95apm->pwrkey.codes[2]);
	if (id95apm->pwrkey.codes[0]) {
		/* Register medium press interrupts */
		id95apm_register_irq(id95apm, ID95APM_IRQ_MID_SW, id95apm_pwrkey_irq_handler, NULL);
		dev_info(&pdev->dev, "Registered key %d on ID95APM_IRQ_MID_SW\n", id95apm->pwrkey.codes[0]);
	}
	
	//id95apm_clrset_bits(id95apm, ID95APM_PCON_SW_STAT, 0x05, 0x05);
	id95apm_clrset_bits(id95apm, ID95APM_PCON_SW_IRQ, 0x05, 0x05);

	dev_info(&pdev->dev, "ID95APM powerkey probe\n");

	return 0;

err_register:
	input_free_device(id95apm->pwrkey.input);
err_allocate:
	destroy_workqueue(id95apm->pwrkey.shortkey_workq);
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
	destroy_workqueue(id95apm->pwrkey.shortkey_workq);

	return 0;
}

static struct platform_driver id95apm_pwrkey_driver = {
	.driver = {
		.name = ID95APM_PWRKEY_NAME,
		.owner	= THIS_MODULE,
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


MODULE_AUTHOR("Pierluigi Passaro <p.passaro@u-mobo.com>");
MODULE_DESCRIPTION("ID95APM power key Driver");
MODULE_LICENSE("GPL");
