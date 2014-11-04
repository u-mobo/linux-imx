/*
 * hwmon support for IDT ID95APM PMIC
 *
 * Copyright 2010 Stefan Roese <sr@denx.de>, DENX Software Engineering
 *
 * This code is based on wm8350-hwmon.c:
 * Copyright (C) 2009 Wolfson Microelectronics plc
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License v2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#include <linux/mfd/id95apm.h>

/*
 * Define the auto-update interval for ADC & temp measurement.
 * Resulting interval is 2^n milli-seconds:
 *
 * ID95APM_HWMON_UPDATE_INTERVAL = 8
 * -> update-interval = 2^8 = 256ms
 *
 * min = 0  -> update-interval =     1ms
 * max = 15 -> update-interval = 32768ms
 */
#define ID95APM_HWMON_UPDATE_INTERVAL	0
#define ID95APM_EN_MEASURE	((ID95APM_HWMON_UPDATE_INTERVAL << 4) | 0x01)

/*
 * Define the reference voltage here. Internal Vref is 2.5V (2500 mV)
 */
#define ID95APM_HWMON_VREF	2500

enum id95apm_auxadc {
	ID95APM_AUX_AUX1 = 0,
	ID95APM_AUX_AUX2 = 1,
	ID95APM_AUX_AUX3 = 2,
	ID95APM_AUX_AUX4 = 3,
	ID95APM_AUX_VSYS = 4,
	ID95APM_AUX_BATT = 5,
	ID95APM_AUX_CHIP_TEMP = 6,
	ID95APM_AUX_CHARGE_CUR = 7,
};

struct id95apm_hwmon {
	struct id95apm *id95apm;
	struct device *classdev;
};

static ssize_t show_name(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "id95apm\n");
}

static const char *input_names[] = {
	[ID95APM_AUX_VSYS]	 = "VSYS",
	[ID95APM_AUX_BATT]	 = "Battery",
	[ID95APM_AUX_CHIP_TEMP]	 = "PMIC",
	[ID95APM_AUX_CHARGE_CUR] = "Charger",
};

static ssize_t show_voltage(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct id95apm_hwmon *hwmon = dev_get_drvdata(dev);
	int channel = to_sensor_dev_attr(attr)->index;
	int ret = 0;
	int reg = 0;

	switch (channel) {
	case ID95APM_AUX_AUX1:
		reg = ID95APM_TSC_X_CH1_RES;
		break;
	case ID95APM_AUX_AUX2:
		reg = ID95APM_TSC_Y_CH2_RES;
		break;
	case ID95APM_AUX_AUX3:
		reg = ID95APM_TSC_CH3_RES;
		break;
	case ID95APM_AUX_AUX4:
		reg = ID95APM_TSC_CH4_RES;
		break;
	case ID95APM_AUX_VSYS:
		ret = id95apm_reg16_read(hwmon->id95apm, ID95APM_TSC_VSYS_RES);
		ret = (ret * 2 * ID95APM_HWMON_VREF) / 4096;
		break;
	case ID95APM_AUX_BATT:
		ret = id95apm_reg16_read(hwmon->id95apm, ID95APM_TSC_BAT_RES);
		ret = (ret * 4200) / 4096;
		break;
	case ID95APM_AUX_CHARGE_CUR:
		ret = id95apm_reg16_read(hwmon->id95apm,
					 ID95APM_TSC_CHARGE_RES);
		ret = ((ret * ID95APM_HWMON_VREF) / 4096);
		break;
	}

	if (reg) {
		ret = id95apm_reg16_read(hwmon->id95apm, reg);
		ret = (ret * ID95APM_HWMON_VREF) / 4096;
	}

	return sprintf(buf, "%d\n", ret);
}

static ssize_t show_chip_temp(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct id95apm_hwmon *hwmon = dev_get_drvdata(dev);
	int ret;

	ret = id95apm_reg16_read(hwmon->id95apm, ID95APM_TSC_TEMP_RES);
	if (ret < 0)
		return ret;

	/* Degrees celsius = 0.115 * ret -  278.257 */
	ret = (115 * ret) - 278257;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t show_label(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int channel = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%s\n", input_names[channel]);
}

#define ID95APM_VOLTAGE(id, name) \
	static SENSOR_DEVICE_ATTR(in##id##_input, S_IRUGO, show_voltage, \
				  NULL, name)

#define ID95APM_NAMED_VOLTAGE(id, name) \
	ID95APM_VOLTAGE(id, name); \
	static SENSOR_DEVICE_ATTR(in##id##_label, S_IRUGO, show_label,	\
				  NULL, name)

static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);

ID95APM_VOLTAGE(0, ID95APM_AUX_AUX1);
ID95APM_VOLTAGE(1, ID95APM_AUX_AUX2);
ID95APM_VOLTAGE(2, ID95APM_AUX_AUX3);
ID95APM_VOLTAGE(3, ID95APM_AUX_AUX4);

ID95APM_NAMED_VOLTAGE(4, ID95APM_AUX_VSYS);
ID95APM_NAMED_VOLTAGE(5, ID95APM_AUX_BATT);
ID95APM_NAMED_VOLTAGE(6, ID95APM_AUX_CHARGE_CUR);

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_chip_temp, NULL,
			  ID95APM_AUX_CHIP_TEMP);
static SENSOR_DEVICE_ATTR(temp1_label, S_IRUGO, show_label, NULL,
			  ID95APM_AUX_CHIP_TEMP);

static struct attribute *id95apm_attributes[] = {
	&dev_attr_name.attr,

	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in3_input.dev_attr.attr,

	&sensor_dev_attr_in4_input.dev_attr.attr,
	&sensor_dev_attr_in4_label.dev_attr.attr,
	&sensor_dev_attr_in5_input.dev_attr.attr,
	&sensor_dev_attr_in5_label.dev_attr.attr,

	&sensor_dev_attr_in6_input.dev_attr.attr,
	&sensor_dev_attr_in6_label.dev_attr.attr,

	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_label.dev_attr.attr,

	NULL
};

static const struct attribute_group id95apm_attr_group = {
	.attrs	= id95apm_attributes,
};

static int id95apm_hwmon_probe(struct platform_device *pdev)
{
	struct id95apm *id95apm = dev_get_drvdata(pdev->dev.parent);
	struct id95apm_hwmon *hwmon;
	int ret;

	hwmon = kzalloc(sizeof(struct id95apm_hwmon), GFP_KERNEL);
	if (!hwmon)
		return -ENOMEM;

	hwmon->id95apm = id95apm;

	ret = sysfs_create_group(&pdev->dev.kobj, &id95apm_attr_group);
	if (ret)
		goto err;

	hwmon->classdev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(hwmon->classdev)) {
		ret = PTR_ERR(hwmon->classdev);
		goto err_sysfs;
	}

	platform_set_drvdata(pdev, hwmon);

	/* Enable TSC/measuring subsystem */
	id95apm_reg_write(id95apm, ID95APM_PCON_TSC_CTRL, 0x01);

	/* Select ADC mode instead TSC (touch screen) mode */
	id95apm_reg_write(id95apm, ID95APM_TSC_MEASURE_CONF, 0x04);

	/*
	 * Configure pin multiplexing:
	 * Select ADC0...ADC3 as ADC instead of GPIO6...9
	 */
	id95apm_clrset_bits16(id95apm, ID95APM_PCON_GPIO_OFF, 0x03c0, 0x03c0);

	/* Enable measuring of each signal (1 milli-second interval) */
	id95apm_reg_write(id95apm, ID95APM_TSC_CH1_MEASURE, ID95APM_EN_MEASURE);
	id95apm_reg_write(id95apm, ID95APM_TSC_CH2_MEASURE, ID95APM_EN_MEASURE);
	id95apm_reg_write(id95apm, ID95APM_TSC_CH3_MEASURE, ID95APM_EN_MEASURE);
	id95apm_reg_write(id95apm, ID95APM_TSC_CH4_MEASURE, ID95APM_EN_MEASURE);
	id95apm_reg_write(id95apm, ID95APM_TSC_VSYS_MEASURE,
			  ID95APM_EN_MEASURE);
	id95apm_reg_write(id95apm, ID95APM_TSC_CHARGE_MEASURE,
			  ID95APM_EN_MEASURE);
	id95apm_reg_write(id95apm, ID95APM_TSC_TEMP_MEASURE,
			  ID95APM_EN_MEASURE);
	id95apm_reg_write(id95apm, ID95APM_TSC_BAT_MEASURE, ID95APM_EN_MEASURE);

	return 0;

err_sysfs:
	sysfs_remove_group(&pdev->dev.kobj, &id95apm_attr_group);
err:
	kfree(hwmon);
	return ret;
}

static int id95apm_hwmon_remove(struct platform_device *pdev)
{
	struct id95apm *id95apm = dev_get_drvdata(pdev->dev.parent);
	struct id95apm_hwmon *hwmon = platform_get_drvdata(pdev);

	/* Disable TSC/measuring subsystem */
	id95apm_reg_write(id95apm, ID95APM_PCON_TSC_CTRL, 0x00);

	hwmon_device_unregister(hwmon->classdev);
	sysfs_remove_group(&pdev->dev.kobj, &id95apm_attr_group);
	platform_set_drvdata(pdev, NULL);
	kfree(hwmon);

	return 0;
}

static struct platform_driver id95apm_hwmon_driver = {
	.probe = id95apm_hwmon_probe,
	.remove = id95apm_hwmon_remove,
	.driver = {
		.name = "id95apm-hwmon",
		.owner = THIS_MODULE,
	},
};

static int __init id95apm_hwmon_init(void)
{
	return platform_driver_register(&id95apm_hwmon_driver);
}
module_init(id95apm_hwmon_init);

static void __exit id95apm_hwmon_exit(void)
{
	platform_driver_unregister(&id95apm_hwmon_driver);
}
module_exit(id95apm_hwmon_exit);

MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_DESCRIPTION("ID95APM Hardware Monitoring");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:id95apm-hwmon");
