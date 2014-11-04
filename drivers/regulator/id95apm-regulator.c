/*
 * id95apm-regulator.c - Voltage and current regulation for the ID95APM PMIC
 *
 * Copyright 2009-2010 Stefan Roese <sr@denx.de>, DENX Software Engineering
 *
 * This code is based on pcf50633-regulator.c:
 * (C) 2006-2008 by Openmoko, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of.h>
#include <linux/mfd/id95apm.h>

#if 1 /* test-only: regulator sysfs write support only for testing */
/*
 * Enabling sysfs write access to the regulators is dangerous!
 * Please make sure that you don't have anything connected to the
 * regulator outputs, that might get damaged by these voltages/currents!
 */
#define CONFIG_REGULATOR_WRITE_ACCESS
#endif

#if 0 /* test-only: don't enable interrupt stuff yet, not fully tested */
/*
 * DC/DC fault interrupts couldn't be tested til now, since the
 * pre 2.0 silicon has this not implemented fully. Only the LDO's have
 * this feature and here the interrupts are tested successfully by
 * shorting the output.
 */
#define CONFIG_USE_IRQ

/*
 * test-only: Set current limit for tests (shortcut) remove from
 * driver release.
 */
#define CONFIG_SHORT_CURCUIT_TEST
#endif

#define ID95APM_VOLTAGE_REGULATOR(_name, _id, _ops, _n)	\
	{					\
		.name = _name, 			\
		.id = _id,			\
		.ops = &_ops,			\
		.n_voltages = _n,		\
		.type = REGULATOR_VOLTAGE, 	\
		.owner = THIS_MODULE, 		\
	}

#define ID95APM_CURRENT_REGULATOR(_name, _id, _ops, _n)	\
	{					\
		.name = _name, 			\
		.id = _id,			\
		.ops = &_ops,			\
		.n_voltages = _n,		\
		.type = REGULATOR_CURRENT, 	\
		.owner = THIS_MODULE, 		\
	}

/* Defines for 0.750 ... 3.700V regulators */
#define ID95APM_3700MV_MIN	750
#define ID95APM_3700MV_MAX	3700
#define ID95APM_3700MV_STEP	25

/* Defines for 4.050 ... 5.600V regulators */
#define ID95APM_5600MV_MIN	4050
#define ID95APM_5600MV_MAX	5600
#define ID95APM_5600MV_STEP	50

/* Defines for 0.78 ... 25.00 mA regulators */
#define ID95APM_25MA_MIN	780
#define ID95APM_25MA_MAX	25000
#define ID95APM_25MA_STEP	780

#define ID95APM_LDO_STEPS	((3700 - 750) / 25)
#define ID95APM_BOOST5_STEPS	((5600 - 4050) / 50)
#define ID95APM_LED_STEPS	(2500 / 78)

#define MAX_BITS(min, max, step)	((max -	min) / step)

struct id95apm_regulator {
	enum id95apm_regulator_id id;
	const u8 reg;
	int min;
	int max;
	int step;
};

static struct id95apm_regulator id95apm_reg[] = {
	{ ID95APM_REGULATOR_LDO0, ID95APM_LDO_150MA_00,
	  ID95APM_3700MV_MIN, ID95APM_3700MV_MAX, ID95APM_3700MV_STEP },
	{ ID95APM_REGULATOR_LDO1, ID95APM_LDO_150MA_01,
	  ID95APM_3700MV_MIN, ID95APM_3700MV_MAX, ID95APM_3700MV_STEP },
	{ ID95APM_REGULATOR_LDO2, ID95APM_LDO_150MA_02,
	  ID95APM_3700MV_MIN, ID95APM_3700MV_MAX, ID95APM_3700MV_STEP },
	{ ID95APM_REGULATOR_LDO3, ID95APM_LDO_50MA_03,
	  ID95APM_3700MV_MIN, ID95APM_3700MV_MAX, ID95APM_3700MV_STEP },
	{ ID95APM_REGULATOR_LDO4, ID95APM_LDO_50MA_04,
	  ID95APM_3700MV_MIN, ID95APM_3700MV_MAX, ID95APM_3700MV_STEP },
	{ ID95APM_REGULATOR_LDO5, ID95APM_LDO_50MA_05,
	  ID95APM_3700MV_MIN, ID95APM_3700MV_MAX, ID95APM_3700MV_STEP },
	{ ID95APM_REGULATOR_LDO6, ID95APM_LDO_50MA_06,
	  ID95APM_3700MV_MIN, ID95APM_3700MV_MAX, ID95APM_3700MV_STEP },
	{ ID95APM_REGULATOR_LDO7, ID95APM_LDO_1MA_LP,
	  0, 0, 0 },		/* no calculation, fixed values here */
	{ ID95APM_REGULATOR_DCDC0, ID95APM_DCDC_BUCK500_0,
	  ID95APM_3700MV_MIN, ID95APM_3700MV_MAX, ID95APM_3700MV_STEP },
	{ ID95APM_REGULATOR_DCDC1, ID95APM_DCDC_BUCK500_1,
	  ID95APM_3700MV_MIN, ID95APM_3700MV_MAX, ID95APM_3700MV_STEP },
	{ ID95APM_REGULATOR_DCDC2, ID95APM_DCDC_BUCK1000,
	  ID95APM_3700MV_MIN, ID95APM_3700MV_MAX, ID95APM_3700MV_STEP },
	{ ID95APM_REGULATOR_DCDC3, ID95APM_DCDC_BOOST5,
	  ID95APM_5600MV_MIN, ID95APM_5600MV_MAX, ID95APM_5600MV_STEP },
	{ ID95APM_REGULATOR_DCDC4, ID95APM_DCDC_LED_BOOST,
	  ID95APM_25MA_MIN, ID95APM_25MA_MAX, ID95APM_25MA_STEP },
};

/* LDO global enable bits (ID95APM_GLOB_LDO_ENABLE) */
static u8 ldo_en[] = { 0x10, 0x20, 0x40, 0x01, 0x02, 0x04, 0x08 };

/* DC/DC global enable bits (ID95APM_GLOB_DCDC_ENABLE) */
static u8 dcdc_en[] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x80 };

static u8 value_to_bits(int id, unsigned int value)
{
	if (value < id95apm_reg[id].min)
		return 0;
	else if (value > id95apm_reg[id].max)
		return MAX_BITS(id95apm_reg[id].min, id95apm_reg[id].max,
				id95apm_reg[id].step);

	value -= id95apm_reg[id].min;
	return value / id95apm_reg[id].step;
}

static unsigned int bits_to_value(int id, u8 bits)
{
	bits = min((int)bits, MAX_BITS(id95apm_reg[id].min, id95apm_reg[id].max,
				       id95apm_reg[id].step));

	return id95apm_reg[id].min + (bits * id95apm_reg[id].step);
}

static inline int id_valid(int id)
{
	if ((id < ID95APM_REGULATOR_LDO0) || (id > ID95APM_REGULATOR_DCDC4))
		return -EINVAL;
	return 0;
}

/*
 * Common routines for voltage and current, so paramters are not
 * called min_uV or min_uA, but min_uX instead
 */
static int id95apm_set_voltage(struct regulator_dev *rdev, int min_uX, int max_uX, unsigned *selector)
{
	struct id95apm *id95apm;
	int id, milli;
	u8 bits, mask = ID95APM_LDO_VSET_MASK;

	id95apm = rdev_get_drvdata(rdev);

	id = rdev_get_id(rdev);
	if (id_valid(id))
		return -EINVAL;

	milli = min_uX / 1000;

	switch (id) {
	case ID95APM_REGULATOR_LDO7:
		switch (milli) {
		case 3000:
			bits = ID95APM_LDO_1MA_LP_30V;
			break;

		case 3300:
			bits = 0;
			break;

		default:
			return -EINVAL;
		}
		break;

	case ID95APM_REGULATOR_DCDC4:
		/*
		 * LED isink, set full scale
		 */
		id95apm_clrset_bits(id95apm, id95apm_reg[id].reg,
				    0, ID95APM_LED_BOOST_SCALE_FULL);
		mask = ID95APM_LED_BOOST_CSET_MASK;

	default:
		/*
		 * All other regulators calculate the bits in this
		 * common routine
		 */
		bits = value_to_bits(id, milli);
	}

	if (selector)
		*selector = bits;

	return id95apm_clrset_bits(id95apm, id95apm_reg[id].reg, mask, bits);
}

static int id95apm_set_value(struct regulator_dev *rdev, int min_uX, int max_uX)
{
	return id95apm_set_voltage(rdev, min_uX, max_uX, NULL);
}

static int id95apm_value(enum id95apm_regulator_id id, u8 bits)
{
	int milli;

	switch (id) {
	case ID95APM_REGULATOR_LDO7:
		if (bits & ID95APM_LDO_1MA_LP_30V)
			milli = 3000;
		else
			milli = 3300;
		break;

	default:
		/*
		 * All other regulators calculate the bits in this
		 * common routine
		 */
		milli = bits_to_value(id, bits);
	}

	return milli * 1000;
}

static int id95apm_get_value(struct regulator_dev *rdev)
{
	struct id95apm *id95apm;
	int id;
	u8 bits, mask;

	id95apm = rdev_get_drvdata(rdev);

	id = rdev_get_id(rdev);
	if (id_valid(id))
		return -EINVAL;

	if (id == ID95APM_REGULATOR_DCDC4)
		mask = ID95APM_LED_BOOST_CSET_MASK;
	else
		mask = ID95APM_LDO_VSET_MASK;

	bits = id95apm_reg_read(id95apm, id95apm_reg[id].reg) & mask;

	return id95apm_value(id, bits);
}

static int id95apm_list_value(struct regulator_dev *rdev, unsigned int index)
{
	struct id95apm *id95apm;
	int id;

	id95apm = rdev_get_drvdata(rdev);

	id = rdev_get_id(rdev);
	if (id_valid(id))
		return -EINVAL;

	return id95apm_value(id, index);
}

static int id95apm_enable(struct regulator_dev *rdev)
{
	struct id95apm *id95apm = rdev_get_drvdata(rdev);
	int id;

	id = rdev_get_id(rdev);
	if (id_valid(id))
		return -EINVAL;

	/* LDO8 (low power, 1mA) is always enabled */
	if (id == ID95APM_REGULATOR_LDO7)
		return 0;

	/* Globally enable the LDO/DCDC and unmask interrupt */
	if (id <= ID95APM_REGULATOR_LDO7) {
#ifdef CONFIG_SHORT_CURCUIT_TEST
		id95apm_clrset_bits(id95apm, id95apm_reg[id].reg + 1,
				    0, 0x3);
#endif
		id95apm_clrset_bits(id95apm, ID95APM_GLOB_LDO_ENABLE,
				    0, ldo_en[id]);
#ifdef CONFIG_USE_IRQ
		id95apm_clrset_bits(id95apm, ID95APM_LDO_FAULT_IRQ_EN,
				    0, ldo_en[id]);
#endif
	} else {
#ifdef CONFIG_SHORT_CURCUIT_TEST
		id95apm_clrset_bits(id95apm, id95apm_reg[id].reg + 1,
				    0x0c, 0x00);
#endif
		id95apm_clrset_bits(id95apm, ID95APM_GLOB_DCDC_ENABLE,
				    0, dcdc_en[id - ID95APM_REGULATOR_DCDC0]);
#ifdef CONFIG_USE_IRQ
		/* Only the first 3 DCDC's have IRQ's */
		if ((id >= ID95APM_REGULATOR_DCDC0) &&
		    (id <= ID95APM_REGULATOR_DCDC2)) {
			id95apm_clrset_bits(id95apm, ID95APM_PCON_DCDC_IRQ_EN,
					    0, dcdc_en[id - ID95APM_REGULATOR_DCDC0]);
		}
#endif
	}

	return id95apm_clrset_bits(id95apm, id95apm_reg[id].reg,
				   0, ID95APM_LDO_EN);
}

static int id95apm_disable(struct regulator_dev *rdev)
{
	struct id95apm *id95apm = rdev_get_drvdata(rdev);
	int id;

	id = rdev_get_id(rdev);
	if (id_valid(id))
		return -EINVAL;

	/* LDO8 (low power, 1mA) is always enabled */
	if (id == ID95APM_REGULATOR_LDO7)
		return 0;

	/* Globally disable the LDO/DCDC and mask interrupt */
	if (id <= ID95APM_REGULATOR_LDO7) {
		id95apm_clrset_bits(id95apm, ID95APM_GLOB_LDO_ENABLE,
				    ldo_en[id], 0);
#ifdef CONFIG_USE_IRQ
		id95apm_clrset_bits(id95apm, ID95APM_LDO_FAULT_IRQ_EN,
				    ldo_en[id], 0);
#endif
	} else {
		id95apm_clrset_bits(id95apm, ID95APM_GLOB_DCDC_ENABLE,
				    dcdc_en[id - ID95APM_REGULATOR_DCDC0], 0);
#ifdef CONFIG_USE_IRQ
		/* Only the first 3 DCDC's have IRQ's */
		if ((id >= ID95APM_REGULATOR_DCDC0) &&
		    (id <= ID95APM_REGULATOR_DCDC2)) {
			id95apm_clrset_bits(id95apm, ID95APM_PCON_DCDC_IRQ_EN,
					    dcdc_en[id - ID95APM_REGULATOR_DCDC0], 0);
		}
#endif
	}

	return id95apm_clrset_bits(id95apm, id95apm_reg[id].reg,
				   ID95APM_LDO_EN, 0);
}

static int id95apm_is_enabled(struct regulator_dev *rdev)
{
	struct id95apm *id95apm = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	u8 en;

	id = rdev_get_id(rdev);
	if (id_valid(id))
		return -EINVAL;

	/* LDO8 (low power, 1mA) is always enabled */
	if (id == ID95APM_REGULATOR_LDO7)
		return 1;

	/* Check global enable register */
	if (id <= ID95APM_REGULATOR_LDO7) {
		en = id95apm_reg_read(id95apm, ID95APM_GLOB_LDO_ENABLE);
		en &= ldo_en[id];
	} else {
		en = id95apm_reg_read(id95apm, ID95APM_GLOB_DCDC_ENABLE);
		en &= dcdc_en[id - ID95APM_REGULATOR_DCDC0];
	}

	/* Global enable *and* local enable need to be set */
	return (en && (id95apm_reg_read(id95apm, id95apm_reg[id].reg) &
		       ID95APM_LDO_EN));
}

/* Operations permitted on LDOx */
static struct regulator_ops id95apm_ldo_ops = {
	.set_voltage = id95apm_set_voltage,
	.get_voltage = id95apm_get_value,
	.list_voltage = id95apm_list_value,
	.enable = id95apm_enable,
	.disable = id95apm_disable,
	.is_enabled = id95apm_is_enabled,
};

static struct regulator_ops id95apm_isink_ops = {
	.set_current_limit = id95apm_set_value,
	.get_current_limit = id95apm_get_value,
	.enable = id95apm_enable,
	.disable = id95apm_disable,
	.is_enabled = id95apm_is_enabled,
};


static struct regulator_desc regulators[] = {
	/*
	 * LDO
	 */

	/* 3 * 150mA */
	[ID95APM_REGULATOR_LDO0] =
		ID95APM_VOLTAGE_REGULATOR("ldo0", ID95APM_REGULATOR_LDO0,
				  id95apm_ldo_ops, ID95APM_LDO_STEPS),
	[ID95APM_REGULATOR_LDO1] =
		ID95APM_VOLTAGE_REGULATOR("ldo1", ID95APM_REGULATOR_LDO1,
				  id95apm_ldo_ops, ID95APM_LDO_STEPS),
	[ID95APM_REGULATOR_LDO2] =
		ID95APM_VOLTAGE_REGULATOR("ldo2", ID95APM_REGULATOR_LDO2,
				  id95apm_ldo_ops, ID95APM_LDO_STEPS),

	/* 4 * 50mA */
	[ID95APM_REGULATOR_LDO3] =
		ID95APM_VOLTAGE_REGULATOR("ldo3", ID95APM_REGULATOR_LDO3,
				  id95apm_ldo_ops, ID95APM_LDO_STEPS),
	[ID95APM_REGULATOR_LDO4] =
		ID95APM_VOLTAGE_REGULATOR("ldo4", ID95APM_REGULATOR_LDO4,
				  id95apm_ldo_ops, ID95APM_LDO_STEPS),
	[ID95APM_REGULATOR_LDO5] =
		ID95APM_VOLTAGE_REGULATOR("ldo5", ID95APM_REGULATOR_LDO5,
				  id95apm_ldo_ops, ID95APM_LDO_STEPS),
	[ID95APM_REGULATOR_LDO6] =
		ID95APM_VOLTAGE_REGULATOR("ldo6", ID95APM_REGULATOR_LDO6,
				  id95apm_ldo_ops, ID95APM_LDO_STEPS),

	/* 1 * 1mA */
	[ID95APM_REGULATOR_LDO7] =
		ID95APM_VOLTAGE_REGULATOR("ldo7", ID95APM_REGULATOR_LDO7,
				  id95apm_ldo_ops, 2),

	/*
	 * DC/DC
	 */

	/* 2 * Buck 500 */
	[ID95APM_REGULATOR_DCDC0] =
		ID95APM_VOLTAGE_REGULATOR("dcdc0", ID95APM_REGULATOR_DCDC0,
				  id95apm_ldo_ops, ID95APM_LDO_STEPS),
	[ID95APM_REGULATOR_DCDC1] =
		ID95APM_VOLTAGE_REGULATOR("dcdc1", ID95APM_REGULATOR_DCDC1,
				  id95apm_ldo_ops, ID95APM_LDO_STEPS),

	/* Buck 1000 */
	[ID95APM_REGULATOR_DCDC2] =
		ID95APM_VOLTAGE_REGULATOR("dcdc2", ID95APM_REGULATOR_DCDC2,
				  id95apm_ldo_ops, ID95APM_LDO_STEPS),

	/* Boost 5 */
	[ID95APM_REGULATOR_DCDC3] =
		ID95APM_VOLTAGE_REGULATOR("dcdc3", ID95APM_REGULATOR_DCDC3,
				  id95apm_ldo_ops, ID95APM_BOOST5_STEPS),

	/* LED Boost current sink */
	[ID95APM_REGULATOR_DCDC4] =
		ID95APM_CURRENT_REGULATOR("dcdc4", ID95APM_REGULATOR_DCDC4,
				  id95apm_isink_ops, ID95APM_LED_STEPS),
};

#ifdef CONFIG_REGULATOR_WRITE_ACCESS
/*
 * Write access for test-purpose:
 *
 * echo 1800000 > /sys/class/i2c-dev/i2c-0/device/bus/devices/id95apm-regltr.0/set
 */
static ssize_t set(struct device *dev, struct device_attribute *attr,
		   const char *buf, size_t count)
{
	struct regulator_dev *rdev = dev_get_drvdata(dev);
	long val;

	if (strict_strtol(buf, 10, &val) != 0)
		return count;

	id95apm_set_value(rdev, val, 0);
	id95apm_enable(rdev);

	return count;
}

static DEVICE_ATTR(set, 0222, NULL, set);

static struct device_attribute *attributes[] = {
	&dev_attr_set,
};

static int id95apm_sysfs_write_access(struct platform_device *pdev)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		ret = device_create_file(&pdev->dev, attributes[i]);
		if (ret != 0)
			goto err;
	}

	return 0;

err:
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(&pdev->dev, attributes[i]);

	return ret;
}
#endif

#ifdef CONFIG_USE_IRQ
static void ldo_handler(struct id95apm *id95apm, int irq, void *data)
{
	struct regulator_dev *rdev;
	u8 val;
	int id;

	/*
	 * Determine the interrupting LDO
	 */
	val = id95apm_reg_read(id95apm, ID95APM_LDO_EXT_PG);
	for (id = ID95APM_REGULATOR_LDO0; id <= ID95APM_REGULATOR_LDO6; id++) {
		rdev = id95apm->rdev[id];
		if (id95apm_is_enabled(rdev) && !(val & (1 << id))) {
			printk("%s: Fault on LDO %d detected!\n", __func__, id); // test-only
			regulator_notifier_call_chain(rdev,
						      REGULATOR_EVENT_FAIL,
						      id95apm);
		}
	}
}

static void dcdc_handler(struct id95apm *id95apm, int irq, void *data)
{
	struct regulator_dev *rdev;
	u8 val;
	int id;

	/*
	 * Determine the interrupting DCDC
	 */
	for (id = 0; id <= 2; id++) {
		val = id95apm_reg_read(id95apm, ID95APM_DCDC_BUCK500_0 +
				       (id * 2) + 1);
		printk("%s: irq=%d val=%02x!!!\n", __func__, irq, val); // test-only
		if (val & ID95APM_DCDC_BUCK_FAULT) {
			printk("%s: Fault on DCDC %d detected!!!\n", __func__, id); // test-only
			rdev = id95apm->rdev[id + ID95APM_REGULATOR_DCDC0];
			regulator_notifier_call_chain(rdev,
						      REGULATOR_EVENT_FAIL,
						      id95apm);
		}
	}
}
#endif

#ifdef CONFIG_OF
static struct of_regulator_match id95apm_reg_matches[] = {
	{ .name = "ldo0", },
	{ .name = "ldo1", },
	{ .name = "ldo2", },
	{ .name = "ldo3", },
	{ .name = "ldo4", },
	{ .name = "ldo5", },
	{ .name = "ldo6", },
	{ .name = "ldo7", },
	{ .name = "dcdc0", },
	{ .name = "dcdc1", },
	{ .name = "dcdc2", },
	{ .name = "dcdc3", },
	{ .name = "dcdc4", },
};

int id95apm_regulator_dt_init(struct id95apm *id95apm, struct regulator_config *config, int regid)
{
	struct device_node *np, *parent;
	int ret;

	np = id95apm->dev->of_node;
	if (!np) {
		dev_err(id95apm->dev, "main node not found\n");
		return -EINVAL;
	}

	parent = of_get_child_by_name(np, "regulators");
	if (!parent) {
		dev_err(id95apm->dev, "regulators node not found\n");
		return -EINVAL;
	}
	ret = of_regulator_match(id95apm->dev, parent, &id95apm_reg_matches[regid], 1);
	of_node_put(parent);
	if (ret < 0) {
		dev_err(id95apm->dev, "Error parsing regulator init data: %d\n", ret);
		return ret;
	}

	config->init_data = id95apm_reg_matches[regid].init_data;
	config->of_node = id95apm_reg_matches[regid].of_node;

	if (!config->of_node)
		return -ENODEV;

	return 0;
}
#else
int id95apm_regulator_dt_init(struct id95apm *id95apm, struct regulator_config *config, int regid);
{
	config->init_data = id95apm->pdata.reg_init_data[regid];
	return 0;
}
#endif /* CONFIG_OF */

static int id95apm_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;
	struct id95apm *id95apm;
	struct regulator_config config = { };

	/* Already set by core driver */
	id95apm = platform_get_drvdata(pdev);

	config.dev = &pdev->dev;
	config.driver_data = id95apm;

	id95apm_regulator_dt_init(id95apm, &config, pdev->id);

	rdev = regulator_register(&regulators[pdev->id], &config);
	if (IS_ERR(rdev))
		return PTR_ERR(rdev);

	platform_set_drvdata(pdev, rdev);
	id95apm->rdev[pdev->id] = rdev;

#ifdef CONFIG_REGULATOR_WRITE_ACCESS
	id95apm_sysfs_write_access(pdev);
#endif

#ifdef CONFIG_USE_IRQ
	/*
	 * Register interrupt handler
	 */
	if ((pdev->id >= ID95APM_REGULATOR_LDO0) &&
	    (pdev->id <= ID95APM_REGULATOR_LDO6)) {
		id95apm_register_irq(id95apm, ID95APM_IRQ_LDO,
				     ldo_handler, NULL);
	}

	if ((pdev->id >= ID95APM_REGULATOR_DCDC0) &&
	    (pdev->id <= ID95APM_REGULATOR_DCDC2)) {
		id95apm_register_irq(id95apm, ID95APM_IRQ_DCDC,
				     dcdc_handler, NULL);
	}
#endif

	return 0;
}

static int id95apm_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
#ifdef CONFIG_USE_IRQ
	struct id95apm *id95apm = dev_get_drvdata(pdev->dev.parent);

	/*
	 * Free interrupt handler
	 */
	if ((pdev->id >= ID95APM_REGULATOR_LDO0) &&
	    (pdev->id <= ID95APM_REGULATOR_LDO6))
		id95apm_free_irq(id95apm, ID95APM_IRQ_LDO);

	if ((pdev->id >= ID95APM_REGULATOR_DCDC0) &&
	    (pdev->id <= ID95APM_REGULATOR_DCDC2))
		id95apm_free_irq(id95apm, ID95APM_IRQ_DCDC);
#endif

	regulator_unregister(rdev);

	return 0;
}

static struct platform_driver id95apm_regulator_driver = {
	.driver = {
		.name = "id95apm-regltr", /* "id95apm-regulator" is too long */
	},
	.probe = id95apm_regulator_probe,
	.remove = id95apm_regulator_remove,
};

static int __init id95apm_regulator_init(void)
{
	return platform_driver_register(&id95apm_regulator_driver);
}
subsys_initcall(id95apm_regulator_init);

static void __exit id95apm_regulator_exit(void)
{
	platform_driver_unregister(&id95apm_regulator_driver);
}
module_exit(id95apm_regulator_exit);

MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_DESCRIPTION("ID95APM regulator driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:id95apm-regulator");
