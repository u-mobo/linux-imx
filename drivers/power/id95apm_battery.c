/*
 * Battery driver for ID95APM PMIC
 *
 * Copyright 2009-2010 Stefan Roese <sr@denx.de>, DENX Software Engineering
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/power_supply.h>
#include <linux/mfd/core.h>
#include <linux/mfd/id95apm.h>
#include <linux/slab.h>

#define UEVENT_UPDATE_INTERVALL_MS	1000

static struct timer_list uevent_timer;

static void uevent_timer_fn(long unsigned int data)
{
	struct id95apm *id95apm = (struct id95apm *)data;

	power_supply_changed(id95apm->battery);
	power_supply_changed(id95apm->charger);

	mod_timer(&uevent_timer, jiffies + msecs_to_jiffies(UEVENT_UPDATE_INTERVALL_MS));
}

static int id95apm_battery_status(struct id95apm *id95apm)
{
	int state1;
	int state2;

	state1 = id95apm_reg_read(id95apm, ID95APM_CHGR_OP_STAT);
	state2 = id95apm_reg_read(id95apm, ID95APM_CHGR_BLOCK_FAULT);

	if (!(state1 & ID95APM_CHGR_OP_STAT_ADAPTER))
		return POWER_SUPPLY_STATUS_DISCHARGING;
	if (state2 & ID95APM_CHGR_BLOCK_FAULT_CHRG)
		return POWER_SUPPLY_STATUS_CHARGING;
	if (state2 & ID95APM_CHGR_BLOCK_FAULT_DONE)
		return POWER_SUPPLY_STATUS_FULL;
	if ((state1 & ID95APM_CHGR_OP_STAT_CHMODE) == ID95APM_CHGR_OP_STAT_HOLD)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	return POWER_SUPPLY_STATUS_UNKNOWN;
}

static int id95apm_battery_health(struct id95apm *id95apm)
{
	int state1;
	int state2;

	state1 = id95apm_reg_read(id95apm, ID95APM_CHGR_OP_STAT);
	state2 = id95apm_reg_read(id95apm, ID95APM_CHGR_BLOCK_FAULT);

	/* Is NTC enabled? */
	if (!(state2 & ID95APM_CHGR_BLOCK_FAULT_NO_NTC)) {
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30))
		if (state1 & ID95APM_CHGR_OP_STAT_COLD)
			return POWER_SUPPLY_HEALTH_COLD;
#endif
		if (state1 & ID95APM_CHGR_OP_STAT_HOT)
			return POWER_SUPPLY_HEALTH_OVERHEAT;
	}
	if (state1 & ID95APM_CHGR_OP_STAT_FAULT)
		return POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;

	return POWER_SUPPLY_HEALTH_GOOD;
}

static int id95apm_battery_current(struct id95apm *id95apm)
{
	int state;
	int cur;

	state = id95apm_reg_read(id95apm, ID95APM_CHGR_OP_STAT);

	if ((state & ID95APM_CHGR_OP_STAT_CHMODE) ==
	    ID95APM_CHGR_OP_STAT_CONST_CUR) {
		cur = id95apm_reg_read(id95apm, ID95APM_CHGR_CUR_VOL_CTRL) &
			ID95APM_CHGR_CUR_VOL_CTRL_CUR;
		cur = min(cur, 1);
		if (cur == 0)
			cur = 1;

		/* Return current in uA */
		return cur * 100 * 1000;
	}

	return 0;
}

static int id95apm_battery_voltage(struct id95apm *id95apm)
{
	int ret;

	if (id95apm_battery_status(id95apm) == POWER_SUPPLY_STATUS_UNKNOWN)
		return 0;

	/* Calculate voltage in mV */
	ret = id95apm_reg16_read(id95apm, ID95APM_TSC_BAT_RES);
	ret = (ret * 4200) / 4096;

	/* Return voltage in uV */
	return ret * 1000;
}

static int id95apm_battery_temp(struct id95apm *id95apm)
{
	int ret;

	/*
	 * Calculate temperature in tenths of a degree Centigrade
	 * t[ C] = VTEMP * 0.114822 - 278.2565 (refer to page 121)
	 * t[dC] = VTEMP * 1.14822 - 2782.565
	 */
	ret = id95apm_reg16_read(id95apm, ID95APM_TSC_TEMP_RES);
	return ((ret * 300999 - 729432719) >> 18);
}

static int id95apm_battery_get_property(struct power_supply *psy,
					enum power_supply_property prop,
					union power_supply_propval *val)
{
	struct id95apm *id95apm = dev_get_drvdata(psy->dev->parent);
	int ret = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = id95apm_battery_status(id95apm);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = id95apm_battery_health(id95apm);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = id95apm_battery_current(id95apm);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = id95apm_battery_voltage(id95apm);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = id95apm_battery_voltage(id95apm) / 42000;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = (id95apm_battery_status(id95apm) != POWER_SUPPLY_STATUS_UNKNOWN);
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = id95apm_battery_temp(id95apm);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static enum power_supply_property id95apm_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_TEMP,
};

static int id95apm_charger_status(struct id95apm *id95apm)
{
	int state1;

	state1 = id95apm_reg_read(id95apm, ID95APM_CHGR_OP_STAT);

	return state1 & ID95APM_CHGR_OP_STAT_ADAPTER;
}

static int id95apm_charger_get_property(struct power_supply *psy,
					enum power_supply_property prop,
					union power_supply_propval *val)
{
	struct id95apm *id95apm = dev_get_drvdata(psy->dev->parent);
	int ret = 0;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = id95apm_charger_status(id95apm);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static enum power_supply_property id95apm_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static void charger_handler(struct id95apm *id95apm, int irq, void *data)
{
	u8 val;

	val = id95apm_reg_read(id95apm, ID95APM_CHGR_IRQ_STAT);

	if (val & ID95APM_CHGR_IRQ_ADAPTER) {
		dev_dbg(id95apm->dev,
			"Battery adapter input status changed!\n");
	}

	if (val & ID95APM_CHGR_IRQ_CUR_LIM) {
		dev_dbg(id95apm->dev,
			"Battery current limit status changed!\n");
		power_supply_changed(id95apm->battery);
	}

	if (val & ID95APM_CHGR_IRQ_CHRG_STATE) {
		dev_dbg(id95apm->dev, "Battery charging state changed!\n");
		power_supply_changed(id95apm->battery);
	}

	/*
	 * Acknowledge interrupts
	 */
	id95apm_reg_write(id95apm, ID95APM_CHGR_IRQ_STAT, val);
}

static int id95apm_battery_probe(struct platform_device *pdev)
{
	struct id95apm *id95apm;
	struct power_supply *battery;
	struct power_supply *charger;
	int ret;

	/* Already set by core driver */
	id95apm = platform_get_drvdata(pdev);

	battery = kzalloc(sizeof(*battery), GFP_KERNEL);
	if (!battery) {
		ret = -ENOMEM;
		goto fail;
	}
	id95apm->battery = battery;

	battery->name = "id95apm-battery";
	battery->type = POWER_SUPPLY_TYPE_BATTERY;
	battery->properties = id95apm_battery_props;
	battery->num_properties = ARRAY_SIZE(id95apm_battery_props);
	battery->get_property = id95apm_battery_get_property;
	ret = power_supply_register(&pdev->dev, battery);
	if (ret) {
		dev_err(id95apm->dev, "failed to register battery\n");
		goto fail;
	}

	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger) {
		ret = -ENOMEM;
		goto fail;
	}
	id95apm->charger = charger;

	charger->name = "id95apm-charger";
	charger->type = POWER_SUPPLY_TYPE_MAINS;
	charger->properties = id95apm_charger_props;
	charger->num_properties = ARRAY_SIZE(id95apm_charger_props);
	charger->get_property = id95apm_charger_get_property;
	ret = power_supply_register(&pdev->dev, charger);
	if (ret) {
		dev_err(id95apm->dev, "failed to register charger\n");
		goto fail;
	}

	/* Set 500mA charging current */
	id95apm_reg_write(id95apm, ID95APM_CHGR_CUR_VOL_CTRL, 0x25);

	/* Enable TSC/measuring subsystem */
	id95apm_reg_write(id95apm, ID95APM_PCON_TSC_CTRL, 0x01);

	/* Select ADC mode instead TSC (touch screen) mode */
	id95apm_reg_write(id95apm, ID95APM_TSC_MEASURE_CONF, 0x04);

	id95apm_reg_write(id95apm, ID95APM_TSC_CHARGE_MEASURE, 0x01);
	id95apm_reg_write(id95apm, ID95APM_TSC_TEMP_MEASURE, 0x01);
	id95apm_reg_write(id95apm, ID95APM_TSC_BAT_MEASURE, 0x01);

	/*
	 * Register interrupt handler
	 */
	id95apm_register_irq(id95apm, ID95APM_IRQ_CHRG, charger_handler, NULL);

	/*
	 * Enable interrupts
	 */
	id95apm_reg_write(id95apm, ID95APM_CHGR_IRQ_EN, ID95APM_CHGR_IRQ_MASK);

	setup_timer(&uevent_timer, uevent_timer_fn, (long unsigned int)id95apm);
	mod_timer(&uevent_timer, jiffies + msecs_to_jiffies(UEVENT_UPDATE_INTERVALL_MS));

	return 0;

fail:
	kfree(id95apm->battery);

	return ret;
}

static int id95apm_battery_remove(struct platform_device *pdev)
{
	struct id95apm *id95apm = platform_get_drvdata(pdev);

	id95apm_free_irq(id95apm, ID95APM_IRQ_CHRG);
	power_supply_unregister(id95apm->battery);
	kfree(id95apm->battery);

	del_timer(&uevent_timer);

	return 0;
}

static struct platform_driver id95apm_battery_driver = {
	.probe = id95apm_battery_probe,
	.remove = __devexit_p(id95apm_battery_remove),
	.driver = {
		.name = "id95apm-battery",
	},
};

static int __init id95apm_battery_init(void)
{
	return platform_driver_register(&id95apm_battery_driver);
}
/* Init early because of potentionally already pending irq's */
subsys_initcall(id95apm_battery_init);

static void __exit id95apm_battery_exit(void)
{
	platform_driver_unregister(&id95apm_battery_driver);
}
module_exit(id95apm_battery_exit);

MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_DESCRIPTION("ID95APM battery driver");
MODULE_LICENSE("GPL");
