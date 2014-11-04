/*
 * Device access for IDT ID95APM PMIC
 *
 * Copyright 2009-2010 Stefan Roese <sr@denx.de>, DENX Software Engineering
 *
 * This code is based on wm8400-core.c & wm831x-core.c:
 * Copyright 2008 & 2009 Wolfson Microelectronics PLC.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/mfd/core.h>
#include <linux/mfd/id95apm.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>

/* test-only: sysfs support only for testing purpose */
#define CONFIG_TESTING_SYSFS

static DEFINE_MUTEX(io_mutex);

static struct id95apm *glob_id95apm;
static struct platform_device *pdevs[ID95APM_NUM_PDEVS];
static int pdev_count;

struct id95apm *id95apm_get_id95apm(void)
{
	return glob_id95apm;
}
EXPORT_SYMBOL_GPL(id95apm_get_id95apm);

static int id95apm_set_page(struct id95apm *id95apm, u8 page)
{
	struct i2c_client *client = id95apm->client;
	u8 msg[2];
	int ret;

	msg[0] = ID95APM_GLOB_PAGE_CTRL;
	msg[1] = page;

	ret = i2c_master_send(client, msg, 2);
	if (ret == 2)
		ret = 0;
	else if (ret >= 0)
		ret = -EIO;

	return ret;
}

static int id95apm_read(struct id95apm *id95apm, u16 reg, int count, u8 *dest)
{
	struct i2c_client *client = id95apm->client;
	struct i2c_msg xfer[2];
	u8 page = GET_PAGE(reg);
	u8 reg8 = GET_REG8(reg);
	int ret;

	/* Check if new page needs to be selected */
	if (page != id95apm->current_page) {
		id95apm_set_page(id95apm, page);
		id95apm->current_page = page;
	};

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 1;
	xfer[0].buf = &reg8;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = count;
	xfer[1].buf = dest;

	ret = i2c_transfer(client->adapter, xfer, 2);
	if (ret == 2)
		ret = 0;
	else if (ret >= 0)
		ret = -EIO;

	return ret;
}

static int id95apm_write(struct id95apm *id95apm, u16 reg, int count,
			 const u8 *src)
{
	struct i2c_client *client = id95apm->client;
	u8 *msg;
	u8 page = GET_PAGE(reg);
	u8 reg8 = GET_REG8(reg);
	int ret;

	/* Check if new page needs to be selected */
	if (page != id95apm->current_page) {
		id95apm_set_page(id95apm, page);
		id95apm->current_page = page;
	};

	/* We add 1 byte for device register - ideally I2C would gather. */
	msg = kmalloc((count * sizeof(u16)) + 1, GFP_KERNEL);
	if (msg == NULL)
		return -ENOMEM;

	msg[0] = reg8;
	memcpy(&msg[1], src, count);

	ret = i2c_master_send(client, msg, count + 1);

	if (ret == count + 1)
		ret = 0;
	else if (ret >= 0)
		ret = -EIO;

	kfree(msg);

	return ret;
}

u8 id95apm_reg_read(struct id95apm *id95apm, u16 reg)
{
	u8 val;
	int ret;

	mutex_lock(&io_mutex);

	ret = id95apm_read(id95apm, reg, 1, &val);
	if (ret)
		dev_err(id95apm->dev, "read from reg 0x%x failed\n", reg);

	mutex_unlock(&io_mutex);

	return val;
}
EXPORT_SYMBOL_GPL(id95apm_reg_read);

u16 id95apm_reg16_read(struct id95apm *id95apm, u16 reg)
{
	u16 val;
	int ret;

	mutex_lock(&io_mutex);

	ret = id95apm_read(id95apm, reg, 2, (u8 *)&val);
	if (ret)
		dev_err(id95apm->dev, "block read starting from 0x%x failed\n",
			reg);

	mutex_unlock(&io_mutex);

	val = le16_to_cpu(val);

	return val;
}
EXPORT_SYMBOL_GPL(id95apm_reg16_read);

int id95apm_block_read(struct id95apm *id95apm, u16 reg, int count, u8 *data)
{
	int ret;

	mutex_lock(&io_mutex);

	ret = id95apm_read(id95apm, reg, count, data);
	if (ret)
		dev_err(id95apm->dev, "block read starting from 0x%x failed\n",
			reg);

	mutex_unlock(&io_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(id95apm_block_read);

int id95apm_reg_write(struct id95apm *id95apm, u16 reg, u8 val)
{
	int ret;

	mutex_lock(&io_mutex);

	ret = id95apm_write(id95apm, reg, 1, &val);
	if (ret)
		dev_err(id95apm->dev, "write to reg 0x%x failed\n", reg);

	mutex_unlock(&io_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(id95apm_reg_write);

int id95apm_reg16_write(struct id95apm *id95apm, u16 reg, u16 val)
{
	int ret;
	u16 val_le;

	val_le = cpu_to_le16(val);

	mutex_lock(&io_mutex);

	ret = id95apm_write(id95apm, reg, 2, (u8 *)&val_le);
	if (ret)
		dev_err(id95apm->dev, "block write starting at 0x%x failed\n",
			reg);

	mutex_unlock(&io_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(id95apm_reg16_write);

int id95apm_block_write(struct id95apm *id95apm, u16 reg, int count, u8 *src)
{
	int ret;

	mutex_lock(&io_mutex);

	ret = id95apm_write(id95apm, reg, count, src);
	if (ret)
		dev_err(id95apm->dev, "block write starting at 0x%x failed\n",
			reg);

	mutex_unlock(&io_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(id95apm_block_write);

int id95apm_clrset_bits(struct id95apm *id95apm, u16 reg, u8 mask, u8 val)
{
	u8 data;
	int ret;

	mutex_lock(&io_mutex);

	ret = id95apm_read(id95apm, reg, 1, &data);
	if (ret) {
		dev_err(id95apm->dev, "read from reg 0x%x failed\n", reg);
		goto out;
	}

	data &= ~mask;
	data |= val;

	ret = id95apm_write(id95apm, reg, 1, &data);
	if (ret)
		dev_err(id95apm->dev, "write to reg 0x%x failed\n", reg);

out:
	mutex_unlock(&io_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(id95apm_clrset_bits);

int id95apm_clrset_bits16(struct id95apm *id95apm, u16 reg, u16 mask, u16 val)
{
	u16 data;
	int ret;

	mutex_lock(&io_mutex);

	ret = id95apm_read(id95apm, reg, 2, (u8 *)&data);
	if (ret) {
		dev_err(id95apm->dev, "read from reg 0x%x failed\n", reg);
		goto out;
	}

	data &= ~mask;
	data |= val;

	ret = id95apm_write(id95apm, reg, 2, (u8 *)&data);
	if (ret)
		dev_err(id95apm->dev, "write to reg 0x%x failed\n", reg);

out:
	mutex_unlock(&io_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(id95apm_clrset_bits16);

static void id95apm_irq_call_handler(struct id95apm *id95apm, int irq)
{
	mutex_lock(&id95apm->irq_mutex);

	if (id95apm->irq[irq].handler)
		id95apm->irq[irq].handler(id95apm, irq, id95apm->irq[irq].data);
	else {
		dev_err(id95apm->dev, "irq %d nobody cared. now masked.\n",
			irq);
		/*
		 * Unfortunately there is no generic masking of just
		 * one subdevice.
		 */
	}

	mutex_unlock(&id95apm->irq_mutex);
}

/*
 * id95apm_irq_worker actually handles the interrupts.  Since all
 * interrupts are clear on read the IRQ line will be reasserted and
 * the physical IRQ will be handled again if another interrupt is
 * asserted while we run - in the normal course of events this is a
 * rare occurrence so we save I2C reads.
 */
static void id95apm_irq_worker(struct work_struct *work)
{
	struct id95apm *id95apm = container_of(work, struct id95apm,
					       irq_work);
	u32 val;
	int i;

	/*
	 * 4 byte registers contain all interrupt sources. Let's
	 * read all 4 together into one 32bit value.
	 */
	id95apm_block_read(id95apm, ID95APM_GLOB_EXT_IRQ_STAT0, 4, (u8 *)&val);
	val = le32_to_cpu(val);

	for (i = 0; i < ID95APM_NUM_IRQ; i++) {
		if (val & ID95APM_IRQ_MASK(i))
			id95apm_irq_call_handler(id95apm, i);
	}

	enable_irq(id95apm->chip_irq);
}

static irqreturn_t id95apm_irq(int irq, void *data)
{
	struct id95apm *id95apm = data;

	disable_irq_nosync(irq);
	schedule_work(&id95apm->irq_work);

	return IRQ_HANDLED;
}

int id95apm_register_irq(struct id95apm *id95apm, int irq,
			 void (*handler) (struct id95apm *, int, void *),
			 void *data)
{
	if (irq < 0 || irq > ID95APM_NUM_IRQ || !handler) {
		dev_err(id95apm->dev, "trying to register invalid irq %d (handler 0x%p)\n", irq, handler);
		return -EINVAL;
	}

	if (id95apm->irq[irq].handler) {
		dev_err(id95apm->dev, "irq %d handler already registered\n", irq);
		return -EBUSY;
	}

	mutex_lock(&id95apm->irq_mutex);
	id95apm->irq[irq].handler = handler;
	id95apm->irq[irq].data = data;
	mutex_unlock(&id95apm->irq_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(id95apm_register_irq);

int id95apm_free_irq(struct id95apm *id95apm, int irq)
{
	if (irq < 0 || irq > ID95APM_NUM_IRQ)
		return -EINVAL;

	mutex_lock(&id95apm->irq_mutex);
	id95apm->irq[irq].handler = NULL;
	mutex_unlock(&id95apm->irq_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(id95apm_free_irq);

static struct platform_device *id95apm_add_pdev(struct id95apm *id95apm,
						void *pdata, char *pdev_name,
						int count)
{
	struct platform_device *pdev;

	if (pdev_count > ID95APM_NUM_PDEVS) {
		dev_err(id95apm->dev,
			"Maximum number of platform_device's exceeded!\n");
		return NULL;
	}

	pdev = platform_device_alloc(pdev_name, count);
	if (!pdev) {
		dev_err(id95apm->dev, "Cannot create %s\n", pdev_name);
		return NULL;
	}
	pdev->dev.parent = id95apm->dev;
	pdev->dev.platform_data = pdata;
	dev_set_drvdata(&pdev->dev, id95apm);
	platform_device_add(pdev);

	pdevs[pdev_count++] = pdev;

	return pdev;
}

static void id95apm_cleanup(struct id95apm *id95apm)
{
	int i;

	free_irq(id95apm->chip_irq, id95apm);

	for (i = 0; i < pdev_count; i++)
		platform_device_unregister(pdevs[i]);
}

#ifdef CONFIG_OF
static int id95apm_set_pdata_from_of(struct id95apm *id95apm)
{
	struct device_node *node = id95apm->dev->of_node;
	struct id95apm_platform_data *pdata = &id95apm->pdata;

	if (!node) {
		dev_err(id95apm->dev, "main node not found\n");
		return -EINVAL;
	}

	if (of_find_property(node, "enable-32kHz", NULL)) {
		pdata->pll_stat |= ID95APM_PCON_PLL_STAT_32KOUT2;
	}

	return 0;
}
#else
static int id95apm_set_pdata_from_of(struct id95apm *id95apm)
{
	return 0;
}
#endif

/*
 * id95apm_init - Generic initialisation
 */
static int id95apm_init(struct id95apm *id95apm)
{
	struct id95apm_platform_data *pdata = NULL;
	struct platform_device *pdev;
	int ret = -ENOMEM;
	u8 id;
	u8 chip_options;
	u8 dev_rev;
	int i;

	pdata = dev_get_platdata(id95apm->dev);
	if (pdata)
		id95apm->pdata = *pdata;

	pdata = &id95apm->pdata;

	dev_set_drvdata(id95apm->dev, id95apm);

	id95apm_set_pdata_from_of(id95apm);

	/*
	 * Preset current_page to invalid value -> next access
	 * will re-write the page address
	 */
	id95apm->current_page = 0xff;

	/*
	 * Check that this is actually an ID95APM
	 */
	id = id95apm_reg_read(id95apm, ID95APM_GLOB_RESET_ID);
	if (id != ID95APM_ID) {
		dev_err(id95apm->dev, "not supported device\n");
		ret = -EINVAL;
		goto err;
	}

	chip_options = id95apm_reg_read(id95apm, ID95APM_ACCM_CHIP_OPTIONS);
	dev_rev = id95apm_reg_read(id95apm, ID95APM_ACCM_DEV_REV);
	dev_info(id95apm->dev, "id 0x%02x, chip-options 0x%02x, dev-rev 0x%02x\n",
		 id, chip_options, dev_rev);

	/*
	 * Basic initialization
	 */

	/* Set PLL to 32kHz, needed for example for DC/DC operation */
	id95apm_clrset_bits(id95apm, ID95APM_PCON_PLL_CFG,
			    ID95APM_PCON_PLL_CFG_MASK,
			    ID95APM_PCON_PLL_CFG_32KHZ);

	if (pdata->pll_stat)
		id95apm_reg_write(id95apm, ID95APM_PCON_PLL_STAT, pdata->pll_stat);

	/* Disable SW interrupts */
	id95apm_reg_write(id95apm, ID95APM_PCON_SW_IRQ, 0x00);

	/* Dispatch interrupts to both internal and external processors */
	id95apm_clrset_bits(id95apm, ID95APM_ACCM_IRQ_DIR1, 0x80, 0x80);

	/*
	 * Init this irq stuff *before* the subdevices are initialized
	 */
	mutex_init(&id95apm->irq_mutex);
	INIT_WORK(&id95apm->irq_work, id95apm_irq_worker);

	/*
	 * Register subdevices
	 */

	/*
	 * Regulator subdevices
	 */
	for (i = 0; i < ID95APM_NUM_REGULATORS; i++) {
		pdev = id95apm_add_pdev(id95apm, &pdata->reg_init_data[i],
					"id95apm-regltr", i);
		if (!pdev)
			goto err;
	}

	/*
	 * power/battery subdevice
	 */
	pdev = id95apm_add_pdev(id95apm, NULL, "id95apm-battery", 0);
	if (!pdev)
		goto err;

	/*
	 * Watchdog subdevice
	 */
	pdev = id95apm_add_pdev(id95apm, NULL, "id95apm-wdt", 0);
	if (!pdev)
		goto err;

	/*
	 * RTC subdevice
	 */
	pdev = id95apm_add_pdev(id95apm, NULL, "id95apm-rtc", 0);
	if (!pdev)
		goto err;

	/*
	 * GPIO subdevice
	 */
	pdev = id95apm_add_pdev(id95apm, NULL, "id95apm-gpio", 0);
	if (!pdev)
		goto err;

	/*
	 * hwmon subdevice
	 */
	pdev = id95apm_add_pdev(id95apm, NULL, "id95apm-hwmon", 0);
	if (!pdev)
		goto err;

	/*
	 * backlight subdevice
	 */
	pdev = id95apm_add_pdev(id95apm, NULL, "id95apm-backlight", 0);
	if (!pdev)
		goto err;

	/*
	 * codec subdevice
	 */
	pdev = id95apm_add_pdev(id95apm, NULL, "id95apm-codec", 0);
	if (!pdev)
		goto err;

	/*
	 * pwrkey subdevice
	 */
	pdev = id95apm_add_pdev(id95apm, &pdata->pwrkey_init, "id95apm-pwrkey", 0);
	if (!pdev)
		goto err;

	/*
	 * touch subdevice
	 */
	pdev = id95apm_add_pdev(id95apm, NULL, ID95APM_TSC_NAME, 0);
	if (!pdev)
		goto err;

	/*
	 * Now the real interrupt setup
	 */
	if (id95apm->chip_irq) {
		ret = request_irq(id95apm->chip_irq, id95apm_irq, IRQF_TRIGGER_LOW, "id95apm", id95apm);
		if (ret != 0) {
			dev_err(id95apm->dev, "Failed to request IRQ: %d\n",
				ret);
			goto err;
		}
	} else {
		dev_err(id95apm->dev, "No IRQ configured\n");
		ret = -ENODEV;
		goto err;
	}

	return 0;

err:
	id95apm_cleanup(id95apm);

	return ret;
}

static void id95apm_release(struct id95apm *id95apm)
{
	id95apm_cleanup(id95apm);
}

#ifdef CONFIG_TESTING_SYSFS
/*
 * Usage:
 *
 * Dump all 0x300 registers:
 * cat /sys/class/i2c-dev/i2c-0/device/0-002a/dump
 *
 * Dump watchdog register:
 * cat /sys/class/i2c-dev/i2c-0/device/0-002a/dump_wdt
 *
 * Dump ldo/dcdc register:
 * cat /sys/class/i2c-dev/i2c-0/device/0-002a/dump_ldo
 *
 * Dump charger register:
 * cat /sys/class/i2c-dev/i2c-0/device/0-002a/dump_charge
 *
 * Dump rtc register:
 * cat /sys/class/i2c-dev/i2c-0/device/0-002a/dump_rtc
 */
static ssize_t id95apm_sysfs_dump(struct device *dev,
				  struct device_attribute *attr, char *buf,
				  int start, int size)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct id95apm *id95apm = i2c_get_clientdata(client);
	int i;
	u8 val;
	ssize_t len = 0;

	for (i = start; i < (start + size); i++) {
		if ((i % 16) == 0) {
			if (i)
				len += sprintf(buf + len, "\n");
			len += sprintf(buf + len, "%s: %03x -", __func__, i);
		}
		val = id95apm_reg_read(id95apm, i);
		len += sprintf(buf + len, " %02x", val);
	}
	len += sprintf(buf + len, "\n");

	return len;
}

static ssize_t id95apm_sysfs_show_dump(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	return id95apm_sysfs_dump(dev, attr, buf, 0x000, 0x300);
}

static ssize_t id95apm_sysfs_show_dump_wdt(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	return id95apm_sysfs_dump(dev, attr, buf, 0x0a0, 0x010);
}

static ssize_t id95apm_sysfs_show_dump_ldo(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	return id95apm_sysfs_dump(dev, attr, buf, 0x060, 0x030);
}

static ssize_t id95apm_sysfs_show_dump_charge(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	return id95apm_sysfs_dump(dev, attr, buf, 0x090, 0x009);
}

static ssize_t id95apm_sysfs_show_dump_rtc(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	return id95apm_sysfs_dump(dev, attr, buf, 0x040, 0x011);
}

static DEVICE_ATTR(dump, S_IRUGO, id95apm_sysfs_show_dump, NULL);
static DEVICE_ATTR(dump_wdt, S_IRUGO, id95apm_sysfs_show_dump_wdt, NULL);
static DEVICE_ATTR(dump_ldo, S_IRUGO, id95apm_sysfs_show_dump_ldo, NULL);
static DEVICE_ATTR(dump_charge, S_IRUGO, id95apm_sysfs_show_dump_charge, NULL);
static DEVICE_ATTR(dump_rtc, S_IRUGO, id95apm_sysfs_show_dump_rtc, NULL);

static struct attribute *attrs[] = {
	&dev_attr_dump.attr,
	&dev_attr_dump_wdt.attr,
	&dev_attr_dump_ldo.attr,
	&dev_attr_dump_charge.attr,
	&dev_attr_dump_rtc.attr,
	NULL,
};
static struct attribute_group attr_group = {
	.attrs = attrs,
};

static int id95apm_sysfs_register(struct device *dev)
{
	return sysfs_create_group(&dev->kobj, &attr_group);
}
#endif

static const struct of_device_id id95apm_of_match[] = {
	{ .compatible = "idt,id95apm", .data = NULL },
	{ }
};
MODULE_DEVICE_TABLE(of, id95apm_of_match);

static int id95apm_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct id95apm *id95apm;
	int ret;

	id95apm = devm_kzalloc(&i2c->dev, sizeof(struct id95apm), GFP_KERNEL);
	if (id95apm == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	i2c_set_clientdata(i2c, id95apm);
	id95apm->client = i2c;
	id95apm->dev = &i2c->dev;
	id95apm->chip_irq = i2c->irq;

	ret = id95apm_init(id95apm);
	if (ret)
		goto struct_err;

	glob_id95apm = id95apm;

#ifdef CONFIG_TESTING_SYSFS
	ret = id95apm_sysfs_register(&i2c->dev);
	if (ret)
		goto struct_err;
#endif

	return 0;

struct_err:
	i2c_set_clientdata(i2c, NULL);
err:
	return ret;
}

static int id95apm_i2c_remove(struct i2c_client *i2c)
{
	struct id95apm *id95apm = i2c_get_clientdata(i2c);

	id95apm_release(id95apm);
	i2c_set_clientdata(i2c, NULL);

	return 0;
}

static const struct i2c_device_id id95apm_i2c_id[] = {
       { "id95apm", 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, id95apm_i2c_id);

#ifdef CONFIG_PM_RUNTIME
static int id95apm_suspend(struct device *dev)
{
	/* FIXME: TODO */
	return 0;
}

static int id95apm_resume(struct device *dev)
{
	/* FIXME: TODO */
	return 0;
}
#endif

static const struct dev_pm_ops id95apm_pm_ops = {
	SET_RUNTIME_PM_OPS(id95apm_suspend, id95apm_resume, NULL)
};

static struct i2c_driver id95apm_i2c_driver = {
	.driver = {
		.name = "ID95APM",
		.owner = THIS_MODULE,
		.pm = &id95apm_pm_ops,
		.of_match_table = of_match_ptr(id95apm_of_match),
	},
	.probe    = id95apm_i2c_probe,
	.remove   = id95apm_i2c_remove,
	.id_table = id95apm_i2c_id,
};

module_i2c_driver(id95apm_i2c_driver);

MODULE_DESCRIPTION("IDT 95APM audio & PMIC core driver");
MODULE_AUTHOR("Stefan Roese <sr@denx.de>");
MODULE_LICENSE("GPL");
