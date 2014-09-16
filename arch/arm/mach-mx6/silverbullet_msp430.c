/*
 * Copyright (C) 2011-2013 U-MoBo
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/delay.h>
#include <linux/i2c.h>

static char *umobo_msp430_wan_command_string[] = {
	"GetStatus",
	"Reset",
	"ModemOn",
	"ModemOff",
	"ModemReset",
	NULL
};

enum umobo_msp430_wan_command {
	umobo_msp430_wan_cmdGetStatus = 0,
	umobo_msp430_wan_cmdReset,
	umobo_msp430_wan_cmdModemOn,
	umobo_msp430_wan_cmdModemOff,
	umobo_msp430_wan_cmdModemReset
};

enum umobo_msp430_wan_status {
	umobo_msp430_wan_statBusy		= 1 << 7,
	umobo_msp430_wan_statVUSBHostEnable	= 1 << 5,
	umobo_msp430_wan_statVMDMEnable		= 1 << 4,
	umobo_msp430_wan_statVUSBEnable		= 1 << 3,
	umobo_msp430_wan_statPowerPulse		= 1 << 2,
	umobo_msp430_wan_statReset		= 1 << 1,
	umobo_msp430_wan_statModemPower		= 1 << 0
};

static int msp430_read(struct i2c_client *client, u8 reg, u8 *buf, int len)
{
	struct i2c_msg xfer_msg[2];
	int res;

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = 1;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = &reg;

	xfer_msg[1].addr = client->addr;
	xfer_msg[1].len = len;
	xfer_msg[1].flags = I2C_M_RD;
	xfer_msg[1].buf = buf;

	res = i2c_transfer(client->adapter, xfer_msg, 2);
	if (res != 2)
		dev_err(&client->dev, "reg %d, len %d, read err %d\n", reg, len, res);

	return res;
}

static bool umobo_msp430_wan_WaitNotBusy(struct i2c_client *client, int milliseconds)
{
	int res, retry = milliseconds / 100;
	unsigned char status;

	do {
		res = msp430_read(client, umobo_msp430_wan_cmdGetStatus, &status, sizeof(status));
		if (res < 0)
			return res;

		if (!(status & umobo_msp430_wan_statBusy))
			return 0;

		mdelay(100);
	} while (retry--);

	return -EBUSY;
}
static int umobo_msp430_wan_command_run(struct i2c_client *client, int command)
{
	int res;
	unsigned char status;

	res = umobo_msp430_wan_WaitNotBusy(client, 5000);
	if (res < 0)
		return res;

	res = msp430_read(client, command, &status, sizeof(status));
	if (res < 0)
		return res;

	if (!(status & umobo_msp430_wan_statBusy))
		return -ENOEXEC;	// Command not accepted

	return 0;
}

static ssize_t umobo_msp430_wan_command_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int cmd, i, res;

	for (cmd = -1, i = 0; (cmd < 0) && umobo_msp430_wan_command_string[i]; i++) {
		if (!strncmp(buf, umobo_msp430_wan_command_string[i], strlen(umobo_msp430_wan_command_string[i])))
			cmd = i;
	}

	if (cmd < 0)
		return -EINVAL;

	res = umobo_msp430_wan_command_run(client, cmd);
	if (res < 0)
		return res;

	return count;
}

static ssize_t umobo_msp430_wan_status_get(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char status;
	int res;

	res = msp430_read(client, umobo_msp430_wan_cmdGetStatus, &status, sizeof(status));
	if (res < 0)
		return res;

	return sprintf(buf, "%d\n", status);
}

static DEVICE_ATTR(umobo_msp430_wan_command, S_IWUSR, NULL, umobo_msp430_wan_command_set);
static DEVICE_ATTR(umobo_msp430_wan_status, S_IRUGO, umobo_msp430_wan_status_get, NULL);

static struct attribute *umobo_msp430_wan_attributes[] = {
	&dev_attr_umobo_msp430_wan_command.attr,
	&dev_attr_umobo_msp430_wan_status.attr,
	NULL
};

static const struct attribute_group umobo_msp430_wan_group = {
	.attrs = umobo_msp430_wan_attributes,
};

static int umobo_msp430_wan_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	int res;
	unsigned char buffer[10];

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_WORD_DATA)) {
		dev_err(&client->dev, "I2C functionality not supported\n");
		return -ENODEV;
	}

	/* check status & model */
	res = msp430_read(client, umobo_msp430_wan_cmdGetStatus, buffer, sizeof(buffer));
	if (res < 0)
		return res;

	/* Register sysfs hooks */
	res = sysfs_create_group(&client->dev.kobj, &umobo_msp430_wan_group);
	if (res) {
		dev_err(&client->dev, "sysfs_create_group failed %d\n", res);
		return -res;
	}

	dev_info(&client->dev, "status 0x%02x, model %s\n", buffer[0], buffer + 1);

	return 0;
}

static int umobo_msp430_wan_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &umobo_msp430_wan_group);
	return 0;
}

static const struct i2c_device_id umobo_msp430_wan_ids[] = {
	{ "umobo_msp430_wan", 0, },
	{ /* end of list */ },
};

static struct i2c_driver umobo_msp430_wan_driver = {
	.driver.name	= "umobo_msp430_wan",
	.id_table	= umobo_msp430_wan_ids,
	.probe		= umobo_msp430_wan_probe,
	.remove		= umobo_msp430_wan_remove,
};

static int __init silverbullet_msp430_init(void)
{
	int res;

	res = i2c_add_driver(&umobo_msp430_wan_driver);
	if (res < 0) {
		printk(KERN_INFO "add silverbullet_msp430_wan i2c driver failed\n");
		return -ENODEV;
	}
	return res;
}

static void __exit silverbullet_msp430_exit(void)
{
	i2c_del_driver(&umobo_msp430_wan_driver);
}

module_init(silverbullet_msp430_init);
module_exit(silverbullet_msp430_exit);
