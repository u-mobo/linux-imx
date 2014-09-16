/*
 * Copyright (C) 2014 U-MoBo. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/init.h>

/*
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/ltc3676.h>
#include <linux/io.h>
#include <mach/irqs.h>
#include <mach/system.h>
*/
static struct i2c_board_info __initdata ltc3676_i2c_device = {
	I2C_BOARD_INFO("ltc3676", 0x3c),
	//I2C_BOARD_INFO("ltc3676-1", 0x3d),
	//.platform_data = &ltc3676_plat,
};

int __init silverbullet_init_ltc3676(u32 int_gpio)
{
	ltc3676_i2c_device.irq = gpio_to_irq(int_gpio); /*update INT gpio */
	return i2c_register_board_info(1, &ltc3676_i2c_device, 1);
}
