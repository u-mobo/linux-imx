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

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/mfd/id95apm.h>

/*
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>
#include <linux/io.h>
#include <mach/irqs.h>
#include <mach/system.h>
*/

#define ID95APM_LDO(max, min, rname, suspend_mv, num_consumers, consumers) \
{\
	.constraints = {\
		.name		= (rname), \
		.max_uV		= (max) * 1000,\
		.min_uV		= (min) * 1000,\
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE\
		|REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,\
		.valid_modes_mask = REGULATOR_MODE_NORMAL,\
		.state_mem = { \
			.uV = suspend_mv * 1000, \
			.mode = REGULATOR_MODE_NORMAL, \
			.enabled = (0 == suspend_mv) ? 0 : 1, \
			.disabled = 0, \
		}, \
	},\
	.num_consumer_supplies = (num_consumers), \
	.consumer_supplies = (consumers), \
}


static struct regulator_consumer_supply ldo6_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.0"),
};

struct id95apm_platform_data umobo_id95apm_pdata = {
	/* Core driver */
	//.init = umobo_id95apm_init,
	.irq_high = 1,	/* interrupt is active high */
#ifdef CONFIG_WL12XX_PLATFORM_DATA
	.pll_stat = ID95APM_PCON_PLL_STAT_32KOUT2, /* Enable SYS_32kHz OUT2 only */
#endif
	/* Regulator */
	.reg_init_data = {
		/* LDO0: LDO150-0, 0.75-3.70V, 150mA */
		[ID95APM_REGULATOR_LDO0] = {
			.constraints = {
				.name = "3V3_LDO",
				.min_uV = 3300000,
				.max_uV = 3300000,
				.always_on = 1,
				.apply_uV = 1,
			},
		},

		/* LDO1: LDO150-1, 0.75-3.70V, 150mA */
		[ID95APM_REGULATOR_LDO1] = {
			.constraints = {
				.name = "3V3_VT",
				.min_uV = 3300000,
				.max_uV = 3300000,
				.always_on = 1,
				.apply_uV = 1,
			},
		},

		/* LDO2: LDO150-2, 0.75-3.70V, 150mA */
		[ID95APM_REGULATOR_LDO2] = {
			.constraints = {
				.name = "3V3_CAM",
				.min_uV = 3300000,
				.max_uV = 3300000,
				.always_on = 1,
				.apply_uV = 1,
			},
		},

		/* LDO3: LDO050-0, 0.75-3.70V, 50mA */
		[ID95APM_REGULATOR_LDO3] = {
			.constraints = {
				.name = "3V3_SD_ON",
				.min_uV = 3300000,
				.max_uV = 3300000,
				.always_on = 1,
				.apply_uV = 1,
			},
		},

		/* LDO4: LDO050-1, 0.75-3.70V, 50mA */
		[ID95APM_REGULATOR_LDO4] = {
			.constraints = {
				.name = "3V3_SLOTS_ON",
				.min_uV = 3300000,
				.max_uV = 3300000,
				.always_on = 1,
				.apply_uV = 1,
			},
		},

		/* LDO5: LDO050-2, 0.75-3.70V, 50mA */
		[ID95APM_REGULATOR_LDO5] = {
			.constraints = {
				.name = "3V3_LCD_ON",
				.min_uV = 3300000,
				.max_uV = 3300000,
				.always_on = 1,
				.apply_uV = 1,
			},
		},

		/* LDO6: LDO050-3, 0.75-3.70V, 50mA */
		[ID95APM_REGULATOR_LDO6] = {
			.constraints = {
				.name = "3V3_SYS_ON",
				.min_uV = 3300000,
				.max_uV = 3300000,
				.always_on = 1,
				.apply_uV = 1,
			},
			.num_consumer_supplies = ARRAY_SIZE(ldo6_consumers),
			.consumer_supplies = ldo6_consumers,
		},

		/* LDO7: LP-LDO, 3.0 & 3.30V, 1mA */
		[ID95APM_REGULATOR_LDO7] = {
			.constraints = {
				.name = "3V3_RTC",
				.min_uV = 3300000,
				.max_uV = 3300000,
				.always_on = 1,
				.apply_uV = 1,
			},
		},

		/* DCDC0: Buck 500, 0.75-3.70V, 500mA */
		[ID95APM_REGULATOR_DCDC0] = {
			.constraints = {
				.name = "3V3_HUB",
				.min_uV = 3300000,
				.max_uV = 3300000,
				.always_on = 1,
				.apply_uV = 1,
			},
		},

		/* DCDC1: Buck 500, 0.75-3.70V, 500mA */
		[ID95APM_REGULATOR_DCDC1] = {
			.constraints = {
				.name = "3V3_SD",
				.min_uV = 3300000,
				.max_uV = 3300000,
				.always_on = 1,
				.apply_uV = 1,
			},
		},

		/* DCDC2: Buck 1000, 0.75-3.70V, 1A -> VDD_CORE */
		[ID95APM_REGULATOR_DCDC2] = {
			.constraints = {
				.name = "VREF_3V3",
				.min_uV = 3300000,
				.max_uV = 3300000,
				.always_on = 1,
				.apply_uV = 1,
			},
		},

		/* DCDC3: Boost 5, 4.05-5.60V, 1000mA */
		[ID95APM_REGULATOR_DCDC3] = {
			.constraints = {
				.name = "5V_BOOST",
				.min_uV = 5000000,
				.max_uV = 5000000,
				.always_on = 1,
				.apply_uV = 1,
			},
		},

		/* DCDC4: Led Boost, 0.78-25.00 mA */
		[ID95APM_REGULATOR_DCDC4] = {
			.constraints = {
				.name = "VBOOST_LED",
				.min_uV = 780,
				.max_uV = 25000,
				.always_on = 1,
				.apply_uV = 1,
			},
		},
	},

	/* GPIO */
	.id95apm_gpio_init = {
	/* id95apm-gpio driver use GPIO0 as base */
		//.gpio_base = SILVERBULLET_GPIO_ID95APM,
	},

	/* Power Switch Detector: SW_DET */
	.pwrkey_init = {
		.codes = { KEY_HOMEPAGE, KEY_BACK, KEY_MENU},
	},
};

static struct i2c_board_info __initdata id95apm_i2c_device = {
	I2C_BOARD_INFO("id95apm", 0x2a),
	.platform_data = &umobo_id95apm_pdata,
};

int __init silverbullet_init_id95apm(u32 int_gpio, u32 base_gpio)
{
	id95apm_i2c_device.irq = gpio_to_irq(int_gpio); /*update INT gpio */
	umobo_id95apm_pdata.id95apm_gpio_init.gpio_base = base_gpio;
	return i2c_register_board_info(0, &id95apm_i2c_device, 1);
}
