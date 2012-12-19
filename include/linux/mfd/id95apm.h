/*
 * Core interface for IDT ID95APM
 *
 * Copyright 2009-2010 Stefan Roese <sr@denx.de>, DENX Software Engineering
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#ifndef __MFD_ID95APM_CORE_H__
#define __MFD_ID95APM_CORE_H__

#include <linux/regulator/machine.h>

/*
 * The registers of the ID95APM are organized in 3 pages of 256 bytes.
 * To easily address those registers, we use a 16bit register address
 * which consists of the page (upper 8 bits) and the register address
 * in the page (lower 8 bits).
 *
 * Example: Page 1, Register 0x23 -> 0x123
 */
#define GET_PAGE(reg)			(((reg) & 0x300) >> 8)
#define GET_REG8(reg)			((reg) & 0xff)

/* Registers */
/* Global: all pages, 0x00 - 0x0f */
#define ID95APM_GLOB_RESET_ID		0x000
#define ID95APM_GLOB_PAGE_CTRL		0x001
#define ID95APM_GLOB_DCDC_FAULT_STAT	0x002
#define ID95APM_GLOB_LDO_FAULT_STAT	0x003
#define ID95APM_GLOB_LDO_ENABLE		0x004
#define ID95APM_GLOB_DCDC_ENABLE	0x005
#define ID95APM_GLOB_EXT_IRQ_STAT0	0x006
#define ID95APM_GLOB_EXT_IRQ_STAT1	0x007
#define ID95APM_GLOB_EXT_IRQ_STAT2	0x008
#define ID95APM_GLOB_EXT_IRQ_STAT3	0x009
#define ID95APM_GLOB_INT_IRQ_STAT0	0x00a
#define ID95APM_GLOB_INT_IRQ_STAT1	0x00b
#define ID95APM_GLOB_INT_IRQ_STAT2	0x00c
#define ID95APM_GLOB_INT_IRQ_STAT3	0x00d
#define ID95APM_GLOB_I2C_SLAVE_ADDR	0x00e

/* ACCM: page 0, 0x10 - 0x1f */
#define ID95APM_ACCM_IRQ_DIR0		0x010
#define ID95APM_ACCM_IRQ_DIR1		0x011
#define ID95APM_ACCM_IRQ_DIR2		0x012
#define ID95APM_ACCM_IRQ_DIR3		0x013
#define ID95APM_ACCM_EXT_IRQ_DATA	0x014
#define ID95APM_ACCM_EXT_IRQ_STAT	0x015
#define ID95APM_ACCM_INT_IRQ_DATA	0x016
#define ID95APM_ACCM_INT_IRQ_STAT	0x017
#define ID95APM_ACCM_UP_CONTEXT0	0x018
#define ID95APM_ACCM_UP_CONTEXT1	0x019
#define ID95APM_ACCM_DATA_BUF0		0x01a
#define ID95APM_ACCM_DATA_BUF1		0x01b
#define ID95APM_ACCM_DATA_BUF2		0x01c
#define ID95APM_ACCM_DATA_BUF3		0x01d
#define ID95APM_ACCM_CHIP_OPTIONS	0x01e
#define ID95APM_ACCM_DEV_REV		0x01f

/* PCON: page 0, 0x20 - 0x3f */
#define ID95APM_PCON_GPIO_DIR		0x020
#define ID95APM_PCON_GPIO_DATA		0x022
#define ID95APM_PCON_GPIO_IN_MODE	0x024
#define ID95APM_PCON_GPIO_IRQ_ENABLE	0x026
#define ID95APM_PCON_GPIO_IN_EDGE	0x028
#define ID95APM_PCON_GPIO_IRQ_STAT	0x02a
#define ID95APM_PCON_GPIO_OUT_MODE	0x02c
#define ID95APM_PCON_GPIO_OFF		0x02e
#define ID95APM_PCON_GPIO_SPECIAL	0x030
#define ID95APM_PCON_SW_STAT		0x031
#define ID95APM_PCON_SW_IRQ		0x032
#define ID95APM_PCON_DCDC_IRQ_EN	0x033
#define ID95APM_PCON_PLL_CFG		0x034
#define ID95APM_PCON_PLL_STAT		0x035
#define ID95APM_PCON_HOTSWP_CONFIG	0x036
#define ID95APM_PCON_MCLK_RATE		0x037
#define ID95APM_PCON_AUDIO_CTRL		0x038
#define ID95APM_PCON_TSC_CTRL		0x039
#define ID95APM_PCON_GP_TIMER_CTRL	0x03a
#define ID95APM_PCON_IDT_EMBUP_CTRL	0x03b
#define ID95APM_PCON_IDT_POR_OUT	0x03c
#define ID95APM_PCON_POR_DELAY		0x03d
#define ID95APM_PCON_POR_SOURCE0	0x03e
#define ID95APM_PCON_POR_SOURCE1	0x03f

/* RTC: page 0, 0x40 - 0x5f */
#define ID95APM_RTC_SECOND_BASE		0x040

/* Offsets to base address */
#define ID95APM_RTC_SECOND		0x000
#define ID95APM_RTC_MINUTE		0x001
#define ID95APM_RTC_HOUR		0x002
#define ID95APM_RTC_DAY			0x003
#define ID95APM_RTC_DATE		0x004
#define ID95APM_RTC_MONTH		0x005
#define ID95APM_RTC_YEAR		0x006

#define ID95APM_RTC_ALARM1_SECOND	0x047
#define ID95APM_RTC_ALARM1_MONUTE	0x048
#define ID95APM_RTC_ALARM1_HOUR		0x049
#define ID95APM_RTC_ALARM1_DAY		0x04a
#define ID95APM_RTC_ALARM2_SECOND	0x04b
#define ID95APM_RTC_ALARM2_MONUTE	0x04c
#define ID95APM_RTC_ALARM2_HOUR		0x04d
#define ID95APM_RTC_ALARM2_DAY		0x04e
#define ID95APM_RTC_IRQ_EN		0x04f
#define ID95APM_RTC_IRQ_STAT		0x050

/* LDO: page 0, 0x60 - 0x7f */
#define ID95APM_LDO_150MA_00		0x060
#define ID95APM_LDO_150MA_01		0x062
#define ID95APM_LDO_150MA_02		0x064
#define ID95APM_LDO_50MA_03		0x066
#define ID95APM_LDO_50MA_04		0x068
#define ID95APM_LDO_50MA_05		0x06a
#define ID95APM_LDO_50MA_06		0x06c
#define ID95APM_LDO_AUDIO_18		0x06e
#define ID95APM_LDO_AUDIO_33		0x06f
#define ID95APM_LDO_EXT_PG		0x070
#define ID95APM_LDO_INT_PG		0x071
#define ID95APM_LDO_1MA_LP		0x072
#define ID95APM_LDO_FAULT_IRQ_EN	0x073
#define ID95APM_LDO_INT_TEST		0x074

/* DCDC: page 0, 0x80 - 0x8f */
#define ID95APM_DCDC_BUCK500_0		0x080
#define ID95APM_DCDC_BUCK500_1		0x082
#define ID95APM_DCDC_BUCK1000		0x084
#define ID95APM_DCDC_LED_BOOST		0x086
#define ID95APM_DCDC_BOOST5		0x088
#define ID95APM_DCDC_CLASS_D_EN0	0x08a
#define ID95APM_DCDC_CLASS_D_CFG0	0x08b
#define ID95APM_DCDC_CLASS_D_EN1	0x08c
#define ID95APM_DCDC_CLASS_D_CFG1	0x08d

/* CHGR: page 0, 0x90 - 0x9f */
#define ID95APM_CHGR_IN_CUR_LIMIT	0x090
#define ID95APM_CHGR_CUR_VOL_CTRL	0x091
#define ID95APM_CHGR_TERM_CTRL		0x092
#define ID95APM_CHGR_BAT_TEST_IN_CTRL	0x093
#define ID95APM_CHGR_FUNC_MOD_CTRL	0x094
#define ID95APM_CHGR_OP_STAT		0x095
#define ID95APM_CHGR_BLOCK_FAULT	0x096
#define ID95APM_CHGR_IRQ_STAT		0x097
#define ID95APM_CHGR_IRQ_EN		0x098

/* GPTIMER: page 0, 0xa0 - 0xaf */
#define ID95APM_GPTIMER_WDT_EN		0x0a0
#define ID95APM_GPTIMER_GPT_EN		0x0a1
#define ID95APM_GPTIMER_IRQ_STAT	0x0a2
#define ID95APM_GPTIMER_GPT_COUNT	0x0a3
#define ID95APM_GPTIMER_WDT_COUNT	0x0a4
#define ID95APM_GPTIMER_TBASE		0x0a5
#define ID95APM_GPTIMER_IRQ_EN		0x0a6

/* OPT: page 0, 0xb0 - 0xbf */

/* TSC: page 0, 0xc0 - 0xff */
#define ID95APM_TSC_MEASURE_STAT	0x0c0
#define ID95APM_TSC_X_CH1_RES		0x0c1
#define ID95APM_TSC_Y_CH2_RES		0x0c3
#define ID95APM_TSC_CH3_RES		0x0c5
#define ID95APM_TSC_CH4_RES		0x0c7
#define ID95APM_TSC_BAT_RES		0x0c9
#define ID95APM_TSC_TEMP_RES		0x0cb
#define ID95APM_TSC_VSYS_RES		0x0cd
#define ID95APM_TSC_CHARGE_RES		0x0cf
#define ID95APM_TSC_MEASURE_CONF	0x0d1
#define ID95APM_TSC_MEASURE_EN		0x0d2
#define ID95APM_TSC_CH1_MEASURE		0x0d3
#define ID95APM_TSC_CH2_MEASURE		0x0d4
#define ID95APM_TSC_CH3_MEASURE		0x0d5
#define ID95APM_TSC_CH4_MEASURE		0x0d6
#define ID95APM_TSC_VSYS_MEASURE	0x0d7
#define ID95APM_TSC_CHARGE_MEASURE	0x0d8
#define ID95APM_TSC_TEMP_MEASURE	0x0d9
#define ID95APM_TSC_BAT_MEASURE		0x0da
#define ID95APM_TSC_GP_AUTO_RANGE_HIGH	0x0db
#define ID95APM_TSC_GP_AUTO_RANGE_LOW	0x0dd
#define ID95APM_TSC_BAT_RANGE_HIGH	0x0df
#define ID95APM_TSC_BAT_RANGE_LOW	0x0e1
#define ID95APM_TSC_TEMP_RANGE_HIGH	0x0e3
#define ID95APM_TSC_TEMP_RANGE_LOW	0x0e5
#define ID95APM_TSC_TEMP_RANGE_XHIGH	0x0e7
#define ID95APM_TSC_TEMP_CONFIG		0x0e8
#define ID95APM_TSC_AVERAGE_TIMER	0x0ea
#define ID95APM_TSC_CONFIG		0x0eb
#define ID95APM_TSC_PENDING_IRQ		0x0ec
#define ID95APM_TSC_XTEMP_PENDING_IRQ	0x0ed
#define ID95APM_TSC_VSYS_MARGIN		0x0ee
#define ID95APM_TSC_BAT_MARGIN		0x0ef
#define ID95APM_TSC_TEMP_MARGIN		0x0f0

/* Audio: page 1, 0x10 - 0xff */

#define ID95APM_DAC0L_VOL		0x1a0
#define ID95APM_DAC0R_VOL		0x1a1
#define ID95APM_DAC1L_VOL		0x1a2
#define ID95APM_DAC1R_VOL		0x1a3

#define ID95APM_ADC0L_IN_AGAIN		0x1b0
#define ID95APM_ADC0R_IN_AGAIN		0x1b1
#define ID95APM_ADC1L_IN_AGAIN		0x1b4
#define ID95APM_ADC1R_IN_AGAIN		0x1b5
#define ID95APM_ADC1_IN_DBOOST		0x1b6

#define ID95APM_MIC_MODE		0x1b8
#define ID95APM_MIC_ABOOST		0x1b9
#define ID95APM_MIC_CTRL		0x1bb

#define ID95APM_I2S0_SOURCE		0x1c2
#define ID95APM_I2S1_SOURCE		0x1c3
#define ID95APM_DAC0_SOURCE		0x1c4
#define ID95APM_DAC1_SOURCE		0x1c5
#define ID95APM_CLASSD_SOURCE		0x1c6
#define ID95APM_LINE_OUT_CTRL		0x1c7
#define ID95APM_HP_SCTRL		0x1c8

#define ID95APM_I2S0_CFG1		0x1c9
#define ID95APM_I2S0_CFG2		0x1ca
#define ID95APM_I2S1_CFG1		0x1cb
#define ID95APM_I2S1_CFG2		0x1cc

#define ID95APM_PWR_CTRL2		0x1d2
#define ID95APM_PWR_CTRL3		0x1d3

#define ID95APM_DAC_ALL_CTRL		0x1e7

/* Class-D: page 2, 0x10 - 0xff */

#define ID95APM_CLASSD_CONFIG0		0x218
#define ID95APM_CLASSD_CONFIG1		0x219
#define ID95APM_CLASSD_PWM2		0x21d
#define ID95APM_CLASSD_MUTE		0x226
#define ID95APM_CLASSD_ATTEN		0x227
#define ID95APM_CLASSD_VOLUME0		0x228
#define ID95APM_CLASSD_VOLUME1		0x229

#define ID95APM_AUDIO_REG_FIRST		ID95APM_DAC0L_VOL
#define ID95APM_AUDIO_REG_LAST		ID95APM_CLASSD_VOLUME1
#define ID95APM_AUDIO_REG_NUM		(ID95APM_AUDIO_REG_LAST - \
					 ID95APM_AUDIO_REG_FIRST + 1)

/* Register bits */
#define ID95APM_ID			0x55

#define ID95APM_IRQ_RSV0		0
#define ID95APM_IRQ_GPIO1		1
#define ID95APM_IRQ_GPIO2		2
#define ID95APM_IRQ_GPIO3		3
#define ID95APM_IRQ_GPIO4		4
#define ID95APM_IRQ_GPIO5		5
#define ID95APM_IRQ_GPIO6		6
#define ID95APM_IRQ_GPIO7		7
#define ID95APM_IRQ_GPIO8		8
#define ID95APM_IRQ_GPIO9		9
#define ID95APM_IRQ_GPIO10		10
#define ID95APM_IRQ_RSV11		11
#define ID95APM_IRQ_SHORT_SW		12
#define ID95APM_IRQ_RSV13		13
#define ID95APM_IRQ_MID_SW		14
#define ID95APM_IRQ_BOTH		15
#define ID95APM_IRQ_WDT			16
#define ID95APM_IRQ_GPT			17
#define ID95APM_IRQ_RTC_ALARM1		18
#define ID95APM_IRQ_RTC_ALARM2		19
#define ID95APM_IRQ_LDO			20
#define ID95APM_IRQ_DCDC		21
#define ID95APM_IRQ_CHRG		22
#define ID95APM_IRQ_CLASSD		23
#define ID95APM_IRQ_TOUCH		24
#define ID95APM_IRQ_DIE_TEMP		25
#define ID95APM_IRQ_BAT_LOW		26
#define ID95APM_IRQ_VSYS_LOW		27
#define ID95APM_IRQ_ADC			28
#define ID95APM_IRQ_BAT_EXTREME_LOW	29
#define ID95APM_IRQ_DIE_TEMP_EXTREME	30
#define ID95APM_IRQ_RSV31		31

#define ID95APM_NUM_IRQ			(ID95APM_IRQ_DIE_TEMP_EXTREME + 1)

#define ID95APM_IRQ_MASK(num)		(1 << (num))

#define ID95APM_PCON_GP_TIMER_CTRL_EN	0x01

#define ID95APM_PCON_PLL_CFG_MASK	0x03
#define ID95APM_PCON_PLL_CFG_32KHZ	0x02

#define ID95APM_PCON_PLL_STAT_TCXO1	0x02
#define ID95APM_PCON_PLL_STAT_TCXO2	0x04
#define ID95APM_PCON_PLL_STAT_32KOUT1	0x10
#define ID95APM_PCON_PLL_STAT_32KOUT2	0x20

#define ID95APM_RTC_MONTH_CENTURY	0x80

#define ID95APM_RTC_IRQ_A1_EN		0x01

#define ID95APM_GPTIMER_WDT_EN_EN	0x01
#define ID95APM_GPTIMER_WDT_EN_RST	0x10

#define ID95APM_GPTIMER_IRQ_STAT_WDT	0x10

#define ID95APM_GPTIMER_IRQ_EN_GPT	0x01
#define ID95APM_GPTIMER_IRQ_EN_WDT	0x10

#define ID95APM_GPTIMER_TBASE_WDT_MASK	0x30

#define ID95APM_CHGR_CUR_VOL_CTRL_CUR	0x0f

#define ID95APM_CHGR_OP_STAT_ADAPTER	0x01
#define ID95APM_CHGR_OP_STAT_COLD	0x02
#define ID95APM_CHGR_OP_STAT_HOT	0x04
#define ID95APM_CHGR_OP_STAT_CHMODE	0x18
#define ID95APM_CHGR_OP_STAT_HOLD	0x00
#define ID95APM_CHGR_OP_STAT_RECOVER	0x08
#define ID95APM_CHGR_OP_STAT_CONST_CUR	0x10
#define ID95APM_CHGR_OP_STAT_CONST_VOLT	0x18
#define ID95APM_CHGR_OP_STAT_FAULT	0x20

#define ID95APM_CHGR_BLOCK_FAULT_ACTIVE	0x10
#define ID95APM_CHGR_BLOCK_FAULT_DONE	0x20

#define ID95APM_CHGR_BLOCK_FAULT_DISC	0x01
#define ID95APM_CHGR_BLOCK_FAULT_NO_NTC	0x02
#define ID95APM_CHGR_BLOCK_FAULT_CHRG	0x10
#define ID95APM_CHGR_BLOCK_FAULT_DONE	0x20

#define ID95APM_CHGR_IRQ_ADAPTER	0x01
#define ID95APM_CHGR_IRQ_CUR_LIM	0x02
#define ID95APM_CHGR_IRQ_CHRG_STATE	0x04
#define ID95APM_CHGR_IRQ_MASK		0x07

#define ID95APM_LDO_VSET_MASK		0x7f
#define ID95APM_LDO_EN			0x80

#define ID95APM_LDO_1MA_LP_30V		0x01

#define ID95APM_DCDC_BUCK_FAULT		0x30

#define ID95APM_LED_BOOST_CSET_MASK	0x1f
#define ID95APM_LED_BOOST_SCALE_FULL	0x40
#define ID95APM_LED_BOOST_EN		0x80

enum id95apm_regulator_id {
	ID95APM_REGULATOR_LDO0 = 0,	/* LDO150, 0.75-3.70V, 150mA */
	ID95APM_REGULATOR_LDO1,		/* LDO150, 0.75-3.70V, 150mA */
	ID95APM_REGULATOR_LDO2,		/* LDO150, 0.75-3.70V, 150mA */
	ID95APM_REGULATOR_LDO3,		/* LDO050, 0.75-3.70V, 50mA */
	ID95APM_REGULATOR_LDO4,		/* LDO050, 0.75-3.70V, 50mA */
	ID95APM_REGULATOR_LDO5,		/* LDO050, 0.75-3.70V, 50mA */
	ID95APM_REGULATOR_LDO6,		/* LDO050, 0.75-3.70V, 50mA */
	ID95APM_REGULATOR_LDO7,		/* LPLDO, 1.8-3.3V, 1mA */
	ID95APM_REGULATOR_DCDC0,	/* Buck 500, 0.75-3.70V, 500mA */
	ID95APM_REGULATOR_DCDC1,	/* Buck 500, 0.75-3.70V, 500mA */
	ID95APM_REGULATOR_DCDC2,	/* Buck 1000, 0.75-3.70V, 1A */
	ID95APM_REGULATOR_DCDC3,	/* Boost 5, 4.05-5.60V, 1000mA */
	ID95APM_REGULATOR_DCDC4,	/* LED Boost, 0.39-25.00mA */
	ID95APM_NUM_REGULATORS
};

enum id95apm_device_id {
	ID95APM_DEV_BATTERY = ID95APM_NUM_REGULATORS,
	ID95APM_DEV_WDT,
	ID95APM_DEV_RTC,
	ID95APM_DEV_GPIO,
	ID95APM_DEV_HWMON,
	ID95APM_DEV_BACKLIGHT,
	ID95APM_DEV_CODEC,
	ID95APM_DEV_PWRKEY,
	ID95APM_DEV_TOUCH,
	ID95APM_NUM_PDEVS
};

struct id95apm;

struct id95apm_irq {
	void (*handler) (struct id95apm *, int, void *);
	void *data;
};

struct snd_soc_codec;

struct id95apm_codec {
	struct platform_device *pdev;
	struct snd_soc_codec *codec;
};

#define ID95APM_PWRKEY_NUM_CODES	10

struct id95apm_pwrkey {
	struct input_dev *input;
	struct mutex shortkey_mutex;
	struct workqueue_struct *shortkey_workq;
	struct delayed_work shortkey_monitor;
	int shortkey_monitor_intervall;
	int shortkey_counter;
	int codes[ID95APM_PWRKEY_NUM_CODES];
};

#define ID95APM_TSC_NAME "id95apm_ts"

struct id95apm_tsc {
	struct input_dev *input;
	struct delayed_work work;
	int button_pressed;
};

/* Structure for each ID95APM Slave */
struct id95apm {
	struct device *dev;

	struct i2c_client *client;

	/* register page currently accessed */
	u8 current_page;

	/* Interrupt handling */
	struct work_struct irq_work;
	struct mutex irq_mutex; /* IRQ table mutex */
	struct id95apm_irq irq[ID95APM_NUM_IRQ];
	int chip_irq;

	struct regulator_dev *rdev[ID95APM_NUM_REGULATORS];

	struct power_supply *battery;
	struct power_supply *charger;

	struct rtc_device *rtc;

	struct id95apm_codec codec;

	struct id95apm_pwrkey pwrkey;

	struct id95apm_tsc tsc;
};

struct id95apm_gpio_init {
	int gpio_base;
};

struct id95apm_pwrkey_init {
	int shortkey_monitor_intervall;
	int codes[ID95APM_PWRKEY_NUM_CODES];
};

/**
 * Data to be supplied by the platform to initialise the WM8350.
 *
 * @init: Function called during driver initialisation.  Should be
 *        used by the platform to configure GPIO functions and similar.
 * @irq_high: Set if ID95APM IRQ is active high.
 */
struct id95apm_platform_data {
	/* Core driver */
	int (*init)(struct id95apm *id95apm);
	int irq_high;
	u16 pll_stat;

	/* Regulator */
	struct regulator_init_data reg_init_data[ID95APM_NUM_REGULATORS];

	/* GPIO */
	struct id95apm_gpio_init id95apm_gpio_init;

	/* Power Switch Detector: SW_DET */
	struct id95apm_pwrkey_init pwrkey_init;
};

/* Device I/O API */
u8 id95apm_reg_read(struct id95apm *id95apm, u16 reg);
u16 id95apm_reg16_read(struct id95apm *id95apm, u16 reg);
int id95apm_block_read(struct id95apm *id95apm, u16 reg, int count, u8 *data);
int id95apm_reg_write(struct id95apm *id95apm, u16 reg, u8 val);
int id95apm_reg16_write(struct id95apm *id95apm, u16 reg, u16 val);
int id95apm_block_write(struct id95apm *id95apm, u16 reg, int count, u8 *src);
int id95apm_clrset_bits(struct id95apm *id95apm, u16 reg, u8 mask, u8 val);
int id95apm_clrset_bits16(struct id95apm *id95apm, u16 reg, u16 mask, u16 val);

/*
 * ID95APM internal interrupts
 */
int id95apm_register_irq(struct id95apm *id95apm, int irq,
			 void (*handler) (struct id95apm *, int, void *),
			 void *data);
int id95apm_free_irq(struct id95apm *id95apm, int irq);

struct id95apm *id95apm_get_id95apm(void);

#endif
