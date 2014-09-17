/*
 * Copyright (C) 2012-2014 Freescale Semiconductor, Inc. All Rights Reserved.
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

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/ion.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include <linux/i2c/pca953x.h>
#include <linux/input/adxl34x.h>
#include <linux/input/l3g_gyro.h>
#include <linux/wl12xx.h>
#include <linux/ti_wilink_st.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>
#include <mach/mxc_ir.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_silverbullet.h"
#include "board-mx6dl_silverbullet.h"
#include <mach/imx_rfkill.h>

#define SILVERBULLET_ENET_IRQ		IMX_GPIO_NR(1, 26)
#define SILVERBULLET_ENET_RESET		IMX_GPIO_NR(1, 28)
#define SILVERBULLET_UART4_DTR		IMX_GPIO_NR(1, 29)

#define SILVERBULLET_GPIO0		IMX_GPIO_NR(2, 0)
#define SILVERBULLET_GPIO1		IMX_GPIO_NR(2, 1)
#define SILVERBULLET_GPIO2		IMX_GPIO_NR(2, 2)
#define SILVERBULLET_GPIO3		IMX_GPIO_NR(2, 3)
#define SILVERBULLET_GPIO4		IMX_GPIO_NR(2, 4)
#define SILVERBULLET_GPIO5		IMX_GPIO_NR(2, 5)
#define SILVERBULLET_GPIO6		IMX_GPIO_NR(2, 6)
#define SILVERBULLET_GPIO7		IMX_GPIO_NR(2, 7)
#define SILVERBULLET_UART4_DSR		IMX_GPIO_NR(2, 19)
#define SILVERBULLET_UART4_RI		IMX_GPIO_NR(2, 21)
#define SILVERBULLET_UART4_DCD		IMX_GPIO_NR(2, 22)
#define SILVERBULLET_ECSPI1_SS0		IMX_GPIO_NR(2, 30)

#define SILVERBULLET_OTG_VBUSEN		IMX_GPIO_NR(3, 22)

#define SILVERBULLET_ECSPI1_SS1		IMX_GPIO_NR(4, 10)
#define SILVERBULLET_ECSPI1_SS2		IMX_GPIO_NR(4, 11)

#define SILVERBULLET_VID1_RESET		IMX_GPIO_NR(5, 0)
#define SILVERBULLET_VID1_PWREN		IMX_GPIO_NR(5, 4)
#define SILVERBULLET_GPIO11		IMX_GPIO_NR(5, 20)
#define SILVERBULLET_CSI_CTL		IMX_GPIO_NR(5, 28)
#define SILVERBULLET_CSI_RESET		IMX_GPIO_NR(5, 29)

#define SILVERBULLET_GPIO8		IMX_GPIO_NR(6, 7)
#define SILVERBULLET_SD4_RESET		IMX_GPIO_NR(6, 8)	// FIXME: we should not use as gpio!!!
#define SILVERBULLET_GPIO9		IMX_GPIO_NR(6, 9)
#define SILVERBULLET_GPIO10		IMX_GPIO_NR(6, 10)
#define SILVERBULLET_USBH_VBUSEN	IMX_GPIO_NR(6, 11)
#define SILVERBULLET_SD1_CD		IMX_GPIO_NR(6, 14)
#define SILVERBULLET_SD1_WP		IMX_GPIO_NR(6, 15)
#define SILVERBULLET_PMIC_IRQ		IMX_GPIO_NR(6, 16)
#define SILVERBULLET_RGMII_RX_CTL	IMX_GPIO_NR(6, 24)
#define SILVERBULLET_RGMII_RD0		IMX_GPIO_NR(6, 25)
#define SILVERBULLET_RGMII_RD1		IMX_GPIO_NR(6, 27)
#define SILVERBULLET_RGMII_RD2		IMX_GPIO_NR(6, 28)
#define SILVERBULLET_RGMII_RD3		IMX_GPIO_NR(6, 29)
#define SILVERBULLET_RGMII_RXC		IMX_GPIO_NR(6, 30)
#define SILVERBULLET_ESAI_RESET		IMX_GPIO_NR(6, 31)

#define SILVERBULLET_GPIO15		IMX_GPIO_NR(7, 0)
#define SILVERBULLET_GPIO14		IMX_GPIO_NR(7, 1)
#define SILVERBULLET_GPIO12		IMX_GPIO_NR(7, 6)
#define SILVERBULLET_GPIO13		IMX_GPIO_NR(7, 7)
#define SILVERBULLET_SYS_IRQ		IMX_GPIO_NR(7, 8)

#define SILVERBULLET_GPIO_ID95APM	IMX_GPIO_NR(8, 16)		/* start at GPIO8_16 that not exist */
#define SILVERBULLET_BASEBOARD_LCD_nIRQ	(SILVERBULLET_GPIO_ID95APM + 1)	/* GPIO8_17 */
#define SILVERBULLET_BASEBOARD_DL2	(SILVERBULLET_GPIO_ID95APM + 2)	/* GPIO8_18 */
#define SILVERBULLET_BASEBOARD_DL1	(SILVERBULLET_GPIO_ID95APM + 3)	/* GPIO8_19 */
#define SILVERBULLET_BASEBOARD_DL3	(SILVERBULLET_GPIO_ID95APM + 4)	/* GPIO8_20 */
#define SILVERBULLET_HP_DET		(SILVERBULLET_GPIO_ID95APM + 10)	/* GPIO8_26 */

#define SILVERBULLET_SENS_GPIO_PCA953X		IMX_GPIO_NR(9, 0)			/* start at GPIO9_0 that not exist */
#define SILVERBULLET_SENS_ADXL34X_IRQ1		(SILVERBULLET_SENS_GPIO_PCA953X + 0)	/* GPIO9_0 */
#define SILVERBULLET_SENS_ADXL34X_IRQ2		(SILVERBULLET_SENS_GPIO_PCA953X + 1)	/* GPIO9_1 */
#define SILVERBULLET_SENS_MAG3110_IRQ		(SILVERBULLET_SENS_GPIO_PCA953X + 2)	/* GPIO9_2 */
#define SILVERBULLET_SENS_L3G_GYRO_IRQ1		(SILVERBULLET_SENS_GPIO_PCA953X + 3)	/* GPIO9_3 */
#define SILVERBULLET_SENS_L3G_GYRO_IRQ2		(SILVERBULLET_SENS_GPIO_PCA953X + 4)	/* GPIO9_4 */
#define SILVERBULLET_SENS_MPL3115A2_IRQ1	(SILVERBULLET_SENS_GPIO_PCA953X + 5)	/* GPIO9_5 */
#define SILVERBULLET_SENS_MPL3115A2_IRQ2	(SILVERBULLET_SENS_GPIO_PCA953X + 6)	/* GPIO9_6 */
#define SILVERBULLET_SENS_UC430_RESET		(SILVERBULLET_SENS_GPIO_PCA953X + 7)	/* GPIO9_7 */

#define SILVERBULLET_LCD_GPIOIRQ	SILVERBULLET_BASEBOARD_LCD_nIRQ
#define SILVERBULLET_LCD_GPIO_PCA953X	IMX_GPIO_NR(9, 8)	/* start at GPIO9_8 that not exist */
#define SILVERBULLET_LCD_LED1		(SILVERBULLET_LCD_GPIO_PCA953X + 0)	/* GPIO9_8 */
#define SILVERBULLET_LCD_LED2		(SILVERBULLET_LCD_GPIO_PCA953X + 1)	/* GPIO9_9 */
#define SILVERBULLET_LCD_LED3		(SILVERBULLET_LCD_GPIO_PCA953X + 2)	/* GPIO9_10 */
#define SILVERBULLET_LCD_SW1		(SILVERBULLET_LCD_GPIO_PCA953X + 3)	/* GPIO9_11 */
#define SILVERBULLET_LCD_SW2		(SILVERBULLET_LCD_GPIO_PCA953X + 4)	/* GPIO9_12 */
#define SILVERBULLET_LCD_SW3		(SILVERBULLET_LCD_GPIO_PCA953X + 5)	/* GPIO9_13 */
#define SILVERBULLET_LCD_TMA_IO		(SILVERBULLET_LCD_GPIO_PCA953X + 6)	/* GPIO9_14 */
#define SILVERBULLET_LCD_TOUCH_RESET	(SILVERBULLET_LCD_GPIO_PCA953X + 7)	/* GPIO9_15 */

#define SILVERBULLET_SENS_GPIOIRQ	SILVERBULLET_GPIO7
#define SILVERBULLET_UC430_ONOFF	SILVERBULLET_GPIO12
#define SILVERBULLET_UC430_RESET	SILVERBULLET_SENS_UC430_RESET

#define SILVERBULLET_UC864_ONOFF	SILVERBULLET_GPIO8
#define SILVERBULLET_UC864_RESET	SILVERBULLET_GPIO9
#define SILVERBULLET_UC864_nON		SILVERBULLET_GPIO10

#define SILVERBULLET_TIWI_FM_IRQ	SILVERBULLET_GPIO0
#define SILVERBULLET_TIWI_BT_WAKEUP	SILVERBULLET_GPIO1
#define SILVERBULLET_TIWI_WLAN_IRQ	SILVERBULLET_GPIO2
#define SILVERBULLET_TIWI_WLAN_EN	SILVERBULLET_GPIO13
#define SILVERBULLET_TIWI_BT_EN		SILVERBULLET_GPIO14
#define SILVERBULLET_TIWI_FM_EN		SILVERBULLET_GPIO15

#define SILVERBULLET_ID95APM_INT	SILVERBULLET_SYS_IRQ
#define SILVERBULLET_LTC3676_INT	SILVERBULLET_PMIC_IRQ

static struct clk *sata_clk;
static int mag3110_position = 6;
static int caam_enabled;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

/* SDIO Card Slot on U-MoBo */
static const struct esdhc_platform_data silverbullet_sd1_data __initconst = {
	.cd_gpio = SILVERBULLET_SD1_CD,
	.wp_gpio = SILVERBULLET_SD1_WP,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
	.runtime_pm = 1,
};

#ifdef CONFIG_WL12XX_PLATFORM_DATA
static void tiwi_set_power(int status)
{
	gpio_set_value(SILVERBULLET_TIWI_WLAN_EN, status);
}

/* SDIO WiFi on ExM */
static const struct esdhc_platform_data silverbullet_sd2_data __initconst = {
	.always_present = 1,
	.cd_gpio = -1,
	.wp_gpio = -1,
	.keep_power_at_suspend = 0,
	.runtime_pm = 1,
	//.max_clk = 15000000,
	.set_power = tiwi_set_power,
};
#endif /* CONFIG_WL12XX_PLATFORM_DATA */

/* SDIO Card Slot on SoM */
static const struct esdhc_platform_data silverbullet_sd4_data __initconst = {
	.always_present = 1,
	.cd_gpio = -1,
	.wp_gpio = -1,
	.keep_power_at_suspend = 1,
	//.max_clk = 25000000,
};

static const struct anatop_thermal_platform_data
	mx6q_silverbullet_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

static const struct imxuart_platform_data silverbullet_uart1_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS,
};

static const struct imxuart_platform_data silverbullet_uart2_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS,
};

static const struct imxuart_platform_data silverbullet_uart3_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS,
};

static const struct imxuart_platform_data silverbullet_uart4_data __initconst = {
	.flags      = 0,
};

static const struct imxuart_platform_data silverbullet_uart5_data __initconst = {
	.flags      = IMXUART_HAVE_RTSCTS | IMXUART_USE_DCEDTE,
};

static inline void silverbullet_init_uart(void)
{
	imx6q_add_imx_uart(0, &silverbullet_uart1_data);
	imx6q_add_imx_uart(1, &silverbullet_uart2_data);
	imx6q_add_imx_uart(2, &silverbullet_uart3_data);
	imx6q_add_imx_uart(3, &silverbullet_uart4_data);
	imx6q_add_imx_uart(4, &silverbullet_uart5_data);
}

static void mmd_write_reg(struct phy_device *dev, int device, int reg, int val)
{
	phy_write(dev, 0x0d, device);
	phy_write(dev, 0x0e, reg);
	phy_write(dev, 0x0d, (1 << 14) | device);
	phy_write(dev, 0x0e, val);
}

static int ksz9031rn_phy_fixup(struct phy_device *dev)
{
	/*
	 * min rx data delay, max rx/tx clock delay,
	 * min rx/tx control delay
	 */
	mmd_write_reg(dev, 2, 4, 0);
	mmd_write_reg(dev, 2, 5, 0);
	mmd_write_reg(dev, 2, 8, 0x03ff);

	return 0;
}

static int ksz9031rn_phy_reset(void)
{
	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_silverbullet_rgmii_rx_gpio_pads,
			ARRAY_SIZE(mx6q_silverbullet_rgmii_rx_gpio_pads));
	} else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_silverbullet_rgmii_rx_gpio_pads,
			ARRAY_SIZE(mx6dl_silverbullet_rgmii_rx_gpio_pads));
	}

	gpio_request(SILVERBULLET_RGMII_RX_CTL, "rgmii-rx-ctl");
	gpio_request(SILVERBULLET_RGMII_RD0, "rgmii-rd0");	
	gpio_request(SILVERBULLET_RGMII_RD1, "rgmii-rd1");
	gpio_request(SILVERBULLET_RGMII_RD2, "rgmii-rd2");
	gpio_request(SILVERBULLET_RGMII_RD3, "rgmii-rd3");
	//gpio_request(SILVERBULLET_RGMII_RXC, "rgmii-rxc");

	gpio_direction_output(SILVERBULLET_RGMII_RX_CTL, 1);
	gpio_direction_output(SILVERBULLET_RGMII_RD0, 1);
	gpio_direction_output(SILVERBULLET_RGMII_RD1, 1);
	gpio_direction_output(SILVERBULLET_RGMII_RD2, 1);
	gpio_direction_output(SILVERBULLET_RGMII_RD3, 1);
	//gpio_direction_output(SILVERBULLET_RGMII_RXC, 1);

	gpio_set_value(SILVERBULLET_ENET_RESET, 0);
	msleep(2);
	gpio_set_value(SILVERBULLET_ENET_RESET, 1);
	msleep(1);

	gpio_free(SILVERBULLET_RGMII_RX_CTL);
	gpio_free(SILVERBULLET_RGMII_RD0);
	gpio_free(SILVERBULLET_RGMII_RD1);
	gpio_free(SILVERBULLET_RGMII_RD2);
	gpio_free(SILVERBULLET_RGMII_RD3);
	//gpio_free(SILVERBULLET_RGMII_RXC);

	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_silverbullet_rgmii_rx_enet_pads,
			ARRAY_SIZE(mx6q_silverbullet_rgmii_rx_enet_pads));
	} else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_silverbullet_rgmii_rx_enet_pads,
			ARRAY_SIZE(mx6dl_silverbullet_rgmii_rx_enet_pads));
	}

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = ksz9031rn_phy_fixup,
	.phy = PHY_INTERFACE_MODE_RGMII,
	//.gpio_irq = SILVERBULLET_ENET_IRQ,
};

static int mx6q_silverbullet_spi1_cs[] = {
	SILVERBULLET_ECSPI1_SS0,
	SILVERBULLET_ECSPI1_SS1,
	SILVERBULLET_ECSPI1_SS2,
};

static const struct spi_imx_master mx6q_silverbullet_spi1_data __initconst = {
	.chipselect     = mx6q_silverbullet_spi1_cs,
	.num_chipselect = ARRAY_SIZE(mx6q_silverbullet_spi1_cs),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct mtd_partition silverbullet_spi_flash_partitions[] = {
	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00100000,
	},
	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data silverbullet_spi_flash_data = {
	.name = "m25p80",
	.parts = silverbullet_spi_flash_partitions,
	.nr_parts = ARRAY_SIZE(silverbullet_spi_flash_partitions),
	.type = "at25df321a",
};
#endif

static struct spi_board_info silverbullet_spi_board_info[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		.modalias = "m25p80",
		.max_speed_hz = 85000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 0,
		.chip_select = 0,
		.platform_data = &silverbullet_spi_flash_data,
	},
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(silverbullet_spi_board_info,
				ARRAY_SIZE(silverbullet_spi_board_info));
}

static struct imx_ssi_platform_data silverbullet_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

extern int silverbullet_init_ltc3676(u32 int_gpio);
extern int silverbullet_init_id95apm(u32 int_gpio, u32 base_gpio);

#define mV_to_uV(mV) (mV * 1000)
#define uV_to_mV(uV) (uV / 1000)
#define V_to_uV(V) (mV_to_uV(V * 1000))
#define uV_to_V(uV) (uV_to_mV(uV) / 1000)

static struct imxi2c_platform_data mx6q_silverbullet_i2c_data = {
	.bitrate = 100000,
};

static struct fsl_mxc_lightsensor_platform_data ls_data = {
	.rext = 100,	/* calibration: 499K->700K */
};

#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= descr,				\
	.wakeup		= wake,					\
	.debounce_interval = debounce,				\
}

static struct gpio_keys_button silverbullet_lcd_buttons[] = {
	GPIO_BUTTON(SILVERBULLET_LCD_SW1, KEY_HOMEPAGE, 1, "lcd-sw1", 0, 1),
	GPIO_BUTTON(SILVERBULLET_LCD_SW2, KEY_MENU, 1, "lcd-sw2", 0, 1),
	GPIO_BUTTON(SILVERBULLET_LCD_SW3, KEY_BACK, 1, "lcd-sw3", 0, 1),
};

#define UMOBO_LCD_BUTTONS_POLL_MS 100

static struct gpio_keys_platform_data silverbullet_lcd_button_data = {
	.buttons	= silverbullet_lcd_buttons,
	.nbuttons	= ARRAY_SIZE(silverbullet_lcd_buttons),
	.poll_interval	= UMOBO_LCD_BUTTONS_POLL_MS,
};

static struct platform_device silverbullet_lcd_button_device = {
	.name           = "gpio-keys-polled",
	.id             = -1,
	.num_resources  = 0,
	.dev            = {
		.platform_data = &silverbullet_lcd_button_data,
	}
};

#endif

static int silverbullet_lcd_pca953x_setup(struct i2c_client *client,
					unsigned gpio_base, unsigned ngpio,
					void *context) {

	/* enable touch */
	gpio_request(SILVERBULLET_LCD_TOUCH_RESET, "touch-reset");
	gpio_direction_output(SILVERBULLET_LCD_TOUCH_RESET, 1);
	gpio_free(SILVERBULLET_LCD_TOUCH_RESET);
	platform_device_register(&silverbullet_lcd_button_device);
	return 0;
}

static int silverbullet_sens_pca953x_setup(struct i2c_client *client,
					unsigned gpio_base, unsigned ngpio,
					void *context) {
	return 0;
}

static struct pca953x_platform_data silverbullet_lcd_pca953x_data = {
	.gpio_base = SILVERBULLET_LCD_GPIO_PCA953X,
	.setup = silverbullet_lcd_pca953x_setup,
	.irq_base = -1,
	//.irq_base = gpio_to_irq(SILVERBULLET_LCD_GPIO_PCA953X),
	// 8 irq lines: refer to MXC_BOARD_IRQS have more than 16 irq lines
};	// LCD

static struct pca953x_platform_data silverbullet_sens_pca953x_data = {
	.gpio_base = SILVERBULLET_SENS_GPIO_PCA953X,
	.setup = silverbullet_sens_pca953x_setup,
	.irq_base = gpio_to_irq(SILVERBULLET_SENS_GPIO_PCA953X),
	// 8 irq lines: refer to MXC_BOARD_IRQS have more than 16 irq lines
};	// ExM GPS-SENS

static struct l3g_gyro_platform_data silverbullet_l3g_gyro_data = {
	.poll_interval = 80,
	.fs_range = L3G_GYRO_FS_2000DPS,
	.temp_calibration = L3G_GYRO_DEFAULT_TEMP_CALIBRATION,
	.gpio_int1 = SILVERBULLET_SENS_L3G_GYRO_IRQ1,
	.gpio_int2 = SILVERBULLET_SENS_L3G_GYRO_IRQ2,
	.axis_map_x = ABS_X,
	.axis_map_y = ABS_Y,
	.axis_map_z = ABS_Z,
	.negate_x = 1,
	.negate_y = 0,
	.negate_z = 1,
};

static struct adxl34x_platform_data silverbullet_adxl34x_data = {
	.tap_threshold = 35,
	.tap_duration = 3,
	.tap_latency = 20,
	.tap_window = 20,
	.tap_axis_control = ADXL_TAP_X_EN | ADXL_TAP_Y_EN | ADXL_TAP_Z_EN,
	.act_axis_control = 0xFF,
	.activity_threshold = 6,
	.inactivity_threshold = 4,
	.inactivity_time = 3,
	.free_fall_threshold = 8,
	.free_fall_time = 0x20,
	.data_rate = 8,	// 12.5 Hz, 80 ms
	.data_range = ADXL_FULL_RES,

	.ev_type = EV_ABS,
	.ev_code_x = ABS_Y,	/* EV_REL */
	.ev_code_y = ABS_X,	/* EV_REL */
	.ev_code_z = ABS_Z,	/* EV_REL */

	.ev_code_tap = {0, 0, 0}, /* EV_KEY {x,y,z} */
	.power_mode = ADXL_AUTO_SLEEP | ADXL_LINK,
	.fifo_mode = ADXL_FIFO_STREAM,
	.watermark = 0,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("isl29023", 0x44),
		.platform_data = &ls_data,
	},
	{	// LCD interface
		I2C_BOARD_INFO("pca9538", 0x71),
		.platform_data = &silverbullet_lcd_pca953x_data,
		/* FIXME: ID95APM gpio driver does not support IRQ */
		//.irq = gpio_to_irq(SILVERBULLET_BASEBOARD_LCD_nIRQ),
	},
	{
		I2C_BOARD_INFO("clicktouch_ts", 0x1f),
	},
	{
		I2C_BOARD_INFO("sx8651", 0x48),
		//.platform_data = &silverbullet_lcd_sx865x_data,
		//.irq = gpio_to_irq(SILVERBULLET_BASEBOARD_LCD_nIRQ),
	},
	{	// ExM GPS-SENS
		I2C_BOARD_INFO("pca9538", 0x70),
		.platform_data = &silverbullet_sens_pca953x_data,
		.irq = gpio_to_irq(SILVERBULLET_SENS_GPIOIRQ),
	},
	{
		I2C_BOARD_INFO("l3gd20", 0x6b),
		.platform_data = &silverbullet_l3g_gyro_data,
		// FIXME? ST driver does not implement IRQ management
	},
	{
		I2C_BOARD_INFO("l3g4200d", 0x69),
		.platform_data = &silverbullet_l3g_gyro_data,
		// FIXME? ST driver does not implement IRQ management
	},
	{
		I2C_BOARD_INFO("mpl3115", 0x60),
		// FIXME? mpl3115 driver does not implement IRQ management
	},
	{
		I2C_BOARD_INFO("adxl34x", 0x1d),
		.platform_data = &silverbullet_adxl34x_data,
		.irq = gpio_to_irq(SILVERBULLET_SENS_ADXL34X_IRQ1),
	},
	{
		I2C_BOARD_INFO("mag3110", 0x0e),
		.platform_data = (void *)&mag3110_position,
		.irq = gpio_to_irq(SILVERBULLET_SENS_MAG3110_IRQ),
	},
	{
		I2C_BOARD_INFO("umobo_msp430_wan", 0x3b),
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
};

static void imx6q_silverbullet_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(SILVERBULLET_OTG_VBUSEN, 1);
	else
		gpio_set_value(SILVERBULLET_OTG_VBUSEN, 0);
}

static void imx6q_silverbullet_host1_vbus(bool on)
{
	if (on)
		gpio_set_value(SILVERBULLET_USBH_VBUSEN, 1);
	else
		gpio_set_value(SILVERBULLET_USBH_VBUSEN, 0);
}

static void __init imx6q_silverbullet_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(SILVERBULLET_OTG_VBUSEN, "otg-vbusen");
	if (ret) {
		pr_err("failed to get GPIO SILVERBULLET_OTG_VBUSEN: %d\n",
			ret);
		return;
	}
	gpio_direction_output(SILVERBULLET_OTG_VBUSEN, 0);
	/* keep USB host1 VBUS always on */
	ret = gpio_request(SILVERBULLET_USBH_VBUSEN, "usbh-vbusen");
	if (ret) {
		pr_err("failed to get GPIO SILVERBULLET_USBH_VBUSEN: %d\n",
			ret);
		return;
	}
	gpio_direction_output(SILVERBULLET_USBH_VBUSEN, 0);
	if (board_is_mx6_reva())
		mxc_iomux_set_gpr_register(1, 13, 1, 1);
	else
		mxc_iomux_set_gpr_register(1, 13, 1, 0);

	mx6_set_otghost_vbus_func(imx6q_silverbullet_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_silverbullet_host1_vbus);

}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_silverbullet_sata_init(struct device *dev, void __iomem *mmio)
{
	u32 tmpdata;
	int ret = 0, i;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFF) | 0x0593E4C4), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	usleep_range(100, 200);
	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_RESET, mmio);
	sata_phy_cr_write(SATA_PHY_CR_RESET_EN, mmio);
	usleep_range(100, 200);
	/* waiting for the rx_pll is stable */
	for (i = 0; i <= 5; i++) {
		sata_phy_cr_addr(SATA_PHY_CR_LANE0_OUT_STAT, mmio);
		sata_phy_cr_read(&ret, mmio);
		if (ret & SATA_PHY_CR_LANE0_RX_STABLE) {
			pr_info("sata phy rx_pll is stable!\n");
			break;
		} else if (i == 5)
			pr_info("wating for sata rx_pll lock time out\n");
		usleep_range(1000, 2000);
	}

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(mmio, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(mmio + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, mmio + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(mmio + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_silverbullet_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static int imx_ahci_suspend(struct device *dev)
{
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);

	return 0;
}

static int imx_ahci_resume(struct device *dev)
{
	int ret;

	ret = clk_enable(sata_clk);
	if (ret)
		dev_err(dev, "can't enable sata clock.\n");

	writel(((readl(IOMUXC_GPR13) & ~0x2) | 0x2), IOMUXC_GPR13);

	return 0;
}

static struct ahci_platform_data mx6q_silverbullet_sata_data = {
	.init = mx6q_silverbullet_sata_init,
	.exit = mx6q_silverbullet_sata_exit,
	.suspend = imx_ahci_suspend,
	.resume = imx_ahci_resume,
};
#endif

static void mx6q_silverbullet_flexcan0_switch(int enable)
{
	if (enable) {
		//gpio_set_value(SILVERBULLET_CAN1_STBY, 1);
	} else {
		//gpio_set_value(SILVERBULLET_CAN1_STBY, 0);
	}
}

static const struct flexcan_platform_data
	mx6q_silverbullet_flexcan0_pdata __initconst = {
	.transceiver_switch = mx6q_silverbullet_flexcan0_switch,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M + SZ_64M - SZ_16M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data silverbullet_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "hdmi",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "1920x1080M@60",
	.default_bpp = 32,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x silverbullet board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_silverbullet_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_silverbullet_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_silverbullet_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_silverbullet_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_silverbullet_i2c2_pads,
			ARRAY_SIZE(mx6dl_silverbullet_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_silverbullet_i2c2_pads,
			ARRAY_SIZE(mx6q_silverbullet_i2c2_pads));
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
	.phy_reg_vlev = 0x0294,
	.phy_reg_cksymtx = 0x800d,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 1,
	.disp_id = 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

static struct ion_platform_data imx_ion_data = {
	.nr = 1,
	.heaps = {
		{
		.id = 0,
		.type = ION_HEAP_TYPE_CARVEOUT,
		.name = "vpu_ion",
		.size = SZ_16M,
		.cacheable = 1,
		},
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};

struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = 0,
};

static void silverbullet_suspend_enter(void)
{
	/* suspend preparation */
}

static void silverbullet_suspend_exit(void)
{
	/* resume restore */
}
static const struct pm_platform_data mx6q_silverbullet_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = silverbullet_suspend_enter,
	.suspend_exit = silverbullet_suspend_exit,
};

static struct regulator_consumer_supply silverbullet_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.0"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data silverbullet_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(silverbullet_vmmc_consumers),
	.consumer_supplies = silverbullet_vmmc_consumers,
};

static struct fixed_voltage_config silverbullet_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &silverbullet_vmmc_init,
};

static struct platform_device silverbullet_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &silverbullet_vmmc_reg_config,
	},
};

static int __init imx6q_init_audio(void)
{
	imx6q_add_imx_ssi(1, &silverbullet_ssi_pdata);

	return 0;
}

#if defined(CONFIG_LEDS_TRIGGER) || defined(CONFIG_LEDS_GPIO)

#define GPIO_LED(gpio_led, name_led, act_low, trigger)	\
{									\
	.gpio			= gpio_led,				\
	.name			= name_led,				\
	.active_low		= act_low,				\
	.retain_state_suspended = 1,			\
	.default_state		= 0,					\
	.default_trigger	= trigger,		\
}

static struct gpio_led imx6q_gpio_leds[] = {
	GPIO_LED(SILVERBULLET_BASEBOARD_DL1, "dl1", 1, "none"),
	GPIO_LED(SILVERBULLET_BASEBOARD_DL2, "dl2", 1, "none"),
	GPIO_LED(SILVERBULLET_BASEBOARD_DL3, "dl3", 1, "heartbeat"),
	GPIO_LED(SILVERBULLET_UC864_ONOFF, "mdm-onoff", 0, "none"),
	GPIO_LED(SILVERBULLET_UC864_RESET, "mdm-reset", 0, "none"),
	GPIO_LED(SILVERBULLET_UC430_RESET, "gps-reset", 1, "none"),
	GPIO_LED(SILVERBULLET_UC430_ONOFF, "gps-onoff", 0, "none"),
	GPIO_LED(SILVERBULLET_LCD_LED1, "led1", 1, "none"),
	GPIO_LED(SILVERBULLET_LCD_LED2, "led2", 1, "none"),
	GPIO_LED(SILVERBULLET_LCD_LED3, "led3", 1, "none"),
	GPIO_LED(SILVERBULLET_LCD_TOUCH_RESET, "touch-reset", 1, "none"),
};

static struct gpio_led_platform_data imx6q_gpio_leds_data = {
	.leds		= imx6q_gpio_leds,
	.num_leds	= ARRAY_SIZE(imx6q_gpio_leds),
};

static struct platform_device imx6q_gpio_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &imx6q_gpio_leds_data,
	}
};

/* For BT_PWD_L is conflict with charger's LED trigger gpio on silverbullet_revC.
 * add mutual exclusion here to be decided which one to be used by board config
 */
static void __init imx6q_add_device_gpio_leds(void)
{
	platform_device_register(&imx6q_gpio_led_device);
}
#else
static void __init imx6q_add_device_gpio_leds(void) {}
#endif

#ifdef CONFIG_WL12XX_PLATFORM_DATA
#ifdef CONFIG_TI_ST
int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}

int plat_kim_chip_enable(struct kim_data_s *kim_data)
{
	/* reset pulse to the BT controller */
	usleep_range(150, 220);
	gpio_set_value_cansleep(kim_data->nshutdown, 0);
	usleep_range(150, 220);
	gpio_set_value_cansleep(kim_data->nshutdown, 1);
	usleep_range(150, 220);
	gpio_set_value_cansleep(kim_data->nshutdown, 0);
	usleep_range(150, 220);
	gpio_set_value_cansleep(kim_data->nshutdown, 1);
	usleep_range(1, 2);
	return 0;
}

int plat_kim_chip_disable(struct kim_data_s *kim_data)
{
	gpio_set_value_cansleep(kim_data->nshutdown, 0);
	return 0;
}

static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = SILVERBULLET_TIWI_BT_EN,
	.dev_name = "/dev/ttymxc1",
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
	.chip_enable = plat_kim_chip_enable,
	.chip_disable = plat_kim_chip_disable
};

static struct platform_device wl127x_bt_device = {
	.name	= "kim",
	.id	= -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};
#else
static int silverbullet_bt_power_change(int status)
{
	/* gpio request already performed during umobo_som_pca953x_setup */
	if (status) {
		usleep_range(150, 220);
		gpio_set_value_cansleep(SILVERBULLET_TIWI_BT_EN, 0);
		usleep_range(150, 220);
		gpio_set_value_cansleep(SILVERBULLET_TIWI_BT_EN, 1);
		usleep_range(150, 220);
		gpio_set_value_cansleep(SILVERBULLET_TIWI_BT_EN, 0);
		usleep_range(150, 220);
		gpio_set_value_cansleep(SILVERBULLET_TIWI_BT_EN, 1);
		usleep_range(1, 2);
	} else {
		gpio_set_value_cansleep(SILVERBULLET_TIWI_BT_EN, 0);
	}
	return 0;
}

static struct platform_device mxc_bt_rfkill = {
	.name = "mxc_bt_rfkill",
};

static struct imx_bt_rfkill_platform_data mxc_bt_rfkill_data = {
	.power_change = silverbullet_bt_power_change,
};
#endif /* CONFIG_TI_ST */

static struct wl12xx_platform_data silverbullet_wl1271_data __initdata = {
	.irq = gpio_to_irq(SILVERBULLET_TIWI_WLAN_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_38,
};
#endif /* CONFIG_WL12XX_PLATFORM_DATA */

static int __init silverbullet_gpio_init(void) {

	/* enable VID1 */
	gpio_request(SILVERBULLET_VID1_PWREN, "vid1-pwren");
	gpio_request(SILVERBULLET_VID1_RESET, "vid1-reset");
	gpio_direction_output(SILVERBULLET_VID1_PWREN, 1);
	gpio_direction_output(SILVERBULLET_VID1_RESET, 1);
	gpio_free(SILVERBULLET_VID1_PWREN);
	gpio_free(SILVERBULLET_VID1_RESET);

	gpio_request(SILVERBULLET_SD4_RESET, "sd4-reset");
	gpio_direction_output(SILVERBULLET_SD4_RESET, 1);

#ifdef CONFIG_WL12XX_PLATFORM_DATA
	gpio_request(SILVERBULLET_TIWI_WLAN_EN, "wlan-en");
	gpio_request(SILVERBULLET_TIWI_BT_EN, "bt-en");
	gpio_request(SILVERBULLET_TIWI_FM_EN, "fm-en");
	gpio_direction_output(SILVERBULLET_TIWI_WLAN_EN, 0);
	gpio_direction_output(SILVERBULLET_TIWI_BT_EN, 0);
	gpio_direction_output(SILVERBULLET_TIWI_FM_EN, 0);

	/* WL12xx WLAN Init */
	if (wl12xx_set_platform_data(&silverbullet_wl1271_data))
		pr_err("error setting wl12xx data\n");

	/* provide BT */
#ifdef CONFIG_TI_ST
	platform_device_register(&wl127x_bt_device);
	platform_device_register(&btwilink_device);
#else
	mxc_register_device(&mxc_bt_rfkill, &mxc_bt_rfkill_data);
#endif /* CONFIG_TI_ST */
#endif /* CONFIG_WL12XX_PLATFORM_DATA */

	return 0;
}

static struct platform_pwm_backlight_data silverbullet_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

#ifdef CONFIG_HAVE_EPIT
static struct platform_ir_data silverbullet_ir_data = {
    .pwm_id = 1,
    .epit_id = 0,
    .gpio_id = 0,
};
#endif

static struct mxc_dvfs_platform_data silverbullet_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;
	struct ipuv3_fb_platform_data *pdata_fb = silverbullet_fb_data;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				pdata_fb[i++].res_size[0] = memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(silverbullet_fb_data)) {
					str++;
					pdata_fb[i++].res_size[0] = memparse(str, &str);
				}
			}
			/* ION reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "ionmem=");
			if (str != NULL) {
				str += 7;
				imx_ion_data.heaps[0].size = memparse(str, &str);
			}
			/* Primary framebuffer base address */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fb0base=");
			if (str != NULL) {
				str += 8;
				pdata_fb[0].res_base[0] =
						simple_strtol(str, &str, 16);
			}
			/* GPU reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpumem=");
			if (str != NULL) {
				str += 7;
				imx6q_gpu_pdata.reserved_mem_size = memparse(str, &str);
			}
			break;
		}
	}
}

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 1,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{

	void __iomem *mx6_snvs_base =  MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);
	u32 value;
	value = readl(mx6_snvs_base + SNVS_LPCR);
	/*set TOP and DP_EN bit*/
	writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource = {
	.name = "android ram console",
	.flags = IORESOURCE_MEM,
};

static struct platform_device android_ram_console = {
	.name = "ram_console",
	.num_resources = 1,
	.resource = &ram_console_resource,
};

static int __init imx6x_add_ram_console(void)
{
	return platform_device_register(&android_ram_console);
}
#else
#define imx6x_add_ram_console() do {} while (0)
#endif

/*!
 * Board specific initialization.
 */
static void __init silverbullet_board_init(void)
{
	int i;
	int ret;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;
	struct platform_device *voutdev;

	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_silverbullet_pads,
			ARRAY_SIZE(mx6q_silverbullet_pads));
	} else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_silverbullet_pads,
			ARRAY_SIZE(mx6dl_silverbullet_pads));
	}

	silverbullet_gpio_init();

#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	 mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gpio_request(SILVERBULLET_ENET_RESET, "fec-reset");
	gpio_direction_output(SILVERBULLET_ENET_RESET, 1);

	gp_reg_id = silverbullet_dvfscore_data.reg_id;
	soc_reg_id = silverbullet_dvfscore_data.soc_id;
	silverbullet_init_uart();
	imx6x_add_ram_console();

	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		hdmi_core_data.ipu_id = 0;
		hdmi_core_data.disp_id = 0;
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(silverbullet_fb_data); i++)
			imx6q_add_ipuv3fb(i, &silverbullet_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(silverbullet_fb_data); i++)
			imx6q_add_ipuv3fb(i, &silverbullet_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_lcdif(&lcdif_data);
	voutdev = imx6q_add_v4l2_output(0);
	if (vout_mem.res_msize && voutdev) {
		dma_declare_coherent_memory(&voutdev->dev,
					    vout_mem.res_mbase,
					    vout_mem.res_mbase,
					    vout_mem.res_msize,
					    (DMA_MEMORY_MAP |
					     DMA_MEMORY_EXCLUSIVE));
	}

	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_device_gpio_leds();

	imx6q_add_imx_i2c(0, &mx6q_silverbullet_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_silverbullet_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_silverbullet_i2c_data);
	if (cpu_is_mx6dl())
		imx6q_add_imx_i2c(3, &mx6q_silverbullet_i2c_data);
	ret = gpio_request(SILVERBULLET_LTC3676_INT, "ltc3676-int");
	if (ret) {
		printk(KERN_ERR"request ltc3676-int error!!\n");
		return;
	} else {
		gpio_direction_input(SILVERBULLET_LTC3676_INT);
		silverbullet_init_ltc3676(SILVERBULLET_LTC3676_INT);
	}
	ret = gpio_request(SILVERBULLET_ID95APM_INT, "id95apm-int");
	if (ret) {
		printk(KERN_ERR"request id95apm-int error!!\n");
		return;
	} else {
		gpio_direction_input(SILVERBULLET_ID95APM_INT);
		silverbullet_init_id95apm(SILVERBULLET_ID95APM_INT, SILVERBULLET_GPIO_ID95APM);
	}
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));
	/* SPI */
	imx6q_add_ecspi(0, &mx6q_silverbullet_spi1_data);
	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_silverbullet_anatop_thermal_data);

	imx6_init_fec(fec_data);

	imx6q_add_pm_imx(0, &mx6q_silverbullet_pm_data);

	/*
	 * 1) sd4 on SoM, connected to emmc.
	 * 2) sd1 on U-MoBo
	 * 3) sd2 for wifi if present
	 */
	imx6q_add_sdhci_usdhc_imx(3, &silverbullet_sd4_data);
	imx6q_add_sdhci_usdhc_imx(0, &silverbullet_sd1_data);
	imx6q_add_sdhci_usdhc_imx(1, &silverbullet_sd2_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_silverbullet_init_usb();
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_silverbullet_sata_data);
#else
		mx6q_silverbullet_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&silverbullet_vmmc_reg_devices);

	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

#ifdef CONFIG_HAVE_EPIT
	imx6q_add_mxc_epit(0);
	imx6q_add_mxc_epit(1);
#endif

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(0, &silverbullet_pwm_backlight_data);

#ifdef CONFIG_MX6_IR
	/* add MXC IR device */
	imx6q_add_mxc_ir(0, &silverbullet_ir_data);
#endif

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&silverbullet_dvfscore_data);

	if (imx_ion_data.heaps[0].size)
		imx6q_add_ion(0, &imx_ion_data,
			sizeof(imx_ion_data) + sizeof(struct ion_platform_heap));

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);

	/* Register charger chips */
	pm_power_off = mx6_snvs_poweroff;
	imx6q_add_busfreq();

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
}

extern void __iomem *twd_base;
static void __init silverbullet_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.3", NULL);
	early_console_setup(UART4_BASE_ADDR, uart_clk);
}

static struct sys_timer silverbullet_timer = {
	.init   = silverbullet_timer_init,
};

static void __init mx6q_silverbullet_reserve(void)
{
	phys_addr_t phys;
	int i, fb0_reserved = 0, fb_array_size;

	/*
	 * Reserve primary framebuffer memory if its base address
	 * is set by kernel command line.
	 */
	fb_array_size = ARRAY_SIZE(silverbullet_fb_data);
	if (fb_array_size > 0 && silverbullet_fb_data[0].res_base[0] &&
	    silverbullet_fb_data[0].res_size[0]) {
		if (silverbullet_fb_data[0].res_base[0] > SZ_2G)
			printk(KERN_INFO"UI Performance downgrade with FB phys address %x!\n",
			    silverbullet_fb_data[0].res_base[0]);
		memblock_reserve(silverbullet_fb_data[0].res_base[0],
				 silverbullet_fb_data[0].res_size[0]);
		memblock_remove(silverbullet_fb_data[0].res_base[0],
				silverbullet_fb_data[0].res_size[0]);
		silverbullet_fb_data[0].late_init = true;
		fb0_reserved = 1;
	}
	for (i = fb0_reserved; i < fb_array_size; i++)
		if (silverbullet_fb_data[i].res_size[0]) {
			/* Reserve for other background buffer. */
			phys = memblock_alloc_base(silverbullet_fb_data[i].res_size[0],
						SZ_4K, SZ_2G);
			memblock_remove(phys, silverbullet_fb_data[i].res_size[0]);
			silverbullet_fb_data[i].res_base[0] = phys;
		}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	phys = memblock_alloc_base(SZ_1M, SZ_4K, SZ_1G);
	memblock_remove(phys, SZ_1M);
	memblock_free(phys, SZ_1M);
	ram_console_resource.start = phys;
	ram_console_resource.end   = phys + SZ_1M - 1;
#endif

#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_2G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif

#if defined(CONFIG_ION)
	if (imx_ion_data.heaps[0].size) {
		phys = memblock_alloc(imx_ion_data.heaps[0].size, SZ_4K);
		memblock_remove(phys, imx_ion_data.heaps[0].size);
		imx_ion_data.heaps[0].base = phys;
	}
#endif

	if (vout_mem.res_msize) {
		phys = memblock_alloc_base(vout_mem.res_msize,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, vout_mem.res_msize);
		vout_mem.res_mbase = phys;
	}
}

/*
 * initialize __mach_desc_SILVERBULLET data structure.
 */
MACHINE_START(SILVERBULLET, "U-MoBo i.MX 6Quad/DualLite/Solo Silverbullet Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = silverbullet_board_init,
	.timer = &silverbullet_timer,
	.reserve = mx6q_silverbullet_reserve,
MACHINE_END
