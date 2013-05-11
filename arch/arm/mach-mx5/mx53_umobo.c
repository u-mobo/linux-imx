/*
 * Copyright (C) 2012 U-MoBo Srl.
 */

/*
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
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/fixed.h>
#include <linux/android_pmem.h>
#include <linux/usb/android_composite.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/powerkey.h>
#include <linux/ahci_platform.h>
#include <linux/gpio_keys.h>
#include <linux/mfd/da9052/da9052.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/keypad.h>
#include <asm/mach/flash.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include <mach/iomux-mx53.h>
#include <mach/i2c.h>
#include <mach/mxc_iim.h>
#include <mach/mxc_rfkill.h>
#include <mach/check_fuse.h>

#include "crm_regs.h"
#include "devices.h"
#include "usb.h"
#include "pmic.h"

#include <linux/regulator/machine.h>
#include <linux/mfd/id95apm.h>
#include <linux/l3g4200d.h>
#include <linux/input/adxl34x.h>
#include <linux/wl12xx.h>

extern int __init mx53_umobo_init_da9052(void);
extern int __init mx53_umobo_init_msp430(void);

/*!
 * @file mach-mx5/mx53_umobo.c
 *
 * @brief This file contains MX53 U-MoBo board specific initialization routines.
 *
 * @ingroup MSL_MX53
 */

/* MX53 U-MoBo GPIO PIN configurations */
#define FEC_RST				(4*32 + 20)	/* GPIO_5_20 */
#define USB_PWREN			(6*32 + 8)	/* GPIO_7_8 */

#define MX53_UMOBO_SYSIRQ		(6*32 + 12)	/* GPIO7_12 */

#define MX53_UMOBO_SOM_GPIOIRQ		(3*32 + 3)	/* GPIO4_3 */
#define MX53_UMOBO_SOM_GPIO_PCA953X	(7*32)		/* start at GPIO8_0 that not exist */
#define MX53_UMOBO_SOM_GPIO8		(MX53_UMOBO_SOM_GPIO_PCA953X + 0)	/* GPIO8_0 */
#define MX53_UMOBO_SOM_GPIO9		(MX53_UMOBO_SOM_GPIO_PCA953X + 1)	/* GPIO8_1 */
#define MX53_UMOBO_SOM_GPIO10		(MX53_UMOBO_SOM_GPIO_PCA953X + 2)	/* GPIO8_2 */
#define MX53_UMOBO_SOM_GPIO11		(MX53_UMOBO_SOM_GPIO_PCA953X + 3)	/* GPIO8_3 */
#define MX53_UMOBO_SOM_GPIO12		(MX53_UMOBO_SOM_GPIO_PCA953X + 4)	/* GPIO8_4 */
#define MX53_UMOBO_SOM_GPIO13		(MX53_UMOBO_SOM_GPIO_PCA953X + 5)	/* GPIO8_5 */
#define MX53_UMOBO_SOM_GPIO14		(MX53_UMOBO_SOM_GPIO_PCA953X + 6)	/* GPIO8_3 */
#define MX53_UMOBO_SOM_GPIO15		(MX53_UMOBO_SOM_GPIO_PCA953X + 7)	/* GPIO8_7 */
#define MX53_UMOBO_SOM_VID1_PWREN	(MX53_UMOBO_SOM_GPIO_PCA953X + 8)	/* GPIO8_8 */
#define MX53_UMOBO_SOM_VID1_RESET	(MX53_UMOBO_SOM_GPIO_PCA953X + 9)	/* GPIO8_9 */
#define MX53_UMOBO_SOM_VID2_PWREN	(MX53_UMOBO_SOM_GPIO_PCA953X + 10)	/* GPIO8_10 */
#define MX53_UMOBO_SOM_VID2_RESET	(MX53_UMOBO_SOM_GPIO_PCA953X + 11)	/* GPIO8_11 */
#define MX53_UMOBO_SOM_CIS_RESET	(MX53_UMOBO_SOM_GPIO_PCA953X + 12)	/* GPIO8_12 */
#define MX53_UMOBO_SOM_CIS_CTL		(MX53_UMOBO_SOM_GPIO_PCA953X + 13)	/* GPIO8_13 */
#define MX53_UMOBO_SOM_OTG_VBUSEN	(MX53_UMOBO_SOM_GPIO_PCA953X + 14)	/* GPIO8_14 */
#define MX53_UMOBO_SOM_USBH_VBUSEN	(MX53_UMOBO_SOM_GPIO_PCA953X + 15)	/* GPIO8_15 */

#define MX53_UMOBO_GPIO_ID95APM		(7*32 + 16)	/* start at GPIO8_16 that not exist */
#define MX53_UMOBO_BASEBOARD_LCD_nIRQ	(MX53_UMOBO_GPIO_ID95APM + 1)	/* GPIO8_17 */
#define MX53_UMOBO_BASEBOARD_DL2	(MX53_UMOBO_GPIO_ID95APM + 2)	/* GPIO8_18 */
#define MX53_UMOBO_BASEBOARD_DL1	(MX53_UMOBO_GPIO_ID95APM + 3)	/* GPIO8_19 */
#define MX53_UMOBO_BASEBOARD_DL3	(MX53_UMOBO_GPIO_ID95APM + 4)	/* GPIO8_20 */
#define MX53_UMOBO_HP_DET		(MX53_UMOBO_GPIO_ID95APM + 10)	/* GPIO8_26 */

#define MX53_UMOBO_SENS_GPIOIRQ		(2*32 + 29)	/* GPIO3_29 */
#define MX53_UMOBO_SENS_GPIO_PCA953X	(8*32)		/* start at GPIO9_0 that not exist */
#define MX53_UMOBO_SENS_ADXL34X_IRQ1	(MX53_UMOBO_SENS_GPIO_PCA953X + 0)	/* GPIO9_0 */
#define MX53_UMOBO_SENS_ADXL34X_IRQ2	(MX53_UMOBO_SENS_GPIO_PCA953X + 1)	/* GPIO9_1 */
#define MX53_UMOBO_SENS_MAG3110_IRQ	(MX53_UMOBO_SENS_GPIO_PCA953X + 2)	/* GPIO9_2 */
#define MX53_UMOBO_SENS_L3GD4200_IRQ1	(MX53_UMOBO_SENS_GPIO_PCA953X + 3)	/* GPIO9_3 */
#define MX53_UMOBO_SENS_L3GD4200_IRQ2	(MX53_UMOBO_SENS_GPIO_PCA953X + 4)	/* GPIO9_4 */
#define MX53_UMOBO_SENS_MPL3115A2_IRQ1	(MX53_UMOBO_SENS_GPIO_PCA953X + 5)	/* GPIO9_5 */
#define MX53_UMOBO_SENS_MPL3115A2_IRQ2	(MX53_UMOBO_SENS_GPIO_PCA953X + 6)	/* GPIO9_6 */
#define MX53_UMOBO_SENS_UC430_RESET	(MX53_UMOBO_SENS_GPIO_PCA953X + 7)	/* GPIO9_7 */

#define MX53_UMOBO_LCD_GPIOIRQ		MX53_UMOBO_BASEBOARD_LCD_nIRQ
#define MX53_UMOBO_LCD_GPIO_PCA953X	(8*32 + 8)	/* start at GPIO9_8 that not exist */
#define MX53_UMOBO_LCD_LED1		(MX53_UMOBO_LCD_GPIO_PCA953X + 0)	/* GPIO9_8 */
#define MX53_UMOBO_LCD_LED2		(MX53_UMOBO_LCD_GPIO_PCA953X + 1)	/* GPIO9_9 */
#define MX53_UMOBO_LCD_LED3		(MX53_UMOBO_LCD_GPIO_PCA953X + 2)	/* GPIO9_10 */
#define MX53_UMOBO_LCD_SW1		(MX53_UMOBO_LCD_GPIO_PCA953X + 3)	/* GPIO9_11 */
#define MX53_UMOBO_LCD_SW2		(MX53_UMOBO_LCD_GPIO_PCA953X + 4)	/* GPIO9_12 */
#define MX53_UMOBO_LCD_SW3		(MX53_UMOBO_LCD_GPIO_PCA953X + 5)	/* GPIO9_13 */
#define MX53_UMOBO_LCD_TMA_IO		(MX53_UMOBO_LCD_GPIO_PCA953X + 6)	/* GPIO9_14 */
#define MX53_UMOBO_LCD_TOUCH_RESET	(MX53_UMOBO_LCD_GPIO_PCA953X + 7)	/* GPIO9_15 */

#define MX53_UMOBO_UC864_ONOFF		MX53_UMOBO_SOM_GPIO8
#define MX53_UMOBO_UC864_RESET		MX53_UMOBO_SOM_GPIO9
#define MX53_UMOBO_UC864_nON		MX53_UMOBO_SOM_GPIO10
#define MX53_UMOBO_UC430_ONOFF		MX53_UMOBO_SOM_GPIO12
#define MX53_UMOBO_UC430_RESET		MX53_UMOBO_SENS_UC430_RESET
#define MX53_UMOBO_TIWI_WLAN_EN		MX53_UMOBO_SOM_GPIO13
#define MX53_UMOBO_TIWI_WLAN_IRQ	(2*32 + 13)	/* GPIO3_13 */
#define MX53_UMOBO_TIWI_BT_EN		MX53_UMOBO_SOM_GPIO14
#define MX53_UMOBO_TIWI_BT_WAKEUP	(0*32 + 5)	/* GPIO1_5 */
#define MX53_UMOBO_TIWI_FM_EN		MX53_UMOBO_SOM_GPIO15

#define MX53_OFFSET			(0x20000000)
#define TZIC_WAKEUP0_OFFSET		(0x0E00)
#define TZIC_WAKEUP1_OFFSET		(0x0E04)
#define TZIC_WAKEUP2_OFFSET		(0x0E08)
#define TZIC_WAKEUP3_OFFSET		(0x0E0C)
#define GPIO7_0_12_IRQ_BIT		(0x1<<12)

#define MX53_UMOBO_CKIH1		24576000

#define MX53_TIWI_IRQ_PAD_CTRL		(PAD_CTL_PUS_100K_DOWN | PAD_CTL_PUE | PAD_CTL_PKE)
extern void pm_i2c_init(u32 base_addr);
static iomux_v3_cfg_t mx53_umobo_pads[] = {
	/* UART1 (UART4 on schematics) */
	MX53_PAD_PATA_DMACK__UART1_RXD_MUX,
	MX53_PAD_PATA_DIOW__UART1_TXD_MUX,
	MX53_PAD_EIM_D19__UART1_CTS,
	MX53_PAD_EIM_D20__UART1_RTS,
	MX53_PAD_EIM_D24__UART1_DTR,
	MX53_PAD_EIM_D25__UART1_DSR,
	MX53_PAD_PATA_DA_0__GPIO7_6,	/* RI */
	MX53_PAD_PATA_DMARQ__GPIO7_0,	/* DCD */

	/* UART2 (UART1 on schematics) */
	MX53_PAD_PATA_BUFFER_EN__UART2_RXD_MUX,
	MX53_PAD_GPIO_7__UART2_TXD_MUX,
	MX53_PAD_PATA_DIOR__UART2_RTS,
	MX53_PAD_EIM_D28__UART2_CTS,

	/* UART3 (UART2 on schematics) */
	MX53_PAD_PATA_CS_1__UART3_RXD_MUX,
	MX53_PAD_PATA_CS_0__UART3_TXD_MUX,
	MX53_PAD_EIM_EB3__UART3_RTS,
	MX53_PAD_EIM_D23__UART3_CTS,

	/* UART4 (DUART on schematics) */
	MX53_PAD_KEY_COL0__UART4_TXD_MUX,
	MX53_PAD_KEY_ROW0__UART4_RXD_MUX,

	/* UART5 (UART3 on schematics) */
	MX53_PAD_KEY_COL1__UART5_TXD_MUX,
	MX53_PAD_KEY_ROW1__UART5_RXD_MUX,
	MX53_PAD_KEY_COL4__UART5_RTS,
	MX53_PAD_KEY_ROW4__UART5_CTS,

	/* FEC */
	MX53_PAD_FEC_MDC__FEC_MDC,
	MX53_PAD_FEC_MDIO__FEC_MDIO,
	MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,
	MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
	MX53_PAD_FEC_RXD1__FEC_RDATA_1,
	MX53_PAD_FEC_RXD0__FEC_RDATA_0,
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,
	MX53_PAD_FEC_TXD1__FEC_TDATA_1,
	MX53_PAD_FEC_TXD0__FEC_TDATA_0,
	/* FEC_nRST */
	MX53_PAD_CSI0_DATA_EN__GPIO5_20,
	/* FEC_nINT */
	MX53_PAD_EIM_WAIT__GPIO5_0,

	/* AUDMUX3 */
	MX53_PAD_CSI0_DAT4__AUDMUX_AUD3_TXC,
	MX53_PAD_CSI0_DAT5__AUDMUX_AUD3_TXD,
	MX53_PAD_CSI0_DAT6__AUDMUX_AUD3_TXFS,
	MX53_PAD_CSI0_DAT7__AUDMUX_AUD3_RXD,

	/* I2C1 */
	MX53_PAD_CSI0_DAT8__I2C1_SDA,
	MX53_PAD_CSI0_DAT9__I2C1_SCL,
	/* I2C2 */
	MX53_PAD_KEY_COL3__I2C2_SCL,
	MX53_PAD_KEY_ROW3__I2C2_SDA,
	/* I2C3 */
	MX53_PAD_GPIO_3__I2C3_SCL,
	MX53_PAD_GPIO_6__I2C3_SDA,

	/* SPI1 */
	MX53_PAD_EIM_EB2__ECSPI1_SS0,
	MX53_PAD_KEY_COL2__ECSPI1_SS1,
	MX53_PAD_EIM_D16__ECSPI1_SCLK,
	MX53_PAD_EIM_D17__ECSPI1_MISO,
	MX53_PAD_EIM_D18__ECSPI1_MOSI,
	MX53_PAD_GPIO_19__ECSPI1_RDY,

	/* SPI2 */
	MX53_PAD_EIM_RW__ECSPI2_SS0,
	MX53_PAD_EIM_LBA__ECSPI2_SS1,
	MX53_PAD_EIM_CS0__ECSPI2_SCLK,
	MX53_PAD_EIM_OE__ECSPI2_MISO,
	MX53_PAD_EIM_CS1__ECSPI2_MOSI,
	MX53_PAD_EIM_A25__ECSPI2_RDY,

	/* SD1: on SoM */
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX53_PAD_SD1_DATA3__ESDHC1_DAT3,

	/* SD2: on U-MoBo */
	MX53_PAD_SD2_CMD__ESDHC2_CMD,
	MX53_PAD_SD2_CLK__ESDHC2_CLK,
	MX53_PAD_SD2_DATA0__ESDHC2_DAT0,
	MX53_PAD_SD2_DATA1__ESDHC2_DAT1,
	MX53_PAD_SD2_DATA2__ESDHC2_DAT2,
	MX53_PAD_SD2_DATA3__ESDHC2_DAT3,
	MX53_PAD_GPIO_4__ESDHC2_CD,	/* SD2_CD */
	MX53_PAD_GPIO_2__ESDHC2_WP,	/* SD2_WP */

	/* SD3: not used */

	/* SD4: on external module */
	MX53_PAD_PATA_DA_1__ESDHC4_CMD,
	MX53_PAD_PATA_DA_2__ESDHC4_CLK,
	MX53_PAD_PATA_DATA12__ESDHC4_DAT0,
	MX53_PAD_PATA_DATA13__ESDHC4_DAT1,
	MX53_PAD_PATA_DATA14__ESDHC4_DAT2,
	MX53_PAD_PATA_DATA15__ESDHC4_DAT3,

	/* VGA */
	MX53_PAD_EIM_DA11__IPU_DI1_PIN2,
	MX53_PAD_EIM_DA12__IPU_DI1_PIN3,

	/* LVDS */
	MX53_PAD_LVDS0_TX3_P__LDB_LVDS0_TX3,
	MX53_PAD_LVDS0_CLK_P__LDB_LVDS0_CLK,
	MX53_PAD_LVDS0_TX2_P__LDB_LVDS0_TX2,
	MX53_PAD_LVDS0_TX1_P__LDB_LVDS0_TX1,
	MX53_PAD_LVDS0_TX0_P__LDB_LVDS0_TX0,

	/* CSI0 */
	MX53_PAD_CSI0_DAT10__IPU_CSI0_D_10,
	MX53_PAD_CSI0_DAT11__IPU_CSI0_D_11,
	MX53_PAD_CSI0_DAT12__IPU_CSI0_D_12,
	MX53_PAD_CSI0_DAT13__IPU_CSI0_D_13,
	MX53_PAD_CSI0_DAT14__IPU_CSI0_D_14,
	MX53_PAD_CSI0_DAT15__IPU_CSI0_D_15,
	MX53_PAD_CSI0_DAT16__IPU_CSI0_D_16,
	MX53_PAD_CSI0_DAT17__IPU_CSI0_D_17,
	MX53_PAD_CSI0_DAT18__IPU_CSI0_D_18,
	MX53_PAD_CSI0_DAT19__IPU_CSI0_D_19,
	MX53_PAD_CSI0_VSYNC__IPU_CSI0_VSYNC,
	MX53_PAD_CSI0_MCLK__IPU_CSI0_HSYNC,
	MX53_PAD_CSI0_PIXCLK__IPU_CSI0_PIXCLK,

	/* DISPLAY */
	MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,
	MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
	MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
	MX53_PAD_DI0_PIN3__IPU_DI0_PIN3,
	MX53_PAD_DI0_PIN4__GPIO4_20, /* LVDS1_BKLCTL */
	MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0,
	MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1,
	MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2,
	MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3,
	MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4,
	MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5,
	MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6,
	MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7,
	MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8,
	MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9,
	MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10,
	MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11,
	MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12,
	MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13,
	MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14,
	MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15,
	MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16,
	MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17,
	MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18,
	MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19,
	MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20,
	MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21,
	MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22,
	MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23,

	MX53_PAD_GPIO_0__GPIO1_0,	/* GPIO/nIRQ0 */
	MX53_PAD_GPIO_5__GPIO1_5,	/* GPIO/nIRQ1 */
	//MX53_PAD_EIM_DA13__GPIO3_13,	/* GPIO/nIRQ2 */
	(_MX53_PAD_EIM_DA13__GPIO3_13 | MUX_PAD_CTRL(MX53_TIWI_IRQ_PAD_CTRL)),	/* GPIO/nIRQ2 */
	MX53_PAD_EIM_DA14__GPIO3_14,	/* GPIO/nIRQ3 */
	MX53_PAD_EIM_DA15__GPIO3_15,	/* GPIO/nIRQ4 */
	MX53_PAD_EIM_D21__GPIO3_21,	/* GPIO/nIRQ5 */
	MX53_PAD_EIM_D22__GPIO3_22,	/* GPIO/nIRQ6 */
	MX53_PAD_EIM_D29__GPIO3_29,	/* GPIO/nIRQ7 */

	MX53_PAD_GPIO_1__PWM2_PWMO,	/* PWM2 */
	MX53_PAD_GPIO_9__PWM1_PWMO,	/* PWM1 */

	/* CAN */
	MX53_PAD_GPIO_8__CAN1_RXCAN,
	MX53_PAD_PATA_INTRQ__CAN1_TXCAN,
	MX53_PAD_PATA_IORDY__CAN2_RXCAN,
	MX53_PAD_PATA_RESET_B__CAN2_TXCAN,

	MX53_PAD_GPIO_10__GPIO4_0,	/* USBH_nOC */
	MX53_PAD_GPIO_11__GPIO4_1,	/* OTG_nOC */
	MX53_PAD_GPIO_12__GPIO4_2,	/* ADC_IRQ */
	MX53_PAD_GPIO_13__GPIO4_3,	/* SOM GPIO_IRQ */
	MX53_PAD_GPIO_14__GPIO4_4,	/* PMIC_IRQ */
	MX53_PAD_EIM_D29__GPIO3_29,	/* SENS GPIO_IRQ */

	MX53_PAD_GPIO_16__GPIO7_11,
	MX53_PAD_GPIO_17__GPIO7_12,
	MX53_PAD_GPIO_18__GPIO7_13,
	
};

static void umobo_id95apm_irq_wakeup_only_fixup(void)
{
	void __iomem *tzic_base;
	tzic_base = ioremap(MX53_TZIC_BASE_ADDR, SZ_4K);
	if (NULL == tzic_base) {
		pr_err("fail to map MX53_TZIC_BASE_ADDR\n");
		return;
	}
	__raw_writel(0, tzic_base + TZIC_WAKEUP0_OFFSET);
	__raw_writel(0, tzic_base + TZIC_WAKEUP1_OFFSET);
	__raw_writel(0, tzic_base + TZIC_WAKEUP2_OFFSET);
	/* only enable irq wakeup for id95apm */
	__raw_writel(GPIO7_0_12_IRQ_BIT, tzic_base + TZIC_WAKEUP3_OFFSET);
	iounmap(tzic_base);
	pr_info("only id95apm irq is wakeup-enabled\n");
}

static void umobo_suspend_enter(void)
{
	umobo_id95apm_irq_wakeup_only_fixup();
	da9053_suspend_cmd_sw();
}

static void umobo_suspend_exit(void)
{
	if (da9053_get_chip_version())
		da9053_restore_volt_settings();
}

static struct mxc_pm_platform_data umobo_pm_data = {
	.suspend_enter = umobo_suspend_enter,
	.suspend_exit = umobo_suspend_exit,
};

static struct fb_videomode video_modes[] = {
	{
	 /* NTSC TV output */
	 "TV-NTSC", 60, 720, 480, 74074,
	 122, 15,
	 18, 26,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED,
	 0,},
	{
	 /* PAL TV output */
	 "TV-PAL", 50, 720, 576, 74074,
	 132, 11,
	 22, 26,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED | FB_VMODE_ODD_FLD_FIRST,
	 0,},
	{
	 /* 1080i50 TV output */
	 "1080I50", 50, 1920, 1080, 13468,
	 192, 527,
	 20, 24,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED | FB_VMODE_ODD_FLD_FIRST,
	 0,},
	{
	 /* 1080i60 TV output */
	 "1080I60", 60, 1920, 1080, 13468,
	 192, 87,
	 20, 24,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED | FB_VMODE_ODD_FLD_FIRST,
	 0,},
	{
	 /* 800x480 @ 57 Hz , pixel clk @ 27MHz */
	 "CLAA-WVGA", 57, 800, 480, 37037, 40, 60, 10, 10, 20, 10,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 800x480 @ 60 Hz , pixel clk @ 32MHz */
	 "SEIKO-WVGA", 60, 800, 480, 29850, 89, 164, 23, 10, 10, 10,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 800x480 @ 60 Hz , pixel clk @ 24.5MHz */
	 "SAMSUNG-LMS700", 60, 800, 480, 40816, 16, 8, 4, 9, 8, 4,
	 FB_SYNC_CLK_LAT_FALL,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 "XGA", 60, 1024, 768, 15385,
	 220, 40,
	 21, 7,
	 60, 10,
	 0,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 720p30 TV output */
	 "720P30", 30, 1280, 720, 13468,
	 260, 1759,
	 25, 4,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 "720P60", 60, 1280, 720, 13468,
	 260, 109,
	 25, 4,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	/* VGA 1280x1024 108M pixel clk output */
	"SXGA", 60, 1280, 1024, 9259,
	48, 248,
	1, 38,
	112, 3,
	0,
	FB_VMODE_NONINTERLACED,
	0,},
	{
	/* 1600x1200 @ 60 Hz 162M pixel clk*/
	"UXGA", 60, 1600, 1200, 6172,
	304, 64,
	1, 46,
	192, 3,
	FB_SYNC_HOR_HIGH_ACT|FB_SYNC_VERT_HIGH_ACT,
	FB_VMODE_NONINTERLACED,
	0,},
	{
	 /* 1080p24 TV output */
	 "1080P24", 24, 1920, 1080, 13468,
	 192, 637,
	 38, 6,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 1080p25 TV output */
	 "1080P25", 25, 1920, 1080, 13468,
	 192, 527,
	 38, 6,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 /* 1080p30 TV output */
	 "1080P30", 30, 1920, 1080, 13468,
	 192, 87,
	 38, 6,
	 1, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_NONINTERLACED,
	 0,},
	{
	 "1080P60", 60, 1920, 1080, 7692,
	 100, 40,
	 30, 3,
	 10, 2,
	 0,
	 FB_VMODE_NONINTERLACED,
	 0,},
};

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

extern void mx5_ipu_reset(void);
static struct mxc_ipu_config mxc_ipu_data = {
	.rev = 3,
	.reset = mx5_ipu_reset,
};

extern void mx5_vpu_reset(void);
static struct mxc_vpu_platform_data mxc_vpu_data = {
	.iram_enable = true,
	.iram_size = 0x14000,
	.reset = mx5_vpu_reset,
};

static struct fec_platform_data fec_data = {
	.phy = PHY_INTERFACE_MODE_RMII,
};

static struct mxc_dvfs_platform_data dvfs_core_data = {
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.gpc_vcr_offset = MXC_GPC_VCR_OFFSET,
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
	.delay_time = 30,
};

static struct mxc_bus_freq_platform_data bus_freq_data;

static struct tve_platform_data tve_data = {
	.boot_enable = MXC_TVE_VGA,
};

static struct ldb_platform_data ldb_data = {
	.ext_ref = 1,
};

static void mxc_iim_enable_fuse(void)
{
	u32 reg;

	if (!ccm_base)
		return;

	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg |= 0x10;
	writel(reg, ccm_base + 0x64);
}

static void mxc_iim_disable_fuse(void)
{
	u32 reg;

	if (!ccm_base)
		return;
	/* enable fuse blown */
	reg = readl(ccm_base + 0x64);
	reg &= ~0x10;
	writel(reg, ccm_base + 0x64);
}

static struct mxc_iim_data iim_data = {
	.bank_start = MXC_IIM_MX53_BANK_START_ADDR,
	.bank_end   = MXC_IIM_MX53_BANK_END_ADDR,
	.enable_fuse = mxc_iim_enable_fuse,
	.disable_fuse = mxc_iim_disable_fuse,
};

static struct resource mxcfb_resources[] = {
	[0] = {
	       .flags = IORESOURCE_MEM,
	       },
};

static struct mxc_fb_platform_data fb_data[] = {
	{
	 .interface_pix_fmt = IPU_PIX_FMT_RGB24,
	 .mode_str = "SEIKO-WVGA",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
	{
	 .interface_pix_fmt = IPU_PIX_FMT_GBR24,
	 .mode_str = "SXGA",
	 .mode = video_modes,
	 .num_modes = ARRAY_SIZE(video_modes),
	 },
};

extern int primary_di;
static int __init mxc_init_fb(void)
{
	if (!machine_is_mx53_umobo())
		return 0;

	/*for U-MoBo board, set default display as VGA*/
	if (primary_di < 0)
		primary_di = 1;

	if (primary_di) {
		printk(KERN_INFO "DI1 is primary\n");
		/* DI1 -> DP-BG channel: */
		mxc_fb_devices[1].num_resources = ARRAY_SIZE(mxcfb_resources);
		mxc_fb_devices[1].resource = mxcfb_resources;
		mxc_register_device(&mxc_fb_devices[1], &fb_data[1]);

		/* DI0 -> DC channel: */
		mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);
	} else {
		printk(KERN_INFO "DI0 is primary\n");

		/* DI0 -> DP-BG channel: */
		mxc_fb_devices[0].num_resources = ARRAY_SIZE(mxcfb_resources);
		mxc_fb_devices[0].resource = mxcfb_resources;
		mxc_register_device(&mxc_fb_devices[0], &fb_data[0]);

		/* DI1 -> DC channel: */
		mxc_register_device(&mxc_fb_devices[1], &fb_data[1]);
	}

	/*
	 * DI0/1 DP-FG channel:
	 */
	mxc_register_device(&mxc_fb_devices[2], NULL);

	return 0;
}
device_initcall(mxc_init_fb);

static struct imxi2c_platform_data mxci2c_data = {
       .bitrate = 100000,
};

static int __init umobo_id95apm_init(struct id95apm *id95apm)
{
	gpio_request(MX53_UMOBO_SYSIRQ, "sysirq");
	gpio_direction_input(MX53_UMOBO_SYSIRQ);
	return 0;
}

struct id95apm_platform_data umobo_id95apm_pdata = {
	/* Core driver */
	.init = umobo_id95apm_init,
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
#ifdef OLD_UMOBO
				.name = "1V8_LDO",
				.min_uV = 1800000,
				.max_uV = 1800000,
#else
				.name = "3V3_VT",
				.min_uV = 3300000,
				.max_uV = 3300000,
#endif
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
#ifdef OLD_UMOBO
				.name = "3V3_VT",
#else
				.name = "3V3_SD_ON",
#endif
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
				.min_uV = 25000000,
				.max_uV = 25000000,
				.always_on = 1,
				.apply_uV = 1,
			},
		},
	},

	/* GPIO */
	.id95apm_gpio_init = {
	/* id95apm-gpio driver use GPIO0 as base */
		.gpio_base = MX53_UMOBO_GPIO_ID95APM,
	},

	/* Power Switch Detector: SW_DET */
	.pwrkey_init = {
		.codes = { KEY_HOME, KEY_BACK, KEY_MENU},
	},
};

static struct mxc_lightsensor_platform_data umobo_isl29023_data = {
	.rext = 100,	/* calibration: 499K->700K */
};

#ifdef CONFIG_WL12XX_PLATFORM_DATA
static struct regulator_consumer_supply umobo_mmc4_supply =
	REGULATOR_SUPPLY("vmmc", "mxsdhci.3");

struct regulator_init_data umobo_wl1271_reg_initdata = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies = &umobo_mmc4_supply,
};

static struct fixed_voltage_config umobo_wl1271_reg_config = {
	.supply_name		= "WLAN_EN",
	.microvolts		= 3300000,
	.gpio			= MX53_UMOBO_TIWI_WLAN_EN,
	.startup_delay		= 70000, /* 70msec */
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &umobo_wl1271_reg_initdata,
};

static struct platform_device umobo_wl1271_reg_device = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &umobo_wl1271_reg_config,
	},
};

static struct wl12xx_platform_data umobo_wl1271_data __initdata = {
	.irq = gpio_to_irq(MX53_UMOBO_TIWI_WLAN_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_38,
};

static int mx53_umobo_bt_power_change(int status)
{
	/* gpio request already performed during umobo_som_pca953x_setup */
	if (status) {
		gpio_set_value(MX53_UMOBO_TIWI_BT_EN, 1);
		mdelay(15);
	} else {
		gpio_set_value(MX53_UMOBO_TIWI_BT_EN, 0);
		udelay(100);
	}
	return 0;
}

static struct platform_device mxc_bt_rfkill = {
	.name = "mxc_bt_rfkill",
};

static struct mxc_bt_rfkill_platform_data mxc_bt_rfkill_data = {
	.power_change = mx53_umobo_bt_power_change,
};
#endif /* CONFIG_WL12XX_PLATFORM_DATA */

#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
}

static struct gpio_keys_button umobo_lcd_buttons[] = {
	GPIO_BUTTON(MX53_UMOBO_LCD_SW1, KEY_HOME, 1, "lcd-sw1", 0),
	GPIO_BUTTON(MX53_UMOBO_LCD_SW2, KEY_MENU, 1, "lcd-sw2", 0),
	GPIO_BUTTON(MX53_UMOBO_LCD_SW3, KEY_BACK, 1, "lcd-sw3", 0),
};

#define UMOBO_LCD_BUTTONS_POLL_MS 100

static struct gpio_keys_platform_data umobo_lcd_button_data = {
	.buttons	= umobo_lcd_buttons,
	.nbuttons	= ARRAY_SIZE(umobo_lcd_buttons),
	.poll_interval	= UMOBO_LCD_BUTTONS_POLL_MS,
};

static struct platform_device umobo_lcd_button_device = {
	.name		= "gpio-keys-polled",
	.id		= -1,
	.dev		= {
		.platform_data = &umobo_lcd_button_data,
	}
};

static int umobo_lcd_pca953x_setup(struct i2c_client *client,
					unsigned gpio_base, unsigned ngpio,
					void *context) {

	/* enable touch */
	gpio_request(MX53_UMOBO_LCD_TOUCH_RESET, "touch-reset");
	gpio_direction_output(MX53_UMOBO_LCD_TOUCH_RESET, 1);
	gpio_free(MX53_UMOBO_LCD_TOUCH_RESET);
	platform_device_register(&umobo_lcd_button_device);
	return 0;
}

static int umobo_sens_pca953x_setup(struct i2c_client *client,
					unsigned gpio_base, unsigned ngpio,
					void *context) {
	return 0;
}

static int umobo_som_pca953x_setup(struct i2c_client *client,
					unsigned gpio_base, unsigned ngpio,
					void *context) {
	/* enable LCD */
	gpio_request(MX53_UMOBO_SOM_VID1_PWREN, "vid1-pwren");
	gpio_request(MX53_UMOBO_SOM_VID1_RESET, "vid1-reset");
	gpio_direction_output(MX53_UMOBO_SOM_VID1_PWREN, 1);
	gpio_direction_output(MX53_UMOBO_SOM_VID1_RESET, 1);
	gpio_free(MX53_UMOBO_SOM_VID1_PWREN);
	gpio_free(MX53_UMOBO_SOM_VID1_RESET);

	/* enable OTG VBUS */
	gpio_request(MX53_UMOBO_SOM_OTG_VBUSEN, "otg-vbusen");
	gpio_direction_output(MX53_UMOBO_SOM_OTG_VBUSEN, 0);
	gpio_free(MX53_UMOBO_SOM_OTG_VBUSEN);

	/* enable USBH VBUS */
	gpio_request(MX53_UMOBO_SOM_USBH_VBUSEN, "usbh-vbusen");
	gpio_direction_output(MX53_UMOBO_SOM_USBH_VBUSEN, 1);
	gpio_free(MX53_UMOBO_SOM_USBH_VBUSEN);

#ifdef CONFIG_WL12XX_PLATFORM_DATA
	gpio_request(MX53_UMOBO_TIWI_WLAN_EN, "wlan-en");
	gpio_request(MX53_UMOBO_TIWI_BT_EN, "bt-en");
	gpio_request(MX53_UMOBO_TIWI_FM_EN, "fm-en");

	/* low VIO-leakage state sequence */
	gpio_direction_output(MX53_UMOBO_TIWI_WLAN_EN, 0);
	gpio_direction_output(MX53_UMOBO_TIWI_BT_EN, 0);
	gpio_direction_output(MX53_UMOBO_TIWI_FM_EN, 0);
	udelay(100);
	gpio_set_value(MX53_UMOBO_TIWI_WLAN_EN, 1);
	gpio_set_value(MX53_UMOBO_TIWI_BT_EN, 1);
	gpio_set_value(MX53_UMOBO_TIWI_FM_EN, 1);
	mdelay(15);
	gpio_set_value(MX53_UMOBO_TIWI_WLAN_EN, 0);
	gpio_set_value(MX53_UMOBO_TIWI_BT_EN, 0);
	gpio_set_value(MX53_UMOBO_TIWI_FM_EN, 0);
	udelay(100);

	/* setup WLAN_IRQ for the driver */
	gpio_request(MX53_UMOBO_TIWI_WLAN_IRQ, "wlan-irq");
	gpio_direction_input(MX53_UMOBO_TIWI_WLAN_IRQ);
	/* avoid false positive at request_irq */
	set_irq_type(umobo_wl1271_data.irq, IRQ_TYPE_EDGE_RISING);
	irq_to_desc(MX53_UMOBO_TIWI_WLAN_IRQ)->status |= IRQ_NOAUTOEN;
	wl12xx_set_platform_data(&umobo_wl1271_data);

	/* let WLAN_EN work as regulator */
	gpio_free(MX53_UMOBO_TIWI_WLAN_EN);
	platform_device_register(&umobo_wl1271_reg_device);

	/* provide BT */
	mxc_register_device(&mxc_bt_rfkill, &mxc_bt_rfkill_data);
#endif

	return 0;
}

static struct pca953x_platform_data umobo_lcd_pca953x_data = {
	.gpio_base = MX53_UMOBO_LCD_GPIO_PCA953X,
	.setup = umobo_lcd_pca953x_setup,
	.irq_base = gpio_to_irq(MX53_UMOBO_LCD_GPIO_PCA953X),
	// 8 irq lines: refer to MXC_BOARD_IRQS have more than 16 irq lines
};	// LCD

static struct pca953x_platform_data umobo_sens_pca953x_data = {
	.gpio_base = MX53_UMOBO_SENS_GPIO_PCA953X,
	.setup = umobo_sens_pca953x_setup,
	.irq_base = gpio_to_irq(MX53_UMOBO_SENS_GPIO_PCA953X),
	// 8 irq lines: refer to MXC_BOARD_IRQS have more than 16 irq lines
};	// ExM GPS-SENS

static struct pca953x_platform_data umobo_som_pca953x_data = {
	.gpio_base = MX53_UMOBO_SOM_GPIO_PCA953X,	// no line used as irq
	.setup = umobo_som_pca953x_setup,
	.irq_base = -1,
};	// SoM

static struct l3g4200d_platform_data umobo_l3g4200d_data = {
	.poll_interval = 80,
	.fs_range = L3G4200D_FS_2000DPS,
	.temp_calibration = L3G4200D_DEFAULT_TEMP_CALIBRATION,
	.axis_map_x = ABS_X,
	.axis_map_y = ABS_Y,
	.axis_map_z = ABS_Z,
	.negate_x = 1,
	.negate_y = 0,
	.negate_z = 1,
};

static struct adxl34x_platform_data umobo_adxl34x_data = {
	.tap_threshold = 35,
	.tap_duration = 3,
	.tap_latency = 20,
	.tap_window = 20,
	.tap_axis_control = ADXL_TAP_X_EN | ADXL_TAP_Y_EN | ADXL_TAP_Z_EN | ADXL_TAP_X_INV | ADXL_TAP_Y_INV,
	.act_axis_control = 0xFF,
	.activity_threshold = 6,
	.inactivity_threshold = 4,
	.inactivity_time = 3,
	.free_fall_threshold = 8,
	.free_fall_time = 0x20,
	.data_rate = 8,	// 12.5 Hz, 80 ms
	.data_range = ADXL_FULL_RES,

	.ev_type = EV_ABS,
	.ev_code_x = ABS_X,	/* EV_REL */
	.ev_code_y = ABS_Y,	/* EV_REL */
	.ev_code_z = ABS_Z,	/* EV_REL */

	.ev_code_tap = {BTN_TOUCH, BTN_TOUCH, BTN_TOUCH}, /* EV_KEY {x,y,z} */
	.power_mode = ADXL_AUTO_SLEEP | ADXL_LINK,
	.fifo_mode = ADXL_FIFO_STREAM,
	.watermark = 0,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
/*
 * The TCA9539 is identical to the PCA9555, except for the removal of the
 * internal I/O pullup resistor, which greatly reduces power consumption when
 * the I/Os are held low, replacement of A2 with RESET, and a different address
 * range.
 */
	{	// SoM
		I2C_BOARD_INFO("pca9555", 0x74),
		.platform_data = &umobo_som_pca953x_data,
		.irq = gpio_to_irq(MX53_UMOBO_SOM_GPIOIRQ),
	},
	{	// Base board
		I2C_BOARD_INFO("id95apm", 0x2a),
		.platform_data = &umobo_id95apm_pdata,
		.irq = gpio_to_irq(MX53_UMOBO_SYSIRQ),
	},
	{	// LCD interface
		I2C_BOARD_INFO("pca9538", 0x71),
		.platform_data = &umobo_lcd_pca953x_data,
		/* FIXME: ID95APM gpio driver does not support IRQ */
		//.irq = gpio_to_irq(MX53_UMOBO_BASEBOARD_LCD_nIRQ),
	},
	/* for PMIC DA9053 (SoM addr 0x58): see mx53_umobo_pmic_da9053.c */
	{
		I2C_BOARD_INFO("ads7924", 0x49),
	},
	{
		I2C_BOARD_INFO("isl29023", 0x44),
		.platform_data = &umobo_isl29023_data,
	},
	{
		I2C_BOARD_INFO("clicktouch_ts", 0x1f),
	},
/*
 * FIXME: we are using sensors in polling mode.
 * It seems that PCA9538 IRQ management is buggy, either for SW or HW issue.
 */
	{	// ExM GPS-SENS
		I2C_BOARD_INFO("pca9538", 0x70),
		.platform_data = &umobo_sens_pca953x_data,
		.irq = gpio_to_irq(MX53_UMOBO_SENS_GPIOIRQ),
	},
	{
		I2C_BOARD_INFO("l3gd20", 0x6b),
		.platform_data = &umobo_l3g4200d_data,
		// FIXME? ST driver does not implement IRQ management
	},
	{
		I2C_BOARD_INFO("l3g4200d", 0x69),
		.platform_data = &umobo_l3g4200d_data,
		// FIXME? ST driver does not implement IRQ management
	},
	{
		I2C_BOARD_INFO("adxl34x", 0x1d),
		.platform_data = &umobo_adxl34x_data,
		//.irq = gpio_to_irq(MX53_UMOBO_SENS_ADXL34X_IRQ1),
	},
	{
		I2C_BOARD_INFO("mag3110", 0x0e),
		//.irq = gpio_to_irq(MX53_UMOBO_SENS_MAG3110_IRQ),
	},
/*
 * FIXME: we are using touch in polling mode.
 * It seems that PCA9538 IRQ management is buggy, either for SW or HW issue.
 */
	{
		I2C_BOARD_INFO("sx8651", 0x48),
		//.platform_data = &umobo_lcd_sx865x_data,
		//.irq = gpio_to_irq(MX53_UMOBO_BASEBOARD_LCD_nIRQ),
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
};

static unsigned int sdhc_get_card_det_true(struct device *dev)
{
	return 0;
}

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_true,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static struct mxc_mmc_platform_data mmc4_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_DATA_DDR | MMC_CAP_POWER_OFF_CARD,
	.min_clk = 400000,
	.max_clk = 20000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_true,
	.clock_mmc = "esdhc_clk",
	.power_mmc = "vmmc",
};

static int headphone_det_status(void)
{
	return (gpio_get_value(MX53_UMOBO_HP_DET) == 1);
}

static int mxc_id95apm_audio_init(void);

static struct mxc_audio_platform_data id95apm_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	//.hp_irq = gpio_to_irq(MX53_UMOBO_HP_DET),
	.hp_status = headphone_det_status,
	.init = mxc_id95apm_audio_init,
	.ext_ram_rx = 1,
};

static int mxc_id95apm_audio_init(void)
{
	/*
	 * SSI pre-dividers should use divider values larger than '1'.
	 * Ref i.MX53 Reference Manual, rev 2.1 06/2012,
	 * pag 829, end of paragraph 18.2.1.4.7
	 */
	id95apm_audio_data.sysclk = MX53_UMOBO_CKIH1 >> 1;


	/* delayed gpio request to allow id95apm-gpio driver to be loaded */
	gpio_request(MX53_UMOBO_HP_DET, "hp-det");
	gpio_direction_input(MX53_UMOBO_HP_DET);

	return 0;
}

static struct platform_device mxc_id95apm_audio_device = {
	.name = "imx-umobo-id95apm",
};

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data android_pmem_data = {
	.name = "pmem_adsp",
	.size = SZ_32M,
};

static struct android_pmem_platform_data android_pmem_gpu_data = {
	.name = "pmem_gpu",
	.size = SZ_64M,
	.cached = 1,
};

static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_all[] = {
	"rndis",
	"usb_mass_storage",
	"adb"
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= 0x0c01,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= 0x0c02,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
	{
		.product_id	= 0x0c10,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},

};

static struct usb_mass_storage_platform_data mass_storage_data = {
	.nluns		= 3,
	.vendor		= "U-MoBo",
	.product	= "MX53 Android Phone",
	.release	= 0x0100,
};

static struct usb_ether_platform_data rndis_data = {
	.vendorID	= 0x15a2,
	.vendorDescr	= "Freescale",
};

static struct android_usb_platform_data android_usb_data = {
	.vendor_id      = 0x15a2,
	.product_id     = 0x0c01,
	.version        = 0x0100,
	.product_name   = "MX53 Android Phone",
	.manufacturer_name = "U-MoBo",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};
#endif /* CONFIG_ANDROID_PMEM */

static struct mxc_asrc_platform_data mxc_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct gpio_led gpio_leds[] = {
	{
		.name			= "dl1",
		.default_trigger	= "id95apm-battery-full",
		.gpio			= MX53_UMOBO_BASEBOARD_DL1,
		.active_low		= 1,
		.default_state		= 0,
	},
	{
		.name			= "dl2",
		.default_trigger	= "id95apm-battery-charging",
		.gpio			= MX53_UMOBO_BASEBOARD_DL2,
		.active_low		= 1,
		.default_state		= 0,
	},
	{
		.name			= "dl3",
		.default_trigger	= "heartbeat",
		.gpio			= MX53_UMOBO_BASEBOARD_DL3,
		.active_low		= 1,
		.default_state		= 0,
	},
	{
		.name			= "mdm-onoff",
		.default_trigger	= "none",
		.gpio			= MX53_UMOBO_UC864_ONOFF,
		.active_low		= 0,
		.default_state		= 0,
	},
	{
		.name			= "mdm-reset",
		.default_trigger	= "none",
		.gpio			= MX53_UMOBO_UC864_RESET,
		.active_low		= 0,
		.default_state		= 0,
	},
	{
		.name			= "gps-reset",
		.default_trigger	= "none",
		.gpio			= MX53_UMOBO_UC430_RESET,
		.active_low		= 1,
		.default_state		= 0,
	},
	{
		.name			= "gps-onoff",
		.default_trigger	= "none",
		.gpio			= MX53_UMOBO_UC430_ONOFF,
		.active_low		= 0,
		.default_state		= 0,
	},
	{
		.name			= "led1",
		.default_trigger	= "none",
		.gpio			= MX53_UMOBO_LCD_LED1,
		.active_low		= 1,
		.default_state		= 0,
	},
	{
		.name			= "led2",
		.default_trigger	= "none",
		.gpio			= MX53_UMOBO_LCD_LED2,
		.active_low		= 1,
		.default_state		= 0,
	},
	{
		.name			= "led3",
		.default_trigger	= "none",
		.gpio			= MX53_UMOBO_LCD_LED3,
		.active_low		= 1,
		.default_state		= 0,
	},
	{
		.name			= "touch-reset",
		.default_trigger	= "none",
		.gpio			= MX53_UMOBO_LCD_TOUCH_RESET,
		.active_low		= 1,
		.default_state		= 0,
	},
};

static struct gpio_led_platform_data gpio_leds_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_leds_info,
	},
};

static void mx53_umobo_usbh1_vbus(bool on)
{/* FIXME: managed by TCA9539 ?
	if (on)
		gpio_set_value(USB_PWREN, 1);
	else
		gpio_set_value(USB_PWREN, 0);
*/}

static struct gpio_keys_button umobo_buttons[] = {
};

static struct gpio_keys_platform_data umobo_button_data = {
	.buttons	= umobo_buttons,
	.nbuttons	= ARRAY_SIZE(umobo_buttons),
};

static struct platform_device umobo_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &umobo_button_data,
	}
};

static void __init umobo_add_device_buttons(void)
{
	platform_device_register(&umobo_button_device);
}

/* workaround for ecspi chipselect pin may not keep correct level when idle */
static void mx53_evk_gpio_spi_chipselect_active(int cspi_mode, int status,
					     int chipselect)
{
	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			{
			iomux_v3_cfg_t eim_d19_gpio = MX53_PAD_EIM_D19__GPIO3_19;
			iomux_v3_cfg_t cspi_ss0 = MX53_PAD_EIM_EB2__ECSPI1_SS0;

			/* de-select SS1 of instance: ecspi1. */
			mxc_iomux_v3_setup_pad(eim_d19_gpio);
			mxc_iomux_v3_setup_pad(cspi_ss0);
			}
			break;
		case 0x2:
			{
			iomux_v3_cfg_t eim_eb2_gpio = MX53_PAD_EIM_EB2__GPIO2_30;
			iomux_v3_cfg_t cspi_ss1 = MX53_PAD_EIM_D19__ECSPI1_SS1;

			/* de-select SS0 of instance: ecspi1. */
			mxc_iomux_v3_setup_pad(eim_eb2_gpio);
			mxc_iomux_v3_setup_pad(cspi_ss1);
			}
			break;
		default:
			break;
		}
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}

static void mx53_evk_gpio_spi_chipselect_inactive(int cspi_mode, int status,
					       int chipselect)
{
	switch (cspi_mode) {
	case 1:
		switch (chipselect) {
		case 0x1:
			break;
		case 0x2:
			break;
		default:
			break;
		}
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}

static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = mx53_evk_gpio_spi_chipselect_active,
	.chipselect_inactive = mx53_evk_gpio_spi_chipselect_inactive,
};

static struct spi_board_info mxc_psoc_device[] __initdata = {
	{
	 .modalias = "spidev",
	 .max_speed_hz = 20000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 1,
	 .chip_select = 0,		/* DEVICE = /dev/spidev1.0 */
	},
	{
	 .modalias = "spidev",
	 .max_speed_hz = 20000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 1,
	 .chip_select = 1,		/* DEVICE = /dev/spidev1.1 */
	},
};

#ifndef CONFIG_ANDROID_PMEM
/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	struct tag *t;
	struct tag *mem_tag = 0;
	int total_mem = SZ_1G;
	int left_mem = 0;
	int gpu_mem = SZ_128M;
	int fb_mem = SZ_32M;
	char *str;

	mxc_set_cpu_type(MXC_CPU_MX53);

	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
			break;
		}
	}

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "mem=");
			if (str != NULL) {
				str += 4;
				left_mem = memparse(str, &str);
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_nommu");
			if (str != NULL)
				gpu_data.enable_mmu = 0;

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				gpu_mem = memparse(str, &str);
			}

			break;
		}
	}

	if (gpu_data.enable_mmu)
		gpu_mem = 0;

	if (left_mem == 0 || left_mem > total_mem)
		left_mem = total_mem - gpu_mem - fb_mem;

	if (mem_tag) {
		fb_mem = total_mem - left_mem - gpu_mem;
		if (fb_mem < 0) {
			gpu_mem = total_mem - left_mem;
			fb_mem = 0;
		}
		mem_tag->u.mem.size = left_mem;

		/*reserve memory for gpu*/
		if (!gpu_data.enable_mmu) {
			gpu_device.resource[5].start =
				mem_tag->u.mem.start + left_mem;
			gpu_device.resource[5].end =
				gpu_device.resource[5].start + gpu_mem - 1;
		}
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		if (fb_mem) {
			mxcfb_resources[0].start =
				gpu_data.enable_mmu ?
				mem_tag->u.mem.start + left_mem :
				gpu_device.resource[5].end + 1;
			mxcfb_resources[0].end =
				mxcfb_resources[0].start + fb_mem - 1;
		} else {
			mxcfb_resources[0].start = 0;
			mxcfb_resources[0].end = 0;
		}
#endif
	}
}
#endif /* CONFIG_ANDROID_PMEM */

static void __init mx53_umobo_io_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx53_umobo_pads,
					ARRAY_SIZE(mx53_umobo_pads));

	/* reset FEC PHY */
	gpio_request(FEC_RST, "fec-rst");
	gpio_direction_output(FEC_RST, 0);
	gpio_set_value(FEC_RST, 0);
	msleep(1);
	gpio_set_value(FEC_RST, 1);
}

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{

	mxc_ipu_data.di_clk[0] = clk_get(NULL, "ipu_di0_clk");
	mxc_ipu_data.di_clk[1] = clk_get(NULL, "ipu_di1_clk");
	mxc_ipu_data.csi_clk[0] = clk_get(NULL, "ssi_ext1_clk");
	/*
	 *ssi_ext1_clk was enbled in arch/arm/mach-mx5/clock.c, and it was kept
	 *open to provide clock for audio codec on i.Mx53 Quickstart, but U-MoBo
	 *have no needs to do that, so we close it here
	 */
	clk_disable(mxc_ipu_data.csi_clk[0]);


	/* use ckih1 reference clock (24.576 MHz) for ssi_lp_apm */
	clk_set_parent(clk_get(NULL, "ssi_lp_apm_clk"), clk_get(NULL, "ckih"));

	/* FIXME: need to let someone know about SD2 CD&WP ?*/

	mxc_cpu_common_init();
	mx53_umobo_io_init();

	mxc_register_device(&mxc_dma_device, NULL);
	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxci2c_devices[0], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[2], &mxci2c_data);

	mx53_umobo_init_da9052();
	dvfs_core_data.reg_id = "DA9052_BUCK_CORE";
	tve_data.dac_reg = "DA9052_LDO7";
	bus_freq_data.gp_reg_id = "DA9052_BUCK_CORE";
	bus_freq_data.lp_reg_id = "DA9052_BUCK_PRO";

	mxc_register_device(&mxc_rtc_device, NULL);
	mxc_register_device(&mxc_ipu_device, &mxc_ipu_data);
	mxc_register_device(&mxc_ldb_device, &ldb_data);
	mxc_register_device(&mxc_tve_device, &tve_data);
	if (!mxc_fuse_get_vpu_status())
		mxc_register_device(&mxcvpu_device, &mxc_vpu_data);
	if (!mxc_fuse_get_gpu_status())
		mxc_register_device(&gpu_device, &gpu_data);
	mxc_register_device(&mxcscc_device, NULL);
	mxc_register_device(&pm_device, &umobo_pm_data);
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	mxc_register_device(&busfreq_device, &bus_freq_data);
	mxc_register_device(&mxc_iim_device, &iim_data);
	mxc_register_device(&mxc_pwm2_device, NULL);
	mxc_register_device(&mxc_pwm1_backlight_device, &mxc_pwm_backlight_data);
	mxc_register_device(&mxcsdhc1_device, &mmc1_data);
	mxc_register_device(&mxcsdhc2_device, &mmc2_data);
	mxc_register_device(&mxcsdhc4_device, &mmc4_data);
	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
#ifdef CONFIG_ANDROID_PMEM
	mxc_register_device(&mxc_android_pmem_device, &android_pmem_data);
	mxc_register_device(&mxc_android_pmem_gpu_device,
				&android_pmem_gpu_data);
	mxc_register_device(&usb_mass_storage_device, &mass_storage_data);
	mxc_register_device(&usb_rndis_device, &rndis_data);
	mxc_register_device(&android_usb_device, &android_usb_data);
#endif /* CONFIG_ANDROID_PMEM */
	mxc_register_device(&ahci_fsl_device, &sata_data);
	mxc_register_device(&mxc_fec_device, &fec_data);
	mxc_register_device(&mxc_ptp_device, NULL);
	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
	/* ASRC is only available for MX53 TO2.0 */
	if (mx53_revision() >= IMX_CHIP_REVISION_2_0) {
		mxc_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
		clk_put(mxc_asrc_data.asrc_core_clk);
		mxc_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
		clk_put(mxc_asrc_data.asrc_audio_clk);
		mxc_register_device(&mxc_asrc_device, &mxc_asrc_data);
	}

	spi_register_board_info(mxc_psoc_device,
				ARRAY_SIZE(mxc_psoc_device));
	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));

	id95apm_audio_data.ext_ram_clk = clk_get(NULL, "emi_fast_clk");
	clk_put(id95apm_audio_data.ext_ram_clk);
	mxc_register_device(&mxc_id95apm_audio_device, &id95apm_audio_data);

	mx5_usb_dr_init();
	mx5_set_host1_vbus_func(mx53_umobo_usbh1_vbus);
	mx5_usbh1_init();
	mxc_register_device(&mxc_v4l2_device, NULL);
	mxc_register_device(&mxc_v4l2out_device, NULL);
	pm_power_off = da9053_power_off;
	pm_i2c_init(I2C1_BASE_ADDR - MX53_OFFSET);

	platform_device_register(&leds_gpio);
	umobo_add_device_buttons();
	mx53_umobo_init_msp430();
}

static void __init mx53_umobo_timer_init(void)
{
	struct clk *uart_clk;

	mx53_clocks_init(32768, 24000000, MX53_UMOBO_CKIH1, 0);

	uart_clk = clk_get_sys("mxcintuart.3", NULL);
	early_console_setup(MX53_BASE_ADDR(UART4_BASE_ADDR), uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= mx53_umobo_timer_init,
};

#ifdef CONFIG_ANDROID_PMEM
static void __init fixup_android_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	struct tag *mem_tag = 0;
	int total_mem = SZ_1G;
	int left_mem = 0, avali_mem = 0;
	int gpu_mem = SZ_64M;
	int pmem_gpu_size = android_pmem_gpu_data.size;
	int pmem_adsp_size = android_pmem_data.size;

	mxc_set_cpu_type(MXC_CPU_MX53);

	/* get mem= and gpu_memory= from cmdline */
	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "mem=");
			if (str != NULL) {
				str += 4;
				avali_mem = memparse(str, &str);
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_nommu");
			if (str != NULL)
				gpu_data.enable_mmu = 0;

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				gpu_mem = memparse(str, &str);
			}
			break;
		}
	}

	if (gpu_data.enable_mmu)
		gpu_mem = 0;

	/* get total memory from TAGS */
	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
			left_mem = total_mem - gpu_mem
				- pmem_gpu_size - pmem_adsp_size;
			break;
		}
	}

	if (avali_mem > 0 && avali_mem < left_mem)
		left_mem = avali_mem;

	if (mem_tag) {
		android_pmem_data.start = mem_tag->u.mem.start
				+ left_mem + gpu_mem + pmem_gpu_size;
		android_pmem_gpu_data.start = mem_tag->u.mem.start
				+ left_mem + gpu_mem;
		mem_tag->u.mem.size = left_mem;

		/*reserve memory for gpu*/
		if (!gpu_data.enable_mmu) {
			gpu_device.resource[5].start =
				mem_tag->u.mem.start + left_mem;
			gpu_device.resource[5].end =
				gpu_device.resource[5].start + gpu_mem - 1;
		}
	}
}
#endif /* CONFIG_ANDROID_PMEM */

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX53_UMOBO data structure.
 */
MACHINE_START(MX53_UMOBO, "Freescale MX53 U-MoBo Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
#ifdef CONFIG_ANDROID_PMEM
	.fixup = fixup_android_board,
#else
	.fixup = fixup_mxc_board,
#endif
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
