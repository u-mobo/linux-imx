/*
 * Copyright (C) 2011-2013 U-MoBo
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

#include <linux/init.h>
#include <linux/clk.h>
#include <linux/fec.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/fsl_devices.h>
#include <linux/ahci_platform.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif
#ifdef CONFIG_ION
#include <linux/ion.h>
#endif
#include <linux/pwm_backlight.h>
#include <linux/mxcfb.h>
#include <linux/ipu.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mfd/da9052/da9052.h>

#include <linux/i2c/pca953x.h>
#include <linux/input/adxl34x.h>
#include <linux/input/l3g_gyro.h>
#include <linux/mfd/id95apm.h>
#include <linux/wl12xx.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/memblock.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/ipu-v3.h>
#include <mach/imx-uart.h>
#include <mach/iomux-mx53.h>
#include <mach/ahci_sata.h>
#include <mach/imx_rfkill.h>
#include <mach/mxc_asrc.h>
#include <mach/mxc_dvfs.h>
#include <mach/check_fuse.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include "crm_regs.h"
#include "devices-imx53.h"
#include "devices.h"
#include "usb.h"
#include "pmic.h"

/* MX53 U-MoBo GPIO PIN configurations */
#define MX53_UMOBO_ECSPI1_CS0		IMX_GPIO_NR(2, 30)
#define MX53_UMOBO_ECSPI1_CS1		IMX_GPIO_NR(4, 10)
#define MX53_UMOBO_ECSPI2_CS0		IMX_GPIO_NR(2, 26)
#define MX53_UMOBO_ECSPI2_CS1		IMX_GPIO_NR(2, 27)
#define MX53_UMOBO_FEC_PHY_RST		IMX_GPIO_NR(5, 20)	/* GPIO_5_20 */
#define MX53_UMOBO_USB_PWREN		IMX_GPIO_NR(7, 8)	/* GPIO_7_8 */

#define MX53_UMOBO_SYSIRQ		IMX_GPIO_NR(7, 12)	/* GPIO7_12 */

#define MX53_UMOBO_SOM_GPIOIRQ		IMX_GPIO_NR(4, 3)	/* GPIO4_3 */
#define MX53_UMOBO_SOM_GPIO_PCA953X	IMX_GPIO_NR(8, 0)	/* start at GPIO8_0 that not exist */
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

#define MX53_UMOBO_GPIO_ID95APM		IMX_GPIO_NR(8, 16)	/* start at GPIO8_16 that not exist */
#define MX53_UMOBO_BASEBOARD_LCD_nIRQ	(MX53_UMOBO_GPIO_ID95APM + 1)	/* GPIO8_17 */
#define MX53_UMOBO_BASEBOARD_DL2	(MX53_UMOBO_GPIO_ID95APM + 2)	/* GPIO8_18 */
#define MX53_UMOBO_BASEBOARD_DL1	(MX53_UMOBO_GPIO_ID95APM + 3)	/* GPIO8_19 */
#define MX53_UMOBO_BASEBOARD_DL3	(MX53_UMOBO_GPIO_ID95APM + 4)	/* GPIO8_20 */
#define MX53_UMOBO_HP_DET		(MX53_UMOBO_GPIO_ID95APM + 10)	/* GPIO8_26 */

#define MX53_UMOBO_SENS_GPIOIRQ		IMX_GPIO_NR(3, 29)	/* GPIO3_29 */
#define MX53_UMOBO_SENS_GPIO_PCA953X	IMX_GPIO_NR(9, 0)		/* start at GPIO9_0 that not exist */
#define MX53_UMOBO_SENS_ADXL34X_IRQ1	(MX53_UMOBO_SENS_GPIO_PCA953X + 0)	/* GPIO9_0 */
#define MX53_UMOBO_SENS_ADXL34X_IRQ2	(MX53_UMOBO_SENS_GPIO_PCA953X + 1)	/* GPIO9_1 */
#define MX53_UMOBO_SENS_MAG3110_IRQ	(MX53_UMOBO_SENS_GPIO_PCA953X + 2)	/* GPIO9_2 */
#define MX53_UMOBO_SENS_L3G_GYRO_IRQ1	(MX53_UMOBO_SENS_GPIO_PCA953X + 3)	/* GPIO9_3 */
#define MX53_UMOBO_SENS_L3G_GYRO_IRQ2	(MX53_UMOBO_SENS_GPIO_PCA953X + 4)	/* GPIO9_4 */
#define MX53_UMOBO_SENS_MPL3115A2_IRQ1	(MX53_UMOBO_SENS_GPIO_PCA953X + 5)	/* GPIO9_5 */
#define MX53_UMOBO_SENS_MPL3115A2_IRQ2	(MX53_UMOBO_SENS_GPIO_PCA953X + 6)	/* GPIO9_6 */
#define MX53_UMOBO_SENS_UC430_RESET	(MX53_UMOBO_SENS_GPIO_PCA953X + 7)	/* GPIO9_7 */

#define MX53_UMOBO_LCD_GPIOIRQ		MX53_UMOBO_BASEBOARD_LCD_nIRQ
#define MX53_UMOBO_LCD_GPIO_PCA953X	IMX_GPIO_NR(9, 8)	/* start at GPIO9_8 that not exist */
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
#define MX53_UMOBO_TIWI_WLAN_IRQ	IMX_GPIO_NR(3, 13)	/* GPIO3_13 */
#define MX53_UMOBO_TIWI_BT_EN		MX53_UMOBO_SOM_GPIO14
#define MX53_UMOBO_TIWI_BT_WAKEUP	IMX_GPIO_NR(1, 5)	/* GPIO1_5 */
#define MX53_UMOBO_TIWI_FM_EN		MX53_UMOBO_SOM_GPIO15

#define MX53_UMOBO_SD2_CD		IMX_GPIO_NR(1, 4)
#define MX53_UMOBO_SD2_WP		IMX_GPIO_NR(1, 2)

#define TZIC_WAKEUP0_OFFSET		(0x0E00)
#define TZIC_WAKEUP1_OFFSET		(0x0E04)
#define TZIC_WAKEUP2_OFFSET		(0x0E08)
#define TZIC_WAKEUP3_OFFSET		(0x0E0C)
#define GPIO7_0_12_IRQ_BIT		(0x1<<12)

#define MX53_UMOBO_CKIH1		24576000

#define MX53_TIWI_IRQ_PAD_CTRL		(PAD_CTL_PUS_100K_DOWN | PAD_CTL_PUE | PAD_CTL_PKE)

void __init early_console_setup(unsigned long base, struct clk *clk);
static struct clk *sata_clk, *sata_ref_clk;

#ifdef CONFIG_ANDROID_PMEM
extern struct platform_device mxc_android_pmem_device;
extern struct platform_device mxc_android_pmem_gpu_device;
#endif

extern char *lp_reg_id;
extern char *gp_reg_id;
extern void mx5_cpu_regulator_init(void);
extern int mx53_umobo_init_da9052(void);
extern int mx53_umobo_init_msp430(void);

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
	MX53_PAD_PATA_DATA8__ESDHC1_DAT4,
	MX53_PAD_PATA_DATA9__ESDHC1_DAT5,
	MX53_PAD_PATA_DATA10__ESDHC1_DAT6,
	MX53_PAD_PATA_DATA11__ESDHC1_DAT7,

	/* SD2: on U-MoBo */
	MX53_PAD_SD2_CMD__ESDHC2_CMD,
	MX53_PAD_SD2_CLK__ESDHC2_CLK,
	MX53_PAD_SD2_DATA0__ESDHC2_DAT0,
	MX53_PAD_SD2_DATA1__ESDHC2_DAT1,
	MX53_PAD_SD2_DATA2__ESDHC2_DAT2,
	MX53_PAD_SD2_DATA3__ESDHC2_DAT3,
	//MX53_PAD_GPIO_4__ESDHC2_CD,	/* SD2_CD */
	//MX53_PAD_GPIO_2__ESDHC2_WP,	/* SD2_WP */
	MX53_PAD_GPIO_4__GPIO1_4,	/* SD2_CD */
	MX53_PAD_GPIO_2__GPIO1_2,	/* SD2_WP */

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

#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake, debounce_ms) \
{                                                               \
	.gpio           = gpio_num,                             \
	.type           = EV_KEY,                               \
	.code           = ev_code,                              \
	.active_low     = act_low,                              \
	.desc           = descr,                         \
	.wakeup         = wake,                                 \
	.debounce_interval = debounce_ms,                       \
}

static struct gpio_keys_button umobo_lcd_buttons[] = {
	GPIO_BUTTON(MX53_UMOBO_LCD_SW1, KEY_HOMEPAGE, 1, "lcd-sw1", 0, 0),
	GPIO_BUTTON(MX53_UMOBO_LCD_SW2, KEY_MENU, 1, "lcd-sw2", 0, 0),
	GPIO_BUTTON(MX53_UMOBO_LCD_SW3, KEY_BACK, 1, "lcd-sw3", 0, 0),
};

#define UMOBO_LCD_BUTTONS_POLL_MS 100

static struct gpio_keys_platform_data umobo_lcd_button_data = {
	.buttons        = umobo_lcd_buttons,
	.nbuttons       = ARRAY_SIZE(umobo_lcd_buttons),
	.poll_interval	= UMOBO_LCD_BUTTONS_POLL_MS,
};

static struct platform_device umobo_lcd_button_device = {
	.name           = "gpio-keys-polled",
	.id             = -1,
	.num_resources  = 0,
	.dev            = {
		.platform_data = &umobo_lcd_button_data,
		}
};

static struct gpio_led umobo_gpio_leds[] = {
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

static struct gpio_led_platform_data umobo_gpio_leds_info = {
	.leds		= umobo_gpio_leds,
	.num_leds	= ARRAY_SIZE(umobo_gpio_leds),
};

static struct platform_device umobo_leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &umobo_gpio_leds_info,
	},
};

static const struct imxuart_platform_data mx53_umobo_uart0_data __initconst = {
	.flags = IMXUART_HAVE_RTSCTS | IMXUART_USE_DCEDTE,
};

static const struct imxuart_platform_data mx53_umobo_uart1_data __initconst = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static const struct imxuart_platform_data mx53_umobo_uart2_data __initconst = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static const struct imxuart_platform_data mx53_umobo_uart3_data __initconst = {
	.flags = 0,
};

static const struct imxuart_platform_data mx53_umobo_uart4_data __initconst = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static inline void mx53_umobo_init_uart(void)
{
	imx53_add_imx_uart(0, &mx53_umobo_uart0_data);
	imx53_add_imx_uart(1, &mx53_umobo_uart1_data);
	imx53_add_imx_uart(2, &mx53_umobo_uart2_data);
	imx53_add_imx_uart(3, &mx53_umobo_uart3_data);
	imx53_add_imx_uart(4, &mx53_umobo_uart4_data);
}

static inline void mx53_umobo_fec_reset(void)
{
	int ret;

	/* reset FEC PHY */
	ret = gpio_request(MX53_UMOBO_FEC_PHY_RST, "fec-phy-reset");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO_FEC_PHY_RESET: %d\n", ret);
		return;
	}
	gpio_direction_output(MX53_UMOBO_FEC_PHY_RST, 0);
	msleep(1);
	gpio_set_value(MX53_UMOBO_FEC_PHY_RST, 1);
}

static struct fec_platform_data mx53_umobo_fec_data = {
	.phy = PHY_INTERFACE_MODE_RMII,
};

static const struct imxi2c_platform_data mx53_umobo_i2c_data __initconst = {
	.bitrate = 100000,
};

extern void __iomem *tzic_base;
static void umobo_da9053_irq_wakeup_only_fixup(void)
{
	if (NULL == tzic_base) {
		pr_err("fail to map MX53_TZIC_BASE_ADDR\n");
		return;
	}
	__raw_writel(0, tzic_base + TZIC_WAKEUP0_OFFSET);
	__raw_writel(0, tzic_base + TZIC_WAKEUP1_OFFSET);
	__raw_writel(0, tzic_base + TZIC_WAKEUP2_OFFSET);
	/* only enable irq wakeup for da9053 */
	__raw_writel(GPIO7_0_12_IRQ_BIT, tzic_base + TZIC_WAKEUP3_OFFSET);
	pr_info("only da9053 irq is wakeup-enabled\n");
}

static void umobo_suspend_enter(void)
{
	/* FIXME: we have board_is_rev??? */
	if (board_is_rev(IMX_BOARD_REV_4)) {
		umobo_da9053_irq_wakeup_only_fixup();
		da9053_suspend_cmd_sw();
	} else {
		if (da9053_get_chip_version() != DA9053_VERSION_BB)
			umobo_da9053_irq_wakeup_only_fixup();

		da9053_suspend_cmd_hw();
	}
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

/* SDIO Card Slot on SoM */
static const struct esdhc_platform_data mx53_umobo_sd1_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.delay_line = 0,
	.support_8bit = 1,
	.cd_type = ESDHC_CD_PERMANENT,
};

/* SDIO Card Slot on U-MoBo */
static const struct esdhc_platform_data mx53_umobo_sd2_data __initconst = {
	.cd_gpio = MX53_UMOBO_SD2_CD,
	.wp_gpio = MX53_UMOBO_SD2_WP,
	.keep_power_at_suspend = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
};

/* SDIO WiFi */
static const struct esdhc_platform_data mx53_umobo_sd4_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
	.runtime_pm = 1,
};

#ifdef UMOBO_CAMERA
static void mx53_umobo_csi0_cam_powerdown(int powerdown)
{
	struct clk *clk = clk_get(NULL, "ssi_ext1_clk");
	if (!clk)
		printk(KERN_DEBUG "Failed to get ssi_ext1_clk\n");

	if (powerdown) {
		/* Power off */
		gpio_set_value_cansleep(MX53_UMOBO_SOM_CIS_CTL, 0);
		if (clk)
			clk_disable(clk);
	} else {
		if (clk)
			clk_enable(clk);
		/* Power Up */
		gpio_set_value_cansleep(MX53_UMOBO_SOM_CIS_CTL, 1);
		msleep(2);
	}
}

static void mx53_umobo_csi0_io_init(void)
{
	struct clk *clk;
	uint32_t freq = 0;

	clk = clk_get(NULL, "ssi_ext1_clk");
	if (clk) {
		freq = clk_round_rate(clk, 24000000);
		clk_set_rate(clk, freq);
		clk_enable(clk);
	} else
		printk(KERN_DEBUG "Failed to get ssi_ext1_clk\n");

	/* Camera reset */
	gpio_request(MX53_UMOBO_SOM_CIS_RESET, "cam-reset");
	gpio_direction_output(MX53_UMOBO_SOM_CIS_RESET, 1);

	/* Camera power down */
	gpio_request(MX53_UMOBO_SOM_CIS_CTL, "cam-pwdn");
	gpio_direction_output(MX53_UMOBO_SOM_CIS_CTL, 0);
	mx53_umobo_csi0_cam_powerdown(1);
	msleep(5);
	mx53_umobo_csi0_cam_powerdown(0);
	msleep(5);
	gpio_set_value_cansleep(MX53_UMOBO_SOM_CIS_RESET, 0);
	msleep(1);
	gpio_set_value_cansleep(MX53_UMOBO_SOM_CIS_RESET, 1);
	msleep(5);
	mx53_umobo_csi0_cam_powerdown(1);
}

static struct fsl_mxc_camera_platform_data camera_data = {
	.analog_regulator = "DA9052_LDO7",
	.core_regulator = "DA9052_LDO9",
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx53_umobo_csi0_io_init,
	.pwdn = mx53_umobo_csi0_cam_powerdown,
};
#endif

static struct fsl_mxc_capture_platform_data capture_data = {
	.csi = 0,
	.ipu = 0,
	.mclk_source = 0,
	.is_mipi = 0,
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

#ifdef CONFIG_WL12XX_PLATFORM_DATA
static struct regulator_consumer_supply umobo_mmc4_supply =
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3");

struct regulator_init_data umobo_wl1271_reg_initdata = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies = &umobo_mmc4_supply,
};

static struct fixed_voltage_config umobo_wl1271_reg_config = {
	.supply_name		= "WLAN_EN",
	.microvolts		= 1800000,
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
		gpio_set_value_cansleep(MX53_UMOBO_TIWI_BT_EN, 1);
		mdelay(15);
	} else {
		gpio_set_value_cansleep(MX53_UMOBO_TIWI_BT_EN, 0);
		udelay(100);
	}
	return 0;
}

static struct platform_device umobo_bt_rfkill = {
	.name = "mxc_bt_rfkill",
};

static struct imx_bt_rfkill_platform_data umobo_bt_rfkill_data = {
	.power_change = mx53_umobo_bt_power_change,
};
#endif /* CONFIG_WL12XX_PLATFORM_DATA */

static void mx53_umobo_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value_cansleep(MX53_UMOBO_SOM_OTG_VBUSEN, 1);
	else
		gpio_set_value_cansleep(MX53_UMOBO_SOM_OTG_VBUSEN, 0);
}

static void mx53_umobo_usbh1_vbus(bool on)
{
	if (on)
		gpio_set_value_cansleep(MX53_UMOBO_SOM_USBH_VBUSEN, 1);
	else
		gpio_set_value_cansleep(MX53_UMOBO_SOM_USBH_VBUSEN, 0);
}

static void __init mx53_umobo_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX53_IO_ADDRESS(MX53_OTG_BASE_ADDR);

	ret = gpio_request(MX53_UMOBO_SOM_OTG_VBUSEN, "otg-vbusen");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO MX53_UMOBO_SOM_OTG_VBUSEN: %d\n", ret);
		return;
	}
	gpio_direction_output(MX53_UMOBO_SOM_OTG_VBUSEN, 0);

	ret = gpio_request(MX53_UMOBO_SOM_USBH_VBUSEN, "usbh-vbusen");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO MX53_UMOBO_SOM_USBH_VBUSEN: %d\n", ret);
		return;
	}
	gpio_direction_output(MX53_UMOBO_SOM_USBH_VBUSEN, 0);

	mx5_set_otghost_vbus_func(mx53_umobo_usbotg_vbus);
	mx5_usb_dr_init();
	mx5_set_host1_vbus_func(mx53_umobo_usbh1_vbus);
	mx5_usbh1_init();
}

static int __init umobo_som_pca953x_setup(struct i2c_client *client,
					unsigned gpio_base, unsigned ngpio,
					void *context) {
	mx53_umobo_init_usb();

	/* enable VID1 & VID2 */
	gpio_request(MX53_UMOBO_SOM_VID1_PWREN, "vid1-pwren");
	gpio_request(MX53_UMOBO_SOM_VID1_RESET, "vid1-reset");
	gpio_request(MX53_UMOBO_SOM_VID2_PWREN, "vid2-pwren");
	gpio_request(MX53_UMOBO_SOM_VID2_RESET, "vid2-reset");
	gpio_direction_output(MX53_UMOBO_SOM_VID1_PWREN, 1);
	gpio_direction_output(MX53_UMOBO_SOM_VID1_RESET, 1);
	gpio_direction_output(MX53_UMOBO_SOM_VID2_PWREN, 1);
	gpio_direction_output(MX53_UMOBO_SOM_VID2_RESET, 1);
	gpio_free(MX53_UMOBO_SOM_VID1_PWREN);
	gpio_free(MX53_UMOBO_SOM_VID1_RESET);
	gpio_free(MX53_UMOBO_SOM_VID2_PWREN);
	gpio_free(MX53_UMOBO_SOM_VID2_RESET);

#ifdef CONFIG_WL12XX_PLATFORM_DATA
	gpio_request(MX53_UMOBO_TIWI_WLAN_EN, "wlan-en");
	gpio_request(MX53_UMOBO_TIWI_BT_EN, "bt-en");
	gpio_request(MX53_UMOBO_TIWI_FM_EN, "fm-en");
	gpio_direction_output(MX53_UMOBO_TIWI_WLAN_EN, 0);
	gpio_direction_output(MX53_UMOBO_TIWI_BT_EN, 0);
	gpio_direction_output(MX53_UMOBO_TIWI_FM_EN, 0);
	gpio_free(MX53_UMOBO_TIWI_WLAN_EN);

	/* WL12xx WLAN Init */
	if (wl12xx_set_platform_data(&umobo_wl1271_data))
		pr_err("error setting wl12xx data\n");
	platform_device_register(&umobo_wl1271_reg_device);

	/* provide BT */
	mxc_register_device(&umobo_bt_rfkill, &umobo_bt_rfkill_data);
#endif /* CONFIG_WL12XX_PLATFORM_DATA */

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

static struct pca953x_platform_data umobo_som_pca953x_data __initdata = {
	.gpio_base = MX53_UMOBO_SOM_GPIO_PCA953X,	// no line used as irq
	.setup = umobo_som_pca953x_setup,
	.irq_base = -1,
};	// SoM

static struct l3g_gyro_platform_data umobo_l3g_gyro_data = {
	.poll_interval = 80,
	.fs_range = L3G_GYRO_FS_2000DPS,
	.temp_calibration = L3G_GYRO_DEFAULT_TEMP_CALIBRATION,
	.gpio_int1 = MX53_UMOBO_SENS_L3G_GYRO_IRQ1,
	.gpio_int2 = MX53_UMOBO_SENS_L3G_GYRO_IRQ2,
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

static struct fsl_mxc_lightsensor_platform_data umobo_isl29023_data = {
	.rext = 100,    /* calibration: 499K->700K */
};

static int mag3110_position = 6;

static int umobo_id95apm_init(struct id95apm *id95apm)
{
	gpio_request(MX53_UMOBO_SYSIRQ, "sysirq");
	gpio_direction_input(MX53_UMOBO_SYSIRQ);
	return 0;
}

static struct regulator_consumer_supply ldo6_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
};

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
		.gpio_base = MX53_UMOBO_GPIO_ID95APM,
	},

	/* Power Switch Detector: SW_DET */
	.pwrkey_init = {
		.codes = { KEY_HOMEPAGE, KEY_BACK, KEY_MENU},
	},
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
		.platform_data = &umobo_l3g_gyro_data,
		// FIXME? ST driver does not implement IRQ management
	},
	{
		I2C_BOARD_INFO("l3g4200d", 0x69),
		.platform_data = &umobo_l3g_gyro_data,
		// FIXME? ST driver does not implement IRQ management
	},
	{
		I2C_BOARD_INFO("mpl3115", 0x60),
		// FIXME? mpl3115 driver does not implement IRQ management
	},
	{
		I2C_BOARD_INFO("adxl34x", 0x1d),
		.platform_data = &umobo_adxl34x_data,
		.irq = gpio_to_irq(MX53_UMOBO_SENS_ADXL34X_IRQ1),
	},
	{
		I2C_BOARD_INFO("mag3110", 0x0e),
		.platform_data = (void *)&mag3110_position,
		.irq = gpio_to_irq(MX53_UMOBO_SENS_MAG3110_IRQ),
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

static int mx53_umobo_spi1_cs[] = {
	MX53_UMOBO_ECSPI1_CS0,
	MX53_UMOBO_ECSPI1_CS1,
};

static int mx53_umobo_spi2_cs[] = {
	MX53_UMOBO_ECSPI2_CS0,
	MX53_UMOBO_ECSPI2_CS1,
};

static struct spi_imx_master mx53_umobo_spi1_data = {
	.chipselect = mx53_umobo_spi1_cs,
	.num_chipselect = ARRAY_SIZE(mx53_umobo_spi1_cs),
};

static struct spi_imx_master mx53_umobo_spi2_data = {
	.chipselect = mx53_umobo_spi2_cs,
	.num_chipselect = ARRAY_SIZE(mx53_umobo_spi2_cs),
};

static struct spi_board_info mxc_spi1_board_info[] __initdata = {
	{
	 .modalias = "spidev",
	 .max_speed_hz = 20000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 0,
	 .chip_select = 0,		/* DEVICE = /dev/spidev0.0 */
	},
	{
	 .modalias = "spidev",
	 .max_speed_hz = 20000000,	/* max spi clock (SCK) speed in HZ */
	 .bus_num = 0,
	 .chip_select = 1,		/* DEVICE = /dev/spidev0.1 */
	},
};

static struct spi_board_info mxc_spi2_board_info[] __initdata = {
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


static struct mxc_iim_platform_data iim_data = {
	.bank_start = MXC_IIM_MX53_BANK_START_ADDR,
	.bank_end   = MXC_IIM_MX53_BANK_END_ADDR,
	.enable_fuse = mxc_iim_enable_fuse,
	.disable_fuse = mxc_iim_disable_fuse,
};


#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data android_pmem_data = {
	.name = "pmem_adsp",
	.size = SZ_64M,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_gpu_data = {
	.name = "pmem_gpu",
	.size = SZ_64M,
	.cached = 1,
};
#endif

#ifdef CONFIG_ION
#define	ION_VPU	0
#define	ION_GPU	1
static struct ion_platform_data imx_ion_data = {
	.nr = 2,
	.heaps = {
		{
		.id = ION_VPU,
		.type = ION_HEAP_TYPE_CARVEOUT,
		.name = "vpu_ion",
		.size = SZ_64M,
		},
		{
		.id = ION_GPU,
		.type = ION_HEAP_TYPE_CARVEOUT,
		.name = "gpu_ion",
		.size = SZ_64M,
		},
	},
};
#endif

/* HW Initialization, if return 0, initialization is successful. */
static int mx53_umobo_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	/* Enable SATA PWR: DA9052 LDO5  */
	/* FIXME */

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

	sata_ref_clk = clk_get(NULL, "usb_phy1_clk");
	if (IS_ERR(sata_ref_clk)) {
		dev_err(dev, "no sata ref clock.\n");
		ret = PTR_ERR(sata_ref_clk);
		goto release_sata_clk;
	}
	ret = clk_enable(sata_ref_clk);
	if (ret) {
		dev_err(dev, "can't enable sata ref clock.\n");
		goto put_sata_ref_clk;
	}

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb_clk");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_ref_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;

release_sata_ref_clk:
	clk_disable(sata_ref_clk);
put_sata_ref_clk:
	clk_put(sata_ref_clk);
release_sata_clk:
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

static void mx53_umobo_sata_exit(struct device *dev)
{
	clk_disable(sata_ref_clk);
	clk_put(sata_ref_clk);

	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx53_umobo_sata_data = {
	.init = mx53_umobo_sata_init,
	.exit = mx53_umobo_sata_exit,
};

static struct mxc_audio_platform_data id95apm_audio_data = {
	/*
	 * SSI pre-dividers should use divider values larger than '1'.
	 * Ref i.MX53 Reference Manual, rev 2.1 06/2012,
	 * pag 829, end of paragraph 18.2.1.4.7
	 */
	.sysclk = MX53_UMOBO_CKIH1 >> 1,
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.hp_gpio = MX53_UMOBO_HP_DET,
	.hp_active_low = 0,
};

static struct platform_device mxc_id95apm_audio_device = {
	.name = "imx-id95apm",
};

static struct imx_ssi_platform_data umobo_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB24,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static struct ipuv3_fb_platform_data umobo_fb_data[] = {
	{
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "SAMSUNG-LMS700",
	.default_bpp = 32,
	.int_clk = false,
	.late_init = false,
	.panel_width_mm = 152,
	.panel_height_mm = 91,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 32,
	.int_clk = false,
	.late_init = false,
	.panel_width_mm = 203,
	.panel_height_mm = 152,
	},
};

static struct imx_ipuv3_platform_data ipu_data = {
	.rev = 3,
	.csi_clk[0] = "ssi_ext1_clk",
	.bypass_reset = false,
};

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_gpu_platform_data mx53_umobo_gpu_pdata __initdata = {
	.enable_mmu = 0,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 0,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SIN1,
};

static struct mxc_dvfs_platform_data umobo_dvfs_core_data = {
	.reg_id = "cpu_vddgp",
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

static struct mxc_regulator_platform_data umobo_regulator_data = {
	.cpu_reg_id = "cpu_vddgp",
};

extern struct imx_mxc_gpu_data imx53_gpu_data;

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
#ifdef CONFIG_ANDROID_PMEM
			str = t->u.cmdline.cmdline;
			str = strstr(str, "pmem=");
			if (str != NULL) {
				str += 5;
				android_pmem_gpu_data.size =
						memparse(str, &str);
				if (*str == ',') {
					str++;
					android_pmem_data.size =
						memparse(str, &str);
				}
			}
#endif
#ifdef CONFIG_ION
			str = t->u.cmdline.cmdline;
			str = strstr(str, "ion=");
			if (str != NULL) {
				str += 4;
				imx_ion_data.heaps[ION_GPU].size =
						memparse(str, &str);
				if (*str == ',') {
					str++;
					imx_ion_data.heaps[ION_VPU].size =
						memparse(str, &str);
				}
			}
#endif

			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				umobo_fb_data[i++].res_size[0] =
						memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(umobo_fb_data)) {
					str++;
					umobo_fb_data[i++].res_size[0] =
						memparse(str, &str);
				}
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				imx53_gpu_data.gmem_reserved_size =
						memparse(str, &str);
			}
			break;
		}
	}
}

static void mx53_umobo_power_off(void)
{
	/* FIXME: disable LVDS0/1 power */
	/* FIXME: disable 1V8 voltage */

	/* power off by sending shutdown command to da9053*/
	da9053_power_off();
}

static int __init mx53_umobo_power_init(void)
{
	/* cpu get regulator needs to be in lateinit so that
	   regulator list gets updated for i2c da9052 regulators */
	mx5_cpu_regulator_init();

	if (machine_is_mx53_umobo())
		pm_power_off = mx53_umobo_power_off;

	return 0;
}
late_initcall(mx53_umobo_power_init);

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

static int __init imx5x_add_ram_console(void)
{
	return platform_device_register(&android_ram_console);
}
#else
#define imx5x_add_ram_console() do {} while (0)
#endif

static void __init mx53_umobo_board_init(void)
{
	int i;

	mxc_iomux_v3_setup_multiple_pads(mx53_umobo_pads,
					ARRAY_SIZE(mx53_umobo_pads));

	/* FIXME: enable MX53_UMOBO_DCDC1V8_EN */
	/* FIXME: enable MX53_UMOBO_DCDC5V_EN */

	gp_reg_id = umobo_regulator_data.cpu_reg_id;
	lp_reg_id = umobo_regulator_data.vcc_reg_id;

	mx53_umobo_init_uart();
	imx5x_add_ram_console();
	mx53_umobo_fec_reset();
	mxc_register_device(&mxc_pm_device, &umobo_pm_data);
	imx53_add_fec(&mx53_umobo_fec_data);
	imx53_add_imx2_wdt(0, NULL);
	imx53_add_srtc();
	imx53_add_imx_i2c(0, &mx53_umobo_i2c_data);
	imx53_add_imx_i2c(1, &mx53_umobo_i2c_data);
	imx53_add_imx_i2c(2, &mx53_umobo_i2c_data);
	imx53_add_ecspi(0, &mx53_umobo_spi1_data);
	imx53_add_ecspi(1, &mx53_umobo_spi2_data);

	imx53_add_ipuv3(0, &ipu_data);
	for (i = 0; i < ARRAY_SIZE(umobo_fb_data); i++)
		imx53_add_ipuv3fb(i, &umobo_fb_data[i]);
	imx53_add_lcdif(&lcdif_data);
	if (!mxc_fuse_get_vpu_status())
		imx53_add_vpu();
	imx53_add_ldb(&ldb_data);
	imx53_add_v4l2_output(0);
	imx53_add_v4l2_capture(0, &capture_data);

	imx53_add_mxc_pwm(1);
	imx53_add_mxc_pwm_backlight(0, &mxc_pwm_backlight_data);

	imx53_add_sdhci_esdhc_imx(0, &mx53_umobo_sd1_data);
	imx53_add_sdhci_esdhc_imx(1, &mx53_umobo_sd2_data);
	imx53_add_sdhci_esdhc_imx(3, &mx53_umobo_sd4_data);

	imx53_add_ahci(0, &mx53_umobo_sata_data);
	mxc_register_device(&imx_ahci_device_hwmon, NULL);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx53_add_asrc(&imx_asrc_data);

	imx53_add_iim(&iim_data);

	mx53_umobo_init_da9052();
	mx53_umobo_init_msp430();

	spi_register_board_info(mxc_spi1_board_info,
				ARRAY_SIZE(mxc_spi1_board_info));
	spi_register_board_info(mxc_spi2_board_info,
				ARRAY_SIZE(mxc_spi2_board_info));

	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
				ARRAY_SIZE(mxc_i2c2_board_info));

	imx53_add_imx_ssi(1, &umobo_ssi_pdata);
	mxc_register_device(&mxc_id95apm_audio_device, &id95apm_audio_data);

#ifdef CONFIG_ANDROID_PMEM
	mxc_register_device(&mxc_android_pmem_device, &android_pmem_data);
	mxc_register_device(&mxc_android_pmem_gpu_device,
				&android_pmem_gpu_data);
#endif
#ifdef CONFIG_ION
	imx53_add_ion(0, &imx_ion_data,
		sizeof(imx_ion_data) + (imx_ion_data.nr * sizeof(struct ion_platform_heap)));
#endif

	/*GPU*/
	if (mx53_revision() >= IMX_CHIP_REVISION_2_0)
		mx53_umobo_gpu_pdata.z160_revision = 1;
	else
		mx53_umobo_gpu_pdata.z160_revision = 0;

	if (!mxc_fuse_get_gpu_status())
		imx53_add_mxc_gpu(&mx53_umobo_gpu_pdata);

	/* this call required to release SCC RAM partition held by ROM
	  * during boot, even if SCC2 driver is not part of the image
	  */
	imx53_add_mxc_scc2();
	pm_i2c_init(MX53_I2C1_BASE_ADDR);

	imx53_add_dvfs_core(&umobo_dvfs_core_data);
	imx53_add_busfreq();

	platform_device_register(&umobo_leds_gpio);

}

static void __init mx53_umobo_timer_init(void)
{
	struct clk *uart_clk;

	mx53_clocks_init(32768, 24000000, MX53_UMOBO_CKIH1, 0);

	uart_clk = clk_get_sys("imx-uart.3", NULL);
	early_console_setup(MX53_UART4_BASE_ADDR, uart_clk);
}

static struct sys_timer mx53_umobo_timer = {
	.init	= mx53_umobo_timer_init,
};

#define SZ_TRIPLE_1080P	ALIGN((1920*ALIGN(1080, 128)*2*3), SZ_4K)
static void __init mx53_umobo_reserve(void)
{
	phys_addr_t phys;
	int i;

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	phys = memblock_alloc(SZ_1M, SZ_4K);
	memblock_remove(phys, SZ_1M);
	ram_console_resource.start = phys;
	ram_console_resource.end   = phys + SZ_1M - 1;
#endif

	if (imx53_gpu_data.gmem_reserved_size) {
		phys = memblock_alloc(imx53_gpu_data.gmem_reserved_size,
					   SZ_4K);
		memblock_remove(phys, imx53_gpu_data.gmem_reserved_size);
		imx53_gpu_data.gmem_reserved_base = phys;
	}
#ifdef CONFIG_ANDROID_PMEM
	if (android_pmem_data.size) {
		phys = memblock_alloc(android_pmem_data.size, SZ_4K);
		memblock_remove(phys, android_pmem_data.size);
		android_pmem_data.start = phys;
	}

	if (android_pmem_gpu_data.size) {
		phys = memblock_alloc(android_pmem_gpu_data.size, SZ_4K);
		memblock_remove(phys, android_pmem_gpu_data.size);
		android_pmem_gpu_data.start = phys;
	}
#endif
#ifdef CONFIG_ION
	if (imx_ion_data.heaps[ION_VPU].size) {
		phys = memblock_alloc(imx_ion_data.heaps[ION_VPU].size, SZ_4K);
		memblock_remove(phys, imx_ion_data.heaps[ION_VPU].size);
		imx_ion_data.heaps[ION_VPU].base = phys;
	}

	if (imx_ion_data.heaps[ION_GPU].size) {
		phys = memblock_alloc(imx_ion_data.heaps[ION_GPU].size, SZ_4K);
		memblock_remove(phys, imx_ion_data.heaps[ION_GPU].size);
		imx_ion_data.heaps[ION_GPU].base = phys;
	}
#endif

	for (i = 0; i < ARRAY_SIZE(umobo_fb_data); i++)
		if (umobo_fb_data[i].res_size[0]) {
			/* reserve for background buffer */
			phys = memblock_alloc(umobo_fb_data[i].res_size[0],
						SZ_4K);
			memblock_remove(phys, umobo_fb_data[i].res_size[0]);
			umobo_fb_data[i].res_base[0] = phys;

			/* reserve for overlay buffer */
			phys = memblock_alloc(SZ_TRIPLE_1080P, SZ_4K);
			memblock_remove(phys, SZ_TRIPLE_1080P);
			umobo_fb_data[i].res_base[1] = phys;
			umobo_fb_data[i].res_size[1] = SZ_TRIPLE_1080P;
		}
}

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX53_UMOBO data structure.
 */
MACHINE_START(MX53_UMOBO, "Freescale iMX53 U-MoBo Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.fixup = fixup_mxc_board,
	.map_io = mx53_map_io,
	.init_early = imx53_init_early,
	.init_irq = mx53_init_irq,
	.timer = &mx53_umobo_timer,
	.init_machine = mx53_umobo_board_init,
	.reserve = mx53_umobo_reserve,
MACHINE_END
