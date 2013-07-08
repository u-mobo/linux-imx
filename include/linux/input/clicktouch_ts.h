/*
 * Clicktouch Prototype touchscreen driver.
 *
 * Copyright (C) 2012, Clicktouch NV. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Cypress reserves the right to make changes without further notice
 * to the materials described herein. Cypress does not assume any
 * liability arising out of the application described herein.
 *
 * Contact Clicktouch NV at www.clicktouch.eu
 *
 */

#ifndef __CLICKTOUCH_TS_H__
#define __CLICKTOUCH_TS_H__

#include <linux/gpio_keys.h>

#define CLICKTOUCH_TS_INVERT_X	(1 << 0)
#define CLICKTOUCH_TS_INVERT_Y	(1 << 1)
#define CLICKTOUCH_TS_SWAP_XY	(1 << 2)

struct clicktouch_ts_platform_data {
	struct gpio_keys_button gpio_irq;
	int (*power_on)(int on);
	u32 flags;
};

#endif /* __CLICKTOUCH_TS_H__ */
