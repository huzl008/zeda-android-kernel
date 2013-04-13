/*
 * linux/arch/arm/mach-omap2/board-omap3evm.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/input.h>
#ifdef CONFIG_MACH_FLASHBOARD
#include <linux/gpio_keys.h>
#include <linux/i2c/lis331dlh.h>
#include <linux/i2c/l3g4200dh.h>
#else
#include <linux/input/matrix_keypad.h>
#endif
#include <linux/leds.h>
#include <linux/interrupt.h>
#include <linux/mtd/nand.h>

#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c/twl.h>
#include <linux/usb/otg.h>
#include <linux/smsc911x.h>
#include <linux/usb/android_composite.h>

#include <linux/regulator/machine.h>
#include <linux/mmc/host.h>

#ifdef CONFIG_WL12XX_PLATFORM_DATA
#include <linux/wl12xx.h>
#include <linux/regulator/fixed.h>
#endif

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/usb.h>
#include <plat/common.h>
#include <plat/mcspi.h>
#include <plat/display.h>
#include <plat/omap-pm.h>
#include <plat/gpmc.h>

#include "mux.h"
#include "sdram-micron-mt46h32m32lf-6.h"
#include "sdram-hynix-h8kds0un0mer-4em.h"
#include "hsmmc.h"
#include "board-flash.h"

#ifdef CONFIG_SND_SOC_WL1271BT
#include "control.h"
#endif

#ifdef CONFIG_MACH_FLASHBOARD
#define OMAP3_EVM_TS_GPIO	177
#else
#define OMAP3_EVM_TS_GPIO	175
#endif

#define OMAP3_EVM_EHCI_VBUS	22
#define OMAP3_EVM_EHCI_SELECT	61

#define OMAP3EVM_ETHR_START	0x2c000000
#define OMAP3EVM_ETHR_SIZE	1024
#define OMAP3EVM_ETHR_ID_REV	0x50

#ifdef CONFIG_MACH_FLASHBOARD
#define OMAP3EVM_ETHR_GPIO_IRQ	13
#define OMAP3EVM_SMSC911X_CS	3
#else
#define OMAP3EVM_ETHR_GPIO_IRQ	176
#define OMAP3EVM_SMSC911X_CS	5
#endif



#ifdef CONFIG_USB_ANDROID

#define GOOGLE_VENDOR_ID		0x18d1
#define GOOGLE_PRODUCT_ID		0x9018
#define GOOGLE_ADB_PRODUCT_ID		0x9015

static char *usb_functions_adb[] = {
	"adb",
};

static char *usb_functions_mass_storage[] = {
	"usb_mass_storage",
};
static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_all[] = {
	"adb", "usb_mass_storage",
};

static struct android_usb_product usb_products[] = {
	{
		.product_id	= GOOGLE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,
	},
	{
		.product_id	= GOOGLE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_mass_storage),
		.functions	= usb_functions_mass_storage,
	},
	{
		.product_id	= GOOGLE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "rowboat",
	.product	= "rowboat gadget",
	.release	= 0x100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= GOOGLE_VENDOR_ID,
	.product_id	= GOOGLE_PRODUCT_ID,
	.functions	= usb_functions_all,
	.num_products	= ARRAY_SIZE(usb_products),
	.products	= usb_products,
	.version	= 0x0100,
	.product_name	= "rowboat gadget",
	.manufacturer_name	= "rowboat",
	.serial_number	= "20100720",
	.num_functions	= ARRAY_SIZE(usb_functions_all),
};

static struct platform_device androidusb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

static void omap3evm_android_gadget_init(void)
{
	platform_device_register(&androidusb_device);
}

#endif
static u8 omap3_evm_version;

extern void omap_pm_sys_offmode_select(int);
extern void omap_pm_sys_offmode_pol(int);
extern void omap_pm_sys_clkreq_pol(int);
extern void omap_pm_auto_off(int);
extern void omap_pm_auto_ret(int);

u8 get_omap3_evm_rev(void)
{
	return omap3_evm_version;
}
EXPORT_SYMBOL(get_omap3_evm_rev);

static void __init omap3_evm_get_revision(void)
{
	void __iomem *ioaddr;
	unsigned int smsc_id;

	/* Ethernet PHY ID is stored at ID_REV register */
	ioaddr = ioremap_nocache(OMAP3EVM_ETHR_START, SZ_1K);
	if (!ioaddr)
		return;
	smsc_id = readl(ioaddr + OMAP3EVM_ETHR_ID_REV) & 0xFFFF0000;
	iounmap(ioaddr);

	switch (smsc_id) {
	/*SMSC9115 chipset*/
	case 0x01150000:
		omap3_evm_version = OMAP3EVM_BOARD_GEN_1;
		break;
	/*SMSC 9220 chipset*/
	case 0x92200000:
	default:
		omap3_evm_version = OMAP3EVM_BOARD_GEN_2;
	}
}

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct resource omap3evm_smsc911x_resources[] = {
	[0] =	{
		.start	= OMAP3EVM_ETHR_START,
		.end	= (OMAP3EVM_ETHR_START + OMAP3EVM_ETHR_SIZE - 1),
		.flags	= IORESOURCE_MEM,
	},
	[1] =	{
		.start	= OMAP_GPIO_IRQ(OMAP3EVM_ETHR_GPIO_IRQ),
		.end	= OMAP_GPIO_IRQ(OMAP3EVM_ETHR_GPIO_IRQ),
		.flags	= (IORESOURCE_IRQ | IRQF_TRIGGER_LOW),
	},
};

static struct smsc911x_platform_config smsc911x_config = {
	.phy_interface  = PHY_INTERFACE_MODE_MII,
	.irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type       = SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags          = (SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS),
};

static struct platform_device omap3evm_smsc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(omap3evm_smsc911x_resources),
	.resource	= &omap3evm_smsc911x_resources[0],
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static inline void __init omap3evm_init_smsc911x(void)
{
	int eth_cs, eth_rst;
	struct clk *l3ck;
	unsigned int rate;

	if (get_omap3_evm_rev() == OMAP3EVM_BOARD_GEN_1)
		eth_rst = 64;
	else
		eth_rst = 7;

	eth_cs = OMAP3EVM_SMSC911X_CS;

	l3ck = clk_get(NULL, "l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	/* Configure ethernet controller reset gpio */
	if (cpu_is_omap3430()) {
		if (gpio_request(eth_rst, "SMSC911x gpio") < 0) {
			pr_err(KERN_ERR "Failed to request GPIO7 for smsc911x\n");
			return;
		}

		gpio_direction_output(eth_rst, 1);
		/* reset pulse to ethernet controller*/
		usleep_range(150, 220);
		gpio_set_value(eth_rst, 0);
		usleep_range(150, 220);
		gpio_set_value(eth_rst, 1);
		usleep_range(1, 2);
	}

	if (gpio_request(OMAP3EVM_ETHR_GPIO_IRQ, "SMSC911x irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smsc911x IRQ\n",
			OMAP3EVM_ETHR_GPIO_IRQ);
		return;
	}

	gpio_direction_input(OMAP3EVM_ETHR_GPIO_IRQ);
	platform_device_register(&omap3evm_smsc911x_device);
}

#else
static inline void __init omap3evm_init_smsc911x(void) { return; }
#endif

/*
 * OMAP3EVM LCD Panel control signals
 */
#ifdef CONFIG_MACH_FLASHBOARD
#define OMAP3EVM_LCD_PANEL_DISP_MODE    27
#define OMAP3EVM_LCD_PANEL_PWR_CTRL     28
#define OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO 58
#define OMAP3EVM_DVI_PANEL_EN_GPIO      29
#else
#define OMAP3EVM_LCD_PANEL_LR		2
#define OMAP3EVM_LCD_PANEL_UD		3
#define OMAP3EVM_LCD_PANEL_INI		152
#define OMAP3EVM_LCD_PANEL_ENVDD	153
#define OMAP3EVM_LCD_PANEL_QVGA		154
#define OMAP3EVM_LCD_PANEL_RESB		155
#define OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO	210
#define OMAP3EVM_DVI_PANEL_EN_GPIO	199
#endif

static int lcd_enabled;
static int dvi_enabled;

#ifdef CONFIG_MACH_FLASHBOARD
static void __init omap3_evm_display_init(void)
{
	int r;

	r = gpio_request(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, \
			 "lcd_panel_bklight_gpio");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_bklight_gpio\n");
		return;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 0);

	r = gpio_request(OMAP3EVM_LCD_PANEL_DISP_MODE, "lcd_panel_disp_mode");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_disp_mode\n");
		goto err_1;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_DISP_MODE, 0);

	r = gpio_request(OMAP3EVM_LCD_PANEL_PWR_CTRL, "lcd_panel_pwr_ctrl");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_pwr_ctrl\n");
		goto err_2;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_PWR_CTRL, 1);

	return;

err_2:
	gpio_free(OMAP3EVM_LCD_PANEL_DISP_MODE);
err_1:
	gpio_free(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO);
}

static int omap3_evm_enable_lcd(struct omap_dss_device *dssdev)
{
	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}
	gpio_set_value(OMAP3EVM_LCD_PANEL_PWR_CTRL, 0);

	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2) {
		gpio_set_value(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 1);
		gpio_set_value(OMAP3EVM_LCD_PANEL_DISP_MODE, 1);
	} else {
		gpio_set_value(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 0);
		gpio_set_value(OMAP3EVM_LCD_PANEL_DISP_MODE, 0);
	}

	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 400000);

	lcd_enabled = 1;
	return 0;
}

static void omap3_evm_disable_lcd(struct omap_dss_device *dssdev)
{
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 0);
	gpio_set_value(OMAP3EVM_LCD_PANEL_PWR_CTRL, 1);

	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2) {
		gpio_set_value(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 0);
		gpio_set_value(OMAP3EVM_LCD_PANEL_DISP_MODE, 0);
	} else {
		gpio_set_value(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 1);
		gpio_set_value(OMAP3EVM_LCD_PANEL_DISP_MODE, 1);
	}

	lcd_enabled = 0;
}

#else /* #ifdef CONFIG_MACH_FLASHBOARD */
static void __init omap3_evm_display_init(void)
{
	int r;

	r = gpio_request(OMAP3EVM_LCD_PANEL_RESB, "lcd_panel_resb");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_resb\n");
		return;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_RESB, 1);

	r = gpio_request(OMAP3EVM_LCD_PANEL_INI, "lcd_panel_ini");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_ini\n");
		goto err_1;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_INI, 1);

	r = gpio_request(OMAP3EVM_LCD_PANEL_QVGA, "lcd_panel_qvga");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_qvga\n");
		goto err_2;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_QVGA, 0);

	r = gpio_request(OMAP3EVM_LCD_PANEL_LR, "lcd_panel_lr");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_lr\n");
		goto err_3;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_LR, 1);

	r = gpio_request(OMAP3EVM_LCD_PANEL_UD, "lcd_panel_ud");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_ud\n");
		goto err_4;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_UD, 1);

	r = gpio_request(OMAP3EVM_LCD_PANEL_ENVDD, "lcd_panel_envdd");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_envdd\n");
		goto err_5;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_ENVDD, 0);

	return;

err_5:
	gpio_free(OMAP3EVM_LCD_PANEL_UD);
err_4:
	gpio_free(OMAP3EVM_LCD_PANEL_LR);
err_3:
	gpio_free(OMAP3EVM_LCD_PANEL_QVGA);
err_2:
	gpio_free(OMAP3EVM_LCD_PANEL_INI);
err_1:
	gpio_free(OMAP3EVM_LCD_PANEL_RESB);

}

static int omap3_evm_enable_lcd(struct omap_dss_device *dssdev)
{
	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}
	gpio_set_value(OMAP3EVM_LCD_PANEL_ENVDD, 0);

	if (cpu_is_omap3630()) {
		omap_mux_set_gpio(OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT, 2);
		omap_mux_set_gpio(OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT, 3);
		gpio_direction_output(OMAP3EVM_LCD_PANEL_LR, 1);
		gpio_direction_output(OMAP3EVM_LCD_PANEL_UD, 1);
	}
	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2)
		gpio_set_value_cansleep(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 0);
	else
		gpio_set_value_cansleep(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 1);

	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 400000);

	lcd_enabled = 1;
	return 0;
}

static void omap3_evm_disable_lcd(struct omap_dss_device *dssdev)
{
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 0);
	gpio_set_value(OMAP3EVM_LCD_PANEL_ENVDD, 1);

	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2)
		gpio_set_value_cansleep(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 1);
	else
		gpio_set_value_cansleep(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 0);

	lcd_enabled = 0;
}

static int omap3evm_set_bl_intensity(struct omap_dss_device *dssdev, int level)
{
	unsigned char c;

	if (level > dssdev->max_backlight_level)
		level = dssdev->max_backlight_level;

	c = ((125 * (100 - level)) / 100);
	c += get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2 ? 1 : 2;

/*
 * PWMA register offsets (TWL4030_MODULE_PWMA)
 */
#define TWL_LED_PWMON	0x0
	twl_i2c_write_u8(TWL4030_MODULE_PWMA, c, TWL_LED_PWMON);

	return 0;
}
#endif /* #ifdef CONFIG_MACH_FLASHBOARD */

static struct omap_dss_device omap3_evm_lcd_device = {
	.name			= "lcd",
#ifdef CONFIG_MACH_FLASHBOARD
	.driver_name		= "samsung_lms_panel",
#else
	.driver_name		= "sharp_ls_panel",
#endif
	.type			= OMAP_DISPLAY_TYPE_DPI,
#ifdef CONFIG_MACH_FLASHBOARD
	.phy.dpi.data_lines	= 24,
#else
	.phy.dpi.data_lines	= 18,
#endif
	.max_backlight_level	= 100,
	.platform_enable	= omap3_evm_enable_lcd,
	.platform_disable	= omap3_evm_disable_lcd,
#ifndef CONFIG_MACH_FLASHBOARD
	.set_backlight		= omap3evm_set_bl_intensity,
#endif
};

static int omap3_evm_enable_tv(struct omap_dss_device *dssdev)
{
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 400000);
	return 0;
}

static void omap3_evm_disable_tv(struct omap_dss_device *dssdev)
{
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 0);
}

static struct omap_dss_device omap3_evm_tv_device = {
	.name			= "tv",
	.driver_name		= "venc",
	.type			= OMAP_DISPLAY_TYPE_VENC,
#if defined(CONFIG_OMAP2_VENC_OUT_TYPE_SVIDEO)
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
#elif defined(CONFIG_OMAP2_VENC_OUT_TYPE_COMPOSITE)
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_COMPOSITE,
#endif
	.platform_enable	= omap3_evm_enable_tv,
	.platform_disable	= omap3_evm_disable_tv,
};

static int omap3_evm_enable_dvi(struct omap_dss_device *dssdev)
{
	if (lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}

	gpio_set_value_cansleep(OMAP3EVM_DVI_PANEL_EN_GPIO, 1);
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 400000);

	dvi_enabled = 1;
	return 0;
}

static void omap3_evm_disable_dvi(struct omap_dss_device *dssdev)
{
	omap_pm_set_min_bus_tput(&dssdev->dev, OCP_INITIATOR_AGENT, 0);
	gpio_set_value_cansleep(OMAP3EVM_DVI_PANEL_EN_GPIO, 0);

	dvi_enabled = 0;
}

static struct omap_dss_device omap3_evm_dvi_device = {
	.name			= "dvi",
	.driver_name		= "generic_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.platform_enable	= omap3_evm_enable_dvi,
	.platform_disable	= omap3_evm_disable_dvi,
};

static struct omap_dss_device *omap3_evm_dss_devices[] = {
	&omap3_evm_lcd_device,
	&omap3_evm_tv_device,
	&omap3_evm_dvi_device,
};

static struct omap_dss_board_info omap3_evm_dss_data = {
	.num_devices	= ARRAY_SIZE(omap3_evm_dss_devices),
	.devices	= omap3_evm_dss_devices,
	.default_device	= &omap3_evm_lcd_device,
};

static struct platform_device omap3_evm_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data = &omap3_evm_dss_data,
	},
};

#ifndef CONFIG_MACH_FLASHBOARD
static struct regulator_consumer_supply omap3evm_vaux3_supply = {
	.supply         = "cam_2v8",
};

/* VAUX3 for CAM_2V8 */
static struct regulator_init_data omap3evm_vaux3 = {
	.constraints = {
		.min_uV                 = 2800000,
		.max_uV                 = 2800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &omap3evm_vaux3_supply,
};
#endif

static struct regulator_consumer_supply omap3evm_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply omap3evm_vsim_supply = {
	.supply			= "vmmc_aux",
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data omap3evm_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3evm_vmmc1_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data omap3evm_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3evm_vsim_supply,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
#ifdef CONFIG_MACH_FLASHBOARD
		.gpio_wp	= 126,
#else
		.gpio_wp	= 63,
#endif
	},
#ifdef CONFIG_WL12XX_PLATFORM_DATA
	{
	.name           = "wl1271",
	.mmc            = 2,
	.caps           = MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
	.gpio_wp        = -EINVAL,
	.gpio_cd        = -EINVAL,
	.nonremovable   = true,
	},
#endif
	{}	/* Terminator */
};

#ifdef CONFIG_MACH_FLASHBOARD
static struct gpio_keys_button gpio_buttons[] = {
	{
		.code                   = KEY_ENTER,
		.gpio                   = 18,
		.active_low             = true,
		.desc                   = "enter",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_BACK,
		.gpio                   = 19,
		.active_low             = true,
		.desc                   = "back",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_LEFT,
		.gpio                   = 20,
		.active_low             = true,
		.desc                   = "left",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
	{
		.code                   = KEY_RIGHT,
		.gpio                   = 21,
		.active_low             = true,
		.desc                   = "right",
		.type                   = EV_KEY,
		.wakeup                 = 1,
	},
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons        = gpio_buttons,
	.nbuttons       = ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
		.platform_data  = &gpio_key_info,
	},
};
#endif /* #ifdef CONFIG_MACH_FLASHBOARD */

static struct gpio_led gpio_leds[] = {
	{
		.name			= "omap3evm::ledb",
		/* normally not visible (board underside) */
		.default_trigger	= "default-on",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};


static int omap3evm_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	omap_mux_init_gpio(63, OMAP_PIN_INPUT);
	mmc[0].gpio_cd = gpio + 0;
	omap2_hsmmc_init(mmc);

	/* link regulators to MMC adapters */
	omap3evm_vmmc1_supply.dev = mmc[0].dev;
	omap3evm_vsim_supply.dev = mmc[0].dev;

	/*
	 * Most GPIOs are for USB OTG.  Some are mostly sent to
	 * the P2 connector; notably LEDA for the LCD backlight.
	 */

	/* TWL4030_GPIO_MAX + 0 == ledA, LCD Backlight control */
	gpio_request(gpio + TWL4030_GPIO_MAX, "EN_LCD_BKL");
	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2)
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 1);
	else
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 0);

	/* gpio + 7 == DVI Enable */
	gpio_request(gpio + 7, "EN_DVI");
	gpio_direction_output(gpio + 7, 0);

	/* TWL4030_GPIO_MAX + 1 == ledB (out, active low LED) */
	gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1;

	platform_device_register(&leds_gpio);

	return 0;
}

static struct twl4030_gpio_platform_data omap3evm_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.setup		= omap3evm_twl_gpio_setup,
};

static struct twl4030_usb_data omap3evm_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};
/**
 * Macro to configure resources
 */
#define TWL4030_RESCONFIG(res,grp,typ1,typ2,state)	\
	{						\
		.resource	= res,			\
		.devgroup	= grp,			\
		.type		= typ1,			\
		.type2		= typ2,			\
		.remap_sleep	= state			\
	}

static struct twl4030_resconfig  __initdata board_twl4030_rconfig[] = {
	TWL4030_RESCONFIG(RES_VPLL1, DEV_GRP_P1, 3, 1, RES_STATE_OFF),		/* ? */
	TWL4030_RESCONFIG(RES_VINTANA1, DEV_GRP_ALL, 1, 2, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_VINTANA2, DEV_GRP_ALL, 0, 2, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_VINTDIG, DEV_GRP_ALL, 1, 2, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_VIO, DEV_GRP_ALL, 2, 2, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_VDD1, DEV_GRP_P1, 4, 1, RES_STATE_OFF),		/* ? */
	TWL4030_RESCONFIG(RES_VDD2, DEV_GRP_P1, 3, 1, RES_STATE_OFF),		/* ? */
	TWL4030_RESCONFIG(RES_REGEN, DEV_GRP_ALL, 2, 1, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_NRES_PWRON, DEV_GRP_ALL, 0, 1, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_CLKEN, DEV_GRP_ALL, 3, 2, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_SYSEN, DEV_GRP_ALL, 6, 1, RES_STATE_SLEEP),
	TWL4030_RESCONFIG(RES_HFCLKOUT, DEV_GRP_P3, 0, 2, RES_STATE_SLEEP),	/* ? */
	TWL4030_RESCONFIG(0, 0, 0, 0, 0),
};

/**
 * Optimized 'Active to Sleep' sequence
 */
static struct twl4030_ins omap3evm_sleep_seq[] __initdata = {
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_HFCLKOUT, RES_STATE_SLEEP), 20},
	{ MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_SLEEP), 2 },
	{ MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_SLEEP), 2 },
};

static struct twl4030_script omap3evm_sleep_script __initdata = {
	.script	= omap3evm_sleep_seq,
	.size	= ARRAY_SIZE(omap3evm_sleep_seq),
	.flags	= TWL4030_SLEEP_SCRIPT,
};

/**
 * Optimized 'Sleep to Active (P12)' sequence
 */
static struct twl4030_ins omap3evm_wake_p12_seq[] __initdata = {
	{ MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_ACTIVE), 2 }
};

static struct twl4030_script omap3evm_wake_p12_script __initdata = {
	.script = omap3evm_wake_p12_seq,
	.size   = ARRAY_SIZE(omap3evm_wake_p12_seq),
	.flags  = TWL4030_WAKEUP12_SCRIPT,
};

/**
 * Optimized 'Sleep to Active' (P3) sequence
 */
static struct twl4030_ins omap3evm_wake_p3_seq[] __initdata = {
	{ MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_ACTIVE), 2 }
};

static struct twl4030_script omap3evm_wake_p3_script __initdata = {
	.script = omap3evm_wake_p3_seq,
	.size   = ARRAY_SIZE(omap3evm_wake_p3_seq),
	.flags  = TWL4030_WAKEUP3_SCRIPT,
};

/**
 * Optimized warm reset sequence (for less power surge)
 */
static struct twl4030_ins omap3evm_wrst_seq[] __initdata = {
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_OFF), 0x2 },
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_MAIN_REF, RES_STATE_WRST), 2 },
	{ MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_WRST), 0x2},
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_VUSB_3V1, RES_STATE_WRST), 0x2 },
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_VPLL1, RES_STATE_WRST), 0x2 },
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_VDD2, RES_STATE_WRST), 0x7 },
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_VDD1, RES_STATE_WRST), 0x25 },
	{ MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_RC, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_WRST), 0x2 },
	{ MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_ACTIVE), 0x2 },
};

static struct twl4030_script omap3evm_wrst_script __initdata = {
	.script = omap3evm_wrst_seq,
	.size   = ARRAY_SIZE(omap3evm_wrst_seq),
	.flags  = TWL4030_WRST_SCRIPT,
};

static struct twl4030_script __initdata *board_twl4030_scripts[] = {
	&omap3evm_wake_p12_script,
	&omap3evm_wake_p3_script,
	&omap3evm_sleep_script,
	&omap3evm_wrst_script
};

static struct twl4030_power_data __initdata omap3evm_script_data = {
	.scripts		= board_twl4030_scripts,
	.num			= ARRAY_SIZE(board_twl4030_scripts),
	.resource_config	= board_twl4030_rconfig,
};

#ifndef CONFIG_MACH_FLASHBOARD
static uint32_t board_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_DOWN),
	KEY(0, 2, KEY_ENTER),
	KEY(0, 3, KEY_M),

	KEY(1, 0, KEY_RIGHT),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_I),
	KEY(1, 3, KEY_N),

	KEY(2, 0, KEY_A),
	KEY(2, 1, KEY_E),
	KEY(2, 2, KEY_J),
	KEY(2, 3, KEY_O),

	KEY(3, 0, KEY_B),
	KEY(3, 1, KEY_F),
	KEY(3, 2, KEY_K),
	KEY(3, 3, KEY_P)
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data omap3evm_kp_data = {
	.keymap_data	= &board_map_data,
	.rows		= 4,
	.cols		= 4,
	.rep		= 1,
};
#endif

static struct twl4030_madc_platform_data omap3evm_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_codec_audio_data omap3evm_audio_data = {
	.audio_mclk = 26000000,
	.digimic_delay = 1,
	.ramp_delay_value = 1,
	.offset_cncl_path = 1,
	.check_defaults = false,
	.reset_registers = false,
	.reset_registers = false,
};

static struct twl4030_codec_data omap3evm_codec_data = {
	.audio_mclk = 26000000,
	.audio = &omap3evm_audio_data,
};

static struct regulator_consumer_supply omap3_evm_vdda_dac_supply = {
	.supply		= "vdda_dac",
	.dev		= &omap3_evm_dss_device.dev,
};

/* VDAC for DSS driving S-Video */
static struct regulator_init_data omap3_evm_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3_evm_vdda_dac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_consumer_supply omap3_evm_vpll2_supply =
	REGULATOR_SUPPLY("vdds_dsi", "omapdss");

static struct regulator_init_data omap3_evm_vpll2 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3_evm_vpll2_supply,
};

/* VAUX2 for USB */
static struct regulator_consumer_supply omap3evm_vaux2_supplies = {
	.supply		= "hsusb1",
};

static struct regulator_init_data omap3evm_vaux2 = {
	.constraints = {
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies		= 1,
	.consumer_supplies		= &omap3evm_vaux2_supplies,
};

/* VUSB1V8 */
static struct regulator_consumer_supply omap3_evm_vusb_supply = {
	.supply		= "hsusb1",
};

static struct regulator_init_data omap3_evm_vusb = {
	.constraints = {
		.name			= "VUSB1V8",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3_evm_vusb_supply,
};

/* VIO_1V8 is required for diff modules: ads7846 on SPI, pull-ups, etc... */
static struct regulator_consumer_supply omap3_evm_vio_supply[] = {
	REGULATOR_SUPPLY("vcc", "spi1.0"),
	REGULATOR_SUPPLY("vio_1v8", NULL),
};

static struct regulator_init_data omap3_evm_vio = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(omap3_evm_vio_supply),
	.consumer_supplies      = omap3_evm_vio_supply,
};

#ifdef CONFIG_WL12XX_PLATFORM_DATA

#ifdef CONFIG_MACH_FLASHBOARD
#define OMAP3EVM_WLAN_PMENA_GPIO       (139)
#define OMAP3EVM_WLAN_IRQ_GPIO         (138)
#else
#define OMAP3EVM_WLAN_PMENA_GPIO       (150)
#define OMAP3EVM_WLAN_IRQ_GPIO         (149)
#endif

static struct regulator_consumer_supply omap3evm_vmmc2_supply =
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.1");

/* VMMC2 for driving the WL12xx module */
static struct regulator_init_data omap3evm_vmmc2 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies = &omap3evm_vmmc2_supply,
};

static struct fixed_voltage_config omap3evm_vwlan = {
	.supply_name            = "vwl1271",
	.microvolts             = 1800000, /* 1.80V */
	.gpio                   = OMAP3EVM_WLAN_PMENA_GPIO,
	.startup_delay          = 70000, /* 70ms */
	.enable_high            = 1,
	.enabled_at_boot        = 0,
	.init_data              = &omap3evm_vmmc2,
};

static struct platform_device omap3evm_wlan_regulator = {
	.name           = "reg-fixed-voltage",
	.id             = 1,
	.dev = {
		.platform_data  = &omap3evm_vwlan,
	},
};

struct wl12xx_platform_data omap3evm_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(OMAP3EVM_WLAN_IRQ_GPIO),
	.board_ref_clock = WL12XX_REFCLOCK_38, /* 38.4 MHz */
};
#endif

static struct twl4030_platform_data omap3evm_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
#ifndef CONFIG_MACH_FLASHBOARD
	.keypad		= &omap3evm_kp_data,
#endif
	.madc		= &omap3evm_madc_data,
	.usb		= &omap3evm_usb_data,
	.gpio		= &omap3evm_gpio_data,
	.codec		= &omap3evm_codec_data,
	.vdac		= &omap3_evm_vdac,
	.vpll2		= &omap3_evm_vpll2,
	.vaux2          = &omap3evm_vaux2,
	.vio		= &omap3_evm_vio,
#ifndef CONFIG_MACH_FLASHBOARD
	.vaux3		= &omap3evm_vaux3,
#endif
	.power		= &omap3evm_script_data,
};

#ifdef CONFIG_MACH_FLASHBOARD

struct lis331dlh_platform_data  lis331dlh_omap3evm_data  = {

	.min_interval = 1,
	.poll_interval = 200,

	.g_range = LIS331DLH_G_8G,

	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,

	.negate_x = 0,  
	.negate_y = 0,  
	.negate_z = 0,  

};

int l3g4200dh_setup_resources (void  *unused)
{
        int ret;
        int irq;

        ret = gpio_request(L3G4200DH_IRQ_GPIO , "l3g4200dh irq");
        if (ret < 0)
                goto fail;

        ret = gpio_direction_input(L3G4200DH_IRQ_GPIO);
        if (ret < 0)
                goto fail_irq;

        irq = gpio_to_irq(L3G4200DH_IRQ_GPIO);
        if (irq < 0)
                goto fail_irq;

        return 0;

fail_irq:
        gpio_free(L3G4200DH_IRQ_GPIO);
fail:
        printk(KERN_ERR "l3g4200dh initialisation failed\n");
        return -1;
}

void l3g4200dh_release_resources (void  *unused)
{
        gpio_free(L3G4200DH_IRQ_GPIO);
}

struct l3g4200dh_platform_data  l3g4200dh_pdata = {

        .setup_resources = l3g4200dh_setup_resources,
        .release_resources = l3g4200dh_release_resources,
};

#endif

static struct i2c_board_info __initdata omap3evm_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &omap3evm_twldata,
	},
#ifdef CONFIG_MACH_FLASHBOARD
        {
                I2C_BOARD_INFO("lis331dlh", 0x18),
                .platform_data = &lis331dlh_omap3evm_data,
        },
	{
		I2C_BOARD_INFO("l3g4200dh_i2c", 0x68),
		.platform_data = &l3g4200dh_pdata,
		.irq = OMAP_GPIO_IRQ(L3G4200DH_IRQ_GPIO),
	},

#endif
};

static int __init omap3_evm_i2c_init(void)
{
	/*
	 * REVISIT: These entries can be set in omap3evm_twl_data
	 * after a merge with MFD tree
	 */
	omap3evm_twldata.vmmc1 = &omap3evm_vmmc1;
	omap3evm_twldata.vsim = &omap3evm_vsim;

	omap_register_i2c_bus(1, 2600, omap3evm_i2c_boardinfo,
			ARRAY_SIZE(omap3evm_i2c_boardinfo));
	/* Bus 2 is used for Camera/Sensor interface */
	omap_register_i2c_bus(2, 400, NULL, 0);

	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

static void ads7846_dev_init(void)
{
	if (gpio_request(OMAP3_EVM_TS_GPIO, "ADS7846 pendown") < 0)
		printk(KERN_ERR "can't get ads7846 pen down GPIO\n");

	gpio_direction_input(OMAP3_EVM_TS_GPIO);
	gpio_set_debounce(OMAP3_EVM_TS_GPIO, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(OMAP3_EVM_TS_GPIO);
}

static struct ads7846_platform_data ads7846_config = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
	.x_plate_ohms		= 180,
	.pressure_max		= 255,
	.debounce_max		= 10,
	.debounce_tol		= 3,
	.debounce_rep		= 1,
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.settle_delay_usecs	= 150,
	.wakeup			= true,
};

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info omap3evm_spi_board_info[] = {
	[0] = {
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &ads7846_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(OMAP3_EVM_TS_GPIO),
		.platform_data		= &ads7846_config,
	},
};

static struct omap_board_config_kernel omap3_evm_config[] __initdata = {
};

static void __init omap3_evm_init_irq(void)
{
	omap_board_config = omap3_evm_config;
	omap_board_config_size = ARRAY_SIZE(omap3_evm_config);
	omap2_init_common_infrastructure();
	if (cpu_is_omap3630())
		omap2_init_common_devices(h8kds0un0mer4em_sdrc_params, NULL);
	else
		omap2_init_common_devices(mt46h32m32lf6_sdrc_params, NULL);

	omap_init_irq();
	gpmc_init();
}

#ifdef CONFIG_SND_SOC_WL1271BT
/* WL1271 Audio */
static struct platform_device wl1271bt_audio_device = {
	.name		= "wl1271bt",
	.id		= -1,
};

static struct platform_device wl1271bt_codec_device = {
	.name		= "wl1271bt-dummy-codec",
	.id		= -1,
};

static void wl1271bt_clk_setup(void)
{
	u16 reg;
	u32 val;

	/*
	 * Set DEVCONF0 register to connect
	 * MCBSP1_CLKR -> MCBSP1_CLKX & MCBSP1_FSR -> MCBSP1_FSX
	 */
	reg = OMAP2_CONTROL_DEVCONF0;
	val = omap_ctrl_readl(reg);
	val = val | 0x18;
	omap_ctrl_writel(val, reg);
}
#endif

static struct platform_device *omap3_evm_devices[] __initdata = {
	&omap3_evm_dss_device,
	&usb_mass_storage_device,
#ifdef CONFIG_SND_SOC_WL1271BT
	&wl1271bt_audio_device,
	&wl1271bt_codec_device,
#endif
#ifdef CONFIG_MACH_FLASHBOARD
	&keys_gpio,
#endif
};

#ifndef CONFIG_MACH_FLASHBOARD
static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	/* PHY reset GPIO will be runtime programmed based on EVM version */
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

/*
 * OneNAND
 */
static struct mtd_partition omap3_evm_onenand_partitions[] = {
	{
		.name		= "X-Loader-OneNAND",
		.offset		= 0,
		.size		= 4 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE  /* force read-only */
	},
	{
		.name		= "U-Boot-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 2 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE  /* force read-only */
	},
	{
		.name		= "U-Boot Environment-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 1 * (64 * 2048),
	},
	{
		.name		= "Kernel-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 16 * (64 * 2048),
	},
	{
		.name		= "File System-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
};
#endif

/*
 * NAND
 */
static struct mtd_partition omap3_evm_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader-NAND",
		.offset		= 0,
		.size		= 4 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 10 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "Boot Env-NAND",

		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x1c0000 */
		.size		= 6 * (64 * 2048),
	},
	{
		.name		= "Kernel-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 40 * (64 * 2048),
	},
	{
		.name		= "File System - NAND",
		.size		= MTDPART_SIZ_FULL,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x780000 */
	},
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux omap35x_board_mux[] __initdata = {
	OMAP3_MUX(SYS_NIRQ, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_INPUT_PULLUP | OMAP_PIN_OFF_OUTPUT_LOW |
				OMAP_PIN_OFF_WAKEUPENABLE),
	OMAP3_MUX(MCSPI1_CS1, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_INPUT_PULLUP | OMAP_PIN_OFF_OUTPUT_LOW |
				OMAP_PIN_OFF_WAKEUPENABLE),
	OMAP3_MUX(SYS_BOOT5, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_NONE),
	OMAP3_MUX(GPMC_WAIT2, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_NONE),
	OMAP3_MUX(MCBSP1_FSR, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_NONE),
#ifdef CONFIG_WL12XX_PLATFORM_DATA
	/* WLAN IRQ - GPIO 149 */
	OMAP3_MUX(UART1_RTS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),

	/* WLAN POWER ENABLE - GPIO 150 */
	OMAP3_MUX(UART1_CTS, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* MMC2 SDIO pin muxes for WL12xx */
	OMAP3_MUX(SDMMC2_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
#endif
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

static struct omap_board_mux omap36x_board_mux[] __initdata = {
#ifndef CONFIG_MACH_FLASHBOARD
	OMAP3_MUX(SYS_NIRQ, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP |
			OMAP_PIN_OFF_INPUT_PULLUP | OMAP_PIN_OFF_OUTPUT_LOW |
			OMAP_PIN_OFF_WAKEUPENABLE),
	OMAP3_MUX(MCSPI1_CS1, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP |
			OMAP_PIN_OFF_INPUT_PULLUP | OMAP_PIN_OFF_OUTPUT_LOW |
			OMAP_PIN_OFF_WAKEUPENABLE),
#else /* #ifndef CONFIG_MACH_FLASHBOARD */
	OMAP3_MUX(MCSPI1_CS3, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP |
			OMAP_PIN_OFF_INPUT_PULLUP | OMAP_PIN_OFF_OUTPUT_LOW |
			OMAP_PIN_OFF_WAKEUPENABLE),
#endif
	OMAP3_MUX(MCBSP1_FSR, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_NONE),
	/* AM/DM37x EVM: DSS data bus muxed with sys_boot */
	OMAP3_MUX(DSS_DATA18, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(DSS_DATA19, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(DSS_DATA20, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(DSS_DATA21, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(DSS_DATA22, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(DSS_DATA23, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT0, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT1, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT3, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT4, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT5, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT6, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
#ifdef CONFIG_WL12XX_PLATFORM_DATA
#ifdef CONFIG_MACH_FLASHBOARD
        /* WLAN IRQ - GPIO 138 */
        OMAP3_MUX(SDMMC2_DAT6, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
        /* WLAN POWER ENABLE - GPIO 139 */
        OMAP3_MUX(SDMMC2_DAT7, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
#else
	/* WLAN IRQ - GPIO 149 */
	OMAP3_MUX(UART1_RTS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	/* WLAN POWER ENABLE - GPIO 150 */
	OMAP3_MUX(UART1_CTS, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
#endif
	/* MMC2 SDIO pin muxes for WL12xx */
	OMAP3_MUX(SDMMC2_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
#endif
#ifdef CONFIG_MACH_FLASHBOARD
	/* GPIO LEDS */
	OMAP3_MUX(DSS_DATA0, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* GPIO-70 */
	OMAP3_MUX(DSS_DATA1, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* GPIO-71 */
	OMAP3_MUX(DSS_DATA2, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* GPIO-72 */
	OMAP3_MUX(DSS_DATA3, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* GPIO-73 */
	OMAP3_MUX(DSS_DATA4, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* GPIO-74 */
	OMAP3_MUX(DSS_DATA5, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* GPIO-75 */
	/* GPIO Keys */
	OMAP3_MUX(ETK_D4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP), /* GPIO-18 */
	OMAP3_MUX(ETK_D5, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP), /* GPIO-19 */
	OMAP3_MUX(ETK_D6, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP), /* GPIO-20 */
	OMAP3_MUX(ETK_D7, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP), /* GPIO-21 */
	/* LCD power and display on/off */
	OMAP3_MUX(ETK_D13, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* GPIO-27 */
	OMAP3_MUX(ETK_D14, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* GPIO-28 */
	/* Backlight power */
	OMAP3_MUX(GPMC_NCS7, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* GPIO-58 */
	/* Bluetooth power */
	OMAP3_MUX(SDMMC2_DAT4, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT), /* GPIO-136 */
	/*accel*/
	OMAP3_MUX(ETK_D12, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP), /* GPIO-26 */
	OMAP3_MUX(GPMC_A9, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP), /* GPIO-42 */
	/*gyro */
	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT), /* GPIO-55 */
#endif
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define omap35x_board_mux	NULL
#define omap36x_board_mux	NULL
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

/**
 * Board specific initialization of PM components
 */
static void __init omap3_evm_pm_init(void)
{
	/* Use sys_offmode signal */
	omap_pm_sys_offmode_select(1);

	/* sys_clkreq - active high */
	omap_pm_sys_clkreq_pol(1);

	/* sys_offmode - active low */
	omap_pm_sys_offmode_pol(0);

	/* Automatically send OFF command */
	omap_pm_auto_off(1);

	/* Automatically send RET command */
	omap_pm_auto_ret(1);
}

static void __init omap3_evm_init(void)
{
	omap3_evm_get_revision();

	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2)
		omap3evm_twldata.vaux2 = &omap3evm_vaux2;
	else
		omap3evm_twldata.vusb = &omap3_evm_vusb;

	if (cpu_is_omap3630())
		omap3_mux_init(omap36x_board_mux, OMAP_PACKAGE_CBB);
	else
		omap3_mux_init(omap35x_board_mux, OMAP_PACKAGE_CBB);

	omap3_evm_i2c_init();

	platform_add_devices(omap3_evm_devices, ARRAY_SIZE(omap3_evm_devices));

	spi_register_board_info(omap3evm_spi_board_info,
				ARRAY_SIZE(omap3evm_spi_board_info));

	omap_serial_init();

	/* OMAP3EVM uses ISP1504 phy and so register nop transceiver */
	usb_nop_xceiv_register(0);

#ifndef CONFIG_MACH_FLASHBOARD
	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2) {
		/* enable EHCI VBUS using GPIO22 */
		omap_mux_init_gpio(22, OMAP_PIN_INPUT_PULLUP);
		gpio_request(OMAP3_EVM_EHCI_VBUS, "enable EHCI VBUS");
		gpio_direction_output(OMAP3_EVM_EHCI_VBUS, 0);
		gpio_set_value(OMAP3_EVM_EHCI_VBUS, 1);

		/* Select EHCI port on main board */
		omap_mux_init_gpio(61, OMAP_PIN_INPUT_PULLUP);
		gpio_request(OMAP3_EVM_EHCI_SELECT, "select EHCI port");
		gpio_direction_output(OMAP3_EVM_EHCI_SELECT, 0);
		gpio_set_value(OMAP3_EVM_EHCI_SELECT, 0);

		/* setup EHCI phy reset config */
		omap_mux_init_gpio(21, OMAP_PIN_INPUT_PULLUP);
		ehci_pdata.reset_gpio_port[1] = 21;

		/* EVM REV >= E can supply 500mA with EXTVBUS programming */
		musb_board_data.power = 500;
		musb_board_data.extvbus = 1;
	} else {
		/* setup EHCI phy reset on MDC */
		omap_mux_init_gpio(135, OMAP_PIN_OUTPUT);
		ehci_pdata.reset_gpio_port[1] = 135;
	}
#else
	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2) {
		/* EVM REV >= E can supply 500mA with EXTVBUS programming */
		musb_board_data.power = 500;
		musb_board_data.extvbus = 1;
	}
#endif
	usb_musb_init(&musb_board_data);
#ifndef CONFIG_MACH_FLASHBOARD
	usb_ehci_init(&ehci_pdata);
#endif
	ads7846_dev_init();
	omap3evm_init_smsc911x();
	omap3_evm_display_init();

#ifdef CONFIG_USB_ANDROID
	omap3evm_android_gadget_init();
#endif

	omap3_evm_pm_init();

	/* NAND */
	board_nand_init(omap3_evm_nand_partitions,
			ARRAY_SIZE(omap3_evm_nand_partitions),
			0, NAND_BUSWIDTH_16);
#ifndef CONFIG_MACH_FLASHBOARD
	board_onenand_init(omap3_evm_onenand_partitions,
			ARRAY_SIZE(omap3_evm_onenand_partitions), 0);
#endif
#ifdef CONFIG_WL12XX_PLATFORM_DATA
	/* WL12xx WLAN Init */
	if (wl12xx_set_platform_data(&omap3evm_wlan_data))
		pr_err("error setting wl12xx data\n");
	platform_device_register(&omap3evm_wlan_regulator);
#endif
#ifdef CONFIG_SND_SOC_WL1271BT
	wl1271bt_clk_setup();
#endif
}

MACHINE_START(OMAP3EVM, "OMAP3 EVM")
	/* Maintainer: Syed Mohammed Khasim - Texas Instruments */
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= omap3_evm_init_irq,
	.init_machine	= omap3_evm_init,
	.timer		= &omap_timer,
MACHINE_END
