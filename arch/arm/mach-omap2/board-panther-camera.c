/*
 * BeagleXM: Driver for Leopard Module Board
 *
 * Copyright (C) 2011 Texas Instruments Inc
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <mach/gpio.h>

#include <media/mt9v113.h>
#ifdef CONFIG_VIDEO_OV5640
#include <media/ov5640.h>
#endif
#ifdef CONFIG_VIDEO_MT9T111
#include <media/mt9t111.h>
#endif

#include <../drivers/media/video/isp/isp.h>

#include "devices.h"

#define CAM_USE_XCLKA			0
#define LEOPARD_RESET_GPIO		98
/* hood add for debug */
#define POWER_DOWN_GPIO 167
#define VDD_CAM_GPIO	27	
#define CAM_EN_GPIO	126
/* hood add end */

static struct regulator *panther_cam_digital;
static struct regulator *panther_cam_io;

#ifdef CONFIG_VIDEO_MT9V113
static int panther_mt9v113_s_power(struct v4l2_subdev *subdev, int on)
{
	struct isp_device *isp = v4l2_dev_to_isp_device(subdev->v4l2_dev);

	if (!panther_cam_digital || !panther_cam_io) {
		dev_err(isp->dev, "No regulator available\n");
		return -ENODEV;
	}
	if (on) {
		/* Check Voltage-Levels */
		if (regulator_get_voltage(panther_cam_digital) != 1800000)
			regulator_set_voltage(panther_cam_digital, 1800000, 1800000);
		if (regulator_get_voltage(panther_cam_io) != 1800000)
			regulator_set_voltage(panther_cam_io, 1800000, 1800000);
		/*
		 * Power Up Sequence
		 */
		/* Set RESET_BAR to 0 */
		gpio_set_value(LEOPARD_RESET_GPIO, 0);
		/* Turn on VDD */
		regulator_enable(panther_cam_digital);
		mdelay(1);
		regulator_enable(panther_cam_io);

		mdelay(50);
		/* Enable EXTCLK */
		if (isp->platform_cb.set_xclk)
			isp->platform_cb.set_xclk(isp, 24000000, CAM_USE_XCLKA);
		/*
		 * Wait at least 70 CLK cycles (w/EXTCLK = 24MHz):
		 * ((1000000 * 70) / 24000000) = aprox 3 us.
		 */
		udelay(3);
		/* Set RESET_BAR to 1 */
		gpio_set_value(LEOPARD_RESET_GPIO, 1);
		/*
		 * Wait at least 100 CLK cycles (w/EXTCLK = 24MHz):
		 * ((1000000 * 100) / 24000000) = aprox 5 us.
		 */
		udelay(5);
	} else {
		/*
		 * Power Down Sequence
		 */
		if (regulator_is_enabled(panther_cam_digital))
			regulator_disable(panther_cam_digital);
		if (regulator_is_enabled(panther_cam_io))
			regulator_disable(panther_cam_io);

		if (isp->platform_cb.set_xclk)
			isp->platform_cb.set_xclk(isp, 0, CAM_USE_XCLKA);
	}

	return 0;
}
#endif
#ifdef CONFIG_VIDEO_OV5640
static int panther_ov5640_s_power(struct v4l2_subdev *subdev, int on)
{
	struct isp_device *isp = v4l2_dev_to_isp_device(subdev->v4l2_dev);


	if (on) {
		/*
		 * Power Up Sequence
		 */
		printk("cam led on.............\n");
		gpio_direction_output(CAM_EN_GPIO, 1);
		/* Set POWER_DOWN to 1 */
		gpio_set_value(POWER_DOWN_GPIO, 1);
		/* Set RESET_BAR to 0 */
		gpio_set_value(LEOPARD_RESET_GPIO, 0);
		/* Turn on VDD */
		gpio_set_value(VDD_CAM_GPIO, 1);

		mdelay(50);
		/* Enable EXTCLK */
		if (isp->platform_cb.set_xclk)
			isp->platform_cb.set_xclk(isp, 24000000, CAM_USE_XCLKA);
		/*
		 * Wait at least 70 CLK cycles (w/EXTCLK = 24MHz):
		 * ((1000000 * 70) / 24000000) = aprox 3 us.
		 */
		udelay(3);
		/* Set RESET_BAR to 1 */
		gpio_set_value(LEOPARD_RESET_GPIO, 1);
		/* Set POWER_DOWN to 0 */
		gpio_set_value(POWER_DOWN_GPIO, 0);
		/*
		 * Wait at least 100 CLK cycles (w/EXTCLK = 24MHz):
		 * ((1000000 * 100) / 24000000) = aprox 5 us.
		 */
		msleep(20);
	} else {
		/*
		 * Power Down Sequence
		 */
		printk("cam led off.............\n");
		gpio_set_value(CAM_EN_GPIO, 0);
		gpio_set_value(VDD_CAM_GPIO, 0);

		mdelay(20);

		if (isp->platform_cb.set_xclk)
			isp->platform_cb.set_xclk(isp, 0, CAM_USE_XCLKA);
	}

	return 0;
}
/* hood modify for debug */
#if 0
static int panther_ov5640_s_power(struct v4l2_subdev *subdev, int on)
{
	struct isp_device *isp = v4l2_dev_to_isp_device(subdev->v4l2_dev);

	if (!panther_cam_digital || !panther_cam_io) {
		dev_err(isp->dev, "No regulator available\n");
		return -ENODEV;
	}
	if (on) {
		/* Check Voltage-Levels */
		/* hood modify for debug */
		if (regulator_get_voltage(panther_cam_digital) != 1500000)
			regulator_set_voltage(panther_cam_digital, 1500000, 1500000);
		if (regulator_get_voltage(panther_cam_io) != 1800000)
			regulator_set_voltage(panther_cam_io, 1800000, 1800000);
		/* Request POWER_DOWN_GPIO and set to output */
		gpio_request(POWER_DOWN_GPIO, "CAM_PWRDN");
		gpio_direction_output(POWER_DOWN_GPIO, true);
		/*
		 * Power Up Sequence
		 */
		/* Set POWER_DOWN to 1 */
		gpio_set_value(POWER_DOWN_GPIO, 1);
		/* Set RESET_BAR to 0 */
		gpio_set_value(LEOPARD_RESET_GPIO, 0);
		/* Turn on VDD */
		regulator_enable(panther_cam_io);
		mdelay(1);
		regulator_enable(panther_cam_digital);

		mdelay(50);
		/* Enable EXTCLK */
		if (isp->platform_cb.set_xclk)
		/* hood modify */
			isp->platform_cb.set_xclk(isp, 24000000, CAM_USE_XCLKA);
		/*
		 * Wait at least 70 CLK cycles (w/EXTCLK = 24MHz):
		 * ((1000000 * 70) / 24000000) = aprox 3 us.
		 */
		udelay(3);
		/* Set RESET_BAR to 1 */
		gpio_set_value(LEOPARD_RESET_GPIO, 1);
		/* Set POWER_DOWN to 0 */
		gpio_set_value(POWER_DOWN_GPIO, 0);
		/*
		 * Wait at least 100 CLK cycles (w/EXTCLK = 24MHz):
		 * ((1000000 * 100) / 24000000) = aprox 5 us.
		 */
		/* hood modify for debug */
//		udelay(5);
		msleep(20);
	} else {
		/*
		 * Power Down Sequence
		 */
		if (regulator_is_enabled(panther_cam_digital))
			regulator_disable(panther_cam_digital);
		if (regulator_is_enabled(panther_cam_io))
			regulator_disable(panther_cam_io);

		if (isp->platform_cb.set_xclk)
			isp->platform_cb.set_xclk(isp, 0, CAM_USE_XCLKA);
	}

	return 0;
}
#endif
#endif

#ifdef CONFIG_VIDEO_MT9T111
static int panther_mt9t111_s_power(struct v4l2_subdev *subdev, int on)
{
	struct isp_device *isp = v4l2_dev_to_isp_device(subdev->v4l2_dev);

	if (!panther_cam_digital || !panther_cam_io) {
		dev_err(isp->dev, "No regulator available\n");
		return -ENODEV;
	}
	if (on) {
		/* Check Voltage-Levels */
		if (regulator_get_voltage(panther_cam_digital) != 1800000)
			regulator_set_voltage(panther_cam_digital, 1800000, 1800000);
		if (regulator_get_voltage(panther_cam_io) != 1800000)
			regulator_set_voltage(panther_cam_io, 1800000, 1800000);
		/*
		 * Power Up Sequence
		 */
		/* Set RESET_BAR to 0 */
		gpio_set_value(LEOPARD_RESET_GPIO, 0);
		/* Turn on VDD */
		regulator_enable(panther_cam_digital);
		mdelay(1);
		regulator_enable(panther_cam_io);

		mdelay(50);
		/* Enable EXTCLK */
		if (isp->platform_cb.set_xclk)
			isp->platform_cb.set_xclk(isp, 24000000, CAM_USE_XCLKA);
		/*
		 * Wait at least 70 CLK cycles (w/EXTCLK = 24MHz):
		 * ((1000000 * 70) / 24000000) = aprox 3 us.
		 */
		udelay(3);
		/* Set RESET_BAR to 1 */
		gpio_set_value(LEOPARD_RESET_GPIO, 1);
		/*
		 * Wait at least 100 CLK cycles (w/EXTCLK = 24MHz):
		 * ((1000000 * 100) / 24000000) = aprox 5 us.
		 */
		udelay(5);
	} else {
		/*
		 * Power Down Sequence
		 */
		if (regulator_is_enabled(panther_cam_digital))
			regulator_disable(panther_cam_digital);
		if (regulator_is_enabled(panther_cam_io))
			regulator_disable(panther_cam_io);

		if (isp->platform_cb.set_xclk)
			isp->platform_cb.set_xclk(isp, 0, CAM_USE_XCLKA);
	}

	return 0;
}
#endif

#ifdef CONFIG_VIDEO_MT9V113
static int panther_mt9v113_configure_interface(struct v4l2_subdev *subdev,
					      u32 pixclk)
{
	struct isp_device *isp = v4l2_dev_to_isp_device(subdev->v4l2_dev);

	if (isp->platform_cb.set_pixel_clock)
		isp->platform_cb.set_pixel_clock(isp, pixclk);

	return 0;
}
#endif
#ifdef CONFIG_VIDEO_OV5640
static int panther_ov5640_configure_interface(struct v4l2_subdev *subdev,
					      u32 pixclk)
{
	struct isp_device *isp = v4l2_dev_to_isp_device(subdev->v4l2_dev);

	if (isp->platform_cb.set_pixel_clock)
		isp->platform_cb.set_pixel_clock(isp, pixclk);

	return 0;
}
#endif
#ifdef CONFIG_VIDEO_MT9T111
static int panther_mt9t111_configure_interface(struct v4l2_subdev *subdev,
					      u32 pixclk)
{
	struct isp_device *isp = v4l2_dev_to_isp_device(subdev->v4l2_dev);

	if (isp->platform_cb.set_pixel_clock)
		isp->platform_cb.set_pixel_clock(isp, pixclk);

	return 0;
}
#endif

#ifdef CONFIG_VIDEO_MT9V113
static struct mt9v113_platform_data panther_mt9v113_platform_data = {
	.s_power		= panther_mt9v113_s_power,
	.configure_interface	= panther_mt9v113_configure_interface,
};
#endif
#ifdef CONFIG_VIDEO_OV5640
static struct ov5640_platform_data panther_ov5640_platform_data = {
	.s_power		= panther_ov5640_s_power,
	.configure_interface	= panther_ov5640_configure_interface,
};
#endif
#ifdef CONFIG_VIDEO_MT9P111
static struct mt9p111_platform_data panther_mt9p111_platform_data = {
	.s_power		= panther_mt9p111_s_power,
	.configure_interface	= panther_mt9p111_configure_interface,
};
#endif
#ifdef CONFIG_VIDEO_MT9T111
static struct mt9t111_platform_data panther_mt9t111_platform_data = {
	.s_power		= panther_mt9t111_s_power,
	.configure_interface	= panther_mt9t111_configure_interface,
};
#endif

#define CAMERA_I2C_BUS_NUM		2

#ifdef CONFIG_VIDEO_MT9V113
static struct i2c_board_info panther_mt9v113_i2c_devices[] = {
	{
		I2C_BOARD_INFO(MT9V113_MODULE_NAME, MT9V113_I2C_ADDR),
		.platform_data = &panther_mt9v113_platform_data,
	},
};
#endif
#ifdef CONFIG_VIDEO_OV5640
static struct i2c_board_info panther_ov5640_i2c_devices[] = {
	{
		I2C_BOARD_INFO(OV5640_MODULE_NAME, OV5640_I2C_ADDR),
		.platform_data = &panther_ov5640_platform_data,
	},
};
#endif
#ifdef CONFIG_VIDEO_MT9T111
static struct i2c_board_info panther_mt9t111_i2c_devices[] = {
	{
		I2C_BOARD_INFO(MT9T111_MODULE_NAME, MT9T111_I2C_ADDR),
		.platform_data = &panther_mt9t111_platform_data,
	},
};
#endif

#ifdef CONFIG_VIDEO_MT9V113
static struct isp_subdev_i2c_board_info panther_mt9v113_primary_subdevs[] = {
	{
		.board_info = &panther_mt9v113_i2c_devices[0],
		.i2c_adapter_id = CAMERA_I2C_BUS_NUM,
	},
	{ NULL, 0 },
};
#endif
#ifdef CONFIG_VIDEO_OV5640
static struct isp_subdev_i2c_board_info panther_ov5640_primary_subdevs[] = {
	{
		.board_info = &panther_ov5640_i2c_devices[0],
		.i2c_adapter_id = CAMERA_I2C_BUS_NUM,
	},
	{ NULL, 0 },
};
#endif
#ifdef CONFIG_VIDEO_MT9T111
static struct isp_subdev_i2c_board_info panther_mt9t111_primary_subdevs[] = {
	{
		.board_info = &panther_mt9t111_i2c_devices[0],
		.i2c_adapter_id = CAMERA_I2C_BUS_NUM,
	},
	{ NULL, 0 },
};
#endif

static struct isp_v4l2_subdevs_group panther_camera_subdevs[] = {
#ifdef CONFIG_VIDEO_MT9V113
	{
		.subdevs = panther_mt9v113_primary_subdevs,
		.interface = ISP_INTERFACE_PARALLEL,
		.bus = {
			.parallel = {
				.data_lane_shift	= 2,
				.clk_pol		= 0,
				.hdpol			= 0,
				.vdpol			= 0,
				.fldmode		= 0,
				.bridge			= 3,
			},
		},
	},
#endif
#ifdef CONFIG_VIDEO_OV5640
	{
		.subdevs = panther_ov5640_primary_subdevs,
		.interface = ISP_INTERFACE_PARALLEL,
		.bus = {
			.parallel = {
				.data_lane_shift	= 2,
				.clk_pol		= 0,
				.hdpol			= 0,
				.vdpol			= 0,
				.fldmode		= 0,
				.bridge			= 3,
			},
		},
	},
#endif
#ifdef CONFIG_VIDEO_MT9T111
	{
		.subdevs = panther_mt9t111_primary_subdevs,
		.interface = ISP_INTERFACE_PARALLEL,
		.bus = {
			.parallel = {
				.data_lane_shift	= 2,
				.clk_pol		= 0,
				.hdpol			= 0,
				.vdpol			= 0,
				.fldmode		= 0,
				.bridge			= 3,
			},
		},
	},
#endif
	{ NULL, 0 },
};

static struct isp_platform_data panther_isp_platform_data = {
	.subdevs = panther_camera_subdevs,
};

static int __init panther_cam_init(void)
{
	/*
	 * Sensor reset GPIO
	 */
	if (gpio_request(LEOPARD_RESET_GPIO, "cam_rst") != 0) {
		printk(KERN_ERR "panther-cam: Could not request GPIO %d\n",
				LEOPARD_RESET_GPIO);
		return -ENODEV;
	}

	
	if (gpio_request(CAM_EN_GPIO, "cam led") < 0)
		printk(KERN_ERR "can't control cam-led\n");


	/* set to output mode */
	gpio_direction_output(LEOPARD_RESET_GPIO, 0);

	/* Request POWER_DOWN_GPIO and set to output */
	if(gpio_request(POWER_DOWN_GPIO, "CAM power down") !=0)
	{
		printk(KERN_ERR "panther-cam: Could not request GPIO %d\n",
				POWER_DOWN_GPIO);
		return -ENODEV;
	}
	gpio_direction_output(POWER_DOWN_GPIO, 0); /* hood modify for debug */

	/* Request POWER_DOWN_GPIO and set to output */
	if(gpio_request(VDD_CAM_GPIO, "CAM vdd") !=0)
	{
		printk(KERN_ERR "panther-cam: Could not request GPIO %d\n",
				VDD_CAM_GPIO);
		return -ENODEV;
	}
	gpio_direction_output(VDD_CAM_GPIO, 0);

	omap3_init_camera(&panther_isp_platform_data);

	return 0;
}

static void __exit panther_cam_exit(void)
{
	gpio_free(LEOPARD_RESET_GPIO);
	gpio_free(POWER_DOWN_GPIO);
	gpio_free(VDD_CAM_GPIO);
}

/* hood modify for debug */


/* hood modify for debug */
#if 0
static int __init panther_cam_init(void)
{
	/*
	 * Regulator supply required for camera interface
	 */
	panther_cam_digital = regulator_get(NULL, "cam_digital");
	if (IS_ERR(panther_cam_digital)) {
		printk(KERN_ERR "cam_digital regulator missing\n");
		return PTR_ERR(panther_cam_digital);
	}
	panther_cam_io = regulator_get(NULL, "cam_io");
	if (IS_ERR(panther_cam_io)) {
		printk(KERN_ERR "cam_io regulator missing\n");
		regulator_put(panther_cam_digital);
		return PTR_ERR(panther_cam_io);
	}
	/*
	 * Sensor reset GPIO
	 */
	if (gpio_request(LEOPARD_RESET_GPIO, "cam_rst") != 0) {
		printk(KERN_ERR "panther-cam: Could not request GPIO %d\n",
				LEOPARD_RESET_GPIO);
		regulator_put(panther_cam_digital);
		regulator_put(panther_cam_io);
		return -ENODEV;
	}
	/* set to output mode */
	gpio_direction_output(LEOPARD_RESET_GPIO, 0);

	omap3_init_camera(&panther_isp_platform_data);

	return 0;
}

static void __exit panther_cam_exit(void)
{
	if (regulator_is_enabled(panther_cam_digital))
		regulator_disable(panther_cam_digital);
	regulator_put(panther_cam_digital);
	if (regulator_is_enabled(panther_cam_io))
		regulator_disable(panther_cam_io);
	regulator_put(panther_cam_io);

	gpio_free(LEOPARD_RESET_GPIO);
}
#endif

module_init(panther_cam_init);
module_exit(panther_cam_exit);

MODULE_AUTHOR("Jorjin Technologies");
MODULE_DESCRIPTION("Panther Camera Module");
MODULE_LICENSE("GPL");
