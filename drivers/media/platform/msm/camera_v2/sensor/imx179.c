/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#define IMX179_SENSOR_NAME "imx179"

#include <mach/device_info.h>
#define DEVICE_VERSION_14027_B	"Imx179"
#define DEVICE_MANUFACUTRE_14027_B	"Sony"


DEFINE_MSM_MUTEX(imx179_mut);

static struct msm_sensor_ctrl_t imx179_s_ctrl;
#if 1
static struct msm_sensor_power_setting imx179_power_setting[] = 
{	
//gpio-front pwdn -begin
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{ 
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},	
//zhaozhengtao modify end 20140327

	{	
		  .seq_type = SENSOR_GPIO,
		  .seq_val = SENSOR_GPIO_RESET,	
		  .config_val = GPIO_OUT_LOW,
		  .delay =0,
       },	
		  
      {		
	       .seq_type = SENSOR_VREG,		
	       .seq_val = CAM_VANA,		
		.config_val = 0,		
		.delay = 0,	
	},	
	{		
		.seq_type = SENSOR_VREG,	
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
	},	
	{		
	       .seq_type = SENSOR_CLK,	
		 .seq_val = SENSOR_CAM_MCLK,	
		 .config_val = 0,		
		 .delay = 1,	
	},		
	{	
		 .seq_type = SENSOR_VREG,	
		 .seq_val = CAM_VDIG,	
		 .config_val = 0,		
		 .delay = 1,
	},
	{		
	        .seq_type = SENSOR_VREG,
		  .seq_val = CAM_VAF,	
		  .config_val = 0,		
		  .delay = 0,	
       },

       {		
		  .seq_type = SENSOR_GPIO,	
		  .seq_val = SENSOR_GPIO_RESET,	
		  .config_val = GPIO_OUT_HIGH,	
		  .delay =2,
       },
	{	
		  .seq_type = SENSOR_I2C_MUX,
		  .seq_val = 0,	
		  .config_val = 0,
		  .delay = 0,	
       },
};
#else
static struct msm_sensor_power_setting imx179_power_setting[] =

{
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};
#endif
static struct v4l2_subdev_info imx179_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static struct msm_camera_i2c_client imx179_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id imx179_dt_match[] = {
	{.compatible = "qcom,imx179", .data = &imx179_s_ctrl},
};

MODULE_DEVICE_TABLE(of, imx179_dt_match);

static struct platform_driver imx179_platform_driver = {
	.driver = {
		.name = "qcom,imx179",
		.owner = THIS_MODULE,
		.of_match_table = imx179_dt_match,
	},
};

static int32_t imx179_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(imx179_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
    if(!rc)
    {
		printk("%s: register device proc \n", __func__);
        register_device_proc("r_camera", DEVICE_VERSION_14027_B, DEVICE_MANUFACUTRE_14027_B);
    }

	return rc;
}

static int __init imx179_init_module(void)
{
	int32_t rc = 0; 	
	rc = platform_driver_probe(&imx179_platform_driver,
		imx179_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);

    return rc;
}
 
static void __exit imx179_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);

      if (imx179_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx179_s_ctrl);
		platform_driver_unregister(&imx179_platform_driver);
	}

	return;
}

static struct msm_sensor_ctrl_t imx179_s_ctrl = {
	.sensor_i2c_client = &imx179_sensor_i2c_client,
	.power_setting_array.power_setting = imx179_power_setting,
	.power_setting_array.size = ARRAY_SIZE(imx179_power_setting),
	.msm_sensor_mutex = &imx179_mut,
	.sensor_v4l2_subdev_info = imx179_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx179_subdev_info),
};

module_init(imx179_init_module);
module_exit(imx179_exit_module);
MODULE_DESCRIPTION("imx179");
MODULE_LICENSE("GPL v2");
