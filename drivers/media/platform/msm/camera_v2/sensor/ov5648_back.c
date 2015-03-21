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
#define OV5648_BACK_SENSOR_NAME "ov5648_back"

#include <mach/device_info.h>
#define DEVICE_VERSION_5648_SUNNY	"ov5648_sunny"
#define DEVICE_VERSION_5648_TRULY	"ov5648_truly"
#define DEVICE_MANUFACUTRE_5648	"OmniVision"

 
DEFINE_MSM_MUTEX(ov5648_back_mut);

static struct msm_sensor_ctrl_t ov5648_back_s_ctrl;
//lxl add for actuator driver
#ifdef VENDOR_EDIT
extern void set_vcm_vendor_id(unsigned char a);
static unsigned char vcm_id =1;
#endif

static struct msm_sensor_power_setting ov5648_back_power_setting[] = {	
//gpio-front pwdn
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{ 
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},	
//lxl add end


	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{ 
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},	
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 2,
	},
	
	//lxl add 
	#if 1
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 2,
	},
	#endif
	//lxl end
	
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000, // 24000000
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};


//lxl add for actuator driver
#ifdef VENDOR_EDIT
static void get_vcm_ID(struct msm_sensor_ctrl_t *s_ctrl)
{ 
//	unsigned char  flag;   	
	uint16_t flag;
	int i;
	int32_t rc = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,
		0x0103,
		0x01, MSM_CAMERA_I2C_BYTE_DATA);
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,
		0x0100,
		0x01, MSM_CAMERA_I2C_BYTE_DATA);
	mdelay(30);
	
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,
		0x3d84,
		0xc0, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: ov5648_back write1 failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
	}	
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,
		0x3d85,
		0x00, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: ov5648_back write2 failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
	}	
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,
		0x3d86,
		0x0f, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: ov5648_back write3 failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
	}	
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client,
		0x3d81,
		0x01, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: ov5648_back write4 failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
	}	
	mdelay(15);
	
#if 0
// lxl just for test
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client,
		0x3d84,
		&flag, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("lxl : 0x3d84:id =%d\n", flag);
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client,
		0x3d85,
		&flag, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("lxl : 0x3d85:id =%d\n", flag);
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client,
		0x3d86,
		&flag, MSM_CAMERA_I2C_BYTE_DATA);
	pr_err("lxl : 0x3d86:id =%d\n", flag);
//lxl add end
#endif
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client,
		0x3d05,
		&flag, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: ov5648_back read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
	}

	pr_err("lxl : 0x3d05:id =%d\n", flag);
	
	if ((!(flag&0x80)) && ((flag&0x7f) == 0x02))	  
	{	  
		for( i=0;i<16;i++)	
		{
       	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
       		s_ctrl->sensor_i2c_client,
       		0x3d00+i,
       		0x00, MSM_CAMERA_I2C_BYTE_DATA);
       	if (rc < 0) {
       		pr_err("%s: %s: ov5648_back write5 failed\n", __func__,
       			s_ctrl->sensordata->sensor_name);
       	}
		}
	   vcm_id = 2;
	}
	else if((!(flag&0x80)) && ((flag&0x7f) == 0x01)) 
	{
		for( i=0;i<16;i++)
		{
       	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
       		s_ctrl->sensor_i2c_client,
       		0x3d00+i,
       		0x00, MSM_CAMERA_I2C_BYTE_DATA);
       	if (rc < 0) {
       		pr_err("%s: %s: ov5648_back write6 failed\n", __func__,
       			s_ctrl->sensordata->sensor_name);
       	}	
		}
		vcm_id = 1;
	}
	else
	{	
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
       	s_ctrl->sensor_i2c_client,
       	0x3d0e,
       	&flag, MSM_CAMERA_I2C_BYTE_DATA);
       	if (rc < 0) {
       		pr_err("%s: %s: ov5648_back read id failed\n", __func__,
       			s_ctrl->sensordata->sensor_name);
       	}

			pr_err("lxl : 0x3d0e:id =%d\n", flag);
				
			for( i=0;i<16;i++)	
			{
              	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
              		s_ctrl->sensor_i2c_client,
              		0x3d00+i,
              		0x00, MSM_CAMERA_I2C_BYTE_DATA);
              	if (rc < 0) {
              		pr_err("%s: %s: ov5648_back write7 failed\n", __func__,
              			s_ctrl->sensordata->sensor_name);
              	}	
		  }	
			if ((!(flag&0x80)) && ((flag&0x7f) == 0x02))	  
			{	  
                 vcm_id = 2;
			}
			else if((!(flag&0x80)) && ((flag&0x7f) == 0x01)) 
			{
                 vcm_id = 1;
			} 				
			else
			{
               	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
              		s_ctrl->sensor_i2c_client,
              		0x3d84,
              		0xc0, MSM_CAMERA_I2C_BYTE_DATA);
              	if (rc < 0) {
              		pr_err("%s: %s: ov5648_back write11 failed\n", __func__,
              			s_ctrl->sensordata->sensor_name);
              	}	
              	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
              		s_ctrl->sensor_i2c_client,
              		0x3d85,
              		0x10, MSM_CAMERA_I2C_BYTE_DATA);
              	if (rc < 0) {
              		pr_err("%s: %s: ov5648_back write12 failed\n", __func__,
              			s_ctrl->sensordata->sensor_name);
              	}	
              	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
              		s_ctrl->sensor_i2c_client,
              		0x3d86,
              		0x1f, MSM_CAMERA_I2C_BYTE_DATA);
              	if (rc < 0) {
              		pr_err("%s: %s: ov5648_back write13 failed\n", __func__,
              			s_ctrl->sensordata->sensor_name);
              	}	
              	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
              		s_ctrl->sensor_i2c_client,
              		0x3d81,
              		0x01, MSM_CAMERA_I2C_BYTE_DATA);
              	if (rc < 0) {
              		pr_err("%s: %s: ov5648_back write14 failed\n", __func__,
              			s_ctrl->sensordata->sensor_name);
              	}	
	             mdelay(15);  
							 
       			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
              	s_ctrl->sensor_i2c_client,
              	0x3d07,
              	&flag, MSM_CAMERA_I2C_BYTE_DATA);
              	if (rc < 0) {
              		pr_err("%s: %s: ov5648_back read id failed\n", __func__,
              			s_ctrl->sensordata->sensor_name);
              	}  

                 pr_err("lxl : 0x3d07:id =%d\n", flag);
					
            		for( i=0;i<16;i++)
            		{
                   	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                   		s_ctrl->sensor_i2c_client,
                   		0x3d00+i,
                   		0x00, MSM_CAMERA_I2C_BYTE_DATA);
                   	if (rc < 0) {
                   		pr_err("%s: %s: ov5648_back write22 failed\n", __func__,
                   			s_ctrl->sensordata->sensor_name);
                   	}	
            		}			
					if ((!(flag&0x80)) && ((flag&0x7f) == 0x02))	  
					{	  
                      vcm_id = 2;
					}
					else if((!(flag&0x80)) && ((flag&0x7f) == 0x01)) 
					{
					     vcm_id = 1;							 
					} 
					else
					{
                   		pr_err("%s: %s: ov5648_back write24 failed\n", __func__,
                   			s_ctrl->sensordata->sensor_name);	 
					}						
			}  
	}	
	set_vcm_vendor_id(vcm_id);
	s_ctrl->sensordata->sensor_info->module_vendor_id = (0xa0|vcm_id); //lxl add 
}

static int32_t ov5648_back_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
		&chipid, MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: %s: ov5648_back read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	pr_err("%s: read id: %x expected id %x:\n", __func__, chipid,
		s_ctrl->sensordata->slave_info->sensor_id);
	if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}	
	get_vcm_ID(s_ctrl);
	return rc;
}


static struct msm_sensor_fn_t ov5648_back_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = ov5648_back_sensor_match_id,
};
#endif

static struct v4l2_subdev_info ov5648_back_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static struct msm_camera_i2c_client ov5648_back_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static struct msm_sensor_ctrl_t ov5648_back_s_ctrl = {
	.sensor_i2c_client = &ov5648_back_sensor_i2c_client,
	.power_setting_array.power_setting = ov5648_back_power_setting,
	.power_setting_array.size =
			ARRAY_SIZE(ov5648_back_power_setting),
	.msm_sensor_mutex = &ov5648_back_mut,
	.sensor_v4l2_subdev_info = ov5648_back_subdev_info,
	.sensor_v4l2_subdev_info_size =
			ARRAY_SIZE(ov5648_back_subdev_info),
	.func_tbl = &ov5648_back_sensor_func_tbl, // lxl add 
};

static const struct of_device_id ov5648_back_dt_match[] = {
	{
		.compatible = "qcom,ov5648_back",
		.data = &ov5648_back_s_ctrl
	},
	{}
};

MODULE_DEVICE_TABLE(of, ov5648_back_dt_match);

static struct platform_driver ov5648_back_platform_driver = {
	.driver = {
		.name = "qcom,ov5648_back",
		.owner = THIS_MODULE,
		.of_match_table = ov5648_back_dt_match,
	},
};

static int32_t ov5648_back_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov5648_back_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);

//lxl@camera add for module vendor distinction 	
    if(!rc)
    {
        if(1 == vcm_id)
            register_device_proc("r_camera", DEVICE_VERSION_5648_SUNNY, DEVICE_MANUFACUTRE_5648);
        else if(2 == vcm_id)
            register_device_proc("r_camera", DEVICE_VERSION_5648_TRULY, DEVICE_MANUFACUTRE_5648);
    }
    return rc;
}
 
static int __init ov5648_back_init_module(void)
{
	int32_t rc = 0;
	rc = platform_driver_probe(&ov5648_back_platform_driver,
		ov5648_back_platform_probe);
	if (!rc)
		return rc;

      return rc;
}

static void __exit ov5648_back_exit_module(void)
{
	if (ov5648_back_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov5648_back_s_ctrl);
		platform_driver_unregister(&ov5648_back_platform_driver);
	}
    
	return;
}

module_init(ov5648_back_init_module);
module_exit(ov5648_back_exit_module);
MODULE_DESCRIPTION("ov5648_back");
MODULE_LICENSE("GPL v2");
