/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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
/*
*This is a cci compatible led flash driver with third party driver ic.
*The cci is used, therefore we can not use i2c.
*Add the register table in probe function to support new ic.
*please mind the regulators and gpios for various board config.
*/
#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include "msm_led_cci.h"

#define FLASH_NAME "camera-led-flash"

#undef CDBG
#undef CONFIG_MSMB_CAMERA_DEBUG

//lxl add 
#ifndef CONFIG_MSMB_CAMERA_DEBUG
#define CONFIG_MSMB_CAMERA_DEBUG
#endif

#define CONFIG_MSMB_CAMERA_DEBUG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static struct msm_led_cci_ctrl_t fctrl;

static struct msm_camera_i2c_fn_t msm_led_cci_i2c_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
};

// guanjd@camtdt modify
static struct msm_camera_i2c_reg_array adp1650_init_array[] = {
	{0x04, 0xa4},
	//{0x0f, 0x00},
};

static struct msm_camera_i2c_reg_array adp1650_off_array[] = {
	{0x04, 0xa4},
};

static struct msm_camera_i2c_reg_array adp1650_release_array[] = {
	{0x04, 0xa4},
	//{0x0f, 0x00},
};

static struct msm_camera_i2c_reg_array adp1650_low_array[] = {
	{0x03, 0x03},  //torch 100ma
	{0x04, 0xae},  //set torch mode
};

static struct msm_camera_i2c_reg_array adp1650_high_array[] = {
	//{0x03, 0x48},  //flash 750ma
	{0x03, 0x70},  //flash 1A
	{0x04, 0xaf},  //set flash mode
};

static struct msm_camera_i2c_reg_setting adp1650_init_setting = {
	.reg_setting = adp1650_init_array,
	.size = ARRAY_SIZE(adp1650_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting adp1650_off_setting = {
	.reg_setting = adp1650_off_array,
	.size = ARRAY_SIZE(adp1650_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting adp1650_release_setting = {
	.reg_setting = adp1650_release_array,
	.size = ARRAY_SIZE(adp1650_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting adp1650_low_setting = {
	.reg_setting = adp1650_low_array,
	.size = ARRAY_SIZE(adp1650_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting adp1650_high_setting = {
	.reg_setting = adp1650_high_array,
	.size = ARRAY_SIZE(adp1650_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_cci_reg_t adp1650_regs = {
	.init_setting = &adp1650_init_setting,
	.off_setting = &adp1650_off_setting,
	.low_setting = &adp1650_low_setting,
	.high_setting = &adp1650_high_setting,
	.release_setting = &adp1650_release_setting,
};

static struct msm_camera_i2c_reg_array lm3642_init_array[] = {
	{0x0a, 0x00},
	//{0x0f, 0x00},
};

static struct msm_camera_i2c_reg_array lm3642_off_array[] = {
	{0x0a, 0x00},
};

static struct msm_camera_i2c_reg_array lm3642_release_array[] = {
	{0x0a, 0x00},
	//{0x0f, 0x00},
};

static struct msm_camera_i2c_reg_array lm3642_low_array[] = {
	//{0x09, 0x10},  //torch 100ma
	//{0x0a, 0x02},  //set torch mode
	{0x09, 0x10},  //Torch current 93.74mA				
	{0x06, 0x00},  //Torch Ramp-Up/Down Time				
	{0x0A, 0x12}, //Torch mode enable
};

static struct msm_camera_i2c_reg_array lm3642_high_array[] = {
	
	//{0x09, 0x0a},  //flash 1A
	//{0x0a, 0x03},  //set flash mode
	{0x09, 0x0A},  //flash 1A
	{0x08, 0x57}, 
	{0x0A, 0x23},
};

static struct msm_camera_i2c_reg_setting lm3642_init_setting = {
	.reg_setting = lm3642_init_array,
	.size = ARRAY_SIZE(lm3642_init_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_off_setting = {
	.reg_setting = lm3642_off_array,
	.size = ARRAY_SIZE(lm3642_off_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_release_setting = {
	.reg_setting = lm3642_release_array,
	.size = ARRAY_SIZE(lm3642_release_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_low_setting = {
	.reg_setting = lm3642_low_array,
	.size = ARRAY_SIZE(lm3642_low_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_camera_i2c_reg_setting lm3642_high_setting = {
	.reg_setting = lm3642_high_array,
	.size = ARRAY_SIZE(lm3642_high_array),
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	.data_type = MSM_CAMERA_I2C_BYTE_DATA,
	.delay = 0,
};

static struct msm_led_cci_reg_t lm3642_regs = {
	.init_setting = &lm3642_init_setting,
	.off_setting = &lm3642_off_setting,
	.low_setting = &lm3642_low_setting,
	.high_setting = &lm3642_high_setting,
	.release_setting = &lm3642_release_setting,
};


static int32_t msm_led_cci_get_subdev_id(struct msm_led_cci_ctrl_t *fctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	if (!subdev_id) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return -EINVAL;
	}
	*subdev_id = fctrl->pdev->id;
	CDBG("%s:%d subdev_id %d\n", __func__, __LINE__, *subdev_id);
	return 0;
}

static void msm_led_cci_set_brightness(struct msm_led_cci_ctrl_t *fctrl,
				      enum msm_camera_led_config_t cfgtype)
{
    int rc = 0;
    //uint16_t reg_val = 0;
    struct msm_camera_i2c_client *i2c_client = NULL;
    struct msm_led_cci_led_info_t *led_info = NULL;

	pr_err("cfgtype is %d\n", cfgtype);

	if (fctrl->led_info->status == cfgtype) {
	    pr_err("status the same as old\n");
	    return;
	}

    led_info = fctrl->led_info;
    i2c_client = fctrl->flash_i2c_client;
    if (!i2c_client || !led_info) {
        pr_err("null i2c_client or led_info\n");
        return;
    }


    switch (cfgtype) {
    case MSM_CAMERA_LED_INIT:
        /*request gpio flash_en*/
        rc = gpio_request(fctrl->led_info->gpio_en, "msm_led_cci");
        if (rc < 0) {
            pr_err("get gpio failed\n");
        }
        rc = gpio_direction_output(fctrl->led_info->gpio_en, 1);
        if (rc < 0)
            pr_err("set gpio to 1 failed\n");

        /*request gpio strobe_en*/
        rc = gpio_request(fctrl->led_info->gpio_strobe_en, "msm_led_cci");
        if (rc < 0) {
            pr_err("get gpio failed\n");
        }
        rc = gpio_direction_output(fctrl->led_info->gpio_strobe_en, 0);
        if (rc < 0)
            pr_err("set gpio to 0 failed\n");

        /*init array*/
        /*rc = i2c_client->i2c_func_tbl->i2c_write_table(i2c_client, fctrl->reg_setting->init_setting);
        if (rc < 0)
            pr_err("MSM_CAMERA_LED_INIT failed\n");*/

        break;

    case MSM_CAMERA_LED_LOW:
        rc = i2c_client->i2c_func_tbl->i2c_write_table(i2c_client, fctrl->reg_setting->low_setting);
        if (rc < 0)
            pr_err("MSM_CAMERA_LED_LOW failed\n");

        break;

    case MSM_CAMERA_LED_HIGH:
        rc = i2c_client->i2c_func_tbl->i2c_write_table(i2c_client, fctrl->reg_setting->high_setting);
        if (rc < 0)
            pr_err("MSM_CAMERA_LED_HIGH failed\n");

        rc = gpio_direction_output(fctrl->led_info->gpio_strobe_en, 1);
        if (rc < 0)
            pr_err("set gpio to 1 failed\n");

        break;

    case MSM_CAMERA_LED_OFF:
        #if 0  //not read fault_info_reg
        /*read fault reg*/
        rc = i2c_client->i2c_func_tbl->i2c_read(i2c_client, led_info->fault_info_reg,
            &reg_val, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc < 0) {
            pr_err("cci read failed\n");
        } else {
            if (reg_val)
                pr_err("fault occured, reg value is 0x%X\n", reg_val);
        }
        #endif
        /*write off reg*/
        rc = i2c_client->i2c_func_tbl->i2c_write_table(i2c_client, fctrl->reg_setting->off_setting);
        if (rc < 0)
            pr_err("MSM_CAMERA_LED_OFF failed\n");

        /*disable*/
        if (fctrl->led_info->status == MSM_CAMERA_LED_HIGH) {
            rc = gpio_direction_output(fctrl->led_info->gpio_strobe_en, 0);
            if (rc < 0)
                pr_err("set gpio to 0 failed\n");
        }

        break;

    case MSM_CAMERA_LED_RELEASE:
        rc = gpio_direction_output(fctrl->led_info->gpio_en, 0);
        if (!rc)
            gpio_free(fctrl->led_info->gpio_en);
        else
            pr_err("set gpio to 0 failed\n");

        rc = gpio_direction_output(fctrl->led_info->gpio_strobe_en, 0);
        if (!rc)
            gpio_free(fctrl->led_info->gpio_strobe_en);
        else
            pr_err("set gpio to 0 failed\n");

        break;

    default:
        pr_err("invalid cfgtype\n");
        break;
    }

    fctrl->led_info->status = cfgtype;

    return;
}

static int32_t msm_led_cci_config(struct msm_led_cci_ctrl_t *fctrl,
	void *data)
{
	int rc = 0;
	struct msm_camera_led_cfg_t *cfg = (struct msm_camera_led_cfg_t *)data;

	CDBG("called, set to %d\n", cfg->cfgtype);

	if (!fctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}

    msm_led_cci_set_brightness(fctrl, cfg->cfgtype);

	return rc;
}

static long msm_led_cci_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	struct msm_led_cci_ctrl_t *fctrl = NULL;
	void __user *argp = (void __user *)arg;
	if (!sd) {
		pr_err("sd NULL\n");
		return -EINVAL;
	}
	fctrl = v4l2_get_subdevdata(sd);
	if (!fctrl) {
		pr_err("fctrl NULL\n");
		return -EINVAL;
	}
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return fctrl->func_tbl->flash_get_subdev_id(fctrl, argp);
	case VIDIOC_MSM_FLASH_LED_DATA_CFG:
		return fctrl->func_tbl->flash_led_config(fctrl, argp);
	case MSM_SD_SHUTDOWN:
		*(int *)argp = MSM_CAMERA_LED_RELEASE;
		return fctrl->func_tbl->flash_led_config(fctrl, argp);
	default:
		pr_err("invalid cmd %d\n", cmd);
		return -ENOIOCTLCMD;
	}
}

static struct v4l2_subdev_core_ops msm_led_cci_subdev_core_ops = {
	.ioctl = msm_led_cci_subdev_ioctl,
};

static struct v4l2_subdev_ops msm_led_cci_subdev_ops = {
	.core = &msm_led_cci_subdev_core_ops,
};


static const struct v4l2_subdev_internal_ops msm_led_cci_internal_ops;

static int32_t msm_led_cci_create_v4lsubdev(struct platform_device *pdev, void *data)
{
	struct msm_led_cci_ctrl_t *fctrl =
		(struct msm_led_cci_ctrl_t *)data;
	CDBG("Enter\n");

	if (!fctrl) {
		pr_err("fctrl NULL\n");
		return -EINVAL;
	}

	/* Initialize sub device */
	v4l2_subdev_init(&fctrl->msm_sd.sd, &msm_led_cci_subdev_ops);
	v4l2_set_subdevdata(&fctrl->msm_sd.sd, fctrl);

	fctrl->pdev = pdev;
	fctrl->msm_sd.sd.internal_ops = &msm_led_cci_internal_ops;
	fctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(fctrl->msm_sd.sd.name, ARRAY_SIZE(fctrl->msm_sd.sd.name),
		"msm_flash");
	media_entity_init(&fctrl->msm_sd.sd.entity, 0, NULL, 0);
	fctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	fctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_LED_FLASH;
	fctrl->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x1;
	msm_sd_register(&fctrl->msm_sd);

	CDBG("probe success\n");
	return 0;
}

static int32_t msm_led_cci_get_dt_led_data(struct msm_led_cci_ctrl_t *fctrl,
    struct device_node *of_node, uint32_t index)
{
	int32_t rc = 0;
    struct msm_led_cci_led_info_t *led_info;
    const char *flash_name;
    char *flash_name1;

    led_info = kzalloc(sizeof(struct msm_led_cci_led_info_t), GFP_KERNEL);

    flash_name = flash_name1 = kzalloc(16, GFP_KERNEL);
	rc = of_property_read_string(of_node,
		"qcom,name", &flash_name);
	if (rc < 0) {
		pr_err("failed of_property_read\n");
		return -EINVAL;
	}
    CDBG("flash name is %s\n", flash_name);
    strcpy(led_info->name, flash_name);
    kzfree(flash_name1);
    flash_name = flash_name1 = NULL;

	rc = of_property_read_u32(of_node,
		"qcom,cci-master", &led_info->cci_master);
	if (rc < 0) {
		pr_err("failed of_property_read\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(of_node,
		"qcom,slave-id", &led_info->slave_id);
	if (rc < 0) {
		pr_err("failed of_property_read\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(of_node,
		"qcom,chip-id-reg", &led_info->chip_id_reg);
	if (rc < 0) {
		pr_err("failed of_property_read\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(of_node,
		"qcom,chip-id", &led_info->chip_id);
	if (rc < 0) {
		pr_err("failed of_property_read\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(of_node,
		"qcom,fault-info-reg", &led_info->fault_info_reg);
	if (rc < 0) {
		pr_err("failed of_property_read\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(of_node,
		"qcom,max-torch-current", &led_info->max_torch_current);
	if (rc < 0) {
		pr_err("failed of_property_read\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(of_node,
		"qcom,max-flash-current", &led_info->max_flash_current);
	if (rc < 0) {
		pr_err("failed of_property_read\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(of_node,
		"qcom,gpio-en", &led_info->gpio_en);
	if (rc < 0) {
		pr_err("failed of_property_read\n");
	}
      /*
      rc = of_property_read_u32(of_node,
		"qcom,gpio-torch-en", &led_info->gpio_torch_en);
	if (rc < 0) {
		pr_err("failed of_property_read\n");
	}
	*/
	rc = of_property_read_u32(of_node,
		"qcom,gpio-strobe-en", &led_info->gpio_strobe_en);
	if (rc < 0) {
		pr_err("failed of_property_read\n");
	}
	rc = of_property_read_string(of_node,
		"qcom,regulator-name", &led_info->regulator_name);
	if (rc < 0) {
		pr_err("failed of_property_read\n");
		return -EINVAL;
	}

      CDBG("name is %s, regulator name %s, gpio en-%d strobe-%d\n",
          led_info->name, led_info->regulator_name, led_info->gpio_en, led_info->gpio_strobe_en);
  
      fctrl->led_info = led_info;

      return 0;

}

static int32_t msm_led_cci_query_ic(struct msm_led_cci_ctrl_t *fctrl, struct device_node *of_node)
{
    int32_t i, rc, ret = 0;
    uint32_t count = 0;
    uint16_t chip_id_reg, chip_id;
    bool matched = false;
    struct regulator *cci_regulator = NULL;
    struct msm_camera_i2c_client *cci_i2c_client = NULL;
    struct device_node *flash_src_node = NULL;

    CDBG("called\n");

    if (!of_get_property(of_node, "qcom,flash-cci-source", &count)) {
        pr_err("can not get qcom,flash-cci-source\n");
        return -EINVAL;
    }

    count /= sizeof(uint32_t);
	CDBG("count %d\n", count);

    for (i=0; i<count && !matched; i++) {
        fctrl->led_info = NULL;
        /*get led info form dt*/
        flash_src_node = of_parse_phandle(of_node,
            "qcom,flash-cci-source", i);
        if (!flash_src_node) {
            continue;
        }
        rc = msm_led_cci_get_dt_led_data(fctrl, flash_src_node, i);
        if (rc < 0) {
            pr_err("get dt data failed\n");
            of_node_put(flash_src_node);
            goto FAILED;
        }
        of_node_put(flash_src_node);

    #if 0  //not read id when proble
        /*init cci*/
        fctrl->flash_i2c_client->cci_client->sid = fctrl->led_info->slave_id;
        fctrl->flash_i2c_client->cci_client->cci_i2c_master = fctrl->led_info->cci_master;   
        matched = true;
	  ret = 0;
    #else
        /*get power*/
        cci_regulator = regulator_get(&fctrl->pdev->dev, fctrl->led_info->regulator_name);
        if (IS_ERR_OR_NULL(cci_regulator)) {
            pr_err("regulator_get failed\n");
            goto FAILED;
        }
        /*enable power*/
		#ifdef VENDOR_EDIT
		//guanjd add for cci power
		rc = regulator_set_voltage(
				cci_regulator,1800000,
				1900000);
			pr_err("%s: %s is ldo\n", __func__,
				fctrl->led_info->regulator_name);
			if (rc < 0) {
				pr_err("%s: %s set voltage failed\n",
					__func__, fctrl->led_info->regulator_name);
				
			}
		#endif
		
        rc = regulator_enable(cci_regulator);
        if (rc < 0) {
            pr_err("regulator_enable failed\n");
            regulator_put(cci_regulator);
            goto FAILED;
        }
        /*request gpio*/
        rc = gpio_request(fctrl->led_info->gpio_en, "msm_led_cci");
        if (rc < 0) {
            pr_err("get gpio failed\n");
        }
        rc = gpio_direction_output(fctrl->led_info->gpio_en, 1);
        if (rc < 0)
            pr_err("set gpio to 1 failed\n");
        
        /*init cci*/
        fctrl->flash_i2c_client->cci_client->sid = fctrl->led_info->slave_id;
        fctrl->flash_i2c_client->cci_client->cci_i2c_master =
            fctrl->led_info->cci_master;
        cci_i2c_client = fctrl->flash_i2c_client;
        rc = cci_i2c_client->i2c_func_tbl->i2c_util(
    			cci_i2c_client, MSM_CCI_INIT);
    	if (rc < 0) {
    	    pr_err("cci init failed\n");
    	}
    	/*read id*/
    	chip_id_reg = fctrl->led_info->chip_id_reg;
        rc = cci_i2c_client->i2c_func_tbl->i2c_read(
    			cci_i2c_client, chip_id_reg, &chip_id, MSM_CAMERA_I2C_BYTE_DATA);
		pr_err("cci read extenal flash id=%d\n",chip_id);
    	if (rc < 0) {
                pr_err("cci read failed\n");
                /*release cci*/
                rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
                    fctrl->flash_i2c_client, MSM_CCI_RELEASE);
                if (rc < 0) {
                    pr_err("cci release failed\n");
                }

                /*release gpio*/
                rc = gpio_direction_output(fctrl->led_info->gpio_en, 0);
                if (!rc)
                    gpio_free(fctrl->led_info->gpio_en);
                else
                    pr_err("set gpio to 0 failed\n");

                /*disable power*/
                rc = regulator_disable(cci_regulator);
                if (rc < 0) {
                    pr_err("regulator_disable failed\n");
                    regulator_put(cci_regulator);
                    goto FAILED;
                }
                regulator_put(cci_regulator);

                if (fctrl->led_info) {
                    kzfree(fctrl->led_info);
                    fctrl->led_info = NULL;
                }
                ret = -EINVAL;
                return ret;
    	} else {
            if (chip_id == fctrl->led_info->chip_id) {
        	    pr_err("chip id match: 0x%x i=%d\n", chip_id,i);
        	    matched = true;
        	    ret = 0;
    	    } else {
        	    pr_err("chip id not match: 0x%x i=%d\n", chip_id,i);
        	    if (fctrl->led_info) {
        	        kzfree(fctrl->led_info);
        	    }
        	    ret = -EINVAL;
    	    }
    	}
    	/*release cci*/
        rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
            fctrl->flash_i2c_client, MSM_CCI_RELEASE);
    	if (rc < 0) {
    	    pr_err("cci release failed\n");
    	}

        /*release gpio*/
        rc = gpio_direction_output(fctrl->led_info->gpio_en, 0);
        if (!rc)
            gpio_free(fctrl->led_info->gpio_en);
        else
            pr_err("set gpio to 0 failed\n");
        
    	/*disable power*/
        rc = regulator_disable(cci_regulator);
        if (rc < 0) {
            pr_err("regulator_disable failed\n");
            regulator_put(cci_regulator);
            goto FAILED;
        }
        regulator_put(cci_regulator);
    #endif
    }

    return ret;
FAILED:
    if (fctrl->led_info) {
        kzfree(fctrl->led_info);
        fctrl->led_info = NULL;
    }
    return -EINVAL;
}

static const struct of_device_id msm_led_cci_dt_match[] = {
	{.compatible = "qcom,camera-led-flash"},
	{}
};

static void msm_led_cci_test_init(void)
{
    int rc;
	struct msm_camera_i2c_client *cci_i2c_client = NULL;

    /*get power*/
    fctrl.led_info->retulator = regulator_get(&fctrl.pdev->dev, fctrl.led_info->regulator_name);
    if (IS_ERR_OR_NULL(fctrl.led_info->retulator)) {
        pr_err("regulator_get failed\n");
        return;
    }
    /*enable power*/
    rc = regulator_enable(fctrl.led_info->retulator);
    if (rc < 0) {
        pr_err("regulator_enable failed\n");
        regulator_put(fctrl.led_info->retulator);
        return;
    }

    msm_led_cci_set_brightness(&fctrl, MSM_CAMERA_LED_INIT);
    
    /*init cci*/
    fctrl.flash_i2c_client->cci_client->sid = fctrl.led_info->slave_id;
    fctrl.flash_i2c_client->cci_client->cci_i2c_master =
        fctrl.led_info->cci_master;
    cci_i2c_client = fctrl.flash_i2c_client;
    rc = cci_i2c_client->i2c_func_tbl->i2c_util(
            cci_i2c_client, MSM_CCI_INIT);
    if (rc < 0) {
        pr_err("cci init failed\n");
    }

    return;
}

static void msm_led_cci_test_off(void)
{
    int rc;

    //fctrl.led_info->status = MSM_CAMERA_LED_OFF;
    if (fctrl.led_info->test_mode == 2) {
        cancel_delayed_work_sync(&fctrl.led_info->dwork);
    }
    msm_led_cci_set_brightness(&fctrl, MSM_CAMERA_LED_OFF);

    /*release cci*/
    rc = fctrl.flash_i2c_client->i2c_func_tbl->i2c_util(
        fctrl.flash_i2c_client, MSM_CCI_RELEASE);
    if (rc < 0) {
        pr_err("cci release failed\n");
    }
    
    msm_led_cci_set_brightness(&fctrl, MSM_CAMERA_LED_RELEASE);

    /*disable power*/
    rc = regulator_disable(fctrl.led_info->retulator);
    if (rc < 0) {
        pr_err("regulator_disable failed\n");
        regulator_put(fctrl.led_info->retulator);
        return;
    }
    regulator_put(fctrl.led_info->retulator);

    return;
}

static void msm_led_cci_test_blink_work(struct work_struct *work)
{
    struct delayed_work *dwork = to_delayed_work(work);
    msm_led_cci_set_brightness(&fctrl, fctrl.led_info->test_status);
    fctrl.led_info->test_status = !fctrl.led_info->test_status;
    schedule_delayed_work(dwork, msecs_to_jiffies(1100));
}

static ssize_t msm_led_cci_test_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int new_mode = simple_strtoul(buf, NULL, 10);
    int *pold_mode = &fctrl.led_info->test_mode;
    if (new_mode == *pold_mode) {
        pr_err("the same mode as old\n");
        return count;
    }

    if (*pold_mode!=0 && new_mode!=0) {
        fctrl.led_info->test_status = 0;
        msm_led_cci_test_off();
        *pold_mode = 0;
    }

    switch (new_mode) {
    case 0:
        if (*pold_mode==1 || *pold_mode==2 || *pold_mode==3) {
            fctrl.led_info->test_status = 0;
            msm_led_cci_test_off();
            *pold_mode = 0;
        }
        break;
    case 1:
        if (*pold_mode==0) {
            fctrl.led_info->test_status = 1;
            msm_led_cci_test_init();
            msm_led_cci_set_brightness(&fctrl, MSM_CAMERA_LED_LOW);
            *pold_mode = 1;
        }
        break;
    case 2:
        if (*pold_mode==0) {
            fctrl.led_info->test_status = 1;
            msm_led_cci_test_init();
            INIT_DELAYED_WORK(&fctrl.led_info->dwork, msm_led_cci_test_blink_work);
            schedule_delayed_work(&fctrl.led_info->dwork, msecs_to_jiffies(50));
            *pold_mode = 2;
        }
        break;
    case 3:
        if (*pold_mode==0) {
            fctrl.led_info->test_status = 1;
            msm_led_cci_test_init();
            msm_led_cci_set_brightness(&fctrl, MSM_CAMERA_LED_HIGH);
            *pold_mode = 3;
        }
        break;
    default:
        pr_err("invalid mode\n");
        break;
    }
	
	return count;
}

static DEVICE_ATTR(test, 0660,
		   NULL, msm_led_cci_test_store);

#ifdef VENDOR_EDIT
//LiuBin@Camera, 2014/04/14, Add proc for flash test
#include <linux/proc_fs.h>
static int flash_proc_read(char *page, char **start, off_t off, int count,
   int *eof, void *data)
{
	
	int len = 0;

	if (fctrl.led_info == NULL)
	{
		pr_err("led_info is NULL \n");
		return 0;
	}
	
	len = sprintf(page, fctrl.led_info->name);
	if (len <= off+count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	return len;
}

static int flash_proc_write(struct file *filp, const char __user *buff,
                        	unsigned long len, void *data)
{
	char buf[8] = {0};
	int new_mode = 0;
	
    int *pold_mode = &fctrl.led_info->test_mode;

	if (len > 8)
		len = 8;

	if (copy_from_user(buf, buff, len)) {
		pr_err("proc write error.\n");
		return -EFAULT;
	}

	new_mode = simple_strtoul(buf, NULL, 10);
	if (new_mode == *pold_mode) {
        pr_err("the same mode as old\n");
        return len;
    }
	
	switch (new_mode) {
    case 0:
        if (*pold_mode==1 || *pold_mode==2 || *pold_mode==3) {
            fctrl.led_info->test_status = 0;
            msm_led_cci_test_off();
            *pold_mode = 0;
        }
        break;
    case 1:
        if (*pold_mode==0) {
            fctrl.led_info->test_status = 1;
            msm_led_cci_test_init();
            msm_led_cci_set_brightness(&fctrl, MSM_CAMERA_LED_LOW);
            *pold_mode = 1;
        }
        break;
    case 2:
        if (*pold_mode==0) {
            fctrl.led_info->test_status = 1;
            msm_led_cci_test_init();
            INIT_DELAYED_WORK(&fctrl.led_info->dwork, msm_led_cci_test_blink_work);
            schedule_delayed_work(&fctrl.led_info->dwork, msecs_to_jiffies(50));
            *pold_mode = 2;
        }
        break;
    case 3:
        if (*pold_mode==0) {
            fctrl.led_info->test_status = 1;
            msm_led_cci_test_init();
            msm_led_cci_set_brightness(&fctrl, MSM_CAMERA_LED_HIGH);
            *pold_mode = 3;
        }
        break;
    default:
        pr_err("invalid mode\n");
        break;
    }
	
	return len;
}

static int flash_proc_init(struct msm_led_cci_ctrl_t *flash_ctl)
{
	int ret=0;
	
	struct proc_dir_entry *proc_entry = create_proc_entry( "qcom_flash", 0666, NULL);

	if (proc_entry == NULL)
	{
		ret = -ENOMEM;
	  	pr_err("[%s]: Error! Couldn't create qcom_flash proc entry\n", __func__);
	}
	else
	{
		proc_entry->data = flash_ctl;
		proc_entry->read_proc = flash_proc_read;
		proc_entry->write_proc = flash_proc_write;
		pr_err("[%s]: create qcom_flash proc success \n", __func__);
	}
	
	return ret;
}
#endif /* VENDOR_EDIT */

static struct attribute *msm_led_cci_attributes[] = {
        &dev_attr_test.attr,
        NULL
};

static const struct attribute_group msm_led_cci_attr_group = {
	.attrs = msm_led_cci_attributes,
};

MODULE_DEVICE_TABLE(of, msm_led_cci_dt_match);

static struct platform_driver msm_led_cci_driver = {
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = msm_led_cci_dt_match,
	},
};

static int32_t msm_led_cci_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct device_node *of_node = pdev->dev.of_node;

	CDBG("called\n");

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	fctrl.pdev = pdev;

	rc = of_property_read_u32(of_node, "cell-index", &pdev->id);
	if (rc < 0) {
		pr_err("failed\n");
		return -EINVAL;
	}
	CDBG("pdev id %d\n", pdev->id);

    fctrl.flash_i2c_client = kzalloc(sizeof(
        struct msm_camera_i2c_client), GFP_KERNEL);
    if (!fctrl.flash_i2c_client) {
        pr_err("failed no memory\n");
        return -ENOMEM;
    }
    
    fctrl.flash_i2c_client->cci_client = kzalloc(sizeof(
        struct msm_camera_cci_client), GFP_KERNEL);
    if (!fctrl.flash_i2c_client->cci_client) {
        pr_err("failed no memory\n");
        return -ENOMEM;
    }
        
    fctrl.flash_i2c_client->cci_client->cci_subdev = msm_cci_get_subdev();
    fctrl.flash_i2c_client->cci_client->retries = 2;
    fctrl.flash_i2c_client->cci_client->id_map = 0;
    fctrl.flash_i2c_client->i2c_func_tbl = &msm_led_cci_i2c_func_tbl;
    fctrl.flash_i2c_client->addr_type = MSM_CAMERA_I2C_BYTE_ADDR;

    rc = msm_led_cci_query_ic(&fctrl, of_node);
    if (rc < 0) {
        pr_err("can not find matched flash IC\n");
        return -ENODEV;
    }

//zhaozhengtao modify for 13095 and 14029 exteral led driver 
    if (!strcmp(fctrl.led_info->name, "lm3642")) {
        fctrl.reg_setting = &lm3642_regs;
        pr_err("find a device lm3642");	
    } 
    else if(!strcmp(fctrl.led_info->name, "adp1650")) {
        fctrl.reg_setting = &adp1650_regs;
        pr_err("find a device adp1650");		
    } else {
        pr_err("can't find a known chip\n");
        return -ENODEV;
    }   
    rc = msm_led_cci_create_v4lsubdev(pdev, &fctrl);

    if (rc >= 0)
        rc = sysfs_create_group(&pdev->dev.kobj, &msm_led_cci_attr_group);

#ifdef VENDOR_EDIT
//LiuBin@MtkCamera, 2014/04/14, Add for flash proc
	if (rc >= 0)
    	flash_proc_init(&fctrl);
#endif /* VENDOR_EDIT */
	
    return rc;
}

static int __init msm_led_cci_add_driver(void)
{
	CDBG("called\n");
	return platform_driver_probe(&msm_led_cci_driver,
		msm_led_cci_probe);
}

static struct msm_led_cci_fn_t msm_led_cci_func_tbl = {
	.flash_get_subdev_id = msm_led_cci_get_subdev_id,
	.flash_led_config = msm_led_cci_config,
};

static struct msm_led_cci_ctrl_t fctrl = {
	.func_tbl = &msm_led_cci_func_tbl,
};

module_init(msm_led_cci_add_driver);
MODULE_DESCRIPTION("LED TRIGGER FLASH");
MODULE_LICENSE("GPL v2");

