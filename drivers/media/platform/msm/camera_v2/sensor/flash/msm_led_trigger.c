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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_led_flash.h"

#define FLASH_NAME "camera-led-flash"

/*#define CONFIG_MSMB_CAMERA_DEBUG*/
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

extern int32_t msm_led_torch_create_classdev(
				struct platform_device *pdev, void *data);

static enum flash_type flashtype;
static struct msm_led_flash_ctrl_t fctrl;
#ifdef VENDOR_EDIT
//lingjianing add for blink test
bool blink_state;
int flash_state;
#endif
static int32_t msm_led_trigger_get_subdev_id(struct msm_led_flash_ctrl_t *fctrl,
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

static int32_t msm_led_trigger_config(struct msm_led_flash_ctrl_t *fctrl,
	void *data)
{
	int rc = 0;
	struct msm_camera_led_cfg_t *cfg = (struct msm_camera_led_cfg_t *)data;
	uint32_t i;
	uint32_t curr_l, max_curr_l;
	CDBG("called led_state %d\n", cfg->cfgtype);

	if (!fctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}

	switch (cfg->cfgtype) {
	case MSM_CAMERA_LED_OFF:
		for (i = 0; i < fctrl->num_sources; i++)
			if (fctrl->flash_trigger[i])
				led_trigger_event(fctrl->flash_trigger[i], 0);
		if (fctrl->torch_trigger)
			led_trigger_event(fctrl->torch_trigger, 0);
		break;

	case MSM_CAMERA_LED_LOW:
		if (fctrl->torch_trigger) {
			max_curr_l = fctrl->torch_max_current;
			if (cfg->led_current > 0 &&
					cfg->led_current < max_curr_l) {
				curr_l = cfg->led_current;
			} else {
				curr_l = fctrl->torch_op_current;
				pr_err("LED current clamped to %d\n",
					curr_l);
			}
			led_trigger_event(fctrl->torch_trigger,
				curr_l);
		}
		break;

	case MSM_CAMERA_LED_HIGH:
		if (fctrl->torch_trigger)
			led_trigger_event(fctrl->torch_trigger, 0);
		for (i = 0; i < fctrl->num_sources; i++)
			if (fctrl->flash_trigger[i]) {
				max_curr_l = fctrl->flash_max_current[i];
				if (cfg->led_current > 0 &&
						cfg->led_current < max_curr_l) {
					curr_l = cfg->led_current;
				} else {
					curr_l = fctrl->flash_op_current[i];
					pr_err("LED current clamped to %d\n",
						curr_l);
				}
				led_trigger_event(fctrl->flash_trigger[i],
					curr_l);
			}
		break;

	case MSM_CAMERA_LED_INIT:
	case MSM_CAMERA_LED_RELEASE:
		for (i = 0; i < fctrl->num_sources; i++)
			if (fctrl->flash_trigger[i])
				led_trigger_event(fctrl->flash_trigger[i], 0);
		if (fctrl->torch_trigger)
			led_trigger_event(fctrl->torch_trigger, 0);
		break;

	default:
		rc = -EFAULT;
		break;
	}
	CDBG("flash_set_led_state: return %d\n", rc);
	return rc;
}

#ifdef VENDOR_EDIT
//lingjianing add for blink test
static void msm_led_trigger_test_blink_work(struct work_struct *work)
{
        struct delayed_work *dwork = to_delayed_work(work);
        if(blink_state)
          led_trigger_event(fctrl.torch_trigger,fctrl.torch_op_current);
        else 
         led_trigger_event(fctrl.torch_trigger,0);
        blink_state = !blink_state;
        schedule_delayed_work(dwork, msecs_to_jiffies(1100));
        return;
}
#endif
#ifdef VENDOR_EDIT
//zhangzr add for flashlight test
static ssize_t msm_led_trigger_test_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
    int new_mode = simple_strtoul(buf, NULL, 10);
    uint32_t i;
    uint32_t curr_l;

    if(new_mode == 0)
    {
            printk("close flash called\n");
	     #ifdef VENDOR_EDIT
           //lingjianing add for blink test
	     if (flash_state == 2) 
            cancel_delayed_work_sync(&fctrl.dwork);
	     flash_state = 0;
	     #endif
            for (i = 0; i < fctrl.num_sources; i++)
                if (fctrl.flash_trigger[i])
                    led_trigger_event(fctrl.flash_trigger[i], 0);
            if (fctrl.torch_trigger)
                led_trigger_event(fctrl.torch_trigger, 0);
    }
    else if(new_mode == 1)
    {
            printk("open flash called\n");
	    #ifdef VENDOR_EDIT
           //lingjianing add for blink test	
	    if (flash_state == 2) 
           cancel_delayed_work_sync(&fctrl.dwork);
	    flash_state = 1;
	    #endif
            if (fctrl.torch_trigger) {
                curr_l = fctrl.torch_op_current;
                pr_err("LED current clamped to %d\n",
                	curr_l);
                led_trigger_event(fctrl.torch_trigger,
                curr_l);
            }
    } 
     #ifdef VENDOR_EDIT
       //lingjianing add for blink test
    else if(new_mode == 2)
    {   
         printk("blink called\n");
         if (flash_state == 2) 
         cancel_delayed_work_sync(&fctrl.dwork);
	  blink_state=true;
	  flash_state = 2;
	  INIT_DELAYED_WORK(&fctrl.dwork, msm_led_trigger_test_blink_work);
    	  schedule_delayed_work(&fctrl.dwork, msecs_to_jiffies(50));
     }
    #endif
    return count; 
}

static DEVICE_ATTR(test, 0660,
		   NULL, msm_led_trigger_test_store);

static struct attribute *msm_led_trigger_attributes[] = {
        &dev_attr_test.attr,
        NULL
};

static const struct attribute_group msm_led_trigger_attr_group = {
	.attrs = msm_led_trigger_attributes,
};
//zhangzr add end
#endif

#ifdef VENDOR_EDIT
//LiuBin@Camera, 2014/04/14, Add proc for flash test
#include <linux/proc_fs.h>
static int flash_proc_read(char *page, char **start, off_t off, int count,
   int *eof, void *data)
{
	
	int len = 0;

	if (fctrl.flash_trigger_name == NULL)
	{
		pr_err("flash_trigger_name is NULL \r\n");
		return 0;
	}
	
	len = sprintf(page, fctrl.flash_trigger_name[0]);
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
	int i = 0;
	char buf[8] = {0};
	int new_mode = 0;
	uint32_t curr_l;

	if (len > 8)
		len = 8;

	if (copy_from_user(buf, buff, len)) {
		pr_err("proc write error.\n");
		return -EFAULT;
	}

	new_mode = simple_strtoul(buf, NULL, 10);
	if(new_mode == 0)
    {
		printk("close flash called\n");
		if (flash_state == 2) 
			cancel_delayed_work_sync(&fctrl.dwork);
		flash_state = 0;
		for (i = 0; i < fctrl.num_sources; i++)
		if (fctrl.flash_trigger[i])
		led_trigger_event(fctrl.flash_trigger[i], 0);
		if (fctrl.torch_trigger)
		led_trigger_event(fctrl.torch_trigger, 0);
    }
    else if(new_mode == 1)
    {
		printk("open flash called\n");
	    if (flash_state == 2) 
           cancel_delayed_work_sync(&fctrl.dwork);
	    flash_state = 1;
		if (fctrl.torch_trigger) {
			curr_l = fctrl.torch_op_current;
			pr_err("LED current clamped to %d\n",
				curr_l);
			led_trigger_event(fctrl.torch_trigger,
				curr_l);
		}
    } 
    else if(new_mode == 2)
    {   
		printk("blink called\n");
		if (flash_state == 2) 
			cancel_delayed_work_sync(&fctrl.dwork);
		blink_state=true;
		flash_state = 2;
		INIT_DELAYED_WORK(&fctrl.dwork, msm_led_trigger_test_blink_work);
		schedule_delayed_work(&fctrl.dwork, msecs_to_jiffies(50));
	}
	
	return len;
}

static int flash_proc_init(struct msm_led_flash_ctrl_t *flash_ctl)
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


static const struct of_device_id msm_led_trigger_dt_match[] = {
	{.compatible = "qcom,camera-led-flash"},
	{}
};

MODULE_DEVICE_TABLE(of, msm_led_trigger_dt_match);

static struct platform_driver msm_led_trigger_driver = {
	.driver = {
		.name = FLASH_NAME,
		.owner = THIS_MODULE,
		.of_match_table = msm_led_trigger_dt_match,
	},
};

static int32_t msm_led_trigger_probe(struct platform_device *pdev)
{
	int32_t rc = 0, rc_1 = 0, i = 0;
	struct device_node *of_node = pdev->dev.of_node;
	struct device_node *flash_src_node = NULL;
	uint32_t count = 0;
	struct led_trigger *temp = NULL;

	CDBG("called\n");

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	fctrl.pdev = pdev;
	fctrl.num_sources = 0;

	rc = of_property_read_u32(of_node, "cell-index", &pdev->id);
	if (rc < 0) {
		pr_err("failed\n");
		return -EINVAL;
	}
	CDBG("pdev id %d\n", pdev->id);

	rc = of_property_read_u32(of_node,
			"qcom,flash-type", &flashtype);
	if (rc < 0) {
		pr_err("flash-type: read failed\n");
		return -EINVAL;
	}

	if (of_get_property(of_node, "qcom,flash-source", &count)) {
		count /= sizeof(uint32_t);
		CDBG("count %d\n", count);
		if (count > MAX_LED_TRIGGERS) {
			pr_err("invalid count\n");
			return -EINVAL;
		}
		fctrl.num_sources = count;
		for (i = 0; i < count; i++) {
			flash_src_node = of_parse_phandle(of_node,
				"qcom,flash-source", i);
			if (!flash_src_node) {
				pr_err("flash_src_node NULL\n");
				continue;
			}

			rc = of_property_read_string(flash_src_node,
				"linux,default-trigger",
				&fctrl.flash_trigger_name[i]);
			if (rc < 0) {
				pr_err("default-trigger: read failed\n");
				of_node_put(flash_src_node);
				continue;
			}

			CDBG("default trigger %s\n",
				fctrl.flash_trigger_name[i]);

			if (flashtype == GPIO_FLASH) {
				/* use fake current */
				fctrl.flash_op_current[i] = LED_FULL;
			} else {
				rc = of_property_read_u32(flash_src_node,
					"qcom,current",
					&fctrl.flash_op_current[i]);
				rc_1 = of_property_read_u32(flash_src_node,
					"qcom,max-current",
					&fctrl.flash_max_current[i]);
				if ((rc < 0) || (rc_1 < 0)) {
					pr_err("current: read failed\n");
					of_node_put(flash_src_node);
					continue;
				}
			}

			of_node_put(flash_src_node);

			CDBG("max_current[%d] %d\n",
				i, fctrl.flash_op_current[i]);

			led_trigger_register_simple(fctrl.flash_trigger_name[i],
				&fctrl.flash_trigger[i]);

			if (flashtype == GPIO_FLASH)
				if (fctrl.flash_trigger[i])
					temp = fctrl.flash_trigger[i];
		}

		/* Torch source */
		flash_src_node = of_parse_phandle(of_node, "qcom,torch-source",
			0);
		if (flash_src_node) {
			rc = of_property_read_string(flash_src_node,
				"linux,default-trigger",
				&fctrl.torch_trigger_name);
			if (rc < 0) {
				pr_err("default-trigger: read failed\n");
				goto torch_failed;
			}

			CDBG("default trigger %s\n",
				fctrl.torch_trigger_name);

			if (flashtype == GPIO_FLASH) {
				/* use fake current */
				fctrl.torch_op_current = LED_FULL;
				if (temp)
					fctrl.torch_trigger = temp;
				else
					led_trigger_register_simple(
						fctrl.torch_trigger_name,
						&fctrl.torch_trigger);
			} else {
				rc = of_property_read_u32(flash_src_node,
					"qcom,current",
					&fctrl.torch_op_current);
				rc_1 = of_property_read_u32(flash_src_node,
					"qcom,max-current",
					&fctrl.torch_max_current);

				if ((rc < 0) || (rc_1 < 0)) {
					pr_err("current: read failed\n");
					goto torch_failed;
				}

				CDBG("torch max_current %d\n",
					fctrl.torch_op_current);

				led_trigger_register_simple(
					fctrl.torch_trigger_name,
					&fctrl.torch_trigger);
			}
torch_failed:
			of_node_put(flash_src_node);
		}
	}

	rc = msm_led_flash_create_v4lsubdev(pdev, &fctrl);
	if (!rc)
		msm_led_torch_create_classdev(pdev, &fctrl);

#ifdef VENDOR_EDIT
//zhangzr add for flashlight test
      if (rc >= 0)
        rc = sysfs_create_group(&pdev->dev.kobj, &msm_led_trigger_attr_group);
//zhangzr add end
#endif

#ifdef VENDOR_EDIT
//LiuBin@MtkCamera, 2014/04/14, Add for flash proc
	if (rc >= 0)
    	flash_proc_init(&fctrl);
#endif /* VENDOR_EDIT */

	return rc;
}

static int __init msm_led_trigger_add_driver(void)
{
	CDBG("called\n");
	return platform_driver_probe(&msm_led_trigger_driver,
		msm_led_trigger_probe);
}

static struct msm_flash_fn_t msm_led_trigger_func_tbl = {
	.flash_get_subdev_id = msm_led_trigger_get_subdev_id,
	.flash_led_config = msm_led_trigger_config,
};

static struct msm_led_flash_ctrl_t fctrl = {
	.func_tbl = &msm_led_trigger_func_tbl,
};

module_init(msm_led_trigger_add_driver);
MODULE_DESCRIPTION("LED TRIGGER FLASH");
MODULE_LICENSE("GPL v2");
