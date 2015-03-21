/************************************************************************************
** File: - /android/kernel/drivers/input/touchscreen/synaptic_s3203_13095/synaptics_s3203_13095.c
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description:  
**      touch panel driver for synaptics
**      can change MAX_POINT_NUM value to support multipoint
** Version: 1.0
** Date created: 10:49:46,18/01/2012
** Author: Yixue.Ge@BasicDrv.TP
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
** Tong.han@BasicDrv.TP 1/01/2014 migrating code from 13059(MTK) to 13095(Qcom)
************************************************************************************/
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/hrtimer.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <mach/device_info.h>
#include <mach/oppo_boot_mode.h>
#include <mach/oppo_project.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_FB
	#include <linux/fb.h>
	#include <linux/notifier.h>
#endif
#include "oppo_tp_devices.h"

#include "13095/synaptics_s3202_truly.h"
#include "13095/synaptics_s3202_tpk.h"
#include "13095/synaptics_s3202_g2y.h"
#include "13095/synaptics_s3202_tpk_hx8394a.h"
#include "13095/synaptics_test_rawdate.h"

#include "14033/synaptics_s3203_truly_14033.h"
#include "14033/synaptics_s3203_ofilm_14033.h"
#include "14033/synaptics_s3203_test_rawdata_14033.h"

#include "14027/synaptics_s3203_truly_14027.h"
#include "14027/synaptics_s3203_test_rawdata_14027.h"

#include "14029/synaptics_s3203_tpk_14029.h"
#include "14029/synaptics_s3202_tpk_14029.h"
#include "14029/synaptics_s3202_truly_14029.h"
#include "14029/synaptics_s3202_test_rawdata_14029.h"

#include "14013/synaptics_s3203_truly_14013.h"
#include "14013/synaptics_s3203_ofilm_14013.h"
#include "14013/synaptics_s3203_truly_nitto_14013.h"
#include "14013/synaptics_s3203_ofilm_nitto_14013.h"
#include "14013/synaptics_s3203_test_rawdata_14013.h"

#include "14017/synaptics_s3310_firmware_incell.h"
#include "14017/synaptics_s3310_test_rawdata_14017.h"

/*------------------------------------------------Global Define--------------------------------------------*/

#define VKNUMBER 3
#define TPD_USE_EINT
static int LCD_WIDTH ;
static int LCD_HEIGHT ;

#define TPD_DEVICE "synaptic-rmi-s3203"
//#define KEY_USE

#define SUPPORT_GESTURE
#define RESET_ONESECOND
#define SUPPORT_GLOVES_MODE
#define SUPPORT_TP_SLEEP_MODE

/******************for Red function*****************/
#define CONFIG_SYNAPTIC_RED

/*********************for gesture*******************/
#ifdef SUPPORT_GESTURE
	#define ENABLE_UNICODE  0x40
	#define ENABLE_VEE      0x20
	#define ENABLE_CIRCLE   0x08
	#define ENABLE_SWIPE    0x02
	#define ENABLE_DTAP     0x01

	#define UNICODE_DETECT  0x40
	#define VEE_DETECT      0x20
	#define CIRCLE_DETECT   0x08
	#define SWIPE_DETECT    0x02
	#define DTAP_DETECT     0x01


	#define UnkownGestrue       0
	#define DouTap              1   // double tap
	#define UpVee               2   // V
	#define DownVee             3   // ^
	#define LeftVee             4   // >
	#define RightVee            5   // <
	#define Circle              6   // O
	#define DouSwip             7   // ||
	#define Left2RightSwip      8   // -->
	#define Right2LeftSwip      9   // <--
	#define Up2DownSwip         10  // |v
	#define Down2UpSwip         11  // |^
	#define Mgestrue            12  // M
	#define Wgestrue            13  // W
#endif

/*********************for Debug LOG switch*******************/
#define TPD_ERR(a,arg...) pr_err(TPD_DEVICE ": " a,##arg)	
#define TPDTM_DMESG(a,arg...) printk(TPD_DEVICE ": " a,##arg)

#define TPD_DEBUG(a,arg...)\
	do{\
		if(tp_debug)\
			pr_err(TPD_DEVICE ": " a,##arg);\
	}while(0)	
	
/*---------------------------------------------Global Variable----------------------------------------------*/
static int TP_FW;
static int tp_dev = 1;
static unsigned int tp_debug = 0;
static unsigned int is_suspend = 0;
static int button_map[3];
static int tx_rx_num[2];
static int16_t Rxdata[30][30];
static int16_t delta_baseline[30][30];	
static int TX_NUM;
static int RX_NUM;
static int report_key_point_y=0;
static int tp_probe_ok =0;
static atomic_t is_touch;

static DEFINE_SEMAPHORE(work_sem);
struct manufacture_info tp_info;
static struct synaptics_ts_data *ts_g;
static struct workqueue_struct *synaptics_wq = NULL;
static struct workqueue_struct *speedup_resume_wq = NULL;
static struct proc_dir_entry *prEntry_tp = NULL; 
static struct proc_dir_entry *prEntry_tpreset = NULL;

#ifdef SUPPORT_TP_SLEEP_MODE
	static atomic_t sleep_enable;
#endif

#ifdef SUPPORT_GLOVES_MODE	
	static atomic_t glove_enable;
#endif

#ifdef SUPPORT_GESTURE
static uint32_t clockwise;
static uint32_t gesture;

static atomic_t double_enable;
static int is_gesture_enable = 0;
static struct proc_dir_entry *prEntry_dtap = NULL;
static struct proc_dir_entry *prEntry_coodinate  = NULL; 
/****point position*****/
struct Coordinate {
	uint32_t x;
	uint32_t y;
};
static struct Coordinate Point_start;
static struct Coordinate Point_end;
static struct Coordinate Point_1st;
static struct Coordinate Point_2nd;
static struct Coordinate Point_3rd;
static struct Coordinate Point_4th;
#endif

/*-----------------------------------------Global Registers----------------------------------------------*/
static unsigned short SynaF34DataBase;
static unsigned short SynaF34QueryBase;
static unsigned short SynaF01DataBase;
static unsigned short SynaF01CommandBase;

static unsigned short SynaF34Reflash_BlockNum;
static unsigned short SynaF34Reflash_BlockData;
static unsigned short SynaF34ReflashQuery_BootID;
static unsigned short SynaF34ReflashQuery_FlashPropertyQuery;
static unsigned short SynaF34ReflashQuery_FirmwareBlockSize;
static unsigned short SynaF34ReflashQuery_FirmwareBlockCount;
static unsigned short SynaF34ReflashQuery_ConfigBlockSize;
static unsigned short SynaF34ReflashQuery_ConfigBlockCount;

static unsigned short SynaFirmwareBlockSize;
static unsigned short SynaF34_FlashControl;

static int F11_2D_QUERY_BASE;
static int F11_2D_CMD_BASE;
static int F11_2D_CTRL_BASE;
static int F11_2D_DATA_BASE;

static int F01_RMI_QUERY_BASE;
static int F01_RMI_CMD_BASE;
static int F01_RMI_CTRL_BASE;
static int F01_RMI_DATA_BASE;

static int F34_FLASH_QUERY_BASE;
static int F34_FLASH_CMD_BASE;
static int F34_FLASH_CTRL_BASE;
static int F34_FLASH_DATA_BASE;

static int F51_CUSTOM_QUERY_BASE;
static int F51_CUSTOM_CMD_BASE;
static int F51_CUSTOM_CTRL_BASE;
static int F51_CUSTOM_DATA_BASE;

static int F01_RMI_QUERY11;
static int F01_RMI_DATA01;
static int F01_RMI_CMD00;
static int F01_RMI_CTRL00;
static int F01_RMI_CTRL01;
static int F11_2D_CTRL00;
static int F11_2D_CTRL06;
static int F11_2D_CTRL08;
static int F11_2D_CTRL32;
static int F11_2D_DATA38;
static int F11_2D_DATA39;
static int F11_2D_DATA01;
static int F11_2D_CMD00;
static int F34_FLASH_CTRL00;

static int F51_CUSTOM_CTRL00;
static int F51_CUSTOM_DATA11;

#if TP_TEST_ENABLE
static int F54_ANALOG_QUERY_BASE;//0x73
static int F54_ANALOG_COMMAND_BASE;//0x72
static int F54_ANALOG_CONTROL_BASE;//0x0d
static int F54_ANALOG_DATA_BASE;//0x00
#endif

/***********for example of key event***********/
#ifdef KEY_USE
static int tpd_keys[VKNUMBER][5] = {
	{KEY_MENU,90, 2050,180,100},
	{KEY_HOME,500,2050,180,100},
	{KEY_BACK,855,2050,180,100},
};
#endif

/*------------------------------------------Fuction Declare----------------------------------------------*/
static int synaptics_i2c_suspend(struct device *dev);
static int synaptics_i2c_resume(struct device *dev);
/**************I2C resume && suspend end*********/
static int synaptics_ts_resume(struct device *dev);
static int synaptics_ts_suspend(struct device *dev);
static int synaptics_ts_remove(struct i2c_client *client);
static void speedup_synaptics_resume(struct work_struct *work);
static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id);
static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf);
static ssize_t synaptics_rmi4_baseline_show(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t synaptics_rmi4_vendor_id_show(struct device *dev,struct device_attribute *attr, char *buf);

/**************Added temp PVT remove 14017*********/
static int TP_14017_old=0;
/*
u8 write_14017_old[20]={0x04,0x04,0x04,0x04,0x04,
						 0x04,0x04,0x04,0x04,0x04,
						 0x04,0x04,0x04,0x04,0x04,
						 0x0f,0x0f,0x09,0x00,0x00};
						 */
u8 write_14017_old[20]={0x06,0x07,0x07,0x06,0x06,
						 0x06,0x06,0x06,0x06,0x06,
						 0x06,0x06,0x08,0x07,0x07,
						 0x0f,0x0f,0x09,0x00,0x00};
/**************Added temp PVT remove 14017 end*********/

#ifdef TPD_USE_EINT
static irqreturn_t synaptics_ts_irq_handler(int irq,void *dev_id);
#endif
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,unsigned long event,void *data);
#endif
#ifdef CONFIG_SYNAPTIC_RED
extern void rmidev_remove_device(void);
extern int rmidev_init_device(void);
#endif

/*-------------------------------Using Struct----------------------------------*/
struct point_info {
    int x;
    int raw_x;
    int y;
    int raw_y;
    int z;
};

static const struct i2c_device_id synaptics_ts_id[] = {
	{ TPD_DEVICE, 0 },
	{ }
};

static struct of_device_id synaptic_match_table[] = {
	{ .compatible = "synaptic,s3203",},
	{ },
};

static const struct dev_pm_ops synaptic_pm_ops = {
#ifdef CONFIG_FB
	.suspend = synaptics_i2c_suspend,
	.resume = synaptics_i2c_resume,
#endif
};	

static struct i2c_driver tpd_i2c_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,                         
	.id_table	= synaptics_ts_id,
	.driver = {
//		.owner    = THIS_MODULE,
		.name	= TPD_DEVICE,
		.of_match_table =  synaptic_match_table,
		.pm = &synaptic_pm_ops,
	},
};

struct synaptics_ts_data {
	int irq;
	int irq_gpio;
	int id1_gpio;
	int id2_gpio;
	int id3_gpio;
	int reset_gpio;
	int max_num;
	uint32_t irq_flags;
	uint32_t max_x;
    uint32_t max_y;
	uint32_t btn_state;
    uint32_t pre_finger_state;
	struct input_dev *kpd;
	struct work_struct  work;
	struct work_struct speed_up_work;
	struct i2c_client *client;
	struct input_dev *input_dev;	
	struct hrtimer timer;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif	
	/******power*******/
	struct regulator *vdd_2v8;
	struct regulator *vcc_i2c_1v8;
};

/*Virtual Keys Setting Start*/

struct kobject *syna_properties_kobj;

static struct kobj_attribute qrd_virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys."TPD_DEVICE,
        .mode = S_IRUGO,
    },
    .show = &cap_vk_show,
};

static struct attribute *qrd_properties_attrs[] = {
    &qrd_virtual_keys_attr.attr,
    NULL
};

static struct attribute_group qrd_properties_attr_group = {
    .attrs = qrd_properties_attrs,
};
/*Virtual Keys Setting End*/
static struct device_attribute attrs_oppo[] = {
	__ATTR(baseline_test, 0664,synaptics_rmi4_baseline_show,NULL),
	__ATTR(vendor_id, 0664,synaptics_rmi4_vendor_id_show,NULL),
 };
/*---------------------------------------------Fuction Apply------------------------------------------------*/

static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf){
      /* LEFT: search: CENTER: menu ,home:search 412, RIGHT: BACK */
	return sprintf(buf,
        	__stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":%d:%d:%d:%d"
        ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)   ":%d:%d:%d:%d"
        ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":%d:%d:%d:%d"
        "\n",LCD_WIDTH/6,button_map[2],button_map[0],button_map[1],LCD_WIDTH/2,button_map[2],button_map[0],button_map[1],LCD_WIDTH*5/6,button_map[2],button_map[0],button_map[1]);
}

static int synaptics_tpd_button_init(struct synaptics_ts_data *ts)
{
	int ret = 0;
	ts->kpd = input_allocate_device();
    if (ts->kpd == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_tpd_button_init: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->kpd->name = TPD_DEVICE "-kpd";
    set_bit(EV_KEY, ts->kpd->evbit);
	__set_bit(KEY_MENU, ts->kpd->keybit);
	__set_bit(KEY_HOME, ts->kpd->keybit);
	__set_bit(KEY_BACK, ts->kpd->keybit);
	ts->kpd->id.bustype = BUS_HOST;
    ts->kpd->id.vendor  = 0x0001;
    ts->kpd->id.product = 0x0001;
    ts->kpd->id.version = 0x0100;
	
	if(input_register_device(ts->kpd))
        TPDTM_DMESG("input_register_device failed.(kpd)\n");
    set_bit(EV_KEY, ts->kpd->evbit);
	__set_bit(KEY_MENU, ts->kpd->keybit);
	__set_bit(KEY_HOME, ts->kpd->keybit);
	__set_bit(KEY_BACK, ts->kpd->keybit);
	
	report_key_point_y = ts->max_y*button_map[2]/LCD_HEIGHT;
    syna_properties_kobj = kobject_create_and_add("board_properties", NULL);
    if(syna_properties_kobj)
        ret = sysfs_create_group(syna_properties_kobj,&qrd_properties_attr_group);
    if(!syna_properties_kobj || ret)
		printk("failed to create board_properties\n");	

	err_input_dev_alloc_failed:		
		return ret;
}

static int Dot_report = 0;
static void tpd_down(struct synaptics_ts_data *ts,int raw_x, int raw_y, int x, int y, int p) 
{
    if(ts && ts->input_dev) {
        input_report_key(ts->input_dev, BTN_TOUCH, 1);
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, p);
        input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, (raw_x+raw_y)/2);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
        input_mt_sync(ts->input_dev);
		if(Dot_report == 40) {
			printk("Synaptics:D[%4d %4d %4d]\n", x, y, p);		
			Dot_report=0;
		}else{
			Dot_report++;
		}
    }  
}

static void tpd_up(struct synaptics_ts_data *ts,int raw_x, int raw_y, int x, int y, int p) {	
	if(ts && ts->input_dev) {
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
        input_mt_sync(ts->input_dev);
		printk("U[%4d %4d %4d]\n", x, y, 0);	
    }  
}

#ifdef KEY_USE
static void tpd_button(struct synaptics_ts_data *ts,
    unsigned int x, unsigned int y, unsigned int down) {
    int i;
	if(ts->max_x == 1145){
	    if(down) {
	        for(i=0;i<VKNUMBER;i++) {
	            if(x>=tpd_keys[i][1]&&
	               x<=tpd_keys[i][1]+tpd_keys[i][3] &&
	               y>=tpd_keys[i][2]&&
	               y<=tpd_keys[i][2]+tpd_keys[i][4]&&
	               !(ts->btn_state&(1<<i))) {
	                input_report_key(ts->input_dev, tpd_keys[i][0], 1);
	                ts->btn_state|=(1<<i);
	            }
	        }
	    } else {
	        for(i=0;i<4;i++) {
	            if(ts->btn_state&(1<<i)) {
	                input_report_key(ts->input_dev, tpd_keys[i][0], 0);
	            }
	        }
	        ts->btn_state=0;
	    }
	}
}
#endif

static int tpd_hw_pwron(struct synaptics_ts_data *ts)
{
	int rc;		
	if (regulator_count_voltages(ts->vdd_2v8) > 0) {
		if(is_project(OPPO_14017)){
			rc = regulator_set_voltage(ts->vdd_2v8, 2900000,3100000);
		}else{		
			rc = regulator_set_voltage(ts->vdd_2v8, 2800000,3300000);
		}
		if (rc) {
			dev_err(&ts->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
				return rc;
		}
	}

	if (regulator_count_voltages(ts->vcc_i2c_1v8) > 0) {
		rc = regulator_set_voltage(ts->vcc_i2c_1v8, 1800000,
					   1800000);
		if (rc) {
			dev_err(&ts->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			return rc;
		}
	}	
	
    /***enable the 2v8 power*****/
	rc = regulator_enable(ts->vdd_2v8);
	if (rc) {
		dev_err(&ts->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
			return rc;
	}
	/***should enable the 1v8 power*****/
	msleep(1);
	rc = regulator_enable(ts->vcc_i2c_1v8);
	if (rc) {
		dev_err(&ts->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(ts->vdd_2v8);
			return rc;
	}

	msleep(5);
	if( ts->reset_gpio > 0 )
	{	
		TPD_ERR("synaptics:enable the reset_gpio\n");
		gpio_direction_output(ts->reset_gpio, 1);		
	}
	msleep(200);
	return rc;
}

static int tpd_hw_pwroff(struct synaptics_ts_data *ts)
{ 
	int rc = 0;
	
	if( ts->reset_gpio > 0 ){
		TPD_ERR("synaptics:disable the reset_gpio\n");
		gpio_direction_output(ts->reset_gpio, 0);
	}
		
	rc = regulator_disable(ts->vdd_2v8);
	if (rc) {
		dev_err(&ts->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	msleep(10);	
	rc = regulator_disable(ts->vcc_i2c_1v8);
	if (rc) {
		dev_err(&ts->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		regulator_enable(ts->vdd_2v8);
		return rc;
	}
		
	return rc;
}

static int tpd_power(struct synaptics_ts_data *ts,unsigned int on)
{
	int ret;
	if(on) {
		ret = tpd_hw_pwron(ts);	
	} else {
		ret = tpd_hw_pwroff(ts);
	}		
	return ret;
}

static int synaptics_read_register_map(struct synaptics_ts_data *ts)
{
	uint8_t buf[4];   
	int ret;

	memset(buf, 0, sizeof(buf));
   	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if (ret < 0) {
		TPD_DEBUG("synaptics_read_register_map: failed for page select\n");
		return -1;
	}
	ret = i2c_smbus_read_i2c_block_data(ts->client, 0xDD, 4, &(buf[0x0]));
    F11_2D_QUERY_BASE = buf[0];
	F11_2D_CMD_BASE = buf[1];
	F11_2D_CTRL_BASE = buf[2]; 
	F11_2D_DATA_BASE = buf[3];
	
    printk("F11_2D_QUERY_BASE = %x \n \
		F11_2D_CMD_BASE  = %x \n\
		   F11_2D_CTRL_BASE	= %x \n\
		   F11_2D_DATA_BASE	= %x \n\
		   ",F11_2D_QUERY_BASE,F11_2D_CMD_BASE,F11_2D_CTRL_BASE,F11_2D_DATA_BASE);

	
	ret = i2c_smbus_read_i2c_block_data(ts->client, 0xE3, 4, &(buf[0x0]));    
	F01_RMI_QUERY_BASE = buf[0];
	F01_RMI_CMD_BASE = buf[1];
	F01_RMI_CTRL_BASE = buf[2]; 
	F01_RMI_DATA_BASE = buf[3];
    printk("F01_RMI_QUERY_BASE = %x \n\
          F01_RMI_CMD_BASE  = %x \n\
		    F01_RMI_CTRL_BASE	= %x \n\
		    F01_RMI_DATA_BASE	= %x \n\
		   ",F01_RMI_QUERY_BASE,F01_RMI_CMD_BASE,F01_RMI_CTRL_BASE,F01_RMI_DATA_BASE);


	ret = i2c_smbus_read_i2c_block_data(ts->client, 0xE9, 4, &(buf[0x0]));	  
	F34_FLASH_QUERY_BASE = buf[0];
	F34_FLASH_CMD_BASE = buf[1];
	F34_FLASH_CTRL_BASE = buf[2]; 
	F34_FLASH_DATA_BASE = buf[3];
	printk("F34_FLASH_QUERY_BASE = %x \n\
			  F34_FLASH_CMD_BASE	= %x \n\
				F34_FLASH_CTRL_BASE	= %x \n\
				F34_FLASH_DATA_BASE	= %x \n\
			   ",F34_FLASH_QUERY_BASE,F34_FLASH_CMD_BASE,F34_FLASH_CTRL_BASE,F34_FLASH_DATA_BASE);

	
	F01_RMI_QUERY11 = F11_2D_QUERY_BASE+11;
	F01_RMI_CTRL00 = F01_RMI_CTRL_BASE;
	F01_RMI_CTRL01 = F01_RMI_CTRL_BASE + 1;
	F01_RMI_CMD00 = F01_RMI_CMD_BASE;
	F01_RMI_DATA01 = F01_RMI_DATA_BASE + 1; 
		
	F11_2D_CTRL00 = F11_2D_CTRL_BASE;
	F11_2D_CTRL06 = F11_2D_CTRL_BASE + 6;
	F11_2D_CTRL08 = F11_2D_CTRL_BASE + 8;
	F11_2D_CTRL32 = F11_2D_CTRL_BASE + 15;
	F11_2D_DATA38 = F11_2D_DATA_BASE + 54;
	F11_2D_DATA39 = F11_2D_DATA_BASE + 55;
	F11_2D_DATA01 = F11_2D_DATA_BASE + 2;
	F11_2D_CMD00 = F11_2D_CMD_BASE;
		
	F34_FLASH_CTRL00 = F34_FLASH_CTRL_BASE;
	
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4); 
	if (ret < 0) {
		TPD_DEBUG("synaptics_read_register_map: failed for page select\n");
		return -1;
		}
	ret = i2c_smbus_read_i2c_block_data(ts->client, 0xE9, 4, &(buf[0x0])); 
		   
	F51_CUSTOM_QUERY_BASE = buf[0];
	F51_CUSTOM_CMD_BASE = buf[1];
	F51_CUSTOM_CTRL_BASE = buf[2]; 
	F51_CUSTOM_DATA_BASE = buf[3];
	F51_CUSTOM_CTRL00 = F51_CUSTOM_CTRL_BASE;
	F51_CUSTOM_DATA11 = F51_CUSTOM_DATA_BASE;	
	printk("F51_CUSTOM_QUERY_BASE = %x \n\
				 F51_CUSTOM_CMD_BASE  = %x \n\
			   F51_CUSTOM_CTRL_BASE    = %x \n\
			   F51_CUSTOM_DATA_BASE    = %x \n\
			  ",F51_CUSTOM_QUERY_BASE,F51_CUSTOM_CMD_BASE,F51_CUSTOM_CTRL_BASE,F51_CUSTOM_DATA_BASE);  	
			  
#if TP_TEST_ENABLE	
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x1); 
	if (ret < 0) {
		TPD_DEBUG("synaptics_read_register_map: failed for page select\n");
		return -1;
		}
	ret = i2c_smbus_read_i2c_block_data(ts->client, 0xE9, 4, &(buf[0x0])); 
	  F54_ANALOG_QUERY_BASE = buf[0];
		F54_ANALOG_COMMAND_BASE = buf[1];
		F54_ANALOG_CONTROL_BASE = buf[2];
		F54_ANALOG_DATA_BASE = buf[3];
		printk("F54_QUERY_BASE = %x \n\
					  F54_CMD_BASE  = %x \n\
					F54_CTRL_BASE	= %x \n\
					F54_DATA_BASE	= %x \n\
				   ",F54_ANALOG_QUERY_BASE,F54_ANALOG_COMMAND_BASE ,F54_ANALOG_CONTROL_BASE,F54_ANALOG_DATA_BASE);	
#endif
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 	
	return 0;
}

#ifdef SUPPORT_GESTURE
static int synaptics_enable_interrupt_for_gesture(struct synaptics_ts_data *ts,int enable)
{
	int ret;
	uint8_t status_int;
	uint8_t abs_status_int;
    /* page select = 0x0 */
	printk("%s is called\n",__func__);
	is_gesture_enable = enable;
	
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if(ret < 0) {
		msleep(20);
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
		if(ret<0)
			TPD_DEBUG("%s: select page failed ret = %d\n",__func__, ret);
		return -1;
	}

	ret = i2c_smbus_read_byte_data(ts->client, F11_2D_CTRL00);
	if(ret < 0) {
		TPD_DEBUG("read reg F11_2D_CTRL00 failed\n");
		return -1;
	}
	if(enable) {		
		status_int = (ret & 0xF8) | 0x04;
		/*enable gpio wake system through intterrupt*/		
		enable_irq_wake(ts->client->irq);
		gesture = UnkownGestrue ;
		/*clear interrupt bits for previous touch*/
		ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_DATA01);
		if(ret < 0) {
			TPD_ERR("%s :clear interrupt bits failed\n",__func__);
			return -1;
		}		
	} else {
		status_int = ret & 0xF8;
		/*disable gpio wake system through intterrupt*/	
		disable_irq_wake(ts->client->irq);
	}
	printk("%s:status_int = 0x%x\n", __func__, status_int);
	
	ret = i2c_smbus_write_byte_data(ts->client, F11_2D_CTRL00, status_int);
	if(ret < 0) {
		TPD_ERR("%s: enable or disable\
		    interrupt failed,abs_int =%d\n",__func__,status_int);			
		ret = i2c_smbus_write_byte_data(ts->client, F11_2D_CTRL00, status_int);
		if(ret <0) {
			TPD_ERR("%s: enable or disable\
				interrupt failed,abs_int =%d second!\n",__func__,status_int);	
			return -1;
		}
	}
	TPD_DEBUG("%s gesture enable = %d\n", __func__,enable);

	if(enable) {
		abs_status_int = 0x3f;
		/*clear interrupt bits for previous touch*/
		ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_DATA01);
		if(ret < 0) {
			TPD_DEBUG("%s :clear interrupt bits failed\n",__func__);
			return -1;
		}
	} else {
		abs_status_int = 0x0;
	}	
	
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL01, abs_status_int);
	if(ret < 0) {
		TPD_DEBUG("%s: enable or disable abs \
		    interrupt failed,abs_int =%d\n",__func__,abs_status_int);
		return -1;
	}	
	return 0;	
}

static int synaptics_glove_mode_enable(struct synaptics_ts_data *ts)
{
	int ret;
	/* page select = 0x4 */	
	if(1 == atomic_read(&glove_enable)) {	    
	 /*0x00:enable glove mode,0x02:disable glove mode,*/
	    printk("glove mode enable\n");
		if(is_project(OPPO_14017)){
			ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);  
			if (ret < 0) {
				TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
				goto GLOVE_ENABLE_END;
			}
			ret = i2c_smbus_read_byte_data(ts->client, 0x18);
			TPDTM_DMESG("enable glove  ret is %x ret|0x20 is %x\n",ret,ret|0x20);
			ret = i2c_smbus_write_byte_data(ts->client, 0x18, ret | 0x20);  
			if (ret < 0) {
				TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
				goto GLOVE_ENABLE_END;
			}
			ret = i2c_smbus_read_byte_data(ts->client, 0x19);
			TPDTM_DMESG("enable glove ret is %x ret|0x01 is %x\n",ret,ret|0x01);		
			ret = i2c_smbus_write_byte_data(ts->client, 0x19, ret | 0x01);  
			if (ret < 0) {
				TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
				goto GLOVE_ENABLE_END;
			}
		}else{
			ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4); 
			if (ret < 0) {
				TPD_DEBUG("i2c_smbus_write_byte_data failed for page select\n");
				goto GLOVE_ENABLE_END;
			}
			ret = i2c_smbus_write_byte_data(ts->client, F51_CUSTOM_CTRL00,0x00 ); 
			if (ret < 0) {
				TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
				goto GLOVE_ENABLE_END;
			}
		}
	} else {
		if(is_project(OPPO_14017)){
			ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);  
			if (ret < 0) {
				TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
				goto GLOVE_ENABLE_END;
			}
			ret = i2c_smbus_read_byte_data(ts->client, 0x18);
			ret = i2c_smbus_write_byte_data(ts->client, 0x18, ret & 0xDF);  
			if (ret < 0) {
				TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
				goto GLOVE_ENABLE_END;
			}
			ret = i2c_smbus_read_byte_data(ts->client, 0x19);
			ret = i2c_smbus_write_byte_data(ts->client, 0x19, ret & 0xFE);  
			if (ret < 0) {
				TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
				goto GLOVE_ENABLE_END;
			}			
		}else{
			ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4); 
			if (ret < 0) {
				TPD_DEBUG("i2c_smbus_write_byte_data failed for page select\n");
				goto GLOVE_ENABLE_END;
			}
			printk("glove mode disable\n");
			ret = i2c_smbus_write_byte_data(ts->client, F51_CUSTOM_CTRL00,0x02 ); 
			if (ret < 0) {
				TPD_ERR("i2c_smbus_write_byte_data failed for mode select\n");
				goto GLOVE_ENABLE_END;
			}	
		}
	}		
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
	if (ret < 0) {
		TPD_DEBUG("i2c_smbus_write_byte_data failed for page select\n");
		goto GLOVE_ENABLE_END;
	}	
	
GLOVE_ENABLE_END: 
	return ret;
}

#endif

#ifdef SUPPORT_TP_SLEEP_MODE
static int synaptics_sleep_mode_enable(struct synaptics_ts_data *ts)
{
	int ret;
	/* page select = 0x0 */
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
	if (ret < 0)  {
		TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
		goto SLEEP_ENABLE_END;
	}
	
	if(1 == atomic_read(&sleep_enable)) {	    
	/*0x00:enable glove mode,0x02:disable glove mode,*/
	    TPDTM_DMESG("sleep mode enable\n");
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x01 ); 
		if (ret < 0)  {
			TPD_ERR("i2c_smbus_write_byte_data failed for mode select\n");
			goto SLEEP_ENABLE_END;
		}
	} else {		  
	    TPDTM_DMESG("sleep mode disable\n");
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x80 ); 
		if (ret < 0) {
			TPD_ERR("i2c_smbus_write_byte_data failed for mode select\n");
			goto SLEEP_ENABLE_END;
		}
	}
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
	if (ret < 0) {
		TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
		goto SLEEP_ENABLE_END;
	}
	
SLEEP_ENABLE_END:
	return ret;
}
#endif

#ifdef SUPPORT_GESTURE
static void synaptics_get_coordinate_point(struct synaptics_ts_data *ts)
{
    int ret,i;
	uint8_t coordinate_buf[25] = {0};
	uint16_t trspoint = 0;

	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4); 
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11,
		  8, &(coordinate_buf[0]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 8,
		  8, &(coordinate_buf[8]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 16,
		  8, &(coordinate_buf[16])); 
	ret = i2c_smbus_read_i2c_block_data(ts->client, F51_CUSTOM_DATA11 + 24,
		  1, &(coordinate_buf[24]));
	for(i = 0; i< 23; i += 2) {
		trspoint = coordinate_buf[i]|coordinate_buf[i+1] << 8;
		TPD_DEBUG("synaptics TP read coordinate_point[%d] = %d\n",i,trspoint);
    }
	
	TPD_DEBUG("synaptics TP coordinate_buf = 0x%x\n",coordinate_buf[24]);
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
	Point_start.x = (coordinate_buf[0] | (coordinate_buf[1] << 8)) * LCD_WIDTH/ (ts->max_x);
	Point_start.y = (coordinate_buf[2] | (coordinate_buf[3] << 8)) * LCD_HEIGHT/ (1745);
	Point_end.x   = (coordinate_buf[4] | (coordinate_buf[5] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_end.y   = (coordinate_buf[6] | (coordinate_buf[7] << 8)) * LCD_HEIGHT / (1745);
	Point_1st.x   = (coordinate_buf[8] | (coordinate_buf[9] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_1st.y   = (coordinate_buf[10] | (coordinate_buf[11] << 8)) * LCD_HEIGHT / (1745);
	Point_2nd.x   = (coordinate_buf[12] | (coordinate_buf[13] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_2nd.y   = (coordinate_buf[14] | (coordinate_buf[15] << 8)) * LCD_HEIGHT / (1745);
	Point_3rd.x   = (coordinate_buf[16] | (coordinate_buf[17] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_3rd.y   = (coordinate_buf[18] | (coordinate_buf[19] << 8)) * LCD_HEIGHT / (1745);
	Point_4th.x   = (coordinate_buf[20] | (coordinate_buf[21] << 8)) * LCD_WIDTH / (ts->max_x);
	Point_4th.y   = (coordinate_buf[22] | (coordinate_buf[23] << 8)) * LCD_HEIGHT / (1745);
	TPD_DEBUG("synaptics TP (xStart,yStart)=(%d,%d),(xEnd,yEnd) = (%d,%d)\n",
		    Point_start.x,Point_start.y,Point_end.x,Point_end.y);	
	TPD_DEBUG("synaptics TP (x1,y1) = (%d,%d)(x2,y2) = (%d,%d) \n \
         (x3,y3) = (%d,%d),(x4,y4) = (%d,%d)\n",
			Point_1st.x,Point_1st.x,Point_2nd.x,Point_2nd.y,
			Point_3rd.x,Point_3rd.y,Point_4th.x,Point_4th.y);
	clockwise     = (coordinate_buf[24] & 0x10) ? 1 : 
                                    (coordinate_buf[24] & 0x20) ? 0 : 2; // 1--clockwise, 0--anticlockwise, not circle, report 2
};

static void gesture_judge(struct synaptics_ts_data *ts)
{
      int ret = 0,gesture_sign, regswipe;
      uint8_t gesture_buffer[10];
	  
	  ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
	  
	  gesture_sign = i2c_smbus_read_byte_data(ts->client, F11_2D_DATA38);
	  
	  ret = i2c_smbus_read_i2c_block_data(ts->client,  F11_2D_DATA39, 9, &(gesture_buffer[0]));
	  
	  ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4); 
	  regswipe = i2c_smbus_read_byte_data(ts->client, F51_CUSTOM_DATA11+0x18);
	  TPDTM_DMESG("  gesture_sign = %x, regswipe = %x,gesture_buffer[6] = %x, gesture_buffer[8] = %x\n",gesture_sign,regswipe,gesture_buffer[6],gesture_buffer[8]);
	  
      ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 

//detect the gesture mode
      switch (gesture_sign) {
          case DTAP_DETECT:
              gesture = DouTap;
              break;
          case SWIPE_DETECT:
              gesture =     (regswipe == 0x41) ? Left2RightSwip   :
                            (regswipe == 0x42) ? Right2LeftSwip   :
                            (regswipe == 0x44) ? Up2DownSwip      :
                            (regswipe == 0x48) ? Down2UpSwip      :
                            (regswipe == 0x80) ? DouSwip          :
                             UnkownGestrue;
              break;
          case CIRCLE_DETECT:
              gesture = Circle;
              break;
          case VEE_DETECT:
              gesture =    (gesture_buffer[6] == 0x01) ? DownVee  :
                           (gesture_buffer[6] == 0x02) ? UpVee    :
                           (gesture_buffer[6] == 0x04) ? RightVee :
                           (gesture_buffer[6] == 0x08) ? LeftVee  : 
                            UnkownGestrue;
              break;
          case UNICODE_DETECT:
              gesture =  (gesture_buffer[8] == 0x77) ? Wgestrue :
                         (gesture_buffer[8] == 0x6d) ? Mgestrue :
                         UnkownGestrue;
	   }

		TPDTM_DMESG("detect %s gesture\n", gesture == DouTap ? "double tap" :
                                                        gesture == UpVee ? "up vee" :
                                                        gesture == DownVee ? "down vee" :
                                                        gesture == LeftVee ? "(>)" :
                                                        gesture == RightVee ? "(<)" :
                                                        gesture == Circle ? "circle" :
														gesture == DouSwip ? "(||)" :
                                                        gesture == Left2RightSwip ? "(-->)" :
                                                        gesture == Right2LeftSwip ? "(<--)" :
                                                        gesture == Up2DownSwip ? "up to down |" :
                                                        gesture == Down2UpSwip ? "down to up |" :
                                                        gesture == Mgestrue ? "(M)" :
                                                        gesture == Wgestrue ? "(W)" : "unknown");

        // read the coordinate
        synaptics_get_coordinate_point(ts);
        //report Key to notify
       if(gesture != UnkownGestrue){
           input_report_key(ts->input_dev, KEY_F4, 1);
           input_sync(ts->input_dev);
           input_report_key(ts->input_dev, KEY_F4, 0);
           input_sync(ts->input_dev);
       }
} 
#endif

static int synaptics_read_product_id(struct synaptics_ts_data *ts)
{
	uint8_t buf1[11];
	int ret ;
	
	memset(buf1, 0 , sizeof(buf1));
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if (ret < 0) {
		TPDTM_DMESG("synaptics_read_product_id: failed for page select\n");
		return -1;
	}
	ret = i2c_smbus_read_i2c_block_data(ts->client, F01_RMI_QUERY_BASE+11, 8, &(buf1[0x0]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F01_RMI_QUERY_BASE+19, 2, &(buf1[0x8]));	
	if (ret < 0) {
		TPD_ERR("synaptics_read_product_id: failed to read product info\n");
		return -1;
	}
	TPDTM_DMESG("synaptics product id: %s \n",buf1);
	return 0;
}

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret;
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if (ret < 0) {
		TPD_ERR("init_panel failed for page select\n");
		return -1;
	}
	/*device control: normal operation, configur=1*/\
	if(is_project(OPPO_14017)){
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x84); 
		if (ret < 0) {
			msleep(150);
			ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x84); 
			if (ret < 0) {
				TPD_ERR("%s failed for mode select\n",__func__);
			}
		}
	}else{
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x80); 
		if (ret < 0) {
			msleep(150);
			ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x80); 
			if (ret < 0) {
				TPD_ERR("%s failed for mode select\n",__func__);
			}
		}
	}
    /*enable absolutePosFilter,rezero*/
	return ret;
}

static int synaptics_enable_interrupt(struct synaptics_ts_data *ts,
    int enable)
{
	int ret;
	uint8_t abs_status_int;

	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if(ret < 0) {
		
		TPDTM_DMESG("synaptics_enable_interrupt: select page failed ret = %d\n",
		    ret);
		return -1;
	}
    
	if(enable) {
		abs_status_int = 0x7f;
		/*clear interrupt bits for previous touch*/
		ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_DATA_BASE+1);
		if(ret < 0) {
			TPDTM_DMESG("synaptics_enable_interrupt :clear interrupt bits failed\n");
			return -1;
		}
	} else {
		abs_status_int = 0x0;		
	}	
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00+1, abs_status_int);
	if(ret < 0) {
		TPDTM_DMESG("%s: enable or disable abs \
		    interrupt failed,abs_int =%d\n",__func__,abs_status_int);
		return -1;
	}
	ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_CTRL00+1);
	TPDTM_DMESG("S3202-----------0x5E=%x---------\n",ret);
	return 0;	
}

static void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;
    for (i = 0; i < w_ms; i++){
        for (j = 0; j < 1000; j++){
            udelay(1);
        }
    }
}

static void int_state(struct synaptics_ts_data *ts)
{
    int ret = -1;
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD00,0x01);
	if(ret) {
		TPD_ERR("int_state:cannot  reset touch panel \n");
		return;
	}
	printk("ESD detected!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! init state now!\n");
	delay_qt_ms(250);
	
	/**************Added temp PVT remove 14017*********/
	if(TP_14017_old==1)
	{
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x01); 
		i2c_smbus_write_i2c_block_data(ts->client, 0x3c, 20, write_14017_old);	
		ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);//force update 
		msleep(60);
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
	}
	/**************Added temp PVT remove 14017 end*********/
	
#ifdef SUPPORT_GLOVES_MODE
    synaptics_glove_mode_enable(ts_g);
#endif	

	synaptics_init_panel(ts);
	if (ret < 0) {
		TPD_DEBUG("int_state: control tm1400 to sleep failed\n");
		return;
	}
	ret = synaptics_enable_interrupt(ts, 1);
	if(ret) {
		TPD_DEBUG("int_state:cannot  enable interrupt \n");
		return;
	}

}

//Added for larger than 32 length read!
static int synaptics_rmi4_i2c_read(struct synaptics_ts_data *ts,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf;	
	struct i2c_msg msg[] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.len = 1,
			.buf = &buf,
		},
		{
			.addr = ts->client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		},
	};
	buf = addr & 0xFF;
	for (retry = 0; retry < 2; retry++) {
		if (i2c_transfer(ts->client->adapter, msg, 2) == 2) {
			retval = length;
			break;
		}
		msleep(20);
	}

	if (retry == 2) {
		dev_err(&ts->client->dev,
				"%s: I2C read over retry limit\n",
				__func__);
		retval = -5;
	}
	return retval;
}

static void int_touch_14017(struct synaptics_ts_data *ts)
{   int ret= -1,i=0;
    uint8_t buf[80];
    uint32_t finger_state = 0;
	uint8_t finger_num = 0;
	int F12_2D_DATA01_14017 = 0x06;
    struct point_info points;
    memset(buf,0,sizeof(buf));

	points.x=0;
	points.y=0;
	
	ret = i2c_smbus_read_word_data(ts->client, F12_2D_DATA01_14017+0x1);
	if((ret&0x03FF)!= 0){	
		ret=synaptics_rmi4_i2c_read(ts,F12_2D_DATA01_14017, buf, 80);
		if (ret < 0) {
			TPD_ERR("synaptics_int_touch: i2c_transfer failed\n");
	        return;
		}		
		for(i = 0;i < ts->max_num;i++){ 
			if((buf[i*8]==1)||(buf[i*8]==6)){
	//		pr_err("caven buf[i*8] is %d\n",buf[i*8]);
				points.x = ((buf[i*8+2]&0x0f)<<8) | (buf[i*8+1] & 0xff);
				points.raw_x = buf[i*8+6]&0x0f;
				points.y = ((buf[i*8+4]&0x0f)<<8) | (buf[i*8+3] & 0xff);
				points.raw_y = buf[i*8+7]&0x0f;
				points.z = buf[i*8+5];
				if(points.z > 0){		
					tpd_down(ts,points.raw_x, points.raw_y, points.x, points.y,points.z);		
					finger_num++;
				}
			}
		}
	}else{	
		tpd_up(ts,points.raw_x, points.raw_y, points.x, points.y,points.z);			
	}			
	atomic_set(&is_touch,finger_num);
	input_sync(ts->input_dev);
	ts->pre_finger_state = finger_state;  
}

static void int_touch(struct synaptics_ts_data *ts)
{
    int ret= -1,i=0, j = 0;
    uint8_t buf[5];
    uint32_t finger_state = 0;
	uint8_t finger_num = 0;
    struct point_info points;
    memset(buf,0,sizeof(buf));
    ret = i2c_smbus_read_i2c_block_data(ts->client, F11_2D_DATA_BASE, 3, &(buf[0]));
	if (ret < 0) {
		TPD_ERR("synaptics_int_touch: i2c_transfer failed\n");
        return;
	}
	points.x=0;
	points.y=0;
    finger_state = ((buf[2]&0x0f)<<16)|(buf[1]<<8)|buf[0];	
	for(j = 0;j < ts->max_num;j++){
	    if(finger_state&(0x03<<j*2))
	    finger_num = finger_num+1;
		atomic_set(&is_touch,finger_num);
	}

	if(finger_num > 0) {	
		for(i = 0;i < ts->max_num;i++){
		    ret = i2c_smbus_read_i2c_block_data(ts->client, F11_2D_DATA_BASE + 3 + i*5,
			        5, &(buf[0]));
			if (ret < 0) {
				TPD_ERR("synaptics_int_touch: i2c_transfer failed\n");
	        	return;
			}
			points.x = (buf[0]<<4) | (buf[2] & 0x0f);
			points.raw_x = buf[3]&0x0f;
			points.y = (buf[1]<<4) | ((buf[2] & 0xf0)>>4);
			points.raw_y = (buf[3]&0xf0)>>4;
		    points.z = buf[4];

			if(points.z > 0){
			tpd_down(ts,points.raw_x, points.raw_y, points.x, points.y,points.z);
			}			
		}
	} else {
		tpd_up(ts,points.raw_x, points.raw_y, points.x, points.y,points.z);
	}
			
	input_sync(ts->input_dev);
	ts->pre_finger_state = finger_state;  

#ifdef SUPPORT_GESTURE
	if (is_gesture_enable == 1) {
		gesture_judge(ts);
	}
#endif	
}

static void int_key_report_s3202_3310(struct synaptics_ts_data *ts)
{
    int ret= 0;
	int F1A_0D_DATA00=0x0;
	i2c_smbus_write_byte_data(ts->client, 0xff, 0x2);
	ret = i2c_smbus_read_byte_data(ts->client, F1A_0D_DATA00);
	TPD_DEBUG("caven F1A_0D_DATA00 is 0x%x",ret);
	
	if((ret&0x07)!=0){
		if(ret&0x01)//menu
		{          
			tpd_down(ts,40, 20, ts->max_x/6, report_key_point_y,44);
		}
	
		if(ret&0x02)//home
		{	              
			tpd_down(ts,40, 20, ts->max_x/2, report_key_point_y,44);
		}
	
		if(ret&0x04)//reback
		{				
			tpd_down(ts,40, 20, ts->max_x*5/6, report_key_point_y,44);
		}
	}else{
		if(0 == atomic_read(&is_touch)){		
			TPD_DEBUG("key_up\n");
			tpd_up(ts,0, 0, 0, 0,0);
		}
	}
	input_sync(ts->input_dev);
	i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
}

static void synaptics_ts_work_func(struct work_struct *work)
{
	int ret;
	uint8_t buf[5]; 	
    uint8_t status = 0;
	uint8_t inte = 0;
	uint8_t i2c_err_count = 0;	
	struct synaptics_ts_data *ts = container_of(work,struct synaptics_ts_data, work);
	memset(buf,0,sizeof(buf));
	down(&work_sem);
	if( is_suspend == 1 )
		goto FREE_IRQ;
	ret = i2c_smbus_read_word_data(ts->client, F01_RMI_DATA_BASE);
		
	if (ret < 0) {
		if(ret != -5) {
			TPDTM_DMESG("synaptics_ts_work_func: i2c_transfer failed\n");
			goto FREE_IRQ;
		}
	/*for bug :i2c error when wake up system through gesture*/ 
		while((ret == -5)&&(i2c_err_count<10)) {
			msleep(15);
			ret = i2c_smbus_read_word_data(ts->client, F01_RMI_DATA_BASE);
			i2c_err_count++;
			TPDTM_DMESG("Synaptic:ret == %d and try %d times\n",ret,i2c_err_count);
		}
	}
	status = ret&0xff;
	inte = (ret&0x7f00)>>8;
	TPD_DEBUG("synaptics_ts_work_func,inte = %x, status = %x\n",inte,status);
		
	if(status)	
		int_state(ts);
		
	if(is_project(OPPO_14029)){
		if(inte&0x10){
			if(0 == atomic_read(&is_touch)){
				int_key_report_s3202_3310(ts);
			}
		}
		if(inte&0x04){
			int_touch(ts);
		}
	}else if(is_project(OPPO_14017)){	
		if(inte&0x10){
			if(0 == atomic_read(&is_touch)){
				int_key_report_s3202_3310(ts);
			}
		}
		if(inte&0x04){
			int_touch_14017(ts);
		}	
	}else{
		if(inte&0x04)
			int_touch(ts);	
	}	
	
	FREE_IRQ:
		enable_irq(ts_g->client->irq);
		up(&work_sem);
	return;
}

#ifndef TPD_USE_EINT
static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer,
	struct synaptics_ts_data, timer);
	/* TPDTM_DMESG("synaptics_ts_timer_func\n"); */
	queue_work(synaptics_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}
#else
static irqreturn_t synaptics_ts_irq_handler(int irq,void *dev_id)
{
	disable_irq_nosync(ts_g->client->irq);
	queue_work(synaptics_wq, &ts_g->work);
	return IRQ_HANDLED;
}
#endif

#ifdef SUPPORT_GESTURE
static int tp_double_read_func(char *page, char **start, off_t off, int count, int *eof,  void *data)
{
	int ret = 0;
	printk("double tap enable is: %d\n", atomic_read(&double_enable));
	ret = sprintf(page, "%d\n", atomic_read(&double_enable));
	return ret;
}

static int tp_double_write_func(struct file *file,const char *buffer, unsigned long count,void *data)
{ 
	int ret = 0;
	char buf[10];
	
	if (count > 2) 
		return count;	
	if (copy_from_user( buf, buffer, count)) {
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}	 
	sscanf(buf,"%d",&ret);
	TPDTM_DMESG("tp_double_write_func:buf = %d,ret = %d\n",*buf,ret);
	
	if((ret == 0 )||(ret == 1)){
		if(!is_project(OPPO_14017)){
			atomic_set(&double_enable,ret);
		}else{
			atomic_set(&double_enable,0);		
		}
			
	}
	switch(ret){
		case 0:
			TPDTM_DMESG("tp_guesture_func will be disable\n");
			break;
		case 1:
			TPDTM_DMESG("tp_guesture_func will be enable\n");
			break;
		default:
			TPDTM_DMESG("Please enter 0 or 1 to open or close the double-tap function\n");
	}
	return count;

}

static int coordinate_proc_read_func(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{	
	return sprintf(page, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", gesture,
                   Point_start.x, Point_start.y, Point_end.x, Point_end.y,
                   Point_1st.x, Point_1st.y, Point_2nd.x, Point_2nd.y,
                   Point_3rd.x, Point_3rd.y, Point_4th.x, Point_4th.y,
                   clockwise);
}
#endif

static int tp_glove_read_func(char *page, char **start, off_t off, int count, int *eof,  void *data)
{
	int len = 0;
	printk("glove mode enable is: %d\n", atomic_read(&glove_enable));
	len = sprintf(page, "%d\n", atomic_read(&glove_enable));
	return len;
}

static int tp_glove_write_func(struct file *file,const char *buffer, unsigned long count,void *data)
{
	struct synaptics_ts_data *ts;
	int ret = 0 ;
	char buf[10];
	
	//	down(&work_sem);
	if (count > 10)
		goto GLOVE_ENABLE_END;
	if (copy_from_user( buf, buffer, count)){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);	
		goto GLOVE_ENABLE_END;
	}
	sscanf(buf,"%d",&ret);

	ts = ts_g;
	TPDTM_DMESG("tp_glove_write_func:buf = %d,ret = %d\n",*buf,ret);
	if((ret == 0 )||(ret == 1))	{	
		atomic_set(&glove_enable,ret);
		synaptics_glove_mode_enable(ts);
	}	
	switch(ret)	 {	
		case 0:	
			TPDTM_DMESG("tp_glove_func will be disable\n");
			break;
		case 1:	
			TPDTM_DMESG("tp_glove_func will be enable\n");
			break;		
		default:
			TPDTM_DMESG("Please enter 0 or 1 to open or close the glove function\n");
	}
GLOVE_ENABLE_END:
//	up(&work_sem);
	return count;
}

#ifdef SUPPORT_TP_SLEEP_MODE
static int tp_sleep_read_func(char *page, char **start, off_t off, int count, int *eof,  void *data)
{
    int len = 0;
	printk("sleep mode enable is: %d\n", atomic_read(&sleep_enable));
	len = sprintf(page, "%d\n", atomic_read(&sleep_enable));	
	return len;
}

static int tp_sleep_write_func(struct file *file,const char *buffer, unsigned long count,void *data)
{
    struct synaptics_ts_data *ts; 
    int ret = 0 ;
	char buf[10];
	 
	if (count > 10) 
		return count;	
	if (copy_from_user( buf, buffer, count)) {
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return count;
	}

	sscanf(buf,"%d",&ret);	 
	ts = ts_g;
	TPDTM_DMESG("tp_sleep_write_func:buf = %d,ret = %d\n",*buf,ret);
	if((ret == 0 )||(ret == 1)) {
		atomic_set(&sleep_enable,ret);
		synaptics_sleep_mode_enable(ts);		 
	}
	switch(ret) {
		case 0:
			TPDTM_DMESG("tp_sleep_func will be disable\n");
			break;
		case 1:
			TPDTM_DMESG("tp_sleep_func will be enable\n");
			break;
		default:
			TPDTM_DMESG("Please enter 0 or 1 to open or close the sleep function\n");
	}
	return count;
}
#endif

static ssize_t tp_show(struct device_driver *ddri, char *buf)
{
    uint8_t ret = 0;
	ret = i2c_smbus_read_word_data(ts_g->client, F01_RMI_DATA_BASE);
	if(ret < 0)
		printk("tp_show read i2c err\n");	
	//ret0 = i2c_smbus_read_byte_data(ts_g->client, F01_RMI_DATA01);
	//if(ret0 < 0)
		//printk("tp_show read i2c err\n");
	return sprintf(buf, "0x13=0x%x\n", ret);
}

static ssize_t store_tp(struct device_driver *ddri, const char *buf, size_t count)
{
	int tmp = 0;
	if (1 == sscanf(buf, "%d", &tmp)) {
		tp_debug = tmp;
	}
	else {
		TPDTM_DMESG("invalid content: '%s', length = %d\n", buf, count);
	}	
	return count;
}

#if TP_TEST_ENABLE
static int synaptics_read_register_map_page1(struct synaptics_ts_data *ts)
{
	unsigned char buf[4];
	int ret;
	printk("synaptics_read_register_map_page1 start\n");
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x1); 
	if (ret < 0) {
		TPDTM_DMESG("i2c_smbus_write_byte_data failed for page select\n");
		return -1;
	}
	ret = i2c_smbus_read_i2c_block_data(ts->client, 0xE9, 4, &(buf[0x0]));
	F54_ANALOG_QUERY_BASE = buf[0];
	printk("F54_ANALOG_QUERY_BASE = 0x%x\n",F54_ANALOG_QUERY_BASE);
	F54_ANALOG_COMMAND_BASE = buf[1];
	printk("F54_ANALOG_COMMAND_BASE = 0x%x\n",F54_ANALOG_COMMAND_BASE);
	F54_ANALOG_CONTROL_BASE = buf[2];
	printk("F54_ANALOG_CONTROL_BASE = 0x%x\n",F54_ANALOG_CONTROL_BASE);
	F54_ANALOG_DATA_BASE = buf[3];
	printk("F54_ANALOG_DATA_BASE = 0x%x\n",F54_ANALOG_DATA_BASE);
	return 0;
}

static void checkCMD(void)
{
	int ret;
	int flag_err = 0;
	do {
		delay_qt_ms(10); //wait 10ms
		ret =  i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE);
		flag_err++;
    } while ((ret > 0x00) && (flag_err < 15)); 
	if(ret > 0x00)
		TPD_ERR("checkCMD error ret is %x flag_err is %d\n",ret,flag_err);	
}
#endif

static ssize_t tp_baseline_show(struct device_driver *ddri, char *buf)
{
    int ret = 0;
	int x,y;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0,tmp_h = 0; 
	uint8_t tmp_old;
	uint16_t count = 0;
	if(is_suspend == 1)
	   return count;
	memset(delta_baseline,0,sizeof(delta_baseline));
/*disable irq when read data from IC*/	
    disable_irq_nosync(ts_g->client->irq);		
	synaptics_read_register_map_page1(ts_g);
    down(&work_sem);
	printk("\nstep 1:select report type 0x03 baseline\n");
	 //step 1:check raw capacitance.
	if(is_project(OPPO_14017)){
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x14);//select report type 0x08
		ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27);
		tmp_old = ret&0xff;
	//	printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0xDF));
	//	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27,(tmp_old & 0xDF));
		printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x20));
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27,(tmp_old | 0x20));
		/******write No Relax to 1******/
		ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE);
		tmp_old = ret&0xff;
		printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x01));
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE,(tmp_old | 0x01));		
	}else{
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x03);//select report type 0x03
		if (ret < 0) {
			TPDTM_DMESG("step 1: select report type 0x03 failed \n");
			//return sprintf(buf, "i2c err!");
		}
		ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8);
		tmp_old = ret&0xff;
		printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old & 0xef));
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8,(tmp_old & 0xef));
		ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
		checkCMD();
		TPDTM_DMESG("forbid CBC oK\n");
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE + 81,0X01);
		//Forbid NoiseMitigation F54_ANALOG_CTRL41
	}
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X04);//force F54_ANALOG_CMD00
	checkCMD();
	TPDTM_DMESG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);//Force Cal, F54_ANALOG_CMD00
	checkCMD();
	TPDTM_DMESG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x01);//get report
	checkCMD();
	count = 0;
	for(x = 0;x < TX_NUM; x++) {   	
		printk("\n[%d]",x);
		num_read_chars += sprintf(&(buf[num_read_chars]), "\n[%d]",x);
		for(y = 0; y < RX_NUM; y++){
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h<<8)|tmp_l;
			printk("%d,",delta_baseline[x][y]);	
			num_read_chars += sprintf(&(buf[num_read_chars]), "%d ",delta_baseline[x][y]);
		}
   	}	
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);
    delay_qt_ms(60);
	
	if(is_project(OPPO_14017)){
		ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x00);
		ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CMD00,0x01);
		msleep(250);
	/**************Added temp PVT remove 14017*********/
		if(TP_14017_old==1)
		{
			ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x01); 
			i2c_smbus_write_i2c_block_data(ts_g->client, 0x3c, 20, write_14017_old);	
			ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x04);//force update 
			msleep(60);
			ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x00); 
		}
	/**************Added temp PVT remove 14017 end*********/
		
#ifdef SUPPORT_GLOVES_MODE
		synaptics_glove_mode_enable(ts_g);
#endif	
		synaptics_init_panel(ts_g);	
	}
	
    synaptics_enable_interrupt(ts_g,1);
	enable_irq(ts_g->client->irq);
	up(&work_sem);
	return num_read_chars;
}

static ssize_t tp_baseline_show_with_cbc(struct device_driver *ddri, char *buf)
{ 
    int ret = 0;
	int x,y;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0,tmp_h = 0; 
	uint8_t tmp_old;
	uint16_t count = 0;
	if(is_suspend == 1)
	   return count;
	memset(delta_baseline,0,sizeof(delta_baseline));
/*disable irq when read data from IC*/	
    disable_irq_nosync(ts_g->client->irq);		
	synaptics_read_register_map_page1(ts_g);
    down(&work_sem);
	printk("\nstep 1:select report type 0x03 baseline\n");
	 //step 1:check raw capacitance.
	if(is_project(OPPO_14017)){
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x14);//select report type 0x03
		ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27);
		tmp_old = ret&0xff;
	//	printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0xDF));
	//	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27,(tmp_old & 0xDF));
		printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x20));
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27,(tmp_old | 0x20));
		/******write No Relax to 1******/
		ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE);
		tmp_old = ret&0xff;
		printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x01));
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE,(tmp_old | 0x01));		
	}else{
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x03);//select report type 0x03
		if (ret < 0) {
			TPDTM_DMESG("step 1: select report type 0x03 failed \n");
			//return sprintf(buf, "i2c err!");
		}
		ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8);
		tmp_old = ret&0xff;
		printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x10));
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8,(tmp_old | 0x10));
				ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
		checkCMD();
		TPDTM_DMESG("forbid CBC oK\n");
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE + 81,0X01);
		//Forbid NoiseMitigation F54_ANALOG_CTRL41
	}
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X04);//force F54_ANALOG_CMD00
	checkCMD();
	TPDTM_DMESG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);//Force Cal, F54_ANALOG_CMD00
	checkCMD();
	TPDTM_DMESG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x01);//get report
	checkCMD();
	count = 0;
	for(x = 0;x < TX_NUM; x++) {   	
		printk("\n[%d]",x);
		num_read_chars += sprintf(&(buf[num_read_chars]), "\n[%d]",x);
		for(y = 0; y < RX_NUM; y++){
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y] = (tmp_h<<8)|tmp_l;
			printk("%d,",delta_baseline[x][y]);	
			num_read_chars += sprintf(&(buf[num_read_chars]), "%d ",delta_baseline[x][y]);
		}
   	}
	
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);
    delay_qt_ms(60);
	if(is_project(OPPO_14017)){
		ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x00);
		ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CMD00,0x01);
		msleep(250);
	/**************Added temp PVT remove 14017*********/
		if(TP_14017_old==1)
		{
			ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x01); 
			i2c_smbus_write_i2c_block_data(ts_g->client, 0x3c, 20, write_14017_old);	
			ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x04);//force update 
			msleep(60);
			ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x00); 
		}
	/**************Added temp PVT remove 14017 end*********/
		
#ifdef SUPPORT_GLOVES_MODE
		synaptics_glove_mode_enable(ts_g);
#endif	
		synaptics_init_panel(ts_g);	
	}
    synaptics_enable_interrupt(ts_g,1);
	enable_irq(ts_g->client->irq);
	up(&work_sem);
	return num_read_chars;
}

static ssize_t tp_rawdata_show(struct device_driver *ddri, char *buf)
{    int ret = 0;
	int x,y;
	ssize_t num_read_chars = 0;
	uint8_t tmp_l = 0,tmp_h = 0; 
	uint16_t count = 0;
	if(is_suspend == 1)
	   return count;
	memset(delta_baseline,0,sizeof(delta_baseline));
/*disable irq when read data from IC*/	
    disable_irq_nosync(ts_g->client->irq);		
	synaptics_read_register_map_page1(ts_g);
    down(&work_sem);
	TPDTM_DMESG("\nstep 2:report type2 delta image\n");	
	memset(delta_baseline,0,sizeof(delta_baseline));
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x02);//select report type 0x02
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X01);//get report
	checkCMD();
	count = 0;
	for(x = 0;x < TX_NUM; x++) {
		printk("\n[%d]",x);
		num_read_chars += sprintf(&(buf[num_read_chars]), "\n[%d]",x);
		for(y = 0; y < RX_NUM; y++) {
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_baseline[x][y]= (tmp_h<<8)|tmp_l;       		
			printk("%3d,",delta_baseline[x][y]);
			num_read_chars += sprintf(&(buf[num_read_chars]), "%3d ",delta_baseline[x][y]);
		}	 
	}
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);
    delay_qt_ms(60);
    synaptics_enable_interrupt(ts_g,1);
	enable_irq(ts_g->client->irq);
	up(&work_sem);
	return num_read_chars;
}

static ssize_t tp_delta_store(struct device_driver *ddri,
       const char *buf, size_t count)
{
	  TPDTM_DMESG("tp_test_store is not support\n");
	  return count;
}

static ssize_t synaptics_rmi4_baseline_show_14017(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;		
#if TP_TEST_ENABLE
    int ret = 0;
	uint8_t x,y;
	int tx_datal;
	int16_t baseline_data = 0;
	uint8_t tmp_old = 0;
	uint8_t tmp_l = 0,tmp_h = 0; 
	uint16_t count = 0;
	int error_count = 0;	
	uint8_t buffer[9];
	
	memset(buffer, 0, sizeof(buffer));
    down(&work_sem);	
    disable_irq_nosync(ts_g->client->irq);	
	
	memset(Rxdata,0,sizeof(Rxdata));
	synaptics_read_register_map_page1(ts_g);	
	TPDTM_DMESG("step 1:select report type 0x03\n");
	 //step 1:check raw capacitance.
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x14);//select report type 0x03
	if (ret < 0) {
		TPD_ERR("read_baseline: i2c_smbus_write_byte_data failed \n");
		goto END;
	}	 
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27);
	tmp_old = ret&0xff;
	
	/***********close cbc*********
		printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old & 0xDF));
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27,(tmp_old & 0xDF));
	***********end ***********/
	
	/***********open cbc (because the touch key's baseline only normal when open cbc)*********/
	printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x20));
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27,(tmp_old | 0x20));
	/***********end ***********/
	
	/******write No Relax to 1******/
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE);
	tmp_old = ret&0xff;
	printk("No relax ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x01));
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE,(tmp_old | 0x01));	
		
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
	checkCMD();
	TPDTM_DMESG("forbid CBC oK\n");
	
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);//force F54_ANALOG_CMD00
    checkCMD();
	TPDTM_DMESG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x02);//Force Cal, F54_ANALOG_CMD00
    checkCMD();
	TPDTM_DMESG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x01);//get report
	checkCMD();
	count = 0;	
	for(x = 0;x < TX_NUM; x++) {
		for(y = 0; y < RX_NUM; y++)  {
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			baseline_data = (tmp_h<<8)|tmp_l;
			printk("%d,",baseline_data);
   	
			if((y < (RX_NUM-3))&&(x!=TX_NUM-1)) { 	
			//	pr_err("%d,",baseline_data);
				if((baseline_data < TPK_array_limit_14017[count*2]) || (baseline_data > TPK_array_limit_14017[count*2+1])){
					TPD_ERR("Synaptic tpbaseline_fail;count[%d][%d] =%d ;TPK_array_limit_14017[count*2]=%d,TPK_array_limit_14017[count*2+1]=%d\n ",count*2,(count*2+1),baseline_data,TPK_array_limit_14017[count*2],TPK_array_limit_14017[count*2+1]);
					num_read_chars += sprintf(&(buf[num_read_chars]), "0 baseline_data[%d][%d]=%d[%d,%d]\n",x,y,baseline_data,TPK_array_limit_14017[count*2],TPK_array_limit_14017[count*2+1]);
					error_count++;
					goto END;
				}
			}
			if(TP_14017_old!=1){
				if((x==TX_NUM-1)&&(y>RX_NUM-4)){	
					if((baseline_data < TPK_array_limit_14017[count*2]) || (baseline_data > TPK_array_limit_14017[count*2+1])){
					TPD_ERR("Synaptic tpbaseline_fail;count[%d][%d] =%d ;TPK_array_limit_14017[count*2]=%d,TPK_array_limit_14017[count*2+1]=%d\n ",count*2,(count*2+1),baseline_data,TPK_array_limit_14017[count*2],TPK_array_limit_14017[count*2+1]);
					num_read_chars += sprintf(&(buf[num_read_chars]), "0 baseline_data[%d][%d]=%d[%d,%d]\n",x,y,baseline_data,TPK_array_limit_14017[count*2],TPK_array_limit_14017[count*2+1]);
					error_count++;
					goto END;					
					}
				}
			}
			count++;
		}
		printk("\n");
   	}
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27,tmp_old);
		ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
		checkCMD();
		ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+0x27);
	TPDTM_DMESG("[s3202]tem_new end = %x",ret&0xff);
	
	TPD_ERR("step 2:check TRx-TRx & TRx-Vdd short\n" );
    //step 2 :check tx-to-tx and tx-to-vdd
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,26);//select report type 0x05	 
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x01);//get report
    msleep(100);
    ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x0);	
    tx_datal = i2c_smbus_read_i2c_block_data(ts_g->client,F54_ANALOG_DATA_BASE+3,7,buffer);
	for(x=0;x<7;x++)
	{
		TPD_ERR("Check Trx-trx trx-vdd:buf[%d]=%d",x,buffer[x]);
		if(buffer[x]){
			error_count++;
			goto END;	
		}
	}
	num_read_chars += sprintf(buf, "1");

END:
		
	num_read_chars += sprintf(&(buf[num_read_chars]), "imageid=0x%x,deviceid=0x%x\n",TP_FW,TP_FW);

	num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, error_count?"":"All test passed.");

	TPDTM_DMESG("[s3202]tem_new end = %x",ret&0xff);
	TPDTM_DMESG("4 read F54_ANALOG_CTRL07 is: 0x%x\n",ret);
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);
    delay_qt_ms(60);
    ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x00);
    ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CMD00,0x01);
    msleep(250);
	/**************Added temp PVT remove 14017*********/
	if(TP_14017_old==1)
	{
		ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x01); 
		i2c_smbus_write_i2c_block_data(ts_g->client, 0x3c, 20, write_14017_old);	
		ret = i2c_smbus_write_byte_data(ts_g->client, F54_ANALOG_COMMAND_BASE, 0x04);//force update 
		msleep(60);
		ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x00); 
	}
	/**************Added temp PVT remove 14017 end*********/
		
#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts_g);
#endif	
    synaptics_init_panel(ts_g);
	synaptics_enable_interrupt(ts_g,1);
	enable_irq(ts_g->client->irq);
	printk("\n\nstep5 reset and open irq complete\n");
    up(&work_sem);	
    msleep(50);
#endif
	return num_read_chars;
}

static ssize_t synaptics_rmi4_baseline_show_14029(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	ssize_t num_read_chars = 0;		
#if TP_TEST_ENABLE
    int ret = 0;
	uint8_t x,y;
	int16_t baseline_data = 0;
	uint8_t tmp_old = 0;
	uint8_t tmp_l = 0,tmp_h = 0; 
	uint16_t count = 0;
	int error_count = 0;
	int enable_cbc = 0;
	int16_t *baseline_data_test;
    down(&work_sem);	
    disable_irq_nosync(ts_g->client->irq);	
	
	memset(Rxdata,0,sizeof(Rxdata));
	synaptics_read_register_map_page1(ts_g);	
	TPDTM_DMESG("step 1:select report type 0x03\n");
	 //step 1:check raw capacitance.
TEST_WITH_CBC:
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x03);//select report type 0x03
	if (ret < 0) {
		TPD_ERR("read_baseline: i2c_smbus_write_byte_data failed \n");
		goto END;
	}	 
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8);
	tmp_old = ret&0xff;
	if(enable_cbc){
		printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x10));
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8,(tmp_old & 0x10));		
		if(tp_dev == TP_TPK_3202_OFS)
			baseline_data_test = TPK_array_limit_14029_with_cbc;
		else if(tp_dev == TP_TRULY)
			baseline_data_test = Truly_array_limit_14029_with_cbc;	
		else if(tp_dev == TP_TPK_3202_G2Y)
			baseline_data_test = G2Y_array_limit_14029_with_cbc;	
		TPDTM_DMESG("enable_cbc test baseline\n");		
	}else{	
		printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old & 0xef));
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8,(tmp_old & 0xef));	
		TPDTM_DMESG("disble_cbc test baseline\n");			
		if(tp_dev == TP_TPK_3202_OFS)
			baseline_data_test = TPK_array_limit_14029;
		else if(tp_dev == TP_TRULY)
			baseline_data_test = Truly_array_limit_14029;	
		else if(tp_dev == TP_TPK_3202_G2Y)
			baseline_data_test = G2Y_array_limit_14029;		
	}
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
	checkCMD();
	TPDTM_DMESG("forbid CBC oK\n");
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE + 81,0X01);

	//Forbid NoiseMitigation F54_ANALOG_CTRL41
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X04);//force F54_ANALOG_CMD00
    checkCMD();
	TPDTM_DMESG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x02);//Force Cal, F54_ANALOG_CMD00
    checkCMD();
	TPDTM_DMESG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x01);//get report
	checkCMD();
	count = 0;	
	for(x = 0;x < TX_NUM; x++) {
		for(y = 0; y < RX_NUM; y++)  {
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			baseline_data = (tmp_h<<8)|tmp_l;
			printk("%d,",baseline_data);   			
            if((tp_dev == TP_TPK_3202_G2Y) || (tp_dev == TP_TPK_3202_OFS)|| (tp_dev == TP_TRULY)){
				if((y < (RX_NUM-3))&&(x!=TX_NUM-1))
				{ 	
					if((baseline_data < *(baseline_data_test+count*2)) || (baseline_data > *(baseline_data_test+count*2+1))){
						TPD_ERR("Synaptic tpbaseline_fail;count[%d][%d] =%d ;TPK_array_limit_14029[count*2]=%d,TPK_array_limit_14029[count*2+1]=%d\n ",count*2,(count*2+1),baseline_data,*(baseline_data_test+count*2),*(baseline_data_test+count*2+1));
						num_read_chars += sprintf(&(buf[num_read_chars]), "0 raw data erro baseline_data[%d][%d]=%d[%d,%d]\n",x,y,baseline_data,*(baseline_data_test+count*2),	*(baseline_data_test+count*2+1));
						error_count++;
						goto END;
					} 
				}
				if((x==TX_NUM-1)&&(y>RX_NUM-4))
				{	
					if((baseline_data < *(baseline_data_test+count*2)) || (baseline_data > *(baseline_data_test+count*2+1))){
						TPD_ERR("Synaptic tpbaseline_fail;count[%d][%d] =%d ;TPK_array_limit_14029[count*2]=%d,TPK_array_limit_14029[count*2+1]=%d\n ",count*2,(count*2+1),	baseline_data,*(baseline_data_test+count*2),*(baseline_data_test+count*2+1));
						num_read_chars += sprintf(&(buf[num_read_chars]), "0 raw data erro baseline_data[%d][%d]=%d[%d,%d]\n",x,y,baseline_data,*(baseline_data_test+count*2),*(baseline_data_test+count*2+1));
						error_count++;
						goto END;					
					}
				}
			}				
			count++;
		}
		printk("\n");
   	}	
	if(!enable_cbc){
		enable_cbc = 1;
		TPD_ERR("test cbc baseline again\n");
		goto TEST_WITH_CBC;
	}
	
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8,tmp_old);
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
	checkCMD();
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8);
	TPDTM_DMESG("[s3202]tem_new end = %x",ret&0xff);

END:
		
	num_read_chars += sprintf(&(buf[num_read_chars]), "imageid=0x%x,deviceid=0x%x\n",TP_FW,TP_FW);

	num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, error_count?"":"All test passed.");

    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8,tmp_old);
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
	checkCMD();
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8);
	TPDTM_DMESG("[s3202]tem_new end = %x",ret&0xff);
	TPDTM_DMESG("4 read F54_ANALOG_CTRL07 is: 0x%x\n",ret);
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);
    delay_qt_ms(60);
    ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x00);
    ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CMD00,0x01);
    msleep(150);

#ifdef SUPPORT_GLOVES_MODE
    synaptics_glove_mode_enable(ts_g);
#endif	
    synaptics_init_panel(ts_g);
    synaptics_enable_interrupt(ts_g,1);
	enable_irq(ts_g->client->irq);
	printk("\n\nstep5 reset and open irq complete\n");
    up(&work_sem);	
#endif
	return num_read_chars;

}

static ssize_t synaptics_rmi4_baseline_show_s3203(struct device *dev,
		struct device_attribute *attr, char *buf)	
{

	ssize_t num_read_chars = 0;		
#if TP_TEST_ENABLE
    int ret = 0;
	uint8_t x,y;
	int tx_data = 0;
	int tx_datah;
	int tx_datal;
	int16_t baseline_data = 0;
	uint8_t tmp_old = 0;
	uint8_t tmp_l = 0,tmp_h = 0; 
	uint16_t count = 0;
	int error_count = 0;
	int enable_cbc = 0;
	int16_t *baseline_data_test;
		
    down(&work_sem);	
    disable_irq_nosync(ts_g->client->irq);	
	
	memset(Rxdata,0,sizeof(Rxdata));
	synaptics_read_register_map_page1(ts_g);	
	TPDTM_DMESG("step 1:select report type 0x03\n");
	 //step 1:check raw capacitance.
TEST_WITH_CBC_3203:
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x03);//select report type 0x03
	if (ret < 0) {
		TPD_ERR("read_baseline: i2c_smbus_write_byte_data failed \n");
		goto END;
	}	 
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8);
	tmp_old = ret&0xff;
	if(enable_cbc){
		printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old | 0x10));
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8,(tmp_old | 0x10));
	}else{
		printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,(tmp_old & 0xef));
		ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8,(tmp_old & 0xef));	
	}
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
	checkCMD();
	TPDTM_DMESG("forbid CBC oK\n");
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE + 81,0X01);

	//Forbid NoiseMitigation F54_ANALOG_CTRL41
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X04);//force F54_ANALOG_CMD00
    checkCMD();
	TPDTM_DMESG("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x02);//Force Cal, F54_ANALOG_CMD00
    checkCMD();
	TPDTM_DMESG("Force Cal oK\n");
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x01);//get report
	checkCMD();
	count = 0;	
	if(is_project(OPPO_13095))
	{
		if(!enable_cbc){
			if(tp_dev  == TP_TRULY) {
				baseline_data_test = Truly_array_limit;
			}else if ((tp_dev == TP_TPK)||(tp_dev == TP_TPK_HX8394)){
				baseline_data_test = TPK_array_limit;
			}else if (tp_dev == TP_G2Y){
				baseline_data_test = G2Y_array_limit;
			}
		}else{
			if(tp_dev  == TP_TRULY) {
				baseline_data_test = Truly_array_limit_with_cbc;
			}else if ((tp_dev == TP_TPK)||(tp_dev == TP_TPK_HX8394)){
				baseline_data_test = TPK_array_limit_with_cbc;
			}else if (tp_dev == TP_G2Y){
				baseline_data_test = G2Y_array_limit_with_cbc;
			}	
		}
	}else if(is_project(OPPO_14033)) {
	    if(!enable_cbc){
			if(tp_dev  == TP_TRULY) {
				baseline_data_test = Truly_array_limit_14033;
			} else if (tp_dev == TP_OFILM){
				baseline_data_test = Ofilm_array_limit_14033;
			}	
		}else{	
		   if(tp_dev  == TP_TRULY) {
				baseline_data_test = Truly_array_limit_14033_with_cbc;
			} else if (tp_dev == TP_OFILM){
				baseline_data_test = Ofilm_array_limit_14033_with_cbc;
			}	
		}			
	}else if(is_project(OPPO_14013)){
	if(!enable_cbc){
		if(tp_dev  == TP_TRULY) {
			baseline_data_test =  Truly_array_limit_14013;
		}else if (tp_dev == TP_OFILM){
			baseline_data_test = Ofilm_array_limit_14013;
		}else if (tp_dev == TP_TRULY_NITTO){
			baseline_data_test = Truly_nitto_array_limit_14013;
		}else if (tp_dev == TP_OFILM_NITTO){
			baseline_data_test = Ofilm_nitto_array_limit_14013;
		}else if (tp_dev == TP_OFILM_HG){
			baseline_data_test = Ofilm_hg_array_limit_14013;
		}
	}else{
	    if(tp_dev  == TP_TRULY) {
			baseline_data_test =  Truly_array_limit_14013_with_cbc;
		}else if (tp_dev == TP_OFILM){
			baseline_data_test = Ofilm_array_limit_14013_with_cbc;
		}else if (tp_dev == TP_TRULY_NITTO){
			baseline_data_test = Truly_nitto_array_limit_14013_with_cbc;
		}else if (tp_dev == TP_OFILM_NITTO){
			baseline_data_test = Ofilm_nitto_array_limit_14013_with_cbc;
		}else if (tp_dev == TP_OFILM_HG){
			baseline_data_test = Ofilm_hg_array_limit_14013_with_cbc;
		}
	}
    			
	}else if(is_project(OPPO_14027)){
		if(!enable_cbc){
			if(tp_dev  == TP_TRULY) {
				baseline_data_test =  Truly_array_limit_14027;
			}else if(tp_dev  == TP_OFILM) {
				baseline_data_test =  Oflim_array_limit_14027;
			}
		}else{
			if(tp_dev  == TP_TRULY) {
				baseline_data_test =  Truly_array_limit_14027_with_cbc;
			}else if(tp_dev  == TP_OFILM) {
				baseline_data_test =  Oflim_array_limit_14027_with_cbc;
			}
		}
	}
	for(x = 0;x < TX_NUM; x++) {
		for(y = 0; y < RX_NUM; y++)  {
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			baseline_data = (tmp_h<<8)|tmp_l;
			printk("%d,",baseline_data);					
			if((baseline_data < *(baseline_data_test+count*2)) || (baseline_data > *(baseline_data_test+count*2+1))){
					TPD_ERR("Synaptic tpbaseline_fail;count[%d][%d] =%d ;Array_limit[count*2]=%d,Array_limit[count*2+1]=%d\n ",count*2,(count*2+1),baseline_data,*(baseline_data_test+count*2),*(baseline_data_test+count*2+1));
					num_read_chars += sprintf(&(buf[num_read_chars]), "0 baseline_data[%d][%d]=%d[%d,%d]\n",x,y,baseline_data,*(baseline_data_test+count*2),*(baseline_data_test+count*2+1));
					error_count++;
					goto END;
			}	          			
			count++;
		}
	  printk("\n");
   	}	
	if(!enable_cbc){
		enable_cbc = 1;
		TPD_ERR("test cbc baseline again\n");
		if(is_project(OPPO_14027)||is_project(OPPO_14029)||is_project(OPPO_13095)||is_project(OPPO_14013)||is_project(OPPO_14033))
			goto TEST_WITH_CBC_3203;
	}
	
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8,tmp_old);
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
	checkCMD();
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8);
	TPDTM_DMESG("[s3202]tem_new end = %x",ret&0xff);
	
	TPDTM_DMESG("step 2:check tx-to-tx and tx-to-vdd\n" );
    //step 2 :check tx-to-tx and tx-to-vdd
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x05);//select report type 0x05	 
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x01);//get report
    checkCMD();
    ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x0);	
    tx_datal = i2c_smbus_read_word_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
    tx_datah = i2c_smbus_read_word_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
	tx_data = tx_datal | tx_datah<<16;
    if( tx_data!= 0) {
		TPD_ERR("Step 2 error.\n");
		num_read_chars += sprintf(buf, "0 tx-tx-short or tx-vdd-short");
		error_count++;
		goto END;
	}

  TPDTM_DMESG("step 3 :check rx-to-rx\n" );
 //step 3 :check rx-to-rx
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,7);//select report type 0x07
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE + 81,0X01); //forbid NoisMitigation 
	  //Forbid NoiseMitigation F54_ANALOG_CTRL41
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);//force F54_ANALOG_CMD00
    checkCMD();                               //
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x02);//Force Cal,F54_ANALOG_CMD00
    checkCMD();
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x01);//get report
    checkCMD();
    ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE);//read report
	TPDTM_DMESG("F54_ANALOG_CMD00[2]=%d \n",ret);

	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0);
	for(x = 0;x < TX_NUM; x++) {
		for(y = 0; y < RX_NUM; y++) {
			ret= i2c_smbus_read_word_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			Rxdata[x][y] = ret&0xffff;
		}
	}
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,17);//select report type 0x17 
    ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0);
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X01);//get report
    checkCMD();
	for(x = 0;x < RX_NUM - TX_NUM; x++) {
		printk("Rxdata[%d][%d]:",x+TX_NUM,y);   
		for(y = 0; y < RX_NUM; y++) {
			ret= i2c_smbus_read_word_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			 Rxdata[x + TX_NUM][y] = ret&0xffff;	 
		     printk("%5d",Rxdata[x + TX_NUM][y]);   
		}
		printk("\n");
	}
	
	TPDTM_DMESG("\nstep 4:check rx-rx short\n");
//step 4:check rx-rx short
	if(is_project(OPPO_13095))
	{
		for(x = 0;x < RX_NUM; x++) {
			for(y = 0; y < RX_NUM; y++) {
				if ((x ==y)&&(x!=22)) {
					printk("check Rx-to-Rx and Rx-to-vdd_error,Rxdata[%d]=%d \n",x,Rxdata[x][y]);
					if((Rxdata[x][y] < DiagonalLowerLimit)|| (Rxdata[x][y] >DiagonalUpperLimit)) {
						num_read_chars += sprintf(buf, "0 rx-to-rx short or rx-to-vdd short Rxdata[%d][%d] = %d",x,y,Rxdata[x][y]);
						printk("Synaptic check Rx-to-Rx and Rx-to-vdd_error,Rxdata[%d]=%d \n",x,Rxdata[x][y]);
						error_count++;
						goto END;
					}	
				}
			}
		}
	}
	else if(is_project(OPPO_14033))
	{
		for(x = 0;x < RX_NUM; x++) {
			for(y = 0; y < RX_NUM; y++) {
				if ((x ==y)&&(x!=0)) {
					printk("check Rx-to-Rx and Rx-to-vdd_error,Rxdata[%d]=%d \n",x,Rxdata[x][y]);
					if((Rxdata[x][y] < DiagonalLowerLimit)|| (Rxdata[x][y] >DiagonalUpperLimit)) {
						num_read_chars += sprintf(buf, "0 rx-to-rx short or rx-to-vdd short Rxdata[%d][%d] = %d",x,y,Rxdata[x][y]);
						printk("Synaptic check Rx-to-Rx and Rx-to-vdd_error,Rxdata[%d]=%d \n",x,Rxdata[x][y]);
						error_count++;
						goto END;
					}	
				}
			}
		}
	}
	else if(is_project(OPPO_14027))
	{
		for(x = 0;x < RX_NUM; x++) {
			for(y = 0; y < RX_NUM; y++) {
				if ((x ==y)&&(x!=22)) {
					printk("check Rx-to-Rx and Rx-to-vdd_error,Rxdata[%d]=%d \n",x,Rxdata[x][y]);
					if((Rxdata[x][y] < DiagonalLowerLimit)|| (Rxdata[x][y] >DiagonalUpperLimit)) {
						num_read_chars += sprintf(buf, "0 rx-to-rx short or rx-to-vdd short Rxdata[%d][%d] = %d",x,y,Rxdata[x][y]);
						printk("Synaptic check Rx-to-Rx and Rx-to-vdd_error,Rxdata[%d]=%d \n",x,Rxdata[x][y]);
						error_count++;
						goto END;
					}	
				}
			}
		}
	}
	else
	{	
	   for(x = 0;x < RX_NUM; x++) {
			for(y = 0; y < RX_NUM; y++) {
				if ((x ==y)) {
					printk("check Rx-to-Rx and Rx-to-vdd_error,Rxdata[%d]=%d \n",x,Rxdata[x][y]);
					if((Rxdata[x][y] < DiagonalLowerLimit)|| (Rxdata[x][y] >DiagonalUpperLimit)) {
						num_read_chars += sprintf(buf, "0 rx-to-rx short or rx-to-vdd short Rxdata[%d][%d] = %d",x,y,Rxdata[x][y]);
						printk("Synaptic check Rx-to-Rx and Rx-to-vdd_error,Rxdata[%d]=%d \n",x,Rxdata[x][y]);
						error_count++;
						goto END;
					}	
				}
			}
		}
	
	}
	num_read_chars += sprintf(buf, "1");

END:
		
	num_read_chars += sprintf(&(buf[num_read_chars]), "imageid=0x%x,deviceid=0x%x\n",TP_FW,TP_FW);

	num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, error_count?"":"All test passed.");

    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8,tmp_old);
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
	checkCMD();
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8);
	TPDTM_DMESG("[s3202]tem_new end = %x",ret&0xff);
	TPDTM_DMESG("4 read F54_ANALOG_CTRL07 is: 0x%x\n",ret);
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);
    delay_qt_ms(60);
    ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x00);
    ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CMD00,0x01);
    msleep(150);

#ifdef SUPPORT_GLOVES_MODE
    synaptics_glove_mode_enable(ts_g);
#endif	

    synaptics_init_panel(ts_g);
    synaptics_enable_interrupt(ts_g,1);
	enable_irq(ts_g->client->irq);
	printk("\n\nstep5 reset and open irq complete\n");
    up(&work_sem);	
#endif
	return num_read_chars;
}

static ssize_t synaptics_rmi4_baseline_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	
	if(is_project(OPPO_14017))
		return synaptics_rmi4_baseline_show_14017(dev,attr,buf);		
	else if(is_project(OPPO_14029))
		return synaptics_rmi4_baseline_show_14029(dev,attr,buf);
	else
		return synaptics_rmi4_baseline_show_s3203(dev,attr,buf);
 
}


static ssize_t tp_test_store(struct device_driver *ddri,
       const char *buf, size_t count)
{
   TPDTM_DMESG("tp_test_store is not support\n");
   return count;
}

static ssize_t synaptics_rmi4_vendor_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{	
	if((tp_dev == TP_G2Y)||(tp_dev == TP_TPK)||(tp_dev == TP_TPK_3202_G2Y) || (tp_dev == TP_TPK_3202_OFS))
		return sprintf(buf, "%d\n",TP_TPK); 
	if(tp_dev == TP_TRULY)
		return sprintf(buf, "%d\n",TP_TRULY); 
    if(tp_dev == TP_OFILM)
		return sprintf(buf, "%d\n",TP_OFILM);	
	
	return sprintf(buf, "%d\n",tp_dev); 
}

static DRIVER_ATTR(oppo_tp_baseline_image, 0664, tp_baseline_show, tp_delta_store);

static DRIVER_ATTR(oppo_tp_baseline_image_with_cbc, 0664, tp_baseline_show_with_cbc, tp_test_store);

static DRIVER_ATTR(oppo_tp_rawdata_image, 0664, tp_rawdata_show, NULL);

static DRIVER_ATTR(oppo_tp_debug, 0664, tp_show, store_tp);

static int tp_write_func (struct file *file,const char *buffer,
    unsigned long count,void *data);
	
			
static int init_synaptics_proc(void)
{
	int ret = 0;		
	prEntry_tp = proc_mkdir("touchpanel", NULL);
	if(prEntry_tp == NULL) {
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create TP proc entry\n");
	}		
	if(!is_project(OPPO_14017)){
		#ifdef SUPPORT_GESTURE
			prEntry_dtap = create_proc_entry( "double_tap_enable", 0666, prEntry_tp );
			if(prEntry_dtap == NULL){
				ret = -ENOMEM;
				printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
			}else{
				prEntry_dtap->write_proc = tp_double_write_func;
				prEntry_dtap->read_proc =  tp_double_read_func;
			}		
			prEntry_coodinate = create_proc_entry("coordinate", 0444, prEntry_tp);
			if(prEntry_coodinate == NULL){	   
				ret = -ENOMEM;	   
				printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
			}else{
				prEntry_coodinate->read_proc = coordinate_proc_read_func;
			}
		#endif	 
		#ifdef SUPPORT_GLOVES_MODE
			prEntry_dtap = create_proc_entry( "glove_mode_enable", 0666, prEntry_tp );
			if(prEntry_dtap == NULL) {
				ret = -ENOMEM;
				printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
			}else{
				prEntry_dtap->write_proc = tp_glove_write_func;
				prEntry_dtap->read_proc =  tp_glove_read_func;
			}
		#endif	
	}		
#ifdef SUPPORT_TP_SLEEP_MODE
		prEntry_dtap = create_proc_entry("sleep_mode_enable", 0666, prEntry_tp);
		if(prEntry_dtap == NULL){	   
			ret = -ENOMEM;	   
			printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
		}else{
			prEntry_dtap->write_proc = tp_sleep_write_func;
			prEntry_dtap->read_proc = tp_sleep_read_func;
		}
#endif			
#ifdef RESET_ONESECOND
	prEntry_tpreset = create_proc_entry( "tp_reset", 0666, prEntry_tp );
	if(prEntry_tpreset == NULL){
		ret = -ENOMEM;
		printk(KERN_INFO"init_synaptics_proc: Couldn't create tp reset proc entry\n");
	}else{
		prEntry_tpreset->write_proc = tp_write_func;
	}
#endif
	return ret;
}

static int tp_write_func (struct file *file,const char *buffer, unsigned long count,void *data)
{
	struct synaptics_ts_data *ts; 
	int ret = 0;	
	if(ts_g){
		ts = ts_g;
		if(is_suspend)
		    return count;
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD00, 0x01);
		ret = i2c_smbus_write_byte_data(ts->client,F01_RMI_CMD00+1, 0x01);
		if(ret)	{
			TPD_ERR("%s: rezero error!\n",__func__);
			return count;
		}
		TPDTM_DMESG("%s: rezero/reset successed!\n",__func__);
	}
	return count;
}

/****************************S3310*****update**********************************/
#define SYNAPTICS_RMI4_PRODUCT_ID_SIZE 10
#define SYNAPTICS_RMI4_PRODUCT_INFO_SIZE 2

static void re_scan_PDT_s3310(struct i2c_client *client)
{
	  uint8_t buf[8];
	  i2c_smbus_read_i2c_block_data(client, 0xE9, 6,  buf);
	  SynaF34DataBase = buf[3];
	  SynaF34QueryBase = buf[0];
	  i2c_smbus_read_i2c_block_data(client, 0xE3, 6,  buf);
	  SynaF01DataBase = buf[3];
	  SynaF01CommandBase = buf[1];
	  i2c_smbus_read_i2c_block_data(client, 0xDD, 6,  buf);
	  SynaF34Reflash_BlockNum = SynaF34DataBase;
	  SynaF34Reflash_BlockData = SynaF34DataBase + 1;
	  SynaF34ReflashQuery_BootID = SynaF34QueryBase;
	  SynaF34ReflashQuery_FlashPropertyQuery = SynaF34QueryBase + 1;
	  SynaF34ReflashQuery_FirmwareBlockSize = SynaF34QueryBase + 2;
	  SynaF34ReflashQuery_FirmwareBlockCount = SynaF34QueryBase +3;
	  SynaF34ReflashQuery_ConfigBlockSize = SynaF34QueryBase + 3;
	  SynaF34ReflashQuery_ConfigBlockCount = SynaF34QueryBase + 3;
	  i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_FirmwareBlockSize,2, buf);
	  SynaFirmwareBlockSize = buf[0] | (buf[1] << 8);
	  printk("SynaFirmwareBlockSize 3310 is %d\n",SynaFirmwareBlockSize);
	  SynaF34_FlashControl = SynaF34DataBase + 2;
}

struct image_header {
	/* 0x00 - 0x0f */
	unsigned char checksum[4];
	unsigned char reserved_04;
	unsigned char reserved_05;
	unsigned char options_firmware_id:1;
	unsigned char options_contain_bootloader:1;
	unsigned char options_reserved:6;
	unsigned char bootloader_version;
	unsigned char firmware_size[4];
	unsigned char config_size[4];
	/* 0x10 - 0x1f */
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE];
	unsigned char package_id[2];
	unsigned char package_id_revision[2];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
	/* 0x20 - 0x2f */
	unsigned char reserved_20_2f[16];
	/* 0x30 - 0x3f */
	unsigned char ds_id[16];
	/* 0x40 - 0x4f */
	unsigned char ds_info[10];
	unsigned char reserved_4a_4f[6];
	/* 0x50 - 0x53 */
	unsigned char firmware_id[4];
};

struct image_header_data {
	bool contains_firmware_id;
	unsigned int firmware_id;
	unsigned int checksum;
	unsigned int firmware_size;
	unsigned int config_size;
	unsigned char bootloader_version;
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
};

static unsigned int extract_uint_le(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] +
			(unsigned int)ptr[1] * 0x100 +
			(unsigned int)ptr[2] * 0x10000 +
			(unsigned int)ptr[3] * 0x1000000;
}

static void parse_header(struct image_header_data *header,
		const unsigned char *fw_image)
{
	struct image_header *data = (struct image_header *)fw_image;

	header->checksum = extract_uint_le(data->checksum);
	TPD_ERR(" debug checksume is %x",header->checksum);
	header->bootloader_version = data->bootloader_version;
	TPD_ERR(" debug bootloader_version is %d",header->bootloader_version);

	header->firmware_size = extract_uint_le(data->firmware_size);
	TPD_ERR(" debug firmware_size is %x",header->firmware_size);

	header->config_size = extract_uint_le(data->config_size);
	TPD_ERR(" debug header->config_size is %x",header->config_size);

	memcpy(header->product_id, data->product_id, sizeof(data->product_id));
	header->product_id[sizeof(data->product_id)] = 0;

	memcpy(header->product_info, data->product_info,
			sizeof(data->product_info));

	header->contains_firmware_id = data->options_firmware_id;
	TPD_ERR(" debug header->contains_firmware_id is %x",header->contains_firmware_id);
	if (header->contains_firmware_id)
		header->firmware_id = extract_uint_le(data->firmware_id);

	return;
}

//Added by Tong.han@BSP.group.TP for speeding up 14017's firmware-update
static int checkFlashState(void)
{
	int ret ;
	int count = 0;
	ret =  i2c_smbus_read_byte_data(ts_g->client,SynaF34_FlashControl+1);
	while ((ret != 0x80)&&(count<5))
	{
		msleep(3); //wait 3ms
		ret =  i2c_smbus_read_byte_data(ts_g->client,SynaF34_FlashControl+1);
		count++;
    } 
	if(count == 5)
		return 1;
	else
		return 0;
}

static int synatpitcs_ts_update_s3310(struct i2c_client *client)
{
    uint16_t block,firmware,configuration;
    uint8_t buf[8];
	uint8_t bootloder_id[10];
	struct image_header_data header;
    int ret,j;
	int do_lockdown= 0;
	parse_header(&header,Syna_Firmware_Data_14017);
	i2c_smbus_write_byte_data(client, 0xff, 0x0);	
	re_scan_PDT_s3310(client);
    block = 16;
    TPDTM_DMESG("block is %d \n",block);
    firmware = (header.firmware_size)/16;
    TPDTM_DMESG("firmware is %d \n",firmware);
    configuration = (header.config_size)/16;
    TPDTM_DMESG("configuration is %d \n",configuration);
	ret = i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_BootID, 8, &(bootloder_id[0]));  
/*	
    data_low8 = i2c_smbus_read_byte_data(client, SynaF34ReflashQuery_BootID);
    data_high8 = i2c_smbus_read_byte_data(client,SynaF34ReflashQuery_BootID+1);*/
    TPDTM_DMESG("bootloader id is %x \n",(bootloder_id[1] << 8)|bootloder_id[0]);
	/*
    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData,data_low8);
    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+1,data_high8);*/
	ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
    TPDTM_DMESG("Write bootloader id SynaF34_FlashControl is 0x00%x ret is %d\n",SynaF34_FlashControl,ret);
    i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x0F);
    msleep(2000);
    TPDTM_DMESG("attn step 4\n");
    ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl+1);
    TPDTM_DMESG("The status(enter flash) is %x\n",ret);
    ret = i2c_smbus_read_byte_data(client,0x04);
    TPDTM_DMESG("The status(device state) is %x\n",ret);
    ret= i2c_smbus_read_byte_data(client,F01_RMI_CTRL_BASE);
    TPDTM_DMESG("The status(control f01_RMI_CTRL_DATA) is %x\n",ret);
    ret= i2c_smbus_write_byte_data(client,F01_RMI_CTRL_BASE,ret&0x04);
	/********************get in prog end************/
	
	ret=i2c_smbus_write_i2c_block_data(client, SynaF34Reflash_BlockData, 2, &(bootloder_id[0x0]));
	printk("ret is %d\n",ret);	
	
	
	re_scan_PDT_s3310(client);
    i2c_smbus_read_i2c_block_data(client,SynaF34ReflashQuery_BootID,2,buf);
    i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,2,buf);
    i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x03);
    msleep(2000);
    ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl+1);
    TPDTM_DMESG("The status(erase) is %x\n",ret);
	if(do_lockdown)
	{
		TPDTM_DMESG("Lockdown:TP is going to do lockdown\n");
		for(j=0;j<5;j++)
		{
			buf[0]=j&0x00ff;
			buf[1]=(j&0xff00)>>8;
			i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockNum,2,buf);
			i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,16,&Syna_Firmware_Data_14017[j*16+0xb0]); 
			i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x04);
			ret=checkFlashState();
			if(ret == 1)
				TPD_ERR("Lockdown:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
		//	ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl+1);
		//	TPDTM_DMESG("Lockdown:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
		}	
	}
    for(j=0;j<firmware;j++)
    {
        buf[0]=j&0x00ff;
        buf[1]=(j&0xff00)>>8;
        i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockNum,2,buf);
        i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,16,&Syna_Firmware_Data_14017[j*16+0x100]); 
		i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x02);
		ret=checkFlashState();
		if(ret == 1)
			TPD_ERR("Firmware:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
	}
    //step 7 configure data
    for(j=0;j<configuration;j++)
    {
        //a)write SynaF34Reflash_BlockNum to access
        buf[0]=j&0x00ff;
        buf[1]=(j&0xff00)>>8;
        i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockNum,2,buf);
        //b) write data
			i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,16,&Syna_Firmware_Data_14017[j*16+0x100+0xee00]);
		//c) issue write
        i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x06);
        //d) wait attn
		ret=checkFlashState();
		if(ret == 1)
			TPD_ERR("Configure:The status(Image) of flash data3 is %x,time =%d\n",ret,j);
    }
	//step 1 issue reset
    i2c_smbus_write_byte_data(client,SynaF01CommandBase,0X01);
    //step2 wait ATTN
    //delay_qt_ms(1000);
    mdelay(1000);
    //step 3 check status
    ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl+1);
    TPDTM_DMESG("The status(disable)of flash data3 : %0x\n %s normal End\n",ret,__func__);	
	return 0;
}
/****************************S3310*****end!!!!!**********************************/

static void re_scan_PDT(struct i2c_client *client)
{
	  uint8_t buf[8];
	  i2c_smbus_read_i2c_block_data(client, 0xE9, 6,  buf);
	  SynaF34DataBase = buf[3];
	  SynaF34QueryBase = buf[0];
	  i2c_smbus_read_i2c_block_data(client, 0xE3, 6,  buf);
	  SynaF01DataBase = buf[3];
	  SynaF01CommandBase = buf[1];
	  i2c_smbus_read_i2c_block_data(client, 0xDD, 6,  buf);
	  SynaF34Reflash_BlockNum = SynaF34DataBase;
	  SynaF34Reflash_BlockData = SynaF34DataBase + 2;
	  SynaF34ReflashQuery_BootID = SynaF34QueryBase;
	  SynaF34ReflashQuery_FlashPropertyQuery = SynaF34QueryBase + 2;
	  SynaF34ReflashQuery_FirmwareBlockSize = SynaF34QueryBase + 3;
	  SynaF34ReflashQuery_FirmwareBlockCount = SynaF34QueryBase +5;
	  SynaF34ReflashQuery_ConfigBlockSize = SynaF34QueryBase + 3;
	  SynaF34ReflashQuery_ConfigBlockCount = SynaF34QueryBase + 7;
	  i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_FirmwareBlockSize,2, buf);
	  SynaFirmwareBlockSize = buf[0] | (buf[1] << 8);
	  SynaF34_FlashControl = SynaF34DataBase + SynaFirmwareBlockSize + 2;
}

static int synatpitcs_ts_update(struct i2c_client *client)
{
    uint8_t data_low8,data_high8;
    uint16_t block,firmware,configuration;
    uint8_t buf[8];
    int ret,i,j;
    i2c_smbus_write_byte_data(client, 0xff, 0x0);	
	re_scan_PDT(client);
    block = 16;
    TPDTM_DMESG("block is %d \n",block);
    firmware = 2816;
    TPDTM_DMESG("firmware is %d \n",firmware);
    configuration = 32;
    TPDTM_DMESG("configuration is %d \n",configuration);
    data_low8 = i2c_smbus_read_byte_data(client, SynaF34ReflashQuery_BootID);
    data_high8 = i2c_smbus_read_byte_data(client,SynaF34ReflashQuery_BootID+1);
    TPDTM_DMESG("bootloader id is %x \n",(data_high8 << 8)|data_low8);
    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData,data_low8);
    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+1,data_high8);
    TPDTM_DMESG("Write bootloader id\n");
    i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x0F);
    msleep(10);
    TPDTM_DMESG("attn step 4\n");
    ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
    TPDTM_DMESG("The status(enter flash) is %x\n",ret);
	re_scan_PDT(client);
    i2c_smbus_read_i2c_block_data(client,SynaF34ReflashQuery_BootID,2,buf);
    i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,2,buf);
    i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x03);
    msleep(1000);
    ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
    TPDTM_DMESG("The status(erase) is %x\n",ret);
    for(j=0;j<firmware;j++)
    {
        buf[0]=j&0x00ff;
        buf[1]=(j&0xff00)>>8;
        i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockNum,2,buf);
        for(i=0;i<block;i++) {	

          
		if(is_project(OPPO_13095))
		{
            if(tp_dev == TP_TRULY)			   
            	i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
			      						 TRULY_Firmware_Data[j*16+i]); 
			else if(tp_dev == TP_TPK)			   
                i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
			      						 TPK_Firmware_Data[j*16+i]); 
			else if(tp_dev == TP_TPK_HX8394)			   
                i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
			      						 TPK_hx8394_Firmware_Data[j*16+i]); 
			else if(tp_dev == TP_G2Y)			   
                i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
			      						 G2Y_Firmware_Data[j*16+i]);

		}
		else if(is_project(OPPO_14033)) 
		{
            if(tp_dev == TP_TRULY)			   
            	i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
			      						 TRULY_Firmware_Data_14033[j*16+i]); 
			if(tp_dev == TP_OFILM)			   
                i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
	    	      						OFILM_Firmware_Data_14033[j*16+i]); 
		}
		else if(is_project(OPPO_14027)) 
		{        
		//	if(tp_dev == TP_TRULY)
                i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
	    	      						TRULY_Firmware_Data_14027[j*16+i]); 
		}
			else if(is_project(OPPO_14013)) 
		{
            if(tp_dev == TP_TRULY)			   
            	i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
			      						 TRULY_Firmware_Data_14013[j*16+i]); 
			if(tp_dev == TP_OFILM)			   
                i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
	    	      						OFILM_Firmware_Data_14013[j*16+i]);
            if(tp_dev == TP_TRULY_NITTO)			   
            	i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
			      						 TRULY_NITTO_Firmware_Data_14013[j*16+i]); 
			if(tp_dev == TP_OFILM_NITTO)			   
                i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
	    	      						OFILM_NITTO_Firmware_Data_14013[j*16+i]);
		    if(tp_dev == TP_OFILM_HG)			   
                i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
	    	      						OFILM_Firmware_Data_14013[j*16+i]);
										
		}
			else if(is_project(OPPO_14029)) 
		{
            if((tp_dev == TP_TPK_3202_G2Y) || (tp_dev == TP_TPK_3202_OFS))
            	i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
			      						 TPK_Firmware_Data_14029_3202[j*16+i]);   
			else if(tp_dev == TP_TRULY)			   
            	i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
			      						 TRULY_Firmware_Data_14029_3202[j*16+i]);	
			else if(tp_dev == TP_TPK_3203)			   
            	i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
			      						 TPK_Firmware_Data_14029_3203[j*16+i]);								 
			
		}
	}
		
        i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x02);
        msleep(5);
        ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
		if(ret != 0x80)
           TPDTM_DMESG("config The status(Image) of flash data3 is %x,time =%d\n",ret,j);
	}
    //step 7 configure data
    for(j=0;j<configuration;j++)
    {
        //a)write SynaF34Reflash_BlockNum to access
        buf[0]=j&0x00ff;
        buf[1]=(j&0xff00)>>8;
        i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockNum,2,buf);
        //b) write data
	    for(i=0;i<block;i++) {
		
		
		if(is_project(OPPO_13095)) 
		{
			if(tp_dev == TP_TRULY)
			    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,TRULY_Config_Data[j*16+i]);
			
			else if(tp_dev == TP_TPK)
			    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,TPK_Config_Data[j*16+i]);
			
			else if(tp_dev == TP_TPK_HX8394)			   
			    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,TPK_hx8394_Config_Data[j*16+i]);
				
			else if(tp_dev == TP_G2Y)
			    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,G2Y_Config_Data[j*16+i]);
		}
		else if (is_project(OPPO_14033)) 
		{
		    if(tp_dev == TP_TRULY)
			    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,TRULY_Config_Data_14033[j*16+i]);
			if(tp_dev == TP_OFILM)
			    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,OFILM_Config_Data_14033[j*16+i]);
	    }	
		else if(is_project(OPPO_14027)) 
		{        
		//	if(tp_dev == TP_TRULY)
                i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
	    	      						TRULY_Config_Data_14027[j*16+i]); 
		}	
        else if(is_project(OPPO_14013)) 
		{        
			if(tp_dev == TP_TRULY)
			    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,TRULY_Config_Data_14013[j*16+i]);
			if(tp_dev == TP_OFILM)
			    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,OFILM_Config_Data_14013[j*16+i]); 
			if(tp_dev == TP_TRULY_NITTO)
			    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,TRULY_NITTO_Config_Data_14013[j*16+i]);
			if(tp_dev == TP_OFILM_NITTO)
			    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,OFILM_NITTO_Config_Data_14013[j*16+i]);	
			if(tp_dev == TP_OFILM_HG)
			    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,OFILM_Config_Data_14013[j*16+i]);	
		}
		 else if(is_project(OPPO_14029)) 
		{        
            if((tp_dev == TP_TPK_3202_G2Y) || (tp_dev == TP_TPK_3202_OFS))
			    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,TPK_Config_Data_14029_3202[j*16+i]);
			if(tp_dev == TP_TRULY)
			    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,TRULY_Config_Data_14029_3202[j*16+i]);	
			if(tp_dev == TP_TPK_3203)
			    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,TPK_Config_Data_14029_3203[j*16+i]);	
				
		}
	}	
	
        //c) issue write
        i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x06);
        //d) wait attn
        msleep(5);
        ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
		if(ret != 0x80)
           TPDTM_DMESG("config The status(Image) of flash data3 is %x,time =%d\n",ret,j);
    }
	//step 1 issue reset
    i2c_smbus_write_byte_data(client,SynaF01CommandBase,0X01);
    //step2 wait ATTN
    //delay_qt_ms(1000);
    mdelay(1000);
    //step 3 check status
    ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
    TPDTM_DMESG("The status(disable)of flash data3 : %0x\n %s normal End\n",ret,__func__);	
	return 0;
}

static  void get_tp_id(int TP_ID1,int TP_ID2,int TP_ID3)
{
	int ret,id1 = -1,id2 = -1,id3 = -1;
	if(TP_ID1  >= 0)
	{
		ret = gpio_request(TP_ID1,"TP_ID1");
		if(!ret){
			ret = gpio_tlmm_config(GPIO_CFG(
					TP_ID1, 0,
					GPIO_CFG_INPUT,
					GPIO_CFG_PULL_UP,
					GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
		}
		msleep(100);
	   id1=gpio_get_value(TP_ID1);
	}
	
	if(TP_ID2  >=  0)
	{
		ret = gpio_request(TP_ID2,"TP_ID2");
		if(!ret){
		ret = gpio_tlmm_config(GPIO_CFG(
				TP_ID2, 0,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_2MA),
				GPIO_CFG_ENABLE);
		msleep(100);	
		id2=gpio_get_value(TP_ID2);		
		}
	}
	
	if(TP_ID3  >= 0)
	{
		ret = gpio_request(TP_ID3,"TP_ID3");
		if(!ret){
			ret = gpio_tlmm_config(GPIO_CFG(
					TP_ID3, 0,
					GPIO_CFG_INPUT,
					GPIO_CFG_PULL_UP,
					GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
		}	
		msleep(100);		
		id3=gpio_get_value(TP_ID3);
	}

	TPDTM_DMESG("%s::id1:%d id2:%d id3:%d\n",__func__,id1,id2,id3);	
	
	
	if(is_project(OPPO_13095))
	{	
		if((id1==0)&&(id2==0)&&(id3==0)) {
			TPDTM_DMESG("%s::TPK \n",__func__);
			tp_dev=TP_TPK;
			gpio_tlmm_config(GPIO_CFG(
					22, 0,
					GPIO_CFG_INPUT,
					GPIO_CFG_PULL_UP,
					GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
			gpio_tlmm_config(GPIO_CFG(
					108, 0,
					GPIO_CFG_INPUT,
					GPIO_CFG_PULL_UP,
					GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);
			msleep(100);	
			if(gpio_get_value(22)&&gpio_get_value(108))
				tp_dev= TP_TPK_HX8394;
		} else if((id1==1)&&(id2==0)&&(id3==1)) {
			TPDTM_DMESG("%s::G2Y \n",__func__);
			tp_dev=TP_G2Y;
		}else if((id1==0)&&(id2==0)&&(id3==1)) {
			TPDTM_DMESG("%s::TRULY \n",__func__);
			tp_dev=TP_TRULY;
		}else{		
			TPDTM_DMESG("%s::TP_UNKNOWN\n",__func__);
			tp_dev=TP_TPK;
		}
	}
	else  if(is_project(OPPO_14033))
    {
        if((id1==0)&&(id2==0)&&(id3==1)) {
			TPDTM_DMESG("%s::OFILM\n",__func__);
			tp_dev=TP_OFILM;
		}else if((id1==0)&&(id2==0)&&(id3==0)) {
			TPDTM_DMESG("%s::TRULY \n",__func__);
			tp_dev=TP_TRULY;
		}else{		
			TPDTM_DMESG("%s::TP_UNKNOWN\n",__func__);
			tp_dev=TP_TRULY;
		}
	}
	else if(is_project(OPPO_14017))
		tp_dev=TP_TPK;
	else if(is_project(OPPO_14027)){
		if((id1 == 0)&&(id2 == 0)&&(id3 == 0)){
			tp_dev= TP_TRULY;
			tp_info.manufacture = "TRULY-BLACK";
		}else if((id1 == 1)&&(id2 == 0)&&(id3 == 0)){
			tp_dev= TP_TRULY;
			tp_info.manufacture = "TRULY-WHITE";
		}else if((id1 == 0)&&(id2 == 0)&&(id3 == 1)){
			tp_dev= TP_OFILM;
			tp_info.manufacture = "OFLIM";
		}else{
			TPD_ERR("UNKNOWN TP\n");
			tp_dev= TP_TRULY;	
			tp_info.manufacture = "UNKNOWN";			
		}		
	}
	else if(is_project(OPPO_14013))
	{ 
		if((id1==0)&&(id2==0)&&(id3==0)) {
			TPDTM_DMESG("%s::TRULY\n",__func__);
			tp_dev=TP_TRULY;
		}else if((id1 == 1)&&(id2 == 1)&&(id3 == 0)){
			tp_dev= TP_TRULY_NITTO;
		}else if((id1 == 1)&&(id2 == 0)&&(id3 == 0)){
			tp_dev= TP_OFILM_NITTO;
		}else if((id1 == 0)&&(id2 == 1)&&(id3 == 1)){
			tp_dev= TP_OFILM_HG;
		}
		else {
			TPDTM_DMESG("%s::OFILM \n",__func__);
			tp_dev=TP_OFILM;		
	   }   
	 }
	else if(is_project(OPPO_14029))
	{
        if((id1==0)&&(id2==0)) {
			TPDTM_DMESG("%s::TPK_s3203\n",__func__);
			tp_dev=TP_TPK_3203;
		}else if((id1==1)&&(id2==0)) {
			TPDTM_DMESG("%s::TPK_G2Y_s3202 \n",__func__);
			tp_dev=TP_TPK_3202_G2Y;
		}else if((id1==0)&&(id2==1)) {
			TPDTM_DMESG("%s::TPK_s3202 \n",__func__);
			tp_dev=TP_TPK_3202_OFS;
		}else if((id1==1)&&(id2==1)) {
			TPDTM_DMESG("%s::TRULY_s3202 \n",__func__);
			tp_dev=TP_TRULY;
		}else{		
			TPDTM_DMESG("%s::TP_UNKNOWN\n",__func__);
			tp_dev=TP_TPK_3202_G2Y;
		}	
    }		
	
}

static int synaptics_parse_dts(struct device *dev,struct synaptics_ts_data *ts)
{
	int rc;
	struct device_node *np;
	int lcd_size[2];
	np = dev->of_node;
	ts->irq_gpio = of_get_named_gpio_flags(np, "synaptics,irq-gpio",0, &(ts->irq_flags));
	if(ts->irq_gpio < 0 ){
		TPD_ERR("ts->irq_gpio not specified\n");	
	}
	ts->id1_gpio = of_get_named_gpio(np, "synaptics,id1-gpio",0);
	if(ts->id1_gpio < 0 ){
		TPD_ERR("ts->id1_gpio not specified\n");	
	}
	ts->id2_gpio = of_get_named_gpio(np, "synaptics,id2-gpio",0);
	if(ts->id2_gpio < 0 ){
		TPD_ERR("ts->id2_gpio not specified\n");	
	}
	ts->id3_gpio = of_get_named_gpio(np, "synaptics,id3-gpio",0);	
	if(ts->id3_gpio < 0 ){
		TPD_ERR("ts->id3_gpio not specified\n");	
	}
	ts->reset_gpio = of_get_named_gpio(np, "synaptics,reset-gpio",0);	
	if(ts->reset_gpio < 0 ){
		TPD_ERR("ts->reset-gpio not specified\n");	
	}
	rc = of_property_read_u32(np, "synaptics,max-num-support", &ts->max_num);
	if(rc){
		TPD_ERR("ts->max_num not specified\n");
		ts->max_num = 10;
	}
	rc = of_property_read_u32(np, "synaptics,max-y-point", &ts->max_y);
	if(rc){
		TPD_ERR("ts->max_y not specified\n");
		ts->max_num = 10;
	}
	rc = of_property_read_u32_array(np,"synaptics,button-map", button_map,3);
	if(rc){
		TPD_ERR("button-map not specified\n");
		button_map[0] = 160;
		button_map[1] = 226;
		button_map[2] = 1400;
	}
	TPD_ERR("synaptics:button map readed is %d %d %d\n",button_map[0],button_map[1],button_map[2]);
	
	rc = of_property_read_u32_array(np,"synaptics,tx-rx-num", tx_rx_num,2);
	if(rc){
		TPD_ERR("button-map not specified\n");
		TX_NUM =  13;	
	    RX_NUM =  23;	
	}else{
        TX_NUM =  tx_rx_num[0];	
	    RX_NUM =  tx_rx_num[1];	
	}
	TPD_ERR("synaptics,tx-rx-num is %d %d \n",TX_NUM,RX_NUM);	
	
	rc = of_property_read_u32_array(np,"synaptics,display-coords", lcd_size,2);
	if(rc){
		TPD_ERR("lcd size not specified\n");
		LCD_WIDTH = 720;
		LCD_HEIGHT = 1280;
	}else{
		LCD_WIDTH = lcd_size[0];
		LCD_HEIGHT = lcd_size[1];	
	}
	TPDTM_DMESG("synaptic:ts->irq_gpio:%d irq_flags:%u id1_gpio:%d id2_gpio:%d id3_gpio:%d max_num %d\n"
				,ts->irq_gpio, ts->irq_flags ,ts->id1_gpio ,ts->id2_gpio ,ts->id3_gpio,ts->max_num);
				
/***********power regulator_get****************/
	ts->vdd_2v8 = regulator_get(&ts->client->dev, "vdd_2v8");
	if (IS_ERR(ts->vdd_2v8)) {
		rc = PTR_ERR(ts->vdd_2v8);
		dev_err(&ts->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
	}	
	ts->vcc_i2c_1v8 = regulator_get(&ts->client->dev, "vcc_i2c_1v8");
	if (IS_ERR(ts->vcc_i2c_1v8)) {
		rc = PTR_ERR(ts->vcc_i2c_1v8);
		dev_err(&ts->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
	}
	if(ts->reset_gpio  > 0){
		if (gpio_is_valid(ts->reset_gpio)) {
			rc = gpio_request(ts->reset_gpio,"rmi4-reset-gpio");
			if (rc) {
				TPD_ERR("unable to request gpio [%d]\n",ts->irq_gpio);
			}
		}
	}
	return rc;
}

static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	int ret = 0;
	int rc;
	int attr_count=0;
	uint8_t buf[4];
	uint32_t CURRENT_FIRMWARE_ID = 0;
	uint32_t bootloader_mode;
    uint32_t FIRMWARE_ID = 0;

	uint8_t fw_cnt = 0;
	int boot_mode = 0;
	int max_y_ic = 0;
//Added for device list	
	static char temp[12];
	
	atomic_set(&is_touch,0);
#ifdef SUPPORT_GESTURE
	atomic_set(&double_enable,0);
#endif
#ifdef SUPPORT_GLOVES_MODE
	atomic_set(&glove_enable,0);
#endif
	boot_mode =get_boot_mode();
	printk("synaptic system boot_mode is %d\n",boot_mode);
	

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	memset(ts,0,sizeof(*ts));	
	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts_g = ts;
	printk("Synaptic:%s is called\n",__func__);

	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
    if (!synaptics_wq) {
        return -ENOMEM;
    }   	
	speedup_resume_wq = create_singlethread_workqueue("speedup_resume_wq");
	if (!speedup_resume_wq) {
        return -ENOMEM;
    }   
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	INIT_WORK(&ts->speed_up_work,speedup_synaptics_resume);
	
#ifdef SUPPORT_GESTURE
	is_gesture_enable = 0;
#endif
/*********parse DTS***********/
	synaptics_parse_dts(&client->dev,ts);
/***power_init*****/
	ret = tpd_power(ts,1);	
	if (ret<0)
		TPD_ERR("regulator_enable is called\n");		
	if((boot_mode == MSM_BOOT_MODE__FACTORY ||  boot_mode == MSM_BOOT_MODE__RF || boot_mode == MSM_BOOT_MODE__WLAN) ) {
		TPD_ERR("regulator_disable is called\n");
		if( ts->reset_gpio > 0 )
		{				
			TPD_ERR("synaptics:enable the reset_gpio\n");
			gpio_direction_output(ts->reset_gpio, 0);
			msleep(5);
			if(is_project(OPPO_14017))
				return 0;
		}
		rc = regulator_disable(ts_g->vdd_2v8);
		if (rc) {
			TPD_ERR("regulator_disable failed\n");
		}
		msleep(5);
		rc = regulator_disable(ts_g->vdd_2v8);
		if (rc) {
			TPD_ERR("regulator_disable failed\n");
		}
		
		TPD_ERR("synaptics :Not normal boot and return\n");
		return 0;
    }		
/******power_end*********/
		get_tp_id(ts->id1_gpio,ts->id2_gpio,ts->id3_gpio);	
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TPDTM_DMESG("%s: need I2C_FUNC_I2C\n",__func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}	
	init_synaptics_proc();
    ret = i2c_smbus_read_byte_data(client,0x13);
	if(ret < 0) {
		ret = i2c_smbus_read_byte_data(client,0x13);
		if(ret < 0) {
			tpd_power(ts,0);
			printk("synaptics is no exist!\n");
			return 0;
		}
	}
	
	synaptics_read_register_map(ts);
	bootloader_mode = i2c_smbus_read_byte_data(ts->client,F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40; 
	printk("synaptics:before fw update,bootloader_mode = 0x%x\n",bootloader_mode);
	
	i2c_smbus_read_i2c_block_data(ts->client, F34_FLASH_CTRL00, 4, buf); 
	CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	printk("CURRENT_FIRMWARE_ID = 0x%x\n",CURRENT_FIRMWARE_ID);	
	TP_FW = CURRENT_FIRMWARE_ID;		
	
	if(is_project(OPPO_13095))
	{
		if(tp_dev == TP_TRULY) {
			tp_info.manufacture = "TRULY";
			printk("TP_DEV = TP_TRULY\n");
			buf[0]=TRULY_Config_Data[0];
			buf[1]= TRULY_Config_Data[1];
			buf[2]= TRULY_Config_Data[2];
			buf[3]=TRULY_Config_Data[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
			printk("read firmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}else if(tp_dev == TP_TPK) {
			tp_info.manufacture = "TPK-OFS";
			printk("TP_DEV = TP_TPK\n");
			buf[0]=TPK_Config_Data[0];
			buf[1]= TPK_Config_Data[1];
			buf[2]= TPK_Config_Data[2];
			buf[3]=TPK_Config_Data[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];	
			printk("read TPK firmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}else if(tp_dev == TP_TPK_HX8394) {
			tp_info.manufacture = "TPK-HX8394";
			printk("TP_DEV = TP_TPK_HX8394\n");
			buf[0]=TPK_hx8394_Config_Data[0];
			buf[1]= TPK_hx8394_Config_Data[1];
			buf[2]= TPK_hx8394_Config_Data[2];
			buf[3]=TPK_hx8394_Config_Data[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];	
			printk("read TPK firmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}else if(tp_dev == TP_G2Y) {
			tp_info.manufacture = "TPK-G2Y";
			printk("TP_DEV = TP_G2Y\n");
			buf[0]=G2Y_Config_Data[0];
			buf[1]= G2Y_Config_Data[1];
			buf[2]= G2Y_Config_Data[2];
			buf[3]=G2Y_Config_Data[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];	
			printk("read G2Y firmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}		
	
	}
	else if (is_project(OPPO_14033))
	{
	   
	    if(tp_dev == TP_TRULY) {
			tp_info.manufacture = "TRULY";
			printk("TP_DEV = TP_TRULY\n");
			buf[0]=TRULY_Config_Data_14033[0];
			buf[1]= TRULY_Config_Data_14033[1];
			buf[2]= TRULY_Config_Data_14033[2];
			buf[3]=TRULY_Config_Data_14033[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
			printk("read firmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}else if(tp_dev == TP_OFILM) {
			tp_info.manufacture = "OFILM";
			printk("TP_DEV = TP_OFILM\n");
			buf[0]=OFILM_Config_Data_14033[0];
			buf[1]= OFILM_Config_Data_14033[1];
			buf[2]= OFILM_Config_Data_14033[2];
			buf[3]=OFILM_Config_Data_14033[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];	
			printk("read OFILM firmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}			
	}	
	else if(is_project(OPPO_14027))
	{
			printk("TP_DEV = TP_TRULY\n");
			buf[0]=TRULY_Config_Data_14027[0];
			buf[1]= TRULY_Config_Data_14027[1];
			buf[2]= TRULY_Config_Data_14027[2];
			buf[3]=TRULY_Config_Data_14027[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
			printk("read firmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
//		}
	}	
	else if(is_project(OPPO_14013))
	{
	
		if(tp_dev == TP_TRULY) {
			tp_info.manufacture = "TRULY";
			printk("TP_DEV = TP_TRULY\n");
			buf[0]=TRULY_Config_Data_14013[0];
			buf[1]= TRULY_Config_Data_14013[1];
			buf[2]= TRULY_Config_Data_14013[2];
			buf[3]=TRULY_Config_Data_14013[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
			printk("read firmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}else if(tp_dev == TP_OFILM) {
			tp_info.manufacture = "OFILM";
			printk("TP_DEV = TP_OFILM\n");
			buf[0]=OFILM_Config_Data_14013[0];
			buf[1]= OFILM_Config_Data_14013[1];
			buf[2]= OFILM_Config_Data_14013[2];
			buf[3]=OFILM_Config_Data_14013[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];	
			printk("read OFILM firmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}else if(tp_dev == TP_TRULY_NITTO) {
			tp_info.manufacture = "TRULY_NITTO";
			printk("TP_DEV = TP_TRULY_NITTO\n");
			buf[0]=TRULY_NITTO_Config_Data_14013[0];
			buf[1]=TRULY_NITTO_Config_Data_14013[1];
			buf[2]=TRULY_NITTO_Config_Data_14013[2];
			buf[3]=TRULY_NITTO_Config_Data_14013[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];	
			printk("read TRULY_NITTO firmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}else if(tp_dev == TP_OFILM_NITTO) {
			tp_info.manufacture = "OFILM_NITTO";
			printk("TP_DEV = TP_OFILM_NITTO\n");
			buf[0]=OFILM_NITTO_Config_Data_14013[0];
			buf[1]=OFILM_NITTO_Config_Data_14013[1];
			buf[2]=OFILM_NITTO_Config_Data_14013[2];
			buf[3]=OFILM_NITTO_Config_Data_14013[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];	
			printk("read OFILM_NITTOfirmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}else if(tp_dev == TP_OFILM_HG) {
			tp_info.manufacture = "OFILM_HG";
			printk("TP_DEV = TP_OFILM_HG\n");
			buf[0]=OFILM_Config_Data_14013[0];
			buf[1]=OFILM_Config_Data_14013[1];
			buf[2]=OFILM_Config_Data_14013[2];
			buf[3]=OFILM_Config_Data_14013[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];	
			printk("read OFILM HG firmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}	
	}		
    else if(is_project(OPPO_14029))
	{
	
		if(tp_dev == TP_TPK_3202_G2Y) {
			tp_info.manufacture = "TPK_G2Y_s3202";
			printk("TP_DEV = TPK_G2Y_s3202\n");
			buf[0]= TPK_Config_Data_14029_3202[0];
			buf[1]= TPK_Config_Data_14029_3202[1];
			buf[2]= TPK_Config_Data_14029_3202[2];
			buf[3]= TPK_Config_Data_14029_3202[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
			printk("read firmware ID: S3202 CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}else if(tp_dev == TP_TPK_3202_OFS) {
			tp_info.manufacture = "TPK_OFS_s3202";
			printk("TP_DEV = TPK_OFS_s3202\n");
			buf[0]= TPK_Config_Data_14029_3202[0];
			buf[1]= TPK_Config_Data_14029_3202[1];
			buf[2]= TPK_Config_Data_14029_3202[2];
			buf[3]= TPK_Config_Data_14029_3202[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
			printk("read firmware ID: S3202 CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}else if(tp_dev == TP_TRULY) {
			tp_info.manufacture = "TRULY_s3202";
			printk("TP_DEV = TRULY_s3202\n");
			buf[0]= TRULY_Config_Data_14029_3202[0];
			buf[1]= TRULY_Config_Data_14029_3202[1];
			buf[2]= TRULY_Config_Data_14029_3202[2];
			buf[3]= TRULY_Config_Data_14029_3202[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
			printk("read firmware ID: S3202 CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}else if(tp_dev == TP_TPK_3203) {
			tp_info.manufacture = "TPK_3203";
			printk("TP_DEV = TP_TPK_s3203\n");
			buf[0]= TPK_Config_Data_14029_3203[0];
			buf[1]= TPK_Config_Data_14029_3203[1];
			buf[2]= TPK_Config_Data_14029_3203[2];
			buf[3]= TPK_Config_Data_14029_3203[3];
			FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];	
			printk("read firmware ID: S3203 CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}		
	}	
	else if(is_project(OPPO_14017))
	{
	
		if(tp_dev == TP_TPK) {
			FIRMWARE_ID = FIRMWARE_VERSION;
			printk("read firmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		}		
		if(get_PCB_Version() ==  HW_VERSION__11){
			TP_14017_old=1;
			tp_info.manufacture = "TPK-0D-OLD";
		}else{
			TP_14017_old=0;
			tp_info.manufacture = "TPK-0D-NEW";
		}	
	}	
	if(boot_mode == MSM_BOOT_MODE__NORMAL)
	{	   
	    printk("synaptics_ts_probe:FW update start... \n");
		
		if(CURRENT_FIRMWARE_ID == FIRMWARE_ID)
		{
			printk("CURRENT_FIRMWARE_ID = 0x%x,FIRMWARE_ID_TRULY = 0x%x/n",CURRENT_FIRMWARE_ID,FIRMWARE_ID);
		  	goto after_fw_update;
		}
fw_update:	
		if(is_project(OPPO_14017))
			synatpitcs_ts_update_s3310(ts->client); 
		else
			synatpitcs_ts_update(ts->client);  	
		
		synaptics_read_register_map(ts);
 		i2c_smbus_read_i2c_block_data(ts->client, F34_FLASH_CTRL00, 4, buf); 
 		CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];	
 		fw_cnt++;
 		printk("after FW upate times fw_cnt = %d\n\
 			, CURRENT_FIRMWARE_ID2 = %x,\n",fw_cnt,CURRENT_FIRMWARE_ID);
 	    TP_FW=CURRENT_FIRMWARE_ID;
	}		
	
after_fw_update:
	sprintf(temp,"0x%x",TP_FW);
	tp_info.version=temp;
	register_device_proc("tp", tp_info.version, tp_info.manufacture);	
	/*disable interrupt*/
	ret = synaptics_enable_interrupt(ts, 0);
	if(ret < 0) {
		TPDTM_DMESG(" synaptics_ts_probe: disable interrupt failed\n");
	}
	
	/*read product id */
	ret = synaptics_read_product_id(ts);
	if(ret) {
		TPD_ERR("failed to read product info \n");
	}
   	/*read max_x ,max_y*/
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
	if (ret < 0) {
		TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
	}
	if (is_project(OPPO_14017))
	{
		i2c_smbus_read_i2c_block_data(ts->client, F11_2D_CTRL00, 14, buf);
		printk("buf[0] = 0x%x,buf[1] = 0x%x\n",buf[0],buf[1]);
		ts->max_x = ((buf[1]<<8)&0xffff)|(buf[0]&0xffff);       
		max_y_ic = ((buf[3]<<8)&0xffff)|(buf[2]&0xffff); 
	}else{
		ret = i2c_smbus_read_word_data(ts->client, F11_2D_CTRL06);
   		 if(ret > 0)
	  		ts->max_x = ret&0xffff;       
   		 ret = i2c_smbus_read_word_data(ts->client, F11_2D_CTRL08);
   		 if(ret > 0)
	 	 	max_y_ic = ret&0xffff;	    
	}
	if(is_project(OPPO_14017)||is_project(OPPO_14029))
	{
		ts->max_y = max_y_ic ;
	}
	TPDTM_DMESG("max_x = %d,max_y = %d\n",ts->max_x,max_y_ic);

	bootloader_mode = i2c_smbus_read_byte_data(ts->client,F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40; 	
	TPDTM_DMESG("synaptics:afte fw update,program memory self-check = 0x%x\n",bootloader_mode);	
	
 	if((ts->max_x == 0)||(max_y_ic == 0)||(bootloader_mode == 0x40)){
		if(fw_cnt < 2) {
			TPD_ERR("There is something terrible wrong \n Trying Update the Firmware again\n");
			goto fw_update;			
		}
	}
	
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		TPD_ERR("synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = TPD_DEVICE;;
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR,ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

#ifdef SUPPORT_GESTURE
	set_bit(KEY_F4 , ts->input_dev->keybit);//doulbe-tap resume
#endif
	
	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
			    0, 255, 0, 0);	
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
				0, ts->max_x, 0, 0);
	
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
				0,ts->max_y, 0, 0);
				
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
	input_set_drvdata(ts->input_dev, ts);
	
	if(input_register_device(ts->input_dev)) {
		TPD_ERR("%s: Failed to register input device\n",__func__);
		goto err_input_dev_register;
	}
	synaptics_tpd_button_init(ts);

	
/***Reset TP ******/ 
	if(is_project(OPPO_14017)){
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
		if (ret < 0) {
			TPDTM_DMESG("%s: failed for page select try again later\n",__func__);
			msleep(50);
			ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
			if (ret < 0) {
				TPDTM_DMESG("%s: failed for page select try again later\n",__func__);
			}
		}
		ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_DATA_BASE);
		printk("F01_RMI_DATA_BASE in the resume is %x\n",ret);
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE,0x01);
		msleep(170);
	
		/**************Added temp PVT remove 14017*********/
		if(TP_14017_old==1)
		{
			ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x01); 
			i2c_smbus_write_i2c_block_data(ts->client, 0x3c, 20, write_14017_old);	
			ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);//force update 
			msleep(60);
			ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
		}
	/**************Added temp PVT remove 14017 end*********/	
	}
	/*config tm1429: set report rate, sleep mode */
	ret = synaptics_init_panel(ts); /* will also switch back to page 0x04 */
	if (ret < 0) {
		TPDTM_DMESG("synaptics_init_panel failed\n");	
	}

	ret = synaptics_enable_interrupt(ts, 1);
	printk("synaptics_enable_interrupt\n");
	
	if(ret) {
		TPD_ERR("synaptics_ts_probe: failed to enable synaptics  interrupt \n");
	//	free_irq(client->irq, ts);
		goto exit_init_failed;
	}
#ifdef TPD_USE_EINT
	TPDTM_DMESG("%s: going to set GPIO\n",__func__);
/****************
shoud set the irq GPIO
*******************/		
	if (gpio_is_valid(ts->irq_gpio)) {
		/* configure touchscreen irq gpio */
		ret = gpio_request(ts->irq_gpio,"rmi4_irq_gpio");
		if (ret) {
			TPD_ERR("unable to request gpio [%d]\n",ts->irq_gpio);
			goto exit_init_failed;
		}
		ret = gpio_tlmm_config(GPIO_CFG(
					ts->irq_gpio, 0,
					GPIO_CFG_INPUT,
					GPIO_CFG_PULL_UP,
					GPIO_CFG_2MA),
					GPIO_CFG_ENABLE);	
		if (ret) {
			TPD_ERR("unable to request gpio [%d]\n",ts->irq_gpio);
			goto exit_init_failed;
		}	
		msleep(100);
	}
	
	TPDTM_DMESG("synaptic:ts->client->irq is %d\n",ts->client->irq);
//	ret = request_threaded_irq(client->irq, NULL,synaptics_ts_irq_handler,IRQF_TRIGGER_FALLING,TPD_DEVICE, ts);	
	ret = request_irq(ts_g->client->irq, synaptics_ts_irq_handler, 
	ts->irq_flags, TPD_DEVICE, ts_g);
	//IRQF_TRIGGER_LOW, TPD_DEVICE, ts_g);
	if(ret < 0)
		TPD_ERR("%s request_threaded_irq ret is %d\n",__func__,ret);
#endif

#ifndef TPD_USE_EINT
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = synaptics_ts_timer_func;
	hrtimer_start(&ts->timer, ktime_set(3, 0), HRTIMER_MODE_REL);
#endif

#if defined(CONFIG_FB)
	printk("%s CONFIG_FB is called\n",__func__);
	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		TPD_DEBUG("Unable to register fb_notifier: %d\n",ret);
#endif
	
	printk("synaptics_ts_probe: going to create files\n");	
	if (driver_create_file(&tpd_i2c_driver.driver, &driver_attr_oppo_tp_debug)) {            
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}  	
	if (driver_create_file(&tpd_i2c_driver.driver, &driver_attr_oppo_tp_baseline_image_with_cbc)) {           
		TPDTM_DMESG("driver_create_file failt\n");		
		goto exit_init_failed;
	
	}
	if (driver_create_file(&tpd_i2c_driver.driver, &driver_attr_oppo_tp_baseline_image)) {            
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	if (driver_create_file(&tpd_i2c_driver.driver, &driver_attr_oppo_tp_rawdata_image)) {            
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs_oppo); attr_count++) {
		ret = sysfs_create_file(&ts_g->input_dev->dev.kobj,
				&attrs_oppo[attr_count].attr);
		if (ret < 0) {
			dev_err(&client->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			goto err_sysfs;
		}
	}		
#ifdef CONFIG_SYNAPTIC_RED
	rmidev_init_device();
#endif
	TPDTM_DMESG("synaptics_ts_probe: normal end\n");
	tp_probe_ok=1;
	return 0;
err_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--) {
		sysfs_remove_file(&ts_g->input_dev->dev.kobj,
				&attrs_oppo[attr_count].attr);
	}
exit_init_failed:
	input_unregister_device(ts->input_dev);
err_input_dev_register:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	
#ifdef VENDOR_EDIT//hsy@oppo.com, add 2011/12/14 for tpd power off	
	tpd_power(ts,0);
#endif/*VENDOR_EDIT*/
	printk("synaptics_ts_probe: not normal end\n");
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	int attr_count;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_SYNAPTIC_RED
	rmidev_remove_device();
#endif
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#endif	
#ifndef TPD_USE_EINT
	hrtimer_cancel(&ts->timer);
#endif	
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs_oppo); attr_count++) {
		sysfs_remove_file(&ts_g->input_dev->dev.kobj,
				&attrs_oppo[attr_count].attr);
	}
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	kfree(ts);
    tpd_hw_pwroff(ts);
	return 0;
}

/*******************************14017 suspend && resume************************/
static int synaptics_14017_suspend(struct synaptics_ts_data *ts)
{
	int ret;
	is_suspend = 1;
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
	if (ret < 0) {
		TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
		return 0;
	}
	ret = synaptics_enable_interrupt(ts, 0);
	if(ret) {
		TPD_ERR("synaptics_enable_interrupt failed\n");
	}	
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x04); 
	if (ret < 0) {
		TPD_ERR("%s: control tm1400 to sleep failed\n",__func__);
		return -1;
	}
	if( ts->reset_gpio > 0 ){
		TPD_ERR("synaptics:disable the reset_gpio\n");
		gpio_direction_output(ts->reset_gpio, 0);
	}
	TPDTM_DMESG("%s:normal end\n",__func__);	
	return 0;
}

void synaptics_14017_power_resume(void)
{
	int ret;
	TPD_ERR(" %s is called\n",__func__);
	if(get_boot_mode() != MSM_BOOT_MODE__NORMAL)
		return;
	if ((!ts_g)||(tp_probe_ok==0)){
		TPD_ERR("ts_g is NULL!!! should nerver run here\n");
		return;
	}
	
	free_irq(ts_g->client->irq, ts_g);
/**********reset TP************/
	ret = regulator_disable(ts_g->vdd_2v8);
	if (ret) {
		dev_err(&ts_g->client->dev,
			"Regulator vdd disable failed ret=%d\n", ret);
		return ;
	}
	msleep(1);
	ret = regulator_enable(ts_g->vdd_2v8);
	if (ret) {
		dev_err(&ts_g->client->dev,
			"Regulator vdd enable failed ret=%d\n", ret);
		return;
	}
	msleep(2);
	if( ts_g->reset_gpio > 0 )
	{	
		TPD_ERR("synaptics:enable the reset_gpio\n");
		gpio_direction_output(ts_g->reset_gpio, 1);
		msleep(2);		
		gpio_direction_output(ts_g->reset_gpio, 0);	
		msleep(2);		
		gpio_direction_output(ts_g->reset_gpio, 1);	
	}
}

static void	synaptics_14017_resume(struct synaptics_ts_data *ts)
{
	int ret;	
/**********wait reset TP************/	
	if( ts_g->reset_gpio > 0 )
	{		
		gpio_direction_output(ts_g->reset_gpio, 1);	
	}
	msleep(150);	
	down(&work_sem);
/*****Normal Init TP********/
	
	/**************Added temp PVT remove 14017*********/
	if(TP_14017_old==1)
	{
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x01); 
		i2c_smbus_write_i2c_block_data(ts->client, 0x3c, 20, write_14017_old);	
		ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_COMMAND_BASE, 0x04);//force update 
		msleep(60);
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
	}
	/**************Added temp PVT remove 14017 end*********/
	
#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif	

	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
    ret = synaptics_init_panel(ts);
	if (ret < 0) {
		TPD_DEBUG("%s: TP init failed\n",__func__);
		goto ERR_RESUME;
	}
		
	ret = request_irq(ts_g->client->irq, synaptics_ts_irq_handler, ts->irq_flags, TPD_DEVICE, ts_g);	

	ret = synaptics_enable_interrupt(ts, 1);
	if(ret) {
		TPD_DEBUG("%s:can't  enable interrupt!\n",__func__);
		goto ERR_RESUME;
	}	
	is_suspend = 0;
    TPDTM_DMESG("%s:normal end!\n",__func__);	
	ret = i2c_smbus_read_byte_data(ts_g->client, F01_RMI_DATA01);
	up(&work_sem);
	return;
	
ERR_RESUME:
	up(&work_sem);
	return;
}
/*******************************14017 suspend && resume end************************/

static int synaptics_ts_suspend(struct device *dev)
{
	int ret;
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	TPDTM_DMESG("%s: is called\n",__func__);	
	
	atomic_set(&is_touch,0);
/***********report Up key when suspend********/	
	input_report_key(ts_g->input_dev, BTN_TOUCH, 0);
    input_mt_sync(ts_g->input_dev);	
	input_sync(ts_g->input_dev);
	
/***********incell 14017 suspend*****/	
	if(is_project(OPPO_14017)){
   		synaptics_14017_suspend(ts);
		return 0;
	}
/*****incell 14017 suspend end****/	

#ifndef TPD_USE_EINT
	hrtimer_cancel(&ts->timer);
#endif
#ifdef SUPPORT_GLOVES_MODE    
	if (1 == atomic_read(&glove_enable)){
		/* page select = 0x4 */
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x4); 
		if (ret < 0) {
			TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
			return 0;
		}
		printk("glove mode disable\n");
		ret = i2c_smbus_write_byte_data(ts->client, F51_CUSTOM_CTRL00,0x02 ); 	
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
		if (ret < 0) {
			TPD_ERR("i2c_smbus_write_byte_data failed for page select\n");
			return 0;
		}
	}
#endif

#ifdef SUPPORT_GESTURE	
	if (1 == atomic_read(&double_enable)){
	    synaptics_enable_interrupt_for_gesture(ts, 1);
		TPDTM_DMESG("synaptics:double_tap end suspend\n");
	    return 0;	
    }
#else
    is_suspend = 1;
#endif
	ret = synaptics_enable_interrupt(ts, 0);
	if(ret) {
		TPD_DEBUG("%s: cannot disable interrupt\n",__func__);
		return -1;
	}
	ret = cancel_work_sync(&ts->work);
	if(ret) {
		TPD_DEBUG("%s: cannot disable work\n",__func__);
	}

	ret = synaptics_enable_interrupt(ts, 0);
	if(ret) {
		TPD_ERR("synaptics_enable_interrupt failed\n");
	}	
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x01); 
	if (ret < 0) {
		TPD_ERR("%s: control tm1400 to sleep failed\n",__func__);
		return -1;
	}
	TPDTM_DMESG("%s:normal end\n",__func__);
	return 0;
}

static int synaptics_ts_resume(struct device *dev)
{
	TPD_ERR("%s is called\n",__func__);
	atomic_set(&is_touch,0);
	queue_work(speedup_resume_wq,&ts_g->speed_up_work);
	return 0;
}

static void speedup_synaptics_resume(struct work_struct *work)
{
	int ret;
	struct synaptics_ts_data *ts = ts_g;
	
/***********report Up key when resume********/	
	input_report_key(ts_g->input_dev, BTN_TOUCH, 0);
    input_mt_sync(ts_g->input_dev);	
	input_sync(ts_g->input_dev);
	
/*******for incell 14017 ******/
	if(is_project(OPPO_14017)){
		synaptics_14017_resume(ts);
		return;
	}
/*******14017 resume end******/

down(&work_sem);			
#ifndef TPD_USE_EINT
	hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#else
/***Reset TP ******/ 
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if (ret < 0) {
		TPD_ERR("%s: failed for page select try again later\n",__func__);
		msleep(20);
		ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
		if (ret < 0) {
			TPD_ERR("%s: failed for page select try again later\n",__func__);
		}
	}
	ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_DATA_BASE);
	TPD_ERR("F01_RMI_DATA_BASE in the resume is %x\n",ret);
	free_irq(ts_g->client->irq, ts_g);
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE,0x01);
	msleep(170);
	/*****Gesture Register********/
#ifdef SUPPORT_GESTURE
	if( 1 == atomic_read(&double_enable)){
		ret = synaptics_enable_interrupt_for_gesture(ts, 0); 
		if( ret<0 )
			ret = synaptics_enable_interrupt_for_gesture(ts, 0); 
	}
#endif		
#ifdef SUPPORT_GLOVES_MODE
	synaptics_glove_mode_enable(ts);
#endif	
/*****Normal Init TP********/
    ret = synaptics_init_panel(ts);
	if (ret < 0) {
		TPD_ERR("%s: TP init failed\n",__func__);
		goto ERR_RESUME;
	}
	ret = request_irq(ts_g->client->irq, synaptics_ts_irq_handler, ts->irq_flags, TPD_DEVICE, ts_g);	
#endif
	ret = synaptics_enable_interrupt(ts, 1);
	if(ret) {
		TPD_ERR("%s:can't  enable interrupt!\n",__func__);
		goto ERR_RESUME;
	}	
	is_suspend = 0;
    TPDTM_DMESG("%s:normal end!\n",__func__);	
//	ret = i2c_smbus_read_byte_data(ts_g->client, F01_RMI_DATA01);
	up(&work_sem);
	return;
	
ERR_RESUME:
	up(&work_sem);
	return;
}

static int synaptics_i2c_suspend(struct device *dev)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	TPDTM_DMESG("%s: is called\n",__func__);	
	disable_irq(ts->client->irq);	
	return 0;
}

static int synaptics_i2c_resume(struct device *dev)
{
	struct synaptics_ts_data *ts = dev_get_drvdata(dev);
	TPDTM_DMESG("%s is called\n",__func__);
	enable_irq(ts->client->irq);
	return 0;
}

#if defined(CONFIG_FB)
static int count_resume = 1;
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;	
	struct synaptics_ts_data *ts =
		container_of(self, struct synaptics_ts_data, fb_notif);
	TPDTM_DMESG("%s is called \n",__func__);
	if (evdata && evdata->data && event == FB_EVENT_BLANK && ts && ts->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			if(count_resume == 0){
				TPD_DEBUG("%s going TP resume\n",__func__);
				synaptics_ts_resume(&ts->client->dev);
				count_resume = 1;
			}		
		} else if (*blank == FB_BLANK_POWERDOWN) {
			if(count_resume == 1) {
				TPD_DEBUG("%s : going TP suspend\n",__func__);
				synaptics_ts_suspend(&ts->client->dev);
				count_resume = 0;
			}		
		} 
	}
	return 0;
}
#endif

static int __init tpd_driver_init(void) {
	printk("Synaptic:%s is called\n",__func__);
	 if(i2c_add_driver(&tpd_i2c_driver)!=0) {
        TPDTM_DMESG("unable to add i2c driver.\n");
        return -1;
    }	
	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
 	i2c_del_driver(&tpd_i2c_driver);
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
	return;
}

#ifdef CONFIG_SYNAPTIC_RED
int remote_rmi4_i2c_enable(bool enable)
{
	int ret = 0 ;
	unsigned char status_int ;
	if(!ts_g)
		return 0 ;
	if(enable){
		ret= i2c_smbus_write_byte_data(ts_g->client,0xff,0x0);
		status_int = i2c_smbus_read_byte_data(ts_g->client, F01_RMI_DATA01);
		ret = request_irq(ts_g->client->irq, synaptics_ts_irq_handler, ts_g->irq_flags, ts_g->client->name, ts_g);
		if(ret < 0) {
			printk("[syna] request irq error\n");
		}
	} else {
		free_irq(ts_g->client->irq, ts_g);
	}
	return 0 ;
}

struct input_dev *remote_rmi4_get_input(void)
{
	if(!ts_g)
		return 0 ;
	return ts_g->input_dev ;
}

struct i2c_client *remote_rmi4_get_i2c_client(void)
{
	if(!ts_g)
		return 0 ;
	return ts_g->client;
}

int remote_rmi4_get_irq_gpio(void)
{
	return 17;
}

int remote_rmit_set_page(unsigned int address){
	int ret = 0 ;
	u8 page = 0 ;
	if(!ts_g)
		return 0 ;
	page = ((address >> 8)&0xFF);
	ret = i2c_smbus_write_byte_data(ts_g->client,0xff,page);
	if (ret < 0) {
		TPD_DEBUG("%s: failed for page select\n",__func__);
		return -1;
	}
	return ret ;
}

int remote_rmit_put_page(unsigned int address)
{
    int ret;
   	ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x0); 
	if (ret < 0) {
		TPD_DEBUG("%s: failed for page select\n",__func__);
		return -1;
	}
	return ret;
}
#endif
module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

MODULE_DESCRIPTION("Synaptics S3203 Touchscreen Driver");
MODULE_LICENSE("GPL");

