/* Copyright (c) 2013 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DEBUG
#define pr_fmt(fmt) "SMB358 %s: " fmt, __func__
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/qpnp/qpnp-adc.h>
#include <mach/oppo_boot_mode.h>
#include <mach/oppo_project.h>
#include <linux/power/bq2202a.h>


#if 1
#undef pr_debug
#define pr_debug(fmt, ...) \
	printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)	

#undef dev_dbg
#define dev_dbg(dev, format, arg...)		\
	dev_printk(KERN_INFO, dev, format, ##arg)
#endif

#define SMB358_THREAD_INTERVAL					5000//5S
#define SMB358_THREAD_INIT						1000//5S


/* Config/Control registers */
#define CHG_CURRENT_CTRL_REG		0x0
#define INPUT_CURRENT_LIMIT_REG		0x1
#define VARIOUS_FUNC_REG		0x2
#define VFLOAT_REG			0x3
#define CHG_CTRL_REG			0x4
#define STAT_AND_TIMER_CTRL_REG		0x5
#define CHG_PIN_EN_CTRL_REG		0x6
#define THERM_A_CTRL_REG		0x7
#define SYSOK_AND_USB3_REG		0x8
#define LOW_BATT_THRESHOLD_REG		0x9
#define OTG_TLIM_THERM_REG		0xA
#define FAULT_INT_REG			0xC
#define STATUS_INT_REG			0xD

/* Command registers */
#define CMD_A_REG			0x30
#define CMD_B_REG			0x31

/* Revision register */
#define CHG_REVISION_REG		0x34

/* IRQ status registers */
#define IRQ_A_REG			0x35
#define IRQ_B_REG			0x36
#define IRQ_C_REG			0x37
#define IRQ_D_REG			0x38
#define IRQ_E_REG			0x39
#define IRQ_F_REG			0x3A

/* Status registers */
#define STATUS_C_REG			0x3D
#define STATUS_D_REG			0x3E
#define STATUS_E_REG			0x3F

/* Config bits */
#define CHG_INHI_EN_MASK			BIT(1)
#define CHG_INHI_EN_BIT				BIT(1)
#define CMD_A_STAT_DISABLE_BIT			BIT(0)
#define CMD_A_STAT_DISABLE_MASK			BIT(0)
#define CMD_A_CHG_ENABLE_BIT			BIT(1)
#define CMD_A_VOLATILE_W_PERM_BIT		BIT(7)
#define CMD_A_CHG_SUSP_EN_BIT			BIT(2)
#define CMD_A_CHG_SUSP_EN_MASK			BIT(2)
#define CMD_A_OTG_ENABLE_BIT			BIT(4)
#define CMD_A_OTG_ENABLE_MASK			BIT(4)
#define CMD_A_FAST_CHARGING_SET_BIT		BIT(6)
#define CMD_A_FAST_CHARGING_SET_MASK		BIT(6)
#define CMD_B_CHG_HC_ENABLE_BIT			BIT(0)
//#define CMD_B_CHG_USB3_ENABLE_BIT		BIT(2) //changed to 08h(BIT5)
#define USB3_ENABLE_BIT				BIT(5)
#define USB3_ENABLE_MASK			BIT(5)
#define CMD_B_CHG_USB_500_900_ENABLE_BIT	BIT(1)
#define CHG_CTRL_AUTO_RECHARGE_ENABLE_BIT	0x0
#define CHG_CTRL_CURR_TERM_END_CHG_BIT		0x0
#define CHG_CTRL_BATT_MISSING_DET_THERM_IO	(BIT(5) | BIT(4)) // only therm pin? see 02h
#define CHG_CTRL_AUTO_RECHARGE_MASK		BIT(7)
#define CHG_CTRL_CURR_TERM_END_MASK		BIT(6)
#define CHG_CTRL_BATT_MISSING_DET_MASK		(BIT(5) | BIT(4))
#define CHG_CTRL_APSD_EN_BIT			BIT(2)
#define CHG_CTRL_APSD_EN_MASK			BIT(2)
#define CHG_LOW_BATT_THRESHOLD_MASK		0x0F
#define CHG_PRE_MASK				0x18
#define CHG_ITERM_MASK				0x07
#define CHG_PIN_CTRL_USBCS_REG_BIT		0x0
#define TIMER_CTRL_REG_MASK                     (BIT(3) | BIT(2))
#define TIMER_CTRL_REG_SHIFT                    2
/* This is to select if use external pin EN to control CHG */
#define CHG_PIN_CTRL_CHG_EN_LOW_PIN_BIT		(BIT(5) | BIT(6))
#define CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT		0x0
#define CHG_PIN_CTRL_CHG_EN_MASK		(BIT(5) | BIT(6))

#define CHG_PIN_CTRL_USBCS_REG_MASK		BIT(4) // what's this mean?
#define CHG_PIN_CTRL_APSD_IRQ_BIT		BIT(1)
#define CHG_PIN_CTRL_APSD_IRQ_MASK		BIT(1)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_BIT		BIT(2)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_MASK		BIT(2)
#define VARIOUS_FUNC_USB_SUSP_EN_REG_BIT	BIT(6)
#define VARIOUS_FUNC_USB_SUSP_MASK		BIT(6)
#define FAULT_INT_HOT_COLD_HARD_BIT		BIT(7)
#define FAULT_INT_HOT_COLD_SOFT_BIT		BIT(6)
#define FAULT_INT_INPUT_OV_BIT			BIT(3)
#define FAULT_INT_INPUT_UV_BIT			BIT(2)
#define FAULT_INT_AICL_COMPLETE_BIT		BIT(1)
#define STATUS_INT_CHG_TIMEOUT_BIT		BIT(7)
#define STATUS_INT_OTG_DETECT_BIT		BIT(6)
#define STATUS_INT_BATT_OV_BIT			BIT(5)
#define STATUS_INT_CHGING_BIT			BIT(4)
#define STATUS_INT_CHG_INHI_BIT			BIT(3)
#define STATUS_INT_INOK_BIT			BIT(2) //Use this by default
#define STATUS_INT_MISSING_BATT_BIT		BIT(1)
#define STATUS_INT_LOW_BATT_BIT			BIT(0)
#define THERM_A_THERM_MONITOR_EN_BIT		0x0
#define THERM_A_THERM_MONITOR_EN_MASK		BIT(4)
#define THERM_A_SWITCHING_FREQ_1_5MHZ       BIT(7)
#define THERM_A_SWITCHING_FREQ_MASK		    BIT(7)
#define VFLOAT_MASK				0x3F
#define SMB358_REV_MASK				0x0F
#define SMB358_REV_A4				0x4
#define OTG_CURRENT_LIMIT_BIT                   BIT(3)
#define OTG_CURRENT_LIMIT_MASK                  (BIT(2) | BIT(3))

/* IRQ status bits */
#define IRQ_A_HOT_HARD_BIT			BIT(6)
#define IRQ_A_COLD_HARD_BIT			BIT(4)
#define IRQ_A_HOT_SOFT_BIT			BIT(2)
#define IRQ_A_COLD_SOFT_BIT			BIT(0)
#define IRQ_B_BATT_MISSING_BIT			BIT(4)
#define IRQ_B_BATT_LOW_BIT			BIT(2)
#define IRQ_B_BATT_OV_BIT			BIT(6)
#define IRQ_B_PRE_FAST_CHG_BIT			BIT(0)
#define IRQ_C_TAPER_CHG_BIT			BIT(2)
#define IRQ_C_TERM_BIT				BIT(0)
#define IRQ_C_INT_OVER_TEMP_BIT			BIT(6)
#define IRQ_D_CHG_TIMEOUT_BIT			(BIT(0) | BIT(2))
#define IRQ_D_AICL_DONE_BIT			BIT(4)
#define IRQ_D_APSD_COMPLETE			BIT(6)
#define IRQ_E_INPUT_UV_BIT			BIT(0)
#define IRQ_E_INPUT_OV_BIT			BIT(2)
#define IRQ_E_AFVC_ACTIVE                       BIT(4)
#define IRQ_F_OTG_VALID_BIT			BIT(2)
#define IRQ_F_OTG_BATT_FAIL_BIT			BIT(4)
#define IRQ_F_OTG_OC_BIT			BIT(6)
#define IRQ_F_POWER_OK				BIT(0)

/* Status  bits */
#define STATUS_C_CHARGING_MASK			(BIT(1) | BIT(2))
#define STATUS_C_FAST_CHARGING			BIT(2)
#define STATUS_C_PRE_CHARGING			BIT(1)
#define STATUS_C_TAPER_CHARGING			(BIT(2) | BIT(1))
#define STATUS_C_CHG_ERR_STATUS_BIT		BIT(6)
#define STATUS_C_CHG_ENABLE_STATUS_BIT		BIT(0)
#define STATUS_C_CHG_HOLD_OFF_BIT		BIT(3)
#define STATUS_D_PORT_OTHER			BIT(0)
#define STATUS_D_PORT_SDP			BIT(1)
#define STATUS_D_PORT_DCP			BIT(2)
#define STATUS_D_PORT_CDP			BIT(3)
#define STATUS_D_PORT_ACA_A			BIT(4)
#define STATUS_D_PORT_ACA_B			BIT(5)
#define STATUS_D_PORT_ACA_C			BIT(6)
#define STATUS_D_PORT_ACA_DOCK			BIT(7)

/* constants */
#define USB2_MIN_CURRENT_MA		100
#define USB2_MAX_CURRENT_MA		500
#define USB3_MAX_CURRENT_MA		900
#define AC_CHG_CURRENT_MASK		0xF0
#define SMB358_IRQ_REG_COUNT		6
#define SMB358_FAST_CHG_MIN_MA		200
#define SMB358_FAST_CHG_MAX_MA		2000
#define SMB358_FAST_CHG_SHIFT		5
#define SMB358_INPUT_CURRENT_LIMIT_SHIFT	4
#define SMB358_IMPUT_CURRENT_LIMIT_MAX_MA       2000
#define SMB_FAST_CHG_CURRENT_MASK	0xE0
#define SMB358_DEFAULT_BATT_CAPACITY	50

#define DEFAULT_SOC_SYNC_UP_TIME  8
#define DEFAULT_SOC_SYNC_DOWN_TIME 8
int bat_volt_cp_flag = 0;
int bat_volt_check_point = 50;
int bat_volt_bms_soc = 50;
static int g_soc_sync_time=0;
static int g_soc_sync_up_times = DEFAULT_SOC_SYNC_UP_TIME;
static int g_soc_sync_down_times = DEFAULT_SOC_SYNC_DOWN_TIME;

extern int power_type;
extern void qpnp_bms_backup_smb358_checkpoint(int checkpoint);
extern int get_estimate_ocv(int batt_temp);

#define OPPO_BATTERY_ENCRPTION
bool oppo_high_battery_status = 1;
int oppo_check_ID_status = 0;

#ifdef OPPO_BATTERY_ENCRPTION
int oppo_high_battery_check_counts = 0;
bool oppo_battery_status_init_flag = 0;
static void oppo_battery_status_init(void);
static void oppo_battery_status_check(void);
#endif


#ifdef OPPO_BATTERY_ENCRPTION
static void oppo_battery_status_init(void)
{
    static int CheckIDSign=5;
	if(!oppo_battery_status_init_flag)
	{
		while(CheckIDSign>0)
	    {
	        CheckIDCompare();
	        CheckIDSign--;	        
			printk( "IDSign = %d, check_ID_status = %d: oppo_high_battery_status =%d \r\n", CheckIDSign, oppo_check_ID_status,oppo_high_battery_status);
	        if(oppo_check_ID_status > 0)
	        {
	            oppo_high_battery_status = 1;
	            oppo_check_ID_status=0;
	            CheckIDSign =0;
				oppo_battery_status_init_flag = 1;
				break;
	        }
	        else if(CheckIDSign <= 0)
	        {
	            oppo_high_battery_status = 0;
	            oppo_check_ID_status=0;
				oppo_battery_status_init_flag = 1;
	        }			
	    }
	}
}

static void oppo_battery_status_check(void)
{
	if((oppo_high_battery_status == 0)&&(oppo_battery_status_init_flag))
	{
	    if(oppo_high_battery_check_counts < 10)
		{
		    CheckIDCompare();
			oppo_high_battery_check_counts++;
			printk( "oppo_high_battery_check_counts =%d, oppo_check_ID_status = %d, oppo_high_battery_status =%d\n" ,oppo_high_battery_check_counts, oppo_check_ID_status, oppo_high_battery_status);
		    if(oppo_check_ID_status > 0)
		    {
				oppo_high_battery_status = 1;
				oppo_check_ID_status=0;
				oppo_high_battery_check_counts = 0;
		    }   			 		
	    }
	}
}
#endif

struct smb358_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

typedef struct 
{
    bool 		charger_exist; 
	int   							power_type;
	int			charger_vol;
	bool		is_charging;
	int			charging_current;
	
	bool   		bat_exist;
    u8   		bat_status; 
	int   		bat_instant_vol; 
	int 		bat_estimate_ocv;
	int   		temperature;
    u8  		bat_charging_state;
	u8			bat_temp_status;
	int			total_charging_time;
	
	u8			SOC;
	u8			bat_volt_check_point ;
	u8			battery_request_poweroff;
} PMU_ChargerStruct;


struct smb358_charger {
	struct i2c_client	*client;
	struct device		*dev;
#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv,2014.2.24 add for OTG
	struct mutex			read_write_lock;
	PMU_ChargerStruct		BMT_status;
#endif /* VENDOR_EDIT */
	bool			charger_inhibit_disabled;
	int			recharge_mv;
	bool			iterm_disabled;
	int			iterm_ma;
	int			vfloat_mv;
	int			chg_valid_gpio;
	int			chg_valid_act_low;
	int			chg_present;
	int			fake_battery_soc;
	bool			chg_autonomous_mode;
	bool			disable_apsd;
	bool			battery_missing;
	const char		*bms_psy_name;

	/* status tracking */	
	bool			batt_point_full;
	bool			batt_pre_full;
	bool			batt_full;
	bool			batt_hot;
	bool			batt_cold;
	bool			batt_warm;
	bool			batt_cool;
	bool			charge_voltage_over;
	bool			batt_voltage_over;
	#ifdef VENDOR_EDIT
	bool			multiple_test;
	#endif
	bool			suspending;
	bool			action_pending;
	bool			otg_enable_pending;
	bool			otg_disable_pending;

	int			charging_disabled;
	int			fastchg_current_max_ma;
	int			fastchg_current_ma;
	int			limit_current_max_ma;
	bool			charging_time_out;
	int			charging_smb358_temp_statu;
	int			temp_vfloat_mv;
	int			workaround_flags;

	struct power_supply	*usb_psy;
	struct power_supply	*bms_psy;
	struct power_supply	batt_psy;

	struct delayed_work	update_smb358_thread_work;
	struct delayed_work	smb358_delayed_wakeup_work;
	struct work_struct      smb358_stat_work;
	struct workqueue_struct *smb358_work_queue;
	struct wakeup_source	source;

	struct smb358_regulator	otg_vreg;

	struct dentry		*debug_root;
	u32			peek_poke_address;
	/* TODO */
	struct qpnp_vadc_chip	*vadc_dev;
	struct qpnp_adc_tm_chip	*adc_tm_dev;
	struct qpnp_adc_tm_btm_param	adc_param;
	int			hot_bat_decidegc;
	int			warm_bat_decidegc;
	int			little_cool_bat_decidegc;
	int			cool_bat_decidegc;
	int			cold_bat_decidegc;
	int			bat_present_decidegc;
	int			temp_cool_vfloat_mv;
	int			temp_cool_fastchg_current_ma;
	int			temp_little_cool_vfloat_mv;
	int			temp_little_cool_fastchg_current_ma;
	int			temp_warm_vfloat_mv;
	int			temp_warm_fastchg_current_ma;
	int 		non_standard_vfloat_mv;
	int			non_standard_fastchg_current_ma;
	struct regulator*	vcc_i2c;
	#if 0//VENDOR_EDIT
	int			irq_gpio;
	#else
	int			stat_gpio;
	#endif
	int			fastcharger;
};

struct smb_irq_info {
	const char		*name;
	int			(*smb_irq)(struct smb358_charger *chip,
							u8 rt_stat);
	int			high;
	int			low;
};

struct irq_handler_info {
	u8			stat_reg;
	u8			val;
	u8			prev_val;
	struct smb_irq_info	irq_info[4];
};

struct chg_current_map {
	int	chg_current_ma;
	u8	value;
};

static int input_current[] = {
	300, 500, 700, 1000, 1200, 1500, 1800, 2000,
};

static int fast_chg_current[] = {
	200, 450, 600, 900, 1300, 1500, 1800, 2000,
};

/* add supplied to "bms" function */
static char *pm_batt_supplied_to[] = {
        "bms",
};

static int g_is_wakeup = -1;
static int g_chg_in = -1;
static int smb358_reset_charge_parameters(struct smb358_charger *chip)
{
        int rc = 0;
        
        chip->batt_voltage_over = false;
        chip->charge_voltage_over = false;
        chip->charging_time_out = false;
		chip->batt_point_full = 0;
        chip->batt_pre_full = 0;
        chip->batt_full = 0;
        
        return rc;
}

static int __smb358_read_reg(struct smb358_charger *chip, u8 reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int __smb358_write_reg(struct smb358_charger *chip, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int smb358_read_reg(struct smb358_charger *chip, int reg,
						u8 *val)
{
	int rc;

	mutex_lock(&chip->read_write_lock);
	rc = __smb358_read_reg(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb358_write_reg(struct smb358_charger *chip, int reg,
						u8 val)
{
	int rc;

	mutex_lock(&chip->read_write_lock);
	rc = __smb358_write_reg(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb358_masked_write(struct smb358_charger *chip, int reg,
							u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	mutex_lock(&chip->read_write_lock);
	rc = __smb358_read_reg(chip, reg, &temp);
	if (rc) {
		dev_err(chip->dev,
			"smb358_read_reg Failed: reg=%03X, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = __smb358_write_reg(chip, reg, temp);
	if (rc) {
		dev_err(chip->dev,
			"smb358_write Failed: reg=%03X, rc=%d\n", reg, rc);
	}
out:
	mutex_unlock(&chip->read_write_lock);
	return rc;
}

static int smb358_enable_volatile_writes(struct smb358_charger *chip)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_VOLATILE_W_PERM_BIT,
						CMD_A_VOLATILE_W_PERM_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't write VOLATILE_W_PERM_BIT rc=%d\n",
				rc);

	return rc;
}

static int smb358_fastchg_current_set(struct smb358_charger *chip)
{
	u8 i;

	if ((chip->fastchg_current_ma < SMB358_FAST_CHG_MIN_MA) ||
		(chip->fastchg_current_ma >  SMB358_FAST_CHG_MAX_MA)) {
		dev_dbg(chip->dev, "bad fastchg current mA=%d asked to set\n",
					chip->fastchg_current_ma);
		return -EINVAL;
	}

	for (i = ARRAY_SIZE(fast_chg_current) - 1; i >= 0; i--)
	{
		if (fast_chg_current[i] <= chip->fastchg_current_ma)
			break;
	}

	if (i < 0) {
		dev_err(chip->dev, "Cannot find %dmA\n", chip->fastchg_current_ma);
		i = 0;
	}

	i = i << SMB358_FAST_CHG_SHIFT;
	dev_dbg(chip->dev, "fastchg limit=%d setting %02x\n",
			chip->fastchg_current_ma, i);

	return smb358_masked_write(chip, CHG_CURRENT_CTRL_REG,
				SMB_FAST_CHG_CURRENT_MASK, i);
}

#define MIN_FLOAT_MV		3500
#define MAX_FLOAT_MV		4500
#define VFLOAT_STEP_MV		20
#define VFLOAT_4350MV		4350
static int smb358_float_voltage_set(struct smb358_charger *chip, int vfloat_mv)
{
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dev_err(chip->dev, "bad float voltage mv =%d asked to set\n",
					vfloat_mv);
		return -EINVAL;
	}

	if (VFLOAT_4350MV == vfloat_mv)
		temp = 0x2B;
	else if (vfloat_mv > VFLOAT_4350MV)
		temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV + 1;
	else
		temp = (vfloat_mv - MIN_FLOAT_MV) / VFLOAT_STEP_MV;

	return smb358_masked_write(chip, VFLOAT_REG, VFLOAT_MASK, temp);
}

struct smb358_charger *chip_smb358 = NULL;

int smb358_chg_otg_enable(void)
{
	int rc = 0;
	while(!chip_smb358)
	{
		pr_err("smb358_chg_otg_enable----chip_smb358 is not ok\r\n");
		msleep(500);		
	}
	if(chip_smb358->suspending)
	{
		chip_smb358->otg_enable_pending = true;
		 pr_debug("smb358_chg_otg_enable----chip_smb358 suspending\r\n");
	}
	else
	{
		rc = smb358_masked_write(chip_smb358, CMD_A_REG, CMD_A_OTG_ENABLE_BIT,CMD_A_OTG_ENABLE_BIT);
		pr_debug("smb358_chg_otg_enable----rc = %d\r\n",rc);
	}
	
	//rc = smb358_write_reg(chip_smb358, CMD_A_REG, 0x93);
	return rc;
}

int smb358_chg_otg_disable(void)
{
	int rc = 0;

	if(chip_smb358->suspending)
	{
		chip_smb358->otg_disable_pending = true;
		pr_debug("smb358_chg_otg_disable----chip_smb358 suspending\r\n");
	}
	else
	{		
		rc = smb358_masked_write(chip_smb358, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, 0);
		printk("smb358_chg_otg_disable----rc = %d\r\n",rc);
	}
	return rc;
}
void smb358_chg_otg_read(void)
{
	u8 a_reg = 0;
	smb358_read_reg(chip_smb358, CMD_A_REG, &a_reg);
	printk("kongfanhong------------smb358_chg_otg_read----a_reg = 0x%x\r\n",a_reg);
}
static int smb358_chg_otg_regulator_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT,
							CMD_A_OTG_ENABLE_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't enable  OTG mode rc=%d\n", rc);
	return rc;
}

static int smb358_chg_otg_regulator_disable(struct regulator_dev *rdev)
{
	int rc = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_OTG_ENABLE_BIT, 0);
	if (rc)
		dev_err(chip->dev, "Couldn't disable OTG mode rc=%d\n", rc);
	return rc;
}

static int smb358_chg_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int rc = 0;
	u8 reg = 0;
	struct smb358_charger *chip = rdev_get_drvdata(rdev);

	rc = smb358_read_reg(chip, CMD_A_REG, &reg);
	if (rc) {
		dev_err(chip->dev,
				"Couldn't read OTG enable bit rc=%d\n", rc);
		return rc;
	}

	return  (reg & CMD_A_OTG_ENABLE_BIT) ? 1 : 0;
}

struct regulator_ops smb358_chg_otg_reg_ops = {
	.enable		= smb358_chg_otg_regulator_enable,
	.disable	= smb358_chg_otg_regulator_disable,
	.is_enabled	= smb358_chg_otg_regulator_is_enable,
};

static int smb358_regulator_init(struct smb358_charger *chip)
{
	int rc = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg;

	init_data = of_get_regulator_init_data(chip->dev, chip->dev->of_node);
	if (!init_data) {
		dev_err(chip->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	/* Give the name, then will register */
	if (init_data->constraints.name) {
		chip->otg_vreg.rdesc.owner = THIS_MODULE;
		chip->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		chip->otg_vreg.rdesc.ops = &smb358_chg_otg_reg_ops;
		chip->otg_vreg.rdesc.name = init_data->constraints.name;

		cfg.dev = chip->dev;
		cfg.init_data = init_data;
		cfg.driver_data = chip;
		cfg.of_node = chip->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

/*		chip->otg_vreg.rdev = regulator_register(
*						&chip->otg_vreg.rdesc, &cfg);
*/
		chip->otg_vreg.rdev = regulator_register(
						&chip->otg_vreg.rdesc, cfg.dev,
						cfg.init_data, cfg.driver_data,
						cfg.of_node); 
		if (IS_ERR(chip->otg_vreg.rdev)) {
			rc = PTR_ERR(chip->otg_vreg.rdev);
			chip->otg_vreg.rdev = NULL;
			if (rc != -EPROBE_DEFER)
				dev_err(chip->dev,
					"OTG reg failed, rc=%d\n", rc);
		}
	}

	return rc;
}

#define CHG_PRE_150MA			0x00
#define CHG_PRE_250MA			0x08
#define CHG_PRE_350MA			0x10
#define CHG_PRE_450MA			0x18

#define CHG_ITERM_30MA			0x00
#define CHG_ITERM_40MA			0x01
#define CHG_ITERM_60MA			0x02
#define CHG_ITERM_80MA			0x03
#define CHG_ITERM_100MA			0x04
#define CHG_ITERM_125MA			0x05
#define CHG_ITERM_150MA			0x06
#define CHG_ITERM_200MA			0x07

#define VFLT_300MV			0x0C
#define VFLT_200MV			0x08
#define VFLT_100MV			0x04
#define	VFLT_50MV			0x00
#define VFLT_MASK			0x0C
static int smb358_hw_init(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0, mask = 0;

	/*
	 * If the charger is pre-configured for autonomous operation,
	 * do not apply additonal settings
	 */
	if (chip->chg_autonomous_mode) {
		dev_dbg(chip->dev, "Charger configured for autonomous mode\n");
		return 0;
	}

	rc = smb358_read_reg(chip, CHG_REVISION_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read CHG_REVISION_REG rc=%d\n",
									rc);
		return rc;
	}

	rc = smb358_enable_volatile_writes(chip);
	if (rc) {
		dev_err(chip->dev, "Couldn't configure volatile writes rc=%d\n",
				rc);
		return rc;
	}

	/* setup defaults for CHG_CNTRL_REG */
	reg = CHG_CTRL_BATT_MISSING_DET_THERM_IO;
	mask = CHG_CTRL_BATT_MISSING_DET_MASK;
	rc = smb358_masked_write(chip, CHG_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n", rc);
		return rc;
	}
	/* setup defaults for PIN_CTRL_REG */
	reg = CHG_PIN_CTRL_USBCS_REG_BIT | CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT |
		CHG_PIN_CTRL_APSD_IRQ_BIT | CHG_PIN_CTRL_CHG_ERR_IRQ_BIT;
	mask = CHG_PIN_CTRL_CHG_EN_MASK | CHG_PIN_CTRL_USBCS_REG_MASK |
		CHG_PIN_CTRL_APSD_IRQ_MASK | CHG_PIN_CTRL_CHG_ERR_IRQ_MASK;
	rc = smb358_masked_write(chip, CHG_PIN_EN_CTRL_REG, mask, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_PIN_EN_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}

	/* setup USB suspend and APSD  */
	rc = smb358_masked_write(chip, VARIOUS_FUNC_REG,
		VARIOUS_FUNC_USB_SUSP_MASK, VARIOUS_FUNC_USB_SUSP_EN_REG_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set VARIOUS_FUNC_REG rc=%d\n",
				rc);
		return rc;
	}

	if (!chip->disable_apsd)
		reg = CHG_CTRL_APSD_EN_BIT;
	else
	        reg = 0;
	rc = smb358_masked_write(chip, CHG_CTRL_REG, CHG_CTRL_APSD_EN_MASK, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}
	/* Fault and Status IRQ configuration */
	reg = FAULT_INT_HOT_COLD_HARD_BIT | FAULT_INT_HOT_COLD_SOFT_BIT
		| FAULT_INT_INPUT_UV_BIT | FAULT_INT_AICL_COMPLETE_BIT
		| FAULT_INT_INPUT_OV_BIT;
	rc = smb358_write_reg(chip, FAULT_INT_REG, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set FAULT_INT_REG rc=%d\n", rc);
		return rc;
	}
	reg = STATUS_INT_CHG_TIMEOUT_BIT | STATUS_INT_OTG_DETECT_BIT |
		STATUS_INT_BATT_OV_BIT | STATUS_INT_CHGING_BIT |
		STATUS_INT_CHG_INHI_BIT | STATUS_INT_INOK_BIT |
		STATUS_INT_LOW_BATT_BIT | STATUS_INT_MISSING_BATT_BIT;
	rc = smb358_write_reg(chip, STATUS_INT_REG, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set STATUS_INT_REG rc=%d\n", rc);
		return rc;
	}
	/* setup THERM Monitor */
	rc = smb358_masked_write(chip, THERM_A_CTRL_REG,
		THERM_A_THERM_MONITOR_EN_MASK, THERM_A_THERM_MONITOR_EN_MASK);
	if (rc) {
		dev_err(chip->dev, "Couldn't set THERM_A_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}

    /* setup switching frequency */
	rc = smb358_masked_write(chip, THERM_A_CTRL_REG,
		THERM_A_SWITCHING_FREQ_MASK, THERM_A_SWITCHING_FREQ_1_5MHZ);
	if (rc) {
		dev_err(chip->dev, "Couldn't set THERM_A_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}

	/* setup otg current limit */
	rc = smb358_masked_write(chip, OTG_TLIM_THERM_REG,
		OTG_CURRENT_LIMIT_MASK, OTG_CURRENT_LIMIT_BIT);
	if (rc) {
		dev_err(chip->dev, "Couldn't set OTG_TLIM_THERM_REG rc=%d\n",
				rc);
		return rc;
	}
	
	/* set the fast charge current limit */
	rc = smb358_fastchg_current_set(chip);
	if (rc) {
		dev_err(chip->dev, "Couldn't set fastchg current rc=%d\n", rc);
		return rc;
	}

	/* set the float voltage */
	if (chip->vfloat_mv != -EINVAL) {
		rc = smb358_float_voltage_set(chip, chip->vfloat_mv);
		if (rc < 0) {
			dev_err(chip->dev,
				"Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
	}

#if 0
	/* set low_battery voltage threshold */
	reg = 0x0f;//3.58V
	rc = smb358_masked_write(chip, LOW_BATT_THRESHOLD_REG,
		CHG_LOW_BATT_THRESHOLD_MASK, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set LOW_BATT_THRESHOLD_REG rc=%d\n",
				rc);
		return rc;
	}
#endif
	
	/* set pre-charging current */
	reg = CHG_PRE_450MA;
	rc = smb358_masked_write(chip, CHG_CURRENT_CTRL_REG,
		CHG_PRE_MASK, reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't set CHG_CURRENT_CTRL_REG rc=%d\n",
				rc);
		return rc;
	}

	/* set iterm */
	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled) {
			dev_err(chip->dev, "Error: Both iterm_disabled and iterm_ma set\n");
			return -EINVAL;
		} else {
			if (chip->iterm_ma <= 30)
				reg = CHG_ITERM_30MA;
			else if (chip->iterm_ma <= 40)
				reg = CHG_ITERM_40MA;
			else if (chip->iterm_ma <= 60)
				reg = CHG_ITERM_60MA;
			else if (chip->iterm_ma <= 80)
				reg = CHG_ITERM_80MA;
			else if (chip->iterm_ma <= 100)
				reg = CHG_ITERM_100MA;
			else if (chip->iterm_ma <= 125)
				reg = CHG_ITERM_125MA;
			else if (chip->iterm_ma <= 150)
				reg = CHG_ITERM_150MA;
			else
				reg = CHG_ITERM_200MA;

			rc = smb358_masked_write(chip, CHG_CURRENT_CTRL_REG,
							CHG_ITERM_MASK, reg);
			if (rc) {
				dev_err(chip->dev,
					"Couldn't set iterm rc = %d\n", rc);
				return rc;
			}

			rc = smb358_masked_write(chip, CHG_CTRL_REG,
						CHG_CTRL_CURR_TERM_END_MASK, 0);
			if (rc) {
				dev_err(chip->dev,
					"Couldn't enable iterm rc = %d\n", rc);
				return rc;
			}
		}
	} else  if (chip->iterm_disabled) {
		rc = smb358_masked_write(chip, CHG_CTRL_REG,
					CHG_CTRL_CURR_TERM_END_MASK,
					CHG_CTRL_CURR_TERM_END_MASK);
		if (rc) {
			dev_err(chip->dev, "Couldn't set iterm rc = %d\n",
								rc);
			return rc;
		}
	}

	/* TODO: set inhibit threshold */
	if (chip->charger_inhibit_disabled) {
		rc = smb358_masked_write(chip, INPUT_CURRENT_LIMIT_REG,
					CHG_INHI_EN_MASK, 0x0);
	} else {
		rc = smb358_masked_write(chip, INPUT_CURRENT_LIMIT_REG,
                                        CHG_INHI_EN_MASK, CHG_INHI_EN_BIT);
	}
	reg = 0;
	if (chip->recharge_mv >= 300)
		reg = VFLT_300MV;
	else if (200 <= chip->recharge_mv && chip->recharge_mv < 300)
		reg = VFLT_200MV;
	else if (100 <= chip->recharge_mv && chip->recharge_mv < 200)
		reg = VFLT_100MV;
	else if (50 <= chip->recharge_mv && chip->recharge_mv < 100)
		reg = VFLT_50MV;
	else
		reg = VFLT_50MV;

	rc = smb358_masked_write(chip, INPUT_CURRENT_LIMIT_REG,
					VFLT_MASK, reg);
	if (rc) {
		dev_err(chip->dev,
			"Couldn't set inhibit threshold rc = %d\n", rc);
		return rc;
	}

	/* enable/disable stat output */
	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_STAT_DISABLE_MASK,
			CMD_A_STAT_DISABLE_BIT);
	if (rc) {
		dev_err(chip->dev, "Unable to %s stat pin. rc=%d\n",
			CMD_A_STAT_DISABLE_BIT ? "disable" : "enable", rc);
	}

	/* enable/disable charging */
	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_CHG_ENABLE_BIT,
			chip->charging_disabled ? 0 : CMD_A_CHG_ENABLE_BIT);
	if (rc) {
		dev_err(chip->dev, "Unable to %s charging. rc=%d\n",
			chip->charging_disabled ? "disable" : "enable", rc);
	}

	/* enable/disable fast charging setting */
	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_FAST_CHARGING_SET_MASK,
			CMD_A_FAST_CHARGING_SET_BIT);
	if (rc) {
		dev_err(chip->dev, "Unable to %s fast charging set. rc=%d\n",
			CMD_A_FAST_CHARGING_SET_BIT ? "disable" : "enable", rc);
	}

	return rc;
}

static enum power_supply_property smb358_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_TIMEOUT,
	POWER_SUPPLY_PROP_FAST_CHARGE,
	POWER_SUPPLY_PROP_TEMP_STATU,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_BATTERY_REQUEST_POWEROFF,
};
static int smb358_get_prop_batt_health(struct smb358_charger *chip);
/* DONE */
static int smb358_get_prop_batt_status(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	if (chip->suspending)
		return chip->BMT_status.bat_status; 

	if (chip->batt_full)
	{
		//if((chip->charging_smb358_temp_statu == SMB358_CHG_TEMP_NORMAL) && (oppo_high_battery_status == 1))
		if(chip->charging_smb358_temp_statu == SMB358_CHG_TEMP_NORMAL)
		{
			if (chip->batt_point_full)
			{
				return POWER_SUPPLY_STATUS_FULL;
			}
			else
			{
				return POWER_SUPPLY_STATUS_CHARGING;
			}
		}
		else
		{
			return POWER_SUPPLY_STATUS_FULL;
		}
	}
	else if (chip->batt_pre_full)
	        return POWER_SUPPLY_STATUS_CHARGING;
	        

	rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	//pr_debug(chip->dev, "%s: STATUS_C_REG=%x\n", __func__, reg);

	if (reg & STATUS_C_CHG_HOLD_OFF_BIT)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	if ((reg & STATUS_C_CHARGING_MASK) &&
			!(reg & STATUS_C_CHG_ERR_STATUS_BIT))
		return POWER_SUPPLY_STATUS_CHARGING;

	return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int smb358_get_prop_batt_present(struct smb358_charger *chip)
{
	return !chip->battery_missing;
}

#define SHUTDOWN_INSTANT_LOW_POWER  		3200 
#define SHUTDOWN_AVERAGE_LOW_POWER  		3500
int gBAT_counter_15=1;
int gFG_15_vlot = 3700;
static int shutdown_point_rst_flag = 0;
static int shutdown_instant_times = 0;
//static int shutdown_average_times = 0;

static int smb358_get_prop_battery_voltage_now(struct smb358_charger *chip);	
static int smb358_check_battery_request_poweroff(struct smb358_charger *chip)
{	
	chip->BMT_status.battery_request_poweroff = 0;
	chip->BMT_status.bat_instant_vol = smb358_get_prop_battery_voltage_now(chip)/1000;
	if(chip->BMT_status.bat_instant_vol <= SHUTDOWN_INSTANT_LOW_POWER)
    {
        shutdown_instant_times++;
		printk("SHUT_DOWN,instant_bat_vol=%d <SHUTDOWN_INSTANT_LOW_POWER, times = %d, request power off\n", chip->BMT_status.bat_instant_vol, shutdown_instant_times);
        if(shutdown_instant_times >= 3)
        {
            //chip->BMT_status.battery_request_poweroff=4;
            shutdown_instant_times = 0;
			shutdown_point_rst_flag = 1;
        }
        
    }
	
	if(shutdown_point_rst_flag == 1)
	{
		if(bat_volt_check_point > 2)
		{
			bat_volt_check_point--;
			//set_rtc_spare_fg_value(bat_volt_check_point);
			//set_rtc_spare_fg_value(2);
		}
		else
		{
			chip->BMT_status.battery_request_poweroff=4;
			//set_rtc_spare_fg_value(2);
		}
	}
	return bat_volt_check_point;
}

static int smb358_get_prop_batt_capacity(struct smb358_charger *chip)
{
	union power_supply_propval ret = {0, };	

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;
	if(!chip->bms_psy)
	{
		return SMB358_DEFAULT_BATT_CAPACITY;
	}
	else if (chip->bms_psy) 
	{
		chip->bms_psy->get_property(chip->bms_psy,POWER_SUPPLY_PROP_CAPACITY, &ret);		
	}
	bat_volt_bms_soc = ret.intval;
	
	if (bat_volt_cp_flag == 0) 
	{
		bat_volt_cp_flag = 1;		
		bat_volt_check_point = bat_volt_bms_soc;
		dev_dbg(chip->dev, "smb358_get_prop_batt_capacity bat_volt_cp_flag = 1,bat_volt_check_point = %d\r\n",bat_volt_check_point);
		return bat_volt_check_point;
	}

	//if((chip->chg_present) && (chip->batt_full) && (smb358_get_prop_batt_present(chip) == 1) && (chip->charging_smb358_temp_statu == SMB358_CHG_TEMP_NORMAL) && (oppo_high_battery_status == 1))
	if((chip->chg_present) && (chip->batt_full) && (smb358_get_prop_batt_present(chip) == 1) && (chip->charging_smb358_temp_statu == SMB358_CHG_TEMP_NORMAL))
	{
		if(g_soc_sync_time >= g_soc_sync_up_times)
		{
			g_soc_sync_time = 0;
			bat_volt_check_point++;
		}  
		else
		{
			g_soc_sync_time+=1;   
		}
		
		if(bat_volt_check_point >= 100)
		{
			bat_volt_check_point = 100;
			chip->batt_point_full = 1;
		}
		dev_dbg(chip->dev, "smb358_get_prop_batt_capacity full bat_volt_bms_soc = %d, bat_volt_check_point = %d\r\n",bat_volt_bms_soc,bat_volt_check_point);
		qpnp_bms_backup_smb358_checkpoint(bat_volt_check_point);
		return bat_volt_check_point;
	}
	else if((chip->chg_present) && (smb358_get_prop_batt_status(chip) == POWER_SUPPLY_STATUS_CHARGING)  && (smb358_get_prop_batt_present(chip) == 1))
	{
		if(bat_volt_bms_soc== bat_volt_check_point)
		{		
			dev_dbg(chip->dev, "smb358_get_prop_batt_capacity   battery charging bat_volt_bms_soc == bat_volt_check_point = %d\r\n",bat_volt_bms_soc);
		}
		else if (bat_volt_bms_soc > bat_volt_check_point)
		{						
			if(g_soc_sync_time >= g_soc_sync_up_times)
			{
				g_soc_sync_time = 0;
				bat_volt_check_point++;
						
			}  
			else
			{
				g_soc_sync_time+=1;   
			}			
			dev_dbg(chip->dev, "smb358_get_prop_batt_capacity  battery charging  g_soc_sync_time = %d,bat_volt_bms_soc(%d) > bat_volt_check_point(%d)\r\n",g_soc_sync_time,bat_volt_bms_soc,bat_volt_check_point);
		}		
		else
		{
			if(g_soc_sync_time >= g_soc_sync_down_times)
			{
				g_soc_sync_time=0;
				bat_volt_check_point--;
			}
			else
			{
				g_soc_sync_time+=1;
			}
			dev_dbg(chip->dev, "smb358_get_prop_batt_capacity  battery charging  g_soc_sync_time = %d,bat_volt_bms_soc(%d) < bat_volt_check_point(%d)\r\n",g_soc_sync_time,bat_volt_bms_soc,bat_volt_check_point);
		}
	}
	else
	{
		chip->BMT_status.bat_estimate_ocv = get_estimate_ocv(chip->BMT_status.temperature)/1000;
		#if 0
		if((chip->BMT_status.bat_estimate_ocv <= gFG_15_vlot) && (bat_volt_check_point>=15))
		{		
			if(gBAT_counter_15==0)
			{
				bat_volt_check_point--;
				gBAT_counter_15=1;
			}		
			else
			{
				gBAT_counter_15=0;
			}
			dev_dbg(chip->dev, "smb358_check_battery_point15_volt gBAT_counter_15 = %d,bat_estimate_ocv(%d)<=gFG_15_vlot(%d),bat_volt_check_point(%d)>=15\r\n",gBAT_counter_15,chip->BMT_status.bat_estimate_ocv,gFG_15_vlot,bat_volt_check_point);
		}
		else if ( (chip->BMT_status.bat_estimate_ocv > gFG_15_vlot)&&(bat_volt_check_point==15) )
		{
			gBAT_counter_15=1;			
			dev_dbg(chip->dev, "smb358_check_battery_point15_volt battery discharging gBAT_counter_15 = %d,bat_estimate_ocv(%d)>gFG_15_vlot(%d),bat_volt_check_point(%d)==15 wait \r\n",gBAT_counter_15,chip->BMT_status.bat_estimate_ocv,gFG_15_vlot,bat_volt_check_point);
		}
		else  
		{
			gBAT_counter_15=1;
			if (bat_volt_bms_soc < bat_volt_check_point)
			{
				if(g_soc_sync_time >= g_soc_sync_down_times)
				{
					g_soc_sync_time=0;
					bat_volt_check_point--;
				}
				else
				{
					g_soc_sync_time+=1;
				}
			}
			dev_dbg(chip->dev, "smb358_get_prop_batt_capacity battery discharging gBAT_counter_15 = %d,bat_estimate_ocv(%d)?gFG_15_vlot(%d),bat_volt_bms_soc = %d,bat_volt_check_point(%d),g_soc_sync_time = %d\r\n",gBAT_counter_15,chip->BMT_status.bat_estimate_ocv,gFG_15_vlot,bat_volt_bms_soc,bat_volt_check_point,g_soc_sync_time);
			
		}
		#else
		if (bat_volt_bms_soc < bat_volt_check_point)
		{
			if(g_soc_sync_time >= g_soc_sync_down_times)
			{
				g_soc_sync_time=0;
				bat_volt_check_point--;
			}
			else
			{
				g_soc_sync_time+=1;
			}
		}
		dev_dbg(chip->dev, "smb358_get_prop_batt_capacity battery discharging  g_soc_sync_time = %d,bat_volt_bms_soc(%d) ? bat_volt_check_point(%d)\r\n",g_soc_sync_time,bat_volt_bms_soc,bat_volt_check_point);
		#endif	
		
	}	
	
	smb358_check_battery_request_poweroff(chip);
	
	if(bat_volt_check_point>=100)
	{
		bat_volt_check_point=100;					
	}
	else if(bat_volt_check_point <= 0)
	{
		bat_volt_check_point=0;	
	}
	
	if(bat_volt_check_point <= 2)
	{
		qpnp_bms_backup_smb358_checkpoint(2);
	}
	else
	{
		qpnp_bms_backup_smb358_checkpoint(bat_volt_check_point - 1);
	}
	
	return bat_volt_check_point;
}

   
static void smb358_vendor_log_init(struct smb358_charger *chip)
{
	chip->BMT_status.charger_exist = false;
	chip->BMT_status.power_type = 0;
	chip->BMT_status.charger_vol = 0;
	
	chip->BMT_status.is_charging = false;
	chip->BMT_status.charging_current = 0;
	
	chip->BMT_status.bat_exist = true;
	chip->BMT_status.bat_status = 0;
	chip->BMT_status.bat_instant_vol = 3800; 
	chip->BMT_status.temperature = 250;
	
	chip->BMT_status.bat_charging_state = POWER_SUPPLY_CHARGE_TYPE_NONE;
	chip->BMT_status.bat_temp_status = POWER_SUPPLY_HEALTH_GOOD;
	chip->BMT_status.total_charging_time = false;
	chip->BMT_status.battery_request_poweroff = 0;
	
	chip->BMT_status.SOC = bat_volt_check_point;
	chip->BMT_status.bat_volt_check_point = bat_volt_check_point;
		
}

static void smb358_vendor_print_log(struct smb358_charger *chip)
{

	dev_dbg(chip->dev,"[VENDOR CHGR] chgr_exit = %d, power_type = %d,chgr_vol = %d,is_charging = %d,charging_current = %d",
		chip->BMT_status.charger_exist, chip->BMT_status.power_type,chip->BMT_status.charger_vol,chip->BMT_status.is_charging,chip->BMT_status.charging_current);


	dev_dbg(chip->dev, "[VENDOR BAT] bat_exit = %d, bat_status = %d, bat_instant_vol = %d,bat_estimate_ocv = %d,temp = %d,soc = %d,point = %d,oppo_high_battery_status = %d",
		chip->BMT_status.bat_exist,chip->BMT_status.bat_status,chip->BMT_status.bat_instant_vol,chip->BMT_status.bat_estimate_ocv,chip->BMT_status.temperature,bat_volt_bms_soc,bat_volt_check_point,oppo_high_battery_status);


	dev_dbg(chip->dev, "[VENDOR STATUS] chging_sta = %d, temp_sta = %d,time_out = %d\n",
		chip->BMT_status.bat_charging_state,chip->BMT_status.bat_temp_status,chip->BMT_status.total_charging_time);

}

/* DONE */
static int smb358_get_prop_charge_type(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	//pr_debug(chip->dev, "%s: STATUS_C_REG=%x\n", __func__, reg);

	reg &= STATUS_C_CHARGING_MASK;

	if ((reg == STATUS_C_FAST_CHARGING) || (reg == STATUS_C_TAPER_CHARGING))
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (reg == STATUS_C_PRE_CHARGING)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int smb358_get_prop_batt_health(struct smb358_charger *chip)
{
	union power_supply_propval ret = {0, };

	if (chip->battery_missing)
	        ret.intval = POWER_SUPPLY_HEALTH_UNKNOWN;
	else if (chip->batt_hot)
		ret.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chip->batt_cold)
		ret.intval = POWER_SUPPLY_HEALTH_COLD;
	else if (chip->batt_warm)
		ret.intval = POWER_SUPPLY_HEALTH_WARM;
	else if (chip->batt_cool)
		ret.intval = POWER_SUPPLY_HEALTH_COOL;
	else
		ret.intval = POWER_SUPPLY_HEALTH_GOOD;

	return ret.intval;
}

/* TODO */
#define DEFAULT_TEMP 250
static int smb358_get_prop_batt_temp(struct smb358_charger *chip)
{
	int rc = 0;
        struct qpnp_vadc_result results;

/*        if (!smb358_get_prop_batt_present(chip))
                return DEFAULT_TEMP;
*/
        rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
        if (rc) {
                pr_debug("Unable to read batt temperature rc=%d\n", rc);
                return 0;
        }

		//pr_debug("get_bat_temp %d, %lld\n",results.adc_code, results.physical);

        return (int)results.physical;

}

static int
smb358_get_prop_battery_voltage_now(struct smb358_charger *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	/*if (chip->revision == 0 && chip->type == SMBB) {
		pr_err("vbat reading not supported for 1.0 rc=%d\n", rc);
		return 0;
	} else */{
		rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
		if (rc) {
			pr_err("Unable to read vbat rc=%d\n", rc);
			return 0;
		}
		return results.physical;
	}
}

static int
smb358_get_prop_current_now(struct smb358_charger *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
			  POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
		return ret.intval;
	} else {
		pr_debug("No BMS supply registered return 0\n");
	}

	return 0;
}

static int
smb358_get_prop_charger_voltage_now(struct smb358_charger *chip)
{
	int rc = 0;
	int V_charger = 0;
	int64_t mpp_uV= 0;
	struct qpnp_vadc_result results;

	/*if (chip->revision == 0 && chip->type == SMBB) {
		pr_err("vchg reading not supported for 1.0 rc=%d\n", rc);
		return 0;
	} else */{
		// board version_B
		rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &results);
		if (rc) {
			pr_err("Unable to read vchg rc=%d\n", rc);
			return 0;
		}
		V_charger = (int)results.physical/1000;
		
		// board version_T
		if(V_charger<1000)
		{
			rc = qpnp_vadc_read(chip->vadc_dev, P_MUX7_1_3, &results);		// P_MUX7_1_3 = adc_channel_38
			if (rc) {
				pr_err("Unable to read vchg rc=%d\n", rc);
				return 0;
			}
			mpp_uV =results.physical*102;
			V_charger = (int)mpp_uV/10000;
		}
		//pr_err("smb358 kernel read mpp_uV =%lld,V_charger =%d\n", mpp_uV,V_charger);

		return V_charger;	//return (int)results.physical/1000;
	}
}

static int smb358_get_charging_status(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb358_read_reg(chip, STATUS_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STAT_C rc = %d\n", rc);
		return 0;
	}

	return (reg & STATUS_C_CHG_ENABLE_STATUS_BIT) ? 1 : 0;
}

static int smb358_charging(struct smb358_charger *chip, int enable)
{
	int rc = 0;

	if (chip->chg_autonomous_mode) {
		dev_dbg(chip->dev, "%s: Charger in autonmous mode\n", __func__);
		return 0;
	}

	dev_dbg(chip->dev, "%s: charging enable = %d\n", __func__, enable);

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_CHG_ENABLE_BIT,
					enable ? CMD_A_CHG_ENABLE_BIT : 0);
	if (rc)
		dev_err(chip->dev, "Couldn't enable = %d rc = %d\n",
				enable, rc);
	else
		chip->charging_disabled = !enable;

	return rc;
}

bool is_chg_exist(void)
{
	return chip_smb358->chg_present;
}

int smb358_chg_suspend(struct smb358_charger *chip,int mode)
{
	int rc;
	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_CHG_SUSP_EN_MASK, mode);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't suspend rc = %d\n", rc);
	dev_dbg(chip->dev, "smb358_chg_suspend mode = %d OK, rc = %d\n", mode, rc);
	return rc;
}

static int smb358_set_input_chg_current(struct smb358_charger *chip,
							int current_ma)
{
	int i, rc = 0;
	u8 reg1 = 0, reg2 = 0, mask = 0;
	u8 val = 0;

	dev_dbg(chip->dev, "%s: USB current_ma = %d\n", __func__, current_ma);

	if (chip->chg_autonomous_mode) {
		dev_dbg(chip->dev, "%s: Charger in autonmous mode\n", __func__);
		return 0;
	}

	#if 0
	/* Only set suspend bit when chg present and current_ma = 2 */
	if (current_ma == 2 && chip->chg_present) {
		rc = smb358_masked_write(chip, CMD_A_REG,
			CMD_A_CHG_SUSP_EN_MASK, CMD_A_CHG_SUSP_EN_BIT);
		if (rc < 0)
			dev_err(chip->dev, "Couldn't suspend rc = %d\n", rc);

		return rc;
	}
	#endif
	if (current_ma <= 2)
		current_ma = USB2_MIN_CURRENT_MA;

	if (current_ma <= USB2_MIN_CURRENT_MA)
		current_ma = USB2_MAX_CURRENT_MA;

	if (current_ma == USB2_MIN_CURRENT_MA) {
		/* USB 2.0 - 100mA */
		reg1 &= ~USB3_ENABLE_BIT;
		reg2 &= ~CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma == USB2_MAX_CURRENT_MA) {
		/* USB 2.0 - 500mA */
		reg1 &= ~USB3_ENABLE_BIT;
		reg2 |= CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma == USB3_MAX_CURRENT_MA) {
		/* USB 3.0 - 900mA */
		reg1 |= USB3_ENABLE_BIT;
		reg2 |= CMD_B_CHG_USB_500_900_ENABLE_BIT;
	} else if (current_ma > USB2_MAX_CURRENT_MA) {
		/* HC mode  - if none of the above */
		reg2 |= CMD_B_CHG_HC_ENABLE_BIT;

		for (i = ARRAY_SIZE(input_current) - 1; i >= 0; i--) {
			if (input_current[i] <= current_ma)
				break;
		}
		if (i < 0) {
			dev_err(chip->dev, "Cannot find %dmA\n", current_ma);
			i = 0;
		}

		i = i << SMB358_INPUT_CURRENT_LIMIT_SHIFT;
		rc = smb358_masked_write(chip, INPUT_CURRENT_LIMIT_REG,
						AC_CHG_CURRENT_MASK, i);
		if (rc)
			dev_err(chip->dev, "Couldn't set input mA rc=%d\n", rc);
	}

	mask = CMD_B_CHG_HC_ENABLE_BIT | CMD_B_CHG_USB_500_900_ENABLE_BIT;
	rc = smb358_masked_write(chip, CMD_B_REG, mask, reg2);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set charging mode rc = %d\n", rc);

	mask = USB3_ENABLE_MASK;
	rc = smb358_masked_write(chip, SYSOK_AND_USB3_REG, mask, reg1);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't set USB3 mode rc = %d\n", rc);

	/* unset the susp bit here */
	#ifdef VENDOR_EDIT
	if (chip->multiple_test)
	        val = CMD_A_CHG_SUSP_EN_BIT;
	//else if (chip->batt_voltage_over == true)
	//        val = CMD_A_CHG_SUSP_EN_BIT;
	else if (chip->charge_voltage_over == true)
	        val = CMD_A_CHG_SUSP_EN_BIT;
	else
	        val = 0;
	#endif
	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_CHG_SUSP_EN_MASK, val);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't suspend rc = %d\n", rc);

	return rc;
}

static int smb358_set_complete_charge_timeout(struct smb358_charger *chip,
							int current_ma)
{
	int rc = 0;
	u8 val = 0;

	#ifdef OPPO_CMCC_TEST
	        val = TIME_DISABLED;
	#else
	if (current_ma > USB2_MAX_CURRENT_MA)
	        val = TIME_382MIN;
	else
	        val = TIME_764MIN;
	#endif
	val = val << TIMER_CTRL_REG_SHIFT;
	rc = smb358_masked_write(chip, STAT_AND_TIMER_CTRL_REG, TIMER_CTRL_REG_MASK, val);
	if (rc < 0)
		dev_err(chip->dev, "Couldn't complete charge timeout rc = %d\n", rc);

	return rc;
}

static int
smb358_batt_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CAPACITY:
		return 1;
	default:
		break;
	}

	return 0;
}

static int smb358_battery_set_property(struct power_supply *psy,
					enum power_supply_property prop,
					const union power_supply_propval *val)
{
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		smb358_charging(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		power_supply_changed(&chip->batt_psy);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int smb358_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = smb358_get_prop_batt_status(chip);
		chip->BMT_status.bat_status = val->intval;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb358_get_prop_batt_present(chip);
		chip->BMT_status.bat_exist = val->intval;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bat_volt_check_point;		
		chip->BMT_status.SOC = bat_volt_bms_soc;
		chip->BMT_status.bat_volt_check_point = bat_volt_check_point;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = smb358_get_charging_status(chip);
		chip->BMT_status.is_charging = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = smb358_get_prop_charge_type(chip);
		//chip->BMT_status.bat_charging_state = val->intval;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smb358_get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SMB358";
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = smb358_get_prop_batt_temp(chip);
		chip->BMT_status.temperature = val->intval;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	        //if (chip->batt_voltage_over == true)
	        //        val->intval = 4500000;
	        //else
		        val->intval = smb358_get_prop_battery_voltage_now(chip);
			chip->BMT_status.bat_instant_vol = val->intval/1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = smb358_get_prop_current_now(chip)/1000;
		chip->BMT_status.charging_current = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
	        if (chip->charge_voltage_over == true)
			{
	                val->intval = 5800;
					chip->BMT_status.charger_vol = val->intval;
			}
	        else
			{
		        val->intval = smb358_get_prop_charger_voltage_now(chip);
		        if (val->intval > 5799)
		                val->intval = 5799;
		        }
				chip->BMT_status.charger_vol = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TIMEOUT:
		val->intval = (int)chip->charging_time_out;
		chip->BMT_status.total_charging_time = val->intval;
		break;
	case POWER_SUPPLY_PROP_FAST_CHARGE:
		val->intval = 1;//(int)chip->fastcharger;
		break;
	case POWER_SUPPLY_PROP_TEMP_STATU:
		val->intval = chip->charging_smb358_temp_statu;
		chip->BMT_status.bat_temp_status =val->intval;
		break;	
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->chg_present;
		//chip->BMT_status.charger_exist = val->intval;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = power_type;
		chip->BMT_status.power_type = val->intval;		
		break;
	case POWER_SUPPLY_PROP_BATTERY_REQUEST_POWEROFF:
		val->intval = chip->BMT_status.battery_request_poweroff;
		break;
	
	default:
		return -EINVAL;
	}
	return 0;
}

static int apsd_complete(struct smb358_charger *chip, u8 status)
{
	int rc;
	u8 reg = 0;
	enum power_supply_type type = POWER_SUPPLY_TYPE_UNKNOWN;

	/*
	 * If apsd is disabled, charger detection is done by
	 * DCIN UV irq.
	 * status = ZERO - indicates charger removed, handled
	 * by DCIN UV irq
	 */
	if (chip->disable_apsd || status == 0) {
		dev_dbg(chip->dev, "APSD %s, status = %d\n",
			chip->disable_apsd ? "disabled" : "enabled", !!status);
		return 0;
	}

	rc = smb358_read_reg(chip, STATUS_D_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read STATUS D rc = %d\n", rc);
		return rc;
	}

	dev_dbg(chip->dev, "%s: STATUS_D_REG=%x\n", __func__, reg);

	switch (reg) {
	case STATUS_D_PORT_ACA_DOCK:
	case STATUS_D_PORT_ACA_C:
	case STATUS_D_PORT_ACA_B:
	case STATUS_D_PORT_ACA_A:
		type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case STATUS_D_PORT_CDP:
		type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case STATUS_D_PORT_DCP:
		type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	case STATUS_D_PORT_SDP:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	case STATUS_D_PORT_OTHER:
		type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		type = POWER_SUPPLY_TYPE_USB;
		break;
	}

	chip->chg_present = !!status;

	dev_dbg(chip->dev, "APSD complete. USB type detected=%d chg_present=%d",
						type, chip->chg_present);

	power_supply_set_charge_type(chip->usb_psy, type);

	 /* SMB is now done sampling the D+/D- lines, indicate USB driver */
	dev_dbg(chip->dev, "%s updating usb_psy present=%d", __func__,
			chip->chg_present);
	power_supply_set_present(chip->usb_psy, chip->chg_present);

	return 0;
}

static int chg_uv(struct smb358_charger *chip, u8 status)
{
    int charge_voltage;
    
    smb358_reset_charge_parameters(chip);
    
    if (status == 0){
		//set g_chg_in = 1
		g_chg_in = 1;
		//cancel_delayed_work_sync(&chip->smb358_delayed_wakeup_work);
        charge_voltage = smb358_get_prop_charger_voltage_now(chip);
		
        if (charge_voltage >= 5800){
                chip->charge_voltage_over = true;
        }
		//dev_dbg(chip->dev, "%s is call status == 0 is enter awake_lock = %d chg_in = %d\n",
		//		__func__,g_is_wakeup,g_chg_in);
		if(g_is_wakeup == 0){ //if awake not be lock,lock it here else do nothing
			//dev_dbg(chip->dev,"%s: stay awake\n",__func__);
			__pm_stay_awake(&chip->source);
			g_is_wakeup= 1;
		}		
	}else
	{
		//set g_chg_in = 0
		g_chg_in = 0;		
		schedule_delayed_work(&chip->smb358_delayed_wakeup_work,
				 round_jiffies_relative(msecs_to_jiffies(2000)));
	}
        
	/* use this to detect USB insertion only if !apsd */
	if (chip->disable_apsd && status == 0) {
		chip->chg_present = true;
		dev_dbg(chip->dev, "%s updating usb_psy present=%d",
				__func__, chip->chg_present);
		power_supply_set_supply_type(chip->usb_psy,
						POWER_SUPPLY_TYPE_USB);
		power_supply_set_present(chip->usb_psy, chip->chg_present);
	}

	if (status != 0) {
		chip->chg_present = false;
		dev_dbg(chip->dev, "%s updating usb_psy present=%d",
				__func__, chip->chg_present);
/* we can't set usb_psy as UNKNOWN so early, it'll lead USERSPACE issue */
		power_supply_set_present(chip->usb_psy, chip->chg_present);
	}
	
	shutdown_point_rst_flag = 0;
	g_soc_sync_time = 0;
	chip->BMT_status.charger_exist = chip->chg_present;
	
	power_supply_changed(chip->usb_psy);
	
	dev_dbg(chip->dev, "chip->chg_present = %d\n", chip->chg_present);

	return 0;
}

static int chg_ov(struct smb358_charger *chip, u8 status)
{
	/* disable charging? */
	u8 psy_health_sts;
	if (status) {
		psy_health_sts = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		smb358_charging(chip, false);
	} else {
		psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;
		smb358_charging(chip, true);
	}
	power_supply_set_health_state(
				chip->usb_psy, psy_health_sts);
	power_supply_changed(chip->usb_psy);

	return 0;
}
	
static int fast_chg(struct smb358_charger *chip, u8 status)
{
	power_supply_changed(&chip->batt_psy);
	dev_dbg(chip->dev, "%s\n", __func__);
	return 0;
}

static int chg_term(struct smb358_charger *chip, u8 status)
{
	dev_dbg(chip->dev, "%s\n", __func__);
	pr_debug("chg_term_status = %d\n",status);
	pr_debug("chg_term_capacity = %d\n",smb358_get_prop_batt_capacity(chip));
	pr_debug("chg_term_batt_voltage = %d\n",smb358_get_prop_battery_voltage_now(chip)/1000);
	chip->batt_pre_full = !!status;//chip->batt_full = !!status;
	//power_supply_changed(&chip->batt_psy);
	return 0;
}

#define CHG_CP_COUNT    12
static int smb358_chg_complete_check(struct smb358_charger *chip)
{
	static int chg_complete_count = 0,chg_complete_fg = 0;
        
	if (chip->batt_pre_full){
	        chg_complete_count++;
	        if (chg_complete_count > CHG_CP_COUNT){
	                chg_complete_count = CHG_CP_COUNT;
	                chip->batt_full = 1;
	                if (!chg_complete_fg){
	                        chg_complete_fg = 1;
	                        //power_supply_changed(&chip->batt_psy);
	                }
	        }
	}
	else{
	        chg_complete_fg = 0;
	        chg_complete_count = 0;
	        //chip->batt_full = 0;
	}
	
	return 0;
}

static int do_i2c_action(struct smb358_charger *chip)
{
	if (chip->batt_hot || chip->batt_cold || chip->battery_missing)
		smb358_charging(chip, false);
	else
		smb358_charging(chip, true);

	/* set the fast charge current limit */
	smb358_fastchg_current_set(chip);
	
	/* set the float voltage */
	smb358_float_voltage_set(chip, chip->temp_vfloat_mv);

	return 0;
}

/* TODO */
/* declartion */
static int battery_missing(struct smb358_charger *chip, u8 status);
#define HYSTERISIS_DECIDEGC 20
static void smb_chg_adc_notification(enum qpnp_tm_state state, void *ctx)
{
	struct smb358_charger *chip = ctx;
	bool bat_hot = 0, bat_cold = 0, bat_present = 1;
	int temp;

	if (state >= ADC_TM_STATE_NUM) {
		pr_err("invallid state parameter %d\n", state);
		return;
	}

	temp = smb358_get_prop_batt_temp(chip);

	pr_debug("temp = %d state = %s\n", temp,
				state == ADC_TM_WARM_STATE ? "hot" : "cold");

	if (state == ADC_TM_WARM_STATE) {
		if (temp > chip->hot_bat_decidegc) {
			/* warm to hot */
			bat_hot = true;
			bat_cold = false;
			bat_present = true;
			chip->charging_smb358_temp_statu = SMB358_CHG_TEMP_HOT;

			chip->adc_param.low_temp =
				chip->hot_bat_decidegc - HYSTERISIS_DECIDEGC;
			/* shall we need add high_temp here? */
			chip->adc_param.state_request =
				ADC_TM_COOL_THR_ENABLE;
		} else if (temp >=
			chip->warm_bat_decidegc) {
			/* normal to warm */
			bat_hot = false;
			bat_cold = false;
			bat_present = true;
			chip->charging_smb358_temp_statu = SMB358_CHG_TEMP_WARM;

			chip->adc_param.low_temp = chip->warm_bat_decidegc - HYSTERISIS_DECIDEGC;
			chip->adc_param.high_temp = chip->hot_bat_decidegc;
			chip->adc_param.state_request =
                                        ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp >=
			chip->little_cool_bat_decidegc) {
			/* little_cool to normal */
			bat_hot = false;
			bat_cold = false;
			bat_present = true;
			chip->charging_smb358_temp_statu = SMB358_CHG_TEMP_NORMAL;

			chip->adc_param.low_temp = chip->little_cool_bat_decidegc;
			chip->adc_param.high_temp = chip->warm_bat_decidegc;
			chip->adc_param.state_request =
                                        ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp >=
			chip->cool_bat_decidegc) {
			/* cool to little_cool */
			bat_hot = false;
			bat_cold = false;
			bat_present = true;
			chip->charging_smb358_temp_statu = SMB358_CHG_TEMP_LITTLE_COOL;

			chip->adc_param.low_temp = chip->cool_bat_decidegc;
			chip->adc_param.high_temp = chip->little_cool_bat_decidegc + HYSTERISIS_DECIDEGC;
			chip->adc_param.state_request =
                                        ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp >=
			chip->cold_bat_decidegc) {
			/* cold to cool */
			bat_hot = false;
			bat_cold = false;
			bat_present = true;
			chip->charging_smb358_temp_statu = SMB358_CHG_TEMP_COOL;

			chip->adc_param.low_temp = chip->cold_bat_decidegc;
			chip->adc_param.high_temp = chip->cool_bat_decidegc + HYSTERISIS_DECIDEGC;
			chip->adc_param.state_request =
                                        ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp >= chip->bat_present_decidegc) {
			/* Present to cold */
			bat_hot = false;
			bat_cold = true;
			bat_present = true;
			chip->charging_smb358_temp_statu = SMB358_CHG_TEMP_COLD;

                        chip->adc_param.low_temp = chip->bat_present_decidegc;
                        chip->adc_param.high_temp = chip->cold_bat_decidegc + HYSTERISIS_DECIDEGC;
			chip->adc_param.state_request =
                                        ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	} else {
		if (temp < chip->bat_present_decidegc) {
			/* Cold to present */
			bat_cold = true;
			bat_hot = false;
			bat_present = false;
			chip->charging_smb358_temp_statu = SMB358_CHG_TEMP_PRESENT;
			
			chip->adc_param.high_temp =
                                chip->bat_present_decidegc;
                        chip->adc_param.state_request =
                                ADC_TM_WARM_THR_ENABLE;
		} else if (chip->bat_present_decidegc <= temp &&
				temp < chip->cold_bat_decidegc) {
			/* cool to cold */
			bat_hot = false;
			bat_cold = true;
			bat_present = true;
			chip->charging_smb358_temp_statu = SMB358_CHG_TEMP_COLD;
			
			chip->adc_param.high_temp =
				chip->cold_bat_decidegc + HYSTERISIS_DECIDEGC;
			/* add low_temp to enable batt present check */
			chip->adc_param.low_temp = 
				chip->bat_present_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (chip->cold_bat_decidegc <= temp &&
				temp < chip->cool_bat_decidegc) {
			/* little_cool to cool */
			bat_hot = false;
			bat_cold = false;
			bat_present = true;
			chip->charging_smb358_temp_statu = SMB358_CHG_TEMP_COOL;
			
			chip->adc_param.high_temp =
				chip->cool_bat_decidegc + HYSTERISIS_DECIDEGC;
			/* add low_temp to enable batt present check */
			chip->adc_param.low_temp = 
				chip->cold_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (chip->cool_bat_decidegc <= temp &&
				temp < chip->little_cool_bat_decidegc) {
			/* normal to little_cool */
			bat_hot = false;
			bat_cold = false;
			bat_present = true;
			chip->charging_smb358_temp_statu = SMB358_CHG_TEMP_LITTLE_COOL;
			
			chip->adc_param.high_temp =
				chip->little_cool_bat_decidegc + HYSTERISIS_DECIDEGC;
			/* add low_temp to enable batt present check */
			chip->adc_param.low_temp = 
				chip->cool_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (chip->little_cool_bat_decidegc <= temp &&
				temp < chip->warm_bat_decidegc) {
			/* warm to normal */
			bat_hot = false;
			bat_cold = false;
			bat_present = true;
			chip->charging_smb358_temp_statu = SMB358_CHG_TEMP_NORMAL;
			
			chip->adc_param.high_temp =
				chip->warm_bat_decidegc;
			/* add low_temp to enable batt present check */
			chip->adc_param.low_temp = 
				chip->little_cool_bat_decidegc;
			chip->adc_param.state_request =
				ADC_TM_HIGH_LOW_THR_ENABLE;
		} else if (temp <=
				chip->hot_bat_decidegc) {
			/* hot to Warm */
			bat_hot = false;
			bat_cold = false;
			bat_present = true;
			chip->charging_smb358_temp_statu = SMB358_CHG_TEMP_WARM;

			chip->adc_param.high_temp = chip->hot_bat_decidegc;
			chip->adc_param.low_temp = chip->warm_bat_decidegc - HYSTERISIS_DECIDEGC;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	}

	if (bat_present) 
		battery_missing(chip, false);
	else
		battery_missing(chip, true);

	if (chip->charging_smb358_temp_statu == SMB358_CHG_TEMP_WARM){
	        chip->temp_vfloat_mv = chip->temp_warm_vfloat_mv;
	        chip->fastchg_current_ma = chip->temp_warm_fastchg_current_ma;
	}
	else if (chip->charging_smb358_temp_statu == SMB358_CHG_TEMP_NORMAL){
		#ifdef OPPO_BATTERY_ENCRPTION
		if(oppo_high_battery_status)
		{
			 chip->temp_vfloat_mv = chip->vfloat_mv;
			 chip->fastchg_current_ma = chip->fastchg_current_max_ma;
		}
		else
		{
			chip->temp_vfloat_mv = chip->non_standard_vfloat_mv;
			chip->fastchg_current_ma = chip->non_standard_fastchg_current_ma;
		}
		#else
		chip->temp_vfloat_mv = chip->vfloat_mv;
		chip->fastchg_current_ma = chip->fastchg_current_max_ma;
		#endif
	}
	else if (chip->charging_smb358_temp_statu == SMB358_CHG_TEMP_LITTLE_COOL){
	        chip->temp_vfloat_mv = chip->temp_little_cool_vfloat_mv;
	        chip->fastchg_current_ma = chip->temp_little_cool_fastchg_current_ma;
	}
	else if ((chip->charging_smb358_temp_statu == SMB358_CHG_TEMP_COOL))
	{
	        chip->temp_vfloat_mv = chip->temp_cool_vfloat_mv;
	        chip->fastchg_current_ma = chip->temp_cool_fastchg_current_ma;
	}
	
	if (bat_hot ^ chip->batt_hot || bat_cold ^ chip->batt_cold) {
		chip->batt_hot = bat_hot;
		chip->batt_cold = bat_cold;
	}

	if (!chip->suspending)
			do_i2c_action(chip);
	else
			chip->action_pending = true;
			
	pr_debug("hot %d, cold %d, missing %d, low = %d deciDegC, high = %d deciDegC\n",
                        chip->batt_hot, chip->batt_cold, chip->battery_missing,
                        chip->adc_param.low_temp, chip->adc_param.high_temp);
	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
                pr_err("request ADC error\n");
}

/* hot/cold function must be implemented as adc notification */
/* if using PMIC btm part, can omit these, because no irq will generate */
static int hot_hard_handler(struct smb358_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_hot = !!status;
	return 0;
}
static int cold_hard_handler(struct smb358_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_cold = !!status;
	return 0;
}
static int hot_soft_handler(struct smb358_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_warm = !!status;
	return 0;
}
static int cold_soft_handler(struct smb358_charger *chip, u8 status)
{
	pr_debug("status = 0x%02x\n", status);
	chip->batt_cool = !!status;
	return 0;
}

static int battery_missing(struct smb358_charger *chip, u8 status)
{
	if (status)
		chip->battery_missing = true;
	else
		chip->battery_missing = false;

	return 0;
}

//static int battery_ov(struct smb358_charger *chip, u8 status)
//{
//        pr_err("battery_ov\n");
//        
//	if (status)
//		chip->batt_voltage_over = true;
//	else
//		chip->batt_voltage_over = false;
//
//	return 0;
//}

static int safety_timeout(struct smb358_charger *chip, u8 status)
{
	if (status)
		chip->charging_time_out = true;
	else
		chip->charging_time_out = false;

	return 0;
}

static struct irq_handler_info handlers[] = {
	[0] = {
		.stat_reg	= IRQ_A_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "cold_soft",
				.smb_irq	= cold_soft_handler,
			},
			{
				.name		= "hot_soft",
				.smb_irq	= hot_soft_handler,
			},
			{
				.name		= "cold_hard",
				.smb_irq	= cold_hard_handler,
			},
			{
				.name		= "hot_hard",
				.smb_irq	= hot_hard_handler,
			},
		},
	},
	[1] = {
                .stat_reg       = IRQ_B_REG,
                .val            = 0,
                .prev_val       = 0,
                .irq_info       = {
                        {
                                .name           = "chg_hot",
                        },
                        {
                                .name           = "vbat_low",
                        },
                        {
                                .name           = "battery_missing",
                        //        .smb_irq        = battery_missing
                        },
                        {
                                .name           = "battery_ov",
                        //        .smb_irq        = battery_ov
                        },
                },
        },
	[2] = {
                .stat_reg       = IRQ_C_REG,
                .val            = 0,
                .prev_val       = 0,
                .irq_info       = {
                        {
                                .name           = "chg_term",
                                .smb_irq        = chg_term,
                        },
                        {
                                .name           = "taper",
                        },
                        {
                                .name           = "recharge",
                        },
                        {
                                .name           = "fast_chg",
                                .smb_irq        = fast_chg,
                        },
                },
        },
	[3] = {
		.stat_reg	= IRQ_D_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "prechg_timeout",
			},
			{
				.name		= "safety_timeout",
				.smb_irq	= safety_timeout,
			},
			{
				.name		= "aicl_complete",
			},
			{
				.name		= "src_detect",
				.smb_irq	= apsd_complete,
			},
		},
	},
	[4] = {
                .stat_reg       = IRQ_E_REG,
                .val            = 0,
                .prev_val       = 0,
                .irq_info       = {
                        {
				.name		= "usbin_uv",
                                .smb_irq        = chg_uv,
                        },
                        {
				.name		= "usbin_ov",
				.smb_irq	= chg_ov,
                        },
                        {
                                .name           = "unknown",
                        },
                        {
                                .name           = "unknown",
                        },
                },
        },
	[5] = {
		.stat_reg	= IRQ_F_REG,
		.val		= 0,
		.prev_val	= 0,
		.irq_info	= {
			{
				.name		= "power_ok",
			},
			{
				.name		= "otg_det",
			},
			{
				.name		= "otg_batt_uv",
			},
			{
				.name		= "otg_oc",
			},
		},
	},
};

#define IRQ_LATCHED_MASK	0x02
#define IRQ_STATUS_MASK		0x01
#define BITS_PER_IRQ		2
static irqreturn_t smb358_chg_stat_handler(int irq, void *dev_id)
{
	struct smb358_charger *chip = dev_id;
	int i, j;
	u8 reg = 0;
	u8 triggered;
	u8 changed;
	u8 rt_stat, prev_rt_stat;
	int rc;
	int handler_count = 0;

	rc = smb358_read_reg(chip, FAULT_INT_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read %d rc = %d\n",
				FAULT_INT_REG, rc);
	}

	rc = smb358_read_reg(chip, STATUS_INT_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read %d rc = %d\n",
				STATUS_INT_REG, rc);
	}

	rc = smb358_read_reg(chip, STATUS_D_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read %d rc = %d\n",
				STATUS_D_REG, rc);
	}

	rc = smb358_read_reg(chip, STATUS_E_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read %d rc = %d\n",
				STATUS_E_REG, rc);
	}

	for (i = 0; i < ARRAY_SIZE(handlers); i++) {
		rc = smb358_read_reg(chip, handlers[i].stat_reg,
						&handlers[i].val);
		if (rc < 0) {
			dev_err(chip->dev, "Couldn't read %d rc = %d\n",
					handlers[i].stat_reg, rc);
			continue;
		}

		for (j = 0; j < ARRAY_SIZE(handlers[i].irq_info); j++) {
			triggered = handlers[i].val
			       & (IRQ_LATCHED_MASK << (j * BITS_PER_IRQ));
			rt_stat = handlers[i].val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			prev_rt_stat = handlers[i].prev_val
				& (IRQ_STATUS_MASK << (j * BITS_PER_IRQ));
			changed = prev_rt_stat ^ rt_stat;

			if (triggered || changed) {
				pr_debug("irq %s: triggered = 0x%02x, rt_stat = 0x%02x, prev_rt_stat = 0x%02x\n",
					handlers[i].irq_info[j].name, triggered,
					rt_stat, prev_rt_stat);
				rt_stat ? handlers[i].irq_info[j].high++ :
						handlers[i].irq_info[j].low++;
			}

			if ((triggered || changed)
				&& handlers[i].irq_info[j].smb_irq != NULL) {
				handler_count++;
				rc = handlers[i].irq_info[j].smb_irq(chip,
								rt_stat);
				if (rc < 0)
					dev_err(chip->dev,
						"Couldn't handle %d irq for reg 0x%02x rc = %d\n",
						j, handlers[i].stat_reg, rc);
			}
		}
		handlers[i].prev_val = handlers[i].val;
	}

	pr_debug("handler count = %d\n", handler_count);
	if (handler_count) {
		pr_debug("batt psy changed\n");
		power_supply_changed(&chip->batt_psy);
	}


	return IRQ_HANDLED;
}

static irqreturn_t smb358_chg_valid_handler(int irq, void *dev_id)
{
	struct smb358_charger *chip = dev_id;
	int present;

	present = gpio_get_value_cansleep(chip->chg_valid_gpio);
	if (present < 0) {
		dev_err(chip->dev, "Couldn't read chg_valid gpio=%d\n",
						chip->chg_valid_gpio);
		return IRQ_HANDLED;
	}
	present ^= chip->chg_valid_act_low;

	dev_dbg(chip->dev, "%s: chg_present = %d\n", __func__, present);

	if (present != chip->chg_present) {
		chip->chg_present = present;
		dev_dbg(chip->dev, "%s updating usb_psy present=%d",
				__func__, chip->chg_present);
		power_supply_set_present(chip->usb_psy, chip->chg_present);
	}

	return IRQ_HANDLED;
}

static void smb358_external_power_changed(struct power_supply *psy)
{
	struct smb358_charger *chip = container_of(psy,
				struct smb358_charger, batt_psy);
	union power_supply_propval prop = {0,};
	int rc, current_limit = 0, online = 0;

	if (chip->bms_psy_name)
		chip->bms_psy =
			power_supply_get_by_name((char *)chip->bms_psy_name);

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_ONLINE, &prop);
	if (rc)
		dev_err(chip->dev,
			"Couldn't read USB online property, rc=%d\n", rc);
	else
		online = prop.intval;

	rc = chip->usb_psy->get_property(chip->usb_psy,
				POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (rc)
		dev_err(chip->dev,
			"Couldn't read USB current_max property, rc=%d\n", rc);
	else
		current_limit = prop.intval / 1000;

	if(current_limit > chip->limit_current_max_ma)
	        current_limit = chip->limit_current_max_ma;

	dev_dbg(chip->dev, "online = %d, current_limit = %d\n",
						online, current_limit);

	smb358_enable_volatile_writes(chip);
	smb358_set_input_chg_current(chip, current_limit);
	smb358_fastchg_current_set(chip);
	smb358_set_complete_charge_timeout(chip, current_limit);

	dev_dbg(chip->dev, "%s updating batt psy\n", __func__);
	power_supply_changed(&chip->batt_psy);
}

#define LAST_CNFG_REG	0x13
static int show_cnfg_regs(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cnfg_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_cnfg_regs, chip);
}

static const struct file_operations cnfg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cnfg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_CMD_REG	0x30
#define LAST_CMD_REG	0x33
static int show_cmd_regs(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int cmd_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_cmd_regs, chip);
}

static const struct file_operations cmd_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= cmd_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#define FIRST_STATUS_REG	0x35
#define LAST_STATUS_REG		0x3F
static int show_status_regs(struct seq_file *m, void *data)
{
	struct smb358_charger *chip = m->private;
	int rc;
	u8 reg;
	u8 addr;

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (!rc)
			seq_printf(m, "0x%02x = 0x%02x\n", addr, reg);
	}

	return 0;
}

static int status_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_status_regs, chip);
}

static const struct file_operations status_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= status_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int show_irq_count(struct seq_file *m, void *data)
{
	int i, j, total = 0;

	for (i = 0; i < ARRAY_SIZE(handlers); i++)
		for (j = 0; j < 4; j++) {
			seq_printf(m, "%s=%d\t(high=%d low=%d)\n",
						handlers[i].irq_info[j].name,
						handlers[i].irq_info[j].high
						+ handlers[i].irq_info[j].low,
						handlers[i].irq_info[j].high,
						handlers[i].irq_info[j].low);
			total += (handlers[i].irq_info[j].high
					+ handlers[i].irq_info[j].low);
		}

	seq_printf(m, "\n\tTotal = %d\n", total);

	return 0;
}

static int irq_count_debugfs_open(struct inode *inode, struct file *file)
{
	struct smb358_charger *chip = inode->i_private;

	return single_open(file, show_irq_count, chip);
}

static const struct file_operations irq_count_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= irq_count_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int get_reg(void *data, u64 *val)
{
	struct smb358_charger *chip = data;
	int rc;
	u8 temp;

	rc = smb358_read_reg(chip, chip->peek_poke_address, &temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't read reg %x rc = %d\n",
			chip->peek_poke_address, rc);
		return -EAGAIN;
	}
	*val = temp;
	return 0;
}

static int set_reg(void *data, u64 val)
{
	struct smb358_charger *chip = data;
	int rc;
	u8 temp;

	temp = (u8) val;
	rc = smb358_write_reg(chip, chip->peek_poke_address, temp);
	if (rc < 0) {
		dev_err(chip->dev,
			"Couldn't write 0x%02x to 0x%02x rc= %d\n",
			chip->peek_poke_address, temp, rc);
		return -EAGAIN;
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(poke_poke_debug_ops, get_reg, set_reg, "0x%02llx\n");

static int force_irq_set(void *data, u64 val)
{
	struct smb358_charger *chip = data;

	smb358_chg_stat_handler(chip->client->irq, data);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(force_irq_ops, NULL, force_irq_set, "0x%02llx\n");

#ifdef DEBUG
static void dump_regs(struct smb358_charger *chip)
{
	int rc;
	u8 reg;
	u8 addr;
	u8 regs1[20] = {0};//0x13
	u8 regs2[11] = {0};//0x35~0x3f
	u8 regs3[4] = {0};//0x30~0x33

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			//pr_debug("smb358_read_reg0x%02x = 0x%02x\n", addr, reg);
			regs1[addr] = reg;
	}
	pr_debug("CNFG_REG   regs1[0x00]=0x%02x,regs1[0x01]=0x%02x,regs1[0x02]=0x%02x,regs1[0x03]=0x%02x,regs1[0x04]=0x%02x,regs1[0x05]=0x%02x,regs1[0x06]=0x%02x,regs1[0x07]=0x%02x,regs1[0x08]=0x%02x,regs1[0x09]=0x%02x\n", regs1[0],regs1[1],regs1[0], regs1[3],regs1[4],regs1[5],regs1[6],regs1[7],regs1[8],regs1[9]);
	pr_debug("CNFG_REG   regs1[0x0a]=0x%02x,regs1[0x0b]=0x%02x,regs1[0x0c]=0x%02x,regs1[0x0d]=0x%02x,regs1[0x0e]=0x%02x,regs1[0x0f]=0x%02x,regs1[0x10]=0x%02x,regs1[0x11]=0x%02x,regs1[0x12]=0x%02x,regs1[0x13]=0x%02x\n", regs1[10],regs1[11],regs1[12],regs1[13],regs1[14],regs1[15],regs1[16],regs1[17],regs1[18],regs1[19]);
	
	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			//pr_debug("smb358_read_reg0x%02x = 0x%02x\n", addr, reg);
			regs2[addr-FIRST_STATUS_REG] = reg;
	}
	pr_debug("STATUS_REG regs2[0x35]=0x%02x,regs2[0x36]=0x%02x,regs2[0x37]=0x%02x,regs2[0x38]=0x%02x,regs2[0x39]=0x%02x,regs2[0x3a]=0x%02x,regs2[0x3b]=0x%02x,regs2[0x3c]=0x%02x,regs2[0x3d]=0x%02x,regs2[0x3e]=0x%02x,regs2[0x3f]=0x%02x\n",regs2[0],regs2[1],regs2[2],regs2[3],regs2[4],regs2[5],regs2[6],regs2[7],regs2[8],regs2[9],regs2[10]);

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = smb358_read_reg(chip, addr, &reg);
		if (rc)
			dev_err(chip->dev, "Couldn't read 0x%02x rc = %d\n",
					addr, rc);
		else
			//pr_debug("smb358_read_reg0x%02x = 0x%02x\n", addr, reg);
			regs3[addr-FIRST_CMD_REG] = reg;
	}
	pr_debug("CMD_REG   regs3[0x30]=0x%02x,regs3[0x31]=0x%02x,regs3[0x32]=0x%02x,regs3[0x33]=0x%02x\n",regs3[0],regs3[1],regs3[0],regs3[3]);
}
#else
static void dump_regs(struct smb358_charger *chip)
{
}
#endif

static int smb_parse_dt(struct smb358_charger *chip)
{
	int rc;
	enum of_gpio_flags gpio_flags;
	struct device_node *node = chip->dev->of_node;
	int batt_cold_degree_negative;
	int batt_present_degree_negative;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	chip->charging_disabled = of_property_read_bool(node,
					"qcom,charging-disabled");

	chip->chg_autonomous_mode = of_property_read_bool(node,
					"qcom,chg-autonomous-mode");

	chip->disable_apsd = of_property_read_bool(node, "qcom,disable-apsd");

	rc = of_property_read_string(node, "qcom,bms-psy-name",
						&chip->bms_psy_name);
	if (rc)
		chip->bms_psy_name = NULL;

	chip->chg_valid_gpio = of_get_named_gpio_flags(node,
				"qcom,chg-valid-gpio", 0, &gpio_flags);
	if (!gpio_is_valid(chip->chg_valid_gpio))
		dev_dbg(chip->dev, "Invalid chg-valid-gpio");
	else
		chip->chg_valid_act_low = gpio_flags & OF_GPIO_ACTIVE_LOW;

	#if 1//VENDOR_EDIT
	chip->stat_gpio = of_get_named_gpio_flags(node,
				"qcom,stat-gpio", 0, &gpio_flags);
	if (!gpio_is_valid(chip->stat_gpio))
		dev_dbg(chip->dev, "Invalid stat-gpio");
	#endif

	rc = of_property_read_u32(node, "qcom,fastchg-current-max-ma",
						&chip->fastchg_current_max_ma);
	if (rc)
		chip->fastchg_current_max_ma = SMB358_FAST_CHG_MAX_MA;
	chip->fastchg_current_ma = chip->fastchg_current_max_ma;

	rc = of_property_read_u32(node, "qcom,input-current-max-ma",
						&chip->limit_current_max_ma);
	if (rc)
		chip->limit_current_max_ma = SMB358_IMPUT_CURRENT_LIMIT_MAX_MA;

	chip->iterm_disabled = of_property_read_bool(node,
					"qcom,iterm-disabled");

	rc = of_property_read_u32(node, "qcom,iterm-ma", &chip->iterm_ma);
	if (rc < 0)
		chip->iterm_ma = -EINVAL;

	rc = of_property_read_u32(node, "qcom,float-voltage-mv",
						&chip->vfloat_mv);
	if (rc < 0)
		chip->vfloat_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,recharge-mv",
						&chip->recharge_mv);
	if (rc < 0)
		chip->recharge_mv = -EINVAL;

	chip->charger_inhibit_disabled = of_property_read_bool(node,
					"qcom,charger-inhibit-disabled");

	rc = of_property_read_u32(node, "qcom,hot_bat_decidegc",
						&chip->hot_bat_decidegc);
	if (rc < 0)
		chip->hot_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,warm_bat_decidegc",
						&chip->warm_bat_decidegc);
	if (rc < 0)
		chip->warm_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,little_cool_bat_decidegc",
						&chip->little_cool_bat_decidegc);
	if (rc < 0)
		chip->little_cool_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,cool_bat_decidegc",
						&chip->cool_bat_decidegc);
	if (rc < 0)
		chip->cool_bat_decidegc = -EINVAL;

	rc = of_property_read_u32(node, "qcom,cold_bat_decidegc",
						&batt_cold_degree_negative);
	if (rc < 0)
		chip->cold_bat_decidegc = -EINVAL;
	else
	        chip->cold_bat_decidegc = -batt_cold_degree_negative;

	rc = of_property_read_u32(node, "qcom,bat_present_decidegc",
						&batt_present_degree_negative);
	if (rc < 0)
		chip->bat_present_decidegc = -EINVAL;
	else
		chip->bat_present_decidegc = -batt_present_degree_negative;

	rc = of_property_read_u32(node, "qcom,temp_cool_vfloat_mv",
						&chip->temp_cool_vfloat_mv);
	if (rc < 0)
		chip->temp_cool_vfloat_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,temp_cool_fastchg_current_ma",
						&chip->temp_cool_fastchg_current_ma);
	if (rc < 0)
		chip->temp_cool_fastchg_current_ma = -EINVAL;

	rc = of_property_read_u32(node, "qcom,temp_little_cool_vfloat_mv",
						&chip->temp_little_cool_vfloat_mv);
	if (rc < 0)
		chip->temp_little_cool_vfloat_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,temp_little_cool_fastchg_current_ma",
						&chip->temp_little_cool_fastchg_current_ma);
	if (rc < 0)
		chip->temp_little_cool_fastchg_current_ma = -EINVAL;

	rc = of_property_read_u32(node, "qcom,temp_warm_vfloat_mv",
						&chip->temp_warm_vfloat_mv);
	if (rc < 0)
		chip->temp_warm_vfloat_mv = -EINVAL;

	rc = of_property_read_u32(node, "qcom,temp_warm_fastchg_current_ma",
						&chip->temp_warm_fastchg_current_ma);
	if (rc < 0)
		chip->temp_warm_fastchg_current_ma = -EINVAL;
	
	rc = of_property_read_u32(node, "qcom,non_standard_vfloat_mv",
						&chip->non_standard_vfloat_mv);
	if (rc < 0)
		chip->non_standard_vfloat_mv = -EINVAL;
	rc = of_property_read_u32(node, "qcom,non_standard_fastchg_current_ma",
						&chip->non_standard_fastchg_current_ma);
	if (rc < 0)
		chip->non_standard_fastchg_current_ma = -EINVAL;
	printk("charger_inhibit_disabled = %d, recharge-mv = %d, vfloat_mv = %d, non_standard_vfloat_mv = %d, non_standard_fastchg_current_ma = %d,iterm-disabled = %d, fastchg_current = %d, charging_disabled = %d, disable-apsd = %d bms = %s cold_bat_degree = %d, hot_bat_degree = %d, bat_present_decidegc = %d\n", chip->charger_inhibit_disabled, chip->recharge_mv, chip->vfloat_mv, chip->non_standard_vfloat_mv,chip->non_standard_fastchg_current_ma,chip->iterm_ma, chip->fastchg_current_max_ma, chip->charging_disabled, chip->disable_apsd, chip->bms_psy_name, chip->cold_bat_decidegc, chip->hot_bat_decidegc, chip->bat_present_decidegc);
	return 0;
}

static int determine_initial_state(struct smb358_charger *chip)
{
	int rc;
	u8 reg = 0;

	rc = smb358_read_reg(chip, IRQ_B_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_B rc = %d\n", rc);
		goto fail_init_status;
	}
/* Use PMIC BTM way to detect battery exist */
		
	rc = smb358_read_reg(chip, IRQ_C_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_C rc = %d\n", rc);
		goto fail_init_status;
	}
	chip->batt_full = (reg & IRQ_C_TERM_BIT) ? true : false;

	rc = smb358_read_reg(chip, IRQ_A_REG, &reg);
	if (rc < 0) {
		dev_err(chip->dev, "Couldn't read irq A rc = %d\n", rc);
		return rc;
	}

/* For current design, it'll be all OK */
	if (reg & IRQ_A_HOT_HARD_BIT)
		chip->batt_hot = true;
	if (reg & IRQ_A_COLD_HARD_BIT)
		chip->batt_cold = true;
	if (reg & IRQ_A_HOT_SOFT_BIT)
		chip->batt_warm = true;
	if (reg & IRQ_A_COLD_SOFT_BIT)
		chip->batt_cool = true;

	rc = smb358_read_reg(chip, IRQ_E_REG, &reg);
	if (rc) {
		dev_err(chip->dev, "Couldn't read IRQ_E rc = %d\n", rc);
		goto fail_init_status;
	}

	if (reg & IRQ_E_INPUT_UV_BIT) {
		chg_uv(chip, 1);
	} else {
		chg_uv(chip, 0);
		apsd_complete(chip, 1);
	}

	return 0;

fail_init_status:
	dev_err(chip->dev, "Couldn't determine intial status\n");
	return rc;
}

static void update_smb358_thread(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smb358_charger *chip = container_of(dwork,
				struct smb358_charger, update_smb358_thread_work);
				
	smb358_get_prop_batt_capacity(chip);
	#ifdef OPPO_BATTERY_ENCRPTION
	if((chip->chg_present) && ((is_project(OPPO_14013)) || (is_project(OPPO_14033))))
	{
		oppo_battery_status_check();
		if(!oppo_high_battery_status)
		{
			 smb358_float_voltage_set(chip, chip->non_standard_vfloat_mv);
			 chip->fastchg_current_ma = chip->non_standard_fastchg_current_ma;
		}
	}
	#endif
	smb358_vendor_print_log(chip);
	smb358_chg_complete_check(chip);
	power_supply_changed(&chip->batt_psy);
	
	/*update time 5s*/
	schedule_delayed_work(&chip->update_smb358_thread_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (SMB358_THREAD_INTERVAL)));
}

static void delayed_smb358_wakeup_thread(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smb358_charger *chip = container_of(dwork,
				struct smb358_charger, smb358_delayed_wakeup_work);

	//dev_dbg(chip->dev, "%s awake_lock=%d chg_in = %d\n",__func__,g_is_wakeup,g_chg_in);
	//if awake not be locked,we can`t relax it here, and if usb is present new,we can`t relax is also
	if((g_is_wakeup == 1)&&(g_chg_in == 0)){
		//dev_dbg(chip->dev,"%s: relax awake\n",__func__);
		__pm_relax(&chip->source);
		g_is_wakeup = 0;
	}
	
}


static void smb358_works_init(struct smb358_charger *chip)
{
        INIT_DELAYED_WORK(&chip->update_smb358_thread_work, update_smb358_thread);        
        schedule_delayed_work(&chip->update_smb358_thread_work,
			      round_jiffies_relative(msecs_to_jiffies
						(SMB358_THREAD_INIT)));
	INIT_DELAYED_WORK(&chip->smb358_delayed_wakeup_work, delayed_smb358_wakeup_thread);
	g_is_wakeup = 0;
}

#ifdef OPPO_BATTERY_ENCRPTION
static ssize_t show_ID_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	 pr_debug("[Battery] show_ID_status : %x\n", oppo_high_battery_status);
	return sprintf(buf, "%u\n", 1);
}
static ssize_t store_ID_status(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	char *pvalue = NULL;
	int reg_ID_status = 0;
    
	printk("dengnanwei_[Battery] store_ID_status\n");
	if(buf != NULL && size != 0)
	{
		pr_debug("[Battery] buf is %s and size is %d \n",buf,size);
		reg_ID_status = simple_strtoul(buf,&pvalue,16);
		oppo_high_battery_status = reg_ID_status;
		pr_debug("[Battery] store_ID_status : %x \n",oppo_high_battery_status);
	}		
	return size;
}
//static DEVICE_ATTR(ID_status, 0664, show_ID_status, store_ID_status);
static DEVICE_ATTR(ID_status, 0664, show_ID_status, store_ID_status);
#endif

#define SMB_I2C_VTG_MIN_UV 1800000
#define SMB_I2C_VTG_MAX_UV 1800000
static int smb358_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int rc, irq;
	struct smb358_charger *chip;
	struct power_supply *usb_psy;
	u8 reg = 0;
	int retval;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB psy not found; deferring probe\n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	chip->usb_psy = usb_psy;
	chip->fake_battery_soc = -EINVAL;

	/* early for VADC get, defer probe if needed */
	chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");
			if (IS_ERR(chip->vadc_dev)) {
				rc = PTR_ERR(chip->vadc_dev);
				if (rc != -EPROBE_DEFER)
					pr_err("vadc property missing\n");
				return rc;
                        }

	chip->adc_tm_dev = qpnp_get_adc_tm(chip->dev, "chg");
	if(IS_ERR(chip->adc_tm_dev)) {
		rc = PTR_ERR(chip->adc_tm_dev);
		if (rc != -EPROBE_DEFER)
			pr_err("adc_tm property missing\n");
		return rc;
	}

/* i2c pull up Regulator configuration */
	chip->vcc_i2c = regulator_get(&client->dev,
						"vcc_i2c_smb358");
	if (IS_ERR(chip->vcc_i2c)) {
		dev_err(&client->dev,
				"%s: Failed to get i2c regulator\n",
					__func__);
		retval = PTR_ERR(chip->vcc_i2c);
//		goto err_get_vtg_i2c;
		return -1;//retval;
	}

	if (regulator_count_voltages(chip->vcc_i2c) > 0) {
			retval = regulator_set_voltage(chip->vcc_i2c,
				SMB_I2C_VTG_MIN_UV, SMB_I2C_VTG_MAX_UV);
			if (retval) {
					dev_err(&client->dev,
					"reg set i2c vtg failed retval =%d\n",
					retval);
				goto err_set_vtg_i2c;
			}
	}

	retval = regulator_enable(chip->vcc_i2c);
	if (retval) {
		dev_err(&client->dev,
			"Regulator vcc_i2c enable failed " \
				"rc=%d\n", retval);
		return retval;
	}

	mutex_init(&chip->read_write_lock);
	/* probe the device to check if its actually connected */
	rc = smb358_read_reg(chip, INPUT_CURRENT_LIMIT_REG, &reg);
	if (rc) {
		pr_err("Failed to detect SMB358, device may be absent\n");
		return -ENODEV;
	}

	rc = smb_parse_dt(chip);
	if (rc) {
		dev_err(&client->dev, "Couldn't parse DT nodes rc=%d\n", rc);
		return rc;
	}

	smb358_reset_charge_parameters(chip);

	i2c_set_clientdata(client, chip);

	chip->batt_psy.name		= "battery";
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smb358_battery_get_property;
	chip->batt_psy.set_property	= smb358_battery_set_property;
	chip->batt_psy.property_is_writeable =
					smb358_batt_property_is_writeable;
	chip->batt_psy.properties	= smb358_battery_properties;
	chip->batt_psy.num_properties	= ARRAY_SIZE(smb358_battery_properties);
	chip->batt_psy.external_power_changed = smb358_external_power_changed;
	chip->batt_psy.supplied_to = pm_batt_supplied_to;
	chip->batt_psy.num_supplicants = ARRAY_SIZE(pm_batt_supplied_to);

	chip->fastcharger= 1;						// fast_charger  sign 

	wakeup_source_init(&chip->source, "smb358_wake");
	
	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&client->dev, "Couldn't register batt psy rc=%d\n",
				rc);
		return rc;
	}

	dump_regs(chip);
	smb358_vendor_log_init(chip);
	rc = smb358_regulator_init(chip);
	if  (rc) {
		dev_err(&client->dev,
			"Couldn't initialize smb358 ragulator rc=%d\n", rc);
		return rc;
	}

	rc = smb358_hw_init(chip);
	if (rc) {
		dev_err(&client->dev,
			"Couldn't intialize hardware rc=%d\n", rc);
		goto fail_smb358_hw_init;
	}

	smb358_works_init(chip);
	
	rc = determine_initial_state(chip);
	if (rc) {
		dev_err(&client->dev,
			"Couldn't determine initial state rc=%d\n", rc);
		goto fail_smb358_hw_init;
	}

	/* We will not use it by default */
	if (gpio_is_valid(chip->chg_valid_gpio)) {
		rc = gpio_request(chip->chg_valid_gpio, "smb358_chg_valid");
		if (rc) {
			dev_err(&client->dev,
				"gpio_request for %d failed rc=%d\n",
				chip->chg_valid_gpio, rc);
			goto fail_smb358_hw_init;
		}
		irq = gpio_to_irq(chip->chg_valid_gpio);
		if (irq < 0) {
			dev_err(&client->dev,
				"Invalid chg_valid irq = %d\n", irq);
			goto fail_chg_valid_irq;
		}
		rc = devm_request_threaded_irq(&client->dev, irq,
				NULL, smb358_chg_valid_handler,
				IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"smb358_chg_valid_irq", chip);
		if (rc) {
			dev_err(&client->dev,
				"Failed request_irq irq=%d, gpio=%d rc=%d\n",
						irq, chip->chg_valid_gpio, rc);
			goto fail_chg_valid_irq;
		}
		smb358_chg_valid_handler(irq, chip);
		enable_irq_wake(irq);
	}	

	#if 0//ndef VENDOR_EDIT
	chip->irq_gpio = of_get_named_gpio_flags(chip->dev->of_node,
				"qcom,irq-gpio", 0, NULL);

	/* add some irq gpio get verify operation */
	if (gpio_is_valid(chip->irq_gpio)) {
		retval = gpio_request(chip->irq_gpio, "smb358_irq");
		if (retval) {
			dev_err(&client->dev, "irq gpio request failed");
		}
		retval = gpio_direction_input(chip->irq_gpio);
		if (retval) {
			dev_err(&client->dev,
					"set_direction for irq gpio failed\n");
		}
	}

	/* STAT irq configuration */
	if (client->irq) {
		rc = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				smb358_chg_stat_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"smb358_chg_stat_irq", chip);
		if (rc) {
			dev_err(&client->dev,
				"Failed STAT irq=%d request rc = %d\n",
				client->irq, rc);
			goto fail_chg_valid_irq;
		}
		enable_irq_wake(client->irq);
	}
	#else
	/* STAT irq configuration */
	if (gpio_is_valid(chip->stat_gpio)) {
		rc = gpio_request(chip->stat_gpio, "smb358_stat");
		if (rc) {
			dev_err(&client->dev,
				"gpio_request for %d failed rc=%d\n",
				chip->stat_gpio, rc);
			goto fail_smb358_hw_init;
		}
		rc = gpio_direction_input(chip->stat_gpio);
		if (rc) {
			dev_err(&client->dev,
				"set_direction for stat gpio failed\n");
			goto fail_stat_irq;
		}
		client->irq = gpio_to_irq(chip->stat_gpio);
		if (client->irq < 0) {
			dev_err(&client->dev,
				"Invalid stat irq = %d\n", client->irq);
			goto fail_stat_irq;
		}
		rc = devm_request_threaded_irq(&client->dev, client->irq, NULL,
				smb358_chg_stat_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"smb358_stat_irq", chip);
		if (rc) {
			dev_err(&client->dev,
				"Failed request irq=%d request rc = %d\n",
				client->irq, rc);
			goto fail_stat_irq;
		}
		enable_irq_wake(client->irq);
	}
	#endif

	/* add hot/cold temperature monitor */
	chip->charging_smb358_temp_statu = SMB358_CHG_TEMP_NORMAL;
	chip->adc_param.low_temp = chip->little_cool_bat_decidegc;
	chip->adc_param.high_temp = chip->warm_bat_decidegc;
	chip->adc_param.timer_interval = ADC_MEAS2_INTERVAL_1S;
	chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
	chip->adc_param.btm_ctx = chip;
	chip->adc_param.threshold_notification =
				smb_chg_adc_notification;
	chip->adc_param.channel = LR_MUX1_BATT_THERM;
	/* update battery missing info in tm_channel_measure*/
	rc = qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param);
	if (rc) {
		pr_err("requesting ADC error %d\n", rc);
	}

	chip->debug_root = debugfs_create_dir("smb358", NULL);
	if (!chip->debug_root)
		dev_err(chip->dev, "Couldn't create debug dir\n");

	if (chip->debug_root) {
		struct dentry *ent;

		ent = debugfs_create_file("config_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cnfg_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create cnfg debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("status_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &status_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create status debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("cmd_registers", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &cmd_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create cmd debug file rc = %d\n",
				rc);

		ent = debugfs_create_x32("address", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root,
					  &(chip->peek_poke_address));
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create address debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("data", S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &poke_poke_debug_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("force_irq",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  chip->debug_root, chip,
					  &force_irq_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create data debug file rc = %d\n",
				rc);

		ent = debugfs_create_file("irq_count", S_IFREG | S_IRUGO,
					  chip->debug_root, chip,
					  &irq_count_debugfs_ops);
		if (!ent)
			dev_err(chip->dev,
				"Couldn't create count debug file rc = %d\n",
				rc);
	}

	dump_regs(chip);

	#ifdef VENDOR_EDIT
	if((get_boot_mode() == MSM_BOOT_MODE__RF) || (get_boot_mode() == MSM_BOOT_MODE__WLAN)) 
	{
		chip->multiple_test = 1;
		rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_CHG_SUSP_EN_MASK, CMD_A_CHG_SUSP_EN_BIT);
	}
	else
	{
		chip->multiple_test = 0;
	}	
	#endif
	#ifdef OPPO_BATTERY_ENCRPTION
	rc = device_create_file((chip->dev), &dev_attr_ID_status);
	if((is_project(OPPO_14013)) || (is_project(OPPO_14033)))
	{
		
		Gpio_BatId_Init();
		oppo_battery_status_init();
		if(!oppo_high_battery_status)
		{
			 smb358_float_voltage_set(chip, chip->non_standard_vfloat_mv);
			 chip->fastchg_current_ma = chip->non_standard_fastchg_current_ma;
		}
	}	
	#endif
	
	chip_smb358 = chip;
	dev_err(chip->dev, "SMB358 successfully probed. charger=%d, batt=%d, mode = %d,rc = %d\n",
			chip->chg_present, smb358_get_prop_batt_present(chip), get_boot_mode(),rc);
	return 0;
	
//err_get_vtg_i2c:
//	regulator_put(chip->vcc_i2c);
err_set_vtg_i2c:
	if (regulator_count_voltages(chip->vcc_i2c) > 0)
		regulator_set_voltage(chip->vcc_i2c, 0, SMB_I2C_VTG_MAX_UV);
fail_stat_irq:
	if (gpio_is_valid(chip->stat_gpio))
		gpio_free(chip->stat_gpio);
fail_chg_valid_irq:
	if (gpio_is_valid(chip->chg_valid_gpio))
		gpio_free(chip->chg_valid_gpio);
fail_smb358_hw_init:
	power_supply_unregister(&chip->batt_psy);
	regulator_unregister(chip->otg_vreg.rdev);
	return rc;
}

static int smb358_charger_remove(struct i2c_client *client)
{
	struct smb358_charger *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->batt_psy);
	if (gpio_is_valid(chip->chg_valid_gpio))
		gpio_free(chip->chg_valid_gpio);

	regulator_disable(chip->vcc_i2c);
	mutex_destroy(&chip->read_write_lock);
	debugfs_remove_recursive(chip->debug_root);
	return 0;
}

static int smb358_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);
	int rc;

	cancel_delayed_work_sync(&chip->update_smb358_thread_work);

	chip->suspending = true;
	disable_irq(client->irq);
	rc = regulator_disable(chip->vcc_i2c);
	if (rc) {
		dev_err(chip->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int smb358_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct smb358_charger *chip = i2c_get_clientdata(client);
	int rc;
	//union power_supply_propval ret = {0, };
	
	rc = regulator_enable(chip->vcc_i2c);
	if (rc) {
		dev_err(chip->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		return rc;
	}
	
	chip->suspending = false;
	enable_irq(client->irq);
	
	if (chip->action_pending) {
		do_i2c_action(chip);
		chip->action_pending = false;
	}
	
	if (chip_smb358->otg_enable_pending) {	
		chip_smb358->otg_enable_pending = false;
		smb358_chg_otg_enable();
	}
	
	if (chip_smb358->otg_disable_pending) {		
		chip_smb358->otg_disable_pending = false;
		smb358_chg_otg_disable();
	}
	
	#ifdef VENDOR_EDIT
	if (chip->bms_psy)
	{
		smb358_get_prop_batt_capacity(chip);
		power_supply_changed(&chip->batt_psy);
		//chip->bms_psy->get_property(chip->bms_psy,POWER_SUPPLY_PROP_CAPACITY, &ret);
		//bat_volt_bms_soc = ret.intval;	
		//bat_volt_check_point = bat_volt_bms_soc;
		//g_soc_sync_time = 0;
		dev_dbg(chip->dev, "smb358_resume update  bat_volt_bms_soc = %d, bat_volt_check_point = %d\r\n",bat_volt_bms_soc,bat_volt_check_point);		
	}
	#endif
	
	schedule_delayed_work(&chip->update_smb358_thread_work,
			      round_jiffies_relative(msecs_to_jiffies
						     (SMB358_THREAD_INTERVAL)));
	
	return 0;
}

static const struct dev_pm_ops smb358_pm_ops = {
	.suspend	= smb358_suspend,
	.resume		= smb358_resume,
};

static struct of_device_id smb358_match_table[] = {
	{ .compatible = "qcom,smb358-charger",},
	{ },
};

static const struct i2c_device_id smb358_charger_id[] = {
	{"smb358-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb358_charger_id);

static struct i2c_driver smb358_charger_driver = {
	.driver		= {
		.name		= "smb358-charger",
		.owner		= THIS_MODULE,
		.of_match_table	= smb358_match_table,
		.pm		= &smb358_pm_ops,
	},
	.probe		= smb358_charger_probe,
	.remove		= smb358_charger_remove,
	.id_table	= smb358_charger_id,
};

module_i2c_driver(smb358_charger_driver);

MODULE_DESCRIPTION("SMB358 Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:smb358-charger");