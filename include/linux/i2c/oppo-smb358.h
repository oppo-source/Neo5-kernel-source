/* Copyright (c) 2012 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful;
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __SMB358_H__
#define __SMB358_H__

struct qpnp_external_charger {
	int (*chg_vddmax_set) (int val);
	int (*chg_vbatdet_set) (int val);
	int (*chg_iusbmax_set) (int val);
	int (*chg_ibatmax_set) (int val);
	int (*chg_ibatterm_set) (int val);
	int (*chg_vinmin_set)(int val);
	int (*chg_charge_en) (int val);
	int (*check_charge_timeout) (int val);
	int (*chg_get_system_status) (void);
	int (*chg_usb_suspend_enable) (int val);
	int (*chg_otg_current_set) (int val);
	int (*chg_wdt_set) (int val);
	int (*chg_regs_reset) (void);
};

extern void qpnp_external_charger_register(struct qpnp_external_charger *external_charger);
extern void qpnp_external_charger_unregister(struct qpnp_external_charger *external_charger);

#define SMB358_NAME		"smb358"

/**
 * struct smb358_platform_data
 * structure to pass board specific information to the smb137b charger driver
 * @chg_current_ma:	maximum fast charge current in mA
 * @term_current_ma:	charge termination current in mA
 * @chg_en_n_gpio:	gpio to enable or disable charging
 * @chg_susp_n_gpio:	put active low to allow chip to suspend and disable I2C
 * @stat_gpio:		STAT pin, active low, '0' when charging.
 */
struct smb358_platform_data {
	int chg_en_n_gpio;
	int chg_susp_n_gpio;
	int chg_current_ma;
	int term_current_ma;
	int stat_gpio;
};

/*charge current register*/
/*bits7-bit5 for fast charge current settings */
#define CHG_CURRENT_REGISTER_ADDR		0x00
#define CHG_CURRENT__FAST_CHG_MASK 	0xe0//11100000B
#define CHG_CURRENT__FAST_CHG_SHIFT 5
#define FAST_CHG_CURRENT__200MA		0x0<<5//000B
#define FAST_CHG_CURRENT__450MA		0x1<<5//001B
#define FAST_CHG_CURRENT__600MA		0x2<<5//010B
#define FAST_CHG_CURRENT__900MA		0x3<<5//011B
#define FAST_CHG_CURRENT__1300MA		0x4<<5//100B
#define FAST_CHG_CURRENT__1500MA		0x5<<5//101B
#define FAST_CHG_CURRENT__1800MA		0x6<<5//110B
#define FAST_CHG_CURRENT__2000MA		0x7<<5//111B
/*bit4-bit3 for pre-charge current setttings*/
#define CHG_CURRENT__PRE_CHG_MASK	0x18//00011000B
#define PRE_CHG_CURRENT__150MA		0x0<<3//00B
#define PRE_CHG_CURRENT__250MA		0x1<<3//01B
#define PRE_CHG_CURRENT__350MA		0x2<<3//10B
#define PRE_CHG_CURRENT__450MA		0x3<<3//11B
/*bit2-bit0 for termination curent settings*/
#define CHG_CURRENT__TERM_CHG_MASK	0x07//00000111B
#define TERM_CHG_CURRENT__30MA		0x0<<0//000B
#define TERM_CHG_CURRENT__40MA		0x1<<0//001B
#define TERM_CHG_CURRENT__60MA		0x2<<0//010B
#define TERM_CHG_CURRENT__80MA		0x3<<0//011B
#define TERM_CHG_CURRENT__100MA		0x4<<0//100B
#define TERM_CHG_CURRENT__125MA		0x5<<0//101B
#define TERM_CHG_CURRENT__150MA		0x6<<0//110B
#define TERM_CHG_CURRENT__200MA		0x7<<0//111B


/*input curernt limit register*/
/*bit7-bit4 for max input curernt settings*/
#define INPUT_CURRENT_LIMIT_REGISTER_ADDR				0x01
#define INPUT_CURRENT_LIMIT__MAX_CHG_CURERNT_MASK	0xf0
#define INPUT_CURRENT_LIMIT__MAX_CHG_CURERNT_SHIFT	4
#define MAX_CHG_CURRENT__300MA						0x0<<4//0000B
#define MAX_CHG_CURRENT__500MA						0x1<<4//0001B
#define MAX_CHG_CURRENT__700MA						0x2<<4//0010B
#define MAX_CHG_CURRENT__1000MA						0x3<<4//0011B
#define MAX_CHG_CURRENT__1200MA						0x4<<4//0100B
#define MAX_CHG_CURRENT__1500MA						0x5<<4//0101B
#define MAX_CHG_CURRENT__1800MA						0x6<<4//0110B
#define MAX_CHG_CURRENT__2000MA						0x7<<4//0111B
#define MAX_CHG_CURRENT__NO_LIMINT					0xf<<4//1000B [1***B]
/*bit3-bit2 for charger inhibit threshold*/
#define INPUT_CURRENT_LIMINT__CHG_INHIBIT_THRESHOLD_MASK	0x0c
#define CHG_INHIBIT_THRESHOLD_VFLT_MINUS__50MV				0x0<<2//00B
#define CHG_INHIBIT_THRESHOLD_VFLT_MINUS__100MV				0x1<<2//01B
#define CHG_INHIBIT_THRESHOLD_VFLT_MINUS__200MV				0x2<<2//10B
#define CHG_INHIBIT_THRESHOLD_VFLT_MINUS__300MV				0x3<<2//11B
/*bit1 for charger inhibit control*/
#define INPUT_CURRENT_LIMINT__CHG_INHIBIT_CONTROL_MASK	0x02
#define CHG_INHIBIT_CONTROL__DISABLE						0x0<<1
#define CHG_INHIBIT_CONTROL__ENABLE						0x1<<1
/*bit0 for adc preloading control*/
#define INPUT_CURRENT_LIMINT__ADC_PRE_LOADING_MASK 		0x01
#define ADC_PRE_LOADING__DISABLE							0x0<<0
#define ADC_PRE_LOADING__ENABLE							0x1<<0


/*various functions register*/
#define VARIOUS_FUNCTIONS_REGISTER_ADDR			0x02
/*bit7 for suspend on off control*/
#define SUSPEND_ON_OFF_CONTROL__MASK			0x80
#define SUSPEND_ON_OFF_CONTROL__BY_PIN			0x0<<7//0B
#define SUSPEND_ON_OFF_CONTROL__BY_REGISTER		0x1<<7
/*bit6 for suspend mode when vbat < vbatlow*/
#define SUSPEND_MODE__MASK						0x40
#define SUSPEND_MODE__ALLOWED					0x0<<6
#define SUSPEND_MODE__BLOCKED					0x1<<6
/*bit5 for max system voltage*/
#define MAX_SYSTEM_VOLTAGE__MASK					0x20
#define MAX_SYSTEM_VOLTAGE_VFLT_PLUS__100MV	0x0<<5
#define MAX_SYSTEM_VOLTAGE_VFLT_PLUS__200MV	0x1<<5
/*bit4 for automatic input current limit control*/
#define AUTO_INPUT_CURRENT_LIMIT_CONTROL__MASK		0x10
#define AUTO_INPUT_CURRENT_LIMINT_CONTROL__DISABLE	0x0<<4
#define AUTO_INPUT_CURRENT_LIMINT_CONTROL__ENABLE	0x1<<4
/*bit3 for automatic input current limint detect threshold*/
#define AUTO_INPUT_CURRENT_LIMINT_DETECT_THRESHOLD__MASK 		0x08
#define AUTO_INPUT_CURRENT_LIMINT_DETECT_TYHRESHOLD__4200MV	0x0<<3
#define AUTO_INPUT_CURRENT_LIMINT_DETECT_TYHRESHOLD__4500MV	0x1<<3
/*bit2 for reserve*/
/*bit1 for battery over voltage control:stop or no stop charging*/
#define BATTERY_OVER_VOLTAGE_CONTROL__MASK 			0x02
#define BATTERY_OVER_VOLTAGE_CONTROL__NOT_END_CHG	0x0<<1
#define BATTERY_OVER_VOLTAGE_CONTROL__END_CHG 		0x1<<1
/*bit0 for vchg function control*/
#define VCHG_FUNCTION_CONTROL__MASK					0x01
#define VCHG_FUNCTON_CONTROL__DISABLE				0x0<<0
#define VCHG_FUNCTON_CONTROL__ENABLE				0x1<<0


/*float voltage control register*/
#define FLOAT_VOLTAGE_REGISTER_ADDR					0x03
/*bit7-bit6 for pre-chg to fast-chg voltage threshold settings*/
#define PRE_TO_FAST_CHG_VOLTAGE_THRESHOLD__MASK 	0xc0
#define PRE_TO_FAST_CHG_VOLTAGE_THRESHOLD__SHIFT    6
#define PRE_TO_FAST_CHG_VOLTAGE_THRESHOLD__2300MV	0x0<<6//00B
#define PRE_TO_FAST_CHG_VOLTAGE_THRESHOLD__2500MV	0x1<<6//01B
#define PRE_TO_FAST_CHG_VOLTAGE_THRESHOLD__2800MV	0x2<<6//10B
#define PRE_TO_FAST_CHG_VOLTAGE_THRESHOLD__3000MV	0x3<<6//11B
/*bit5-bit0 for float voltage settings*/
#define FLOAT_VOLTAGE__MASK							0x3f
#define FLOAT_VOLTAGE__SHIFT						0
#define FLOAT_VOLTAGE__BASE_VOLTAGE_MV				3500
#define FLOAT_VOLTAGE__STEP_MV						20
#define FLOAT_VOLTAGE__MAX_VOLTAGE_MV				4340


/*charge control register*/
#define CHG_CONTROL_REGISTER_ADDR			0x04
#define CHG_CONTROL_REG__MASK				0xff
/*bit7 for auto recharge control*/
#define AUTO_RECHARGE_CONTROL__MASK			0x80
#define AUTO_RECHARGE_CONTROL__ENABLE		0x0<<7
#define AUTO_RECHARGE_CONTROL__DISABLE		0x1<<7
/*bit6 for current termination control*/
#define CURRENT_TERM_CONTROL__MASK						0x40
#define CURRENT_TERM_CONTROl__ALLOW_TO_END_CHG			0x0<<6
#define CURRENT_TERM_CONTROl__NOT_ALLOW_TO_END_CHG	0x1<<6
/*bit5-bit4 for battery missing detect setting*/
#define BATTERY_MISSING_DETECT__MASK			0x30
#define BATTERY_MISSING_DETECT__DISABLE		0x0<<4//00B
#define BATTERY_MISSING_DETECT__VIA_INTERNAL_BDM_ALGORITHM_EVERY_3_SECONDS	0x1<<4//01B
#define BATTERY_MISSING_DETECT__VIA_INTERNAL_BDM_ALGORITHM		0x2<<4//10B
#define BATTERY_MISSING_DETECT__VIA_THERM_IO						0x3<<4//11B
/*bit3 for batterygood and systemok/inok output configure*/
#define BATTERYGOOD_SYSTEMOK_INOK_OUTPUT__MASK				0x08
#define BATTERYGOOD_SYSTEMOK_INOK_OUTPUT__OPEN_DRAIN		0x0<<3
#define BATTERYGOOD_SYSTEMOK_INOK_OUTPUT__PUSH_PULL		0x1<<3
/*bit2 for automatic power source detect control*/
#define AUTO_POWER_SOURCE_DETECT_CONTROL__MASK			0x04
#define AUTO_POWER_SOURCE_DETECT_CONTROL__DISABLE			0x0<<2
#define AUTO_POWER_SOURCE_DETECT_CONTROL__ENABLE			0x1<<2
/*bit1 for AICL behavior configure*/
#define AICL_BEHAVIOR_CONTROL__MASK							0x02
#define AICL_BEHAVIOR_CONTROL__CHG_DISABLE					0x0<<1
#define AICL_BEHAVIOR_CONTROL__CHG_DISABLE_INPUT_FET_OPEN	0x1<<1
/*bit0 for AICL rising edge glith filter duration*/
#define AICL_RISING_EDGE_GLITH_FILTER_DURATION__MASK			0x01
#define AICL_RISING_EDGE_GLITH_FILTER_DURATION__20MS			0x0<<0
#define AICL_RISING_EDGE_GLITH_FILTER_DURATION__15US			0x1<<0


/*state and timers control register*/
#define STAT_AND_TIMER_CONTROL_REGISTER_ADDR	0x05
#define STAT_AND_TIMER_CONTROL__MASK				0xff
/*bit7 for stat output polarity*/
#define STAT_OUTPUT_POLARITY__MASK				0x80
#define STAT_OUTPUT_POLARITY__ACTIVE_LOW			0x0<<7
#define STAT_OUTPUT_POLARITY__ACTIVE_HIGH		0x1<<7
/*bit6 for stat output mode*/
#define STAT_OUTPUT_MODE__MASK					0x40
#define STAT_OUTPUT_MODE__INDICATE_CHG_STAT		0x0<<6
#define STAT_OUTPUT_MODE__USB_FAIL				0x1<<6
/*bit5 for stat output control*/
#define STAT_OUTPUT__MASK 							0x20
#define STAT_OUTPUT__ENABLE						0x0<<5
#define STAT_OUTPUT__DISABLE						0x1<<5
/*bit4 for other charger input current limit*/
#define OTHER_CHARGER_INPUT_CURRENT_LIMIT__MASK		0x10
#define OTHER_CHARGER_INPUT_CURRENT_LIMIT__500MA	0x0<<4
#define OTHER_CHARGER_INPUT_CURRENT_LIMIT__HC		0x1<<4
/*bit3-bit2 for complete chg timeout*/
#define COMPLETE_CHG_TIMEOUT__MASK				0x0c
#define COMPLETE_CHG_TIMEOUT__382_MINUTES		0x0<<2//00B
#define COMPLETE_CHG_TIMEOUT__764_MINUTES		0x1<<2//01B
#define COMPLETE_CHG_TIMEOUT__1527_MINUTES		0x2<<2//10B
#define COMPLETE_CHG_TIMEOUT__DISABLE			0x3<<2//11B
/*bit1-bit0 for pre-chg timeout*/
#define PRE_CHG_TIMEOUT__MASK						0x03
#define PRE_CHG_TIMEOUT__48_MINUTES				0x0<<0//00B
#define PRE_CHG_TIMEOUT__95_MINUTES				0x1<<0//01B
#define PRE_CHG_TIMEOUT__191_MINUTES				0x2<<0//10B
#define PRE_CHG_TIMEOUT__DISABLE					0x3<<0//11B


/*pin and enable control register*/
#define PIN_AND_ENABLE_CONTROL_REGISTER_ADDR	0x06
#define PIN_AND_ENABLE_CONTROL__MASK				0xff
/*bit7 for led blinking function control*/
#define LED_BLINKING_FUNCTION__MASK 				0x80
#define LED_BLINKING_FUNCTION__DISABLE			0x0<<7
#define LED_BLINKING_FUNCTION__ENABLE				0x1<<7
/*bit6-bit5 for enable pin control*/
#define ENABLE_PIN_CONTROL__MASK  				0x60
#define ENABLE_PIN_CONTROL__I2C_CONTROL_DISABLE_CHG	0x0<<5
#define ENABLE_PIN_CONTROL__I2C_CONTROL_ENABLE_CHG		0x1<<5
#define ENABLE_PIN_CONTROL__ACTIVE_HIGH					0x2<<5
#define ENABLE_PIN_CONTROL__ACTIVE_LOW					0x3<<5
/*bit4 for usb5/1/HC or usb9/1.5/HC control*/
#define USB5_1_HC_OR_USB9_1P5_HC_CONTROL__MASK				0x10
#define USB5_1_HC_OR_USB9_1P5_HC_CONTROL__REGISTER_CONTROL 	0x0<<4
#define USB5_1_HC_OR_USB9_1P5_HC_CONTROL__PIN_CONTROL 		0x1<<4
/*bit3 for usb5/1/hc input state*/
#define USB5_1_HC_INPUT_STAT__MASK				0x08
#define USB5_1_HC_INPUT_STAT__TRI_STAT 			0x0<<3
#define USB5_1_HC_INPUT_STAT__DUAL_STAT 			0x1<<3
/*bit2 for charger error control*/
#define CHG_ERROR_CONTROL__MASK					0x04
#define CHG_ERROR_CONTROL__NOT_TRIGGER_IRQ		0x0<<2
#define CHG_ERROR_CONTROL__TRIGGER_IRQ			0x1<<2
/*bit1 for APSD (auto power source detect) done control*/
#define APSD_DONE_CONTROL__MASK 					0x02
#define APSD_DONE_CONTROL__NOT_TRIGGER_IRQ		0x0<<1
#define APSD_DONE_CONTROL__TRIGGER_IRQ			0x1<<1
/*bit0 for usbin input voltage bias control*/
#define USBIN_INPUT_VOLTAGE_BIAS__MASK			0x01
#define USBIN_INPUT_VOLTAGE_BIAS__DISABLE			0x0<<0
#define USBIN_INPUT_VOLTAGE_BIAS__ENABLE			0x1<<0


/*therm and system control A register*/
#define THERM_AND_SYSTEM_CONTROL_A_REGISTER_ADDR	0x07
#define THERM_AND_SYSTEM_CONTROL_A__MASK			0xff
/*bit7 for switch frequency setting*/
#define SWITCH_FREQ_CONTROL__MASK					0x80
#define SWITCH_FREQ_CONTROL__3M						0x0<<7
#define SWITCH_FREQ_CONTROL__1P5M					0x1<<7
/*bit6 for min system voltage setting*/
#define MIN_SYSTEM_VOLTAGE__MASK						0x40
#define MIN_SYSTEM_VOLTAGE__3150_OR_3450_MV			0x0<<6
#define MIN_SYSTEM_VOLTAGE__3600_OR_3750_MV			0x1<<6
/*bit5 for therm monitor seletct */
#define THERM_MONITOR_SELECT__MASK					0x20
#define THERM_MONITOR_SELECT__USBIN					0x0<<5
#define THERM_MONITOR_SELECT__VDDCAP					0x1<<5
/*bit4 for thermistor monitor control*/
#define THERMISTOR_MONITOR_CONTROL__MASK			0x10
#define THERMISTOR_MONITOR_CONTROL__ENABLE			0x0<<4
#define THERMISTOR_MONITOR_CONTROL__DISABLE			0x1<<4
/*bit3-bit2 for soft cold temperature limit behavior*/
#define SOFT_COLD_TEMP_LIMIT_BEHAVIOR__MASK			0x0c
#define SOFT_COLD_TEMP_LIMIT_BEHAVIOR__NO_RESPONSE						0x0<<2
#define SOFT_COLD_TEMP_LIMIT_BEHAVIOR__CHG_CURRENT_COMPENSATION		0x1<<2
#define SOFT_COLD_TEMP_LIMIT_BEHAVIOR__FLOAT_VOLTAGE_COMPENSATION	0x2<<2
#define SOFT_COLD_TEMP_LIMIT_BEHAVIOR__CHG_CURRENT_AND_FLOAT_VOLTAGE_COMPENSATION	0x3<<2
/*bit1-bit0 for soft hot temperature limit behavior*/
#define SOFT_HOT_TEMP_LIMIT_BEHAVIOR__MASK			0x03
#define SOFT_HOT_TEMP_LIMIT_BEHAVIOR__NO_RESPONSE	0x0<<0
#define SOFT_HOT_TEMP_LIMIT_BEHAVIOR__CHG_CURRENT_COMPENSATION 		0x1<<0
#define SOFT_HOT_TEMP_LIMIT_BEHAVIOR__FLOAT_VOLTAGE_COMPENSATION 	0x2<<0
#define SOFT_HOT_TEMP_LIMIT_BEHAVIOR__CHG_CURRENT_AND_FLOAT_VOLTAGE_COMPENSATION		0x3<<0


/*SYSOK AND USB3.0 select register*/
#define SYSOK_AND_USB3P0_REGISTER_ADDR	 				0x08
#define SYSOK_AND_USB3P0__MASK 							0xff
/*bit7-bit6 for sysok/chg_det_n output  operation*/
#define SYSOK_AND_USB3P0_OPERATION__MASK				0xc0					
#define SYSOK_AND_USB3P0_OPERATION__INOK				0x0<<6
#define SYSOK_AND_USB3P0_OPERATION__SYSOK_OPERATION_A	0x1<<6
#define SYSOK_AND_USB3P0_OPERATION__SYSOK_OPERATION_B	0x2<<6
#define SYSOK_AND_USB3P0_OPERATION__CHG_DET_N			0x3<<6
/*bit5 for usb2.0 and usb3.0 input current limit*/
#define USB2P0_AND_USB3P0_INPUT_CURRENT_LIMIT__MASK	0x20
#define USB2P0_AND_USB3P0_INPUT_CURRENT_LIMIT__USB2P0	0x0<<5
#define USB2P0_AND_USB3P0_INPUT_CURRENT_LIMIT__USB3P0	0x1<<5
/*bit4-bit3 for float voltage compensation*/
#define FLOAT_VOLTAGE_COMPENSATION__MASK				0x18
#define FLOAT_VOLTAGE_COMPENSATION__VFLT_MINUS_60MV	0x0<<3
#define FLOAT_VOLTAGE_COMPENSATION__VFLT_MINUS_120MV	0x1<<3
#define FLOAT_VOLTAGE_COMPENSATION__VFLT_MINUS_180MV	0x2<<3
#define FLOAT_VOLTAGE_COMPENSATION__VFLT_MINUS_240MV	0x3<<3
/*bit2 for hard temp limit behavior*/
#define HARD_TEMP_LIMIT_BEHAVIOR__MASK					0x04
#define HARD_TEMP_LIMIT_BEHAVIOR__SUSPEND_CHG_WHEN_BEYOND_LIMIT			0x0<<2
#define HARD_TEMP_LIMIT_BEHAVIOR__NOT_SUSPEND_CHG_WHEN_BEYOND_LIMIT	0x1<<2
/*bit1 for pre-chg to fast-chg threshold control*/
#define PRE_CHG_TO_FAST_CHG_THRESHOLD_CONTROL__MASK		0x02
#define PRE_CHG_TO_FAST_CHG_THRESHOLD_CONTROL__ENABLE	0x0<<1
#define PRE_CHG_TO_FAST_CHG_THRESHOLD_CONTROL__DISABLE	0x1<<1
/*bit0 for INOK AND BATGOOD polarity setting*/
#define INOK_AND_BATGOOD_POLARITY__MASK					0x01
#define INOK_AND_BATGOOD_POLARITY__ACTIVE_LOW			0x0<<0
#define INOK_AND_BATGOOD_POLARITY__ACTIVE_HIGH			0x1<<0


/*other control A register */
#define OTHER_CONTROL_A_REGISTER_ADDR		0x09
#define OTHER_CONTROL_A__MASK				0xff
/*bit7-bit6 for otg/id pin control*/
#define OTG_ID_PIN_CONTROL__MASK				0xc0
#define OTG_ID_PIN_CONTROL__RID_DISABLE_AND_OTG_I2C_CONTROL	0x0<<6
#define OTG_ID_PIN_CONTROL__RID_DISABLE_AND_OTG_PIN_CONTROL	0x1<<6
#define OTG_ID_PIN_CONTROL__RID_ENABLE_AND_OTG_I2C_CONTROL	0x2<<6
#define OTG_ID_PIN_CONTROL__RID_ENABLE_AND_AUTO_OTG			0x3<<6
/*bit5 for otg pin polarity*/
#define OTG_PIN_POLARITY__MASK				0x20
#define OTG_PIN_POLARITY__ACTIVE_HIGH			0x0<<5
#define OTG_PIN_POLARITY__ACTIVE_LOW			0x1<<5
/*bit4 for min system voltage, defined already*/
#define MIN_SYS_VOLTAGE__MASK 				0x20
#define MIN_SYS_VOLTAGE__3P45_OR_3P6			0x0<<4
#define MIN_SYS_VOLTAGE__3P15_OR_3P75		0x1<<4
/*bit3-bit0 for low-battery / sysok /batgood voltage threshold*/
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__MASK		0x0f
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__DISABLE	0x0<<0
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__2500MV		0x1<<0
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__2600MV		0x2<<0
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__2700MV		0x3<<0
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__2800MV		0x4<<0
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__2900MV		0x5<<0
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__3000MV		0x6<<0
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__3100MV		0x7<<0
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__3700MV		0x8<<0
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__2880MV		0x9<<0
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__3000MV_1	0xa<<0
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__3100MV_1	0xb<<0
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__3250MV		0xc<<0
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__3350MV		0xd<<0
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__3460MV		0xe<<0
#define LOW_BATTERY_SYSOK_BATGOOD_VOLTAGE_THRESHOLD__3580MV		0xf<<0


/*otg tlim therm control register*/
#define OTG_TLIM_THERM_CONTROL_REGISTER_ADDR	0x0a
#define OTG_TLIM_THERM_CONTROL__MASK				0xff
/*bit7-bit6 for chg current compensation*/
#define CHG_CURRENT_COMPENSATION__MASK			0xc0
#define CHG_CURRENT_COMPENSATION__200MA			0x0<<6
#define CHG_CURRENT_COMPENSATION__450MA			0x1<<6
#define CHG_CURRENT_COMPENSATION__600MA			0x2<<6
#define CHG_CURRENT_COMPENSATION__900MA			0x3<<6
/*bit5-bit4 for digital therm regulation temperature threshold*/
#define DIGITAL_THERM_REGULATION_TEMP_THRESHOLD__MASK 		0x30
#define DIGITAL_THERM_REGULATION_TEMP_THRESHOLD__100C		0x0<<4
#define DIGITAL_THERM_REGULATION_TEMP_THRESHOLD__110C		0x1<<4
#define DIGITAL_THERM_REGULATION_TEMP_THRESHOLD__120C		0x2<<4
#define DIGITAL_THERM_REGULATION_TEMP_THRESHOLD__130C		0x3<<4
/*bit3-bit2 for otg current limit at USBIN*/
#define OTG_CURRENT_LIMIT_AT_USBIN__MASK			0x0c
#define OTG_CURRENT_LIMIT_AT_USB_IN__250MA		0x0<<2
#define OTG_CURRENT_LIMIT_AT_USB_IN__500MA		0x1<<2
#define OTG_CURRENT_LIMIT_AT_USB_IN__750MA		0x2<<2
#define OTG_CURRENT_LIMIT_AT_USB_IN__900MA		0x3<<2
/*bit1-bit0 for otg battery uvlo(under voltage lockout) threshold*/
#define OTG_BAT_UVLO_THRESHOLD__MASK			0x03
#define OTG_BAT_UVLO_THRESHOLD__2700MV			0x0<<0
#define OTG_BAT_UVLO_THRESHOLD__2900MV			0x1<<0
#define OTG_BAT_UVLO_THRESHOLD__3100MV			0x2<<0
#define OTG_BAT_UVLO_THRESHOLD__3300MV			0x3<<0


/*hard /soft limit cell temperature monitor register*/
#define HARD_SOFT_LIMIT_TEMP_MONITOR_ADDR 		0x0b
#define HARD_SOFT_LIMIT_TEMP_MONITOR__MASK		0xff
/*bit7-bit6 for hard cold temp alram trip point*/
#define HARD_COLD_TEMP_ALARM_TRIP_POIN__MASK	0xc0
#define HARD_COLD_TEMP_ALARM_TRIP_POINT__10C	0x0<<6
#define HARD_COLD_TEMP_ALARM_TRIP_POINT__5C		0x1<<6
#define HARD_COLD_TEMP_ALARM_TRIP_POINT__0C		0x2<<6
#define HARD_COLD_TEMP_ALARM_TRIP_POINT__5C_BELOW_ZERO	0x3<<6
/*bit5-bit4 for hard hot temp alram trip point*/
#define HARD_HOT_TEMP_ALARM_TRIP_POIN__MASK		0x30
#define HARD_HOT_TEMP_ALARM_TRIP_POINT__50C		0x0<<4
#define HARD_HOT_TEMP_ALARM_TRIP_POINT__55C		0x1<<4
#define HARD_HOT_TEMP_ALARM_TRIP_POINT__60C		0x2<<4
#define HARD_HOT_TEMP_ALARM_TRIP_POINT__65C		0x3<<4
/*bit3-bit2 for soft cold temp alram trip point*/
#define SOFT_COLD_TEMP_ALARM_TRIP_POIN__MASK	0x0c
#define SOFT_COLD_TEMP_ALARM_TRIP_POINT__15C	0x0<<2
#define SOFT_COLD_TEMP_ALARM_TRIP_POINT__10C	0x1<<2
#define SOFT_COLD_TEMP_ALARM_TRIP_POINT__5C		0x2<<2
#define SOFT_COLD_TEMP_ALARM_TRIP_POINT__0C		0x3<<2
/*bit1-bit0 for soft hot temp alram trip point*/
#define SOFT_HOT_TEMP_ALARM_TRIP_POIN__MASK		0x03
#define SOFT_HOT_TEMP_ALARM_TRIP_POINT__40C		0x0<<0
#define SOFT_HOT_TEMP_ALARM_TRIP_POINT__45C		0x1<<0
#define SOFT_HOT_TEMP_ALARM_TRIP_POINT__50C		0x2<<0
#define SOFT_HOT_TEMP_ALARM_TRIP_POINT__55C		0x3<<0	


/*fault interrupt register*/
#define FAULT_INTERRUPT_REGISTER_ADDR				0x0c
#define FAULT_INTERRUPT__MASK						0xff
#define FAULT_INTERRUPT__TEMP_BEYOND_HARD_LIMIT	0x80
#define FAULT_INTERRUPT__TEMP_BEYOND_SOFT_LIMIT	0x40
#define FAULT_INTERRUPT__OTG_BAT_FAIL				0x20
#define FAULT_INTERRUPT__OTG_OVER_CURRENT_LIMIT	0x10
#define FAULT_INTERRUPT__USB_INPUT_OVER_VOLTAGE	0x08
#define FAULT_INTERRUPT__USB_INPUT_UNDER_VOLTAGE	0x04
#define FAULT_INTERRUPT__ACIL_COMPLETE			0x02
#define FAULT_INTERRUPT__INTERNAL_OVER_TEMP		0x01


/*status interrupt register*/
#define STAT_INTERRUPT_REGISTER_ADDR 				0x0d
#define STAT_INTERRUPT__MASK						0xff
#define STAT_INTERRUPT__CHG_TIME_OUT				0x80
#define STAT_INTERRUPT__OTG_INSERT_OR_REMOVED	0x40
#define STAT_INTERRUPT__BAT_OVER_VOLTAGE			0x20
#define STAT_INTERRUPT__FAST_CHG_TERM_OR_TAPPER_CHG	0x10
#define STAT_INTERRUPT__CHG_INHIBIT				0x08
#define STAT_INTERRUPT__INOK						0x04
#define STAT_INTERRUPT__MISSING_BAT				0x02
#define STAT_INTERRUPT__LOW_BAT					0x01


/*I2C BUS/SLAVE ADDR register*/
#define I2C_ADDR_REGISTER_ADDR 				0x0e
#define I2C_ADDR__SLAVE_ADDR_MASK			0xf0
#define I2C_ADDR__BUS_ADDR_MASK				0x0e
#define I2C_ADDR__WRITE_PERMISSION_MASK		0x01
#define I2C_ADDR_WRITE_PERMISSION__PREVENT	0x0
#define I2C_ADDR_WRITE_PERMISSION__ALLOW		0x1


/*command register A */
#define CMD_REG_A_REGISTER_ADDR	 				0x30
/*bit7 for config register write permission setting*/
#define CONFIG_REG_WRITE_PERMISSION__MASK		0x80
#define CONFIG_REG_WRITE_PERMISSION__PREVENT	0x0<<7
#define CONFIG_REG_WRITE_PERMISSION__ALLOW		0x1<<7
/*bit6 for fast chg setting*/
#define FAST_CHG_SETTING__MASK					0x40
#define FAST_CHG_SETTING__FORCE_PRE_CHG_CURRENT_SETTINGS	0x0<<6
#define FAST_CHG_SETTING__ALOW_FAST_CHG_CURRENT_SETTINGS	0x1<<6
/*bit5 for therm/ntc current override setting*/
#define THERM_NTC_CURRENT_OVERRIDE__MASK		0x20
#define THERM_NTC_CURRENT_OVERRIDE__ITHERM_PER_CONFIG_SETTING	0x0<<5
#define THERM_NTC_CURRENT_OVERRIDE__ENABLE		0x1<<5
/*bit4 for otg enable setting*/
#define OTG_SWITCH__MASK						0x10
#define OTG_SWITCH__DISABLE					0x0<<4
#define OTG_SWITCH__ENABLE					0x1<<4
/*bit3 for AD converter setting*/
#define AD_CONVERTER_CONTROL___MASK			0x08
#define AD_CONVERTER_CONTROL__DISABLE		0x0<<3
#define AD_CONVERTER_CONTROL__ENABLE			0x1<<3
/*bit2 for suspend mode setting*/
#define SUSPEND_MODE_CONTROL__MASK			0x04
#define SUSPEND_MODE_CONTROL__SHIFT			2
#define SUSPEND_MODE_CONTROL__DISABLE		0x0<<2
#define SUSPEND_MODE_CONTROL__ENABLE		0x1<<2
/*bit1 for charging enable setting */
#define CHG_CONTROL__MASK 		0x02
#define CHG_CONTROL__SHIFT 		1
#define CHG_CONTROL__DISABLE		0x0<<1
#define CHG_CONTROL__ENABLE		0x1<<1
/*bit0 for stat output control*/
#define STAT_OUTPUT_CONTROL__MASK		0x01
#define STAT_OUTPUT_CONTROL__DISABLE		0x0<<0
#define STAT_OUTPUT_CONTROL__ENABLE		0x1<<0


/*command register B */
#define CMD_REG_B_REGISTER_ADDR 				0x31
/*bit7 for power on reset control*/
#define POWER_ON_RESET_CONTROL__MASK		0x80
#define POWER_ON_RESET_CONTROL__NO_EFFECT	0x0<<7
#define POWER_ON_RESET_CONTROL__RESET		0x1<<7
/*bit6-bit2 undefined yet*/
/*bit1 for usb5/1(9/1.5) mode select*/
#define USB_5_1_OR_USB_9_1P5_MODE_SELECT__MASK	0x02
#define USB_5_1_OR_USB_9_1P5_MODE_SELECT__USB1_OR_USB1P5	0x0<<1
#define USB_5_1_OR_USB_9_1P5_MODE_SELECT__USB5_OR_USB9	0x1<<1
/*bit0 for usb /hc (high current)mode*/
#define USB_HC_MODE_SELECT__MASK				0x01
#define USB_HC_MODE_SELECT__USB_5_1_OR_USB_9_1P5			0x0<<0
#define USB_HC_MODE_SELECT__HC				0x1<<0


/*command register C*/
#define CMD_REG_C_REGISTER_ADDR	0x33


/*ADC STATUS register E*/
#define ADC_STAT_REGISTER_ADDR		0x34
/*bit7-bit0 for battery voltage*/
/*bat vol=value * 176 mv*/
#define BAT_VOLTAGE__MASK			0xff
#define BAT_VOLTAGE__BASE			0// 0000 0000B
#define BAT_VOLTAGE__STEP_UV		17600//uV  
#define BAT_VOLTAGE__MAX			4500//MV 1111 1111B


/*interrupt status register A*/
#define INTERRUPT_STAT_REG_A_REGISTER_ADDR	0x35
#define HOT_TEMP_HARD_LIMIT_IRQ		0x80
#define HOT_TEMP_HARD_LIMIT_STAT		0x40
#define COLD_TEMP_HARD_LIMIT_IRQ		0x20
#define COLD_TEMP_HARD_LIMIT_STAT		0x10
#define HOT_TEMP_SOFT_LIMIT_IRQ		0x08
#define HOT_TEMP_SOFT_LIMIT_STAT		0x04
#define COLD_TEMP_SOFT_LIMIT_IRQ		0x02
#define COLD_TEMP_SOFT_LIMIT_STAT		0x01


/*interrupt status register B*/
#define INTERRUPT_STAT_REG_B_REGISTER_ADDR	0x36
#define BAT_OVER_VOLTAGE_IRQ		0x80
#define BAT_OVER_VOLTAGE_STAT		0x40
#define BAT_MISSING_IRQ				0x20
#define BAT_MISSING_STAT			0x10
#define LOW_BAT_VOLTAGE_IRQ		0x08
#define LOW_BAT_VOLTAGE_STAT		0x04
#define INTERNAL_TEMP_LIMIT_IRQ	0x02
#define INTERNAL_TEMP_LIMIT_STAT	0x01


/*interrupt status register C*/
#define INTERRUPT_STAT_REG_C_REGISTER_ADDR 		0x37
#define PRE_CHG_TO_FAST_CHG_BAT_VOLTAGE_IRQ		0x80
#define PRE_CHG_TO_FAST_CHG_BAT_VOLTAGE_STAT	0x40
#define RECHG_BAT_THRESHOLD_IRQ					0x20
#define RECHG_BAT_THRESHOLD_STAT					0x10
#define TAPER_CHG_MODE_IRQ						0x08
#define TAPER_CHG_MODE_STAT						0x04
#define TERM_CHG_CURRENT_IRQ						0x02
#define TERM_CHG_CURRENT_STAT						0x01


/*interrupt status register D*/
#define INTERRUPT_STAT_REG_D_REGISTER_ADDR		0x38
#define APSD_COMPLETE_IRQ				0x80
#define APSD_COMPLETE_STAT				0x40
#define AICL_COMPLETE_IRQ				0x20
#define AICL_COMPLETE_STAT				0x10
#define COMPLETE_CHG_TIMEOUT_IRQ		0x08
#define COMPLETE_CHG_TIMEOUT_STAT	0x04
#define PRE_CHG_TIMEOUT_IRQ 			0x02
#define PRE_CHG_TIMEOUT_STAT			0x01


/*interrupt status register E*/
#define INTERRUPT_STAT_REG_E_REGISTER_ADDR	0x39
/*bit7-bit4 not defined yet*/
#define USBIN_OVER_VOLTAGE_IRQ		0x08
#define USBIN_OVER_VOLTAGE_STAT		0x04
#define USBIN_UNDER_VOLTAGE_IRQ		0x02
#define USBIN_UNDER_VOLTAGE_STAT		0x01


/*interrupt status register F*/
#define INTERRUPT_STAT_F_REGISTER_ADDR 	0x3a
#define OTG_OVER_CURRENT_LIMIT_IRQ		0x80
#define OTG_OVER_CURRENT_LIMIT_STAT		0x40
#define OTG_BAT_UNDER_VOLTAGE_IRQ		0x20
#define OTG_BAT_UNDER_VOLTAGE_STAT		0x10
#define OTG_DETECT_IRQ						0x08
#define OTG_DETECT_STAT						0x04
#define POWER_OK_IRQ						0x02
#define POWER_OK_STAT						0x01


/*status register A*/
#define STAT_REG_A_REGISTER_ADDR			0x3b
/*bit7 for therm soft limit regulation*/
#define THERM_SOFT_LIMIT_REGULATION_STAT__MASK	0x80
#define THERM_SOFT_LIMIT_REGULATION__NO_LIMIT_REGULATION	0x0<<7
#define THERM_SOFT_LIMIT_REGULATION__LIMIT_REGULATION		0x1<<7
/*bit6 for therm regulation status*/
#define THERM_REGULATION_STAT__MASK		0x40
#define THERM_REGULATION_STAT__NO_TEMP_REGULATION		0x0<<6
#define THERM_REGULATION_STAT__TEMP_REGULATION			0x1<<6
/*bit5-bit0 for actual float voltage after compesation*/
#define ACTUAL_FLT_VOLTAGE_AFTER_COMPENSATION__MASK 	0x3f
#define ACTUAL_FLT_VOLTAGE_AFTER_COMPENSATION__BASE	3500//MV 00 0000B
#define ACTUAL_FLT_VOLTAGE_AFTER_COMPENSATION__STEP	20 //mv
#define ACTUAL_FLT_VOLTAGE_AFTER_COMPENSATION__MAX 	4500//mv 11 1101[10 11]


/*status register B*/
#define STAT_REG_B_REGISTER_ADDR		0x3c
/*bit7 for usb suspend mode*/
#define USB_SUSPEND_MODE__MASK		0x80
#define USB_SUSPEND_MODE__NOT_ACTIVE	0x0<<7
#define USB_SUSPEND_MODE__ACTIVE		0x1<<7
/*bit6 not defined yet*/
/*bit4-bit3 for chg current after compemsation using *_1__mask*/
#define CHG_CURRENT_AFTER_COMPENSATION_1__MASK		0x18
#define CHG_CURRENT_AFTER_COMPENSATION_1__100MA	0x0<<3//00B
#define CHG_CURRENT_AFTER_COMPENSATION_1__150MA	0x1<<3//01B
#define CHG_CURRENT_AFTER_COMPENSATION_1__200MA	0x2<<3//10B
#define CHG_CURRENT_AFTER_COMPENSATION_1__250MA	0x3<<3//11B
/*bit5 for chg current after compensation if this bit setted means that it's another current calculate way with *_3__mask*/
#define CHG_CURRENT_AFTER_COMPENSATION_2__MASK		0x20
/*bit2-bit0 for current after compensation *_3__mask using with *_2__mask setted*/
#define CHG_CURRENT_AFTER_COMPENSATION_3__MASK		0x07
#define CHG_CURRENT_AFTER_COMPENSATION_3__100MA	0x0<<0
#define CHG_CURRENT_AFTER_COMPENSATION_3__200MA	0x1<<0
#define CHG_CURRENT_AFTER_COMPENSATION_3__450MA	0x2<<0
#define CHG_CURRENT_AFTER_COMPENSATION_3__600MA	0x3<<0
#define CHG_CURRENT_AFTER_COMPENSATION_3__900MA	0x4<<0
#define CHG_CURRENT_AFTER_COMPENSATION_3__1300MA	0x5<<0
#define CHG_CURRENT_AFTER_COMPENSATION_3__1500MA	0x6<<0
#define CHG_CURRENT_AFTER_COMPENSATION_3__1800MA	0x7<<0


/*status register C*/
#define STAT_REG_C_REGISTER_ADDR 	0x3d
/*bit7 for Charger error IRQ*/
#define CHG_ERR_IRQ__MASK			0x80
#define CHG_ERR_IRQ__NOT_ASSERT_IRQ_SIGNAL	0x0
#define CHG_ERR_IRQ__ASSERT_IRQ_SIGNAL		0x1
/*bit6 for Charger error*/
#define CHG_ERR__MASK				0x40
#define CHG_ERR__NO_CHG_ERR		0x0
#define CHG_ERR__CHG_ERR			0x1
/*bit5 for Charging status*/
#define CHG_STAT__MASK				0x20
#define CHG_STAT__NO_CHG_CYCLE_OCCURRED_AND_TERM_SINCE_CHG_FIRST_ENABLE	0x0
#define CHG_STAT__AT_LEAST_ONE_CHG_CYCLE_TERM_SINCE_CHG_FIRST_ENABLE		0x1
/*bit4 for Battery voltage level*/
#define BAT_VOLTAGE_LEVEL__MASK	0x10
#define BAT_VOLTAGE_LEVEL__MORE_THAN_2100MV	0x0
#define BAT_VOLTAGE_LEVEL__LESS_THAN_2100MV		0x1
/*bit3 for Hold-off status*/
#define HOLD_OFF_STAT__MASK		0x08
#define HOLD_OFF_STAT__NOT_IN_HOLD_OFF	0x0
#define HOLD_OFF_STAT__IN_HOLD_OFF		0x1
/*bit2-bit1 for Charging status*/
#define CHG_MODE_STAT__MASK		0x06
#define CHG_MODE_STAT__NO_CHG 	0x0
#define CHG_MODE_STAT__PRE_CHG 	0x1
#define CHG_MODE_STAT__FAST_CHG 	0x2
#define CHG_MODE_STAT__TAPER_CHG 0x3
/*bit0 for Charging enable/disable*/
#define CHG_CONTROL_STAT__MASK		0x01
#define CHG_CONTROL_STAT__DISABLED	0x0
#define CHG_CONTROL_STAT__ENABLED	0x1


/*status register D*/
#define STAT_REG_D_REGISTER_ADDR	0x3e
/*bit7-bit5 for aca status */
#define ACA_STAT__MASK				0xc0
#define ACA_STAT__A					0x0<<5
#define ACA_STAT__B					0x1<<5
#define ACA_STAT__C					0x2<<5
#define ACA_STAT__FLOAT			0x3<<5
#define ACA_STAT_GND__MASK		0x80
#define ACA_STAT_GND				0x1<<7
/*bit4 for APSD Status*/
#define APSD_STAT__MASK			0x10
#define APSD_STAT__NOT_COMPLETED	0x0<<4
#define APSD_STAT__COMPLETED		0x1<<4
/*bit3-bit0 for APSD results (charger type)*/
#define APSD_CHG_TYPE__MASK				0x0f
#define APSD_CHG_TYPE__NO_CONFIRM_CHG	0x0<<0
#define APSD_CHG_TYPE__CDP				0x1<<0
#define APSD_CHG_TYPE__DCP				0x2<<0
#define APSD_CHG_TYPE__OTHER				0x3<<0
#define APSD_CHG_TYPE__SDP				0x4<<0
#define APSD_CHG_TYPE__ACA_A				0x5<<0
#define APSD_CHG_TYPE__ACA_B				0x6<<0
#define APSD_CHG_TYPE__ACA_C				0x7<<0
#define APSD_CHG_TYPE__ACA_DOCK			0x8<<0
#define APSD_CHG_TYPE__UNREGULAR_CHG	0x9<<0


/*status register E*/
#define STAT_REG_E_REGISTER_ADDR	0x3f
/*bit7 not defined yet*/
/*bit6 -bit5for usb mode*/
#define USB_MODE__MASK			0x60
#define USB_MODE__HC				0x0<<5
#define USB_MODE__USB_1_OR_1P5	0x1<<5
#define USB_MODE__USB_5_OR_9		0x2<<5
#define USB_MODE__UNKOWN			0x3<<5
/*bit4 for aicl status */
#define AICL_STAT__MASK				0x10
#define AICL_STAT__NOT_COMPLETED	0x0<<4
#define AICL_STAT__COMPLETED		0x1<<4
/*bit3-bit0 for aicl result*/
#define AICL_RESULT__MASK			0x0f
#define AICL_RESULT__300MA			0x0<<0
#define AICL_RESULT__500MA			0x1<<0		
#define AICL_RESULT__700MA			0x2<<0
#define AICL_RESULT__900MA			0x3<<0
#define AICL_RESULT__1100MA		0x4<<0
#define AICL_RESULT__1300MA		0x5<<0
#define AICL_RESULT__1500MA		0x6<<0
#define AICL_RESULT__2000MA		0x7<<0
#endif
