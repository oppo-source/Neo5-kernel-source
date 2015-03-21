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
 #include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#define HI256_SENSOR_NAME "hi256"
#define PLATFORM_DRIVER_NAME "msm_camera_hi256"
 */
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#define HI258_SENSOR_NAME "hi258"
#define PLATFORM_DRIVER_NAME "msm_camera_hi258"
//#include <linux/productinfo.h>
#include <mach/device_info.h>
#define DEVICE_VERSION_14027_F	"Hi258"
#define DEVICE_MANUFACUTRE_14027_F	"Hynix"

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif


DEFINE_MSM_MUTEX(hi258_mut);
static struct msm_sensor_ctrl_t hi258_s_ctrl;

//lxl modify
static struct msm_sensor_power_setting hi258_power_setting[] = 
{
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
		.delay = 20,
	},	
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 20,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct msm_camera_i2c_reg_conf hi258_uxga_settings[] = {
/* 2M FULL Mode */
{0x03, 0x00},  	
{0x01, 0x01},  // HI258_Sleep_Mode
{0x11, 0x90},		
// 1600*1200	
{0x03, 0x00}, 
{0x10, 0x00}, 
{0xd2, 0x01},  //isp_div[1:0] mipi_4x_div[3:2]  mipi_1x_div[4] pll_bias_opt[7:5]    

{0x03, 0x48}, 
{0x30, 0x0c}, 
{0x31, 0x80}, 

{0x03, 0x00},  
{0x12, 0x04},  // CLK_DIV_REG

{0x03, 0x10},
{0x11, 0x03},
{0x12, 0xf0},
{0x13, 0x03},
{0x42, 0x08}, 

{0x03, 0x18},//Scaling off
{0x10, 0x00},


{0x03, 0x20},
//{0x10, 0x1c}, 
//{0x18, 0x38}, 
{0xb2, 0xc8},



//{0x83, 0x09}, //EXP Normal 10.00 fps 
//{0x84, 0xea}, 
//{0x85, 0xfc}, 
{0x86, 0x01}, //EXPMin 13800.42 fps
{0x87, 0xd7}, 
{0x88, 0x09}, //EXP Max 60hz 10.00 fps 
{0x89, 0xea}, 
{0x8a, 0xfc}, 
{0xa5, 0x09}, //EXP Max 50hz 10.00 fps 
{0xa6, 0xea}, 
{0xa7, 0xfc}, 
{0x8B, 0xfd}, //EXP100 
{0x8C, 0xe6}, 
{0x8D, 0xd3}, //EXP120 
{0x8E, 0x95}, 
{0x9c, 0x19}, //EXP Limit 985.74 fps 
{0x9d, 0xc2}, 
{0x9e, 0x01}, //EXP Unit 
{0x9f, 0xd7}, 
{0xa3, 0x00}, //Outdoor Int 
{0xa4, 0xd3}, 

{0x10, 0x9c}, 
//{0x18, 0x30}, 



{0x03,0x00},  	
{0x01, 0x00},  // HI258_Sleep_Mode

};

//zhaozhengtao modify for ROM Gesture Recognition 20140419
#if 1
static struct msm_camera_i2c_reg_conf hi258_480p_settings[] = {
/* 720*480**24-30fps */
{0x03, 0x00},  	
{0x01, 0x01},  // HI258_Sleep_Mode
{0x11, 0x94},


{0x03, 0x10},  	
{0x41, 0x08},



// 1600*1200	
{0x03, 0x00}, 
{0x10, 0x13}, 
{0xd2, 0x01},  //isp_div[1:0] mipi_4x_div[3:2]  mipi_1x_div[4] pll_bias_opt[7:5]    

{0x03, 0x18},
{0x12, 0x20},
{0x10, 0x07},
{0x11, 0x00},
{0x20, 0x02},
{0x21, 0xd0},
{0x22, 0x02},
{0x23, 0x1c},
{0x24, 0x00},
{0x25, 0x04},
{0x26, 0x00},
{0x27, 0x1e},
{0x28, 0x02},
{0x29, 0xd4},
{0x2a, 0x01},
{0x2b, 0xfe},
{0x2c, 0x08},
{0x2d, 0xe3},
{0x2e, 0x08},
{0x2f, 0xe3},
{0x30, 0x16},

{0x03, 0x20}, //Page 20
{0x18, 0x38},
{0x10, 0x1c},
{0xb2, 0xf0},
	

{0x83, 0x02}, //EXP Normal 27.33 fps 
{0x84, 0xf7}, 
{0x85, 0xb7}, 
{0x86, 0x01}, //EXPMin 24163.57 fps
{0x87, 0x0d}, 
{0x88, 0x03}, //EXP Max 60hz 27.00 fps 
{0x89, 0x4c}, 
{0x8a, 0xd4}, 
{0xa5, 0x02}, //EXP Max 50hz 27.33 fps 
{0xa6, 0xf7}, 
{0xa7, 0xb7}, 
{0x8B, 0xfd}, //EXP100 
{0x8C, 0x3d}, 
{0x8D, 0xd3}, //EXP120 
{0x8E, 0x35}, 
{0x91, 0x01}, //EXP Fix 27.03 fps
{0x92, 0xd5}, 
{0x93, 0xb3}, 
{0x9c, 0x0e}, //EXP Limit 1725.97 fps 
{0x9d, 0xb6}, 
{0x9e, 0x01}, //EXP Unit 
{0x9f, 0x0d}, 
{0xa3, 0x00}, //Outdoor Int 
{0xa4, 0xd3}, 



{0x18, 0x30},
{0x10, 0x9c},


{0x03, 0x48},
{0x30, 0x05},
{0x31, 0xa0}, //long_packet word count


{0x03, 0x00},  
{0x12, 0x04},  // CLK_DIV_REG


{0x03, 0x00},  	
{0x01, 0x00},  // HI258_Sleep_Mode

};
#else
static struct msm_camera_i2c_reg_conf hi258_480p_settings[] = {
/* 640*480**27fps */
{0x03, 0x00},  	
{0x01, 0x01},  // HI258_Sleep_Mode
{0x11, 0x94},


{0x03, 0x10},//Add for 480
{0x12, 0x30},
{0x42, 0x00},
{0x41, 0x08},


// 1600*1200	
{0x03, 0x00}, 
{0x10, 0x13}, 
{0xd2, 0x01},  //isp_div[1:0] mipi_4x_div[3:2]  mipi_1x_div[4] pll_bias_opt[7:5]    

{0x03, 0x18},
{0x10, 0x07},
{0x11, 0x00},
{0x12, 0x20},
{0x20, 0x02},
{0x21, 0x80},
{0x22, 0x01},
{0x23, 0xe0},
{0x24, 0x00},
{0x25, 0x04},
{0x26, 0x00},
{0x27, 0x00},
{0x28, 0x02},
{0x29, 0x84},
{0x2a, 0x01},
{0x2b, 0xe0},
{0x2c, 0x0a},
{0x2d, 0x00},
{0x2e, 0x0a},
{0x2f, 0x00},
{0x30, 0x41}, //41->44


{0x03, 0x20}, //Page 20
{0x18, 0x38},
{0x10, 0x1c},
{0xb2, 0xf0},
	

{0x83, 0x02}, //EXP Normal 33.33 fps 
{0x84, 0xf7}, 
{0x85, 0xb7}, 
{0x86, 0x01}, //EXPMin 24163.57 fps
{0x87, 0x0d}, 
{0x88, 0x03}, //EXP Max 60hz 30.00 fps 
{0x89, 0x4c}, 
{0x8a, 0xd4}, 
{0xa5, 0x02}, //EXP Max 50hz 33.33 fps 
{0xa6, 0xf7}, 
{0xa7, 0xb7}, 
{0x8B, 0xfd}, //EXP100 
{0x8C, 0x3d}, 
{0x8D, 0xd3}, //EXP120 
{0x8E, 0x35}, 
{0x91, 0x01}, //EXP Fix 27.03 fps
{0x92, 0xd5}, 
{0x93, 0xb3}, 
{0x9c, 0x0e}, //EXP Limit 1725.97 fps 
{0x9d, 0xb6}, 
{0x9e, 0x01}, //EXP Unit 
{0x9f, 0x0d}, 
{0xa3, 0x00}, //Outdoor Int 
{0xa4, 0xd3}, 




{0x18, 0x30},
{0x10, 0x9c},


{0x03, 0x48},
{0x30, 0x05},
{0x31, 0x00}, //long_packet word count


{0x03, 0x00},  
{0x12, 0x04},  // CLK_DIV_REG


{0x03, 0x00},  	
{0x01, 0x00},  // HI258_Sleep_Mode

};

#endif
static struct msm_camera_i2c_reg_conf hi258_start_settings[] = {
	{0x03, 0x00},
	{0x01, 0x00},
};

static struct msm_camera_i2c_reg_conf hi258_stop_settings[] = {
	{0x03, 0x00},
	{0x01, 0x01},
};

static struct msm_camera_i2c_reg_conf hi258_recommend_settings[] = {
	{0x01, 0x01}, //sleep on
{0x01, 0x03}, //sleep off
{0x01, 0x01}, //sleep on
// PAGE 20
{0x03, 0x20}, // page 20
{0x10, 0x1c}, // AE off 60hz

// PAGE 22
{0x03, 0x22}, // page 22
{0x10, 0x69}, // AWB off

{0x03, 0x00}, //Dummy 750us
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},

{0x08, 0x0f}, //131002
{0x09, 0x07}, //131002 pad strength 77->07
{0x0a, 0x00}, //131002 pad strength 07->00

//PLL Setting
{0x03, 0x00}, 
{0xd0, 0x05}, //PLL pre_div 1/6 = 4 Mhz
{0xd1, 0x34}, //PLL maim_div 
{0xd2, 0x01}, //isp_div[1:0] mipi_4x_div[3:2]  mipi_1x_div[4] pll_bias_opt[7:5]    
{0xd3, 0x20}, //isp_clk_inv[0]  mipi_4x_inv[1]  mipi_1x_inv[2]
{0xd0, 0x85},
{0xd0, 0x85},
{0xd0, 0x85},
{0xd0, 0x95},

{0x03, 0x00}, //Dummy 750us
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},
{0x03, 0x00},


///// PAGE 20 /////
{0x03, 0x20}, //page 20
{0x10, 0x1c}, //AE off 50hz

///// PAGE 22 /////
{0x03, 0x22}, //page 22
{0x10, 0x69}, //AWB off

///// Initial Start /////
///// PAGE 0 Start /////
{0x03, 0x00}, //page 0
{0x10, 0x00}, //pre1+sub1
{0x11, 0x90}, //Windowing On + 1Frame Skip
{0x12, 0x04}, //Rinsing edge 0x04 // Falling edge 0x00
{0x14, 0x05},

{0x20, 0x00}, //Row H
{0x21, 0x02}, //Row L 04
{0x22, 0x00}, //Col H
{0x23, 0x04}, //Col L  04

{0x24, 0x04}, //Window height_H //= 1200
{0x25, 0xb0}, //Window height_L //
{0x26, 0x06}, //Window width_H  //= 1600
{0x27, 0x40}, //Window wight_L

{0x28, 0x04}, //Full row start y-flip 
{0x29, 0x01}, //Pre1 row start no-flip
{0x2a, 0x02}, //Pre1 row start y-flip 

{0x2b, 0x04}, //Full wid start x-flip 
{0x2c, 0x04}, //Pre2 wid start no-flip
{0x2d, 0x02}, //Pre2 wid start x-flip 

{0x40, 0x01}, //Hblank_260
{0x41, 0x04},
{0x42, 0x00}, //Vblank4
{0x43, 0x04}, //Flick Stop

{0x50, 0x00}, //Test Pattern

///// BLC /////
{0x80, 0x2e},
{0x81, 0x7e},
{0x82, 0x90},
{0x83, 0x00},
{0x84, 0xcc}, //20130604 0x0c->0xcc
{0x85, 0x00},
{0x86, 0x00},
{0x87, 0x0f},
{0x88, 0x34},
{0x8a, 0x0b},
{0x8e, 0x80}, //Pga Blc Hold

{0x90, 0x0a}, //BLC_TIME_TH_ON
{0x91, 0x0a}, //BLC_TIME_TH_OFF 
{0x92, 0x98}, //BLC_AG_TH_ON
{0x93, 0x90}, //BLC_AG_TH_OFF

{0x96, 0xdc}, //BLC Outdoor Th On
{0x97, 0xfe}, //BLC Outdoor Th Off
{0x98, 0x38},

//OutDoor  BLC
{0x99, 0x43}, //R,Gr,B,Gb Offset

//Dark BLC
{0xa0, 0x02}, //R,Gr,B,Gb Offset

//Normal BLC
{0xa8, 0x00}, //R,Gr,B,Gb Offset
///// PAGE 0 END /////

///// PAGE 2 START /////
{0x03, 0x02},
{0x10, 0x00},
{0x13, 0x00},
{0x14, 0x00},
{0x18, 0xcc},
{0x19, 0x01}, // pmos switch on (for cfpn)
{0x1A, 0x39}, //20130604 0x09->0xcc
{0x1B, 0x00},
{0x1C, 0x1a}, // for ncp
{0x1D, 0x14}, // for ncp
{0x1E, 0x30}, // for ncp
{0x1F, 0x10},

{0x20, 0x77},
{0x21, 0xde},
{0x22, 0xa7},
{0x23, 0x30},
{0x24, 0x77},
{0x25, 0x10},
{0x26, 0x10},
{0x27, 0x3c},
{0x2b, 0x80},
{0x2c, 0x02},
{0x2d, 0x58},
{0x2e, 0x11},//20130604 0xde->0x11
{0x2f, 0xa7},//20130604 0xa7->0x11

{0x30, 0x00},
{0x31, 0x99},
{0x32, 0x00},
{0x33, 0x00},
{0x34, 0x22},
{0x36, 0x75},
{0x38, 0x88},
{0x39, 0x88},
{0x3d, 0x03},
{0x3f, 0x02},

{0x49, 0xc1},//20130604 0x87->0xd1 --> mode Change Issue modify -> 0xc1 
{0x4a, 0x10},

{0x50, 0x21},
{0x53, 0xb1},
{0x54, 0x10},
{0x55, 0x1c}, // for ncp
{0x56, 0x11},
{0x58, 0x3a},//20130604 add
{0x59, 0x38},//20130604 add
{0x5d, 0xa2},
{0x5e, 0x5a},

{0x60, 0x87},
{0x61, 0x98},
{0x62, 0x88},
{0x63, 0x96},
{0x64, 0x88},
{0x65, 0x96},
{0x67, 0x3f},
{0x68, 0x3f},
{0x69, 0x3f},

{0x72, 0x89},
{0x73, 0x95},
{0x74, 0x89},
{0x75, 0x95},
{0x7C, 0x84},
{0x7D, 0xaf},

{0x80, 0x01},
{0x81, 0x7a},
{0x82, 0x13},
{0x83, 0x24},
{0x84, 0x78},
{0x85, 0x7c},

{0x92, 0x44},
{0x93, 0x59},
{0x94, 0x78},
{0x95, 0x7c},

{0xA0, 0x02},
{0xA1, 0x74},
{0xA4, 0x74},
{0xA5, 0x02},
{0xA8, 0x85},
{0xA9, 0x8c},
{0xAC, 0x10},
{0xAD, 0x16},

{0xB0, 0x99},
{0xB1, 0xa3},
{0xB4, 0x9b},
{0xB5, 0xa2},
{0xB8, 0x9b},
{0xB9, 0x9f},
{0xBC, 0x9b},
{0xBD, 0x9f},

{0xc4, 0x29},
{0xc5, 0x40},
{0xc6, 0x5c},
{0xc7, 0x72},
{0xc8, 0x2a},
{0xc9, 0x3f},
{0xcc, 0x5d},
{0xcd, 0x71},

{0xd0, 0x10},
{0xd1, 0x14},
{0xd2, 0x20},
{0xd3, 0x00},
{0xd4, 0x0a}, //DCDC_TIME_TH_ON
{0xd5, 0x0a}, //DCDC_TIME_TH_OFF 
{0xd6, 0x98}, //DCDC_AG_TH_ON
{0xd7, 0x90}, //DCDC_AG_TH_OFF
{0xdc, 0x00},
{0xdd, 0xa3},
{0xde, 0x00},
{0xdf, 0x84},

{0xe0, 0xa4},
{0xe1, 0xa4},
{0xe2, 0xa4},
{0xe3, 0xa4},
{0xe4, 0xa4},
{0xe5, 0x01},
{0xe8, 0x00},
{0xe9, 0x00},
{0xea, 0x77},

{0xF0, 0x00},
{0xF1, 0x00},
{0xF2, 0x00},

///// PAGE 2 END /////


///// PAGE 10 START /////
{0x03, 0x10}, //page 10
{0x10, 0x03}, //S2D enable _ YUYV Order 函版
{0x11, 0x03},
{0x12, 0xf0},
{0x13, 0x03},

{0x20, 0x00},
{0x21, 0x40},
{0x22, 0x0f},
{0x24, 0x20},
{0x25, 0x10},
{0x26, 0x01},
{0x27, 0x02},
{0x28, 0x11},

{0x40, 0x88},
{0x41, 0x05}, //D-YOffset Th
{0x42, 0x08}, //Cb Offset 08
{0x43, 0x00}, //Cr Offset
{0x44, 0x80},
{0x45, 0x80},
{0x46, 0xf0},
{0x48, 0x88},
{0x4a, 0x80},

{0x50, 0xc0}, //D-YOffset AG

{0x60, 0x4f},
{0x61, 0x70}, //Sat B
{0x62, 0x7d}, //Sat R
{0x63, 0x80}, //Auto-De Color

{0x66, 0x42},
{0x67, 0x22},

{0x6a, 0x56}, //White Protection Offset Dark/Indoor
{0x74, 0x0a}, //White Protection Offset Outdoor
{0x75, 0x60}, //Sat over th
{0x76, 0x05}, //White Protection Enable
{0x78, 0xff}, //Sat over ratio
///// PAGE 10 END /////

///// PAGE 11 START /////
{0x03, 0x11}, //page 11

//LPF Auto Control
{0x20, 0x00},
{0x21, 0x00},
{0x26, 0x6a}, //pga_dark1_min (on)
{0x27, 0x68}, //pga_dark1_max (off)
{0x28, 0x0f},
{0x29, 0x10},
{0x2b, 0x30},
{0x2c, 0x32},

//GBGR 
{0x70, 0x2b},
{0x74, 0x30},
{0x75, 0x18},
{0x76, 0x30},
{0x77, 0xff},
{0x78, 0xa0},
{0x79, 0xff}, //Dark GbGr Th
{0x7a, 0x30},
{0x7b, 0x20},
{0x7c, 0xf4}, //Dark Dy Th B[7:4]
{0x7d, 0x02},
{0x7e, 0xb0},
{0x7f, 0x10},
///// PAGE 11 END /////

///// PAGE 12 START /////
{0x03, 0x12}, //page 11

//YC2D
{0x10, 0x03}, //Y DPC Enable
{0x11, 0x08}, //
{0x12, 0x10}, //0x30 -> 0x10
{0x20, 0x52}, //Y_lpf_enable53---52off
{0x21, 0x03}, //C_lpf_enable_on
{0x22, 0xe6}, //YC2D_CrCbY_Dy

{0x23, 0x14}, //Outdoor Dy Th
{0x24, 0x1b}, //Indoor Dy Th // For reso Limit 0x20
{0x25, 0x30}, //Dark Dy Th

//Outdoor LPF Flat
{0x30, 0xff}, //Y Hi Th
{0x31, 0x00}, //Y Lo Th
{0x32, 0xf0}, //Std Hi Th //Reso Improve Th Low //50
{0x33, 0x00}, //Std Lo Th
{0x34, 0x00}, //Median ratio

//Indoor LPF Flat
{0x35, 0xff}, //Y Hi Th
{0x36, 0x00}, //Y Lo Th
{0x37, 0xff}, //Std Hi Th //Reso Improve Th Low //50
{0x38, 0x00}, //Std Lo Th
{0x39, 0x00}, //Median ratio

//Dark LPF Flat
{0x3a, 0xff}, //Y Hi Th
{0x3b, 0x00}, //Y Lo Th
{0x3c, 0x93}, //Std Hi Th //Reso Improve Th Low //50
{0x3d, 0x00}, //Std Lo Th
{0x3e, 0x00}, //Median ratio

//Outdoor Cindition
{0x46, 0xa0}, //Out Lum Hi
{0x47, 0x40}, //Out Lum Lo

//Indoor Cindition
{0x4c, 0xb0}, //Indoor Lum Hi
{0x4d, 0x40}, //Indoor Lum Lo

//Dark Cindition
{0x52, 0xb0}, //Dark Lum Hi
{0x53, 0x50}, //Dark Lum Lo

//C-Filter
{0x70, 0x10}, //Outdoor(2:1) AWM Th Horizontal
{0x71, 0x0a}, //Outdoor(2:1) Diff Th Vertical
{0x72, 0x10}, //Indoor,Dark1 AWM Th Horizontal
{0x73, 0x0a}, //Indoor,Dark1 Diff Th Vertical
{0x74, 0x18}, //Dark(2:3) AWM Th Horizontal
{0x75, 0x0f}, //Dark(2:3) Diff Th Vertical

//DPC
{0x90, 0x5d},
{0x91, 0x34},
{0x99, 0x28},
{0x9c, 0x0a},
{0x9d, 0x15},
{0x9e, 0x28},
{0x9f, 0x28},
{0xb0, 0x0e}, //Zipper noise Detault change (0x75->0x0e)
{0xb8, 0x44},
{0xb9, 0x15},
///// PAGE 12 END /////

///// PAGE 13 START /////
{0x03, 0x13}, //page 13

{0x80, 0xfd}, //Sharp2D enable _ YUYV Order
{0x81, 0x07}, //Sharp2D Clip/Limit
{0x82, 0x63}, //Sharp2D Filter
{0x83, 0x00}, //Sharp2D Low Clip
{0x85, 0x00},

{0x92, 0x33}, //Sharp2D Slop n/p
{0x93, 0x30}, //Sharp2D LClip
{0x94, 0x02}, //Sharp2D HiClip1 Th
{0x95, 0xf0}, //Sharp2D HiClip2 Th
{0x96, 0x1e}, //Sharp2D HiClip2 Resolution
{0x97, 0x40}, 
{0x98, 0x80},
{0x99, 0x40},

//Sharp Lclp
{0xa2, 0x01}, //Outdoor Lclip_N
{0xa3, 0x01}, //Outdoor Lclip_P
{0xa4, 0x01}, //Indoor Lclip_N 0x03 For reso Limit 0x0e
{0xa5, 0x01}, //Indoor Lclip_P 0x0f For reso Limit 0x0f
{0xa6, 0x06}, //Dark Lclip_N
{0xa7, 0x06}, //Dark Lclip_P

//Outdoor Slope
{0xb6, 0x28}, //Lum negative Hi
{0xb7, 0x26}, //Lum negative middle
{0xb8, 0x24}, //Lum negative Low
{0xb9, 0x28}, //Lum postive Hi
{0xba, 0x26}, //Lum postive middle
{0xbb, 0x24}, //Lum postive Low

//Indoor Slope
{0xbc, 0x28}, //Lum negative Hi
{0xbd, 0x30}, //Lum negative middle
{0xbe, 0x24}, //Lum negative Low
{0xbf, 0x28}, //Lum postive Hi
{0xc0, 0x30}, //Lum postive middle
{0xc1, 0x24}, //Lum postive Low

//Dark Slope
{0xc2, 0x14}, //Lum negative Hi
{0xc3, 0x28}, //Lum negative middle
{0xc4, 0x1d}, //Lum negative Low
{0xc5, 0x14}, //Lum postive Hi
{0xc6, 0x28}, //Lum postive middle
{0xc7, 0x1d}, //Lum postive Low
///// PAGE 13 END /////

///// PAGE 14 START /////
{0x03, 0x14}, //page 14
{0x10, 0x09},

{0x20, 0x80}, //X-Center
{0x21, 0x80}, //Y-Center

{0x22, 0x78}, //LSC R 1b->15 20130125
{0x23, 0x68}, //LSC G
{0x24, 0x68}, //LSC B

{0x25, 0xf0}, //LSC Off
{0x26, 0xf0}, //LSC On
///// PAGE 14 END /////

/////// PAGE 15 START ///////

{0x03, 0x15}, //15 Page
{0x10, 0x0f},
{0x14, 0x42},	//CMCOFSGH 
{0x15, 0x32},	//CMCOFSGM
{0x16, 0x24},	//CMCOFSGL
{0x17, 0x2f},	//CMC SIGN
//CMC
{0x30, 0x8f},
{0x31, 0x59},
{0x32, 0x0a},
{0x33, 0x15},
{0x34, 0x5b}, 
{0x35, 0x06}, 
{0x36, 0x07}, 
{0x37, 0x40}, 
{0x38, 0x87},			   
//CMC OFS
{0x40, 0x92},
{0x41, 0x1b},
{0x42, 0x89},
{0x43, 0x81},
{0x44, 0x00},
{0x45, 0x01},
{0x46, 0x89},
{0x47, 0x9e},
{0x48, 0x28},
//CMC POFS
{0x50, 0x02},
{0x51, 0x82},
{0x52, 0x00},
{0x53, 0x07},
{0x54, 0x11},
{0x55, 0x98},
{0x56, 0x00},
{0x57, 0x0b},
{0x58, 0x8b},


///// PAGE 15 END /////

///// PAGE 16 START /////
{0x03, 0x16}, //page 16
{0x10, 0x31},
{0x18, 0x5a}, //Double_AG 5e->37
{0x19, 0x58}, //Double_AG 5e->36
{0x1a, 0x0e},
{0x1b, 0x01},
{0x1c, 0xdc},
{0x1d, 0xfe},

//Indoor
{0x30, 0x00},
{0x31, 0x0d},
{0x32, 0x18},
{0x33, 0x2e},
{0x34, 0x4e},
{0x35, 0x69},
{0x36, 0x7e},
{0x37, 0x90},
{0x38, 0xa2},
{0x39, 0xb1},
{0x3a, 0xbe},
{0x3b, 0xca},
{0x3c, 0xd5},
{0x3d, 0xde},
{0x3e, 0xe6},
{0x3f, 0xec},
{0x40, 0xf4},
{0x41, 0xfa},
{0x42, 0xff},


//Outdoor
{0x50, 0x00},
{0x51, 0x03},
{0x52, 0x10},
{0x53, 0x26},
{0x54, 0x43},
{0x55, 0x5d},
{0x56, 0x79},
{0x57, 0x8c},
{0x58, 0x9f},
{0x59, 0xaa},
{0x5a, 0xb6},
{0x5b, 0xc3},
{0x5c, 0xce},
{0x5d, 0xd9},
{0x5e, 0xe1},
{0x5f, 0xe9},
{0x60, 0xf0},
{0x61, 0xf4},
{0x62, 0xf5},

//Dark
{0x70, 0x00},
{0x71, 0x10},
{0x72, 0x1c},
{0x73, 0x2e},
{0x74, 0x4e},
{0x75, 0x6c},
{0x76, 0x82},
{0x77, 0x96},
{0x78, 0xa7},
{0x79, 0xb6},
{0x7a, 0xc4},
{0x7b, 0xd0},
{0x7c, 0xda},
{0x7d, 0xe2},
{0x7e, 0xea},
{0x7f, 0xf0},
{0x80, 0xf6},
{0x81, 0xfa},
{0x82, 0xff},
///// PAGE 16 END /////

///// PAGE 17 START /////
{0x03, 0x17}, //page 17
{0xc1, 0x00},
{0xc4, 0x4b},
{0xc5, 0x3f},
{0xc6, 0x02},
{0xc7, 0x20},
///// PAGE 17 END /////

///// PAGE 18 START /////
{0x03, 0x18}, //page 18
{0x10, 0x00},	//Scale Off
{0x11, 0x00},
{0x12, 0x58},
{0x13, 0x01},
{0x14, 0x00}, //Sawtooth
{0x15, 0x00},
{0x16, 0x00},
{0x17, 0x00},
{0x18, 0x00},
{0x19, 0x00},
{0x1a, 0x00},
{0x1b, 0x00},
{0x1c, 0x00},
{0x1d, 0x00},
{0x1e, 0x00},
{0x1f, 0x00},
{0x20, 0x05},	//zoom wid
{0x21, 0x00},
{0x22, 0x01},	//zoom hgt
{0x23, 0xe0},
{0x24, 0x00},	//zoom start x
{0x25, 0x00},
{0x26, 0x00},	//zoom start y
{0x27, 0x00},
{0x28, 0x05},	//zoom end x
{0x29, 0x00},
{0x2a, 0x01},	//zoom end y
{0x2b, 0xe0},
{0x2c, 0x0a},	//zoom step vert
{0x2d, 0x00},
{0x2e, 0x0a},	//zoom step horz
{0x2f, 0x00},
{0x30, 0x44},	//zoom fifo

///// PAGE 18 END /////

{0x03, 0x19}, //Page 0x19
{0x10, 0x7f}, //mcmc_ctl1
{0x11, 0x7f}, //mcmc_ctl2
{0x12, 0x1e}, //mcmc_delta1
{0x13, 0x32}, //mcmc_center1
{0x14, 0x1e}, //mcmc_delta2
{0x15, 0x6e}, //mcmc_center2
{0x16, 0x1e}, //mcmc_delta3
{0x17, 0xb8}, //mcmc_center3
{0x18, 0x1e}, //mcmc_delta4
{0x19, 0xe2}, //mcmc_center4
{0x1a, 0x9e}, //mcmc_delta5
{0x1b, 0x22}, //mcmc_center5
{0x1c, 0x9e}, //mcmc_delta6
{0x1d, 0x5e}, //mcmc_center6
{0x1e, 0x3b}, //mcmc_sat_gain1
{0x1f, 0x40}, //mcmc_sat_gain2
{0x20, 0x40}, //mcmc_sat_gain3
{0x21, 0x40}, //mcmc_sat_gain4
{0x22, 0x2f}, //mcmc_sat_gain5
{0x23, 0x37}, //mcmc_sat_gain6
{0x24, 0x00}, //mcmc_hue_angle1
{0x25, 0x07}, //mcmc_hue_angle2
{0x26, 0x1e}, //mcmc_hue_angle3
{0x27, 0x06}, //mcmc_hue_angle4
{0x28, 0x00}, //mcmc_hue_angle5
{0x29, 0x8c}, //mcmc_hue_angle6
{0x53, 0x10}, //mcmc_ctl3
{0x6c, 0xff}, //mcmc_lum_ctl1
{0x6d, 0x3f}, //mcmc_lum_ctl2
{0x6e, 0x00}, //mcmc_lum_ctl3
{0x6f, 0x00}, //mcmc_lum_ctl4
{0x70, 0x00}, //mcmc_lum_ctl5
{0x71, 0x3f}, //rg1_lum_gain_wgt_th1
{0x72, 0x3f}, //rg1_lum_gain_wgt_th2
{0x73, 0x3f}, //rg1_lum_gain_wgt_th3
{0x74, 0x3f}, //rg1_lum_gain_wgt_th4
{0x75, 0x30}, //rg1_lum_sp1
{0x76, 0x50}, //rg1_lum_sp2
{0x77, 0x80}, //rg1_lum_sp3
{0x78, 0xb0}, //rg1_lum_sp4
{0x79, 0x3f}, //rg2_gain_wgt_th1
{0x7a, 0x3f}, //rg2_gain_wgt_th2
{0x7b, 0x3f}, //rg2_gain_wgt_th3
{0x7c, 0x3f}, //rg2_gain_wgt_th4
{0x7d, 0x28}, //rg2_lum_sp1
{0x7e, 0x50}, //rg2_lum_sp2
{0x7f, 0x80}, //rg2_lum_sp3
{0x80, 0xb0}, //rg2_lum_sp4
{0x81, 0x28}, //rg3_gain_wgt_th1
{0x82, 0x3f}, //rg3_gain_wgt_th2
{0x83, 0x3f}, //rg3_gain_wgt_th3
{0x84, 0x3f}, //rg3_gain_wgt_th4
{0x85, 0x28}, //rg3_lum_sp1
{0x86, 0x50}, //rg3_lum_sp2
{0x87, 0x80}, //rg3_lum_sp3
{0x88, 0xb0}, //rg3_lum_sp4
{0x89, 0x1a}, //rg4_gain_wgt_th1
{0x8a, 0x28}, //rg4_gain_wgt_th2
{0x8b, 0x3f}, //rg4_gain_wgt_th3
{0x8c, 0x3f}, //rg4_gain_wgt_th4
{0x8d, 0x10}, //rg4_lum_sp1
{0x8e, 0x30}, //rg4_lum_sp2
{0x8f, 0x60}, //rg4_lum_sp3
{0x90, 0x90}, //rg4_lum_sp4
{0x91, 0x1a}, //rg5_gain_wgt_th1
{0x92, 0x28}, //rg5_gain_wgt_th2
{0x93, 0x3f}, //rg5_gain_wgt_th3
{0x94, 0x3f}, //rg5_gain_wgt_th4
{0x95, 0x28}, //rg5_lum_sp1
{0x96, 0x50}, //rg5_lum_sp2
{0x97, 0x80}, //rg5_lum_sp3
{0x98, 0xb0}, //rg5_lum_sp4
{0x99, 0x1a}, //rg6_gain_wgt_th1
{0x9a, 0x28}, //rg6_gain_wgt_th2
{0x9b, 0x3f}, //rg6_gain_wgt_th3
{0x9c, 0x3f}, //rg6_gain_wgt_th4
{0x9d, 0x28}, //rg6_lum_sp1
{0x9e, 0x50}, //rg6_lum_sp2
{0x9f, 0x80}, //rg6_lum_sp3
{0xa0, 0xb0}, //rg6_lum_sp4
{0xa1, 0x00}, //mcmc_allgain_ctl

{0xa2, 0x00},
{0xe5, 0x80}, //add 20120709 Bit[7] On MCMC --> YC2D_LPF

/////// PAGE 20 START ///////
{0x03, 0x20},
{0x10, 0x1c},
{0x11, 0x0c},//14
{0x18, 0x30},
{0x20, 0x65}, //8x8 Ae weight 0~7 Outdoor / Weight Outdoor On B[5]
{0x21, 0x30},
{0x22, 0x10},
{0x23, 0x00},

{0x28, 0xf7},
{0x29, 0x0d},
{0x2a, 0xff},
{0x2b, 0x04}, //Adaptive Off,1/100 Flicker

{0x2c, 0x83}, //AE After CI
{0x2d, 0xe3}, 
{0x2e, 0x13},
{0x2f, 0x0b},

{0x30, 0x78},
{0x31, 0xd7},
{0x32, 0x10},
{0x33, 0x2e},
{0x34, 0x20},
{0x35, 0xd4},
{0x36, 0xfe},
{0x37, 0x32},
{0x38, 0x04},
{0x39, 0x11},
{0x3a, 0xde},
{0x3b, 0x22},
{0x3c, 0xde},
{0x3d, 0xe1},

{0x3e, 0xc9}, //Option of changing Exp max
{0x41, 0x23}, //Option of changing Exp max

{0x50, 0x45},
{0x51, 0x88},

{0x56, 0x1f}, // for tracking
{0x57, 0xa6}, // for tracking
{0x58, 0x1a}, // for tracking
{0x59, 0x7a}, // for tracking

{0x5a, 0x04},
{0x5b, 0x04},

{0x5e, 0xc7},
{0x5f, 0x95},

{0x62, 0x10},
{0x63, 0xc0},
{0x64, 0x10},
{0x65, 0x8a},
{0x66, 0x58},
{0x67, 0x58},

{0x70, 0x40}, //6c
{0x71, 0x89}, //81(+4),89(-4)

{0x76, 0x32},
{0x77, 0xa1},
{0x78, 0x22}, //24
{0x79, 0x30}, // Y Target 70 => 25, 72 => 26 //
{0x7a, 0x23}, //23
{0x7b, 0x22}, //22
{0x7d, 0x23},

{0x03, 0x20}, //Page 20

{0x83, 0x0a}, //EXP Normal 11 fps 
{0x84, 0xe8}, 
{0x85, 0xe2}, 
{0x86, 0x01}, //EXPMin 13800.42 fps
{0x87, 0xd7}, 
{0x88, 0x0a}, //EXP Max 60hz 11 fps 
{0x89, 0xbe}, 
{0x8a, 0x91}, 
{0xa5, 0x0a}, //EXP Max 50hz 11 fps 
{0xa6, 0xe8}, 
{0xa7, 0xe2}, 
{0x8B, 0xfd}, //EXP100 
{0x8C, 0xe6}, 
{0x8D, 0xd3}, //EXP120 
{0x8E, 0x95}, 
{0x9c, 0x17}, //EXP Limit 1061.57 fps 
{0x9d, 0xeb}, 
{0x9e, 0x01}, //EXP Unit 
{0x9f, 0xd7}, 
{0xa3, 0x00}, //Outdoor Int 
{0xa4, 0xd3}, 


{0xb0, 0x80},
{0xb1, 0x14},
{0xb2, 0xc8},
{0xb3, 0x15},
{0xb4, 0x16},
{0xb5, 0x3c},
{0xb6, 0x29},
{0xb7, 0x23},
{0xb8, 0x20},
{0xb9, 0x1e},
{0xba, 0x1c},
{0xbb, 0x1b},
{0xbc, 0x1b},
{0xbd, 0x1a},

{0xc0, 0x10},
{0xc1, 0x40},
{0xc2, 0x40},
{0xc3, 0x40},
{0xc4, 0x06},

{0xc6, 0x80}, //Exp max 1frame target AG

{0xc8, 0x80},
{0xc9, 0x80},



///// PAGE 20 END /////

///// PAGE 21 START /////
{0x03, 0x21}, //page 21

{0x03, 0x21},
{0x20, 0x00},
{0x21, 0x00},
{0x22, 0x00},
{0x23, 0x00},
{0x24, 0x00},
{0x25, 0x00},
{0x26, 0x00},
{0x27, 0x00},
{0x28, 0x00},
{0x29, 0x33},
{0x2a, 0x33},
{0x2b, 0x00},
{0x2c, 0x00},
{0x2d, 0x33},
{0x2e, 0x33},
{0x2f, 0x00},
{0x30, 0x00},
{0x31, 0x33},
{0x32, 0x33},
{0x33, 0x00},
{0x34, 0x00},
{0x35, 0x33},
{0x36, 0x33},
{0x37, 0x00},
{0x38, 0x00},
{0x39, 0x00},
{0x3a, 0x00},
{0x3b, 0x00},
{0x3c, 0x00},
{0x3d, 0x00},
{0x3e, 0x00},
{0x3f, 0x00},
{0x40, 0x00},
{0x41, 0x00},
{0x42, 0x00},
{0x43, 0x00},
{0x44, 0x00},
{0x45, 0x00},
{0x46, 0x00},
{0x47, 0x00},
{0x48, 0x00},
{0x49, 0x33},
{0x4a, 0x33},
{0x4b, 0x00},
{0x4c, 0x00},
{0x4d, 0x33},
{0x4e, 0x33},
{0x4f, 0x00},
{0x50, 0x00},
{0x51, 0x33},
{0x52, 0x33},
{0x53, 0x00},
{0x54, 0x00},
{0x55, 0x33},
{0x56, 0x33},
{0x57, 0x00},
{0x58, 0x00},
{0x59, 0x00},
{0x5a, 0x00},
{0x5b, 0x00},
{0x5c, 0x00},
{0x5d, 0x00},
{0x5e, 0x00},
{0x5f, 0x00},


///// PAGE 22 START /////
{0x03, 0x22}, //page 22
{0x10, 0xfd},
{0x11, 0x2e},
{0x19, 0x00},
{0x20, 0x30}, //For AWB Speed
{0x21, 0x80},
{0x22, 0x00},
{0x23, 0x00},
{0x24, 0x01},
{0x25, 0x4f}, //2013-09-13 AWB Hunting

{0x30, 0x80},
{0x31, 0x80},
{0x38, 0x11},
{0x39, 0x34},
{0x40, 0xe4}, //Stb Yth
{0x41, 0x33}, //Stb cdiff
{0x42, 0x22}, //Stb csum
{0x43, 0xf3}, //Unstb Yth
{0x44, 0x33}, //Unstb cdiff55
{0x45, 0x33}, //Unstb csum
{0x46, 0x00},
{0x47, 0x09}, //2013-09-13 AWB Hunting
{0x48, 0x00}, //2013-09-13 AWB Hunting
{0x49, 0x0a},

{0x60, 0x04},
{0x61, 0xc4},
{0x62, 0x04},
{0x63, 0x92},
{0x66, 0x04},
{0x67, 0xc4},
{0x68, 0x04},
{0x69, 0x92},

{0x80, 0x36},
{0x81, 0x20},
{0x82, 0x2a},

{0x83, 0x58},
{0x84, 0x16},
{0x85, 0x4f},
{0x86, 0x20},

{0x87, 0x3a}, //3b->46 awb_r_gain_max middle
{0x88, 0x30}, //30->3b awb_r_gain_min middle
{0x89, 0x2e}, //29->2c awb_b_gain_max middle
{0x8a, 0x1c}, //18->1b awb_b_gain_min middle

{0x8b, 0x3a}, //3c->45 awb_r_gain_max outdoor
{0x8c, 0x30}, //32->3b awb_r_gain_min outdoor
{0x8d, 0x2e}, //2a->2c awb_b_gain_max outdoor
{0x8e, 0x1c}, //1b->1b awb_b_gain_min outdoor

{0x8f, 0x4f}, // 56},// 4d}, //4e awb_slope_th0
{0x90, 0x48}, // 4f},// 46}, //4d awb_slope_th1
{0x91, 0x42}, // 49},// 40}, //4c awb_slope_th2
{0x92, 0x3c}, // 43},// 3a}, //4a awb_slope_th3
{0x93, 0x31}, // 38},// 2f}, //46 awb_slope_th4
{0x94, 0x23}, // 2a},// 21}, // awb_slope_th5
{0x95, 0x1b}, // 22},// 19}, // awb_slope_th6
{0x96, 0x18}, // 1f},// 16}, // awb_slope_th7
{0x97, 0x15}, // 1c},// 13}, // awb_slope_th8
{0x98, 0x14}, // 1b},// 12}, // awb_slope_th9
{0x99, 0x13}, // 1a},// 11}, // awb_slope_th10
{0x9a, 0x12}, // 19},// 10}, // awb_slope_th11

{0x9b, 0xaa},
{0x9c, 0xaa},
{0x9d, 0x48},
{0x9e, 0x38},
{0x9f, 0x30},

{0xa0, 0x70},
{0xa1, 0x54},
{0xa2, 0x6f},
{0xa3, 0xff},

{0xa4, 0x14}, //1536fps
{0xa5, 0x2c}, //698fps
{0xa6, 0xcf}, //148fps

{0xad, 0x2e},
{0xae, 0x2a},

{0xaf, 0x28}, //Low temp Rgain
{0xb0, 0x26}, //Low temp Rgain

{0xb1, 0x08},
{0xb4, 0xbf}, //For Tracking AWB Weight
{0xb8, 0x91}, //(0+,1-)High Cb , (0+,1-)Low Cr
{0xb9, 0xb0},
/////// PAGE 22 END ///////

//// MIPI Setting /////
{0x03, 0x48},
{0x39, 0x4f}, //lvds_bias_ctl    [2:0]mipi_tx_bias   [4:3]mipi_vlp_sel   [6:5]mipi_vcm_sel
{0x10, 0x1c}, //lvds_ctl_1       [5]mipi_pad_disable [4]lvds_en [0]serial_data_len 
{0x11, 0x00}, //lvds_ctl_2       [4]mipi continous mode setting
//{0x14, 0x00} //ser_out_ctl_1  [2:0]serial_sout_a_phase   [6:4]serial_cout_a_phase

{0x16, 0x00}, //lvds_inout_ctl1  [0]vs_packet_pos_sel [1]data_neg_sel [4]first_vsync_end_opt
{0x18, 0x80}, //lvds_inout_ctl3
{0x19, 0x00}, //lvds_inout_ctl4
{0x1a, 0xf0}, //lvds_time_ctl
{0x24, 0x1e}, //long_packet_id

//====== MIPI Timing Setting =========
{0x36, 0x01}, //clk_tlpx_time_dp
{0x37, 0x05}, //clk_tlpx_time_dn
{0x34, 0x04}, //clk_prepare_time
{0x32, 0x15}, //clk_zero_time
{0x35, 0x04}, //clk_trail_time
{0x33, 0x0d}, //clk_post_time

{0x1c, 0x01}, //tlps_time_l_dp
{0x1d, 0x0b}, //tlps_time_l_dn
{0x1e, 0x06}, //hs_zero_time
{0x1f, 0x09}, //hs_trail_time

//long_packet word count 
{0x30, 0x0c},
{0x31, 0x80}, //long_packet word count

/////// PAGE 20 ///////
{0x03, 0x20},
{0x10, 0x9c}, //AE On 50hz

/////// PAGE 22 ///////
{0x03, 0x22},
{0x10, 0xe9}, //AWB On

{0x03, 0x00},
{0x01, 0x00},
};

static struct v4l2_subdev_info hi258_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt	= 1,
		.order	= 0,
	},
};

static struct msm_camera_i2c_reg_conf hi258_svga_settings[] = {


{0x03, 0x00},
{0x10, 0x90},
{0x11, 0x90},

{0xd2, 0x05},
{0x03, 0x48},

{0x30, 0x06},
{0x31, 0x40}, //long_packet word count

{0x03, 0x18},
{0x18, 0x00},

};

static struct msm_camera_i2c_reg_conf hi258_sleep_settings[] = {

	{0x03, 0x00},
	{0x01, 0xf1},
	{0x03, 0x02},
	{0x55, 0x10},

	{0x01, 0xf1},
	{0x01, 0xf3},
	{0x01, 0xf1},

};

static struct msm_camera_i2c_reg_conf HI258_reg_saturation[11][3] = {
	{
		{0x03, 0x10},
		{0x61, 0x1c},
		{0x62, 0x1c},//-5
	},
	{
		{0x03, 0x10},
		{0x61, 0x30},
		{0x62, 0x30},//-4
	},
	{
		{0x03, 0x10},
		{0x61, 0x44},
		{0x62, 0x44},//-3
	},
	{
		{0x03, 0x10},
		{0x61, 0x58},
		{0x62, 0x58},//-2
	},
	{
		{0x03, 0x10},
		{0x61, 0x60},
		{0x62, 0x60},//-1
	},
	{
		{0x03, 0x10},//0
		{0x61, 0x70},
		{0x62, 0x7d},
	},
	{
		{0x03, 0x10},
		{0x61, 0x80},
		{0x62, 0x80},//+1
	},
	{
		{0x03, 0x10},
		{0x61, 0x90},
		{0x62, 0x90},//+2
	},
	{
		{0x03, 0x10},
		{0x61, 0xa0},
		{0x62, 0xa0},//+3
	},
	{
		{0x03, 0x10},
		{0x61, 0xb0},
		{0x62, 0xb0},//+4
	},
	{
		{0x03, 0x10},
		{0x61, 0xc0},
		{0x62, 0xc0},//+5
	},
};

static struct msm_camera_i2c_reg_conf HI258_reg_contrast[11][3] = {
	{
		{0x03, 0x10},
		{0x13, 0x03},
		{0x48, 0x60},
	},
	{
		{0x03, 0x10},
		{0x13, 0x03},
		{0x48, 0x68},
	},
	{
		{0x03, 0x10},
		{0x13, 0x03},
		{0x48, 0x70},
	},
	{
		{0x03, 0x10},
		{0x13, 0x03},
		{0x48, 0x78},
	},
	{
		{0x03, 0x10},
		{0x13, 0x03},
		{0x48, 0x80},
	},
	{
		{0x03, 0x10},
		{0x13, 0x03},
		{0x48, 0x88},//0
	},
	{
		{0x03, 0x10},
		{0x13, 0x03},
		{0x48, 0x90},
	},
	{
		{0x03, 0x10},
		{0x13, 0x03},
		{0x48, 0x98},
	},
	{
		{0x03, 0x10},
		{0x13, 0x03},
		{0x48, 0xa0},
	},
	{
		{0x03, 0x10},
		{0x13, 0x03},
		{0x48, 0xa8},
	},
	{
		{0x03, 0x10},
		{0x13, 0x03},
		{0x48, 0xb0},
	},
};

static struct msm_camera_i2c_reg_conf HI258_reg_sharpness[7][9] = {
	{
		{0x03, 0x13},
              {0x81, 0x07}, //Sharp2D Clip/Limit
              {0x82, 0x73}, //Sharp2D Filter
              {0x83, 0x00}, //Sharp2D Low Clip
              {0x85, 0x00},
              {0x92, 0x33}, //Sharp2D Slop n/p
              {0x93, 0x30}, //Sharp2D LClip
              {0x94, 0x02}, //Sharp2D HiClip1 Th
              {0x95, 0xf0}, //Sharp2D HiClip2 Th
	}, /* SHARPNESS LEVEL 0*/
	{
		{0x03, 0x13},
              {0x81, 0x07}, //Sharp2D Clip/Limit
              {0x82, 0x73}, //Sharp2D Filter
              {0x83, 0x00}, //Sharp2D Low Clip
              {0x85, 0x00},
              {0x92, 0x33}, //Sharp2D Slop n/p
              {0x93, 0x30}, //Sharp2D LClip
              {0x94, 0x02}, //Sharp2D HiClip1 Th
              {0x95, 0xf0}, //Sharp2D HiClip2 Th
	}, /* SHARPNESS LEVEL 1*/
	{
		{0x03, 0x13},
              {0x81, 0x07}, //Sharp2D Clip/Limit
              {0x82, 0x73}, //Sharp2D Filter
              {0x83, 0x00}, //Sharp2D Low Clip
              {0x85, 0x00},
              {0x92, 0x33}, //Sharp2D Slop n/p
              {0x93, 0x30}, //Sharp2D LClip
              {0x94, 0x02}, //Sharp2D HiClip1 Th
              {0x95, 0xf0}, //Sharp2D HiClip2 Th
	}, /* SHARPNESS LEVEL 2*/
	{
		{0x03, 0x13},
              {0x81, 0x07}, //Sharp2D Clip/Limit
              {0x82, 0x73}, //Sharp2D Filter
              {0x83, 0x00}, //Sharp2D Low Clip
              {0x85, 0x00},
              {0x92, 0x33}, //Sharp2D Slop n/p
              {0x93, 0x30}, //Sharp2D LClip
              {0x94, 0x02}, //Sharp2D HiClip1 Th
              {0x95, 0xf0}, //Sharp2D HiClip2 Th
	}, /* SHARPNESS LEVEL 3*/
	{
		{0x03, 0x13},
              {0x81, 0x07}, //Sharp2D Clip/Limit
              {0x82, 0x73}, //Sharp2D Filter
              {0x83, 0x00}, //Sharp2D Low Clip
              {0x85, 0x00},
              {0x92, 0x33}, //Sharp2D Slop n/p
              {0x93, 0x30}, //Sharp2D LClip
              {0x94, 0x02}, //Sharp2D HiClip1 Th
              {0x95, 0xf0}, //Sharp2D HiClip2 Th
	}, /* SHARPNESS LEVEL 4*/
	{
		{0x03, 0x13},
              {0x81, 0x07}, //Sharp2D Clip/Limit
              {0x82, 0x73}, //Sharp2D Filter
              {0x83, 0x00}, //Sharp2D Low Clip
              {0x85, 0x00},
              {0x92, 0x33}, //Sharp2D Slop n/p
              {0x93, 0x30}, //Sharp2D LClip
              {0x94, 0x02}, //Sharp2D HiClip1 Th
              {0x95, 0xf0}, //Sharp2D HiClip2 Th
	}, /* SHARPNESS LEVEL 5*/
	{
		{0x03, 0x13},
              {0x81, 0x07}, //Sharp2D Clip/Limit
              {0x82, 0x73}, //Sharp2D Filter
              {0x83, 0x00}, //Sharp2D Low Clip
              {0x85, 0x00},
              {0x92, 0x33}, //Sharp2D Slop n/p
              {0x93, 0x30}, //Sharp2D LClip
              {0x94, 0x02}, //Sharp2D HiClip1 Th
              {0x95, 0xf0}, //Sharp2D HiClip2 Th
	}, /* SHARPNESS LEVEL 6*/
};

static struct msm_camera_i2c_reg_conf HI258_reg_iso[7][3] = {
	/* auto */
	{
		{0x03, 0x20},
		{0x10, 0x9c},
		{0xb0, 0x80},
	},
	/* auto hjt */
	{
		{0x03, 0x20},
		{0x10, 0x9c},
		{0xb0, 0x15},
	},
	/* iso 100 */
	{
		{0x03, 0x20},
		{0x10, 0x0c},
		{0xb0, 0x1B},
	},
	/* iso 200 */
	{
		{0x03, 0x20},
		{0x10, 0x0c},
		{0xb0, 0x35},
	},
	/* iso 400 */
	{
		{0x03, 0x20},
		{0x10, 0x0c},
		{0xb0, 0x65},
	},
	/* iso 800 */
	{
		{0x03, 0x20},
		{0x10, 0x0c},
		{0xb0, 0x95},
	},
	/* iso 1600 */
	{
		{0x03, 0x20},
		{0x10, 0x0c},
		{0xb0, 0xd0},
	},
};

static struct msm_camera_i2c_reg_conf HI258_reg_exposure_compensation[5][2] = {
	/* -2 */
	{
		{0x03, 0x10},
		{0x40, 0xa4},
	},
	/* -1 */
	{
		{0x03, 0x10},
		{0x40, 0x94},
	},
	/* 0 */
	{
		{0x03, 0x10},
		{0x40, 0x90},
	},
	/* 1 */
	{
		{0x03, 0x10},
		{0x40, 0x14},
	},
	/* 2 */
	{
		{0x03, 0x10},
		{0x40, 0x24},
	},
};

static struct msm_camera_i2c_reg_conf HI258_reg_antibanding[][2] = {
	/* OFF */
	{
		{0x03, 0x20},
		{0x10, 0xcc},
	},
	/* 50Hz */
	{
		{0x03, 0x20},
		{0x10, 0x9c},
	},
	/* 60Hz */
	{
		{0x03, 0x20},
		{0x10, 0x8c},
	},
	/* AUTO */
	{
		{0x03, 0x20},
		{0x10, 0xcc},
	},
};

static struct msm_camera_i2c_reg_conf HI258_reg_effect_normal[] = {
	/* normal: */
	{0x03, 0x20},
	{0x28, 0xf7},
	{0x03, 0x10},
	{0x11, 0x03},
    {0x12, 0xf0},
    {0x13, 0x03},
  	{0x42, 0x08},  
	{0x44, 0x80},
	{0x45, 0x80},
};

static struct msm_camera_i2c_reg_conf HI258_reg_effect_black_white[] = {
	/* B&W: */
	{0x03, 0x20},
	{0x28, 0xf7},
	{0x03, 0x10},
	{0x11, 0x03},
	{0x12, 0x33},
	{0x13, 0x02},
	{0x44, 0x80},
	{0x45, 0x80},
};

static struct msm_camera_i2c_reg_conf HI258_reg_effect_negative[] = {
	/* Negative: */
	{0x03, 0x20},
	{0x28, 0xf7},
	{0x03, 0x10},
	{0x11, 0x03},
	{0x12, 0x08},
	{0x13, 0x0a},
	{0x14, 0x00},
};

static struct msm_camera_i2c_reg_conf HI258_reg_effect_old_movie[] = {
	/* Sepia(antique): */
	{0x03, 0x20},
	{0x28, 0xf7},
	{0x03, 0x10},
	{0x11, 0x03},
	{0x12, 0x33},
	{0x13, 0x0a},
	{0x44, 0x25},
	{0x45, 0xa6},
};

static struct msm_camera_i2c_reg_conf HI258_reg_effect_solarize[] = {
	{0x03, 0x20},
	{0x28, 0xf7},
	{0x03, 0x10},
	{0x11, 0x0b},
	{0x12, 0x00},
	{0x13, 0x00},
	{0x14, 0x00},
};

static struct msm_camera_i2c_reg_conf HI258_reg_scene_auto[] = {
	/* <SCENE_auto> */
	{0x03, 0x20},
	{0x10, 0x9c},

	{0x03, 0x10},
	{0x41, 0x05},

};

static struct msm_camera_i2c_reg_conf HI258_reg_scene_portrait[] = {
	/* <CAMTUNING_SCENE_PORTRAIT> */
	{0x03, 0x20},
	{0x10, 0x1c},
	{0x10, 0x9c},
};

static struct msm_camera_i2c_reg_conf HI258_reg_scene_landscape[] = {
	/* <CAMTUNING_SCENE_LANDSCAPE> */
	{0x03, 0x20},
	{0x10, 0x1c},
	{0x10, 0x9c},

};

static struct msm_camera_i2c_reg_conf HI258_reg_scene_night[] = {
	/* <SCENE_NIGHT> */
	{0x03, 0x20},
	{0x10, 0x1c},
	{0x10, 0x9c},

	{0x03, 0x10},
	{0x41, 0x10},
};

static struct msm_camera_i2c_reg_conf HI258_reg_wb_auto[] = {
	/* Auto: */
    {0x03, 0x22},
    {0x11, 0x2e},
	{0x83, 0x58},
	{0x84, 0x16},
	{0x85, 0x4f},
	{0x86, 0x20},

    {0x10, 0xfd},
};

static struct msm_camera_i2c_reg_conf HI258_reg_wb_sunny[] = {
	/* Sunny: */
	{0x03, 0x22},
	{0x11, 0x28},
	{0x80, 0x33},
	{0x82, 0x3d},
	{0x83, 0x2e},
	{0x84, 0x24},
	{0x85, 0x43},
	{0x86, 0x3d},

};

static struct msm_camera_i2c_reg_conf HI258_reg_wb_cloudy[] = {
	/* Cloudy: */
	{0x03, 0x22},
	{0x11, 0x28},
	{0x80, 0x49},
	{0x82, 0x24},
	{0x83, 0x50},
	{0x84, 0x45},
	{0x85, 0x24},
	{0x86, 0x1E},
};

static struct msm_camera_i2c_reg_conf HI258_reg_wb_office[] = {
	/* Office: */
	{0x03, 0x22},
    {0x11, 0x2e},
	{0x83, 0x58},
	{0x84, 0x36},
	{0x85, 0x2f},
	{0x86, 0x21},

    {0x10, 0xfd},

	
};

static struct msm_camera_i2c_reg_conf HI258_reg_wb_home[] = {
	/* Home: */
	{0x03, 0x22},
	{0x11, 0x28},
	{0x80, 0x29},
	{0x82, 0x54},
	{0x83, 0x2e},
	{0x84, 0x23},
	{0x85, 0x58},
	{0x86, 0x4f},
};


static const struct i2c_device_id hi258_i2c_id[] = {
	{HI258_SENSOR_NAME, (kernel_ulong_t)&hi258_s_ctrl},
	{ }
};

static int32_t msm_hi258_i2c_probe(struct i2c_client *client,
	   const struct i2c_device_id *id)
{
	int rc;
	rc = msm_sensor_i2c_probe(client, id, &hi258_s_ctrl);	
	CDBG("%s:%d\n", __func__, __LINE__);
//	if(rc==0)
//	{
//		productinfo_register(PRODUCTINFO_FRONT_CAMERA_ID,"HI258",NULL);
//	}
	return rc;
}

static struct i2c_driver hi258_i2c_driver = {
	.id_table = hi258_i2c_id,
	.probe  = msm_hi258_i2c_probe,
	.driver = {
		.name = HI258_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hi258_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static const struct of_device_id hi258_dt_match[] = {
	{.compatible = "qcom,hi258", .data = &hi258_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, hi258_dt_match);

static struct platform_driver hi258_platform_driver = {
	.driver = {
		.name = "qcom,hi258",
		.owner = THIS_MODULE,
		.of_match_table = hi258_dt_match,
	},
};

static void hi258_i2c_write_table(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_camera_i2c_reg_conf *table,
		int num)
{
	int i = 0;
	int rc = 0;
//	uint16_t temp;
	for (i = 0; i < num; ++i) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
			s_ctrl->sensor_i2c_client, table->reg_addr,
			table->reg_data,
			MSM_CAMERA_I2C_BYTE_DATA);
		//CDBG("camera reg addr = 0x%x, 0x%x\n", table->reg_addr,
		//	table->reg_data);
		if (rc < 0) {
			msleep(100);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
				s_ctrl->sensor_i2c_client, table->reg_addr,
				table->reg_data,
				MSM_CAMERA_I2C_BYTE_DATA);
		}
		//s_ctrl->sensor_i2c_client->i2c_func_tbl->
		//	i2c_read(
		//		s_ctrl->sensor_i2c_client, table->reg_addr,
		//		&temp,
		//		MSM_CAMERA_I2C_BYTE_DATA);
		//printk("hi258_i2c_write_table reg_addr:%x reg_data:%x\n",table->reg_addr,temp);
		table++;
	}
}


#ifdef VENDOR_EDIT
//OPPO zhangkw add interface to get exposure time
static uint32_t hi258_get_exp(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint32_t rc = 0, val=0;
	uint32_t exp = 0;
	uint16_t reg_80, reg_81, reg_82;
	//write page
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(s_ctrl->sensor_i2c_client, 0x03,0x20, MSM_CAMERA_I2C_BYTE_DATA);

	//read reg
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_read(s_ctrl->sensor_i2c_client, 0x2080, &reg_80, MSM_CAMERA_I2C_BYTE_DATA);
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_read(s_ctrl->sensor_i2c_client, 0x2081, &reg_81, MSM_CAMERA_I2C_BYTE_DATA);
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_read(s_ctrl->sensor_i2c_client, 0x2082, &reg_82, MSM_CAMERA_I2C_BYTE_DATA);
	//CDBG("%s zhangkw 80=0x%x 81=0x%x, 82=0x%x\n", __func__, reg_80, reg_81, reg_82);
	
	exp = (reg_80 << 8) |reg_81;
	exp = (exp << 8) | reg_82;
	
	val = exp ; //* 4 /26000000;
	CDBG("%s exp=0x%x\n", __func__, exp);
	return val;
}

//OPPO zhangkw add interface to get iso value
static uint32_t hi258_get_iso(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint32_t val = 0, rc=0;
	uint16_t reg_b0, reg_b1;
	//write page
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(s_ctrl->sensor_i2c_client, 0x03,0x20, MSM_CAMERA_I2C_BYTE_DATA);
	//read reg
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_read(s_ctrl->sensor_i2c_client, 0x20b0, &reg_b0, MSM_CAMERA_I2C_BYTE_DATA);
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_read(s_ctrl->sensor_i2c_client, 0x20b1, &reg_b1, MSM_CAMERA_I2C_BYTE_DATA);
	//reg_b1 = 0x14;
	
	val = reg_b0 * 100 /reg_b1;
	CDBG("%s b0=0x%x b1=0x%x  val=%d\n", __func__, reg_b0, reg_b1, val);
	return val;
}
//OPPO zhangkw add interface to read reg
#endif

static int32_t hi258_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	hi258_i2c_write_table(s_ctrl, &hi258_sleep_settings[0],
		ARRAY_SIZE(hi258_sleep_settings));
	return msm_sensor_power_down(s_ctrl);
}

static int32_t hi258_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;	
	CDBG("%s:%d\n", __func__, __LINE__);
	match = of_match_device(hi258_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
      if(!rc)
       {
        register_device_proc("f_camera", DEVICE_VERSION_14027_F, DEVICE_MANUFACUTRE_14027_F);
       }
	return rc;
}

static int __init hi258_init_module(void)
{

	int32_t rc;
	CDBG("%s:%d\n", __func__, __LINE__);
	
	rc = platform_driver_probe(&hi258_platform_driver,
		hi258_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&hi258_i2c_driver);
}

static void __exit hi258_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (hi258_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hi258_s_ctrl);
		platform_driver_unregister(&hi258_platform_driver);
	} else
		i2c_del_driver(&hi258_i2c_driver);
	return;
}

static int32_t hi258_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
		&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: hi258 read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	CDBG("%s: read id: %x expected id %x:\n", __func__, chipid,
		s_ctrl->sensordata->slave_info->sensor_id);
	if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	return rc;
}

static void hi258_set_stauration(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	hi258_i2c_write_table(s_ctrl, &HI258_reg_saturation[value][0],
		ARRAY_SIZE(HI258_reg_saturation[value]));
}

static void hi258_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	hi258_i2c_write_table(s_ctrl, &HI258_reg_contrast[value][0],
		ARRAY_SIZE(HI258_reg_contrast[value]));
}

static void hi258_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	int val = value / 6;
	pr_debug("%s %d", __func__, value);
	hi258_i2c_write_table(s_ctrl, &HI258_reg_sharpness[val][0],
		ARRAY_SIZE(HI258_reg_sharpness[val]));
}


static void hi258_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	hi258_i2c_write_table(s_ctrl, &HI258_reg_iso[value][0],
		ARRAY_SIZE(HI258_reg_iso[value]));
}

static void hi258_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int val = (value + 12) / 6;
	pr_debug("%s %d", __func__, val);
	hi258_i2c_write_table(s_ctrl, &HI258_reg_exposure_compensation[val][0],
		ARRAY_SIZE(HI258_reg_exposure_compensation[val]));
}

static void hi258_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	switch (value) {
	case MSM_CAMERA_EFFECT_MODE_OFF: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_effect_normal[0],
			ARRAY_SIZE(HI258_reg_effect_normal));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_MONO: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_effect_black_white[0],
			ARRAY_SIZE(HI258_reg_effect_black_white));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_NEGATIVE: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_effect_negative[0],
			ARRAY_SIZE(HI258_reg_effect_negative));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SEPIA: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_effect_old_movie[0],
			ARRAY_SIZE(HI258_reg_effect_old_movie));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SOLARIZE: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_effect_solarize[0],
			ARRAY_SIZE(HI258_reg_effect_solarize));
		break;
	}
	default:
		hi258_i2c_write_table(s_ctrl, &HI258_reg_effect_normal[0],
			ARRAY_SIZE(HI258_reg_effect_normal));
	}
}

static void hi258_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	hi258_i2c_write_table(s_ctrl, &HI258_reg_antibanding[value][0],
		ARRAY_SIZE(HI258_reg_antibanding[value]));
}

static void hi258_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	switch (value) {
	case MSM_CAMERA_SCENE_MODE_OFF: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_scene_auto[0],
			ARRAY_SIZE(HI258_reg_scene_auto));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_NIGHT: 
	case MSM_CAMERA_SCENE_MODE_NIGHT_PORTRAIT:
	{
		hi258_i2c_write_table(s_ctrl, &HI258_reg_scene_night[0],
			ARRAY_SIZE(HI258_reg_scene_night));
					break;
	}
	case MSM_CAMERA_SCENE_MODE_LANDSCAPE: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_scene_landscape[0],
			ARRAY_SIZE(HI258_reg_scene_landscape));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_PORTRAIT: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_scene_portrait[0],
			ARRAY_SIZE(HI258_reg_scene_portrait));
		break;
	}
	default:
		hi258_i2c_write_table(s_ctrl, &HI258_reg_scene_auto[0],
			ARRAY_SIZE(HI258_reg_scene_auto));
	}
}

static void hi258_set_white_balance_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	pr_debug("%s %d", __func__, value);
	switch (value) {
	case MSM_CAMERA_WB_MODE_AUTO: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_wb_auto[0],
			ARRAY_SIZE(HI258_reg_wb_auto));
		break;
	}
	case MSM_CAMERA_WB_MODE_INCANDESCENT: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_wb_home[0],
			ARRAY_SIZE(HI258_reg_wb_home));
		break;
	}
	case MSM_CAMERA_WB_MODE_DAYLIGHT: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_wb_sunny[0],
			ARRAY_SIZE(HI258_reg_wb_sunny));
					break;
	}
	case MSM_CAMERA_WB_MODE_FLUORESCENT: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_wb_office[0],
			ARRAY_SIZE(HI258_reg_wb_office));
					break;
	}
	case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_wb_cloudy[0],
			ARRAY_SIZE(HI258_reg_wb_cloudy));
					break;
	}
	default:
		hi258_i2c_write_table(s_ctrl, &HI258_reg_wb_auto[0],
		ARRAY_SIZE(HI258_reg_wb_auto));
	}
}

int32_t hi258_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
		break;
	case CFG_SET_INIT_SETTING:
		CDBG("init setting");
		hi258_i2c_write_table(s_ctrl,
				&hi258_recommend_settings[0],
				ARRAY_SIZE(hi258_recommend_settings));
		CDBG("init setting X");
		break;
	case CFG_SET_RESOLUTION: {
		int val = 0;
		CDBG("svga/xvga setting");
		if (copy_from_user(&val,
			(void *)cdata->cfg.setting, sizeof(int))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		if (0==val)
			hi258_i2c_write_table(s_ctrl, &hi258_uxga_settings[0],
				ARRAY_SIZE(hi258_uxga_settings));
		else if (1==val)
			hi258_i2c_write_table(s_ctrl, &hi258_svga_settings[0],
				ARRAY_SIZE(hi258_svga_settings));
		else if(2==val)
			{
			hi258_i2c_write_table(s_ctrl, &hi258_480p_settings[0],
				ARRAY_SIZE(hi258_480p_settings));		
			}
		CDBG("svga/xvga/vga setting %d X",val);
		
		break;
	}
	case CFG_SET_STOP_STREAM:
		CDBG("stop stream");
		hi258_i2c_write_table(s_ctrl,
			&hi258_stop_settings[0],
			ARRAY_SIZE(hi258_stop_settings));
		CDBG("stop stream X");
		break;
	case CFG_SET_START_STREAM:
		CDBG("start stream");
		hi258_i2c_write_table(s_ctrl,
			&hi258_start_settings[0],
			ARRAY_SIZE(hi258_start_settings));
		CDBG("start stream x");
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params =
			*s_ctrl->sensordata->sensor_init_params;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_sensor_power_setting_array *power_setting_array;
		int slave_index = 0;
		if (copy_from_user(&sensor_slave_info,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		/* Update sensor slave address */
		if (sensor_slave_info.slave_addr) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info.slave_addr >> 1;
		}

		/* Update sensor address type */
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;

		/* Update power up / down sequence */
		s_ctrl->power_setting_array =
			sensor_slave_info.power_setting_array;
		power_setting_array = &s_ctrl->power_setting_array;
		power_setting_array->power_setting = kzalloc(
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting), GFP_KERNEL);
		if (!power_setting_array->power_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(power_setting_array->power_setting,
			(void *)
			sensor_slave_info.power_setting_array.power_setting,
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting))) {
			kfree(power_setting_array->power_setting);
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		s_ctrl->free_power_setting = true;
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (slave_index = 0; slave_index <
			power_setting_array->size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
			slave_index,
			power_setting_array->power_setting[slave_index].
			seq_type,
			power_setting_array->power_setting[slave_index].
			seq_val,
			power_setting_array->power_setting[slave_index].
			config_val,
			power_setting_array->power_setting[slave_index].
			delay);
		}
		kfree(power_setting_array->power_setting);
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->func_tbl->sensor_power_up){
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		    if (rc < 0) {
				pr_err("%s:%d failed rc %ld\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
			pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
			}
		else
			rc = -EFAULT;
		break;

	case CFG_POWER_DOWN:
		if (s_ctrl->func_tbl->sensor_power_down){
			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		    if (rc < 0) {
				pr_err("%s:%d failed rc %ld\n", __func__,
					__LINE__, rc);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
			}
		else
			rc = -EFAULT;
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(stop_setting, (void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
			(void *)reg_setting, stop_setting->size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}
	case CFG_SET_SATURATION: {
		int32_t sat_lev;
		if (copy_from_user(&sat_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Saturation Value is %d", __func__, sat_lev);
		CDBG("set Saturation");
		hi258_set_stauration(s_ctrl, sat_lev);
		CDBG("set Saturation X");
		break;
	}
	case CFG_SET_CONTRAST: {
		int32_t con_lev;
		if (copy_from_user(&con_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Contrast Value is %d", __func__, con_lev);
		CDBG("set contrast");
		hi258_set_contrast(s_ctrl, con_lev);
		CDBG("set contrast X");
		break;
	}
	case CFG_SET_SHARPNESS: {
		int32_t shp_lev;
		if (copy_from_user(&shp_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Sharpness Value is %d", __func__, shp_lev);
		CDBG("set SHARPNESS");
		hi258_set_sharpness(s_ctrl, shp_lev);
		CDBG("set SHARPNESS X");
		break;
	}
	case CFG_SET_ISO: {
		int32_t iso_lev;
		if (copy_from_user(&iso_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: ISO Value is %d", __func__, iso_lev);
		CDBG("set iso");
		hi258_set_iso(s_ctrl, iso_lev);
		CDBG("set iso X");
		break;
	}
	case CFG_SET_EXPOSURE_COMPENSATION: {
		int32_t ec_lev;
		if (copy_from_user(&ec_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Exposure compensation Value is %d",
			__func__, ec_lev);
		CDBG("set exposure");
		hi258_set_exposure_compensation(s_ctrl, ec_lev);
		CDBG("Set exposure X");
		break;
	}
	case CFG_SET_EFFECT: {
		int32_t effect_mode;
		if (copy_from_user(&effect_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Effect mode is %d", __func__, effect_mode);
		CDBG("Set effect");
		hi258_set_effect(s_ctrl, effect_mode);
		CDBG("Set effect X");
		break;
	}
	case CFG_SET_ANTIBANDING: {
		int32_t antibanding_mode;
		if (copy_from_user(&antibanding_mode,
			(void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: anti-banding mode is %d", __func__,
			antibanding_mode);
		CDBG("Set antibanding");
		hi258_set_antibanding(s_ctrl, antibanding_mode);
		CDBG("Set antibanding X");
		break;
	}
	case CFG_SET_BESTSHOT_MODE: {
		int32_t bs_mode;
		if (copy_from_user(&bs_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: best shot mode is %d", __func__, bs_mode);
		CDBG("Set scene mode");
		hi258_set_scene_mode(s_ctrl, bs_mode);
		CDBG("Set scene mode X");
		break;
	}
	case CFG_SET_WHITE_BALANCE: {
		int32_t wb_mode;
		if (copy_from_user(&wb_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: white balance is %d", __func__, wb_mode);
		CDBG("Set wb");
		hi258_set_white_balance_mode(s_ctrl, wb_mode);
		CDBG("Set wb X");
		break;
	}
#ifdef VENDOR_EDIT
//OPPO zhangkw add case to send exp and iso to user space
	case CFG_GET_YUV_INFO: {
		cdata->cfg.yuv_info.exp_time= hi258_get_exp(s_ctrl);
		cdata->cfg.yuv_info.iso= hi258_get_iso(s_ctrl);
		CDBG("%s exp=%d, iso=%d\n", __func__, 
			cdata->cfg.yuv_info.exp_time, cdata->cfg.yuv_info.iso);
		break;
	}	
#endif	
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

static struct msm_sensor_fn_t hi258_sensor_func_tbl = {
	.sensor_config = hi258_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = hi258_sensor_power_down,
	.sensor_match_id = hi258_sensor_match_id,
};

static struct msm_sensor_ctrl_t hi258_s_ctrl = {
	.sensor_i2c_client = &hi258_sensor_i2c_client,
	.power_setting_array.power_setting = hi258_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hi258_power_setting),
	.msm_sensor_mutex = &hi258_mut,
	.sensor_v4l2_subdev_info = hi258_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hi258_subdev_info),
	.func_tbl = &hi258_sensor_func_tbl,
};

module_init(hi258_init_module);
module_exit(hi258_exit_module);
MODULE_DESCRIPTION("Hi256 2MP YUV sensor driver");
MODULE_LICENSE("GPL v2");
