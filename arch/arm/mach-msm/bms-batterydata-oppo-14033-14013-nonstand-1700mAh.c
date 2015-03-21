#include <linux/batterydata-lib.h>

static struct single_row_lut fcc_temp = {
	.x		= {-20, 0, 25, 40, 60},
	.y		= {1723, 1761, 1758, 1758, 1758},
	.cols	= 5
};

static struct single_row_lut fcc_sf = {
	.x		= {0},
	.y		= {100},
	.cols	= 1
};

static struct sf_lut rbatt_sf = {
	.rows		= 30,
	.cols		= 5,
	.row_entries		= {-20, 0, 25, 40, 60},
	.percent	= {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 16, 13, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1},
	.sf		= {
				{513, 219, 100, 69, 68},
				{508, 216, 100, 70, 72},
				{493, 213, 101, 71, 71},
				{483, 224, 106, 73, 71},	
				{473, 206, 106, 76, 77},
				{427, 205, 113, 81, 86},
				{466, 205, 114, 86, 87},
				{472, 191, 113, 88, 112},
				{475, 184, 99, 75, 139},
				{484, 183, 94, 73, 134},
				{488, 187, 91, 74, 131},
				{497, 190, 91, 74, 144},
				{509, 194, 91, 76, 131},
				{519, 199, 91, 77, 142},
				{532, 212, 92, 73, 133},
				{550, 222, 92, 74, 133},
				{604, 231, 92, 74, 141},
				{689, 250, 87, 73, 138},
				{825, 263, 85, 73, 140},
				{872, 279, 87, 75, 141},
				{919, 297, 89, 77, 141},
				{970, 314, 92, 78, 143},
				{1038, 336, 95, 80, 144},
				{2020, 364, 97, 81, 144},
				{3030, 398, 100, 79, 144},
				{4040, 398, 100, 81, 145},
				{4040, 446, 100, 81, 145},
				{5050, 523, 105, 85, 149},
				{7070, 669, 114, 93, 161},
				{12121, 669, 137, 120, 668}
	}
};


static struct pc_temp_ocv_lut pc_temp_ocv = {
	.rows		= 31,
	.cols		= 5,
	.temp		= {-20, 0, 25, 40, 60},
	.percent	= {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 16, 13, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0},
	.ocv		= {
				{4202, 4202, 4203, 4204, 4205},
				{4165, 4158, 4159, 4157, 4159},
				{4101, 4106, 4103, 4103, 4103},
				{4056, 4079, 4067, 4062, 4060},
				{4003, 3990, 4002, 4005, 4012},
				{3956, 3959, 3972, 3979, 3978},
				{3915, 3940, 3948, 3949, 3946},
				{3886, 3893, 3909, 3910, 3909},
				{3861, 3859, 3868, 3868, 3867},
				{3835, 3831, 3831, 3832, 3832},
				{3816, 3815, 3813, 3814, 3812},
				{3800, 3802, 3799, 3799, 3797},
				{3785, 3790, 3784, 3785, 3783},
				{3775, 3785, 3775, 3775, 3772},
				{3764, 3783, 3766, 3760, 3746},
				{3747, 3775, 3755, 3749, 3734},
				{3724, 3753, 3738, 3731, 3716},
				{3714, 3714, 3701, 3695, 3679},
				{3703, 3707, 3685, 3682, 3672},
				{3697, 3701, 3684, 3682, 3670},
				{3690, 3700, 3684, 3681, 3669},
				{3678, 3698, 3684, 3681, 3669},
				{3659, 3695, 3682, 3680, 3667},
				{3600, 3690, 3681, 3676, 3658},
				{3500, 3663, 3671, 3659, 3624},
				{3450, 3663, 3633, 3613, 3568},
				{3450, 3607, 3633, 3613, 3568},
				{3400, 3525, 3571, 3546, 3492},
				{3300, 3398, 3483, 3450, 3376},
				{3200, 3200, 3333, 3278, 3175},
				{3000, 3000, 3000, 3000, 3000}
	}
};


struct bms_battery_data OPPO_14033_14013_nonstand_1700mAh_data = {
	.fcc				= 1700,
	.fcc_temp_lut			= &fcc_temp,
	.fcc_sf_lut				= &fcc_sf,
	.pc_temp_ocv_lut		= &pc_temp_ocv,
	.rbatt_sf_lut			= &rbatt_sf,
	.default_rbatt_mohm	= 274
};