/**
 * Copyright 2008-2013 OPPO Mobile Comm Corp., Ltd, All rights reserved.
 * VENDOR_EDIT:
 * FileName:devinfo.c
 * ModuleName:devinfo
 * Author: wangjc
 * Create Date: 2013-10-23
 * Description:add interface to get device information.
 * History:
   <version >  <time>  <author>  <desc>
   1.0		2013-10-23	wangjc	init
*/

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <mach/device_info.h>
#include <mach/oppo_project.h>
#include <mach/msm_smem.h>


static struct proc_dir_entry *parent = NULL;

static int device_proc_output (char *buf, struct manufacture_info *priv)
{
	char *p = buf;

	p += sprintf(p, "Device version:\t\t%s\n",
		     priv->version);

	p += sprintf(p, "Device manufacture:\t\t%s\n",
			priv->manufacture);

	return p - buf;
}

static int device_read_proc(char *page, char **start, off_t off,
			   int count, int *eof, void *data)
{
	struct manufacture_info *priv = data;
	
	int len = device_proc_output (page, priv);
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

int register_device_proc(char *name, char *version, char *manufacture)
{
	struct proc_dir_entry *d_entry;
	struct manufacture_info *info;

	if(!parent) {
		parent =  proc_mkdir ("devinfo", NULL);
		if(!parent) {
			pr_err("can't create devinfo proc\n");
			return -ENOENT;
		}
	}

	info = kzalloc(sizeof *info, GFP_KERNEL);
	info->version = version;
	info->manufacture = manufacture;

	d_entry = create_proc_read_entry (name, S_IRUGO, parent, device_read_proc, info);
	if(!d_entry) {
		pr_err("create %s proc failed.\n", name);
		kfree(info);
		return -ENOENT;
	}
	return 0;
}

static void dram_type_add(void)
{
	struct manufacture_info dram_info;
	int *p = NULL;
	p  = (int *)smem_alloc2(SMEM_DRAM_TYPE,4);

	if(p)
	{
		switch(*p){
			case DRAM_TYPE0:
				dram_info.version = "EDB8132B3PB-1D-F FBGA";
				dram_info.manufacture = "ELPIDA";
				break;
			case DRAM_TYPE1:
				dram_info.version = "EDB8132B3PB-1D-F FBGA";
				dram_info.manufacture = "ELPIDA";
				break;
			case DRAM_TYPE2:
				dram_info.version = "EDF8132A3PF-GD-F FBGA";
				dram_info.manufacture = "ELPIDA";
				break;
			case DRAM_TYPE3:
				dram_info.version = "K4E8E304ED-AGCC FBGA";
				dram_info.manufacture = "SAMSUNG";
				break;
			default:
				dram_info.version = "unknown";
				dram_info.manufacture = "unknown";
		}

	}else{
		dram_info.version = "unknown";
		dram_info.manufacture = "unknown";

	}

	register_device_proc("ddr", dram_info.version, dram_info.manufacture);


}

//hantong@basic.drv added for mainboard ID
static void mainboard_verify(void)
{
	struct manufacture_info mainboard_info;
	switch(get_PCB_Version()) {
		case HW_VERSION__10:		
			mainboard_info.version ="10";
			mainboard_info.manufacture = "SA(SB)";
			break;
		case HW_VERSION__11:	
			mainboard_info.version = "11";
			mainboard_info.manufacture = "SC";
			break;
		case HW_VERSION__12:
			mainboard_info.version = "12";
			mainboard_info.manufacture = "SD";
			break;
		case HW_VERSION__13:
			mainboard_info.version = "13";
            if (is_project(OPPO_13095)) {
                mainboard_info.manufacture = "-1";
            } else {
                mainboard_info.manufacture = "SE";
            }
			break;
		case HW_VERSION__14:
			mainboard_info.version = "14";
			mainboard_info.manufacture = " ";
			break;
		case HW_VERSION__15:
			mainboard_info.version = "15";
			mainboard_info.manufacture = "T3-T4";
			break;
		default:	
			mainboard_info.version = "UNKOWN";
			mainboard_info.manufacture = "UNKOWN";
	}	
	register_device_proc("mainboard", mainboard_info.version, mainboard_info.manufacture);
}

#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG,modified 2014.4.13 for 14027
static void pa_verify(void)
{
	struct manufacture_info pa_info;
	if(is_project(OPPO_14027))
	{
		switch(get_Modem_Version()) {
			case 0:		
				pa_info.version = "0";
				pa_info.manufacture = "14027 0";
				break;
			case 1:		
				pa_info.version = "1";
				pa_info.manufacture = "14027 1";
				break;
			default:	
				pa_info.version = "UNKOWN";
				pa_info.manufacture = "UNKOWN";
		}
	}else if(is_project(OPPO_14029))
	{
		switch(get_Modem_Version()) {
			case 0:		
				pa_info.version = "0";
				pa_info.manufacture = "14029 0";
				break;
			case 1:		
				pa_info.version = "1";
				pa_info.manufacture = "14029 1";
				break;
			case 2:		
				pa_info.version = "2";
				pa_info.manufacture = "14029 10";
				break;
			case 3:		
				pa_info.version = "3";
				pa_info.manufacture = "14029 11";
				break;
			case 4:		
				pa_info.version = "4";
				pa_info.manufacture = "14029 100";
				break;
			case 5:		
				pa_info.version = "5";
				pa_info.manufacture = "14029 101";
				break;
			case 6:		
				pa_info.version = "6";
				pa_info.manufacture = "14029 110";
				break;
			case 7:		
				pa_info.version = "7";
				pa_info.manufacture = "14029 111";
				break;
			default:	
				pa_info.version = "UNKOWN";
				pa_info.manufacture = "UNKOWN";
		}
	}else{
		switch(get_Modem_Version()) {
			case 0:		
				pa_info.version = "0";
				pa_info.manufacture = "RFMD PA";
				break;
			case 1:	
				pa_info.version = "1";
				pa_info.manufacture = "SKY PA";
				break;
			case 3:
				pa_info.version = "3";
				pa_info.manufacture = "AVAGO PA";
				break;
			default:	
				pa_info.version = "UNKOWN";
				pa_info.manufacture = "UNKOWN";
		}
	}		
	register_device_proc("pa", pa_info.version, pa_info.manufacture);

}
#endif /*VENDOR_EDIT*/	

static int __init device_info_init(void)
{
	int ret = 0;
	
	parent =  proc_mkdir ("devinfo", NULL);
	if(!parent) {
		pr_err("can't create devinfo proc\n");
		ret = -ENOENT;
	}

	dram_type_add();
	mainboard_verify();
	pa_verify();
	return ret;
}

static void __exit device_info_exit(void)
{

	remove_proc_entry("devinfo", NULL);

}

module_init(device_info_init);
module_exit(device_info_exit);


MODULE_DESCRIPTION("OPPO device info");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Wangjc <wjc@oppo.com>");


