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
 */

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/memory.h>
#include <linux/regulator/qpnp-regulator.h>
#include <linux/msm_tsens.h>
#include <asm/mach/map.h>
#include <asm/hardware/gic.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/board.h>
#include <mach/gpiomux.h>
#include <mach/msm_iomap.h>
#include <mach/restart.h>
#ifdef CONFIG_ION_MSM
#include <mach/ion.h>
#endif
#include <mach/msm_memtypes.h>
#include <mach/socinfo.h>
#include <mach/board.h>
#include <mach/clk-provider.h>
#include <mach/msm_smd.h>
#include <mach/rpm-smd.h>
#include <mach/rpm-regulator-smd.h>
#include <mach/msm_smem.h>
#include <linux/msm_thermal.h>
#include "board-dt.h"
#include "clock.h"
#include "platsmp.h"
#include "spm.h"
#include "pm.h"
#include "modem_notifier.h"
#ifdef VENDOR_EDIT
#include <mach/oppo_project.h>
#include <mach/oppo_boot_mode.h>
#include <linux/persistent_ram.h>

static struct kobject *systeminfo_kobj;

static int ftm_mode = MSM_BOOT_MODE__NORMAL;

static int __init  board_mfg_mode_init(void)
{	
    char *substr;

    substr = strstr(boot_command_line, "oppo_ftm_mode=");
    if(substr) {
        substr += strlen("oppo_ftm_mode=");

        if(strncmp(substr, "factory2", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__FACTORY;
        else if(strncmp(substr, "ftmwifi", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__WLAN;
        else if(strncmp(substr, "ftmrf", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__RF;
        else if(strncmp(substr, "ftmrecovery", 5) == 0)
            ftm_mode = MSM_BOOT_MODE__RECOVERY;

    } 	

	pr_err("board_mfg_mode_init, " "ftm_mode=%d\n", ftm_mode);
	
	return 0;

}

int get_boot_mode(void)
{
	return ftm_mode;
}

static ssize_t ftmmode_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%d\n", ftm_mode);
}

static struct kobj_attribute ftmmode_attr = {
    .attr = {"ftmmode", 0644},

    .show = &ftmmode_show,
};

static struct attribute * g[] = {
	&ftmmode_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

#endif //VENDOR_EDIT

#ifdef VENDOR_EDIT
/* OPPO 2013-09-03 zhanglong add for add interface start reason and boot_mode begin */
char pwron_event[16];
static int __init start_reason_init(void)
{
    int i;
    char * substr = strstr(boot_command_line, "androidboot.startupmode=");
	if(substr) {
	    substr += strlen("androidboot.startupmode=");
	    for(i=0; substr[i] != ' '; i++) {
	        pwron_event[i] = substr[i];
	    }
	    pwron_event[i] = '\0';
	}
    printk(KERN_INFO "%s: parse poweron reason %s\n", __func__, pwron_event);

    return 1;
}

char boot_mode[16];
static int __init boot_mode_init(void)
{
    int i;
    char *substr = strstr(boot_command_line, "androidboot.mode=");
	 if(substr) {
    substr += strlen("androidboot.mode=");
    for(i=0; substr[i] != ' '; i++) {
        boot_mode[i] = substr[i];
    }
    boot_mode[i] = '\0';

	if(ftm_mode == MSM_BOOT_MODE__NORMAL){
	 	if(strncmp(substr, "charger", 7) == 0)
            ftm_mode = MSM_BOOT_MODE__CHARGE;	
		}
	}
    printk(KERN_INFO "%s: parse boot_mode is %s\n", __func__, boot_mode);
    return 1;
}
/* OPPO 2013-09-03 zhanglong add for add interface start reason and boot_mode end */
#endif /* VENDOR_EDIT */

static struct memtype_reserve msm8226_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static int msm8226_paddr_to_memtype(unsigned int paddr)
{
	return MEMTYPE_EBI1;
}

static struct of_dev_auxdata msm8226_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9824000, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF98A4000, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,msm-sdcc", 0xF9864000, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9824900, \
			"msm_sdcc.1", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF98A4900, \
			"msm_sdcc.2", NULL),
	OF_DEV_AUXDATA("qcom,sdhci-msm", 0xF9864900, \
			"msm_sdcc.3", NULL),
	OF_DEV_AUXDATA("qcom,hsic-host", 0xF9A00000, "msm_hsic_host", NULL),

	{}
};

static struct reserve_info msm8226_reserve_info __initdata = {
	.memtype_reserve_table = msm8226_reserve_table,
	.paddr_to_memtype = msm8226_paddr_to_memtype,
};

#ifdef VENDOR_EDIT
//Lycan.Wang@Prd.BasicDrv, 2014-02-08 Add for ram_console device (porting from find7)
static struct persistent_ram_descriptor msm_prd[] __initdata = {
    {
        .name = "ram_console",
        .size = SZ_1M,
    },
};

static struct persistent_ram msm_pr __initdata = {
    .descs = msm_prd,
    .num_descs = ARRAY_SIZE(msm_prd),
    .start = PLAT_PHYS_OFFSET + SZ_1G - SZ_1M,
    .size = SZ_1M,
};
#endif  /* VENDOR_EDIT */
static void __init msm8226_early_memory(void)
{
	reserve_info = &msm8226_reserve_info;
    of_scan_flat_dt(dt_scan_for_memory_hole, msm8226_reserve_table);
#ifdef VENDOR_EDIT
    //Lycan.Wang@Prd.BasicDrv, 2014-02-08 Add for ram_console device (porting from find7)
    persistent_ram_early_init(&msm_pr);
#endif  /* VENDOR_EDIT */
}

static void __init msm8226_reserve(void)
{
	reserve_info = &msm8226_reserve_info;
	of_scan_flat_dt(dt_scan_for_memory_reserve, msm8226_reserve_table);
	msm_reserve();
}

/*
 * Used to satisfy dependencies for devices that need to be
 * run early or in a particular order. Most likely your device doesn't fall
 * into this category, and thus the driver should not be added here. The
 * EPROBE_DEFER can satisfy most dependency problems.
 */
void __init msm8226_add_drivers(void)
{
	msm_smem_init();
	msm_init_modem_notifier_list();
	msm_smd_init();
	msm_rpm_driver_init();
	msm_spm_device_init();
	msm_pm_sleep_status_init();
	rpm_regulator_smd_driver_init();
	qpnp_regulator_init();
	if (of_board_is_rumi())
		msm_clock_init(&msm8226_rumi_clock_init_data);
	else
		msm_clock_init(&msm8226_clock_init_data);
	tsens_tm_init_driver();
	msm_thermal_device_init();
}

void __init msm8226_init(void)
{
	struct of_dev_auxdata *adata = msm8226_auxdata_lookup;
	#ifdef VENDOR_EDIT //Yixue.ge@ProDrv.BL add for ftm 2014-01-04
	int rc = 0;
	#endif //VENDOR_EDIT

	if (socinfo_init() < 0)
		pr_err("%s: socinfo_init() failed\n", __func__);
	#ifdef VENDOR_EDIT //Yixue.ge@ProDrv.BL add for ftm 2014-01-04
	board_mfg_mode_init();
	#endif /*VENDOR_EDIT*/

#ifdef VENDOR_EDIT
    /* OPPO 2013-09-03 zhanglong add for add interface start reason and boot_mode begin */
    start_reason_init();      
    boot_mode_init(); 
#ifndef VENDOR_EDIT
/* Xinqin.Yang@PhoneSW.MultiMedia, 2014/05/28  Delete for This function must after init_project_version() */
	init_lcd_gamaflag();
#endif /*CONFIG_VENDOR_EDIT*/
    /* OPPO 2013-09-03 zhanglong add for add interface start reason and boot_mode end */
#endif /* VENDOR_EDIT */

	msm8226_init_gpiomux();
	board_dt_populate(adata);
	msm8226_add_drivers();
	#ifdef VENDOR_EDIT //Yixue.ge@ProDrv.BL add for ftm 2014-01-04
	init_project_version();
#ifdef VENDOR_EDIT
/* Xinqin.Yang@PhoneSW.Driver, 2014/05/28  Add for multi gamma */
    init_lcd_gamaflag();
#endif /*CONFIG_VENDOR_EDIT*/
	//#endif /*VENDOR_EDIT*/	
	systeminfo_kobj = kobject_create_and_add("systeminfo", NULL);
	if (systeminfo_kobj)
		rc = sysfs_create_group(systeminfo_kobj, &attr_group);
	#endif /*VENDOR_EDIT*/	
}

static const char *msm8226_dt_match[] __initconst = {
	"qcom,msm8226",
	"qcom,msm8926",
	"qcom,apq8026",
	NULL
};

DT_MACHINE_START(MSM8226_DT, "Qualcomm MSM 8226 (Flattened Device Tree)")
	.map_io = msm_map_msm8226_io,
	.init_irq = msm_dt_init_irq,
	.init_machine = msm8226_init,
	.handle_irq = gic_handle_irq,
	.timer = &msm_dt_timer,
	.dt_compat = msm8226_dt_match,
	.reserve = msm8226_reserve,
	.init_very_early = msm8226_early_memory,
	.restart = msm_restart,
	.smp = &arm_smp_ops,
MACHINE_END
