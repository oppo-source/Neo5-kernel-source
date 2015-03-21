
#include <mach/oppo_project.h>
#include "board-dt.h"
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h> 
#include <mach/msm_smem.h>
#include <mach/oppo_reserve3.h>
#include <linux/fs.h>



/////////////////////////////////////////////////////////////

static struct proc_dir_entry *oppoReserve3 = NULL;

static unsigned int lcd_gamaflag = 0;

int __init  init_lcd_gamaflag(void)
{	
    char *substr;
	char buf[32] = {0};
	int i = 0;
//Added project control by Tong.han@BSP.group.TP for some device not ota LK
	if((is_project(OPPO_13095)) || (is_project(OPPO_14029))){
  	 	 substr = strstr(boot_command_line, "oppo.lcd_gamaflag=");
  		 if(substr) {
     		   substr += strlen("oppo.lcd_gamaflag=");
				while((substr[i] != ' ') && (i < 32))
				{
					buf[i] = substr[i];
					i++;
				}

				sscanf(buf, "%d", &lcd_gamaflag);
		

   	 	} 	
	}	
	return 0;

}

unsigned int get_lcd_gamaflag(void)
{

	return lcd_gamaflag;
}

#define BUF_SIZE 512

static int set_emmc_lcd_gamaflag(unsigned int flag)
{
	struct file* pfile = NULL;
	loff_t pos = 0;
	mm_segment_t fs;
	int num = -1;
	unsigned char buf[BUF_SIZE];
	reserve3_data *pdata;

	if(NULL == pfile){
		pfile = filp_open("/dev/block/platform/msm_sdcc.1/by-name/reserve3", O_RDWR, 0);
	}
	if(IS_ERR(pfile)){
		pr_err("error occured while opening file /dev/oppo_custom.\n");
		return -1;
	}	
	

	fs = get_fs();
	set_fs(KERNEL_DS);
	num = vfs_read(pfile, buf, BUF_SIZE, &pos);
	if(num != BUF_SIZE)
	{	
		
		printk("%s vfs_read fail %d\n",__func__,num);
		filp_close(pfile, NULL);
		return -1;
	}
	
	pdata = (reserve3_data *)buf;

	//init magic	
	if(pdata->oppomagic1 != OPFS_OPPOMAGIC1 || 
		pdata->oppomagic2 != OPFS_OPPOMAGIC2 ){	
		printk("set lcd gamaflag error for magic and init it!\n");	
		pdata->oppomagic1 = OPFS_OPPOMAGIC1;
		pdata->oppomagic2 = OPFS_OPPOMAGIC2;
	}
	
	pdata->gama_flag = flag;
	
	pos = 0;
	
	num = vfs_write(pfile, buf, BUF_SIZE, &pos);
	if(num != BUF_SIZE)
	{	
		
		printk("%s vfs_write fail %d\n",__func__,num);
		filp_close(pfile, NULL);
		return -1;
	}
	
	filp_close(pfile, NULL);
	
	set_fs(fs);
	
	return 0;
}

static ssize_t lcd_gamaflag_read_proc(struct file *pfile, char __user *buf, size_t count, loff_t *pos)
{
	char page[64];
	int len = sprintf(page,"%d",lcd_gamaflag);

	if (len > *pos)
		len -= *pos;	
	else
		len = 0;
	if (copy_to_user(buf,page,len < count ? len  : count))
		return -EFAULT;	
	*pos = *pos + (len < count ? len  : count);
	
	return len < count ? len  : count;

}

static ssize_t lcd_gamaflag_write_proc(struct file *pfile, const char __user *buf, size_t count, loff_t *pos)
{

	char page[64];
	int len = count;

	if(len > 64)
		len = 64;
	
	if (len > *pos)
		len -= *pos;	
	else
		len = 0;
	
	if(copy_from_user(page,buf,len))
		return len;

	sscanf(page, "%d", &lcd_gamaflag);
	if(set_emmc_lcd_gamaflag(lcd_gamaflag) < 0)
		return -1;

	*pos = *pos + (len < count ? len  : count);
	
	
	return len < count ? len  : count;
}


struct file_operations lcd_gamaflag_fops = {
	.read = lcd_gamaflag_read_proc,
	.write = lcd_gamaflag_write_proc,
};

static int __init oppo_reserve3_init(void)
{
	int ret = 0;
	struct proc_dir_entry *pentry;
	
	oppoReserve3 =  proc_mkdir ("oppoReserve3", NULL);
	if(!oppoReserve3) {
		pr_err("can't create oppoReserve3 proc\n");
		ret = -ENOENT;
	}

	pentry = proc_create ("lcd_gamaflag", S_IRUGO, oppoReserve3, &lcd_gamaflag_fops);
	if(!pentry) {
		pr_err("create prjVersion proc failed.\n");
		return -ENOENT;
	}
	
	return ret;
}

static void __exit oppo_reserve3_exit(void)
{

	remove_proc_entry("oppoReserve3", NULL);

}

module_init(oppo_reserve3_init);
module_exit(oppo_reserve3_exit);

/////////////////////////////////////////////////////////////
static struct proc_dir_entry *oppoVersion = NULL;
static ProjectInfoCDTType *format = NULL;



unsigned int init_project_version(void)
{	
	unsigned int len = (sizeof(ProjectInfoCDTType) + 3)&(~0x3);

	format = (ProjectInfoCDTType *)smem_alloc2(SMEM_PROJECT,len);

	if(format)
		return format->nProject;
	
	return 0;
}


unsigned int get_project(void)
{
	if(format)
		return format->nProject;
	return 0;
}

unsigned int is_project(OPPO_PROJECT project )
{
	return (get_project() == project?1:0);
}

unsigned char get_PCB_Version(void)
{
	if(format)
		return format->nPCBVersion;
	return 0;
}

unsigned char get_Modem_Version(void)
{
	if(format)
		return format->nModem;
	return 0;
}

unsigned char get_Operator_Version(void)
{
	if(format)
		return format->nOperator;
	return 0;
}


//this module just init for creat files to show which version
static int prjVersion_read_proc(char *page, char **start, off_t off,
			   int count, int *eof, void *data)
{
    unsigned char operator_version;
	int len;
	operator_version = get_Operator_Version();
	
    if(is_project(OPPO_14033)){
	    if(operator_version == 3)
		   len = sprintf(page,"%d",14035);
		else 
		   len = sprintf(page,"%d",14033);
    }
	else if(is_project(OPPO_14013)){
	    if(operator_version == 3)
		   len = sprintf(page,"%d",14016);
		else 
		   len = sprintf(page,"%d",14013);
    }
	else if(is_project(OPPO_14017)){
	    if(operator_version == 3)
		   len = sprintf(page,"%d",14018);
		else 
		   len = sprintf(page,"%d",14017);
    }
	else if(is_project(OPPO_14029)){
	    if(operator_version == 3)
		   len = sprintf(page,"%d",14031);
		else 
		   len = sprintf(page,"%d",14029);
    }		
	else if(is_project(OPPO_13095)){
	    if(operator_version == 3)
		   len = sprintf(page,"%d",13084);
		else 
		   len = sprintf(page,"%d",13095);
    }		
	else
	   len = sprintf(page,"%d",get_project());
	
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

static int pcbVersion_read_proc(char *page, char **start, off_t off,
			   int count, int *eof, void *data)
{
	int len = sprintf(page,"%d",get_PCB_Version());
	
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

static int operatorName_read_proc(char *page, char **start, off_t off,
			   int count, int *eof, void *data)
{
	int len = sprintf(page,"%d",get_Operator_Version());
	
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


static int modemType_read_proc(char *page, char **start, off_t off,
			   int count, int *eof, void *data)
{
	int len = sprintf(page,"%d",get_Modem_Version());
	
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

static int __init oppo_project_init(void)
{
	int ret = 0;
	struct proc_dir_entry *pentry;
	
	oppoVersion =  proc_mkdir ("oppoVersion", NULL);
	if(!oppoVersion) {
		pr_err("can't create oppoVersion proc\n");
		ret = -ENOENT;
	}

	pentry = create_proc_read_entry ("prjVersion", S_IRUGO, oppoVersion, prjVersion_read_proc,NULL);
	if(!pentry) {
		pr_err("create prjVersion proc failed.\n");
		return -ENOENT;
	}
	pentry = create_proc_read_entry ("pcbVersion", S_IRUGO, oppoVersion, pcbVersion_read_proc,NULL);
	if(!pentry) {
		pr_err("create pcbVersion proc failed.\n");
		return -ENOENT;
	}
	pentry = create_proc_read_entry ("operatorName", S_IRUGO, oppoVersion, operatorName_read_proc,NULL);
	if(!pentry) {
		pr_err("create operatorName proc failed.\n");
		return -ENOENT;
	}
	pentry = create_proc_read_entry ("modemType", S_IRUGO, oppoVersion, modemType_read_proc,NULL);
	if(!pentry) {
		pr_err("create modemType proc failed.\n");
		return -ENOENT;
	}
	
	return ret;
}

static void __exit oppo_project_init_exit(void)
{

	remove_proc_entry("oppoVersion", NULL);

}

module_init(oppo_project_init);
module_exit(oppo_project_init_exit);


MODULE_DESCRIPTION("OPPO project version");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Joshua <gyx@oppo.com>");
