#include <linux/module.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/kfifo.h>

#include <linux/firmware.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>

#include <mach/mt_boot.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_typedefs.h>
#ifdef CONFIG_MTK_IN_HOUSE_TEE_SUPPORT
#include <mach/mtk_boot_share_page.h>
#endif

//#include <mach/sbchk_base.h>

#define MOD "BOOT"

/* hardware version register */
#define VER_BASE            (DEVINFO_BASE)
#define APHW_CODE           (VER_BASE)
#define APHW_SUBCODE        (VER_BASE + 0x04)
#define APHW_VER            (VER_BASE + 0x08)
#define APSW_VER            (VER_BASE + 0x0C)

/* this vairable will be set by mt_fixup.c */

/*[FACTORY_TEST_BY_APK] modify start*/
BOOTMODE g_boot_mode_ex __nosavedata = UNKNOWN_BOOT;//Huangyisong_add 20130823   //  liguangxian0311
static ssize_t show_boot_status(struct device *dev,struct device_attribute *attr, char *buf)
{
   unsigned int ret_value = 1;
	if(11==g_boot_mode_ex)
//		ret_value = sprintf(buf, "%s", "1"); 	
		*buf = 1 ;	
	else
//		ret_value = sprintf(buf, "%s", "0");
  		*buf = 0 ;
	  return ret_value;
}
static  DEVICE_ATTR(boot_status, 0444,show_boot_status, NULL);
static int hq_boot_status(struct platform_device *dev)
{
		int ret_device_file = 0;
    printk("** hq_boot_status_probe!! **\n" );
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_boot_status)) != 0) goto exit_error;
exit_error:	
    return ret_device_file;
}
static struct platform_driver Hq_boot_status_driver = {
        .probe		= hq_boot_status,
        .driver     = {
        .name = "hq_boot_status",
    },
};

static struct platform_device Hq_boot_status_device = {
    .name   = "hq_boot_status",
    .id	    = -1,
};
static int __init Hq_boot_status_mod_init(void)
{
    int ret = 0;


    ret = platform_device_register(&Hq_boot_status_device);
    if (ret) {
        printk("**hq_boot_status_mod_init  Unable to driver register(%d)\n", ret);
        platform_driver_unregister(&Hq_boot_status_driver);
    }
    

    ret = platform_driver_register(&Hq_boot_status_driver);
    if (ret) {
        printk("**hq_boot_status_mod_init  Unable to driver register(%d)\n", ret);
        platform_device_unregister(&Hq_boot_status_device);
    }

    return ret;   
}

static void __exit Hq_boot_status_mod_exit(void)
{
		
        platform_driver_unregister(&Hq_boot_status_driver);
	platform_device_unregister(&Hq_boot_status_device);
}
module_init(Hq_boot_status_mod_init);
module_exit(Hq_boot_status_mod_exit);
/*[FACTORY_TEST_BY_APK] modify end*/

META_COM_TYPE g_meta_com_type = META_UNKNOWN_COM;
unsigned int g_meta_com_id = 0;

struct meta_driver {
    struct device_driver driver;
    const struct platform_device_id *id_table;
};

static struct meta_driver meta_com_type_info =
{
    .driver  = {
        .name = "meta_com_type_info",
        .bus = &platform_bus_type,
        .owner = THIS_MODULE,
    },
    .id_table = NULL,
};

static struct meta_driver meta_com_id_info =
{
    .driver = {
        .name = "meta_com_id_info",
        .bus = &platform_bus_type,
        .owner = THIS_MODULE,
    },
    .id_table = NULL,
};

/* boot attribute */
struct attribute info_attr = {INFO_SYSFS_ATTR, 0644};

/* return hardware version */
unsigned int get_chip_code(void)
{     
#ifdef CONFIG_MTK_IN_HOUSE_TEE_SUPPORT
    return *(unsigned int*)(BOOT_SHARE_BASE+BOOT_SHARE_DEV_INFO_OFST+0x00000000);
#else
    return DRV_Reg32(APHW_CODE);
#endif
}

unsigned int get_chip_hw_ver_code(void)
{   
#ifdef CONFIG_MTK_IN_HOUSE_TEE_SUPPORT
    return *(unsigned int*)(BOOT_SHARE_BASE+BOOT_SHARE_DEV_INFO_OFST+0x00000008);
#else
    return DRV_Reg32(APHW_VER);
#endif
}

unsigned int get_chip_sw_ver_code(void)
{  
#ifdef CONFIG_MTK_IN_HOUSE_TEE_SUPPORT
    return *(unsigned int*)(BOOT_SHARE_BASE+BOOT_SHARE_DEV_INFO_OFST+0x0000000c);
#else
    return DRV_Reg32(APSW_VER);
#endif
}

unsigned int get_chip_hw_subcode(void)
{
#ifdef CONFIG_MTK_IN_HOUSE_TEE_SUPPORT
    return *(unsigned int*)(BOOT_SHARE_BASE+BOOT_SHARE_DEV_INFO_OFST+0x00000004);
#else
    return DRV_Reg32(APHW_SUBCODE);
#endif
}

unsigned int mt_get_chip_id(void)
{
    unsigned int chip_id = get_chip_code();
    /*convert id if necessary*/
    return chip_id;
}

CHIP_SW_VER mt_get_chip_sw_ver(void)
{
    return (CHIP_SW_VER)get_chip_sw_ver_code();
}
/*[FACTORY_TEST_BY_APK] modify start*/
BOOTMODE get_boot_mode_ex(void)  //  liguangxian0311
{
    return g_boot_mode_ex;
}
/*[FACTORY_TEST_BY_APK] modify end*/

bool com_is_enable(void)  // usb android will check whether is com port enabled default. in normal boot it is default enabled. 
{	
    if(get_boot_mode() == NORMAL_BOOT)
	{	
        return false;
	}
	else
	{	
        return true;
	}
}

void set_meta_com(META_COM_TYPE type, unsigned int id)
{
    g_meta_com_type = type;
    g_meta_com_id = id;
}

META_COM_TYPE get_meta_com_type(void)
{
    return g_meta_com_type;
}

unsigned int get_meta_com_id(void)
{
    return g_meta_com_id;
}

static ssize_t meta_com_type_show(struct device_driver *driver, char *buf)
{
  return sprintf(buf, "%d\n", g_meta_com_type);
}

static ssize_t meta_com_type_store(struct device_driver *driver, const char *buf, size_t count)
{
  /*Do nothing*/
  return count;
}

DRIVER_ATTR(meta_com_type_info, 0644, meta_com_type_show, meta_com_type_store);


static ssize_t meta_com_id_show(struct device_driver *driver, char *buf)
{
  return sprintf(buf, "%d\n", g_meta_com_id);
}

static ssize_t meta_com_id_store(struct device_driver *driver, const char *buf, size_t count)
{
  /*Do nothing*/
  return count;
}

DRIVER_ATTR(meta_com_id_info, 0644, meta_com_id_show, meta_com_id_store);


static int __init boot_mod_init(void)
{
    int ret;
    BOOTMODE bm = get_boot_mode();
    
    if(bm == META_BOOT || bm == ADVMETA_BOOT || bm == ATE_FACTORY_BOOT || bm == FACTORY_BOOT)
    {
        /* register driver and create sysfs files */
        ret = driver_register(&meta_com_type_info.driver);
        if (ret) 
        {
            printk("fail to register META COM TYPE driver\n");
        }
        ret = driver_create_file(&meta_com_type_info.driver, &driver_attr_meta_com_type_info);
        if (ret) 
        {
            printk("[BOOT INIT] Fail to create META COM TPYE sysfs file\n");
        }

        ret = driver_register(&meta_com_id_info.driver);
        if (ret) 
        {
            printk("fail to register META COM ID driver\n");
        }
        ret = driver_create_file(&meta_com_id_info.driver, &driver_attr_meta_com_id_info);
        if (ret) 
        {
            printk("[BOOT INIT] Fail to create META COM ID sysfs file\n");
        }
    }    
    
    return 0;
}

static void __exit boot_mod_exit(void)
{
}

module_init(boot_mod_init);
module_exit(boot_mod_exit);
MODULE_DESCRIPTION("MTK Boot Information Querying Driver");
MODULE_LICENSE("Proprietary");
EXPORT_SYMBOL(mt_get_chip_id);
EXPORT_SYMBOL(mt_get_chip_sw_ver);
