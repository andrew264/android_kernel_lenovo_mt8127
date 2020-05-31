#include <linux/kernel.h>	/* printk() */
#include <linux/module.h>
#include <linux/types.h>	/* size_t */
#include <linux/ctype.h>	/* toupper */
#include <linux/string.h>	/* strlen */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>
#include <linux/proc_fs.h>  /*proc*/
#include <linux/genhd.h>    //special
#include <linux/cdev.h>
#include <asm/uaccess.h>   /*set_fs get_fs mm_segment_t*/

#include "partition_define.h"
#include "pmt.h"
#include <linux/mmc/sd_misc.h>

//#define USING_XLOG

#ifdef USING_XLOG 
#include <linux/xlog.h>

#define TAG     "PMT"

#define pmt_err(fmt, args...)       \
    xlog_printk(ANDROID_LOG_ERROR, TAG, fmt, ##args)
#define pmt_info(fmt, args...)      \
    xlog_printk(ANDROID_LOG_INFO, TAG, fmt, ##args)

#else

#define TAG     "[PMT]"

#define pmt_err(fmt, args...)       \
    printk(KERN_ERR TAG);           \
    printk(KERN_CONT fmt, ##args) 
#define pmt_info(fmt, args...)      \
    printk(KERN_NOTICE TAG);        \
    printk(KERN_CONT fmt, ##args)

#endif

#define CONFIG_PMT_ENABLE


#ifdef CONFIG_MTK_EMMC_SUPPORT

static struct MBR_EBR_struct MBR_EBR_px[XBR_COUNT]={
    {"mbr", {0, 0, 0, 0, }},
    {"",    {0, 0, 0, 0, }},
    {"",    {0, 0, 0, 0, }},
};

int PART_NUM = 0;

static unsigned long long emmc_total_size = 0;
static unsigned long long emmc_user_size = 0;

static pt_resident *last_part;
static struct pt_info pi;

static int pmt_done = 0;

extern int eMMC_rw_x(loff_t addr,u32 *buffer, int host_num, int iswrite, u32 totalsize, int transtype, Region part);
extern void msdc_check_init_done(void);

#define PMT_REGION_SIZE     (0x1000)
#define PMT_REGION_OFFSET   (0x100000)

#define PMT_VER_V1P1        ("1.1")
#define PMT_VER_SIZE        (4)

#define MAIN_PT     0
#define MIRROR_PT   1

struct pmt_region;
struct pmt_region_ops {
    //int (*valid)(struct pmt_region *region, unsigned char *buf);
    int (*read)(struct pmt_region *region, void *buf);
    int (*write)(struct pmt_region *region, void *buf);
    int (*pack)(struct pmt_region *region, void *buf, pt_resident *part);
    int (*unpack)(struct pmt_region *region, void *buf, pt_resident *part);
};

struct pmt_region {
    unsigned long long base_addr;
    unsigned int size;
    unsigned int sig;
    unsigned char name[32];
    struct pmt_region_ops *ops;
};

#ifdef CONFIG_PMT_ENABLE
static int pmt_region_read(struct pmt_region *region, void *buf)
{
    int err;

    err = eMMC_rw_x(region->base_addr, (unsigned int *)buf, 0, 0, region->size, 1, EMMC_PART_USER);
    if (err) {
        pmt_err("[%s]read %s error\n", __func__, region->name);
    }

    return err;
}

static int pmt_region_write(struct pmt_region *region, void *buf)
{
    int err;

	err = eMMC_rw_x(region->base_addr, (unsigned int *)buf, 0, 1, region->size, 1, EMMC_PART_USER);
	if (err) {
        pmt_err("[%s]write %s error\n", __func__, region->name);
	}

    return err;
}

static int pmt_region_pack(struct pmt_region *region, void *buf, pt_resident *part)
{
    void *ptr;
    unsigned char sig[PT_SIG_SIZE];
    
    memset(buf, 0x0, region->size);

	*(int *)sig = region->sig;

    /* head sig info */
    ptr = buf;
    memcpy(ptr, sig, PT_SIG_SIZE); 

    /* version info */
    ptr += PT_SIG_SIZE;
    memcpy(ptr, PMT_VER_V1P1, PMT_VER_SIZE); 

    /* partition info */
    ptr += PMT_VER_SIZE;
	memcpy(ptr, part, PART_NUM * sizeof(pt_resident));

    /* pt_info info */
    ptr = buf + region->size - PT_SIG_SIZE - sizeof(pi);
    memcpy(ptr, &pi, sizeof(pi));

    /* tail sig info */
    ptr += sizeof(pi);
    memcpy(ptr, sig, PT_SIG_SIZE);

    return 0;
}

static int pmt_region_unpack(struct pmt_region *region, void *buf, pt_resident *part)
{
    int err;
    unsigned int head_sig, tail_sig;
    void *ptr;

    /* head sig info */
    ptr = buf;
    head_sig = *(unsigned int *)ptr;

    /* tail sig info */
    ptr += region->size - PT_SIG_SIZE;
    tail_sig = *(unsigned int *)ptr;

    if (head_sig != region->sig || tail_sig != region->sig) {
        err = -EINVAL;
        pmt_err("[%s]%s: head_sig = 0x%08x, tail_sig = 0x%08x\n",
                __func__, region->name, head_sig, tail_sig);
        goto out;
    }

    /* version info */
    ptr = buf + PT_SIG_SIZE;
    if (!memcmp(ptr, PMT_VER_V1P1, PMT_VER_SIZE)) {
        /* partition info */
        ptr += PMT_VER_SIZE;
        memcpy(part, ptr, PART_MAX_COUNT* sizeof(pt_resident));
        err = DM_ERR_OK;
        pmt_info(KERN_NOTICE "find %s at 0x%llx, ver %s\n", 
                region->name, region->base_addr, PMT_VER_V1P1);
    } else {
		pmt_err(KERN_ERR "invalid pt version 0x%x\n", *((u32 *)ptr));
        err = ERR_NO_EXIST;
        goto out;
    }

    /* pt_info info */
    ptr = buf + region->size - PT_SIG_SIZE - sizeof(pi);
    pi.sequencenumber = *(unsigned char *)ptr;
    ptr++;
    pi.tool_or_sd_update = *(unsigned char *)ptr & 0xF;

out:
    return err;
}

#else

static inline int pmt_region_read(struct pmt_region *region, void *buf)
{
    return -EBUSY;
}

static inline int pmt_region_write(struct pmt_region *region, void *buf)
{
    return -EBUSY;
}

static inline int pmt_region_pack(struct pmt_region *region, void *buf, pt_resident *part)
{
    return -EBUSY;
}

static inline int pmt_region_unpack(struct pmt_region *region, void *buf, pt_resident *part)
{
    return -EBUSY;
}


#endif


static struct pmt_region_ops pmt_ops = {
    .read = pmt_region_read,
    .write = pmt_region_write,
    .pack = pmt_region_pack,
    .unpack = pmt_region_unpack,
};


#define PMT_REGION_IDX_MAIN     (0)
#define PMT_REGION_IDX_MIRROR   (1)
#define PMT_REGION_NRS          (2)

static struct pmt_region pmt_regions[] = {
    {
        .size = PMT_REGION_SIZE,
        .sig = PT_SIG,
        .name = "main-pt",
        .ops = &pmt_ops,
    }, { 
        .size = PMT_REGION_SIZE,
        .sig = MPT_SIG,
        .name = "mirror-pt",
        .ops = &pmt_ops,
    },
};

static void init_pmt_region(void)
{
    unsigned long long pt_start;
    unsigned long long mpt_start;

    pt_start = emmc_user_size - PMT_REGION_OFFSET;		
    mpt_start = pt_start + PMT_REGION_SIZE;

    pmt_regions[PMT_REGION_IDX_MAIN].base_addr = pt_start;
    pmt_regions[PMT_REGION_IDX_MIRROR].base_addr = mpt_start;
}


static int __load_pmt(struct pmt_region *region, pt_resident *part)
{
    int err;
    void *buf;

    buf = kzalloc(region->size, GFP_KERNEL);
	if (!buf) {
		err = -ENOMEM;
	    pmt_err("[%s]fail to malloc buf\n", __func__);
        goto fail_malloc;
	}

    err = region->ops->read(region, buf);
    if (err) {
        goto out;
    }

    err = region->ops->unpack(region, buf, part);
    if (err) {
        goto out;
    }

out:
    kfree(buf);

fail_malloc:
    return err;
}

static int __store_pmt(struct pmt_region *region, pt_resident *new_part)
{
	int err;
    void *buf;

    buf = kzalloc(region->size, GFP_KERNEL);
	if (!buf) {
		err = -ENOMEM;
	    pmt_err("[%s]fail to malloc buf\n", __func__);
        goto fail_malloc;
	}
	
	err = region->ops->write(region, buf);
	if (err) {
		goto out;
	}

    err = region->ops->pack(region, buf, new_part);
    if (err) {
        goto out;
    }
	
	err = region->ops->write(region, buf);
	if (err) {
		goto out;
	}

out:
    kfree(buf);

fail_malloc:
	return err;
}

static int load_pmt(void)
{
    int err;
    int i;

    for (i = 0; i < PMT_REGION_NRS; i++) {
        err = __load_pmt(pmt_regions + i, last_part);
        if (!err) {
            break;
        }
    }

    return err;
}

#define PMT_UPDATE_BY_UNKNOWN       (0)
#define PMT_UPDATE_BY_TEST          (1)
#define PMT_UPDATE_BY_UPGRADE       (2)
#define PMT_UPDATE_BY_TOOL          (3)
#define PMT_UPDATE_REASONS          (4)

static const char *const pmt_update_reason[] = {
    [PMT_UPDATE_BY_UNKNOWN] = "unknown",
    [PMT_UPDATE_BY_TOOL]    = "tool",
    [PMT_UPDATE_BY_TEST]    = "test",
    [PMT_UPDATE_BY_UPGRADE] = "upgrade",
};

static int store_pmt(int reason)
{
    int err;
    int i;

    BUG_ON(reason >= PMT_UPDATE_REASONS || reason < 0);
    pi.sequencenumber += 1;
    pi.tool_or_sd_update = reason;

    for (i = 1; i >= 0; i--) {
        err = __store_pmt(pmt_regions + i, last_part);
        if (err) {
            break;
        }
    }

    return err;
}


static int read_pmt(void __user *arg)
{
    pmt_info("read_pmt\n");
    if (copy_to_user(arg, last_part, sizeof(pt_resident) * PART_NUM)) {
        return -EFAULT;
    }
    return 0;
}


static int write_pmt(void __user *arg)
{
    int err = 0;
    pt_resident *new_part;

    new_part = kmalloc(PART_NUM * sizeof(pt_resident), GFP_KERNEL);
    if (!new_part) {
        err = -ENOMEM;
        pmt_err("write_pmt: malloc new_part fail\n");
        goto fail_malloc;
    }

    if (copy_from_user(new_part, arg, PART_NUM * sizeof(pt_resident))) {
        err = -EFAULT;
        goto out;
    }

    err = store_pmt(PMT_UPDATE_BY_TEST);

out:
    kfree(new_part);

fail_malloc:
    return err;
}


static void init_storage_info(void)
{
	struct storage_info s_info = {0};

    msdc_check_init_done();

    BUG_ON(!msdc_get_info(EMMC_CARD_BOOT, EMMC_CAPACITY, &s_info));
    BUG_ON(!msdc_get_info(EMMC_CARD_BOOT, EMMC_USER_CAPACITY, &s_info)); 

	emmc_user_size = s_info.emmc_user_capacity * 512;
	emmc_total_size = s_info.emmc_capacity * 512;
	pmt_info("[%s]emmc_total_size = 0x%llx, user_region_size = 0x%llx\n", __func__,
                emmc_total_size, emmc_user_size);
}

#ifdef CONFIG_MSDOS_PARTITION

#define MSDOS_LABEL_MAGIC1	0x55
#define MSDOS_LABEL_MAGIC2	0xAA

static inline int msdos_magic_present(unsigned char *p)
{
	return (p[0] == MSDOS_LABEL_MAGIC1 && p[1] == MSDOS_LABEL_MAGIC2);
}

#endif

static int parsing_xbr(u64 *xbr_addr, u64 *off2mbr)
{
	int err;
	int slot;
    u8 xbr_buf[512];
    struct partition *p;
    u64 offset;

    u64 this_addr;
    int is_mbr;
    int next;

    this_addr = *xbr_addr;
    is_mbr = (this_addr == 0ULL) ? 1 : 0;
    next = 0;

	err = eMMC_rw_x(this_addr, (unsigned int *)xbr_buf, 0, 0, 512, 1, EMMC_PART_USER);
	if (err || !msdos_magic_present(xbr_buf + 510)) {
		pmt_info("read xbr(0x%llx) error %d (0x%x 0x%x)\n", 
            this_addr, err, xbr_buf[510], xbr_buf[511]);
        next = 0;
		goto out;
	}

    p = (struct partition *)(xbr_buf + 0x1be);

	for (slot = 0; slot < 4; slot++, p++) {
		if (!p->nr_sects)
			continue;

		offset = (u64)p->start_sect * 512 + this_addr;
        if (p->sys_ind == LINUX_DATA_PARTITION) {
            off2mbr[slot] = offset;
        }
    }

    p -= 4;    
	for (slot = 0; slot < 4; slot++, p++) {
		offset = (u64)p->start_sect * 512 + this_addr;
        if (p->sys_ind == DOS_EXTENDED_PARTITION) {
            if (is_mbr) {
                off2mbr[slot] = offset;
            }
    		*xbr_addr = offset;
            next = 1;
            break;
        }
	}

out:
	return next;
}

static void construct_xbr_pmt_mapping(void)
{
    u64 off2mbr[XBR_COUNT][4];
    u64 xbr_addr;
    int next;

    int xbr_idx, xbr_loops;
    int slot;
    int pmt_idx;
    int partno;

    memset(off2mbr, 0x0, sizeof(u64) * 4 * XBR_COUNT);

    xbr_addr = 0;
    for (xbr_idx = 0; xbr_idx < XBR_COUNT; xbr_idx++) {
        next = parsing_xbr(&xbr_addr, off2mbr[xbr_idx]);
        if (!next) {
            break;
        }
    }
    xbr_loops = min(xbr_idx + 1, XBR_COUNT);

    partno = 1;
    for (xbr_idx = 0; xbr_idx < xbr_loops; xbr_idx++) {
        if (xbr_idx) {
            snprintf(MBR_EBR_px[xbr_idx].xbr_name, 
                sizeof(MBR_EBR_px[xbr_idx].xbr_name),"%s%d", "ebr", xbr_idx);
        }

        for (slot = 0; slot < 4; slot++) {
            if (!off2mbr[xbr_idx][slot]) 
                continue;

            for (pmt_idx = 0; pmt_idx < PART_NUM; pmt_idx++) {
                if (PartInfo[pmt_idx].partition_idx)
                    continue;

                if (last_part[pmt_idx].offset == 
                    off2mbr[xbr_idx][slot] ) {
                    MBR_EBR_px[xbr_idx].part_index[slot] = partno;
                    PartInfo[pmt_idx].partition_idx = partno;
                    partno++;
                    continue;
                }
            }
        }
    }
}

int init_pmt(void)
{
	int err;
	int i = 0, j = 0, len;

#ifdef CONFIG_PMT_ENABLE
    pmt_info("[%s]start...(CONFIG_PMT_ENABLE=y)\n", __func__);
#else
    pmt_info("[%s]start...(CONFIG_PMT_ENABLE=n)\n", __func__);
#endif

	if (pmt_done) {
		pmt_info("[%s]skip since init already\n", __func__);
		return 0;
	}

    init_storage_info();

    init_pmt_region();

	last_part = kzalloc(PART_MAX_COUNT* sizeof(pt_resident), GFP_KERNEL);
	if (!last_part) {
		err = -ENOMEM;
		pmt_err("[%s]fail to malloc last_part\n", __func__);
		goto fail_malloc;
	}

    memset(&pi, 0, sizeof(struct pt_info));

    err = load_pmt();
    if (err) { 
        err = -ENODEV;
        pmt_err("[%s]No pmt found\n", __func__);
        goto fail_find;
    }

    while (i < PART_MAX_COUNT && last_part[i].size) {
		PartInfo[i].start_address = last_part[i].offset;
		PartInfo[i].size = last_part[i].size;
		PartInfo[i].type = EMMC;
		PartInfo[i].partition_idx = 0;

		len = strlen(last_part[i].name);
		PartInfo[i].name = kzalloc(len + 1, GFP_KERNEL);//IPOH ??
		for (j = 0; j < len; j++) {
			PartInfo[i].name[j] = tolower(last_part[i].name[j]);
		}
        PartInfo[i].name[len] = '\0';

		if (!strcmp(PartInfo[i].name, "preloader")) {
			PartInfo[i].region = EMMC_PART_BOOT1;
		} else {
			PartInfo[i].region = EMMC_PART_USER;
		}

        i++;
    }

    PART_NUM = i;

    if (!PART_NUM) {
        err = -EINVAL;
        pmt_err("PART_NUM invald: PART_NUM=%d\n", PART_NUM);
        goto fail_find;
    }


	construct_xbr_pmt_mapping();

    for (i = 0; i < PART_NUM; i++) {  
        pmt_info("Partition[%2d](%-16s) - start(0x%16llx) - size(0x%16llx)\n",
                i, PartInfo[i].name, PartInfo[i].start_address, PartInfo[i].size);
    }

	pmt_done = 1;
	err = 0;
    return err;

fail_find:
    kfree(last_part);
fail_malloc: 
	return err;
}
EXPORT_SYMBOL(init_pmt);


static int unpack_and_verify(struct DM_PARTITION_INFO_PACKET_x *packet, 
            pt_resident *new_part, int *changed, int *pt_change_tb)
{
    int err = 0;
    int i;

    //copy new part
	for (i = 0; i < PART_NUM; i++) {
		memcpy(new_part[i].name, packet->part_info[i].part_name, MAX_PARTITION_NAME_LEN);
		new_part[i].part_id = packet->part_info[i].part_id;
		new_part[i].offset = packet->part_info[i].start_addr;
		new_part[i].size = packet->part_info[i].part_len;
		new_part[i].mask_flags = 0;
		pmt_info("[SD_UPGRADE]new_pt %s size %llx \n", new_part[i].name, new_part[i].size);
	}

    //compare new part and lastest part.
	for (i = 0; i < PART_NUM; i++) {
		if (new_part[i].size != last_part[i].size
            || new_part[i].part_id != last_part[i].part_id 
            || new_part[i].offset != last_part[i].offset) {
			pmt_info("[SD_UPGRADE]new_pt %d size changed from %llx to %llx\n", i, last_part[i].size, new_part[i].size);
			*changed = 1;
			pt_change_tb[i] = 1;
			if ((packet->part_info[i].dl_selected == 0)
                && (packet->part_info[i].visible == 1)) {
				pmt_err("[SD_UPGRADE]please download all image\n");
				err = -EINVAL;
                break;
			}
		}
	}

    return err;
}

static void update_part_size(pt_resident *new_part, int *pt_change_tb)
{
    int i;
	struct storage_info s_info = {0};

	BUG_ON(!msdc_get_info(EMMC_CARD_BOOT, EMMC_RESERVE, &s_info));
	for (i = 0; i < PART_NUM; i++) {
        if (pt_change_tb[i] == 1 && new_part[i].size == 0) {
            new_part[i].size = emmc_user_size - new_part[i].offset - s_info.emmc_reserve * 512;
        }
	}
}



static int __update_msdos_partition(int px, unsigned long long start, unsigned long long size)
{
    int err = 0;

	int i, slot; 
	int xbr_idx = 0;
	char *this_name = NULL;
	unsigned long long this_addr;

	unsigned char *buf = NULL;
	struct partition *p;
		
	buf = kzalloc(512, GFP_KERNEL);
	if (!buf) {
		err = -ENOMEM;
		pmt_err("update_msdos_partition: malloc buf fail\n");
		goto fail_malloc;
	}
	
	//find px in which mbr/ebr.
	for (i = 0;i < XBR_COUNT; i++) {
		for (slot = 0; slot < 4; slot++) {
			if (MBR_EBR_px[i].part_index[slot] == px) {
                /* this_name is mbr or ebrx */
				xbr_idx = i;
				this_name = MBR_EBR_px[i].xbr_name;  
                goto found;
			}
		}
	}

	if (slot >= 4) {
		pmt_err("p%d can not be found in mbr\n", px);
		err = -EINVAL;
		goto out;
	}

found:
	pmt_info("update %s\n", this_name);

    /* check mbr or ebrx partition info */
	for (i = 0; i < PART_NUM; i++) {
		if (!strcmp(this_name, PartInfo[i].name)) {
			//this_addr = PartInfo[i].start_address;
			//pmt_info("update %s addr %llx\n", this_name, this_addr);
			break;
		}
	}

	if (i == PART_NUM) {
		pmt_err("can not find %s\n", this_name);
		err = -EINVAL;
		goto out;
	} 

	this_addr = PartInfo[i].start_address;
	pmt_info("update %s addr %llx\n", this_name, this_addr);

    /* read mbr or ebr into buf */
	err = eMMC_rw_x(this_addr, (u32*)buf, 0, 0, 512, 1, EMMC_PART_USER);
	if (err || !msdos_magic_present(buf + 510)) {
		pmt_err("read %s error\n", this_name);
        err = -EIO;
		goto out;
	}

	if (!msdos_magic_present(buf + 510)) {
		pmt_err("read MBR/EBR fail\n");
		err = -1;
		goto out;
	}

	p = (struct partition *) (buf + 0x1be);

	p[slot].start_sect = (unsigned int)((start - this_addr) / 512);
	p[slot].nr_sects = (unsigned int)(size / 512);

	err = eMMC_rw_x(this_addr, (u32*)buf, 0, 1, 512, 1, EMMC_PART_USER);
	if (err) {
		pmt_err("write %s error\n", this_name);
        err = -EIO;
		goto out;
	}

out:
	kfree(buf);
fail_malloc:
	return err;
}

static int update_msdos_partition(pt_resident *new_part, int *pt_change_tb)
{
    int err = 0;
    int i;
    int px;
    unsigned long long start;
    unsigned long long size;

	for (i = 0; i < PART_NUM; i++) {
		if (pt_change_tb[i] == 1 && PartInfo[i].partition_idx != 0) {
            px = PartInfo[i].partition_idx;
            start = new_part[i].offset;
            size = new_part[i].size;
			pmt_info("update p %d %llx %llx\n", px, start, size);
			err = __update_msdos_partition(px, start, size);
			if (err) {
                err = -1;
				pmt_err("[SD_UPGRADE]update_msdos_partition fail\n");
				break;
			}
		}
	}

    return err;
}

#ifdef CONFIG_MTK_EMMC_SUPPORT
static int __update_disk_info(unsigned int px, unsigned int start, unsigned int size)
{
    int found;
    struct disk_part_iter piter;
	struct hd_struct *part;
	struct gendisk *disk;
	struct storage_info s_info = {0};

	BUG_ON(!msdc_get_info(EMMC_CARD_BOOT, DISK_INFO, &s_info));
	disk = s_info.disk;

    found = 0;

    disk_part_iter_init(&piter, disk, 0);

    while ((part = disk_part_iter_next(&piter))) {
        if (px != 0 && px == part->partno) {
            found = 1;
            pmt_info("[update_disk_info]px = %d size %llx -> %x offset %llx -> %x\n",
                    px, part->nr_sects, size, part->start_sect, start);
            part->start_sect = start;
            part->nr_sects = size;
            break;
        }
    }

    disk_part_iter_exit(&piter);

    return !found;
}


static int update_disk_info(pt_resident *new_part, int *pt_change_tb)
{
    int err = 0;
    int i;
    int px;
    unsigned long long start;
    unsigned long long size;


	for (i = 0; i < PART_NUM; i++) {
		if (pt_change_tb[i] == 1 && PartInfo[i].partition_idx != 0) {
            px = PartInfo[i].partition_idx;
            start = new_part[i].offset;
            size = new_part[i].size;
			pmt_info("update p %d %llx %llx\n", px, start, size);
			err = __update_disk_info(px, (u32)(start / 512), (u32)(size/512));
			if (err) {
                err = -1;
				pmt_err("[SD_UPGRADE]update part device offset and size fail\n");
                break;
			}
		}
	}

    return err;
}

#endif

static int pmt_upgrade_handler(struct DM_PARTITION_INFO_PACKET_x *packet)
{
    int err;
	pt_resident *new_part;
	int pt_changed = 0;
	int pt_change_tb[PART_MAX_COUNT] = {0};

	new_part = kzalloc(PART_NUM * sizeof(pt_resident), GFP_KERNEL);
	if (!new_part) {
        err = -ENOMEM;
        pmt_err("sd_upgrade_handler: fail to malloc new_part\n");
        goto fail_malloc;
	}

    err = unpack_and_verify(packet, new_part, &pt_changed, pt_change_tb);
    if (err) {
        goto out;
    }

	if (!pt_changed) {
		pmt_info("[SD_UPGRADE]layout can not change,skip update PMT/MBR\n");
		goto out;
	}

    memcpy(last_part, new_part, PART_NUM * sizeof(pt_resident));
    err = store_pmt(PMT_UPDATE_BY_UPGRADE);

    update_part_size(new_part, pt_change_tb);

    err = update_msdos_partition(new_part, pt_change_tb);
    if (err) {
        goto out;
    }

    err = update_disk_info(new_part, pt_change_tb);
    if (err) {
        goto out;
    }

out:
	kfree(new_part);

fail_malloc:
    return err;
}

static int pmt_upgrade_proc_write(struct file *file, const char __user *buffer, 
                size_t size, loff_t *ppos)
{
	struct DM_PARTITION_INFO_PACKET_x *packet;
	int err;

	packet = kzalloc(sizeof(*packet), GFP_KERNEL);
	if (!packet) {
        err = -ENOMEM;
        pmt_err("upgrade_proc_write: fail to malloc packet\n");
        goto fail_malloc;
	}
	
	if (copy_from_user(packet, buffer, sizeof(*packet))) {
		err = -EFAULT;
		goto out;
	}

    err = pmt_upgrade_handler(packet);
	
out:
	kfree(packet);
fail_malloc:
	if (err)
		return err;
	else
		return size;
}

static const struct file_operations pmt_upgrade_proc_fops = { 
    .write = pmt_upgrade_proc_write,
};


#define  PMT_MAGIC   'p'
#define PMT_READ        _IOW(PMT_MAGIC, 1, int)
#define PMT_WRITE       _IOW(PMT_MAGIC, 2, int)


static long pmt_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long err;
    void __user *argp = (void __user *)arg;

    switch (cmd)
    {
        case PMT_READ:
            err = read_pmt(argp);
            break;
        case PMT_WRITE:
            err = write_pmt(argp);
            break;
        default:
            err = -EINVAL;
    }
    return err;
}


static unsigned int major;
static struct class *pmt_class;
static struct cdev *pmt_cdev;
static struct file_operations pmt_cdev_ops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = pmt_ioctl,
};

static void create_pmt_cdev(void)
{
    int err;
    dev_t devno;
    struct device *pmt_dev;

    err = alloc_chrdev_region(&devno, 0, 1, "pmt");
    if (err) {
        pmt_err("[%s]fail to alloc devno\n", __func__);
        goto fail_alloc_devno;
    }
    
    major = MAJOR(devno);

    pmt_cdev = cdev_alloc();
    if (!pmt_cdev) {
        pmt_err("[%s]fail to alloc cdev\n", __func__);
        goto fail_alloc_cdev;
    }

    pmt_cdev->owner = THIS_MODULE;
    pmt_cdev->ops = &pmt_cdev_ops;

    err = cdev_add(pmt_cdev, devno, 1);
    if (err) {
        pmt_err("[%s]fail to add cdev\n", __func__);
        goto fail_add_cdev;
    }

    pmt_class = class_create(THIS_MODULE, "pmt");
    if (IS_ERR(pmt_class)) {
        pmt_err("[%s]fail to create class pmt\n", __func__);
        goto fail_create_class;
    }
    
    pmt_dev = device_create(pmt_class, NULL, devno, NULL, "pmt");
    if (IS_ERR(pmt_dev)) {
        pmt_err("[%s]fail to create class pmt\n", __func__);
        goto fail_create_device;
    }

    return;

fail_create_device:
    class_destroy(pmt_class);
fail_create_class:
fail_add_cdev:
    cdev_del(pmt_cdev);
fail_alloc_cdev:
    unregister_chrdev_region(devno, 1);
fail_alloc_devno:
    return;
}

static void remove_pmt_cdev(void)
{
    device_destroy(pmt_class, MKDEV(major, 0));
    class_destroy(pmt_class);
    cdev_del(pmt_cdev);
    unregister_chrdev_region(MKDEV(major, 0), 1);
}

static int __init pmt_init(void)
{
    struct proc_dir_entry *pmt_upgrade_proc;

    pmt_upgrade_proc = proc_create("sd_upgrade", 0600, NULL, &pmt_upgrade_proc_fops);
    if (!pmt_upgrade_proc) {
        pmt_err("[%s]fail to register /proc/sd_upgrade\n", __func__);
    }

    create_pmt_cdev();

    return 0;
}

static void __exit pmt_exit(void)
{
    remove_proc_entry("sd_upgrade", NULL);

    remove_pmt_cdev();
}

module_init(pmt_init);
module_exit(pmt_exit);

#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek Partition Table Management Driver");
MODULE_AUTHOR("Jiequn.Chen <Jiequn.Chen@mediatek.com>");
