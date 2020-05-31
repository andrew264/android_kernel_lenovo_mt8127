
#ifndef __NAND_DEVICE_LIST_H__
#define __NAND_DEVICE_LIST_H__

#define NAND_ABTC_ATAG
#define ATAG_FLASH_NUMBER_INFO       0x54430006
#define ATAG_FLASH_INFO       0x54430007

struct tag_nand_number {
	u32 number;
};

#define MAX_FLASH 20 //modify this define if device list is more than 20 later .xiaolei

#define NAND_MAX_ID		6
#define CHIP_CNT		10
#define P_SIZE		16384
#define P_PER_BLK		256
#define C_SIZE		8192
#define RAMDOM_READ		(1<<0)
#define CACHE_READ		(1<<1)
#define RAND_TYPE_SAMSUNG 0
#define RAND_TYPE_TOSHIBA 1
#define RAND_TYPE_NONE 2

#define READ_RETRY_MAX 10
struct gFeature
{
	u32 address;
	u32 feature;
};

enum readRetryType
{
	RTYPE_MICRON,
	RTYPE_SANDISK,
	RTYPE_SANDISK_19NM,
	RTYPE_TOSHIBA,
	RTYPE_HYNIX,
	RTYPE_HYNIX_16NM
};

struct gFeatureSet
{
	u8 sfeatureCmd;
	u8 gfeatureCmd;
	u8 readRetryPreCmd;
	u8 readRetryCnt;
	u32 readRetryAddress;
	u32 readRetryDefault;
	u32 readRetryStart;
	enum readRetryType rtype;
	struct gFeature Interface;
	struct gFeature Async_timing;
};

struct gRandConfig
{
	u8 type;
	u32 seed[6];
};

enum pptbl
{
	MICRON_8K,
	HYNIX_8K,
	SANDISK_16K,
};

struct MLC_feature_set
{
	enum pptbl ptbl_idx;
	struct gFeatureSet 	 FeatureSet;
	struct gRandConfig   randConfig;
};

enum flashdev_vendor
{
	VEND_SAMSUNG,
	VEND_MICRON,
	VEND_TOSHIBA,
	VEND_HYNIX,
	VEND_SANDISK,
	VEND_BIWIN,
	VEND_NONE,
};

enum flashdev_IOWidth
{
	IO_8BIT = 8,
	IO_16BIT = 16,
	IO_TOGGLEDDR = 9,
	IO_TOGGLESDR = 10,
	IO_ONFI = 12,
};

typedef struct
{
   u8 id[NAND_MAX_ID];
   u8 id_length;
   u8 addr_cycle;
   u32 iowidth;
   u16 totalsize;
   u16 blocksize;
   u16 pagesize;
   u16 sparesize;
   u32 timmingsetting;
   u32 s_acccon;
   u32 s_acccon1;
   u32 freq;
   u16 vendor;
   u16 sectorsize;
   u8 devciename[30];
   u32 advancedmode;
   struct MLC_feature_set feature_set;
}flashdev_info_t,*pflashdev_info;
#if 0
static const flashdev_info gen_FlashTable[]={
	{{0x45,0xDE,0x94,0x93,0x76,0x57}, 6,5,IO_8BIT,8192,4096,16384,1280,0x10804222, 0xC03222,0x101,80,VEND_SANDISK,1024, "SDTNQGAMA008G ",0 ,
		{SANDISK_16K, {0xEF,0xEE,0xFF,16,0x11,0,1,RTYPE_SANDISK_19NM,{0x80, 0x00},{0x80, 0x01}},
		{RAND_TYPE_SAMSUNG,{0x2D2D,1,1,1,1,1}}}},
	{{0x98,0xD7,0x84,0x93,0x72,0x00}, 5,5,IO_8BIT,4096,4096,16384,1280,0x10804222, 0xC03222,0x101,80,VEND_TOSHIBA,1024, "TC58TEG5DCKTA00",0 ,
		{SANDISK_16K, {0xEF,0xEE,0xFF,7,0xFF,7,0,RTYPE_TOSHIBA,{0x80, 0x00},{0x80, 0x01}},
		{RAND_TYPE_SAMSUNG,{0x2D2D,1,1,1,1,1}}}},
	{{0x45,0xDE,0x94,0x93,0x76,0x00}, 5,5,IO_8BIT,8192,4096,16384,1280,0x10804222, 0xC03222,0x101,80,VEND_SANDISK,1024, "SDTNRGAMA008GK ",0 ,
		{SANDISK_16K, {0xEF,0xEE,0x5D,32,0x11,0,0xFFFFFFFF,RTYPE_SANDISK,{0x80, 0x00},{0x80, 0x01}},
		{RAND_TYPE_SAMSUNG,{0x2D2D,1,1,1,1,1}}}},
	{{0xAD,0xDE,0x14,0xA7,0x42,0x00}, 5,5,IO_8BIT,8192,4096,16384,1280,0x10804222, 0xC03222,0x101,80,VEND_HYNIX,1024, "H27UCG8T2ETR",0 ,
		{SANDISK_16K, {0xFF,0xFF,0xFF,7,0xFF,0,1,RTYPE_HYNIX_16NM,{0XFF, 0xFF},{0XFF, 0xFF}},
		{RAND_TYPE_SAMSUNG,{0x2D2D,1,1,1,1,1}}}},
	{{0x2C,0x44,0x44,0x4B,0xA9,0x00}, 5,5,IO_8BIT,4096,2048,8192,640,0x10804222, 0xC03222,0x101,80,VEND_MICRON,1024, "MT29F32G08CBADB ",0 ,
		{MICRON_8K, {0xEF,0xEE,0xFF,7,0x89,0,1,RTYPE_MICRON,{0x1, 0x14},{0x1, 0x5}},
		{RAND_TYPE_SAMSUNG,{0x2D2D,1,1,1,1,1}}}},
	{{0xAD,0xDE,0x94,0xA7,0x42,0x00}, 5,5,IO_8BIT,8192,4096,16384,1280,0x10804222, 0xC03222,0x101,80,VEND_BIWIN,1024, "BW27UCG8T2ETR",0 ,
		{SANDISK_16K, {0xFF,0xFF,0xFF,7,0xFF,0,1,RTYPE_HYNIX_16NM,{0XFF, 0xFF},{0XFF, 0xFF}},
		{RAND_TYPE_SAMSUNG,{0x2D2D,1,1,1,1,1}}}},
	{{0x45,0xD7,0x84,0x93,0x72,0x00}, 5,5,IO_8BIT,4096,4096,16384,1280,0x10804222, 0xC03222,0x101,80,VEND_SANDISK,1024, "SDTNRGAMA004GK ",0 ,
		{SANDISK_16K, {0xEF,0xEE,0x5D,32,0x11,0,0xFFFFFFFF,RTYPE_SANDISK,{0x80, 0x00},{0x80, 0x01}},
		{RAND_TYPE_SAMSUNG,{0x2D2D,1,1,1,1,1}}}},
	{{0x2C,0x64,0x44,0x4B,0xA9,0x00}, 5,5,IO_8BIT,8192,2048,8192,640,0x10804222, 0xC03222,0x101,80,VEND_MICRON,1024, "MT29F128G08CFABA ",0 ,
		{MICRON_8K, {0xEF,0xEE,0xFF,7,0x89,0,1,RTYPE_MICRON,{0x1, 0x14},{0x1, 0x5}},
		{RAND_TYPE_SAMSUNG,{0x2D2D,1,1,1,1,1}}}},
	{{0xAD,0xD7,0x94,0x91,0x60,0x00}, 5,5,IO_8BIT,4096,2048,8192,640,0x10804222, 0xC03222,0x101,80,VEND_HYNIX,1024, "H27UBG8T2CTR",0 ,
		{HYNIX_8K, {0xFF,0xFF,0xFF,7,0xFF,0,1,RTYPE_HYNIX,{0XFF, 0xFF},{0XFF, 0xFF}},
		{RAND_TYPE_SAMSUNG,{0x2D2D,1,1,1,1,1}}}},
	{{0x98,0xDE,0x94,0x93,0x76,0x00}, 5,5,IO_8BIT,8192,4096,16384,1280,0x10804222, 0xC03222,0x101,80,VEND_TOSHIBA,1024, "TC58TEG6DDKTA00",0 ,
		{SANDISK_16K, {0xEF,0xEE,0xFF,7,0xFF,7,0,RTYPE_TOSHIBA,{0x80, 0x00},{0x80, 0x01}},
		{RAND_TYPE_SAMSUNG,{0x2D2D,1,1,1,1,1}}}},
};
#endif
#endif
