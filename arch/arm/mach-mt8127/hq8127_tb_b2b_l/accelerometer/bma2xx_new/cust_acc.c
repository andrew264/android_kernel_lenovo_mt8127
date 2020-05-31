#include <linux/types.h>
#include "cust_acc.h"
#include <mach/mt_typedefs.h>
#include <mach/mt_pm_ldo.h>

/*---------------------------------------------------------------------------*/
static struct acc_hw cust_acc_hw2 = {
    .i2c_num = 2,
    .direction = 5,
    .power_id = MT65XX_POWER_NONE,
    .power_vol= VOL_DEFAULT,
    .firlen = 0,
};
/*---------------------------------------------------------------------------*/
struct acc_hw* get_cust_acc_hw2(void) 
{
    return &cust_acc_hw2;
}
