#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>   //wake_up_process()
#include <linux/kthread.h> //kthread_create()kthread_run()
#include <mach/irqs.h>
#include <asm/io.h>

#include <mach/mt_smi.h>
#include <mach/m4u_reg.h>
#include "smi_common.h"

#include <mach/mt_typedefs.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_irq.h>
#include <mach/m4u.h>

#define SMI_LOG_TAG "MAU"


int mau_enable_interrupt(int larb)
{
    unsigned int larb_base = get_larb_base_addr(larb);

    // If SMI can't find the corrosponded larb address, skip the backup for the larb
    if(larb_base == SMI_ERROR_ADDR ){
        SMIMSG("Can't find the base address for Larb%d\n", larb);
        return 0;
    }

    M4U_WriteReg32(larb_base, SMI_LARB_CON_SET, F_SMI_LARB_CON_MAU_IRQ_EN(1));
    return 0;
}

int mau_disable_interrupt(int larb)
{

    unsigned int larb_base = get_larb_base_addr(larb);

    // If SMI can't find the corrosponded larb address, skip the backup for the larb
    if(larb_base == SMI_ERROR_ADDR ){
        SMIMSG("Can't find the base address for Larb%d\n", larb);
        return 0;
    }

    M4U_WriteReg32(larb_base, SMI_LARB_CON_CLR, F_SMI_LARB_CON_MAU_IRQ_EN(1));
    return 0;
}


static irqreturn_t mau_isr(int irq, void *dev_id)
{
    int larb,i;
    unsigned int larb_base = SMI_ERROR_ADDR;
    unsigned int regval;

    switch(irq)
    {
        case SMI_LARB0_IRQ_ID:
            larb = 0;
            break;
        case SMI_LARB1_IRQ_ID:
            larb = 1;
            break;
        case SMI_LARB2_IRQ_ID:
            larb = 2;
            break;
 //       case MJC_SMI_LARB_IRQ_ID:
 //           larb = 5;
        default :
            larb=0;
            SMIERR("unkown irq(%d)\n",irq);

    }


    larb_base = get_larb_base_addr(larb);

    // If SMI can't find the corrosponded larb address, skip the backup for the larb
    if(larb_base == SMI_ERROR_ADDR ){
        SMIMSG("Can't find the base address for Larb%d\n", larb);
        return IRQ_HANDLED;
    }

    larb_clock_on(larb);

    //dump interrupt debug infomation
    for(i=0; i<MAU_ENTRY_NR; i++)
    {
        regval = M4U_ReadReg32(larb_base, SMI_MAU_ENTR_STAT(i));
        if(F_MAU_STAT_ASSERT(regval))
        {
            //violation happens in this entry
            int port =  F_MAU_STAT_ID(regval);
            SMIMSG("[MAU] larb=%d, entry=%d, port=%d\n",larb,i,port);
            regval = M4U_ReadReg32(larb_base, SMI_MAU_ENTR_START(i));
            SMIMSG("start_addr=0x%x, read_en=%d, write_en=%d\n", F_MAU_START_ADDR_VAL(regval),
                F_MAU_START_IS_RD(regval), F_MAU_START_IS_WR(regval));
            regval = M4U_ReadReg32(larb_base, SMI_MAU_ENTR_END(i));
            SMIMSG("end_addr=0x%x, virtual=%d\n", F_MAU_END_ADDR_VAL(regval),
                F_MAU_END_IS_VIR(regval));
            //smi_aee_print("violation by %s\n",smi_port_name[larb][port]);
            SMIMSG("violation by %s\n",smi_port_name[larb][port]);
        }

        //clear interrupt status
        regval = M4U_ReadReg32(larb_base, SMI_MAU_ENTR_STAT(i));
        M4U_WriteReg32(larb_base, SMI_MAU_ENTR_STAT(i), regval);
    }

    larb_clock_off(larb);
    return IRQ_HANDLED;
}


int mau_start_monitor(int larb, int entry, int rd, int wr, int vir,
            unsigned int start, unsigned int end, unsigned int port_msk)
{
    unsigned int regval;

    unsigned int larb_base = SMI_ERROR_ADDR;

    larb_base = get_larb_base_addr(larb);

    // The larb assign doesn't exist
    if(larb_base == SMI_ERROR_ADDR ){
        SMIMSG("Can't find the base address for Larb%d\n", larb);
        return 0;
    }

    //mau entry i start address
    regval = F_MAU_START_WR(wr)|F_MAU_START_RD(rd)|F_MAU_START_ADD(start);
    M4U_WriteReg32(larb_base, SMI_MAU_ENTR_START(entry), regval);
    regval = F_MAU_END_VIR(vir)|F_MAU_END_ADD(end);
    M4U_WriteReg32(larb_base, SMI_MAU_ENTR_END(entry), regval);

    //start monitor
    regval = M4U_ReadReg32(larb_base, SMI_MAU_ENTR_GID(entry));
    M4U_WriteReg32(larb_base, SMI_MAU_ENTR_GID(entry), regval|port_msk);
    return 0;
}

int mau_config(MTK_MAU_CONFIG* pMauConf)
{
    if(pMauConf->larb > SMI_LARB_NR ||
        pMauConf->entry > MAU_ENTRY_NR
      )
    {
        SMIERR("config:larb=%d,entry=%d\n", pMauConf->larb, pMauConf->entry);
        SMIMSG("mau config error: larb=%d,entry=%d,rd=%d,wr=%d,vir=%d,start=0x%x,end=0x%x,port_msk=0x%x\n",
            pMauConf->larb, pMauConf->entry, pMauConf->monitor_read, pMauConf->monitor_write,
            pMauConf->virt,pMauConf->start, pMauConf->end, pMauConf->port_msk);
        return -1;
    }
    SMIMSG("mau config: larb=%d,entry=%d,rd=%d,wr=%d,vir=%d,start=0x%x,end=0x%x,port_msk=0x%x\n",
        pMauConf->larb, pMauConf->entry, pMauConf->monitor_read, pMauConf->monitor_write,
        pMauConf->virt,pMauConf->start, pMauConf->end, pMauConf->port_msk);

    larb_clock_on(pMauConf->larb);
    mau_start_monitor(pMauConf->larb,
                      pMauConf->entry,
                      !!(pMauConf->monitor_read),
                      !!(pMauConf->monitor_write),
                      !!(pMauConf->virt),
                      pMauConf->start,
                      pMauConf->end,
                      pMauConf->port_msk);

    larb_clock_off(pMauConf->larb);
    return 0;
}


int mau_dump_status(int larb)
{
    unsigned int larb_base = SMI_ERROR_ADDR;
    unsigned int regval;
    int i;

    larb_base = get_larb_base_addr(larb);

    // The larb assign doesn't exist
    if(larb_base == SMI_ERROR_ADDR ){
        SMIMSG("Can't find the base address for Larb%d\n", larb);
        return 0;
    }

    larb_clock_on(larb);

    //dump interrupt debug infomation
    for(i=0; i<MAU_ENTRY_NR; i++)
    {
        regval = M4U_ReadReg32(larb_base, SMI_MAU_ENTR_GID(i));
        if(regval!=0)
        {
            SMIMSG("larb(%d), entry(%d)=========>\n", larb, i);
            SMIMSG("port mask = 0x%x\n", regval);
            regval = M4U_ReadReg32(larb_base, SMI_MAU_ENTR_START(i));
            SMIMSG("start_addr=0x%x, read_en=%d, write_en=%d\n",
                F_MAU_START_ADDR_VAL(regval),
                F_MAU_START_IS_RD(regval), F_MAU_START_IS_WR(regval));
            regval = M4U_ReadReg32(larb_base, SMI_MAU_ENTR_END(i));
            SMIMSG("end_addr=0x%x, virtual=%d\n", F_MAU_END_ADDR_VAL(regval),
                F_MAU_END_IS_VIR(regval));
        }
        else
        {
            SMIMSG("larb(%d), entry(%d) is free\n", larb, i);
        }


        //dump interrupt debug infomation
        regval = M4U_ReadReg32(larb_base, SMI_MAU_ENTR_STAT(i));
        if(F_MAU_STAT_ASSERT(regval))
        {
            //violation happens in this entry
            int port =  F_MAU_STAT_ID(regval);
            SMIMSG("[MAU] larb=%d, entry=%d, port=%d\n",larb,i,port);
            regval = M4U_ReadReg32(larb_base, SMI_MAU_ENTR_START(i));
            SMIMSG("start_addr=0x%x, read_en=%d, write_en=%d\n", F_MAU_START_ADDR_VAL(regval),
                F_MAU_START_IS_RD(regval), F_MAU_START_IS_WR(regval));
            regval = M4U_ReadReg32(larb_base, SMI_MAU_ENTR_END(i));
            SMIMSG("end_addr=0x%x, virtual=%d\n", F_MAU_END_ADDR_VAL(regval),
                F_MAU_END_IS_VIR(regval));
            SMIMSG("violation by %s\n",smi_port_name[larb][port]);
        }
        else
        {
            SMIMSG("no violation of entry %d\n", i);
        }

        //clear interrupt status
        regval = M4U_ReadReg32(larb_base, SMI_MAU_ENTR_STAT(i));
        M4U_WriteReg32(larb_base, SMI_MAU_ENTR_STAT(i), regval);

    }


    larb_clock_off(larb);

    return 0;
}


int mau_init(void)
{
    int i;

    if(request_irq(SMI_LARB0_IRQ_ID , (irq_handler_t)mau_isr, IRQF_TRIGGER_LOW, "MAU0" , NULL))
    {
        SMIERR("request MAU0 IRQ line failed");
        return -ENODEV;
    }
    if(request_irq(SMI_LARB1_IRQ_ID , (irq_handler_t)mau_isr, IRQF_TRIGGER_LOW, "MAU1" , NULL))
    {
        SMIERR("request MAU1 IRQ line failed");
        return -ENODEV;
    }
    if(request_irq(SMI_LARB2_IRQ_ID , (irq_handler_t)mau_isr, IRQF_TRIGGER_LOW, "MAU2" , NULL))
    {
        SMIERR("request MAU2 IRQ line failed");
        return -ENODEV;
    }
 /* if(request_irq(MJC_SMI_LARB_IRQ_ID , (irq_handler_t)mau_isr, IRQF_TRIGGER_LOW, "MAU5" , NULL))
    {
        SMIERR("request MAU5 IRQ line failed");
        return -ENODEV;
    }
  */
    for(i=0; i<SMI_LARB_NR; i++)
    {
        larb_clock_on(i);
        mau_enable_interrupt(i);
        larb_clock_off(i);
    }

    return 0;
}





