/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*******************************************************************************
 *
 * Filename:
 * ---------
 *   AudDrv_Ana.h
 *
 * Project:
 * --------
 *   MT6397 Audio Driver Digital/Analog
 *
 * Description:
 * ------------
 *   Audio register
 *
 * Author:
 * -------
 *   Luke Liu
 *   Daniel Hsiao (mtk00469)
 *   Chipeng Chang (mtk02308)
 *
 *------------------------------------------------------------------------------
 * $Revision: #1 $
 * $Modtime:$
 * $Log:$
 *
 *
 *******************************************************************************/

#ifndef _AUDDRV_ANA_H_
#define _AUDDRV_ANA_H_

/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include "AudDrv_Common.h"
#include "AudDrv_Def.h"


/*****************************************************************************
 *                         D A T A   T Y P E S
 *****************************************************************************/


/*****************************************************************************
 *                         M A C R O
 *****************************************************************************/

/*****************************************************************************
 *                  R E G I S T E R       D E F I N I T I O N
 *****************************************************************************/

//---------------digital pmic register define start -------------------------------------------
#define AFE_PMICDIG_AUDIO_BASE        (0x4000)
#define AFE_UL_DL_CON0               (AFE_PMICDIG_AUDIO_BASE+0x0000)
#define AFE_DL_SRC2_CON0_H     (AFE_PMICDIG_AUDIO_BASE+0x0002)
#define AFE_DL_SRC2_CON0_L     (AFE_PMICDIG_AUDIO_BASE+0x0004)
#define AFE_DL_SDM_CON0             (AFE_PMICDIG_AUDIO_BASE + 0x0006)
#define AFE_DL_SDM_CON1             (AFE_PMICDIG_AUDIO_BASE + 0x0008)
#define AFE_UL_SRC_CON0_H           (AFE_PMICDIG_AUDIO_BASE + 0x000A)
#define AFE_UL_SRC_CON0_L           (AFE_PMICDIG_AUDIO_BASE + 0x000C)
#define AFE_UL_SRC_CON1_H           (AFE_PMICDIG_AUDIO_BASE + 0x000E)
#define AFE_UL_SRC_CON1_L           (AFE_PMICDIG_AUDIO_BASE + 0x0010)
#define ANA_AFE_TOP_CON0            (AFE_PMICDIG_AUDIO_BASE + 0x0012)
#define ANA_AUDIO_TOP_CON0          (AFE_PMICDIG_AUDIO_BASE + 0x0014)
#define AFE_DL_SRC_MON0             (AFE_PMICDIG_AUDIO_BASE + 0x0016)
#define AFE_DL_SDM_TEST0            (AFE_PMICDIG_AUDIO_BASE + 0x0018)
#define AFE_MON_DEBUG0              (AFE_PMICDIG_AUDIO_BASE + 0x001A)
#define AFUNC_AUD_CON0              (AFE_PMICDIG_AUDIO_BASE + 0x001C)
#define AFUNC_AUD_CON1              (AFE_PMICDIG_AUDIO_BASE + 0x001E)
#define AFUNC_AUD_CON2              (AFE_PMICDIG_AUDIO_BASE + 0x0020)
#define AFUNC_AUD_CON3              (AFE_PMICDIG_AUDIO_BASE + 0x0022)
#define AFUNC_AUD_CON4              (AFE_PMICDIG_AUDIO_BASE + 0x0024)
#define AFUNC_AUD_MON0              (AFE_PMICDIG_AUDIO_BASE + 0x0026)
#define AFUNC_AUD_MON1              (AFE_PMICDIG_AUDIO_BASE + 0x0028)
#define AUDRC_TUNE_MON0             (AFE_PMICDIG_AUDIO_BASE + 0x002A)
#define AFE_UP8X_FIFO_CFG0          (AFE_PMICDIG_AUDIO_BASE + 0x002C) //6397 new
#define AFE_UP8X_FIFO_LOG_MON0      (AFE_PMICDIG_AUDIO_BASE + 0x002E) //6397 new
#define AFE_UP8X_FIFO_LOG_MON1      (AFE_PMICDIG_AUDIO_BASE + 0x0030) //6397 new
#define AFE_DL_DC_COMP_CFG0         (AFE_PMICDIG_AUDIO_BASE + 0x0032)
#define AFE_DL_DC_COMP_CFG1         (AFE_PMICDIG_AUDIO_BASE + 0x0034)
#define AFE_DL_DC_COMP_CFG2         (AFE_PMICDIG_AUDIO_BASE + 0x0036)
#define AFE_PMIC_NEWIF_CFG0         (AFE_PMICDIG_AUDIO_BASE + 0x0038)
#define AFE_PMIC_NEWIF_CFG1         (AFE_PMICDIG_AUDIO_BASE + 0x003A)
#define AFE_PMIC_NEWIF_CFG2         (AFE_PMICDIG_AUDIO_BASE + 0x003C)
#define AFE_PMIC_NEWIF_CFG3         (AFE_PMICDIG_AUDIO_BASE + 0x003E)
#define AFE_SGEN_CFG0               (AFE_PMICDIG_AUDIO_BASE + 0x0040)
#define AFE_SGEN_CFG1               (AFE_PMICDIG_AUDIO_BASE + 0x0042)
//---------------digital pmic  register define end ---------------------------------------

//---------------analog pmic  register define start --------------------------------------
#if 0
#define AFE_PMICANA_AUDIO_BASE        (0x0)

#define TOP_CKPDN                   (AFE_PMICANA_AUDIO_BASE + 0x0102)
#define TOP_CKPDN_SET               (AFE_PMICANA_AUDIO_BASE + 0x0104)
#define TOP_CKPDN_CLR               (AFE_PMICANA_AUDIO_BASE + 0x0106)
#define TOP_CKPDN2                  (AFE_PMICANA_AUDIO_BASE + 0x0108)
#define TOP_CKPDN2_SET              (AFE_PMICANA_AUDIO_BASE + 0x010a)
#define TOP_CKPDN2_CLR              (AFE_PMICANA_AUDIO_BASE + 0x010c)
#define TOP_CKCON1                  (AFE_PMICANA_AUDIO_BASE + 0x0128)

#define SPK_CON0                    (AFE_PMICANA_AUDIO_BASE + 0x0600)
#define SPK_CON1                    (AFE_PMICANA_AUDIO_BASE + 0x0602)
#define SPK_CON2                    (AFE_PMICANA_AUDIO_BASE + 0x0604)
#define SPK_CON3                    (AFE_PMICANA_AUDIO_BASE + 0x0606)
#define SPK_CON4                    (AFE_PMICANA_AUDIO_BASE + 0x0608)
#define SPK_CON5                    (AFE_PMICANA_AUDIO_BASE + 0x060A)
#define SPK_CON6                    (AFE_PMICANA_AUDIO_BASE + 0x060C)
#define SPK_CON7                    (AFE_PMICANA_AUDIO_BASE + 0x060E)
#define SPK_CON8                    (AFE_PMICANA_AUDIO_BASE + 0x0610)
#define SPK_CON9                    (AFE_PMICANA_AUDIO_BASE + 0x0612)
#define SPK_CON10                   (AFE_PMICANA_AUDIO_BASE + 0x0614)
#define SPK_CON11                   (AFE_PMICANA_AUDIO_BASE + 0x0616)

#define AUDDAC_CON0                 (AFE_PMICANA_AUDIO_BASE + 0x0700)
#define AUDBUF_CFG0                 (AFE_PMICANA_AUDIO_BASE + 0x0702)
#define AUDBUF_CFG1                 (AFE_PMICANA_AUDIO_BASE + 0x0704)
#define AUDBUF_CFG2                 (AFE_PMICANA_AUDIO_BASE + 0x0706)
#define AUDBUF_CFG3                 (AFE_PMICANA_AUDIO_BASE + 0x0708)
#define AUDBUF_CFG4                 (AFE_PMICANA_AUDIO_BASE + 0x070a)
#define IBIASDIST_CFG0              (AFE_PMICANA_AUDIO_BASE + 0x070c)
#define AUDACCDEPOP_CFG0            (AFE_PMICANA_AUDIO_BASE + 0x070e)
#define AUD_IV_CFG0                 (AFE_PMICANA_AUDIO_BASE + 0x0710)
#define AUDCLKGEN_CFG0              (AFE_PMICANA_AUDIO_BASE + 0x0712)
#define AUDLDO_CFG0                 (AFE_PMICANA_AUDIO_BASE + 0x0714)
#define AUDLDO_CFG1                 (AFE_PMICANA_AUDIO_BASE + 0x0716)
#define AUDNVREGGLB_CFG0            (AFE_PMICANA_AUDIO_BASE + 0x0718)
#define AUD_NCP0                    (AFE_PMICANA_AUDIO_BASE + 0x071a)
#define AUDPREAMP_CON0              (AFE_PMICANA_AUDIO_BASE + 0x071c)
#define AUDADC_CON0                 (AFE_PMICANA_AUDIO_BASE + 0x071e)
#define AUDADC_CON1                 (AFE_PMICANA_AUDIO_BASE + 0x0720)
#define AUDADC_CON2                 (AFE_PMICANA_AUDIO_BASE + 0x0722)
#define AUDADC_CON3                 (AFE_PMICANA_AUDIO_BASE + 0x0724)
#define AUDADC_CON4                 (AFE_PMICANA_AUDIO_BASE + 0x0726)
#define AUDADC_CON5                 (AFE_PMICANA_AUDIO_BASE + 0x0728)
#define AUDADC_CON6                 (AFE_PMICANA_AUDIO_BASE + 0x072a)
#define AUDDIGMI_CON0               (AFE_PMICANA_AUDIO_BASE + 0x072c)
#define AUDLSBUF_CON0               (AFE_PMICANA_AUDIO_BASE + 0x072e)
#define AUDLSBUF_CON1               (AFE_PMICANA_AUDIO_BASE + 0x0730)
#define AUDENCSPARE_CON0            (AFE_PMICANA_AUDIO_BASE + 0x0732)
#define AUDENCCLKSQ_CON0            (AFE_PMICANA_AUDIO_BASE + 0x0734)
#define AUDPREAMPGAIN_CON0          (AFE_PMICANA_AUDIO_BASE + 0x0736)
#define ZCD_CON0                    (AFE_PMICANA_AUDIO_BASE + 0x0738)
#define ZCD_CON1                    (AFE_PMICANA_AUDIO_BASE + 0x073a)
#define ZCD_CON2                    (AFE_PMICANA_AUDIO_BASE + 0x073c)
#define ZCD_CON3                    (AFE_PMICANA_AUDIO_BASE + 0x073e)
#define ZCD_CON4                    (AFE_PMICANA_AUDIO_BASE + 0x0740)
#define ZCD_CON5                    (AFE_PMICANA_AUDIO_BASE + 0x0742)
#define NCP_CLKDIV_CON0             (AFE_PMICANA_AUDIO_BASE + 0x0744)
#define NCP_CLKDIV_CON1             (AFE_PMICANA_AUDIO_BASE + 0x0746)
#else
#include <mach/upmu_hw.h>
#endif

//---------------analog pmic register define end ---------------------------------------

typedef struct
{
volatile uint16 Suspend_Ana_AFE_UL_DL_CON0;
volatile uint16 Suspend_Ana_AFE_DL_SRC2_CON0_H;
volatile uint16 Suspend_Ana_AFE_DL_SRC2_CON0_L;
volatile uint16 Suspend_Ana_AFE_DL_SDM_CON0;
volatile uint16 Suspend_Ana_AFE_DL_SDM_CON1;
volatile uint16 Suspend_Ana_AFE_UL_SRC_CON0_H;
volatile uint16 Suspend_Ana_AFE_UL_SRC_CON0_L;
volatile uint16 Suspend_Ana_AFE_UL_SRC_CON1_H;
volatile uint16 Suspend_Ana_AFE_UL_SRC_CON1_L;
volatile uint16 Suspend_Ana_AFE_TOP_CON0;
volatile uint16 Suspend_Ana_AUDIO_TOP_CON0;                                                 
volatile uint16 Suspend_Ana_AFUNC_AUD_CON0;
volatile uint16 Suspend_Ana_AFUNC_AUD_CON1;
volatile uint16 Suspend_Ana_AFUNC_AUD_CON2;
volatile uint16 Suspend_Ana_AFUNC_AUD_CON3;
volatile uint16 Suspend_Ana_AFUNC_AUD_CON4;
volatile uint16 Suspend_Ana_AFE_UP8X_FIFO_CFG0;
volatile uint16 Suspend_Ana_AFE_DL_DC_COMP_CFG0;
volatile uint16 Suspend_Ana_AFE_DL_DC_COMP_CFG1;
volatile uint16 Suspend_Ana_AFE_DL_DC_COMP_CFG2;
volatile uint16 Suspend_Ana_AFE_PMIC_NEWIF_CFG0;
volatile uint16 Suspend_Ana_AFE_PMIC_NEWIF_CFG1;
volatile uint16 Suspend_Ana_AFE_PMIC_NEWIF_CFG2;
volatile uint16 Suspend_Ana_AFE_PMIC_NEWIF_CFG3;
volatile uint16 Suspend_Ana_AFE_SGEN_CFG0;
volatile uint16 Suspend_Ana_AFE_SGEN_CFG1;

volatile uint16 Suspend_Ana_TOP_CKPDN;
volatile uint16 Suspend_Ana_TOP_CKPDN_SET;
volatile uint16 Suspend_Ana_TOP_CKPDN_CLR;
volatile uint16 Suspend_Ana_TOP_CKPDN2;
volatile uint16 Suspend_Ana_TOP_CKPDN2_SET;
volatile uint16 Suspend_Ana_TOP_CKPDN2_CLR;
volatile uint16 Suspend_Ana_TOP_CKCON1;
    volatile uint16 Suspend_Ana_SPK_CON0;
    volatile uint16 Suspend_Ana_SPK_CON1;
    volatile uint16 Suspend_Ana_SPK_CON2;
volatile uint16 Suspend_Ana_SPK_CON3;
volatile uint16 Suspend_Ana_SPK_CON4;
volatile uint16 Suspend_Ana_SPK_CON5;
    volatile uint16 Suspend_Ana_SPK_CON6;
    volatile uint16 Suspend_Ana_SPK_CON7;
    volatile uint16 Suspend_Ana_SPK_CON8;
    volatile uint16 Suspend_Ana_SPK_CON9;
    volatile uint16 Suspend_Ana_SPK_CON10;
    volatile uint16 Suspend_Ana_SPK_CON11;
volatile uint16 Suspend_Ana_AUDDAC_CON0;
volatile uint16 Suspend_Ana_AUDBUF_CFG0;
volatile uint16 Suspend_Ana_AUDBUF_CFG1;
volatile uint16 Suspend_Ana_AUDBUF_CFG2;
volatile uint16 Suspend_Ana_AUDBUF_CFG3;
volatile uint16 Suspend_Ana_AUDBUF_CFG4;
volatile uint16 Suspend_Ana_IBIASDIST_CFG0;
volatile uint16 Suspend_Ana_AUDACCDEPOP_CFG0;
volatile uint16 Suspend_Ana_AUD_IV_CFG0;
volatile uint16 Suspend_Ana_AUDCLKGEN_CFG0;
volatile uint16 Suspend_Ana_AUDLDO_CFG0;
volatile uint16 Suspend_Ana_AUDLDO_CFG1;
volatile uint16 Suspend_Ana_AUDNVREGGLB_CFG0;
volatile uint16 Suspend_Ana_AUD_NCP0;
volatile uint16 Suspend_Ana_AUDPREAMP_CON0;
volatile uint16 Suspend_Ana_AUDADC_CON0;
volatile uint16 Suspend_Ana_AUDADC_CON1;
volatile uint16 Suspend_Ana_AUDADC_CON2;
volatile uint16 Suspend_Ana_AUDADC_CON3;
volatile uint16 Suspend_Ana_AUDADC_CON4;
volatile uint16 Suspend_Ana_AUDADC_CON5;
volatile uint16 Suspend_Ana_AUDADC_CON6;
volatile uint16 Suspend_Ana_AUDDIGMI_CON0;
volatile uint16 Suspend_Ana_AUDLSBUF_CON0;
volatile uint16 Suspend_Ana_AUDLSBUF_CON1;
volatile uint16 Suspend_Ana_AUDENCSPARE_CON0;
volatile uint16 Suspend_Ana_AUDENCCLKSQ_CON0;
volatile uint16 Suspend_Ana_AUDPREAMPGAIN_CON0;
volatile uint16 Suspend_Ana_ZCD_CON0;
volatile uint16 Suspend_Ana_ZCD_CON1;
volatile uint16 Suspend_Ana_ZCD_CON2;
volatile uint16 Suspend_Ana_ZCD_CON3;
volatile uint16 Suspend_Ana_ZCD_CON4;
volatile uint16 Suspend_Ana_ZCD_CON5;
volatile uint16 Suspend_Ana_NCP_CLKDIV_CON0;
volatile uint16 Suspend_Ana_NCP_CLKDIV_CON1;

} AudAna_Suspend_Reg;

void Ana_Set_Reg(uint32 offset, uint32 value, uint32 mask);
uint32  Ana_Get_Reg(uint32 offset);
void AudDrv_Store_reg_ANA(AudAna_Suspend_Reg *pBackup_reg);
void AudDrv_Recover_reg_ANA(AudAna_Suspend_Reg *pBackup_reg);



// for debug usage
void Ana_Log_Print(void);

#endif

