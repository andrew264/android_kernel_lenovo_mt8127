/*
** $Id: stats.c#1 $
*/

/*! \file stats.c
    \brief This file includes statistics support.
*/


/*
** $Log: stats.c $
 *
 * 07 17 2014 samp.lin
 * NULL
 * Initial version.
 */

/*******************************************************************************
 *						C O M P I L E R	 F L A G S
 ********************************************************************************
 */
 
/*******************************************************************************
 *						E X T E R N A L	R E F E R E N C E S
 ********************************************************************************
 */
#include "precomp.h"

#if (CFG_SUPPORT_STATISTICS == 1)

/*******************************************************************************
*						C O N S T A N T S
********************************************************************************
*/

/*******************************************************************************
*						F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/
static void
statsInfoEnvDisplay(
	GLUE_INFO_T							*prGlueInfo,
	UINT8								*prInBuf,
	UINT32 								u4InBufLen
	);

static WLAN_STATUS
statsInfoEnvRequest(
	ADAPTER_T 	  						*prAdapter,
	VOID			  					*pvSetBuffer,
	UINT_32 		  					u4SetBufferLen,
	UINT_32		  						*pu4SetInfoLen
	);

/*******************************************************************************
*						P U B L I C   D A T A
********************************************************************************
*/
UINT_64 u8DrvOwnStart, u8DrvOwnEnd;
UINT32 u4DrvOwnMax = 0;

/*******************************************************************************
*						P R I V A T E  F U N C T I O N S
********************************************************************************
*/

/*----------------------------------------------------------------------------*/
/*! \brief  This routine is called to display all environment log.
*
* \param[in] prGlueInfo		Pointer to the Adapter structure
* \param[in] prInBuf		A pointer to the command string buffer, from u4EventSubId
* \param[in] u4InBufLen	The length of the buffer
* \param[out] None
*
* \retval None
*
*/
/*----------------------------------------------------------------------------*/
static void
statsInfoEnvDisplay(
	GLUE_INFO_T							*prGlueInfo,
	UINT8								*prInBuf,
	UINT32 								u4InBufLen
	)
{
	P_ADAPTER_T prAdapter;
	STA_RECORD_T *prStaRec;
	UINT32 u4NumOfInfo, u4InfoId;
	UINT32 u4RxErrBitmap;
	STATS_INFO_ENV_T rStatsInfoEnv, *prInfo;

/*
[wlan] statsInfoEnvRequest: (INIT INFO) statsInfoEnvRequest cmd ok.
[wlan] statsEventHandle: (INIT INFO) <stats> statsEventHandle: Rcv a event
[wlan] statsEventHandle: (INIT INFO) <stats> statsEventHandle: Rcv a event: 0
[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> Display stats for [00:0c:43:31:35:97]:

[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> TPAM(0x0) RTS(0 0) BA(0x1 0) OK(9 9) ERR(0 0 0 0 0 0 0)
	TPAM (bit0: enable 40M, bit1: enable 20 short GI, bit2: enable 40 short GI, bit3: use 40M TX, bit4: use short GI TX, bit5: use no ack)
	RTS (1st: current use RTS/CTS, 2nd: ever use RTS/CTS)
	BA (1st: TX session BA bitmap for TID0 ~ TID7, 2nd: peer receive maximum agg number)
	OK (1st: total number of tx packet from host, 2nd: total number of tx ok)
	ERR (1st: total number of tx err, 2nd ~ 7st: total number of
		WLAN_STATUS_BUFFER_RETAINED, WLAN_STATUS_PACKET_FLUSHED, WLAN_STATUS_PACKET_AGING_TIMEOUT,
		WLAN_STATUS_PACKET_MPDU_ERROR, WLAN_STATUS_PACKET_RTS_ERROR, WLAN_STATUS_PACKET_LIFETIME_ERROR)

[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> TRATE (6 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0) (0 0 0 0 0 0 0 3)
	TX rate count (1M 2 5.5 11 NA NA NA NA 48 24 12 6 54 36 18 9) (MCS0 ~ MCS7)

[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> RX(148 1 0) BA(0x1 64) OK(2 2) ERR(0)
	RX (1st: latest RCPI, 2nd: chan num)
	BA (1st: RX session BA bitmap for TID0 ~ TID7, 2nd: our receive maximum agg number)
	OK (number of rx packets without error, number of rx packets to OS)
	ERR (number of rx packets with error)

[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> RCCK (0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0)
	CCK MODE (1 2 5.5 11M)
[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> ROFDM (0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0)
	OFDM MODE (NA NA NA NA 6 9 12 18 24 36 48 54M)
[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> RHT (0 0 0 0 0 0 0 2 0 0 0 0 0 0 0 0)
	MIXED MODE (number of rx packets with MCS0 ~ MCS15)

[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> StayIntH2M us (29 29 32) (0 0 0) (0 0 0)
	delay from HIF to MAC own bit=1 (min, avg, max for 500B) (min, avg, max for 1000B) (min, avg, max for others)

[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> AirTime us (608 864 4480) (0 0 0) (0 0 0)
	delay from MAC start TX to MAC TX done

[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> StayInt us (795 1052 4644_4504) (0 0 0_0) (0 0 0_0)
	delay from HIF to MAC TX done (min, avg, max_system time for 500B)

[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> StayIntD2T us (795 1052 4644) (0 0 0) (0 0 0)
	delay from driver to MAC TX done (min, avg, max for 500B)

[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> StayIntR_M2H us (37 40 58) (0 0 0) (0 0 0)
	delay from MAC to HIF (min, avg, max for 500B)

[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> StayIntR_H2D us (0 0 0) (0 0 0) (0 0 0)
	delay from HIF to Driver OS (min, avg, max for 500B)

[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> StayCntD2H unit:10ms (10 0 0 0)
	delay count from Driver to HIF (count in 0~10ms, 10~20ms, 20~30ms, others)

[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> StayCnt unit:1ms (6 3 0 1)
	delay count from HIF to TX DONE (count in 0~1ms, 1~5ms, 5~10ms, others)

[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> StayCnt (0~1161:7) (1161~2322:2) (2322~3483:0) (3483~4644:0) (4644~:1)
	delay count from HIF to TX DONE (count in 0~1161 ticks, 1161~2322, 2322~3483, 3483~4644, others)

[wlan] statsInfoEnvDisplay: (INIT INFO) <stats> OTHER (61877) (0) (38) (0) (0) (0ms)
	Channel idle time, scan count, channel change count, empty tx quota count,
	power save change count from active to PS, maximum delay from PS to active
*/

	/* init */
	prAdapter = prGlueInfo->prAdapter;
	prInfo = &rStatsInfoEnv;

	/* parse */
	u4NumOfInfo = *(UINT32 *)prInBuf;
	u4RxErrBitmap = *(UINT32 *)(prInBuf+4);

	/* print */
	for(u4InfoId=0; u4InfoId<u4NumOfInfo; u4InfoId++)
	{
		kalMemCopy(&rStatsInfoEnv, prInBuf+8, sizeof(STATS_INFO_ENV_T));

		prStaRec = cnmGetStaRecByIndex(prAdapter, rStatsInfoEnv.ucStaRecIdx);
		if (prStaRec == NULL)
			continue;

		DBGLOG(INIT, INFO, ("<stats> Display stats for ["MACSTR"]: %uB %ums\n",
			MAC2STR(prStaRec->aucMacAddr), (UINT32)sizeof(STATS_INFO_ENV_T),
			prInfo->u4ReportSysTime));
		DBGLOG(INIT, INFO, ("<stats> TPAM(0x%x) RTS(%u %u) BA(0x%x %u) "
			"OS(%u) OK(%u %u) ERR(%u %u %u %u %u %u %u)\n",
			prInfo->ucTxParam,
			prInfo->fgTxIsRtsUsed, prInfo->fgTxIsRtsEverUsed,
			prInfo->ucTxAggBitmap, prInfo->ucTxPeerAggMaxSize,
			(UINT32)prGlueInfo->rNetDevStats.tx_packets,
			prInfo->u4TxDataCntAll, prInfo->u4TxDataCntOK,
			prInfo->u4TxDataCntErr,	prInfo->u4TxDataCntErrType[0],
			prInfo->u4TxDataCntErrType[1], prInfo->u4TxDataCntErrType[2],
			prInfo->u4TxDataCntErrType[3], prInfo->u4TxDataCntErrType[4],
			prInfo->u4TxDataCntErrType[5]));

		DBGLOG(INIT, INFO, ("<stats> TRATE "
			"(%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u) "
			"(%u %u %u %u %u %u %u %u)\n",
			prInfo->u4TxRateCntNonHT[0], prInfo->u4TxRateCntNonHT[1],
			prInfo->u4TxRateCntNonHT[2], prInfo->u4TxRateCntNonHT[3],
			prInfo->u4TxRateCntNonHT[4], prInfo->u4TxRateCntNonHT[5],
			prInfo->u4TxRateCntNonHT[6], prInfo->u4TxRateCntNonHT[7],
			prInfo->u4TxRateCntNonHT[8], prInfo->u4TxRateCntNonHT[9],
			prInfo->u4TxRateCntNonHT[10], prInfo->u4TxRateCntNonHT[11],
			prInfo->u4TxRateCntNonHT[12], prInfo->u4TxRateCntNonHT[13],
			prInfo->u4TxRateCntNonHT[14], prInfo->u4TxRateCntNonHT[15],
			prInfo->u4TxRateCntHT[0], prInfo->u4TxRateCntHT[1],
			prInfo->u4TxRateCntHT[2], prInfo->u4TxRateCntHT[3],
			prInfo->u4TxRateCntHT[4], prInfo->u4TxRateCntHT[5],
			prInfo->u4TxRateCntHT[6], prInfo->u4TxRateCntHT[7]));

		DBGLOG(INIT, INFO, ("<stats> TREORDER (%u %u %u)\n",
			prStaRec->u4RxReorderFallAheadCnt,
			prStaRec->u4RxReorderFallBehindCnt,
			prStaRec->u4RxReorderHoleCnt));

		DBGLOG(INIT, INFO, ("<stats> RX(%u %u %u) BA(0x%x %u) OK(%u %u) ERR(%u)\n",
			prInfo->ucRcvRcpi, prInfo->ucHwChanNum, prInfo->fgRxIsShortGI,
			prInfo->ucRxAggBitmap, prInfo->ucRxAggMaxSize,
			prInfo->u4RxDataCntAll, prStaRec->u4StatsRxPassToOsCnt,
			prInfo->u4RxDataCntErr));

		DBGLOG(INIT, INFO, ("<stats> RCCK "
			"(%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u)\n",
			prInfo->u4RxRateCnt[0][0], prInfo->u4RxRateCnt[0][1],
			prInfo->u4RxRateCnt[0][2], prInfo->u4RxRateCnt[0][3],
			prInfo->u4RxRateCnt[0][4], prInfo->u4RxRateCnt[0][5],
			prInfo->u4RxRateCnt[0][6], prInfo->u4RxRateCnt[0][7],
			prInfo->u4RxRateCnt[0][8], prInfo->u4RxRateCnt[0][9],
			prInfo->u4RxRateCnt[0][10], prInfo->u4RxRateCnt[0][11],
			prInfo->u4RxRateCnt[0][12], prInfo->u4RxRateCnt[0][13],
			prInfo->u4RxRateCnt[0][14], prInfo->u4RxRateCnt[0][15]));
		DBGLOG(INIT, INFO, ("<stats> ROFDM "
			"(%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u)\n",
			prInfo->u4RxRateCnt[1][0], prInfo->u4RxRateCnt[1][1],
			prInfo->u4RxRateCnt[1][2], prInfo->u4RxRateCnt[1][3],
			prInfo->u4RxRateCnt[1][4], prInfo->u4RxRateCnt[1][5],
			prInfo->u4RxRateCnt[1][6], prInfo->u4RxRateCnt[1][7],
			prInfo->u4RxRateCnt[1][8], prInfo->u4RxRateCnt[1][9],
			prInfo->u4RxRateCnt[1][10], prInfo->u4RxRateCnt[1][11],
			prInfo->u4RxRateCnt[1][12], prInfo->u4RxRateCnt[1][13],
			prInfo->u4RxRateCnt[1][14], prInfo->u4RxRateCnt[1][15]));
		DBGLOG(INIT, INFO, ("<stats> RHT "
			"(%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u)\n",
			prInfo->u4RxRateCnt[2][0], prInfo->u4RxRateCnt[2][1],
			prInfo->u4RxRateCnt[2][2], prInfo->u4RxRateCnt[2][3],
			prInfo->u4RxRateCnt[2][4], prInfo->u4RxRateCnt[2][5],
			prInfo->u4RxRateCnt[2][6], prInfo->u4RxRateCnt[2][7],
			prInfo->u4RxRateCnt[2][8], prInfo->u4RxRateCnt[2][9],
			prInfo->u4RxRateCnt[2][10], prInfo->u4RxRateCnt[2][11],
			prInfo->u4RxRateCnt[2][12], prInfo->u4RxRateCnt[2][13],
			prInfo->u4RxRateCnt[2][14], prInfo->u4RxRateCnt[2][15]));

		/* delay from HIF to MAC */
		DBGLOG(INIT, INFO, ("<stats> StayIntH2M us (%u %u %u) (%u %u %u) (%u %u %u)\n",
			prInfo->u4StayIntMinH2M[0], prInfo->u4StayIntAvgH2M[0],
			prInfo->u4StayIntMaxH2M[0],
			prInfo->u4StayIntMinH2M[1], prInfo->u4StayIntAvgH2M[1],
			prInfo->u4StayIntMaxH2M[1],
			prInfo->u4StayIntMinH2M[2], prInfo->u4StayIntAvgH2M[2],
			prInfo->u4StayIntMaxH2M[2]));
		/* delay from MAC to TXDONE */
		DBGLOG(INIT, INFO, ("<stats> AirTime us (%u %u %u) (%u %u %u) (%u %u %u)\n",
			prInfo->u4AirDelayMin[0]<<5, prInfo->u4AirDelayAvg[0]<<5,
			prInfo->u4AirDelayMax[0]<<5,
			prInfo->u4AirDelayMin[1]<<5, prInfo->u4AirDelayAvg[1]<<5,
			prInfo->u4AirDelayMax[1]<<5,
			prInfo->u4AirDelayMin[2]<<5, prInfo->u4AirDelayAvg[2]<<5,
			prInfo->u4AirDelayMax[2]<<5));
		/* delay from HIF to TXDONE */
		DBGLOG(INIT, INFO, ("<stats> StayInt us (%u %u %u_%u) (%u %u %u_%u) (%u %u %u_%u)\n",
			prInfo->u4StayIntMin[0], prInfo->u4StayIntAvg[0],
			prInfo->u4StayIntMax[0], prInfo->u4StayIntMaxSysTime[0],
			prInfo->u4StayIntMin[1], prInfo->u4StayIntAvg[1],
			prInfo->u4StayIntMax[1], prInfo->u4StayIntMaxSysTime[1],
			prInfo->u4StayIntMin[2], prInfo->u4StayIntAvg[2],
			prInfo->u4StayIntMax[2], prInfo->u4StayIntMaxSysTime[2]));
		/* delay from Driver to TXDONE */
		DBGLOG(INIT, INFO, ("<stats> StayIntD2T us (%u %u %u) (%u %u %u) (%u %u %u)\n",
			prInfo->u4StayIntMinD2T[0], prInfo->u4StayIntAvgD2T[0],
			prInfo->u4StayIntMaxD2T[0],
			prInfo->u4StayIntMinD2T[1], prInfo->u4StayIntAvgD2T[1],
			prInfo->u4StayIntMaxD2T[1],
			prInfo->u4StayIntMinD2T[2], prInfo->u4StayIntAvgD2T[2],
			prInfo->u4StayIntMaxD2T[2]));

		/* delay from RXDONE to HIF */
		DBGLOG(INIT, INFO, ("<stats> StayIntR_M2H us (%u %u %u) (%u %u %u) (%u %u %u)\n",
			prInfo->u4StayIntMinRx[0], prInfo->u4StayIntAvgRx[0],
			prInfo->u4StayIntMaxRx[0],
			prInfo->u4StayIntMinRx[1], prInfo->u4StayIntAvgRx[1],
			prInfo->u4StayIntMaxRx[1],
			prInfo->u4StayIntMinRx[2], prInfo->u4StayIntAvgRx[2],
			prInfo->u4StayIntMaxRx[2]));
		/* delay from HIF to OS */
		DBGLOG(INIT, INFO, ("<stats> StayIntR_H2D us (%u %u %u) (%u %u %u) (%u %u %u)\n",
			prStaRec->u4StayIntMinRx[0], prStaRec->u4StayIntAvgRx[0],
			prStaRec->u4StayIntMaxRx[0],
			prStaRec->u4StayIntMinRx[1], prStaRec->u4StayIntAvgRx[1],
			prStaRec->u4StayIntMaxRx[1],
			prStaRec->u4StayIntMinRx[2], prStaRec->u4StayIntAvgRx[2],
			prStaRec->u4StayIntMaxRx[2]));

		/* count based on different delay from HIF to TX DONE */
		DBGLOG(INIT, INFO, ("<stats> StayCntD2H unit:%dms (%d %d %d %d)\n",
			STATS_STAY_INT_D2H_CONST,
			prInfo->u4StayIntD2HByConst[0], prInfo->u4StayIntD2HByConst[1],
			prInfo->u4StayIntD2HByConst[2], prInfo->u4StayIntD2HByConst[3]));
		DBGLOG(INIT, INFO, ("<stats> StayCnt unit:%dms (%d %d %d %d)\n",
			STATS_STAY_INT_CONST,
			prInfo->u4StayIntByConst[0], prInfo->u4StayIntByConst[1],
			prInfo->u4StayIntByConst[2], prInfo->u4StayIntByConst[3]));
		DBGLOG(INIT, INFO, ("<stats> StayCnt (%d~%d:%d) (%d~%d:%d) "
			"(%d~%d:%d) (%d~%d:%d) (%d~:%d)\n",
			0, prInfo->u4StayIntMaxPast/4, prInfo->u4StayIntCnt[0],
			prInfo->u4StayIntMaxPast/4, prInfo->u4StayIntMaxPast/2, prInfo->u4StayIntCnt[1],
			prInfo->u4StayIntMaxPast/2, prInfo->u4StayIntMaxPast*3/4, prInfo->u4StayIntCnt[2],
			prInfo->u4StayIntMaxPast*3/4, prInfo->u4StayIntMaxPast, prInfo->u4StayIntCnt[3],
			prInfo->u4StayIntMaxPast, prInfo->u4StayIntCnt[4]));

		/* others */
		DBGLOG(INIT, INFO, ("<stats> OTHER (%u) (%u) (%u) (%u) (%u) (%ums) (%uus)\n",
			prInfo->u4ChanIdleCnt, prAdapter->ucScanTime,
			prInfo->u4NumOfChanChange, prStaRec->u4NumOfNoTxQuota,
			prInfo->ucNumOfPsChange, prInfo->u4PsIntMax,
			u4DrvOwnMax/1000));

		/* reset */
		kalMemZero(prStaRec->u4StayIntMinRx, sizeof(prStaRec->u4StayIntMinRx));
		kalMemZero(prStaRec->u4StayIntAvgRx, sizeof(prStaRec->u4StayIntAvgRx));
		kalMemZero(prStaRec->u4StayIntMaxRx, sizeof(prStaRec->u4StayIntMaxRx));
		prStaRec->u4StatsRxPassToOsCnt = 0;
		prStaRec->u4RxReorderFallAheadCnt = 0;
		prStaRec->u4RxReorderFallBehindCnt = 0;
		prStaRec->u4RxReorderHoleCnt = 0;
	}

	STATS_DRIVER_OWN_RESET();
}


/*----------------------------------------------------------------------------*/
/*! \brief  This routine is called to request firmware to feedback statistics.
*
* \param[in] prAdapter			Pointer to the Adapter structure
* \param[in] pvSetBuffer		A pointer to the buffer that holds the data to be set
* \param[in] u4SetBufferLen		The length of the set buffer
* \param[out] pu4SetInfoLen	If the call is successful, returns the number of
*	bytes read from the set buffer. If the call failed due to invalid length of
*	the set buffer, returns the amount of storage needed.
*
* \retval TDLS_STATUS_xx
*
*/
/*----------------------------------------------------------------------------*/
static WLAN_STATUS
statsInfoEnvRequest(
	ADAPTER_T 	  						*prAdapter,
	VOID			  					*pvSetBuffer,
	UINT_32 		  					u4SetBufferLen,
	UINT_32		  						*pu4SetInfoLen
	)
{
	STATS_CMD_CORE_T *prCmdContent;
	WLAN_STATUS rStatus;


	/* init command buffer */
	prCmdContent = (STATS_CMD_CORE_T *)pvSetBuffer;
	prCmdContent->u4Command = STATS_CORE_CMD_ENV_REQUEST;

	/* send the command */
	rStatus = wlanSendSetQueryCmd (
		prAdapter,					/* prAdapter */
		CMD_ID_STATS,				/* ucCID */
		TRUE,						/* fgSetQuery */
		FALSE, 			   			/* fgNeedResp */
		FALSE,						/* fgIsOid */
		NULL,
		NULL,						/* pfCmdTimeoutHandler */
		sizeof(STATS_CMD_CORE_T),	/* u4SetQueryInfoLen */
		(PUINT_8) prCmdContent, 	/* pucInfoBuffer */
		NULL,						/* pvSetQueryBuffer */
		0							/* u4SetQueryBufferLen */
		);

	if (rStatus != WLAN_STATUS_PENDING)
	{
		DBGLOG(INIT, ERROR, ("%s wlanSendSetQueryCmd allocation fail!\n",
			__FUNCTION__));
		return WLAN_STATUS_RESOURCES;
	}

	DBGLOG(INIT, INFO, ("%s cmd ok.\n", __FUNCTION__));
	return WLAN_STATUS_SUCCESS;
}


/*******************************************************************************
*						P U B L I C  F U N C T I O N S
********************************************************************************
*/

/*----------------------------------------------------------------------------*/
/*! \brief  This routine is called to handle any statistics event.
*
* \param[in] prGlueInfo		Pointer to the Adapter structure
* \param[in] prInBuf		A pointer to the command string buffer
* \param[in] u4InBufLen	The length of the buffer
* \param[out] None
*
* \retval None
*/
/*----------------------------------------------------------------------------*/
VOID
statsEventHandle(
	GLUE_INFO_T							*prGlueInfo,
	UINT8								*prInBuf,
	UINT32 								u4InBufLen
	)
{
	UINT32 u4EventId;


	/* sanity check */
//	DBGLOG(INIT, INFO,
//		("<stats> %s: Rcv a event\n", __FUNCTION__));

	if ((prGlueInfo == NULL) || (prInBuf == NULL))
		return; /* shall not be here */

	/* handle */
	u4EventId = *(UINT32 *)prInBuf;
	u4InBufLen -= 4;

//	DBGLOG(INIT, INFO,
//		("<stats> %s: Rcv a event: %d\n", __FUNCTION__, u4EventId));

	switch(u4EventId)
	{
		case STATS_HOST_EVENT_ENV_REPORT:
			statsInfoEnvDisplay(prGlueInfo, prInBuf+4, u4InBufLen);
			break;

		default:
			break;
	}
}


/*----------------------------------------------------------------------------*/
/*! \brief  This routine is called to detect if we can request firmware to feedback statistics.
*
* \param[in] prGlueInfo		Pointer to the Adapter structure
* \param[in] ucStaRecIndex	The station index
* \param[out] None
*
* \retval None
*/
/*----------------------------------------------------------------------------*/
VOID
statsEnvReportDetect(
	ADAPTER_T							*prAdapter,
	UINT8								ucStaRecIndex
	)
{
	STA_RECORD_T *prStaRec;
	OS_SYSTIME rCurTime;
	STATS_CMD_CORE_T rCmd;


	prStaRec = cnmGetStaRecByIndex(prAdapter, ucStaRecIndex);
	if (prStaRec == NULL)
		return;

	prStaRec->u4StatsEnvTxCnt ++;
	GET_CURRENT_SYSTIME(&rCurTime);

	if (prStaRec->rStatsEnvTxPeriodLastTime == 0)
	{
		prStaRec->rStatsEnvTxLastTime = rCurTime;
		prStaRec->rStatsEnvTxPeriodLastTime = rCurTime;
		return;
	}

	if (prStaRec->u4StatsEnvTxCnt > STATS_ENV_TX_CNT_REPORT_TRIGGER)
	{
		if (CHECK_FOR_TIMEOUT(rCurTime, prStaRec->rStatsEnvTxLastTime,
			SEC_TO_SYSTIME(STATS_ENV_TX_CNT_REPORT_TRIGGER_SEC)))
		{
			rCmd.ucStaRecIdx = ucStaRecIndex;
			statsInfoEnvRequest(prAdapter, &rCmd, 0, NULL);

			prStaRec->rStatsEnvTxLastTime = rCurTime;
			prStaRec->rStatsEnvTxPeriodLastTime = rCurTime;
			prStaRec->u4StatsEnvTxCnt = 0;
			return;
		}
	}

	if (CHECK_FOR_TIMEOUT(rCurTime, prStaRec->rStatsEnvTxPeriodLastTime,
		SEC_TO_SYSTIME(STATS_ENV_TIMEOUT_SEC)))
	{
		rCmd.ucStaRecIdx = ucStaRecIndex;
		statsInfoEnvRequest(prAdapter, &rCmd, 0, NULL);

		prStaRec->rStatsEnvTxPeriodLastTime = rCurTime;
		return;
	}
}


/*----------------------------------------------------------------------------*/
/*! \brief  This routine is called to handle rx done.
*
* \param[in] prStaRec		Pointer to the STA_RECORD_T structure
* \param[in] prSwRfb		Pointer to the received packet
* \param[out] None
*
* \retval None
*/
/*----------------------------------------------------------------------------*/
VOID
StatsEnvRxDone(
	STA_RECORD_T						*prStaRec,
	SW_RFB_T							*prSwRfb
	)
{
	UINT32 u4LenId;
	UINT32 u4CurTime, u4DifTime;


	/* sanity check */
	if (prStaRec == NULL)
		return;

	/* stats: rx done count */
	prStaRec->u4StatsRxPassToOsCnt ++;

	/* get length partition ID */
	u4LenId = 0;
	if ((0 <= prSwRfb->u2PacketLen) &&
		(prSwRfb->u2PacketLen < STATS_STAY_INT_BYTE_THRESHOLD))
	{
		u4LenId = 0;
	}
	else
	{
		if ((STATS_STAY_INT_BYTE_THRESHOLD <= prSwRfb->u2PacketLen) &&
			(prSwRfb->u2PacketLen < (STATS_STAY_INT_BYTE_THRESHOLD<<1)))
		{
			u4LenId = 1;
		}
		else
			u4LenId = 2;
	}

	/* stats: rx delay */
	u4CurTime = kalGetTimeTick();

	if ((u4CurTime > prSwRfb->rRxTime) &&
		(prSwRfb->rRxTime != 0))
	{
		u4DifTime = u4CurTime - prSwRfb->rRxTime;

		if (prStaRec->u4StayIntMinRx[u4LenId] == 0) /* impossible */
			prStaRec->u4StayIntMinRx[u4LenId] = 0xffffffff;

		if (u4DifTime > prStaRec->u4StayIntMaxRx[u4LenId])
			prStaRec->u4StayIntMaxRx[u4LenId] = u4DifTime;
		else if (u4DifTime < prStaRec->u4StayIntMinRx[u4LenId])
			prStaRec->u4StayIntMinRx[u4LenId] = u4DifTime;

		prStaRec->u4StayIntAvgRx[u4LenId] += u4DifTime;
		if (prStaRec->u4StayIntAvgRx[u4LenId] != u4DifTime)
			prStaRec->u4StayIntAvgRx[u4LenId] >>= 1;
	}
}


/*----------------------------------------------------------------------------*/
/*! \brief  This routine is called to handle rx done.
*
* \param[in] prGlueInfo		Pointer to the Adapter structure
* \param[in] prInBuf		A pointer to the command string buffer
* \param[in] u4InBufLen	The length of the buffer
* \param[out] None
*
* \retval None
*/
/*----------------------------------------------------------------------------*/
UINT_64
StatsEnvTimeGet(
	VOID
	)
{
	/* TODO: use better API to get time to save time, jiffies unit is 10ms, too large */

//	struct timeval tv;


//	do_gettimeofday(&tv);
//	return tv.tv_usec + tv.tv_sec * (UINT_64)1000000;

	UINT_64 u8Clk;
//	UINT32 *pClk = &u8Clk;

	u8Clk = sched_clock(); /* unit: naro seconds */
//printk("<stats> sched_clock() = %x %x %u\n", pClk[0], pClk[1], sizeof(jiffies));

	return (UINT_64)u8Clk; /* sched_clock */ /* jiffies size = 4B */
}


/*----------------------------------------------------------------------------*/
/*! \brief  This routine is called to handle rx done.
*
* \param[in] prGlueInfo		Pointer to the Adapter structure
* \param[in] prInBuf		A pointer to the command string buffer
* \param[in] u4InBufLen	The length of the buffer
* \param[out] None
*
* \retval None
*/
/*----------------------------------------------------------------------------*/
VOID
StatsEnvTxTime2Hif(
	MSDU_INFO_T							*prMsduInfo,
	HIF_TX_HEADER_T						*prHwTxHeader
	)
{
	UINT_64 u8SysTime, u8SysTimeIn;
	UINT32 u4TimeDiff;


	u8SysTime = StatsEnvTimeGet();
	u8SysTimeIn = GLUE_GET_PKT_XTIME(prMsduInfo->prPacket);

//	printk("<stats> hif: 0x%x %u %u %u\n",
//		prMsduInfo->prPacket, StatsEnvTimeGet(), u8SysTime, GLUE_GET_PKT_XTIME(prMsduInfo->prPacket));

	if ((u8SysTimeIn > 0) && (u8SysTime > u8SysTimeIn))
	{
		u8SysTime = u8SysTime - u8SysTimeIn;
		u4TimeDiff = (UINT32)u8SysTime;
		u4TimeDiff = u4TimeDiff/1000; /* ns to us */

		if (u4TimeDiff > 0xFFFF)
			*(UINT16 *)prHwTxHeader->aucReserved = (UINT16)0xFFFF; /* 65535 us */
		else
			*(UINT16 *)prHwTxHeader->aucReserved = (UINT16)u4TimeDiff;

//		printk("<stats> u4TimeDiff: %u\n", u4TimeDiff);
	}
	else
	{
		prHwTxHeader->aucReserved[0] = 0;
		prHwTxHeader->aucReserved[1] = 0;
	}
}


#endif /* CFG_SUPPORT_STATISTICS */

/* End of stats.c */

