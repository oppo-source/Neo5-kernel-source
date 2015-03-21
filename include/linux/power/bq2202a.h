/************************************************************************************
** File:  \\172.16.101.171\Linux_Samba\work\kongfanhong\14033\MSM8926.LA.1.0.0.1.101.1_DEV_ROM\android
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      for battery encryption  bq2202a
** 
** Version: 1.0
** Date created: 22:45:46,04/09/2014
** Author: Fanhong.Kong@ProDrv.CHG
** 
** --------------------------- Revision History: ------------------------------------------------------------
** 	<author>	<data>			<desc>
** 1.1          2014-4-19       Fanhong.Kong@ProDrv.CHG             qcom bq2202a ic
************************************************************************************************************/

#ifndef _BQ2202A_SW_H_
#define _BQ2202A_SW_H_

extern void ReadBq2202aID(void);
extern void CheckBq2202aID(void);
extern void CheckIDCompare (void);
extern int Gpio_BatId_Init(void);
#endif // _BQ2202A_SW_H_

