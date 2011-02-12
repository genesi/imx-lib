/*
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*!
 * @file iso7816-3.h
 *
 * @brief Library for Freescale IMX SIM interface
 *
 * @ingroup SIM
 */

#ifndef ISO7816_3_H
#define ISO7816_3_H

/* Extended error codes */
#define SIM_E_INVALIDCALLBACK        19
#define SIM_E_BUFFERTOOSMALL         20
#define SIM_E_INVALPARAM             21
#define SIM_E_READERSTART            22

/* Function prototype */
extern int RegisterCardStateCallbackFunc(int (*cardstatecallbackfunc) (void));
extern int SendReceiveAPDU(unsigned char *cmd, int cmdlen, unsigned char *resp, int resplen);
extern int SendReceivePTS(unsigned char *pts_request, unsigned char *pts_response, int ptslen);
extern int ReaderStart(void);
extern void ReaderStop(void);
extern int GetCardState(void);
extern int GetAtr(unsigned char *buf, int *size);
extern int GetParamAtr(int *fi, int *di, int *t, int *n);
extern int GetParam(int *fi, int *di, int *t, int *n);
extern int SetParam(int fi, int di, int t, int n);
extern int ColdReset(void);
extern int WarmReset(void);
extern int GetError(void);
extern int LockCard(void);
extern int EjectCard(void);

#endif
