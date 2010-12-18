/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */
#ifndef __SCREENLAYER_H__
#define __SCREENLAYER_H__

#ifdef __cplusplus
extern "C"{
#endif

/* data type */
#define u8 		unsigned char
#define u16 		unsigned short
#define u32 		unsigned int
#define s32 		int

typedef enum {
	E_RET_SUCCESS = 0,
	E_RET_DEV_FAIL,
	E_RET_WRONG_FMT,
	E_RET_MEM_ALOC_FAIL,
	E_RET_MMAP_FAIL,
	E_RET_PRIMARY_ERR,
	E_RET_RECT_OVERFLOW,
	E_RET_BUFIDX_ERR,
	E_RET_TASK_SETUP_ERR,
	E_RET_TASK_RUN_ERR,
	E_RET_FLIP_ERR,
	E_RET_NOSUCH_METHODTYPE,
	E_RET_DESTORY_PRI_WITH_SUBSL,
	E_RET_ALPHA_BLENDING_CONFLICT,
	E_RET_LOCAL_ALPHA_BLENDING_DISABLE,
	E_RET_ALPHA_BUF_NOT_ALLOC_ERR,
	E_RET_IPC_SEM_OPEN_FAILED,
	E_RET_IPC_SHM_FAILED,
} SLRetCode;

typedef enum {
	F_DUMMY = 0x1,
} SLFlag;

typedef enum {
	TVOUT_DISABLE = 0,
	TVOUT_PAL,
	TVOUT_NTSC,
} TvMode;

typedef enum {
	E_SET_ALPHA,
	E_SET_COLORKEY,
	E_ENABLE_LAYER,
	E_COPY_TVOUT,
} SetMethodType;

typedef struct {
	u8	globalAlphaEnable;
	u8	sepLocalAlphaEnable;
	u32	alpha;
} MethodAlphaData;

typedef struct {
	u8	enable;
	u32	keyColor;
} MethodColorKeyData;

typedef struct {
	u8	tvMode;
	u32	lcd2tvRotation;
} MethodTvoutData;

typedef struct {
	u16		left;
	u16		top;
	u32		right;
	u32		bottom;
} SLRect;

typedef struct {
	SLRect 		screenRect;
	u32 		fmt;
	u32		bufSize;
	u32		bufAlphaSize;
	bool		supportSepLocalAlpha;
	void 		** bufVaddr;
	dma_addr_t 	* bufPaddr;
	void 		** bufAlphaVaddr;
	dma_addr_t 	* bufAlphaPaddr;
	void	 	* pPrimary;
	char		fbdev[32];
	u32		flag;
	void 		* pPriv;
} ScreenLayer;

typedef struct {
	u32		srcWidth;
	u32		srcHeight;
	u32		srcFmt;
	SLRect		srcRect;
	SLRect		destRect;
	u32		destRot;
	dma_addr_t 	srcPaddr;
} LoadParam;

/* APIs */
SLRetCode CreateScreenLayer(ScreenLayer *pSL, u8 nBufNum);
SLRetCode LoadScreenLayer(ScreenLayer *pSL, LoadParam *pParam, u8 nBufIdx);
SLRetCode LoadAlphaPoint(ScreenLayer *pSL, u32 x, u32 y, u8 alphaVal, u8 nBufIdx);
SLRetCode FlipScreenLayerBuf(ScreenLayer *pSL, u8 nBufIdx);
SLRetCode UpdateScreenLayer(ScreenLayer *pSL);
SLRetCode SetScreenLayer(ScreenLayer *pSL, SetMethodType eType, void *setData);
SLRetCode DestoryScreenLayer(ScreenLayer *pSL);

/*
** Get the handle of Primaray screen layer, which will be used to create the others Non-primary screen layer.
**
** Input  : fbdev, this is the fixed id of frame buffer
** Return : The handle of the Primary Screen Layer
*/
void* GetPrimarySLHandle(char * pFbdev) ;

/*
** Get the width of Primary screen layer.
**
** Input  : pPrimaryHandle, this is the handle of primary screen layer
** Return : the width of Primary screen layer
*/
u32   GetPrimarySLWidth(void * pPrimaryHandle);

/*
** Get the height of Primary screen layer.
**
** Input  : pPrimaryHandle, this is the handle of primary screen layer
** Return : the height of Primary screen layer
*/
u32   GetPrimarySLHeight(void * pPrimaryHandle);

#ifdef __cplusplus
}
#endif

#endif
