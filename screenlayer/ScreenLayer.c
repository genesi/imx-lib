/*
 * Copyright 2009-2010 Freescale Semiconductor, Inc. All Rights Reserved.
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

#ifdef __cplusplus
extern "C"{
#endif

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <malloc.h>
#include <string.h>
#include <semaphore.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/videodev.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include "ScreenLayer.h"
#include "mxc_ipu_hl_lib.h"

#define DBG_DEBUG		3
#define DBG_INFO		2
#define DBG_WARNING		1
#define DBG_ERR			0

static int debug_level = DBG_WARNING;
#define dbg(flag, fmt, args...)	{ if(flag <= debug_level)  printf("%s:%d "fmt, __FILE__, __LINE__,##args); }

/* Indicates the max number which will be supported in the CreateScreenLayer */
#define BUF_POOL_NUMBER		20

/*
**   Privare data will be stored in the shm.
**
**   layour of shared memory
**
**   Primary    Second     Third
**ID:    1          2          3
**  |----------|----------|----------|
*/
typedef struct {
	u8		isPrimary;
	u8 		bufNum;
	u8		curBufIdx;
	dma_addr_t	dispPaddr[2];
	u8		curDispIdx;
	u8		alphaGlobalEnable;
	u8		sepAlphaLocalEnable;
	u8		alpha;
	u8		keyColorEnable;
	u32		keyColor;
	u8		layerEnable;
	u8		tvMode;
	u8		tvEnabled;
	u32		tvRotation;
	ipu_lib_handle_t tvHandle;
	s32		fdIpu;
        ipu_mem_info 	* bufMinfo;
	ipu_mem_info 	* bufAlphaMinfo;
        ipu_mem_info 	dispMinfo;
	/* Add for IPC, backup infor from external ScreenLayer */
	SLRect 		screenRect;
	u32 		fmt;
	bool		supportSepLocalAlpha;
	dma_addr_t 	bufPaddr[BUF_POOL_NUMBER];
	dma_addr_t 	bufAlphaPaddr[BUF_POOL_NUMBER];
	u8		userAllocSLbuf;
	u8		userAllocAlpbuf;
	u32		flag;
	char		fbdev[32];

	int		layerID;
	int		preLayerId;
	int		nextLayerId;
} ScreenLayerPriv;
/* variables for semaphore */
sem_t * 	semMainID;
sem_t * 	semLoadID;
sem_t * 	semDispID;
const  char* 	semMainName="IPU_SL_SEM_MAIN";
const  char* 	semLoadName="IPU_SL_SEM_LOAD";
const  char* 	semDispName="IPU_SL_SEM_DISP";
static char	shmName[12]="shm_fb0";

/* Indicates how many threads in the current process */
static int pthread_counter=0;

/* virtual address of shared memory in current Process */
static ScreenLayerPriv  * vshmSLPriv = NULL;
SLRetCode _UpdateFramebuffer(ScreenLayerPriv *pSLPriv);

int tvmode_system(char *pFileName, char *value)
{
    FILE *fp = NULL;
    fp = fopen(pFileName, "w");
    if(fp == NULL) {
        dbg(DBG_ERR, "Open %s failed.\n", pFileName);
        return -1;
    }
    fwrite(value, 1, strlen(value), fp);
    fflush(fp);
    fclose(fp);
    return 0;
}

int copy_to_tv_init(ScreenLayerPriv *pSLPriv)
{
	ScreenLayerPriv *pPriSLPriv = vshmSLPriv;
	int fd_SLfb = 0, fd_TVfb = 0, ret = 0, screen_size0;
        struct fb_var_screeninfo SLfb_var, TVfb_var;
        struct fb_fix_screeninfo SLfb_fix, TVfb_fix;
        ipu_lib_input_param_t input;
        ipu_lib_output_param_t output;
        int mode;

        dbg(DBG_DEBUG, "copy_to_tv_init\n");

	tvmode_system("/sys/class/graphics/fb1/blank", "1\n");

	switch (pSLPriv->tvMode) {
	case TVOUT_PAL:
		tvmode_system("/sys/class/graphics/fb1/mode", "U:720x576i-50\n");
		break;
	case TVOUT_NTSC:
		tvmode_system("/sys/class/graphics/fb1/mode", "U:720x480i-60\n");
		break;
	default:
		dbg(DBG_ERR, "not support tvout mode %d\n", pSLPriv->tvMode);
		ret = -1;
		goto done;
	}

        if ((fd_SLfb = open(pPriSLPriv->fbdev, O_RDWR, 0)) < 0) {
                dbg(DBG_ERR, "Unable to open %s\n", pPriSLPriv->fbdev);
                ret = -1;
                goto done;
        }
        if ((fd_TVfb = open("/dev/fb1", O_RDWR, 0)) < 0) {
                dbg(DBG_ERR, "Unable to open tv fb /dev/fb1\n");
                ret = -1;
                goto done;
        }

        if ( ioctl(fd_SLfb, FBIOGET_FSCREENINFO, &SLfb_fix) < 0) {
                dbg(DBG_ERR, "Get FB fix info failed!\n");
                ret = -1;
                goto done;
        }

        if ( ioctl(fd_SLfb, FBIOGET_VSCREENINFO, &SLfb_var) < 0) {
                dbg(DBG_ERR, "Get FB var info failed!\n");
                ret = -1;
                goto done;
        }
        if ( ioctl(fd_TVfb, FBIOGET_VSCREENINFO, &TVfb_var) < 0) {
                dbg(DBG_ERR, "Get FB var info failed!\n");
                ret = -1;
                goto done;
        }

	if ((TVfb_var.yres == TVfb_var.yres_virtual)) {
		TVfb_var.yres_virtual = TVfb_var.yres * 2;
	}
	TVfb_var.nonstd = v4l2_fourcc('U', 'Y', 'V', 'Y');
	TVfb_var.bits_per_pixel = 16;
	if ( ioctl(fd_TVfb, FBIOPUT_VSCREENINFO, &TVfb_var) < 0) {
		dbg(DBG_ERR, "Set FB var info failed!\n");
		ret = -1;
		goto done;
	}
	tvmode_system("/sys/class/graphics/fb1/blank", "0\n");

        if ( ioctl(fd_TVfb, FBIOGET_FSCREENINFO, &TVfb_fix) < 0) {
                dbg(DBG_ERR, "Get FB fix info failed!\n");
                ret = -1;
                goto done;
        }

        memset(&(pSLPriv->tvHandle), 0, sizeof(ipu_lib_handle_t));
        memset(&input, 0, sizeof(ipu_lib_input_param_t));
        memset(&output, 0, sizeof(ipu_lib_output_param_t));

	mode = OP_STREAM_MODE | TASK_PP_MODE;
        input.width = SLfb_var.xres;
        input.height = SLfb_var.yres;
	if (SLfb_var.nonstd)
		input.fmt = SLfb_var.nonstd;
	else if (SLfb_var.bits_per_pixel == 24)
                input.fmt = v4l2_fourcc('B', 'G', 'R', '3');
        else
                input.fmt = v4l2_fourcc('R', 'G', 'B', 'P');
        screen_size0 = SLfb_var.yres * SLfb_fix.line_length;
	if (SLfb_var.yoffset) {
		input.user_def_paddr[0] = SLfb_fix.smem_start + screen_size0;
		input.user_def_paddr[1] = SLfb_fix.smem_start + screen_size0;
	} else {
		input.user_def_paddr[0] = SLfb_fix.smem_start;
		input.user_def_paddr[1] = SLfb_fix.smem_start;
	}

        output.width = TVfb_var.xres;
        output.height = TVfb_var.yres;
        output.fmt = v4l2_fourcc('U', 'Y', 'V', 'Y');
        output.rot = pSLPriv->tvRotation;
	output.show_to_fb = 1;
	output.fb_disp.fb_num = 1;

	if (mxc_ipu_lib_task_init(&input, NULL, &output, mode, &(pSLPriv->tvHandle)) < 0) {
		ret = -1;
		goto done;
	}

done:
	if (fd_SLfb)
		close(fd_SLfb);
	if (fd_TVfb)
		close(fd_TVfb);
	return ret;
}

int copy_to_tv_update(ScreenLayerPriv *pSLPriv, dma_addr_t new_addr)
{
        dbg(DBG_DEBUG, "copy_to_tv_update\n");
	if (mxc_ipu_lib_task_buf_update(&(pSLPriv->tvHandle), new_addr, 0, 0, 0, 0) < 0)
		return -1;
	return 0;
}

void copy_to_tv_deinit(ScreenLayerPriv *pSLPriv)
{
        dbg(DBG_DEBUG, "copy_to_tv_deinit\n");
	mxc_ipu_lib_task_uninit(&(pSLPriv->tvHandle));
}

static u32 fmt_to_bpp(u32 pixelformat)
{
	u32 bpp;

	switch (pixelformat)
	{
		case IPU_PIX_FMT_RGB565:
		/*interleaved 422*/
		case IPU_PIX_FMT_YUYV:
		case IPU_PIX_FMT_UYVY:
		/*non-interleaved 422*/
		case IPU_PIX_FMT_YUV422P:
		case IPU_PIX_FMT_YVU422P:
			bpp = 16;
			break;
		case IPU_PIX_FMT_BGR24:
		case IPU_PIX_FMT_RGB24:
		case IPU_PIX_FMT_YUV444:
			bpp = 24;
			break;
		case IPU_PIX_FMT_BGR32:
		case IPU_PIX_FMT_BGRA32:
		case IPU_PIX_FMT_RGB32:
		case IPU_PIX_FMT_RGBA32:
		case IPU_PIX_FMT_ABGR32:
			bpp = 32;
			break;
		/*non-interleaved 420*/
		case IPU_PIX_FMT_YUV420P:
		case IPU_PIX_FMT_YUV420P2:
		case IPU_PIX_FMT_NV12:
			bpp = 12;
			break;
		default:
			bpp = 8;
			break;
	}
	return bpp;
}

SLRetCode _MemAllocSL(ScreenLayer *pSL)
{
	SLRetCode ret = E_RET_SUCCESS;
	ScreenLayerPriv *pSLPriv = (ScreenLayerPriv *)(vshmSLPriv + (int)pSL->pPriv-1);
	u8 i;
	u32 width, height, screen_size;

	pSLPriv->fdIpu = open("/dev/mxc_ipu",O_RDWR);
	if(pSLPriv->fdIpu < 0) {
		ret = E_RET_DEV_FAIL;
		goto done;
	}

	/*
 	 * alloc disp buf, for primary layer should be framebuffer from fb device,
 	 * otherwise, a tmp buffer may be allocated.
 	 *
 	 * 3 cases:
 	 *
 	 * a. only primary
 	 * 	PrimarySL(dispBuf)
 	 * b. primary + one overlay
 	 * 	PrimarySL -> OverlaySL0(dispBuf)
 	 * c. primary + two overlay
 	 * 	PrimarySL -> OverlaySL0(tmpDispBuf) -> OverlaySL1(dispBuf)
 	 */
	if (!pSLPriv->isPrimary) {
		ScreenLayerPriv *pPreSLPriv = (ScreenLayerPriv *)(vshmSLPriv + pSLPriv->preLayerId-1);
		ScreenLayerPriv *pPriSLPriv = vshmSLPriv;

		if (pSLPriv->preLayerId != (int)pSL->pPrimary) {
			/* case b -> c
			 * allocate tmpDispBuf from current SL, make it as PreSL's DispBuf.
			 * and use PreSL's DispBuf as currentSL's display buffer.
			 */
			width  = pPriSLPriv->screenRect.right  - pPriSLPriv->screenRect.left;
			height = pPriSLPriv->screenRect.bottom - pPriSLPriv->screenRect.top;
			pSLPriv->dispMinfo.size = width/8*height*fmt_to_bpp(pPriSLPriv->fmt);
			if (ioctl(pSLPriv->fdIpu, IPU_ALOC_MEM, &(pSLPriv->dispMinfo)) < 0) {
				ret = E_RET_MEM_ALOC_FAIL;
				dbg(DBG_ERR, "_MemAllocSL: IPU memory alloc failed! \n");
				goto done;
			}

			pSLPriv->dispPaddr[0] = pPreSLPriv->dispPaddr[0];
			pSLPriv->dispPaddr[1] = pPreSLPriv->dispPaddr[1];
			pSLPriv->curDispIdx = pPreSLPriv->curDispIdx;
			pPreSLPriv->dispPaddr[0] = pSLPriv->dispMinfo.paddr;
			pPreSLPriv->dispPaddr[1] = 0;
			pPreSLPriv->curDispIdx = 0;

			dbg(DBG_DEBUG, "allocate %d memory paddr 0x%x for pre layer\n", pSLPriv->dispMinfo.size, pSLPriv->dispMinfo.paddr);
		} else {
			/* case a -> b */
			pSLPriv->dispPaddr[0] = pPreSLPriv->dispPaddr[0];
			pSLPriv->dispPaddr[1] = pPreSLPriv->dispPaddr[1];
			pSLPriv->curDispIdx = pPreSLPriv->curDispIdx;
			pPreSLPriv->dispPaddr[0] = 0;
			pPreSLPriv->dispPaddr[1] = 0;
			pPreSLPriv->curDispIdx = 0;
		}
	} else {
		/* case a */
		s32 fd_fb;
		struct fb_fix_screeninfo fb_fix;
		struct fb_var_screeninfo fb_var;

		if ((fd_fb = open(pSL->fbdev, O_RDWR, 0)) < 0) {
			memcpy(pSL->fbdev, "/dev/fb", 8);
			if ((fd_fb = open(pSL->fbdev, O_RDWR, 0)) < 0) {
				ret = E_RET_DEV_FAIL;
				goto done;
			}
		}

		if ( ioctl(fd_fb, FBIOGET_VSCREENINFO, &fb_var) < 0) {
			ret = E_RET_DEV_FAIL;
			close(fd_fb);
			goto done;
		}

		if ((pSL->fmt == v4l2_fourcc('U', 'Y', 'V', 'Y')) ||
				(pSL->fmt == v4l2_fourcc('Y', '4', '4', '4'))) {
			fb_var.nonstd = pSL->fmt;
			if (pSL->fmt == v4l2_fourcc('U', 'Y', 'V', 'Y'))
				fb_var.bits_per_pixel = 16;
			else
				fb_var.bits_per_pixel = 24;
		} else {
			fb_var.nonstd = 0;
			fb_var.bits_per_pixel = fmt_to_bpp(pSL->fmt);
		}

		if ( ioctl(fd_fb, FBIOPUT_VSCREENINFO, &fb_var) < 0) {
			ret = E_RET_DEV_FAIL;
			close(fd_fb);
			goto done;
		}

		if ( ioctl(fd_fb, FBIOGET_VSCREENINFO, &fb_var) < 0) {
			ret = E_RET_DEV_FAIL;
			close(fd_fb);
			goto done;
		}

		if ( ioctl(fd_fb, FBIOGET_FSCREENINFO, &fb_fix) < 0) {
			ret = E_RET_DEV_FAIL;
			close(fd_fb);
			goto done;
		}

		if (fb_var.bits_per_pixel != fmt_to_bpp(pSL->fmt)) {
			dbg(DBG_ERR, "request format should be the same as fb dev!\n");
			ret = E_RET_WRONG_FMT;
			goto done;
		}

		/* make the primary layer the same size as fb device */
		pSL->screenRect.left = pSL->screenRect.top = 0;
		pSL->screenRect.right =  fb_var.xres;
		pSL->screenRect.bottom =  fb_var.yres;

		screen_size = fb_var.yres * fb_fix.line_length;
		pSLPriv->dispPaddr[0] = fb_fix.smem_start;
		pSLPriv->dispPaddr[1] = fb_fix.smem_start + screen_size;

		dbg(DBG_DEBUG, "screen layer display to %s, dispPaddr 0x%x 0x%x\n", pSL->fbdev, pSLPriv->dispPaddr[0], pSLPriv->dispPaddr[1]);

		close(fd_fb);
	}

	width = pSL->screenRect.right - pSL->screenRect.left;
	height = pSL->screenRect.bottom - pSL->screenRect.top;

	/* For Screenlayer buffer */
	if (!pSL->bufPaddr) {
		pSL->bufPaddr = (dma_addr_t *)malloc(pSLPriv->bufNum * sizeof(dma_addr_t));
		pSL->bufVaddr = (void **)malloc(pSLPriv->bufNum * sizeof(void *));
		pSLPriv->bufMinfo = (ipu_mem_info *)malloc(pSLPriv->bufNum * sizeof(ipu_mem_info));
	} else {
		dbg(DBG_DEBUG, "screen layer user define SL buffer\n");
		pSLPriv->userAllocSLbuf = 1;
	}
	pSL->bufSize = width/8*height*fmt_to_bpp(pSL->fmt);

	/* For local alpha blending buffers */
	if (pSL->supportSepLocalAlpha) {
		if (!pSL->bufAlphaPaddr) {
			pSL->bufAlphaPaddr = (dma_addr_t *)malloc(pSLPriv->bufNum * sizeof(dma_addr_t));
			pSL->bufAlphaVaddr = (void **)malloc(pSLPriv->bufNum * sizeof(void *));
			pSLPriv->bufAlphaMinfo = (ipu_mem_info *)malloc(pSLPriv->bufNum * sizeof(ipu_mem_info));
		} else {
			dbg(DBG_DEBUG, "screen layer user define Alpha buffer\n");
			pSLPriv->userAllocAlpbuf = 1;
		}
		pSL->bufAlphaSize = width * height;
	}

	for (i=0;i<pSLPriv->bufNum;i++) {
		if (!pSLPriv->userAllocSLbuf) {
			pSLPriv->bufMinfo[i].size = pSL->bufSize;
			if (ioctl(pSLPriv->fdIpu, IPU_ALOC_MEM, &(pSLPriv->bufMinfo[i])) < 0) {
				ret = E_RET_MEM_ALOC_FAIL;
				goto err;
			}
			pSL->bufPaddr[i] = pSLPriv->bufMinfo[i].paddr;
			/* mmap virtual addr for user*/
			pSL->bufVaddr[i] = mmap (NULL, pSLPriv->bufMinfo[i].size,
					PROT_READ | PROT_WRITE, MAP_SHARED,
					pSLPriv->fdIpu, pSLPriv->bufMinfo[i].paddr);
			if (pSL->bufVaddr[i] == MAP_FAILED) {
				ret = E_RET_MMAP_FAIL;
				goto err;
			}

			dbg(DBG_DEBUG, "allocate %d memory paddr 0x%x, mmap to %p for current layer\n",
					pSLPriv->bufMinfo[i].size, pSL->bufPaddr[i], pSL->bufVaddr[i]);
		}

		/* Allocate local alpha blending buffers */
		if (pSL->supportSepLocalAlpha && !pSLPriv->userAllocAlpbuf) {
			pSLPriv->bufAlphaMinfo[i].size = pSL->bufAlphaSize;
			if (ioctl(pSLPriv->fdIpu, IPU_ALOC_MEM,
				  &(pSLPriv->bufAlphaMinfo[i])) < 0) {
				ret = E_RET_MEM_ALOC_FAIL;
				goto err;
			}
			pSL->bufAlphaPaddr[i] = pSLPriv->bufAlphaMinfo[i].paddr;
			/* mmap virtual addr for user*/
			pSL->bufAlphaVaddr[i] = mmap (NULL,
					pSLPriv->bufAlphaMinfo[i].size,
					PROT_READ | PROT_WRITE, MAP_SHARED,
					pSLPriv->fdIpu,
					pSLPriv->bufAlphaMinfo[i].paddr);
			if (pSL->bufAlphaVaddr[i] == MAP_FAILED) {
				ret = E_RET_MMAP_FAIL;
				goto err;
			}

			dbg(DBG_DEBUG, "allocate %d memory paddr 0x%x, mmap to %p for local alpha blending buffers of \
				current layer\n", pSLPriv->bufAlphaMinfo[i].size, pSL->bufAlphaPaddr[i], pSL->bufAlphaVaddr[i]);
		}
	}

	goto done;

err:
	if (pSL->bufPaddr && !pSLPriv->userAllocSLbuf)
		free(pSL->bufPaddr);
	if (pSL->bufVaddr && !pSLPriv->userAllocSLbuf)
		free(pSL->bufVaddr);
	if (pSLPriv->bufMinfo)
		free(pSLPriv->bufMinfo);
	if (pSL->bufAlphaPaddr && !pSLPriv->userAllocAlpbuf)
		free(pSL->bufAlphaPaddr);
	if (pSL->bufAlphaVaddr && !pSLPriv->userAllocAlpbuf)
		free(pSL->bufAlphaVaddr);
	if (pSLPriv->bufAlphaMinfo)
		free(pSLPriv->bufAlphaMinfo);
done:
	return ret;
}

void _MemFreeSL(ScreenLayer *pSL)
{
	ScreenLayerPriv *pSLPriv = (ScreenLayerPriv *)(vshmSLPriv +(int)pSL->pPriv-1);
	u8 i;

	for (i=0;i<pSLPriv->bufNum;i++) {
		if (!pSLPriv->userAllocSLbuf) {
			dbg(DBG_DEBUG, "free %d memory paddr 0x%x, mmap to %p for current layer\n",
					pSLPriv->bufMinfo[i].size, pSL->bufPaddr[i], pSL->bufVaddr[i]);
			if (pSL->bufVaddr[i])
				munmap(pSL->bufVaddr[i], pSLPriv->bufMinfo[i].size);
			ioctl(pSLPriv->fdIpu, IPU_FREE_MEM, &(pSLPriv->bufMinfo[i]));
		}

		/* Free local alpha blending buffers */
		if (pSL->supportSepLocalAlpha && !pSLPriv->userAllocAlpbuf) {
			dbg(DBG_DEBUG, "free %d memory paddr 0x%x, mmap to %p for local alpha blending buffers of \
				current layer\n", pSLPriv->bufAlphaMinfo[i].size, pSL->bufAlphaPaddr[i], pSL->bufAlphaVaddr[i]);
			if (pSL->bufAlphaVaddr[i])
				munmap(pSL->bufAlphaVaddr[i],
				       pSLPriv->bufAlphaMinfo[i].size);
			ioctl(pSLPriv->fdIpu, IPU_FREE_MEM,
			      &(pSLPriv->bufAlphaMinfo[i]));
		}
	}

	if (pSLPriv->preLayerId && pSLPriv->nextLayerId) {
		/* case c -> b, destory middle layer, do nothing */
		dbg(DBG_DEBUG, "free middle layer. \n");
	} else if (pSLPriv->preLayerId) {
		ScreenLayerPriv *pPreSLPriv = (ScreenLayerPriv *)(vshmSLPriv + pSLPriv->preLayerId-1);
		if (pSLPriv->preLayerId == (int)pSL->pPrimary) {
			/* case b -> a */
			pPreSLPriv->dispPaddr[0] = pSLPriv->dispPaddr[0];
			pPreSLPriv->dispPaddr[1] = pSLPriv->dispPaddr[1];
			pPreSLPriv->curDispIdx = pSLPriv->curDispIdx;
			pSLPriv->dispPaddr[0] = 0;
			pSLPriv->dispPaddr[1] = 0;
		} else {
			/* case c -> b, destory top layer */
			dbg(DBG_DEBUG, "free %d memory disppaddr 0x%x for pre layer\n", pSLPriv->dispMinfo.size, pSLPriv->dispPaddr[0]);
			ioctl(pSLPriv->fdIpu, IPU_FREE_MEM, &(pSLPriv->dispMinfo));
			pPreSLPriv->dispPaddr[0] = pSLPriv->dispPaddr[0];
			pPreSLPriv->dispPaddr[1] = pSLPriv->dispPaddr[1];
			pPreSLPriv->curDispIdx = pSLPriv->curDispIdx;
			pSLPriv->dispPaddr[0] = 0;
			pSLPriv->dispPaddr[1] = 0;
		}
	}

	if (pSLPriv->isPrimary) {
		s32 fd_fb;
		struct fb_var_screeninfo fb_var;

		/* go back to RGB mode */
		fd_fb = open(pSL->fbdev, O_RDWR, 0);
		ioctl(fd_fb, FBIOGET_VSCREENINFO, &fb_var);
		if(fb_var.nonstd) {
			fb_var.nonstd = 0;
			ioctl(fd_fb, FBIOPUT_VSCREENINFO, &fb_var);
		}
		close(fd_fb);
	}
	if (pSL->bufPaddr && !pSLPriv->userAllocSLbuf)
		free(pSL->bufPaddr);
	if (pSL->bufVaddr && !!pSLPriv->userAllocSLbuf)
		free(pSL->bufVaddr);
	if (pSLPriv->bufMinfo)
		free(pSLPriv->bufMinfo);
	if (pSL->bufAlphaPaddr && !pSLPriv->userAllocAlpbuf)
		free(pSL->bufAlphaPaddr);
	if (pSL->bufAlphaVaddr && !pSLPriv->userAllocAlpbuf)
		free(pSL->bufAlphaVaddr);
	if (pSLPriv->bufAlphaMinfo)
		free(pSLPriv->bufAlphaMinfo);

	close(pSLPriv->fdIpu);
}

SLRetCode PreScreenLayerIPC(char *pFbdev)
{
	SLRetCode ret = E_RET_SUCCESS;
	int	shmID;
	struct	stat shmStat;

	dbg(DBG_DEBUG, "PreScreenLayerIPC begin!\n");
	/* Create one new semaphore or opens an existing semaphore */
	semMainID = sem_open(semMainName, O_CREAT, 0666, 1);
	if(SEM_FAILED == semMainID){
		dbg(DBG_ERR, "can not open the semaphore for : %s!\n", semMainName);
		ret = E_RET_IPC_SEM_OPEN_FAILED;
		goto pre_err0;
	}
	semLoadID = sem_open(semLoadName, O_CREAT, 0666, 1);
	if(SEM_FAILED == semLoadID){
		dbg(DBG_ERR, "can not open the semaphore for : %s!\n", semLoadName);
		ret = E_RET_IPC_SEM_OPEN_FAILED;
		sem_close(semMainID);
		goto pre_err0;
	}
	semDispID = sem_open(semDispName, O_CREAT, 0666, 1);
	if(SEM_FAILED == semDispID){
		dbg(DBG_ERR, "can not open the semaphore for : %s!\n", semDispName);
		ret = E_RET_IPC_SEM_OPEN_FAILED;
		sem_close(semMainID);
		sem_close(semLoadID);
		goto pre_err0;
	}

	sem_wait(semMainID);

	/* Create one new shm or get one existing shm object's ID */
	if(strstr(pFbdev,"fb1"))
		strcpy(shmName, "shm_fb1");
	else if(strstr(pFbdev,"fb2"))
		strcpy(shmName, "shm_fb2");

	shmID = shm_open(shmName, O_RDWR|O_CREAT, 0666);
	if(shmID == -1){
		dbg(DBG_ERR, "can not open the shared memory for : %s!\n",pFbdev);
		ret = E_RET_IPC_SHM_FAILED;
		sem_post(semMainID);
		sem_close(semMainID);
		sem_close(semLoadID);
		sem_close(semDispID);
		goto pre_err0;
	}
	/* Get special size shm */
	ftruncate(shmID,3 * sizeof(ScreenLayerPriv));
	/* Connect to the shm */
	fstat(shmID, &shmStat);

	if(vshmSLPriv == NULL)
	{
		vshmSLPriv = (ScreenLayerPriv *)mmap(NULL,shmStat.st_size,PROT_READ|PROT_WRITE,MAP_SHARED,shmID,0);
		dbg(DBG_DEBUG, "PreScreenLayerIPC Memory map done !\n");
	}

	if(vshmSLPriv == MAP_FAILED || vshmSLPriv ==NULL){
		dbg(DBG_ERR, "shm mmap failed!\n");
		ret = E_RET_IPC_SHM_FAILED;
		sem_post(semMainID);
		sem_close(semMainID);
		sem_close(semLoadID);
		sem_close(semDispID);
		goto pre_err1;
	}

	sem_post(semMainID);
	dbg(DBG_DEBUG, "PreScreenLayerIPC end!\n");
pre_err1:
	close(shmID);
pre_err0:
	return ret;
}

SLRetCode CreateScreenLayer(ScreenLayer *pSL, u8 nBufNum)
{
	SLRetCode ret = E_RET_SUCCESS;
	ScreenLayerPriv *pSLPriv;
	int	curShmPrivID;
	int	i, primaryId = (int)(pSL->pPrimary);

	if ((primaryId != 0) && (primaryId != 1)) {
		dbg(DBG_ERR, "Primary Id error!\n");
		ret = E_RET_PRIMARY_ERR;
		goto done;
	}

	if(vshmSLPriv == NULL)
		ret = PreScreenLayerIPC(pSL->fbdev);
	if(ret != E_RET_SUCCESS)
	{
		dbg(DBG_ERR, "Prepared semaphore & shm failed !\n");
		goto done;
	}
	sem_wait(semMainID);
	pthread_counter++;

	/* Alloc shared memory for current Layer private struct */
	/*
	**   layour of shared memory
	**
	**   Primary    Second     Third
	**ID:    1          2          3
	**  |----------|----------|----------|
	*/

	if (primaryId == 1) {
		/* Non Primary SL*/
		ScreenLayerPriv *pPriSLPriv = vshmSLPriv;
		ScreenLayerPriv *pCurSLPriv;

		if(pPriSLPriv->nextLayerId == 0){
			/* The seconde layer*/
			pSLPriv = vshmSLPriv + 1;
			curShmPrivID = 2;
		}else{
			/* The third layer */
			pSLPriv = vshmSLPriv + 2;
			curShmPrivID = 3;
		}
		memset(pSLPriv, 0, sizeof(ScreenLayerPriv));
		pSL->pPriv = (void *)curShmPrivID;
		pSLPriv->layerID = curShmPrivID;

		if (!pPriSLPriv->isPrimary) {
			dbg(DBG_ERR, "new screen layer should created based on a primary one!\n");
			ret = E_RET_PRIMARY_ERR;
			goto err;
		}

		if ((pSL->screenRect.left >= pPriSLPriv->screenRect.right) ||
			(pSL->screenRect.right > pPriSLPriv->screenRect.right) ||
			(pSL->screenRect.top  >= pPriSLPriv->screenRect.bottom) ||
			(pSL->screenRect.bottom> pPriSLPriv->screenRect.bottom)) {
			dbg(DBG_ERR, "new screen layer is bigger than primary one!\n");
			ret = E_RET_RECT_OVERFLOW;
			goto err;
		}

		pCurSLPriv = pPriSLPriv;
		while (pCurSLPriv->nextLayerId) {
			pCurSLPriv = vshmSLPriv + (pCurSLPriv->nextLayerId-1);
		}
		pCurSLPriv->nextLayerId = pSLPriv->layerID;
		pSLPriv->preLayerId = pCurSLPriv->layerID ;

		pSLPriv->isPrimary = 0;
	} else {
		/* Primary SL */
		/* shm initialization */
		memset(vshmSLPriv, 0, 3*sizeof(ScreenLayerPriv));

		pSLPriv = vshmSLPriv;
		curShmPrivID = 1;
		pSL->pPriv = (void *)curShmPrivID;
		pSLPriv->layerID = curShmPrivID;

		if (pSL->supportSepLocalAlpha) {
			dbg(DBG_ERR, "primary screen layer should not support local alpha blending!\n");
			ret = E_RET_PRIMARY_ERR;
			goto err;
		}

		pSLPriv->isPrimary = 1;
	}

	pSLPriv->bufNum = nBufNum;

	ret = _MemAllocSL(pSL);
	/* Back up SL infor from external to private struct */
	pSLPriv->screenRect.left = pSL->screenRect.left;
	pSLPriv->screenRect.top  = pSL->screenRect.top;
	pSLPriv->screenRect.right  = pSL->screenRect.right;
	pSLPriv->screenRect.bottom = pSL->screenRect.bottom;
	pSLPriv->fmt = pSL->fmt;
	pSLPriv->supportSepLocalAlpha = pSL->supportSepLocalAlpha;
	strcpy(pSLPriv->fbdev, pSL->fbdev);
	pSLPriv->flag = pSL->flag;
	for(i=0;i<nBufNum;i++)
	{
		pSLPriv->bufPaddr[i]      = pSL->bufPaddr[i];
		if (pSL->supportSepLocalAlpha)
			pSLPriv->bufAlphaPaddr[i] = pSL->bufAlphaPaddr[i];
	}
err:
	sem_post(semMainID);
done:
	return ret;
}

SLRetCode DestoryScreenLayer(ScreenLayer *pSL)
{
	SLRetCode ret = E_RET_SUCCESS;
	ScreenLayerPriv *pSLPriv = (ScreenLayerPriv *)(vshmSLPriv + (int)pSL->pPriv-1);
	ScreenLayerPriv *pPreSLPriv, *pNextSLPriv;

	sem_wait(semMainID);

	_MemFreeSL(pSL);

	if (pSLPriv->preLayerId) {
		if (pSLPriv->nextLayerId) {
			pPreSLPriv  = (ScreenLayerPriv *)(vshmSLPriv + pSLPriv->preLayerId-1);
			pNextSLPriv = (ScreenLayerPriv *)(vshmSLPriv + pSLPriv->nextLayerId-1);
			pPreSLPriv->nextLayerId = pSLPriv->nextLayerId;
			pNextSLPriv->preLayerId = pSLPriv->preLayerId;
		} else {
			pPreSLPriv = (ScreenLayerPriv *)(vshmSLPriv + pSLPriv->preLayerId-1);
			pPreSLPriv->nextLayerId = 0;
		}
	}

	if (pSLPriv->tvEnabled) {
		copy_to_tv_deinit(pSLPriv);
		pSLPriv->tvEnabled = 0;
		pSLPriv->tvMode = TVOUT_DISABLE;
	}

	pthread_counter--;
	if(pthread_counter == 0) {
		if (vshmSLPriv)
			munmap(vshmSLPriv, 3*sizeof(ScreenLayerPriv));
		vshmSLPriv = NULL;
	}

	sem_post(semMainID);

	return ret;
}

SLRetCode LoadScreenLayer(ScreenLayer *pSL, LoadParam *pParam, u8 nBufIdx)
{
	SLRetCode ret = E_RET_SUCCESS;
	ScreenLayerPriv *pSLPriv = (ScreenLayerPriv *)(vshmSLPriv + (int)pSL->pPriv-1);
	ipu_lib_handle_t ipu_handle;
	ipu_lib_input_param_t input;
	ipu_lib_output_param_t output;
	int mode;
	int total_time = 0;
	struct timeval total_begin,total_end;
	int sec, usec;

	memset(&ipu_handle, 0, sizeof(ipu_lib_handle_t));
	memset(&input, 0, sizeof(ipu_lib_input_param_t));
	memset(&output, 0, sizeof(ipu_lib_output_param_t));

	sem_wait(semMainID);
	if (nBufIdx >= pSLPriv->bufNum) {
		ret = E_RET_BUFIDX_ERR;
		goto done;
	}

	if ((pParam->srcRect.left >=  pParam->srcWidth) ||
		(pParam->srcRect.right > pParam->srcWidth) ||
		(pParam->srcRect.top >= pParam->srcHeight) ||
		(pParam->srcRect.bottom > pParam->srcHeight)){
		dbg(DBG_WARNING, "LoadScreenLayer src rect size not fit!\n")
		pParam->srcRect.left = 0;
		pParam->srcRect.top = 0;
		pParam->srcRect.right = pParam->srcWidth;
		pParam->srcRect.bottom = pParam->srcHeight;
	}

	if ((pParam->destRect.left >=  (pSL->screenRect.right - pSL->screenRect.left)) ||
		(pParam->destRect.right > (pSL->screenRect.right - pSL->screenRect.left)) ||
		(pParam->destRect.top >= (pSL->screenRect.bottom - pSL->screenRect.top)) ||
		(pParam->destRect.bottom > (pSL->screenRect.bottom - pSL->screenRect.top))){
		dbg(DBG_WARNING, "LoadScreenLayer dest rect size not fit!\n")
		pParam->destRect.left = 0;
		pParam->destRect.top = 0;
		pParam->destRect.right = pSL->screenRect.right - pSL->screenRect.left;
		pParam->destRect.bottom = pSL->screenRect.bottom - pSL->screenRect.top;
	}

	mode = OP_NORMAL_MODE | TASK_VF_MODE;
        input.width = pParam->srcWidth;
        input.height = pParam->srcHeight;
	input.input_crop_win.pos.x = pParam->srcRect.left;
	input.input_crop_win.pos.y = pParam->srcRect.top;
	input.input_crop_win.win_w = pParam->srcRect.right - pParam->srcRect.left;
	input.input_crop_win.win_h = pParam->srcRect.bottom - pParam->srcRect.top;
        input.fmt = pParam->srcFmt;
	input.user_def_paddr[0] = pParam->srcPaddr;

	output.width = pSL->screenRect.right -  pSL->screenRect.left;
	output.height = pSL->screenRect.bottom -  pSL->screenRect.top;
	output.output_win.pos.x = pParam->destRect.left;
	output.output_win.pos.y = pParam->destRect.top;
	output.output_win.win_w = pParam->destRect.right - pParam->destRect.left;
	output.output_win.win_h = pParam->destRect.bottom - pParam->destRect.top;
	output.fmt = pSL->fmt;
	output.rot = pParam->destRot;
	output.user_def_paddr[0] = pSL->bufPaddr[nBufIdx];

	sem_wait(semLoadID);
	gettimeofday(&total_begin, NULL);
	if (mxc_ipu_lib_task_init(&input, NULL, &output, mode, &ipu_handle) < 0) {
		ret = E_RET_TASK_SETUP_ERR;
		goto done;
	}

	if (mxc_ipu_lib_task_buf_update(&ipu_handle, 0, 0, 0, 0, 0) < 0) {
		ret = E_RET_TASK_RUN_ERR;
		goto done;
	}

	mxc_ipu_lib_task_uninit(&ipu_handle);

	gettimeofday(&total_end, NULL);

        sec = total_end.tv_sec - total_begin.tv_sec;
        usec = total_end.tv_usec - total_begin.tv_usec;

        if (usec < 0) {
                sec--;
                usec = usec + 1000000;
        }
        total_time += (sec * 1000000) + usec;

        dbg(DBG_DEBUG, "Load time %d us\n", total_time);
	sem_post(semLoadID);
	sem_post(semMainID);
done:
	return ret;
}

SLRetCode LoadAlphaPoint(ScreenLayer *pSL, u32 x, u32 y, u8 alphaVal, u8 nBufIdx)
{
	SLRetCode ret = E_RET_SUCCESS;
	ScreenLayerPriv *pSLPriv = (ScreenLayerPriv *)(vshmSLPriv +(int)pSL->pPriv-1);
	u8 *pPointAlphaVal;

	if (nBufIdx >= pSLPriv->bufNum) {
		ret = E_RET_BUFIDX_ERR;
		goto err;
	}

	if (!pSLPriv->sepAlphaLocalEnable || !pSL->supportSepLocalAlpha) {
		dbg(DBG_ERR, "local alpha blending is disabled!\n");
		ret = E_RET_LOCAL_ALPHA_BLENDING_DISABLE;
		goto err;
	}

	if (!pSL->bufAlphaVaddr[nBufIdx]) {
		dbg(DBG_ERR, "alpha buffer is not allocated!\n");
		ret = E_RET_ALPHA_BUF_NOT_ALLOC_ERR;
		goto err;
	}

	pPointAlphaVal = (u8 *)(pSL->bufAlphaVaddr[nBufIdx] +
	       (pSL->screenRect.right - pSL->screenRect.left)*y + x);

	*pPointAlphaVal = alphaVal;
err:
	return ret;
}

SLRetCode FlipScreenLayerBuf(ScreenLayer *pSL, u8 nBufIdx)
{
	ScreenLayerPriv *pSLPriv = (ScreenLayerPriv *)(vshmSLPriv + (int)pSL->pPriv-1);

	if (nBufIdx >= pSLPriv->bufNum)
		return E_RET_FLIP_ERR;

	sem_wait(semDispID);
	pSLPriv->curBufIdx = nBufIdx;
	sem_post(semDispID);

	return E_RET_SUCCESS;
}

SLRetCode _CopyScreenLayer(ScreenLayerPriv *pSrcSLPriv, ScreenLayerPriv *pTgtSLPriv)
{
	SLRetCode ret = E_RET_SUCCESS;
	ScreenLayerPriv *pPriSLPriv;
	ipu_lib_handle_t ipu_handle;
	ipu_lib_input_param_t input;
	ipu_lib_output_param_t output;
	s32 mode;
	int total_time = 0;
	struct timeval total_begin,total_end;
	int sec, usec;

	gettimeofday(&total_begin, NULL);

	memset(&ipu_handle, 0, sizeof(ipu_lib_handle_t));
	memset(&input, 0, sizeof(ipu_lib_input_param_t));
	memset(&output, 0, sizeof(ipu_lib_output_param_t));

	/* Top SL should show to fb */
	if (!pTgtSLPriv->nextLayerId)
		pTgtSLPriv->curDispIdx = pTgtSLPriv->curDispIdx ? 0 : 1;

	pPriSLPriv = vshmSLPriv;

	/* ignore copy when target layer is the same size as primary*/
	if ((pSrcSLPriv != pTgtSLPriv) &&
		(pTgtSLPriv->screenRect.right == pPriSLPriv->screenRect.right) &&
		(pTgtSLPriv->screenRect.left == pPriSLPriv->screenRect.left) &&
		(pTgtSLPriv->screenRect.bottom == pPriSLPriv->screenRect.bottom) &&
		(pTgtSLPriv->screenRect.top == pPriSLPriv->screenRect.top)) {
		dbg(DBG_DEBUG, "ignore copy process!\n");
		goto done;
	}

	mode = OP_NORMAL_MODE | TASK_VF_MODE;
        input.width = output.width = pPriSLPriv->screenRect.right - pPriSLPriv->screenRect.left;
        input.height = output.height = pPriSLPriv->screenRect.bottom - pPriSLPriv->screenRect.top;
        input.fmt = output.fmt = pPriSLPriv->fmt;
	if(pSrcSLPriv->isPrimary)
		input.user_def_paddr[0] = pSrcSLPriv->bufPaddr[pSrcSLPriv->curBufIdx];
	else
		input.user_def_paddr[0] = pSrcSLPriv->dispPaddr[pSrcSLPriv->curDispIdx];
	output.user_def_paddr[0] = pTgtSLPriv->dispPaddr[pTgtSLPriv->curDispIdx];

	if (mxc_ipu_lib_task_init(&input, NULL, &output, mode, &ipu_handle) < 0) {
		ret = E_RET_TASK_SETUP_ERR;
		goto done;
	}

	if (mxc_ipu_lib_task_buf_update(&ipu_handle, 0, 0, 0, 0, 0) < 0) {
		ret = E_RET_TASK_RUN_ERR;
		goto done;
	}

	mxc_ipu_lib_task_uninit(&ipu_handle);

	gettimeofday(&total_end, NULL);

        sec = total_end.tv_sec - total_begin.tv_sec;
        usec = total_end.tv_usec - total_begin.tv_usec;

        if (usec < 0) {
                sec--;
                usec = usec + 1000000;
        }
        total_time += (sec * 1000000) + usec;

        dbg(DBG_DEBUG, "Copy time %d us\n", total_time);
	dbg(DBG_DEBUG, "Copy screen layer in %d %d, from 0x%x to 0x%x\n", input.width, input.height,
			input.user_def_paddr[0], output.user_def_paddr[0]);
done:
	return ret;
}

SLRetCode _CombScreenLayers(ScreenLayerPriv *pBotSLPriv, ScreenLayerPriv *pTopSLPriv)
{
	SLRetCode ret = E_RET_SUCCESS;
	ScreenLayerPriv *pPriSLPriv;
	ipu_lib_handle_t ipu_handle;
	ipu_lib_input_param_t input;
	ipu_lib_overlay_param_t overlay;
	ipu_lib_output_param_t output;
	s32 mode;
	int total_time = 0;
	struct timeval total_begin,total_end;
	int sec, usec;

	gettimeofday(&total_begin, NULL);

	memset(&ipu_handle, 0, sizeof(ipu_lib_handle_t));
	memset(&input, 0, sizeof(ipu_lib_input_param_t));
	memset(&overlay, 0, sizeof(ipu_lib_overlay_param_t));
	memset(&output, 0, sizeof(ipu_lib_output_param_t));

	pPriSLPriv = vshmSLPriv;

	if (pTopSLPriv->alphaGlobalEnable && (pTopSLPriv->alpha == 255)
		&& !pTopSLPriv->keyColorEnable) {
		/* this case we can do copy directly, no combination needed*/
		dbg(DBG_DEBUG, "Do copy directly for Comb!\n");

		mode = OP_NORMAL_MODE | TASK_VF_MODE;

		/* top/overlay layer (graphic) */
		input.width = pTopSLPriv->screenRect.right - pTopSLPriv->screenRect.left;
		input.height = pTopSLPriv->screenRect.bottom - pTopSLPriv->screenRect.top;
		input.fmt = pTopSLPriv->fmt;
		input.user_def_paddr[0] = pTopSLPriv->bufPaddr[pTopSLPriv->curBufIdx];

		/* output */
		output.width = pPriSLPriv->screenRect.right - pPriSLPriv->screenRect.left;
		output.height = pPriSLPriv->screenRect.bottom - pPriSLPriv->screenRect.top;
		output.output_win.pos.x = pTopSLPriv->screenRect.left;
		output.output_win.pos.y = pTopSLPriv->screenRect.top;
		output.output_win.win_w = pTopSLPriv->screenRect.right - pTopSLPriv->screenRect.left;
		output.output_win.win_h = pTopSLPriv->screenRect.bottom - pTopSLPriv->screenRect.top;
		output.fmt = pPriSLPriv->fmt;
		output.user_def_paddr[0] = pTopSLPriv->dispPaddr[pTopSLPriv->curDispIdx];

		if (mxc_ipu_lib_task_init(&input, NULL, &output, mode, &ipu_handle) < 0) {
			ret = E_RET_TASK_SETUP_ERR;
			goto done;
		}
	} else {
		dbg(DBG_DEBUG, "Use IC Comb!\n");

		mode = OP_NORMAL_MODE | TASK_VF_MODE;
		/* bottom layer */
		input.width = pPriSLPriv->screenRect.right - pPriSLPriv->screenRect.left;
		input.height = pPriSLPriv->screenRect.bottom - pPriSLPriv->screenRect.top;
		input.input_crop_win.pos.x = pTopSLPriv->screenRect.left;
		input.input_crop_win.pos.y = pTopSLPriv->screenRect.top;
		input.input_crop_win.win_w = pTopSLPriv->screenRect.right - pTopSLPriv->screenRect.left;
		input.input_crop_win.win_h = pTopSLPriv->screenRect.bottom - pTopSLPriv->screenRect.top;
		input.fmt = pPriSLPriv->fmt;
		if (pBotSLPriv->isPrimary)
			input.user_def_paddr[0] = pBotSLPriv->bufPaddr[pBotSLPriv->curBufIdx];
		else
			input.user_def_paddr[0] = pBotSLPriv->dispPaddr[pBotSLPriv->curDispIdx];

		/* top/overlay layer (graphic) */
		overlay.width = pTopSLPriv->screenRect.right - pTopSLPriv->screenRect.left;
		overlay.height = pTopSLPriv->screenRect.bottom - pTopSLPriv->screenRect.top;
		overlay.fmt = pTopSLPriv->fmt;
		overlay.user_def_paddr[0] = pTopSLPriv->bufPaddr[pTopSLPriv->curBufIdx];
		overlay.global_alpha_en = pTopSLPriv->alphaGlobalEnable;
		if (pTopSLPriv->sepAlphaLocalEnable &&
		    pTopSLPriv->supportSepLocalAlpha) {
			overlay.local_alpha_en = 1;
			overlay.user_def_alpha_paddr[0] = pTopSLPriv->bufAlphaPaddr[pTopSLPriv->curBufIdx];
		}
		overlay.key_color_en = pTopSLPriv->keyColorEnable;
		overlay.alpha = pTopSLPriv->alpha;
		overlay.key_color = pTopSLPriv->keyColor;

		/* output */
		output.width = pPriSLPriv->screenRect.right - pPriSLPriv->screenRect.left;
		output.height = pPriSLPriv->screenRect.bottom - pPriSLPriv->screenRect.top;
		output.output_win.pos.x = pTopSLPriv->screenRect.left;
		output.output_win.pos.y = pTopSLPriv->screenRect.top;
		output.output_win.win_w = pTopSLPriv->screenRect.right - pTopSLPriv->screenRect.left;
		output.output_win.win_h = pTopSLPriv->screenRect.bottom - pTopSLPriv->screenRect.top;
		output.fmt = pPriSLPriv->fmt;
		output.user_def_paddr[0] = pTopSLPriv->dispPaddr[pTopSLPriv->curDispIdx];

		if (mxc_ipu_lib_task_init(&input, &overlay, &output, mode, &ipu_handle) < 0) {
			ret = E_RET_TASK_SETUP_ERR;
			goto done;
		}
	}

	if (mxc_ipu_lib_task_buf_update(&ipu_handle, 0, 0, 0, 0, 0) < 0) {
		ret = E_RET_TASK_RUN_ERR;
		goto done;
	}

	mxc_ipu_lib_task_uninit(&ipu_handle);

	gettimeofday(&total_end, NULL);

        sec = total_end.tv_sec - total_begin.tv_sec;
        usec = total_end.tv_usec - total_begin.tv_usec;

        if (usec < 0) {
                sec--;
                usec = usec + 1000000;
        }
        total_time += (sec * 1000000) + usec;

        dbg(DBG_DEBUG, "Comb time %d us\n", total_time);
	dbg(DBG_DEBUG, "Comb screen layer in [(%d,%d),(%d,%d)]\n", pTopSLPriv->screenRect.left,
		pTopSLPriv->screenRect.top, pTopSLPriv->screenRect.right, pTopSLPriv->screenRect.bottom);
done:
	return ret;
}

SLRetCode _UpdateFramebuffer(ScreenLayerPriv *pSLPriv)
{
	s32 fd_fb = 0;
	SLRetCode ret = E_RET_SUCCESS;
	ScreenLayerPriv *pPriSLPriv = vshmSLPriv;
	struct fb_var_screeninfo fb_var;
        struct fb_fix_screeninfo fb_fix;

	if ((fd_fb = open(pPriSLPriv->fbdev, O_RDWR, 0)) < 0) {
		ret = E_RET_DEV_FAIL;
		goto done;
	}

	if (ioctl(fd_fb, FBIOGET_VSCREENINFO, &fb_var) < 0) {
		ret = E_RET_DEV_FAIL;
		goto done;
	}

	if (pSLPriv->curDispIdx == 0)
		fb_var.yoffset = 0;
	else
		fb_var.yoffset = fb_var.yres;

	if (ioctl(fd_fb, FBIOPAN_DISPLAY, &fb_var) < 0) {
		ret = E_RET_DEV_FAIL;
		goto done;
	}
	dbg(DBG_DEBUG, "update fb: pan display offset %d\n", fb_var.yoffset);

	if (pSLPriv->tvMode != TVOUT_DISABLE) {
		dma_addr_t new_addr;

		if (!pSLPriv->tvEnabled) {
			copy_to_tv_init(pSLPriv);
			pSLPriv->tvEnabled = 1;
		}
		if ( ioctl(fd_fb, FBIOGET_FSCREENINFO, &fb_fix) < 0) {
			ret = E_RET_DEV_FAIL;
			goto done;
		}
		if (pSLPriv->curDispIdx == 0)
			new_addr = fb_fix.smem_start;
		else
			new_addr = fb_fix.smem_start + fb_var.yres * fb_fix.line_length;
		copy_to_tv_update(pSLPriv, new_addr);
	} else if (pSLPriv->tvEnabled) {
		copy_to_tv_deinit(pSLPriv);
		pSLPriv->tvEnabled = 0;
	}

done:
	if (fd_fb)
		close(fd_fb);

	return ret;
}

SLRetCode UpdateScreenLayer(ScreenLayer *pSL)
{
	SLRetCode ret = E_RET_SUCCESS;
	ScreenLayerPriv *pSLPriv = (ScreenLayerPriv *)(vshmSLPriv + (int)pSL->pPriv-1);
	ScreenLayerPriv *pCurSLPriv;

	sem_wait(semMainID);
	sem_wait(semDispID);
	/* primary update? */
	if (pSLPriv->isPrimary && !pSLPriv->nextLayerId) {
		/* there is only primary, update it only */
		dbg(DBG_DEBUG, "Update primary layer only, just copy!\n");
		pCurSLPriv = pSLPriv;
		ret = _CopyScreenLayer(pCurSLPriv, pCurSLPriv);
		if (ret != E_RET_SUCCESS)
			goto done;
	} else {
		/* update from primary to top SL*/
		dbg(DBG_DEBUG, "Update multi layers, from primary to top!\n");

		pCurSLPriv = vshmSLPriv;
		while(pCurSLPriv->nextLayerId) {
			ScreenLayerPriv *pNextSLPriv = (ScreenLayerPriv *)(vshmSLPriv + pCurSLPriv->nextLayerId-1);

			ret = _CopyScreenLayer(pCurSLPriv, pNextSLPriv);
			if (ret != E_RET_SUCCESS)
				goto done;

			ret = _CombScreenLayers(pCurSLPriv, pNextSLPriv);
			if (ret != E_RET_SUCCESS)
				goto done;
			pCurSLPriv = pNextSLPriv;
		}
	}

	ret = _UpdateFramebuffer(pCurSLPriv);
done:
	sem_post(semDispID);
	sem_post(semMainID);
	return ret;
}

SLRetCode SetScreenLayer(ScreenLayer *pSL, SetMethodType eType, void *setData)
{
	SLRetCode ret = E_RET_SUCCESS;
	ScreenLayerPriv *pSLPriv = (ScreenLayerPriv *)(vshmSLPriv + (int)pSL->pPriv-1);

	sem_wait(semMainID);
	switch (eType) {
	case E_SET_ALPHA:
	{
		MethodAlphaData *data = (MethodAlphaData *)setData;
		if (data->sepLocalAlphaEnable && data->globalAlphaEnable) {
			dbg(DBG_ERR, "global/local alpha blending confliction!\n");
			ret = E_RET_ALPHA_BLENDING_CONFLICT;
			goto err;
		}
		pSLPriv->alphaGlobalEnable = data->globalAlphaEnable;
		pSLPriv->sepAlphaLocalEnable = data->sepLocalAlphaEnable;
		pSLPriv->alpha = data->alpha;
		break;
	}
	case E_SET_COLORKEY:
	{
		MethodColorKeyData *data = (MethodColorKeyData *)setData;
		pSLPriv->keyColorEnable = data->enable;
		pSLPriv->keyColor = data->keyColor;
		break;
	}
	case E_ENABLE_LAYER:
		pSLPriv->layerEnable = *((u8 *)setData);
		break;
	case E_COPY_TVOUT:
	{
		MethodTvoutData *data = (MethodTvoutData *)setData;
		pSLPriv->tvMode = data->tvMode;
		pSLPriv->tvRotation = data->lcd2tvRotation;
		break;
	}
	default:
		ret = E_RET_NOSUCH_METHODTYPE;
	}
err:
	sem_post(semMainID);
	return ret;
}

/*
** Get the handle of Primaray screen layer, which will be used to create the others Non-primary screen layer.
**
** Input  : fbdev, this is the fixed id of frame buffer
** Return : The handle of the Primary Screen Layer
*/
void* GetPrimarySLHandle(char * pFbdev)
{
	SLRetCode ret = E_RET_SUCCESS;

	if(vshmSLPriv == NULL)
		ret = PreScreenLayerIPC(pFbdev);
	if(ret != E_RET_SUCCESS || vshmSLPriv == NULL)
	{
		dbg(DBG_ERR, "Prepared semaphore & shm failed !\n");
		return (void*)0;
	}

	sem_wait(semMainID);
	if(vshmSLPriv->isPrimary)
	{
		dbg(DBG_DEBUG, "GetPrimarySLHandle is OK!\n");
		sem_post(semMainID);
		return	(void *)1;
	}
	else
	{
		dbg(DBG_ERR, "GetPrimarySLHandle Error!\n");
		sem_post(semMainID);
		return  (void *)0;
	}
}

/*
** Get the width of Primary screen layer.
**
** Input  : pPrimaryHandle, this is the handle of primary screen layer
** Return : the width of Primary screen layer
*/
u32   GetPrimarySLWidth(void * pPrimaryHandle)
{
	return (vshmSLPriv->screenRect.left - vshmSLPriv->screenRect.right);
}

/*
** Get the height of Primary screen layer.
**
** Input  : pPrimaryHandle, this is the handle of primary screen layer
** Return : the height of Primary screen layer
*/
u32   GetPrimarySLHeight(void * pPrimaryHandle)
{
	return (vshmSLPriv->screenRect.bottom - vshmSLPriv->screenRect.top);
}
#ifdef __cplusplus
}
#endif

