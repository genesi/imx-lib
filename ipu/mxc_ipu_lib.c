/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

/*!
 * @file mxc_ipu_lib_basic.c
 *
 * @brief IPU basic library implementation
 *
 * @ingroup IPU
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <malloc.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/ipu.h>

int fd_ipu = -1;
static int open_count = 0;

int32_t ipu_init_channel(ipu_channel_t channel, ipu_channel_params_t * params)
{
	ipu_channel_parm parm;
	parm.channel = channel;
	if(params != NULL){
		parm.params = *params;
		parm.flag = 0;
	}else{
		parm.flag = 1;
	}
	return ioctl(fd_ipu, IPU_INIT_CHANNEL, &parm);
}
void ipu_uninit_channel(ipu_channel_t channel)
{
	ioctl(fd_ipu, IPU_UNINIT_CHANNEL, &channel);
}

int32_t ipu_init_channel_buffer(ipu_channel_t channel, ipu_buffer_t type,
		uint32_t pixel_fmt,
		uint16_t width, uint16_t height,
		uint32_t stride,
		ipu_rotate_mode_t rot_mode,
		dma_addr_t phyaddr_0, dma_addr_t phyaddr_1,
		uint32_t u_offset, uint32_t v_offset)
{
	ipu_channel_buf_parm buf_parm;
	buf_parm.channel = channel;
	buf_parm.type = type;
	buf_parm.pixel_fmt = pixel_fmt;
	buf_parm.width = width;
	buf_parm.height = height;
	buf_parm.stride = stride;
	buf_parm.rot_mode = rot_mode;
	buf_parm.phyaddr_0 = phyaddr_0;
	buf_parm.phyaddr_1 = phyaddr_1;
	buf_parm.u_offset = u_offset;
	buf_parm.v_offset = v_offset;
	return ioctl(fd_ipu, IPU_INIT_CHANNEL_BUFFER, &buf_parm);
}

int32_t ipu_update_channel_buffer(ipu_channel_t channel, ipu_buffer_t type,
		uint32_t bufNum, dma_addr_t phyaddr)
{
	ipu_channel_buf_parm buf_parm;
	buf_parm.channel = channel;
	buf_parm.type = type;
	buf_parm.bufNum = bufNum;
	if(bufNum == 0){
		buf_parm.phyaddr_0 = phyaddr;
		buf_parm.phyaddr_1 = (dma_addr_t)NULL;
	}
	else{
		buf_parm.phyaddr_1 = phyaddr;
		buf_parm.phyaddr_0 = (dma_addr_t)NULL;
	}
	return ioctl(fd_ipu, IPU_UPDATE_CHANNEL_BUFFER, &buf_parm);

}

int32_t ipu_select_buffer(ipu_channel_t channel,
		ipu_buffer_t type, uint32_t bufNum)
{
	ipu_channel_buf_parm buf_parm;
	buf_parm.channel = channel;
	buf_parm.type = type;
	buf_parm.bufNum = bufNum;
	return ioctl(fd_ipu, IPU_SELECT_CHANNEL_BUFFER, &buf_parm);
}

int32_t ipu_select_multi_vdi_buffer(uint32_t bufNum)
{
	return ioctl(fd_ipu, IPU_SELECT_MULTI_VDI_BUFFER, &bufNum);
}

int32_t ipu_link_channels(ipu_channel_t src_ch, ipu_channel_t dest_ch)
{
	ipu_channel_link link;
	link.src_ch = src_ch;
	link.dest_ch = dest_ch;
	return ioctl(fd_ipu, IPU_LINK_CHANNELS, &link);
}
int32_t ipu_unlink_channels(ipu_channel_t src_ch, ipu_channel_t dest_ch)
{
	ipu_channel_link link;
	link.src_ch = src_ch;
	link.dest_ch = dest_ch;
	return ioctl(fd_ipu, IPU_UNLINK_CHANNELS, &link);
}

int32_t ipu_enable_channel(ipu_channel_t channel)
{
	return ioctl(fd_ipu, IPU_ENABLE_CHANNEL, &channel);
}
int32_t ipu_disable_channel(ipu_channel_t channel, bool wait_for_stop)
{
	ipu_channel_info c_info;
	c_info.channel = channel;
	c_info.stop = wait_for_stop;
	return ioctl(fd_ipu, IPU_DISABLE_CHANNEL, &c_info);
}

void ipu_enable_irq(uint32_t irq)
{
	ioctl(fd_ipu, IPU_ENABLE_IRQ, &irq);
}
void ipu_disable_irq(uint32_t irq)
{
	ioctl(fd_ipu, IPU_DISABLE_IRQ, &irq);
}
void ipu_clear_irq(uint32_t irq)
{
	ioctl(fd_ipu, IPU_CLEAR_IRQ, &irq);
}

void ipu_free_irq(uint32_t irq, void *dev_id)
{
	ipu_irq_info irq_info;
	irq_info.irq = irq;
	irq_info.dev_id = dev_id;
	ioctl(fd_ipu, IPU_FREE_IRQ, &irq_info);
}

bool ipu_get_irq_status(uint32_t irq)
{
	return ioctl(fd_ipu, IPU_REQUEST_IRQ_STATUS, &irq);
}

uint32_t bytes_per_pixel(uint32_t fmt)
{
	switch (fmt) {
		case IPU_PIX_FMT_GENERIC:       /*generic data */
		case IPU_PIX_FMT_RGB332:
		case IPU_PIX_FMT_YUV420P:
		case IPU_PIX_FMT_YVU420P:
		case IPU_PIX_FMT_YUV420P2:
		case IPU_PIX_FMT_YUV422P:
		case IPU_PIX_FMT_YVU422P:
		case IPU_PIX_FMT_NV12:
			return 1;
			break;
		case IPU_PIX_FMT_RGB565:
		case IPU_PIX_FMT_YUYV:
		case IPU_PIX_FMT_UYVY:
			return 2;
			break;
		case IPU_PIX_FMT_BGR24:
		case IPU_PIX_FMT_RGB24:
		case IPU_PIX_FMT_YUV444:
			return 3;
			break;
		case IPU_PIX_FMT_GENERIC_32:    /*generic data */
		case IPU_PIX_FMT_BGR32:
		case IPU_PIX_FMT_RGB32:
		case IPU_PIX_FMT_RGBA32:
		case IPU_PIX_FMT_BGRA32:
		case IPU_PIX_FMT_ABGR32:
			return 4;
			break;
		default:
			return 1;
			break;
	}
	return 0;
}

/* Extra IPU-lib functions that are required */

int ipu_open()
{
	if (open_count == 0) {
		fd_ipu = open("/dev/mxc_ipu",O_RDWR);
		if(fd_ipu < 0) {
			printf("Unable to Open IPU device\n");
			return fd_ipu;
		}
	}
	open_count++;

	return fd_ipu;
}

void ipu_close()
{
	if (open_count == 0)
		return;
	else {
		open_count--;
		if (open_count == 0) {
			close(fd_ipu);
			fd_ipu = -1;
		}
	}
}

int ipu_register_generic_isr(int irq, void *dev)
{
	ipu_event_info ev; /* Defined in the ipu.h header file */
	ev.irq = irq;
	ev.dev = dev;
	return ioctl(fd_ipu,IPU_REGISTER_GENERIC_ISR,&ev);
}

int ipu_get_interrupt_event(ipu_event_info *ev)
{
	return ioctl(fd_ipu,IPU_GET_EVENT,ev);
}

int ipu_is_channel_busy(ipu_channel_t chan)
{
	return ioctl(fd_ipu,IPU_IS_CHAN_BUSY,&chan);
}

int ipu_calc_stripes_sizes(const unsigned int input_frame_width, /* input frame width;>1 */
                           unsigned int output_frame_width, /* output frame width; >1 */
                           const unsigned int maximal_stripe_width, /* the maximal width allowed for a stripe */
                           const unsigned long long cirr, /* see above */
                           const unsigned int equal_stripes, /* see above */
                           u32 input_pixelformat,/* pixel format after of read channel*/
                           u32 output_pixelformat,/* pixel format after of write channel*/
                           struct stripe_param *left,
                           struct stripe_param *right)
{
	ipu_stripe_parm stripe_parm;
	int ret;

	stripe_parm.input_width = input_frame_width;
	stripe_parm.output_width = output_frame_width;
	stripe_parm.maximal_stripe_width = maximal_stripe_width;
	stripe_parm.cirr = cirr;
	stripe_parm.equal_stripes = equal_stripes;
	stripe_parm.input_pixelformat = input_pixelformat;
	stripe_parm.output_pixelformat = output_pixelformat;

	ret = ioctl(fd_ipu, IPU_CALC_STRIPES_SIZE, &stripe_parm);
	if (ret == 0) {
		memcpy(left, &stripe_parm.left, sizeof(stripe_parm.left));
		memcpy(right, &stripe_parm.right, sizeof(stripe_parm.right));
	}
	return ret;
}

int ipu_update_channel_offset(ipu_channel_t channel, ipu_buffer_t type,
                                uint32_t pixel_fmt,
                                uint16_t width, uint16_t height,
                                uint32_t stride,
                                uint32_t u, uint32_t v,
                                uint32_t vertical_offset, uint32_t horizontal_offset)
{
	ipu_buf_offset_parm offset_parm;
	int ret;

	offset_parm.channel = channel;
	offset_parm.type = type;
	offset_parm.pixel_fmt = pixel_fmt;
	offset_parm.width = width;
	offset_parm.height = height;
	offset_parm.stride = stride;
	offset_parm.u_offset = u;
	offset_parm.v_offset = v;
	offset_parm.vertical_offset = vertical_offset;
	offset_parm.horizontal_offset = horizontal_offset;

	ret = ioctl(fd_ipu, IPU_UPDATE_BUF_OFFSET, &offset_parm);
	return ret;
}

int ipu_update_dp_csc(int **param)
{
	ipu_csc_update csc;
	csc.channel = MEM_FG_SYNC;
	csc.param = param;

	return ioctl(fd_ipu, IPU_CSC_UPDATE, &csc);
}
