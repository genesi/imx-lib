/*
 * pxp_lib - a user space library for PxP
 *
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "pxp_lib.h"

#define	PXP_DEVICE_NAME	"/dev/pxp_device"

#define DBG_DEBUG		3
#define DBG_INFO		2
#define DBG_WARNING		1
#define DBG_ERR			0

static int debug_level = DBG_ERR;
#define dbg(flag, fmt, args...)	{ if(flag <= debug_level)  printf("%s:%d "fmt, __FILE__, __LINE__,##args); }

static int fd = -1;
static int active_open_nr;

int pxp_init(void)
{
	if (fd > 0) {
		active_open_nr++;
		return 0;
	}

	if (fd < 0) {
		fd = open(PXP_DEVICE_NAME, O_RDWR, 0);
		if (fd < 0) {
			dbg(DBG_ERR, "open file error.\n");
			return -1;
		}
	}

	active_open_nr++;
	return 0;
}

int pxp_request_channel(pxp_chan_handle_t *pxp_chan)
{
	int cid;
	int ret = 0;

	ret = ioctl(fd, PXP_IOC_GET_CHAN, &cid);
	if (ret < 0)
		return -1;

	pxp_chan->chan_id = cid;
	return 0;
}

void pxp_release_channel(pxp_chan_handle_t *pxp_chan)
{
	ioctl(fd, PXP_IOC_PUT_CHAN, &(pxp_chan->chan_id));
}

int pxp_config_channel(pxp_chan_handle_t *pxp_chan, struct pxp_config_data *pxp_conf)
{
	int ret = 0;

	pxp_conf->chan_id = pxp_chan->chan_id;
	dbg(DBG_INFO, "chan_id %d\n\n", pxp_conf->chan_id);

	ret = ioctl(fd, PXP_IOC_CONFIG_CHAN, pxp_conf);

	return ret;
}

int pxp_start_channel(pxp_chan_handle_t *pxp_chan)
{
	int ret = 0;

	ret = ioctl(fd, PXP_IOC_START_CHAN, &(pxp_chan->chan_id));

	return ret;
}

int pxp_wait_for_completion(pxp_chan_handle_t *pxp_chan, int times)
{
	int _times = 0;
	int ret = 0;

	while ((ret = ioctl(fd, PXP_IOC_WAIT4CMPLT, pxp_chan)) < 0) {
		_times++;
		if (_times >= times) {
			return ret;
		}
	}

	return ret;
}

void pxp_uninit(void)
{
	if (fd == -1)
		return;

	if (active_open_nr > 1) {
		active_open_nr--;
	} else if (active_open_nr == 1) {
		active_open_nr--;
		if (fd > 0) {
			close(fd);
			fd = -1;
		}
	}
}

/*
 * Below are helper functions to alloc/free the phy/virt buffer
 */
int pxp_get_phymem(struct pxp_mem_desc *mem)
{
	if (ioctl(fd, PXP_IOC_GET_PHYMEM, mem) < 0) {
		mem->phys_addr = 0;
		mem->cpu_addr = 0;
		dbg(DBG_ERR, "PXP_IOC_GET_PHYMEM err\n");
		return -1;
	}

	dbg(DBG_INFO, "succeed mem.phys_addr = 0x%08x\n", mem->phys_addr);
	return 0;
}

int pxp_put_phymem(struct pxp_mem_desc *mem)
{
	if (ioctl(fd, PXP_IOC_PUT_PHYMEM, mem) < 0) {
		mem->phys_addr = 0;
		mem->cpu_addr = 0;
		dbg(DBG_ERR, "PXP_IOC_PUT_PHYMEM err\n");
		return -1;
	}

	dbg(DBG_DEBUG, "succeed\n");
	return 0;
}

int pxp_get_virtmem(struct pxp_mem_desc *mem)
{
	void *va_addr;

	va_addr = mmap(NULL, mem->size, PROT_READ | PROT_WRITE,
				      MAP_SHARED, fd, mem->phys_addr);

	if (va_addr == MAP_FAILED) {
		mem->virt_uaddr = 0;
		dbg(DBG_ERR, "MAP_FAILED.\n");
		return -1;
	}

	mem->virt_uaddr = (unsigned long)va_addr;
	dbg(DBG_INFO, "virt addr = %08x\n", mem->virt_uaddr);

	return 0;
}

int pxp_put_virtmem(struct pxp_mem_desc *mem)
{
	if (mem->virt_uaddr != 0) {
		if (munmap((void *)mem->virt_uaddr, mem->size) != 0)
			dbg(DBG_ERR, "UNMAP_FAILED.\n");
	}

	mem->virt_uaddr = 0;
	return 0;
}

int pxp_get_mem(struct pxp_mem_desc *mem)
{
	int ret;

	ret = pxp_get_phymem(mem);
	if (ret < 0)
		return ret;

	ret = pxp_get_virtmem(mem);
	if (ret < 0)
		return ret;

	return 0;
}
int pxp_put_mem(struct pxp_mem_desc *mem)
{
	int ret;

	ret = pxp_put_virtmem(mem);
	if (ret < 0)
		return ret;

	ret = pxp_put_phymem(mem);
	if (ret < 0)
		return ret;

	return 0;
}
