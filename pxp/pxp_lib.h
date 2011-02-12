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

#ifndef	PXP_LIB_H
#define	PXP_LIB_H

#include <linux/pxp_dma.h>

#ifndef true
#define true    1
#endif
#ifndef false
#define false   0
#endif

typedef struct pxp_chan_handle {
	int chan_id;
	int hist_status;
} pxp_chan_handle_t;

int pxp_init();
void pxp_uninit();
int pxp_request_channel(pxp_chan_handle_t *pxp_chan);
void pxp_release_channel(pxp_chan_handle_t *pxp_chan);
int pxp_config_channel(pxp_chan_handle_t *pxp_chan, struct pxp_config_data *pxp_conf);
int pxp_start_channel(pxp_chan_handle_t *pxp_chan);
int pxp_wait_for_completion(pxp_chan_handle_t *pxp_chan, int times);
int pxp_get_mem(struct pxp_mem_desc *mem);
int pxp_put_mem(struct pxp_mem_desc *mem);

#endif
