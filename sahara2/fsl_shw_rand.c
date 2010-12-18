/*
 * User Space library to access the Security hardware
 * Copyright 2005-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This library is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of the
 * License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301
 * USA
 */

/*!
 * @file lib/sahara2/fsl_shw_rand.c
 *
 * This file implements Random Number Generation functions of the FSL SHW API
 * for Sahara.
 */

#include "sahara.h"
#include "sf_util.h"

#ifdef __KERNEL__
EXPORT_SYMBOL(fsl_shw_get_random);
#endif

/* REQ-S2LRD-PINTFC-API-BASIC-RNG-002 */
/*!
 * Get a random number
 *
 *
 * @param    user_ctx
 * @param    length
 * @param    data
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_get_random(fsl_shw_uco_t * user_ctx,
				    uint32_t length, uint8_t * data)
{
	SAH_SF_DCLS;

	/* perform a sanity check on the uco */
	ret = sah_validate_uco(user_ctx);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}

	header = SAH_HDR_RNG_GENERATE;	/* Desc. #18 */
	DESC_OUT_OUT(header, length, data, 0, NULL);

	SAH_SF_EXECUTE();

      out:
	SAH_SF_DESC_CLEAN();

	return ret;
}

#ifdef __KERNEL__
EXPORT_SYMBOL(fsl_shw_add_entropy);
#endif

/*!
 * Add entropy to a random number generator

 * @param    user_ctx
 * @param    length
 * @param    data
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_add_entropy(fsl_shw_uco_t * user_ctx,
				     uint32_t length, uint8_t * data)
{
	SAH_SF_DCLS;

	/* perform a sanity check on the uco */
	ret = sah_validate_uco(user_ctx);
	if (ret != FSL_RETURN_OK_S) {
		goto out;
	}

	header = SAH_HDR_RNG_GENERATE;	/* Desc. #18 */

	/* create descriptor #18. Generate random data */
	DESC_IN_IN(header, 0, NULL, length, data)

	    SAH_SF_EXECUTE();

      out:
	SAH_SF_DESC_CLEAN();

	return ret;
}
