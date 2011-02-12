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


/**
 * @file fsl_shw_sym.c
 *
 * This file implements Symmetric Cipher functions of the FSL SHW API.  This
 * does not include CCM.
 */

#include "shw_driver.h"


/* REQ-S2LRD-PINTFC-API-BASIC-SYM-002 */
/* PINTFC-API-BASIC-SYM-ARC4-001 */
/* PINTFC-API-BASIC-SYM-ARC4-002 */

/**
 * Compute symmetric encryption
 *
 *
 * @param    user_ctx
 * @param    key_info
 * @param    sym_ctx
 * @param    length
 * @param    pt
 * @param    ct
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_symmetric_encrypt(
                                fsl_shw_uco_t* user_ctx,
                                fsl_shw_sko_t* key_info,
                                fsl_shw_scco_t* sym_ctx,
                                uint32_t length,
                                const uint8_t* pt,
                                uint8_t* ct)
{
    fsl_shw_return_t ret = FSL_RETURN_ERROR_S;


    /* Unused */
    (void)user_ctx;
    (void)key_info;
    (void)sym_ctx;
    (void)length;
    (void)ct;
    (void)pt;

    return ret;
}


/* PINTFC-API-BASIC-SYM-002 */
/* PINTFC-API-BASIC-SYM-ARC4-001 */
/* PINTFC-API-BASIC-SYM-ARC4-002 */

/**
 * Compute symmetric decryption
 *
 *
 * @param    user_ctx
 * @param    key_info
 * @param    sym_ctx
 * @param    length
 * @param    pt
 * @param    ct
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_symmetric_decrypt(
                                fsl_shw_uco_t* user_ctx,
                                fsl_shw_sko_t* key_info,
                                fsl_shw_scco_t* sym_ctx,
                                uint32_t length,
                                const uint8_t* ct,
                                uint8_t* pt)
{
    fsl_shw_return_t ret = FSL_RETURN_ERROR_S;



    /* Unused */
    (void)user_ctx;
    (void)key_info;
    (void)sym_ctx;
    (void)length;
    (void)ct;
    (void)pt;

    return ret;
}

