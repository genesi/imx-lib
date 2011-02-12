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
 * @file fsl_shw_auth.c
 *
 * This file contains the routines which do the combined encrypt+authentication
 * functions.  For now, only AES-CCM is supported.
 */

#include "shw_driver.h"


/**
 * Compute the size, in bytes, of the encoded auth length
 *
 * @param l    The actual auth length
 *
 * @return The encoded length
 */
#define COMPUTE_NIST_AUTH_LEN_SIZE(l)                                         \
({                                                                            \
    unsigned val;                                                             \
    if ((uint32_t)(l) < 65280) {                                              \
        val = 2;                                                              \
    } else {                    /* cannot handle >= 2^32 */                   \
        val = 6;                                                              \
    }                                                                         \
    val;                                                                      \
})

/**
 * Store the encoded Auth Length into the Auth Data
 *
 * @param l    The actual Auth Length
 * @param p    Location to store encoding (must be uint8_t*)
 *
 * @return void
 */
#define STORE_NIST_AUTH_LEN(l, p)                                             \
{                                                                             \
    register uint32_t L = l;                                                  \
    if ((uint32_t)(l) < 65280) {                                              \
        (p)[1] = L & 0xff;                                                    \
        L >>= 8;                                                              \
        (p)[0] = L & 0xff;                                                    \
    } else {                    /* cannot handle >= 2^32 */                   \
        int i;                                                                \
        for (i = 5; i > 1; i--) {                                             \
            (p)[i] = L & 0xff;                                                \
            L >>= 8;                                                          \
        }                                                                     \
        (p)[1] = 0xfe;  /* Markers */                                         \
        (p)[0] = 0xff;                                                        \
    }                                                                         \
}


/**
 * @brief Generate a (CCM) auth code and encrypt the payload.
 *
 *
 * @param user_ctx         The user's context
 * @param auth_ctx         Info on this Auth operation
 * @param cipher_key_info  Key to encrypt payload
 * @param auth_key_info    (unused - same key in CCM)
 * @param auth_data_length Length in bytes of @a auth_data
 * @param auth_data        Any auth-only data
 * @param payload_length   Length in bytes of @a payload
 * @param payload          The data to encrypt
 * @param[out] ct          The location to store encrypted data
 * @param[out] auth_value  The location to store authentication code
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_gen_encrypt(
                                fsl_shw_uco_t* user_ctx,
                                fsl_shw_acco_t* auth_ctx,
                                fsl_shw_sko_t* cipher_key_info,
                                fsl_shw_sko_t* auth_key_info,
                                uint32_t auth_data_length,
                                const uint8_t* auth_data,
                                uint32_t payload_length,
                                const uint8_t* payload,
                                uint8_t* ct,
                                uint8_t* auth_value)
{
    volatile fsl_shw_return_t status = FSL_RETURN_ERROR_S;


    /* Unused */
    (void)user_ctx;
    (void)auth_ctx;
    (void)cipher_key_info;
    (void)auth_key_info;
    (void)auth_data_length;
    (void)auth_data;
    (void)payload_length;
    (void)payload;
    (void)ct;
    (void)auth_value;

    return status;
}


/**
 * @brief Authenticate and decrypt a (CCM) stream.
 *
 * @param user_ctx         The user's context
 * @param auth_ctx         Info on this Auth operation
 * @param cipher_key_info  Key to encrypt payload
 * @param auth_key_info    (unused - same key in CCM)
 * @param auth_data_length Length in bytes of @a auth_data
 * @param auth_data        Any auth-only data
 * @param payload_length   Length in bytes of @a payload
 * @param ct               The encrypted data
 * @param auth_value       The authentication code to validate
 * @param[out] payload     The location to store decrypted data
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_auth_decrypt(
                                fsl_shw_uco_t* user_ctx,
                                fsl_shw_acco_t* auth_ctx,
                                fsl_shw_sko_t* cipher_key_info,
                                fsl_shw_sko_t* auth_key_info,
                                uint32_t auth_data_length,
                                const uint8_t* auth_data,
                                uint32_t payload_length,
                                const uint8_t* ct,
                                const uint8_t* auth_value,
                                uint8_t* payload)
{
    volatile fsl_shw_return_t status = FSL_RETURN_ERROR_S;


    /* Unused */
    (void)user_ctx;
    (void)auth_ctx;
    (void)cipher_key_info;
    (void)auth_key_info;
    (void)auth_data_length;
    (void)auth_data;
    (void)payload_length;
    (void)ct;
    (void)auth_value;
    (void)payload;

    return status;
}
