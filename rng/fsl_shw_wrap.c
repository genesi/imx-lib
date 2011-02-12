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


/*
 * User Space library to access the Security hardware
 */

/**
 * @file fsl_shw_wrap.c
 *
 * This file implements Key-Wrap (Black Key) functions of the FSL SHW API.
 *
 * <ul>
 * <li> Ownerid is an 8-byte, user-supplied, value to keep KEY
 *      confidential.</li>
 * <li> KEY is a 1-32 byte value which starts in SCC RED RAM before
 *     wrapping, and ends up there on unwrap.  Length is limited because of
 *    size of SCC1 RAM.</li>
 * <li> KEY' is the encrypted KEY</li>
 * <li> LEN is a 1-byte (for now) byte-length of KEY</li>
 * <li> ALG is a 1-byte value for the algorithm which which the key is
 *    associated.  Values are defined by the FSL SHW API</li>
 * <li> Ownerid, LEN, and ALG come from the user's "key_info" object, as does
 *    the slot number where KEY already is/will be.</li>
 * <li> T is a Nonce</li>
 * <li> T' is the encrypted T</li>
 * <li> KEK is a Key-Encryption Key for the user's Key</li>
 * <li> ICV is the "Integrity Check Value" for the wrapped key</li>
 * <li> Black Key is the string of bytes returned as the wrapped key</li>
 * </ul>
<table border="0">
<tr><TD align="right">BLACK_KEY <TD width="3">=<TD>ICV | T' | LEN | ALG |
     KEY'</td></tr>
<tr><td>&nbsp;</td></tr>

<tr><th>To Wrap</th></tr>
<tr><TD align="right">T</td> <TD width="3">=</td> <TD>RND()<sub>16</sub>
    </td></tr>
<tr><TD align="right">KEK</td><TD width="3">=</td><TD>HASH<sub>sha1</sub>(T |
     Ownerid)<sub>16</sub></td></tr>
<tr><TD align="right">KEY'<TD width="3">=</td><TD>
     AES<sub>ctr-enc</sub>(Key=KEK, CTR=0, Data=KEY)</td></tr>
<tr><TD align="right">ICV</td><TD width="3">=</td><td>HMAC<sub>sha1</sub>
     (Key=T, Data=Ownerid | LEN | ALG | KEY')<sub>16</sub></td></tr>
<tr><TD align="right">T'</td><TD width="3">=</td><TD>TDES<sub>cbc-enc</sub>
     (Key=SLID, IV=Ownerid, Data=T)</td></tr>

<tr><td>&nbsp;</td></tr>

<tr><th>To Unwrap</th></tr>
<tr><TD align="right">T</td><TD width="3">=</td><TD>TDES<sub>ecb-dec</sub>
    (Key=SLID, IV=Ownerid, Data=T')</td></tr>
<tr><TD align="right">ICV</td><TD width="3">=</td><td>HMAC<sub>sha1</sub>
    (Key=T, Data=Ownerid | LEN | ALG | KEY')<sub>16</sub></td></tr>
<tr><TD align="right">KEK</td><TD width="3">=</td><td>HASH<sub>sha1</sub>
    (T | Ownerid)<sub>16</sub></td></tr>
<tr><TD align="right">KEY<TD width="3">=</td><TD>AES<sub>ctr-dec</sub>
    (Key=KEK, CTR=0, Data=KEY')</td></tr>
</table>

 */

#include "shw_driver.h"


#define ICV_LENGTH 16
#define T_LENGTH 16
#define KEK_LENGTH 16
#define LENGTH_LENGTH 1
#define ALGORITHM_LENGTH 1

/* ICV | T' | LEN | ALG | KEY' */
#define ICV_OFFSET       0
#define T_PRIME_OFFSET   (ICV_OFFSET + ICV_LENGTH)
#define LENGTH_OFFSET    (T_PRIME_OFFSET + T_LENGTH)
#define ALGORITHM_OFFSET (LENGTH_OFFSET + LENGTH_LENGTH)
#define KEY_PRIME_OFFSET (ALGORITHM_OFFSET + ALGORITHM_LENGTH)


/**
 * Place a key into a protected location for use only by cryptographic
 * algorithms.
 *
 * This only needs to be used to a) unwrap a key, or b) set up a key which
 * could be wrapped with a later call to #fsl_shw_extract_key().  Normal
 * cleartext keys can simply be placed into #fsl_shw_sko_t key objects with
 * #fsl_shw_sko_set_key() and used directly.
 *
 * The maximum key size supported for wrapped/unwrapped keys is 32 octets.
 * (This is the maximum reasonable key length on Sahara - 32 octets for an HMAC
 * key based on SHA-256.)  The key size is determined by the @a key_info.  The
 * expected length of @a key can be determined by
 * #fsl_shw_sko_calculate_wrapped_size()
 *
 * The protected key will not be available for use until this operation
 * successfully completes.
 *
 * This feature is not available for all platforms, nor for all algorithms and
 * modes.
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 * @param[in,out] key_info      The information about the key to be which will
 *                              be established.  In the create case, the key
 *                              length must be set.
 * @param      establish_type   How @a key will be interpreted to establish a
 *                              key for use.
 * @param key                   If @a establish_type is #FSL_KEY_WRAP_UNWRAP,
 *                              this is the location of a wrapped key.  If
 *                              @a establish_type is #FSL_KEY_WRAP_CREATE, this
 *                              parameter can be @a NULL.  If @a establish_type
 *                              is #FSL_KEY_WRAP_ACCEPT, this is the location
 *                              of a plaintext key.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_establish_key(
                                       fsl_shw_uco_t* user_ctx,
                                       fsl_shw_sko_t* key_info,
                                       fsl_shw_key_wrap_t establish_type,
                                       const uint8_t* key)
{
    fsl_shw_return_t ret = FSL_RETURN_ERROR_S;

    /* Unused */
    (void)user_ctx;
    (void)key_info;
    (void)establish_type;
    (void)key;

    return ret;
}


/**
 * Wrap a key and retrieve the wrapped value.
 *
 * A wrapped key is a key that has been cryptographically obscured.  It is
 * only able to be used with #fsl_shw_establish_key().
 *
 * This function will also release the key (see #fsl_shw_release_key()) so
 * that it must be re-established before reuse.
 *
 * This feature is not available for all platforms, nor for all algorithms and
 * modes.
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 * @param      key_info         The information about the key to be deleted.
 * @param[out] covered_key      The location to store the 48-octet wrapped key.
 *                              (This size is based upon the maximum key size
 *                              of 32 octets).
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_extract_key(fsl_shw_uco_t* user_ctx,
                                     fsl_shw_sko_t* key_info,
                                     uint8_t* covered_key)
{
    fsl_shw_return_t ret = FSL_RETURN_ERROR_S;


    /* Unused */
    (void)user_ctx;
    (void)key_info;
    (void)covered_key;

    return ret;
}


/**
 * De-establish a key so that it can no longer be accessed.
 *
 * The key will need to be re-established before it can again be used.
 *
 * This feature is not available for all platforms, nor for all algorithms and
 * modes.
 *
 * @param      user_ctx         A user context from #fsl_shw_register_user().
 * @param      key_info         The information about the key to be deleted.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_release_key(
                             fsl_shw_uco_t* user_ctx,
                             fsl_shw_sko_t* key_info)
{
    fsl_shw_return_t ret = FSL_RETURN_ERROR_S;

    /* Unused */
    (void)user_ctx;
    (void)key_info;

    return ret;
}
