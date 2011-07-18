/*
 * User Space library to access the Security hardware
 * Copyright 2008-2009, 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
* @file lib/sahara2/um_adaptor.c
*
* @brief The Adaptor component provides a user-mode interface to the device
* driver.
*/

#include <sf_util.h>

#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <stdio.h>

#include <linux/mxc_sahara.h>
#include <sah_kernel.h>
#include <adaptor.h>

#include <memory.h>
#include <fcntl.h>

#ifdef DIAG_ADAPTOR
#include <diagnostic.h>
#endif				/* DIAG_ADAPTOR */

/* Most of the Linux IO functions return -1 on error */
#define IO_ERROR    -1

#ifdef DIAG_ADAPTOR
void um_Dump_Chain(const sah_Desc * chain);

void um_Dump_Region(const char *prefix, const unsigned char *data,
		    unsigned length);

static void um_Dump_Link(const char *prefix, const sah_Link * link);

void um_Dump_Words(const char *prefix, const unsigned *data, unsigned length);

/*#undef LOG_DIAG

#define LOG_DIAG(x) printf("%s\n", x)*/

#ifndef MAX_DUMP
#define MAX_DUMP 64
#endif

/* This static error message buffer is likely not thread-safe */
static char Diag_msg[200];
#endif

/*!
 * Chain of active user contexts for this process.
 */
static fsl_shw_uco_t *user_chain = NULL;

/*!
 * Flag for whether callback handling has been set up for this process.
 */
static int callback_initialized = 0;

/**** memory routines ****/

static void *my_malloc(void *ref, size_t n)
{
	void *mem;

#ifndef DIAG_MEM_ERRORS
	mem = malloc(n);
#else
	if ((rand() % DIAG_MEM_CONST) == 0) {
		mem = 0;
	} else {
		mem = malloc(n);
	}
#endif

	(void)ref;		/* unused param warning */
	return mem;
}

static sah_Head_Desc *my_alloc_head_desc(void *ref)
{
	sah_Head_Desc *mem;

#ifndef DIAG_MEM_ERRORS
	mem = malloc(sizeof(sah_Head_Desc));
#else
	if ((rand() % DIAG_MEM_CONST) == 0) {
		mem = 0;
	} else {
		mem = malloc(sizeof(sah_Head_Desc));
	}
#endif

	(void)ref;		/* unused param warning */
	return mem;
}

static sah_Desc *my_alloc_desc(void *ref)
{
	sah_Desc *mem;

#ifndef DIAG_MEM_ERRORS
	mem = malloc(sizeof(sah_Desc));
#else
	if ((rand() % DIAG_MEM_CONST) == 0) {
		mem = 0;
	} else {
		mem = malloc(sizeof(sah_Desc));
	}
#endif

	(void)ref;		/* unused param warning */
	return mem;
}

static sah_Link *my_alloc_link(void *ref)
{
	sah_Link *mem;

#ifndef DIAG_MEM_ERRORS
	mem = malloc(sizeof(sah_Link));
#else
	if ((rand() % DIAG_MEM_CONST) == 0) {
		mem = 0;
	} else {
		mem = malloc(sizeof(sah_Link));
	}
#endif

	(void)ref;		/* unused param warning */
	return mem;
}

static void my_free(void *ref, void *ptr)
{
	free(ptr);
	(void)ref;		/* unused param warning */
	return;
}

static void *my_memcpy(void *ref, void *dest, const void *src, size_t n)
{
	(void)ref;		/* unused param warning */
	return memcpy(dest, src, n);
}

static void *my_memset(void *ref, void *ptr, int ch, size_t n)
{
	(void)ref;		/* unused param warning */
	return memset(ptr, ch, n);
}

/*! Standard memory manipulation routines for user-mode API. */
static sah_Mem_Util std_usermode_mem_util = {
	.mu_ref = 0,
	.mu_malloc = my_malloc,
	.mu_alloc_head_desc = my_alloc_head_desc,
	.mu_alloc_desc = my_alloc_desc,
	.mu_alloc_link = my_alloc_link,
	.mu_free = my_free,
	.mu_free_head_desc = (void (*)(void *, sah_Head_Desc *))my_free,
	.mu_free_desc = (void (*)(void *, sah_Desc *))my_free,
	.mu_free_link = (void (*)(void *, sah_Link *))my_free,
	.mu_memcpy = my_memcpy,
	.mu_memset = my_memset
};

static fsl_shw_return_t add_user(fsl_shw_uco_t * uco);
static void remove_user(fsl_shw_uco_t * uco);
static int setup_callback(fsl_shw_uco_t * uco);
static void sah_sighandler(int num);
static fsl_shw_return_t sah_service_request(unsigned int command,
					    void *arg, fsl_shw_uco_t * uco);

/*!
 * @brief    Sends a request to get the platform capabilities
 *
 * @param[in]   user_ctx                User context.
 * @param[out]  sahara2_capabilities    Platform capabilities.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t get_capabilities(fsl_shw_uco_t * user_ctx,
				  fsl_shw_pco_t * capabilities)
{
	return sah_service_request(SAHARA_GET_CAPS, capabilities, user_ctx);
}

/*!
 * @brief    Sends a request to register this user
 *
 * @param[in,out] uco User context.  Part of the structre contains input
 *                                   parameters and part is filled in by the
 *                                   driver.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_register(fsl_shw_uco_t * user_ctx)
{
	fsl_shw_return_t status = FSL_RETURN_ERROR_S;
	unsigned dev_opened = 0;	/* boolean */
	unsigned user_added = 0;	/* boolean */

	/* Link user into process-local chain of contexts */
	status = add_user(user_ctx);
	if (status != FSL_RETURN_OK_S) {
		goto out;
	}
	user_added = 1;

	if (user_ctx->sahara_openfd >= 0) {
		status = FSL_RETURN_ERROR_S;
		goto out;
	}

	/* This code needs to open the device RIGHT HERE */
	user_ctx->sahara_openfd = open(SAHARA_DEVICE, O_RDWR);
	if (user_ctx->sahara_openfd < 0) {
		status = FSL_RETURN_ERROR_S;
		goto out;
	}
	dev_opened = 1;
	user_ctx->mem_util = &std_usermode_mem_util;

	/* check that uco is valid */
	status = sah_validate_uco(user_ctx);
	if (status != FSL_RETURN_OK_S) {
		goto out;
	}

	/*  Life is good, register this user */
	status = sah_service_request(SAHARA_REGISTER, (void *)user_ctx, user_ctx);

      out:
	if (status != FSL_RETURN_OK_S) {
		if (user_added) {
			remove_user(user_ctx);
		}
		if (dev_opened) {
			close(user_ctx->sahara_openfd);
			user_ctx->sahara_openfd = -1;
		}
	}

	return status;
}

/*!
 * @brief    Sends a request to deregister this user
 *
 * @param[in,out] uco User context.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_deregister(fsl_shw_uco_t * uco)
{
	fsl_shw_return_t status = FSL_RETURN_ERROR_S;

	/* Turn off flags to make sure anything outstanding does not make waves. */
	uco->flags &=
	    ~(FSL_UCO_CALLBACK_SETUP_COMPLETE | FSL_UCO_CALLBACK_MODE);

	remove_user(uco);

	/* check that a valid file descriptor could exist */
	if (uco->sahara_openfd >= 0) {
		status =
		    sah_service_request(SAHARA_DEREGISTER, (void *)uco, uco);
	}

	if (status == FSL_RETURN_OK_S) {
		/* close down the ioctl access */
		close(uco->sahara_openfd);
		uco->sahara_openfd = -1;
	}

	return status;
}

/*!
 * @brief    Sends a request to get results from the result pool
 *
 * @param[in,out] arg    Location containing info for retrieving results
 * @param         uco    User context.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_get_results(sah_results * arg, fsl_shw_uco_t * uco)
{
	fsl_shw_return_t code = sah_service_request(SAHARA_GET_RESULTS,
						    (void *)arg, uco);

	if ((code == FSL_RETURN_OK_S) && (arg->actual != 0)) {
		sah_Postprocess_Results(uco, arg);
	}

	return code;
}

/*!
 * @brief   Allocate a slot in the system keystore
 *
 * @param       user_ctx    User context
 * @param       key_length  Requested length (octets)
 * @param       ownerid     Owner ID to associate with key
 * @param[out]  slot        Allocated slot number
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t do_system_keystore_slot_alloc(fsl_shw_uco_t * user_ctx,
					       uint32_t key_length,
					       uint64_t ownerid,
					       uint32_t * slot)
{
	fsl_shw_return_t ret;
	scc_slot_t slot_info;

	slot_info.key_length = key_length;
	slot_info.ownerid = ownerid;

	ret = sah_service_request(SAHARA_SK_ALLOC, &slot_info, user_ctx);

	if (ret == FSL_RETURN_OK_S) {
		*slot = slot_info.slot;
		ret = slot_info.code;
	}

	return ret;
}

/*!
 * @brief   Deallocate an allocated slot in the system keystore
 *
 * @param       user_ctx    User context
 * @param       ownerid     Owner ID to associate with key
 * @param[out]  slot        Allocated slot number
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t do_system_keystore_slot_dealloc(fsl_shw_uco_t * user_ctx,
						 uint64_t ownerid,
						 uint32_t slot)
{
	scc_slot_t slot_info;
	fsl_shw_return_t ret;

	slot_info.ownerid = ownerid;
	slot_info.slot = slot;

	ret = sah_service_request(SAHARA_SK_DEALLOC, &slot_info, user_ctx);
	if (ret == FSL_RETURN_OK_S) {
		ret = slot_info.code;
	}

	return ret;
}

/*!
 * @brief   Load a plaintext key into a slot in the system keystore
 *
 * @param       user_ctx    User context
 * @param       ownerid     Owner ID to associate with key
 * @param       slot        Slot to load the key into
 * @param       key         Plaintext key data to put in the slot
 * @param       key_length  Length of plaintext key (octets)
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t do_system_keystore_slot_load(fsl_shw_uco_t * user_ctx,
					      uint64_t ownerid,
					      uint32_t slot,
					      const uint8_t * key,
					      uint32_t key_length)
{
	scc_slot_t slot_info;
	fsl_shw_return_t ret;

	slot_info.ownerid = ownerid;
	slot_info.slot = slot;
	slot_info.key_length = key_length;
	slot_info.key = (void *)key;

	ret = sah_service_request(SAHARA_SK_LOAD, &slot_info, user_ctx);
	if (ret == FSL_RETURN_OK_S) {
		ret = slot_info.code;
	}

	return ret;
}

/*!
 * @brief   Encrypt a key stored in the system keystore
 *
 * @param       user_ctx    User context
 * @param       ownerid     Owner ID associated with the key
 * @param       slot        Slot to encrypt
 * @param       key_length  Length of plaintext key (octets)
 * @param       black_data  Location to store the encrypted key
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t do_system_keystore_slot_encrypt(fsl_shw_uco_t * user_ctx,
						 uint64_t ownerid,
						 uint32_t slot,
						 uint32_t key_length,
						 uint8_t * black_data)
{
	scc_slot_t slot_info;
	fsl_shw_return_t ret;

	slot_info.ownerid = ownerid;
	slot_info.slot = slot;
	slot_info.key = black_data;
	slot_info.key_length = key_length;

	ret = sah_service_request(SAHARA_SK_SLOT_ENC, &slot_info, user_ctx);
	if (ret == FSL_RETURN_OK_S) {
		ret = slot_info.code;
	}

	return ret;
}

/*!
 * @brief   Decrypt a key to the system keystore
 *
 * @param       user_ctx    User context
 * @param       ownerid     Owner ID associated with the key
 * @param       slot        Slot to fill
 * @param       key_length  Length of plaintext key (octets)
 * @param       black_data  Location of the encrypted key
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t do_system_keystore_slot_decrypt(fsl_shw_uco_t * user_ctx,
						 uint64_t ownerid,
						 uint32_t slot,
						 uint32_t key_length,
						 const uint8_t * black_data)
{
	scc_slot_t slot_info;
	fsl_shw_return_t ret;

	slot_info.ownerid = ownerid;
	slot_info.slot = slot;
	slot_info.key = (uint8_t *) black_data;
	slot_info.key_length = key_length;

	ret = sah_service_request(SAHARA_SK_SLOT_DEC, &slot_info, user_ctx);
	if (ret == FSL_RETURN_OK_S) {
		ret = slot_info.code;
	}

	return ret;
}

/*!
 * @brief   Encrypt a region of secure memory using the hardware secret key
 *
 * @param       user_ctx        User context
 * @param       partition_base  Base address of the partition
 * @param       offset_bytes    Offset of data from the partition base
 * @param       byte_count      Length of the data to encrypt
 * @param       black_data      Location to store the encrypted data
 * @param       IV              IV to use for the encryption routine
 * @param       cypher_mode     Cyphering mode to use, specified by type
 *                              #fsl_shw_cypher_mode_t
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t
do_scc_encrypt_region(fsl_shw_uco_t * user_ctx,
		      void *partition_base, uint32_t offset_bytes,
		      uint32_t byte_count, uint8_t * black_data,
		      uint32_t * IV, fsl_shw_cypher_mode_t cypher_mode)
{
	fsl_shw_return_t ret;
	scc_region_t region_info;
	int i;

	region_info.partition_base = (uint32_t) partition_base;
	region_info.offset = offset_bytes;
	region_info.length = byte_count;
	region_info.black_data = black_data;
	region_info.cypher_mode = cypher_mode;

	if (cypher_mode == FSL_SHW_CYPHER_MODE_CBC) {
		for (i = 0; i < 4; i++) {
			region_info.IV[i] = IV[i];
		}
	}

	ret = sah_service_request(SAHARA_SCC_ENCRYPT, &region_info, user_ctx);

	if (ret == FSL_RETURN_OK_S) {
		ret = region_info.code;
	}

	return ret;
}

/*!
 * @brief   Decrypt a region of secure memory using the hardware secret key
 *
 * @param       user_ctx        User context
 * @param       partition_base  Base address of the partition
 * @param       offset_bytes    Offset of data from the partition base
 * @param       byte_count      Length of the data to decrypt
 * @param       black_data      Location to store the decrypted data
 * @param       IV              IV to use for the decryption routine
 * @param       cypher_mode     Cyphering mode to use, specified by type
 *                              #fsl_shw_cypher_mode_t
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t
do_scc_decrypt_region(fsl_shw_uco_t * user_ctx,
		      void *partition_base, uint32_t offset_bytes,
		      uint32_t byte_count, const uint8_t * black_data,
		      uint32_t * IV, fsl_shw_cypher_mode_t cypher_mode)
{
	fsl_shw_return_t ret;
	scc_region_t region_info;
	int i;

	region_info.partition_base = (uint32_t) partition_base;
	region_info.offset = offset_bytes;
	region_info.length = byte_count;
	region_info.black_data = (uint8_t *) black_data;
	region_info.cypher_mode = cypher_mode;
	if (cypher_mode == FSL_SHW_CYPHER_MODE_CBC) {
		for (i = 0; i < 4; i++) {
			region_info.IV[i] = IV[i];
		}
	}

	ret = sah_service_request(SAHARA_SCC_DECRYPT, &region_info, user_ctx);

	if (ret == FSL_RETURN_OK_S) {
		ret = region_info.code;
	}

	return ret;
}

/*!
 * Allocate a block of secure memory
 *
 * @param       user_ctx        User context
 * @param       size            Memory size (octets).  Note: currently only
 *                              supports only single-partition sized blocks.
 * @param       UMID            User Mode ID to use when registering the
 *                              partition.
 * @param       permissions     Permissions to initialize the partition with.
 *                              Can be made by ORing flags from the
 *                              #fsl_shw_permission_t.
 *
 * @return                      Address of the allocated memory.  NULL if the
 *                              call was not successful.
 */
void *fsl_shw_smalloc(fsl_shw_uco_t * user_ctx,
		      uint32_t size, const uint8_t * UMID, uint32_t permissions)
{
	void *address;
	uint8_t engaged = 0;

	/* Acquire a secure partition by calling mmap() on the SHW file */
	address = mmap(NULL, size,
		       PROT_READ | PROT_WRITE,
		       MAP_SHARED, user_ctx->sahara_openfd, 0);

	if (address == MAP_FAILED) {
		printf("Could not acquire partition!\n");
		goto out;
	}

	/* Finish setup of the secure partition by writing the UMID and permissions
	 * registers.
	 */
	if (do_scc_engage_partition(user_ctx, address, UMID, permissions)
	    == FSL_RETURN_OK_S) {
		engaged = 1;
	}

      out:
	if (address == NULL) {
		return NULL;
	}

	if (engaged == 0) {
		/* engage failed, release partition */
		fsl_shw_sfree(user_ctx, address);
		return NULL;
	}

	return address;
}

fsl_shw_return_t do_scc_engage_partition(fsl_shw_uco_t * user_ctx,
					 void *address,
					 const uint8_t * UMID,
					 uint32_t permissions)
{
#ifdef FSL_HAVE_SCC2
	uint8_t *UMID_base = address + 0x10;
	uint32_t *MAP_base = address;
	uint8_t i;
	fsl_shw_partition_status_t status;

	if (fsl_shw_sstatus(user_ctx, address, &status) != FSL_RETURN_OK_S ||
	    status != FSL_PART_S_ALLOCATED) {
		printf("partition not allocated, cannot engage!\n");
		return FSL_RETURN_ERROR_S;
	}

	if (UMID != NULL) {
		for (i = 0; i < 16; i++) {
			UMID_base[i] = UMID[i];
		}
	} else {
		for (i = 0; i < 16; i++) {
			UMID_base[i] = 0;
		}
	}

	MAP_base[0] = permissions;

	/* Check the partition status */
	if (fsl_shw_sstatus(user_ctx, address, &status) == FSL_RETURN_OK_S &&
	    status == FSL_PART_S_ENGAGED) {
		return FSL_RETURN_OK_S;
	} else {
		printf("partition failed to engage!\n");
		return FSL_RETURN_ERROR_S;
	}
#else				/* FSL_HAVE_SCC2 */
	(void)user_ctx;
	(void)address;
	(void)UMID;
	(void)permissions;
	return FSL_RETURN_ERROR_S;
#endif
}

/*!
 * Free a block of secure memory that was allocated with #fsl_shw_smalloc
 *
 * @param       user_ctx        User context
 * @param       address         Address of the block of secure memory to be
 *                              released.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_sfree(fsl_shw_uco_t * user_ctx, void *address)
{
	/* call ioctl on the SHW file to release the partition
	 *
	 * note: munmap cannot be used because it doesn't notify the driver that
	 * the memory is being unmapped.
	 */
	scc_partition_info_t partition_info;
	fsl_shw_return_t ret;

	partition_info.user_base = (uint32_t) address;

	ret = sah_service_request(SAHARA_SCC_SFREE, &partition_info, user_ctx);

	return ret;
}

fsl_shw_return_t fsl_shw_sstatus(fsl_shw_uco_t * user_ctx,
				 void *address,
				 fsl_shw_partition_status_t * status)
{
	/* call ioctl on the SHW file to release the partition
	 *
	 * note: munmap cannot be used because it doesn't notify the driver that
	 * the memory is being unmapped.
	 */
	scc_partition_info_t partition_info;
	fsl_shw_return_t ret;

	partition_info.user_base = (uint32_t) address;

	ret =
	    sah_service_request(SAHARA_SCC_SSTATUS, &partition_info, user_ctx);

	if (ret == FSL_RETURN_OK_S) {
		*status = partition_info.status;
	}

	return ret;
}

/*!
 * Diminish the permissions of a block of secure memory.  Note that permissions
 * can only be revoked.
 *
 * @param       user_ctx        User context
 * @param       address         Base address of the secure memory to work with
 * @param       permissions     Permissions to initialize the partition with.
 *                              Can be made by ORing flags from the
 *                              #fsl_shw_permission_t.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t fsl_shw_diminish_perms(fsl_shw_uco_t * user_ctx,
					void *address, uint32_t permissions)
{
	/* Call ioctl on the SHW file to diminish permissions on the partition */
	scc_partition_info_t partition_info;
	fsl_shw_return_t ret;

	partition_info.user_base = (uint32_t) address;
	partition_info.permissions = permissions;

	ret =
	    sah_service_request(SAHARA_SCC_DROP_PERMS, &partition_info,
				user_ctx);

	return ret;
}

/*!
 * This function writes the Descriptor Chain to the kernel driver.
 *
 * @brief     Writes the Descriptor Chain to the kernel driver.
 *
 * @param    dar  A pointer to a Descriptor Chain of type sah_Desc
 * @param         uco     User context.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t adaptor_Exec_Descriptor_Chain(sah_Head_Desc * dar,
					       fsl_shw_uco_t * uco)
{
	fsl_shw_return_t ret = FSL_RETURN_OK_S;
	uint32_t blocking = (uco->flags & FSL_UCO_BLOCKING_MODE);

	if ((uco->flags & FSL_UCO_CALLBACK_MODE)
	    && !(uco->flags & FSL_UCO_CALLBACK_SETUP_COMPLETE)) {
		if (setup_callback(uco) == 0) {
#ifdef DIAG_ADAPTOR
			LOG_DIAG("callback setup failed");
#endif
			ret = FSL_RETURN_ERROR_S;
		}
	}
#ifdef DIAG_ADAPTOR
	um_Dump_Chain(&dar->desc);
#endif

	if (ret == FSL_RETURN_OK_S) {
		ret = sah_service_request(SAHARA_DAR, (void *)dar, uco);
	}

#ifdef DIAG_ADAPTOR
		if (ret == FSL_RETURN_OK_S) {
			LOG_DIAG("sah_service_request returned OK");
		} else {
			LOG_DIAG("sah_service_request failed");
		}
#endif

	if (blocking && (ret == FSL_RETURN_OK_S)) {
		/* chain actually executed, or at least queue */
		ret = dar->result;
	};

#ifdef DIAG_ADAPTOR
		if (ret == FSL_RETURN_OK_S) {
			LOG_DIAG("chain return OK");
		} else {
			LOG_DIAG("chain return failed");
		}
#endif

	return ret;
}

/*!
 * Service request pass from UM to KM in this function via an ioctl
 * call.
 *
 * @brief    UM to KM command passing
 *
 * @param    command       the command to pass via the ioctl call
 * @param    arg           pointer to pass to kernel space
 * @param    uco           User context.
 *
 * @return    A return code of type #fsl_shw_return_t.
 */
fsl_shw_return_t sah_service_request(unsigned int command,
				     void *arg, fsl_shw_uco_t * uco)
{
	int linux_return_value;
	fsl_shw_return_t status;

	/* Need to retry the ioctl() in case it was interupted. This
	 * interruption would be due to another thread performing a reset of
	 * the SAHARA HW. Upon interruption, the descriptor chain must be
	 * re-written since the chain still needs to be executed.
	 */
	do {
		linux_return_value = ioctl(uco->sahara_openfd,
					   command, (unsigned long)arg);
	} while ((linux_return_value == IO_ERROR) && (errno == EINTR));

	if (linux_return_value == 0) {
		status = FSL_RETURN_OK_S;
	} else {
#ifdef DIAG_ADAPTOR
		 LOG_DIAG_ARGS("errno from ioctl() is %d", errno);
#endif
		status = FSL_RETURN_ERROR_S;
	}

#ifdef DIAG_ADAPTOR
	if (status != FSL_RETURN_OK_S) {
		LOG_DIAG("failed to perform service ioctl operation");
	}
#endif				/* DIAG_ADAPTOR */

	return status;
}

/*!
 * Link a newly-registered context to the #user_chain.
 *
 * @param uco    User context to add
 *
 * @return FSL_RETURN_OK_S on success; other error code
 */
static fsl_shw_return_t add_user(fsl_shw_uco_t * uco)
{
	fsl_shw_uco_t *current = user_chain;
	fsl_shw_uco_t *prev = NULL;
	fsl_shw_return_t code = FSL_RETURN_ERROR_S;

	/*
	 * Trundle down the chain searching to see whether the 'new' context is
	 * already on the list.
	 */
	while ((current != uco) && (current != NULL)) {
		prev = current;
		current = current->next;
	}

	if (current != uco) {
		uco->next = user_chain;
		user_chain = uco;
		code = FSL_RETURN_OK_S;
	}

	return code;
}

/*!
 * Unlink a deregistered context from the #user_chain
 *
 * @param uco    User context to remove
 */
static void remove_user(fsl_shw_uco_t * uco)
{
	fsl_shw_uco_t *prev = NULL;
	fsl_shw_uco_t *current = user_chain;

	/* Search chain looking for the entry */
	while ((current != uco) && (current != NULL)) {
		prev = current;
		current = current->next;
	}

	/* Did we find it */
	if (current != NULL) {
		if (prev == NULL) {
			/* It is first entry.  Point head to next in chain */
			user_chain = current->next;
		} else {
			/* Remove it from chain by pointing previous to next */
			prev->next = current->next;
		}
		current->next = NULL;	/* just for safety */
	}

	return;
}

/*!
 * Set up API's internal callback handler on SIGUSR2
 *
 * @param uco   User context
 *
 * @return 0 for failure, 1 for success
 */
static int setup_callback(fsl_shw_uco_t * uco)
{
	int code = 0;

	if (!callback_initialized) {
		/* This is defined by POSIX */
		struct sigaction action;

		action.sa_handler = sah_sighandler;
		action.sa_flags = 0;	/* no special flags needed. */
		sigfillset(&action.sa_mask);	/* suspend all signals during handler */
		if ((code = sigaction(SIGUSR2, &action, NULL)) != 0) {
			fprintf(stderr, "sigaction() failed with code %d\n",
				code);
		} else {
			uco->flags |= FSL_UCO_CALLBACK_SETUP_COMPLETE;
			callback_initialized = 1;
			code = 1;
		}
	} else {
		code = 1;
	}

	return code;
}

/*!
 * User-mode signal handler.
 *
 * Called when SIGUSR1 fires.  This will call the user's callback function
 * if the user still has one registered.
 *
 * @param num    Signal number (ignored)
 */
static void sah_sighandler(int num)
{
	fsl_shw_uco_t *current_user = user_chain;

	/* Something happened.  Callback anybody who has callback on.  */
	while (current_user != NULL) {
		fsl_shw_uco_t *next_user = current_user->next;

		if ((current_user->flags & FSL_UCO_CALLBACK_MODE)
		    && (current_user->callback != NULL)) {
			current_user->callback(current_user);
		}
		current_user = next_user;
	}

	(void)num;		/* unused */
	return;
}

#ifdef DIAG_ADAPTOR

#define add_literal(str) snprintf(buffer + output_count,            \
                                  count - output_count, "%s", str)

static int interpret_skha_modes(uint32_t header, char *buffer, int count)
{
	unsigned output_count = 0;

	switch (header & 0x3) {
	case 0:
		output_count += add_literal("AES ");
		break;
	case 1:
		output_count += add_literal("DES ");
		break;
	case 2:
		output_count += add_literal("3DES ");
		break;
	case 3:
		output_count += add_literal("ARC4 ");
		break;
	}

	if (header & 0x4) {
		output_count += add_literal("Encrypt ");
	} else {
		output_count += add_literal("Decrypt ");
	}

	switch ((header >> 3) & 0x3) {
	case 0:
		output_count += add_literal("ECB ");
		break;
	case 1:
		output_count += add_literal("CBC ");
		break;
	case 2:
		output_count += add_literal("CCM ");
		break;
	case 3:
		output_count += add_literal("CTR ");
		break;
	}

	if (header & 0x20) {
		output_count += add_literal("Aux0 ");
	}

	if (header & 0x100) {
		output_count += add_literal("DisKeyPar ");
	}

	if ((((header >> 3) & 0x3) == 2) || (((header >> 3) & 0x3) == 3)) {
		switch ((header >> 9) & 0xF) {
		case 0:
			output_count += add_literal("CtrMod2^8 ");
			break;
		case 1:
			output_count += add_literal("CtrMod2^16 ");
			break;
		case 2:
			output_count += add_literal("CtrMod2^24 ");
			break;
		case 3:
			output_count += add_literal("CtrMod2^32 ");
			break;
		case 4:
			output_count += add_literal("CtrMod2^40 ");
			break;
		case 5:
			output_count += add_literal("CtrMod2^48 ");
			break;
		case 6:
			output_count += add_literal("CtrMod2^56 ");
			break;
		case 7:
			output_count += add_literal("CtrMod2^64 ");
			break;
		case 8:
			output_count += add_literal("CtrMod2^72 ");
			break;
		case 9:
			output_count += add_literal("CtrMod2^80 ");
			break;
		case 10:
			output_count += add_literal("CtrMod2^88 ");
			break;
		case 11:
			output_count += add_literal("CtrMod2^96 ");
			break;
		case 12:
			output_count += add_literal("CtrMod2^104 ");
			break;
		case 13:
			output_count += add_literal("CtrMod2^112 ");
		case 14:
			output_count += add_literal("CtrMod2^120 ");
			break;
		case 15:
			output_count += add_literal("CtrMod2^128 ");
			break;
		}
	}

	if (header & 0xE0C0) {
		output_count += add_literal("unknown_bits ");

	}

	return output_count;
}

static int interpret_mdha_modes(uint32_t header, char *buffer, int count)
{
	unsigned output_count = 0;

	switch (header & 3) {
	case 0:
		output_count += add_literal("SHA-1 ");
		break;
	case 1:
		output_count += add_literal("MD5 ");
		break;
	case 2:
		output_count += add_literal("SHA-256 ");
		break;
	case 3:
		output_count += add_literal("SHA-224 ");
		break;
	}

	if (header & 0x4) {
		output_count += add_literal("Pdata ");
	}

	if (header & 0x8) {
		output_count += add_literal("HMAC ");
	}

	if (header & 0x20) {
		output_count += add_literal("Init ");
	}

	if (header & 0x40) {
		output_count += add_literal("IPad ");
	}

	if (header & 0x80) {
		output_count += add_literal("OPad ");
	}

	if (header & 0x200) {
		output_count += add_literal("MacFull ");
	}

	if (header & 0x400) {
		output_count += add_literal("SSL ");
	}

	if (header & 0x1000) {
		output_count += add_literal("ICV ");
	}

	return output_count;
}

static int interpret_rng_modes(uint32_t header, char *buffer, int count)
{
	unsigned output_count = 0;
	unsigned mode = header & 0xF;

	switch (mode) {
	case 0:
		output_count += add_literal("Normal ");
		break;
	case 1:
		output_count += add_literal("GenSeed ");
		break;
	case 2:
#ifdef FSL_HAVE_SAHARA2
		output_count += add_literal("GenEntropy ");
#else
		output_count += add_literal("Reserved ");
#endif
		break;
	case 4:
		output_count += add_literal("SelfTest ");
		break;
#ifdef FSL_HAVE_SAHAR4
	case 8:
		/* fall through */
	case 9:
		output_count += add_literal("NoZeroBytes ");
		break;
#endif
	default:
		output_count += add_literal("Garbage ");
	}

	return output_count;
}

static int interpret_pkha_modes(uint32_t header, char *buffer, int count)
{
	unsigned output_count = 0;

	switch (header & 0x3F) {
	case 0x00:
		output_count += add_literal("reserved ");
		break;
	case 0x01:
		output_count += add_literal("clr_mem ");
		break;
	case 0x02:
		output_count += add_literal("clr_eram ");
		break;
	case 0x03:
		output_count += add_literal("mod_exp ");
		break;
	case 0x04:
		output_count += add_literal("mod_r2modn ");
		break;
	case 0x05:
		output_count += add_literal("mod_rrmodp ");
		break;
	case 0x06:
		output_count += add_literal("ec_fp_aff_ptmult ");
		break;
	case 0x07:
		output_count += add_literal("ec_f2m_aff_ptmult ");
		break;
	case 0x08:
		output_count += add_literal("ec_fp_proj_ptmult ");
		break;
	case 0x09:
		output_count += add_literal("ec_f2m_proj_ptmult ");
		break;
	case 0x0a:
		output_count += add_literal("ec_fp_add ");
		break;
	case 0x0b:
		output_count += add_literal("ec_fp_double ");
		break;
	case 0x0c:
		output_count += add_literal("ec_f2m_add ");
		break;
	case 0x0d:
		output_count += add_literal("ec_f2m_double ");
		break;
	case 0x0e:
		output_count += add_literal("f2m_r2 ");
		break;
	case 0x0f:
		output_count += add_literal("f2m_inv ");
		break;
	case 0x10:
		output_count += add_literal("mod_inv ");
		break;
	case 0x11:
		output_count += add_literal("rsa_sstep ");
		break;
	case 0x12:
		output_count += add_literal("mod_emodn ");
		break;
	case 0x13:
		output_count += add_literal("f2m_emodn ");
		break;
	case 0x14:
		output_count += add_literal("ec_fp_ptmul ");
		break;
	case 0x15:
		output_count += add_literal("ec_f2m_ptmul ");
		break;
	case 0x16:
		output_count += add_literal("f2m_gcd ");
		break;
	case 0x17:
		output_count += add_literal("mod_gcd ");
		break;
	case 0x18:
		output_count += add_literal("f2m_dbl_aff ");
		break;
	case 0x19:
		output_count += add_literal("fp_dbl_aff ");
		break;
	case 0x1a:
		output_count += add_literal("f2m_add_aff ");
		break;
	case 0x1b:
		output_count += add_literal("fp_add_aff ");
		break;
	case 0x1c:
		output_count += add_literal("f2m_exp ");
		break;
	case 0x1d:
		output_count += add_literal("mod_exp_teq ");
		break;
	case 0x1e:
		output_count += add_literal("rsa_sstep_teq ");
		break;
	case 0x1f:
		output_count += add_literal("f2m_multn ");
		break;
	case 0x20:
		output_count += add_literal("mod_multn ");
		break;
	case 0x21:
		output_count += add_literal("mod_add ");
		break;
	case 0x22:
		output_count += add_literal("mod_sub ");
		break;
	case 0x23:
		output_count += add_literal("mod_mult1_mont ");
		break;
	case 0x24:
		output_count += add_literal("mod_mult2_deconv ");
		break;
	case 0x25:
		output_count += add_literal("f2m_add ");
		break;
	case 0x26:
		output_count += add_literal("f2m_mult1_mont ");
		break;
	case 0x27:
		output_count += add_literal("f2m_mult2_deconv ");
		break;
	case 0x28:
		output_count += add_literal("miller_rabin ");
		break;
	default:
		output_count += add_literal("unknown ");
		break;
	}

	if (header & 0x200) {
		output_count += add_literal("SoftErrFalse ");
	}

	if (header & 0x100) {
		output_count += add_literal("SoftErrTrue ");
	}

	return output_count;
}

static int interpret_header(uint32_t header, char *buffer, int count)
{
	unsigned output_count = 0;
	unsigned desc_type = ((header >> 24) & 0x70) | ((header >> 16) & 0xF);

	switch (desc_type) {
	case 0x12:
		output_count += add_literal("5/SKHA_ST_CTX");
		break;
	case 0x13:
		output_count += add_literal("35/SKHA_LD_MODE_KEY: ");
		output_count += interpret_skha_modes(header,
						     buffer + output_count,
						     count - output_count);
		break;
	case 0x14:
		output_count += add_literal("38/SKHA_LD_MODE_IN_CPHR_ST_CTX: ");
		output_count += interpret_skha_modes(header,
						     buffer + output_count,
						     count - output_count);
		break;
	case 0x15:
		output_count += add_literal("4/SKHA_IN_CPHR_OUT");
		break;
	case 0x16:
		output_count += add_literal("34/SKHA_ST_SBOX");
		break;
	case 0x18:
		output_count += add_literal("1/SKHA_LD_MODE_IV_KEY: ");
		output_count += interpret_skha_modes(header,
						     buffer + output_count,
						     count - output_count);
		break;
	case 0x19:
		output_count += add_literal("33/SKHA_ST_SBOX");
		break;
	case 0x1D:
		output_count += add_literal("2/SKHA_LD_MODE_IN_CPHR_OUT: ");
		output_count += interpret_skha_modes(header,
						     buffer + output_count,
						     count - output_count);
		break;
	case 0x22:
		output_count += add_literal("11/MDHA_ST_MD");
		break;
	case 0x25:
		output_count += add_literal("10/MDHA_HASH_ST_MD");
		break;
	case 0x28:
		output_count += add_literal("6/MDHA_LD_MODE_MD_KEY: ");
		output_count += interpret_mdha_modes(header,
						     buffer + output_count,
						     count - output_count);
		break;
	case 0x2A:
		output_count += add_literal("39/MDHA_ICV");
		break;
	case 0x2D:
		output_count += add_literal("8/MDHA_LD_MODE_HASH_ST_MD: ");
		output_count += interpret_mdha_modes(header,
						     buffer + output_count,
						     count - output_count);
		break;
	case 0x3C:
		output_count += add_literal("18/RNG_GEN: ");
		output_count += interpret_rng_modes(header,
						    buffer + output_count,
						    count - output_count);
		break;
	case 0x40:
		output_count += add_literal("19/PKHA_LD_N_E");
		break;
	case 0x41:
		output_count += add_literal("36/PKHA_LD_A3_B0");
		break;
	case 0x42:
		output_count += add_literal("27/PKHA_ST_A_B");
		break;
	case 0x43:
		output_count += add_literal("22/PKHA_LD_A_B");
		break;
	case 0x44:
		output_count += add_literal("23/PKHA_LD_A0_A1");
		break;
	case 0x45:
		output_count += add_literal("24/PKHA_LD_A2_A3");
		break;
	case 0x46:
		output_count += add_literal("25/PKHA_LD_B0_B1");
		break;
	case 0x47:
		output_count += add_literal("26/PKHA_LD_B2_B3");
		break;
	case 0x48:
		output_count += add_literal("28/PKHA_ST_A0_A1");
		break;
	case 0x49:
		output_count += add_literal("29/PKHA_ST_A2_A3");
		break;
	case 0x4A:
		output_count += add_literal("30/PKHA_ST_B0_B1");
		break;
	case 0x4B:
		output_count += add_literal("31/PKHA_ST_B2_B3");
		break;
	case 0x4C:
		output_count += add_literal("32/PKHA_EX_ST_B1: ");
		output_count += interpret_pkha_modes(header,
						     buffer + output_count,
						     count - output_count);
		break;
	case 0x4D:
		output_count += add_literal("20/PKHA_LD_A_EX_ST_B: ");
		output_count += interpret_pkha_modes(header,
						     buffer + output_count,
						     count - output_count);
		break;
	case 0x4E:
		output_count += add_literal("21/PKHA_LD_N_EX_ST_B: ");
		output_count += interpret_pkha_modes(header,
						     buffer + output_count,
						     count - output_count);
		break;
	case 0x4F:
		output_count += add_literal("37/PKHA_ST_B1_B2 ");
		break;
	default:
		output_count +=
		    snprintf(buffer + output_count, count - output_count,
			     "%d/UNKNOWN", desc_type);
		break;
	}

	return output_count;
}				/* cvt_desc_name() */

/*!
 * Dump chain of descriptors to the log.
 *
 * @brief Dump descriptor chain
 *
 * @param    chain     Kernel virtual address of start of chain of descriptors
 *
 * @return   void
 */
void um_Dump_Chain(const sah_Desc * chain)
{
	int desc_no = 1;

	while (chain != NULL) {
		const int buf_size = 120;
		char desc_name[buf_size];
		int output_bytes = 0;

		output_bytes += snprintf(desc_name, buf_size,
					 "Desc %02d (", desc_no++);

		output_bytes += interpret_header(chain->header,
						 desc_name + output_bytes,
						 buf_size - output_bytes);

		output_bytes += snprintf(desc_name + output_bytes,
					 buf_size - output_bytes, ")\nDesc  ");

		um_Dump_Words(desc_name, (unsigned *)chain,
			      6 /*sizeof(*chain)/sizeof(unsigned) */ );
		/* place this definition elsewhere */
		if (chain->ptr1) {
			if (chain->header & SAH_HDR_LLO) {
				um_Dump_Region(" Data1", chain->ptr1,
					       chain->len1);
			} else {
				um_Dump_Link(" Link1", chain->ptr1);
			}
		}
		if (chain->ptr2) {
			if (chain->header & SAH_HDR_LLO) {
				um_Dump_Region(" Data2", chain->ptr2,
					       chain->len2);
			} else {
				um_Dump_Link(" Link2", chain->ptr2);
			}
		}

		chain = (chain->next ? chain->next : 0);
	}
}

/*!
 * Dump chain of links to the log.
 *
 * @brief Dump chain of links
 *
 * @param    prefix    Text to put in front of dumped data
 * @param    link      Kernel virtual address of start of chain of links
 *
 * @return   void
 */
static void um_Dump_Link(const char *prefix, const sah_Link * link)
{
	while (link != NULL) {
		um_Dump_Words(prefix, (unsigned *)link,
			      3 /* # words in h/w link */ );
		if (link->flags & SAH_STORED_KEY_INFO) {
			LOG_DIAG_ARGS("  SCC: Slot %d", link->slot);
		} else if (link->data != NULL) {
			um_Dump_Region("  Data", link->data, link->len);
		}

		link = (link->next ? link->next : 0);
	}
}

/*!
 * Dump given region of data to the log.
 *
 * @brief Dump data
 *
 * @param    prefix    Text to put in front of dumped data
 * @param    data      Kernel virtual address of start of region to dump
 * @param    length    Amount of data to dump
 *
 * @return   void
 */
void um_Dump_Region(const char *prefix, const unsigned char *data,
		    unsigned length)
{
	unsigned count;
	char *output = Diag_msg;
	unsigned data_len;

	/* Build up the output string with multiple calls to sprintf() */
	output += sprintf(output, "%s (%08X,%u):", prefix, (uint32_t) data,
			  length);

	/* Restrict amount of data to dump */
	if (length > MAX_DUMP) {
		data_len = MAX_DUMP;
	} else {
		data_len = length;
	}

	for (count = 0; count < data_len; count++) {
		if (count % 4 == 0) {
			*output++ = ' ';
		}
		output += sprintf(output, "%02X", *data++);
	}

	LOG_DIAG(Diag_msg);
}

/*!
 * Dump given words of data to the log.
 *
 * @brief Dump data
 *
 * @param    prefix      Text to put in front of dumped data
 * @param    data        Kernel virtual address of start of region to dump
 * @param    word_count  Amount of data to dump
 *
 * @return   void
 */
void um_Dump_Words(const char *prefix, const unsigned *data,
		   unsigned word_count)
{
	char *output = Diag_msg;

	/* Build up the output string with multiple calls to sprintf() */
	output +=
	    sprintf(output, "%s (%08X,%uw): ", prefix, (uint32_t) data,
		    word_count);

	while (word_count--) {
		output += sprintf(output, "%08X ", *data++);
	}

	LOG_DIAG(Diag_msg);
}
#endif				/* DIAG_ADAPTOR */
