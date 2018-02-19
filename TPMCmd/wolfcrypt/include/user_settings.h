/* settings.h
 *
 * Copyright (C) 2006-2017 wolfSSL Inc.
 *
 * This file is part of wolfSSL.
 *
 * wolfSSL is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * wolfSSL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1335, USA
 */


/* Place OS specific preprocessor flags, defines, includes here, will be
   included into every file because types.h includes it */


#ifndef WOLF_CRYPT_USER_SETTINGS_H
#define WOLF_CRYPT_USER_SETTINGS_H

#ifdef __cplusplus
    extern "C" {
#endif

#define NO_OLD_WC_NAMES
#define WOLFSSL_SHA384
#define WOLFSSL_SHA512

#define WOLFSSL_AES_DIRECT
// #if defined ALG_TDES && ALG_TDES == YES
#define WOLFSSL_DES_ECB

// integer mp_gcd (TPM_ALG_RSA)
#define WOLFSSL_KEY_GEN

#define HAVE_ECC
#define ECC_SHAMIR

#define USE_FAST_MATH
//#define FP_MAX_BITS LARGEST_NUMBER_BITS

#define WC_NO_HARDEN

#define WOLFSSL_PUBLIC_ECC_ADD_DBL 

#define LIBRARY_COMPATIBILITY_CHECK

/* Remove the automatic setting of the default I/O functions EmbedSend()
    and EmbedReceive(). */
#define WOLFSSL_USER_IO


#define WOLFSSL_

#ifdef __cplusplus
    }   /* extern "C" */
#endif

#endif
