/*****************************************************************************
**
**  Name:           nfa_mem_co.h
**
**  Description:    Callout functions for memory allocation/deallocatoin
**
**  Copyright (c) 2010, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#ifndef NFA_MEM_CO_H
#define NFA_MEM_CO_H

#include "nfc_target.h"

/*****************************************************************************
**  Constants and data types
*****************************************************************************/


/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
**
** Function         nfa_mem_co_alloc
**
** Description      allocate a buffer from platform's memory pool
**
** Returns:
**                  pointer to buffer if successful
**                  NULL otherwise
**
*******************************************************************************/
NFC_API extern void *nfa_mem_co_alloc (UINT32 num_bytes);


/*******************************************************************************
**
** Function         nfa_mem_co_free
**
** Description      free buffer previously allocated using nfa_mem_co_alloc
**
** Returns:
**                  Nothing
**
*******************************************************************************/
NFC_API extern void nfa_mem_co_free (void *p_buf);


#ifdef __cplusplus
}
#endif

#endif /* NFA_MEM_CO_H */

