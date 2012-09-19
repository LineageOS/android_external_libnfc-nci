/*****************************************************************************
**
**  Name:           nfc_hal_nv_ci.h
**
**  Description:    This is the interface file for non valtile memory
**                  call-in functions.
**
**  Copyright (c) 2003-2009, Broadcom Corp., All Rights Reserved.
**  Widcomm Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#ifndef NFC_HAL_NV_CI_H
#define NFC_HAL_NV_CI_H

#include "nfc_hal_nv_co.h"


/*****************************************************************************
**  Function Declarations
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
**
** Function         nfc_hal_nv_ci_write
**
** Description      This function sends an event to NFAA indicating the phone
**                  has written the number of bytes specified in the call-out
**                  function, nfa_nv_co_write (), and is ready for more data.
**                  This function is used to control the TX data flow.
**                  Note: The data buffer is released by the stack aioer
**                        calling this function.
**
** Parameters       status - NFA_NV_CO_OK, NFA_NV_CO_NOSPACE, or NFA_NV_CO_FAIL
**                  evt - Used Internally by NFA -> MUST be same value passed
**                       in call-out function.
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_nv_ci_write (tNFC_HAL_NV_CO_STATUS status);

/*******************************************************************************
**
** Function         nfc_hal_nv_ci_read
**
** Description      This function sends an event to NCIT indicating the phone has
**                  read in the requested amount of data specified in the
**                  nfa_nv_co_read () call-out function.  It should only be called
**                  when the requested number of bytes has been read.
**
** Parameters       num_bytes_read - number of bytes read into the buffer
**                      specified in the read callout-function.
**                  status - NFC_HAL_NV_CO_OK if full buffer of data,
**                           NFC_HAL_NV_CO_EOF if the end of file has been reached,
**                           NFC_HAL_NV_CO_FAIL if an error has occurred.
**                  evt - Used Internally by NFA -> MUST be same value passed
**                       in call-out function.
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_nv_ci_read (UINT16                  num_bytes_read,
                         tNFC_HAL_NV_CO_STATUS   status,
                         UINT8                   block);


#ifdef __cplusplus
}
#endif

#endif /* NFC_HAL_NV_CI_H */

