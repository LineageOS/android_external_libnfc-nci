/*****************************************************************************
**
**  Name:           nfa_fs_ci.h
**
**  Description:    This is the interface file for non valtile memory
**                  call-in functions.
**
**  Copyright (c) 2003-2009, Broadcom Corp., All Rights Reserved.
**  Widcomm Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#ifndef NFA_NV_CI_H
#define NFA_NV_CI_H

#include "nfa_nv_co.h"

/*****************************************************************************
**  Constants and data types
*****************************************************************************/

/* Read Ready Event */
typedef struct
{
    BT_HDR            hdr;
    tNFA_NV_CO_STATUS status;
    int               fd;
    UINT16            num_read;
} tNFA_NV_CI_READ_EVT;

/* Write Ready Event */
typedef struct
{
    BT_HDR            hdr;
    tNFA_NV_CO_STATUS status;
    int               fd;
} tNFA_NV_CI_WRITE_EVT;

/*****************************************************************************
**  Function Declarations
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
**
** Function         nfa_nv_ci_write
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
NFC_API extern void nfa_nv_ci_write (tNFA_NV_CO_STATUS status);

/*******************************************************************************
**
** Function         nfa_nv_ci_read
**
** Description      This function sends an event to NFA indicating the phone has
**                  read in the requested amount of data specified in the
**                  nfa_nv_co_read () call-out function.  It should only be called
**                  when the requested number of bytes has been read.
**                  
** Parameters       num_bytes_read - number of bytes read into the buffer
**                      specified in the read callout-function.
**                  status - NFA_NV_CO_OK if full buffer of data,
**                           NFA_NV_CO_EOF if the end of file has been reached,
**                           NFA_NV_CO_FAIL if an error has occurred.
**                  evt - Used Internally by NFA -> MUST be same value passed
**                       in call-out function.
**
** Returns          void 
**
*******************************************************************************/
NFC_API extern void nfa_nv_ci_read (UINT16            num_bytes_read,
                                    tNFA_NV_CO_STATUS status,
                                    UINT8             block);


#ifdef __cplusplus
}
#endif

#endif /* BTA_FS_CI_H */

