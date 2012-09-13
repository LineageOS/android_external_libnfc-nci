/*****************************************************************************
**
**  Name:           nfc_hal_hci_ci.c
**
**  Description:    This file contains the call-in functions for NFC HAL HCI
**
**  Copyright (c) 2010-2011, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#include <string.h>
#include "nfc_hal_api.h"
#include "nfc_hal_int.h"
#include "nfc_hal_nv_ci.h"
#include "nfc_hal_nv_co.h"

/*******************************************************************************
**
** Function         nfa_nv_ci_read
**
** Description      call-in function for non volatile memory read acess
**
** Returns          none
**
*******************************************************************************/
void nfc_hal_nv_ci_read (UINT16 num_bytes_read, tNFC_HAL_NV_CO_STATUS status, UINT8 block)
{
    tNFC_HAL_HCI_EVENT_DATA *p_msg;

    /* Send message to NCIT task */
    if ((p_msg = (tNFC_HAL_HCI_EVENT_DATA *) GKI_getbuf (sizeof (tNFC_HAL_HCI_EVENT_DATA))) != NULL)
    {
        p_msg->nv_read.hdr.event  = NFC_HAL_HCI_RSP_NV_READ_EVT;
        p_msg->hdr.offset         = 0;
        p_msg->hdr.len            = sizeof (tNFC_HAL_HCI_RSP_NV_READ_EVT);
        p_msg->hdr.layer_specific = 0;

        if (  (status == NFC_HAL_NV_CO_OK)
            &&(num_bytes_read != 0) )
        {
            p_msg->nv_read.status = HAL_NFC_STATUS_OK;
            p_msg->nv_read.size   = num_bytes_read;
        }
        else
            p_msg->nv_read.status = HAL_NFC_STATUS_FAILED;

        p_msg->nv_read.block = block;

        GKI_send_msg (NFC_HAL_TASK, NFC_HAL_TASK_MBOX, p_msg);
    }
}

/*******************************************************************************
**
** Function         nfa_nv_ci_write
**
** Description      call-in function for non volatile memory write acess
**
** Returns          none
**
*******************************************************************************/
void nfc_hal_nv_ci_write (tNFC_HAL_NV_CO_STATUS status)
{
    tNFC_HAL_HCI_EVENT_DATA *p_msg;

    if ((p_msg = (tNFC_HAL_HCI_EVENT_DATA *) GKI_getbuf (sizeof (tNFC_HAL_HCI_EVENT_DATA))) != NULL)
    {
        p_msg->nv_write.hdr.event          = NFC_HAL_HCI_RSP_NV_WRITE_EVT;
        p_msg->nv_write.hdr.offset         = 0;
        p_msg->nv_write.hdr.len            = sizeof (tNFC_HAL_HCI_RSP_NV_READ_EVT);
        p_msg->nv_write.hdr.layer_specific = 0;
        p_msg->nv_write.status             = HAL_NFC_STATUS_OK;

        GKI_send_msg (NFC_HAL_TASK, NFC_HAL_TASK_MBOX, p_msg);
    }
}

