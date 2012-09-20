/******************************************************************************
 *
 *  Copyright (C) 2012 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/******************************************************************************
 *
 *  Vendor-specific handler for HCI events
 *
 ******************************************************************************/
#include "gki.h"
#include "nfc_hal_api.h"
#include "nfc_hal_int.h"
#include "nfc_hal_nv_ci.h"
#include "nfc_hal_nv_co.h"

#include <string.h>
#include "nfc_hal_nv_co.h"

#ifndef NFC_HAL_HCI_NV_READ_TIMEOUT
#define NFC_HAL_HCI_NV_READ_TIMEOUT    1000
#endif

#ifndef NFC_HAL_HCI_NFCC_RSP_TIMEOUT
#define NFC_HAL_HCI_NFCC_RSP_TIMEOUT   3000
#endif

static void nfc_hal_hci_set_next_hci_netwk_config (UINT8 block);
static void nfc_hal_hci_handle_nv_read (UINT8 block, tHAL_NFC_STATUS status, UINT16 size);
static void nfc_hal_hci_init_complete (tHAL_NFC_STATUS status);
static void nfc_hal_hci_vsc_cback (tNFC_HAL_NCI_EVT event, UINT16 data_len, UINT8 *p_data);

/*******************************************************************************
**
** Function         nfc_hal_hci_evt_hdlr
**
** Description      Processing event for NFA HCI
**
** Returns          None
**
*******************************************************************************/
void nfc_hal_hci_evt_hdlr (tNFC_HAL_HCI_EVENT_DATA *p_evt_data)
{
    switch (p_evt_data->hdr.event)
    {
    case NFC_HAL_HCI_RSP_NV_READ_EVT:
        nfc_hal_hci_handle_nv_read (p_evt_data->nv_read.block, p_evt_data->nv_read.status, p_evt_data->nv_read.size);
        break;

    case NFC_HAL_HCI_RSP_NV_WRITE_EVT:
        /* NV Ram write completed - nothing to do... */
        break;

    case NFC_HAL_HCI_VSC_TIMEOUT_EVT:
        NCI_TRACE_ERROR0 ("nfc_hal_hci_evt_hdlr: Timeout - NFC HAL HCI BRCM Initialization Failed!");
        nfc_hal_hci_init_complete (HAL_NFC_STATUS_FAILED);
        break;

    default:
        break;
    }
}

/*******************************************************************************
**
** Function         nfc_hal_hci_enable
**
** Description      Program nv data on to controller
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_hci_enable (void)
{
    if (nfc_hal_cb.hci_cb.p_hci_netwk_dh_info_buf)
    {
        GKI_freebuf (nfc_hal_cb.hci_cb.p_hci_netwk_dh_info_buf);
        nfc_hal_cb.hci_cb.p_hci_netwk_dh_info_buf = NULL;
    }

    if (nfc_hal_cb.hci_cb.p_hci_netwk_info_buf)
    {
        GKI_freebuf (nfc_hal_cb.hci_cb.p_hci_netwk_info_buf);
        nfc_hal_cb.hci_cb.p_hci_netwk_info_buf = NULL;
    }

    if ((nfc_hal_cb.hci_cb.p_hci_netwk_info_buf = (tNCI_HCI_NETWK *) GKI_getbuf (sizeof (tNCI_HCI_NETWK))) == NULL)
    {
        NCI_TRACE_ERROR0 ("nfc_hal_hci_enable: unable to allocate buffer for reading hci network info from nvram");
        nfc_hal_hci_init_complete (HAL_NFC_STATUS_FAILED);
    }
    else
    {
        nfc_hal_cb.hci_cb.hci_netwk_config_block = 0;
        memset (nfc_hal_cb.hci_cb.p_hci_netwk_info_buf, 0, sizeof (tNCI_HCI_NETWK));
        nfc_hal_nv_co_read ((UINT8 *) nfc_hal_cb.hci_cb.p_hci_netwk_info_buf, sizeof (tNCI_HCI_NETWK), HC_F3_NV_BLOCK);
        nfc_hal_main_start_quick_timer (&nfc_hal_cb.hci_cb.hci_timer, NFC_HAL_HCI_VSC_TIMEOUT_EVT, NFC_HAL_HCI_NV_READ_TIMEOUT);
    }
}

/*******************************************************************************
**
** Function         nfc_hal_hci_handle_hci_netwk_info
**
** Description      Handler function for HCI Network Notification
**
** Returns          None
**
*******************************************************************************/
void nfc_hal_hci_handle_hci_netwk_info (UINT8 *p_data)
{
    UINT8  *p = p_data;
    UINT16 data_len;
    UINT8  target_handle;

    NCI_TRACE_DEBUG0 ("nfc_hal_hci_handle_hci_netwk_info()");

    /* skip NCI header byte0 (MT,GID), byte1 (OID) */
    p += 2;

    STREAM_TO_UINT8 (data_len, p);
    target_handle = *(UINT8 *) p;

    if (target_handle == NFC_HAL_HCI_DH_TARGET_HANDLE)
        nfc_hal_nv_co_write (p, data_len,HC_DH_NV_BLOCK);

    else if (target_handle == NFC_HAL_HCI_UICC0_TARGET_HANDLE)
    {
        /* HCI Network notification received for UICC 0, Update nv data */
        nfc_hal_nv_co_write (p, data_len,HC_F3_NV_BLOCK);
    }
    else if (target_handle == NFC_HAL_HCI_UICC1_TARGET_HANDLE)
    {
        /* HCI Network notification received for UICC 1, Update nv data */
        nfc_hal_nv_co_write (p, data_len,HC_F4_NV_BLOCK);
    }
}

/*******************************************************************************
**
** Function         nfc_hal_hci_handle_nv_read
**
** Description      handler function for nv read complete event
**
** Returns          None
**
*******************************************************************************/
void nfc_hal_hci_handle_nv_read (UINT8 block, tHAL_NFC_STATUS status, UINT16 size)
{
    NFC_HDR *p_data = NULL;
    UINT8   *p_buf;
    UINT8   buff[NCI_MSG_HDR_SIZE + 256];

    if (block == DH_NV_BLOCK)
    {
        return;
    }

    /* Stop timer as NVDATA Read Completed */
    nfc_hal_main_stop_quick_timer (&nfc_hal_cb.hci_cb.hci_timer);

    if (  ((block == HC_F3_NV_BLOCK) || (block == HC_F4_NV_BLOCK))
        &&(nfc_hal_cb.hci_cb.p_hci_netwk_info_buf)
        &&(size <= sizeof (tNCI_HCI_NETWK))  )
    {
        if (status != HAL_NFC_STATUS_OK)
        {
            memset (nfc_hal_cb.hci_cb.p_hci_netwk_info_buf,0,sizeof (tNCI_HCI_NETWK));
            nfc_hal_cb.hci_cb.p_hci_netwk_info_buf->target_handle = (block == HC_F3_NV_BLOCK) ? NFC_HAL_HCI_UICC0_TARGET_HANDLE : NFC_HAL_HCI_UICC1_TARGET_HANDLE;
            memset (nfc_hal_cb.hci_cb.p_hci_netwk_info_buf->session_id, 0xFF, NFC_HAL_HCI_SESSION_ID_LEN);
            size = sizeof (tNCI_HCI_NETWK);
        }

        /* We are in the middle of HCI Network configuration, Valid data read from NVRAM so send HCI Network ntf command */
        p_buf = buff;
        NCI_MSG_BLD_HDR0 (p_buf, NCI_MT_CMD, NCI_GID_PROP);
        NCI_MSG_BLD_HDR1 (p_buf, NCI_MSG_HCI_NETWK);
        UINT8_TO_STREAM  (p_buf, (UINT8) size);

        memcpy (p_buf, (UINT8 *) nfc_hal_cb.hci_cb.p_hci_netwk_info_buf, size);

        nfc_hal_dm_send_nci_cmd (buff, (UINT16) (NCI_MSG_HDR_SIZE + size), nfc_hal_hci_vsc_cback);

        nfc_hal_cb.hci_cb.hci_netwk_config_block = block;
        nfc_hal_main_start_quick_timer (&nfc_hal_cb.hci_cb.hci_timer, NFC_HAL_HCI_VSC_TIMEOUT_EVT, NFC_HAL_HCI_NFCC_RSP_TIMEOUT);
    }
    else if (  (status == HAL_NFC_STATUS_OK)
             &&(block == HC_DH_NV_BLOCK)
             &&(size <= sizeof (tNCI_HCI_NETWK_DH))
             &&(nfc_hal_cb.hci_cb.p_hci_netwk_dh_info_buf) )
    {
        /* We are in the middle of HCI Network configuration, Valid data read from NVRAM so send HCI Network ntf command */
        p_buf = buff;
        NCI_MSG_BLD_HDR0 (p_buf, NCI_MT_CMD, NCI_GID_PROP);
        NCI_MSG_BLD_HDR1 (p_buf, NCI_MSG_HCI_NETWK);
        UINT8_TO_STREAM  (p_buf, (UINT8) size);

        memcpy (p_buf,(UINT8 *) nfc_hal_cb.hci_cb.p_hci_netwk_dh_info_buf, size);

        nfc_hal_dm_send_nci_cmd (buff, (UINT16) (NCI_MSG_HDR_SIZE + size), nfc_hal_hci_vsc_cback);

        nfc_hal_cb.hci_cb.hci_netwk_config_block = block;
        nfc_hal_main_start_quick_timer (&nfc_hal_cb.hci_cb.hci_timer, NFC_HAL_HCI_VSC_TIMEOUT_EVT, NFC_HAL_HCI_NFCC_RSP_TIMEOUT);
    }
    else
    {
        /* We are in the middle of HCI Network configuration, Move to next configuration step */
        nfc_hal_hci_set_next_hci_netwk_config (block);
    }
}

/*******************************************************************************
**
** Function         nfc_hal_hci_init_complete
**
** Description      Notify VSC initialization is complete
**
** Returns          None
**
*******************************************************************************/
void nfc_hal_hci_init_complete (tHAL_NFC_STATUS status)
{
    if (nfc_hal_cb.hci_cb.p_hci_netwk_dh_info_buf)
    {
        GKI_freebuf (nfc_hal_cb.hci_cb.p_hci_netwk_dh_info_buf);
        nfc_hal_cb.hci_cb.p_hci_netwk_dh_info_buf = NULL;
    }

    if (nfc_hal_cb.hci_cb.p_hci_netwk_info_buf)
    {
        GKI_freebuf (nfc_hal_cb.hci_cb.p_hci_netwk_info_buf);
        nfc_hal_cb.hci_cb.p_hci_netwk_info_buf = NULL;
    }
    NFC_HAL_SET_INIT_STATE (NFC_HAL_INIT_STATE_IDLE);
    nfc_hal_cb.p_stack_cback (HAL_NFC_POST_INIT_CPLT_EVT, NULL);
}

/*******************************************************************************
**
** Function         nfc_hal_hci_set_next_hci_netwk_config
**
** Description      set next hci network configuration
**
** Returns          None
**
*******************************************************************************/
void nfc_hal_hci_set_next_hci_netwk_config (UINT8 block)
{
    switch (block)
    {
    case HC_F3_NV_BLOCK:
        if (nfc_hal_cb.hci_cb.p_hci_netwk_info_buf)
        {
            /* Send command to read nvram data for 0xF4 */
            memset (nfc_hal_cb.hci_cb.p_hci_netwk_info_buf,0,sizeof (tNCI_HCI_NETWK));
            nfc_hal_nv_co_read ((UINT8 *) nfc_hal_cb.hci_cb.p_hci_netwk_info_buf, sizeof (tNCI_HCI_NETWK), HC_F4_NV_BLOCK);
            nfc_hal_main_start_quick_timer (&nfc_hal_cb.hci_cb.hci_timer, NFC_HAL_HCI_VSC_TIMEOUT_EVT, NFC_HAL_HCI_NV_READ_TIMEOUT);
        }
        break;

    case HC_F4_NV_BLOCK:
        if ((nfc_hal_cb.hci_cb.p_hci_netwk_dh_info_buf = (tNCI_HCI_NETWK_DH *) GKI_getbuf (sizeof (tNCI_HCI_NETWK_DH))) == NULL)
        {
            NCI_TRACE_ERROR0 ("nfa_hci_conn_cback: unable to allocate buffer for reading hci network info from nvram");
            nfc_hal_hci_init_complete (HAL_NFC_STATUS_FAILED);
        }
        else
        {
            /* Send command to read nvram data for 0xF2 */
            memset (nfc_hal_cb.hci_cb.p_hci_netwk_dh_info_buf,0,sizeof (tNCI_HCI_NETWK_DH));
            nfc_hal_nv_co_read ((UINT8 *) nfc_hal_cb.hci_cb.p_hci_netwk_dh_info_buf, sizeof (tNCI_HCI_NETWK_DH), HC_DH_NV_BLOCK);
            nfc_hal_main_start_quick_timer (&nfc_hal_cb.hci_cb.hci_timer, NFC_HAL_HCI_VSC_TIMEOUT_EVT, NFC_HAL_HCI_NV_READ_TIMEOUT);
        }
        break;

    case HC_DH_NV_BLOCK:
        nfc_hal_hci_init_complete (HAL_NFC_STATUS_OK);
        break;

    default:
        NCI_TRACE_ERROR1 ("nfc_hal_hci_set_next_hci_netwk_config: unable to allocate buffer to send VSC 0x%02x", block);
        /* Brcm initialization failed */
        nfc_hal_hci_init_complete (HAL_NFC_STATUS_FAILED);
        break;
    }
}

/*******************************************************************************
**
** Function         nfc_hal_hci_vsc_cback
**
** Description      process VS callback event from stack
**
** Returns          none
**
*******************************************************************************/
static void nfc_hal_hci_vsc_cback (tNFC_HAL_NCI_EVT event, UINT16 data_len, UINT8 *p_data)
{
    UINT8 *p_ret = NULL;
    UINT8 status;

    p_ret  = p_data + NCI_MSG_HDR_SIZE;
    status = *p_ret;

    if (  (status != HAL_NFC_STATUS_OK)
        ||(event  != (NFC_VS_HCI_NETWK_RSP)) )
        return;

    switch (nfc_hal_cb.hci_cb.hci_netwk_config_block)
    {
    case HC_F3_NV_BLOCK:
    case HC_F4_NV_BLOCK:
    case HC_DH_NV_BLOCK:
        nfc_hal_main_stop_quick_timer (&nfc_hal_cb.hci_cb.hci_timer);
        nfc_hal_hci_set_next_hci_netwk_config (nfc_hal_cb.hci_cb.hci_netwk_config_block);
        break;

    default:
        /* Ignore the event */
        break;
    }
}

