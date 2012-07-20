/*****************************************************************************
**
**  Name:           nfa_hci_brcm.c
**
**  Description:    This file contains the Broadcom vendor specific functions
**                  for NFA-EE
**
**  Copyright (c) 2010-2011, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#include <string.h>
#include "nfc_api.h"
#include "nfa_sys.h"
#include "nfa_sys_int.h"
#include "nfa_api.h"
#include "nfa_dm_int.h"
#include "nfa_hci_api.h"
#include "nfa_hci_int.h"
#include "nfa_ee_api.h"
#include "nfa_ee_int.h"
#include "nfa_nv_co.h"
#include "nfa_mem_co.h"
#include "nfa_hci_defs.h"
#include "trace_api.h"
#if (NFC_BRCM_VS_INCLUDED == TRUE)
#include "nfc_brcm_api.h"
#include "nfa_brcm_api.h"
#include "nfa_brcm_int.h"

#ifndef NFA_HCI_BRCM_NV_READ_TIMEOUT_VAL
#define NFA_HCI_BRCM_NV_READ_TIMEOUT_VAL    1000
#endif

#ifndef NFA_HCI_BRCM_NFCC_RSP_TIMEOUT_VAL
#define NFA_HCI_BRCM_NFCC_RSP_TIMEOUT_VAL   3000
#endif

static void nfa_hci_set_next_hci_netwk_config (UINT8 block);
static void nfa_hci_brcm_handle_nv_read (UINT8 block, tNFA_STATUS status, UINT16 size);
static void nfa_hci_brcm_init_complete (tNFA_STATUS status);

/*******************************************************************************
**
** Function         nfa_hci_brcm_evt_hdlr
**
** Description      Processing event for NFA HCI
**                  
** Returns          None
**
*******************************************************************************/
static void nfa_hci_brcm_evt_hdlr (tNFA_HCI_EVENT_DATA *p_evt_data)
{
    tNFA_HCI_EVENT_DATA *p_msg;
    tNFA_EE_INFO        ee_info[3];
    UINT8               num_nfcee = 3;
    UINT8               nfcee_id;
    UINT8               count = 0;
    UINT8               active_host_bit_mask = 0;


    switch (p_evt_data->hdr.event)
    {
    case NFA_HCI_RSP_NV_READ_EVT:
        nfa_hci_brcm_handle_nv_read (p_evt_data->nv_read.block, p_evt_data->nv_read.status, p_evt_data->nv_read.size);
        break;

    case NFA_HCI_RSP_NV_WRITE_EVT:
        /* NV Ram write completed - nothing to do... */
        break;

    case NFA_HCI_VSC_TIMEOUT_EVT:
        NFA_TRACE_ERROR0 ("nfa_hci_brcm_evt_hdlr: Timeout - NFA HCI BRCM Initialization Failed!");
        nfa_hci_brcm_init_complete (NFA_STATUS_FAILED);
        break;

    case NFA_HCI_EE_DISC_CMPLT_EVT:
        if (p_evt_data->ee_disc_cmplt.b_disc_cmplt)
        {
            /* Discovery operation is complete, retrieve discovery result */
            NFA_EeGetInfo (&num_nfcee, ee_info);
            nfa_hci_cb.num_nfcee = num_nfcee;

            /* Collect active host in the Host Network */
            while (count < num_nfcee)
            {
                nfcee_id = (UINT8) (ee_info[count].ee_handle & 0x00FF);

                if(nfcee_id == NFA_HCI_UICC0_TARGET_HANDLE)
                    active_host_bit_mask |= 0x01;

                else if(nfcee_id == NFA_HCI_UICC1_TARGET_HANDLE)
                    active_host_bit_mask |= 0x02;

                count++;
            }
            nfa_hci_cb.inactive_host[0] = (active_host_bit_mask & 0x01) ? 0x00 : NFA_HCI_HOST_ID_UICC0;
            nfa_hci_cb.inactive_host[1] = (active_host_bit_mask & 0x02) ? 0x00 : NFA_HCI_HOST_ID_UICC1;

            if (nfa_hci_cb.hci_state == NFA_HCI_STATE_WAIT_NETWK_ENABLE)
            {
                nfa_sys_stop_timer (&nfa_hci_cb.timer);
                if ((p_msg = (tNFA_HCI_EVENT_DATA *) GKI_getbuf (sizeof (tNFA_HCI_EVENT_DATA))) != NULL)
                {
                    p_msg->hdr.event = NFA_HCI_RSP_TIMEOUT_EVT;
                    nfa_sys_sendmsg (p_msg);
                }
            }
            else
                nfa_hci_cb.w4_hci_netwk_init = FALSE;
        }
        break;

    case NFA_HCI_HCP_CONN_CREATE_EVT:
        if (nfa_hci_cb.buff_size > NCI_MAX_CTRL_SIZE)
            nfa_hci_cb.buff_size = NCI_MAX_CTRL_SIZE;
        break;

    default:
        break;
    }
}

/*******************************************************************************
**
** Function         nfa_hci_brcm_enable
**
** Description      Program nv data on to controller
**
** Returns          void                 
**
*******************************************************************************/
void nfa_hci_brcm_enable (void)
{
    if ((nfa_brcm_cb.p_hci_netwk_info_buf = (tNCI_HCI_NETWK *) GKI_getbuf (sizeof (tNCI_HCI_NETWK))) == NULL)
    {
        NFA_TRACE_ERROR0 ("nfa_hci_conn_cback: unable to allocate buffer for reading hci network info from nvram");
        nfa_hci_brcm_init_complete (NFA_STATUS_FAILED);
    }
    else
    {
        nfa_brcm_cb.hci_netwk_config_block = 0;
        memset (nfa_brcm_cb.p_hci_netwk_info_buf, 0, sizeof (tNCI_HCI_NETWK));
        nfa_nv_co_read ((UINT8 *) nfa_brcm_cb.p_hci_netwk_info_buf, sizeof (tNCI_HCI_NETWK), HC_F3_NV_BLOCK);
        nfa_sys_start_timer (&nfa_brcm_cb.hci_timer, NFA_HCI_VSC_TIMEOUT_EVT, NFA_HCI_BRCM_NV_READ_TIMEOUT_VAL);
    }
}

/*******************************************************************************
**
** Function         nfa_hci_brcm_init
**
** Description      Initialize the Broadcom control block for NFA-HCI
**
** Returns          void                 
**
*******************************************************************************/
void nfa_hci_brcm_init (void)
{
    /* data members for NFA-HCI */
    nfa_brcm_cb.p_hci_netwk_info_buf = NULL;
    nfa_brcm_cb.p_hci_netwk_dh_info_buf = NULL;
    nfa_brcm_cb.hci_netwk_config_block = 0;
    nfa_hci_cb.w4_vsc_init = TRUE;
    nfa_hci_cb.p_vs_evt_hdlr  = nfa_hci_brcm_evt_hdlr; 
}

/*******************************************************************************
**
** Function         nfa_hci_brcm_proc_nfcc_power_mode
**
** Description      Initialize BRCM HCI control block
**                  Initialize BRCM NFCC with VS configuration and CMD
**
** Returns          void                 
**
*******************************************************************************/
void nfa_hci_brcm_proc_nfcc_power_mode (UINT8 nfcc_power_mode)
{
    NFA_TRACE_DEBUG0 ("nfa_hci_brcm_proc_nfcc_power_mode ()");

    /* if NFCC power state is change to full power */
    if (nfcc_power_mode == NFA_DM_PWR_MODE_FULL)
    {
        nfa_hci_cb.w4_vsc_init = TRUE;

        if (nfa_brcm_cb.p_hci_netwk_dh_info_buf)
        {
            GKI_freebuf (nfa_brcm_cb.p_hci_netwk_dh_info_buf);
            nfa_brcm_cb.p_hci_netwk_dh_info_buf = NULL;
        }

        if (nfa_brcm_cb.p_hci_netwk_info_buf)
        {
            GKI_freebuf (nfa_brcm_cb.p_hci_netwk_info_buf);
            nfa_brcm_cb.p_hci_netwk_info_buf = NULL;
        }

        nfa_hci_brcm_enable ();
    }
    else
        nfa_sys_stop_timer (&nfa_brcm_cb.hci_timer);

}

/*******************************************************************************
**
** Function         nfa_hci_handle_hci_netwk_info
**
** Description      Handler function for HCI Network Notification
**
** Returns          None
**
*******************************************************************************/
void nfa_hci_handle_hci_netwk_info (UINT8 *p_data)
{
    UINT8  *p = p_data;
    UINT16 data_len;
    UINT8  target_handle;

    NFA_TRACE_DEBUG0 ("nfa_hci_handle_hci_netwk_info()");

    /* skip NCI header byte0 (MT,GID), byte1 (OID) */
    p += 2; 

    STREAM_TO_UINT8 (data_len, p);
    target_handle = *(UINT8 *) p;

    if (target_handle == NFA_HCI_DH_TARGET_HANDLE)
        nfa_nv_co_write (p, data_len,HC_DH_NV_BLOCK);

    else if (target_handle == NFA_HCI_UICC0_TARGET_HANDLE)
    {
        /* HCI Network notification received for UICC 0, Update nv data */
        nfa_nv_co_write (p, data_len,HC_F3_NV_BLOCK);
    }
    else if (target_handle == NFA_HCI_UICC1_TARGET_HANDLE)
    {
        /* HCI Network notification received for UICC 1, Update nv data */
        nfa_nv_co_write (p, data_len,HC_F4_NV_BLOCK);
    }
}

/*******************************************************************************
**
** Function         nfa_hci_brcm_handle_nv_read
**
** Description      handler function for nv read complete event
**                  
** Returns          None
**
*******************************************************************************/
void nfa_hci_brcm_handle_nv_read (UINT8 block, tNFA_STATUS status, UINT16 size)
{
    BT_HDR              *p_data     = NULL;
    UINT8               *p_buf;
    UINT8               *p;

    if (block == DH_NV_BLOCK)
    {
        return;
    }

    /* Stop timer as NVDATA Read Completed */
    nfa_sys_stop_timer (&nfa_brcm_cb.hci_timer);

    if (  ((block == HC_F3_NV_BLOCK) || (block == HC_F4_NV_BLOCK))
        &&(nfa_brcm_cb.p_hci_netwk_info_buf)
        &&(size <= sizeof (tNCI_HCI_NETWK))  )
    {
        if (status != NFA_STATUS_OK)
        {
            memset (nfa_brcm_cb.p_hci_netwk_info_buf,0,sizeof (tNCI_HCI_NETWK));
            nfa_brcm_cb.p_hci_netwk_info_buf->target_handle = (block == HC_F3_NV_BLOCK) ? NFA_HCI_UICC0_TARGET_HANDLE : NFA_HCI_UICC1_TARGET_HANDLE;
            memset (nfa_brcm_cb.p_hci_netwk_info_buf->session_id, 0xFF, NFA_HCI_SESSION_ID_LEN);
            size = sizeof (tNCI_HCI_NETWK);
        }

        /* We are in the middle of HCI Network configuration, Valid data read from NVRAM so send HCI Network ntf command */
        if ((p_buf = (UINT8 *) GKI_getbuf ((UINT16)(BT_HDR_SIZE + NCI_VSC_MSG_HDR_SIZE + size))) != NULL)
        {
            p_data          = (BT_HDR *) p_buf;
            p_data->offset  = NCI_VSC_MSG_HDR_SIZE;
            p_data->len     = size;
            p               = (UINT8 *) (p_data + 1) + p_data->offset;

            memcpy (p,(UINT8 *) nfa_brcm_cb.p_hci_netwk_info_buf,size);
            /* Send HCI Network command for UICC */
            NFC_SendVsCommand ((UINT8) NCI_MSG_HCI_NETWK, (BT_HDR *)p_data, nfa_hci_vsc_cback);
            nfa_brcm_cb.hci_netwk_config_block = block;
            nfa_sys_start_timer (&nfa_brcm_cb.hci_timer, NFA_HCI_VSC_TIMEOUT_EVT, NFA_HCI_BRCM_NFCC_RSP_TIMEOUT_VAL);
        }
        else
        {
            /* HCI Network Initialization failed */
            NFA_TRACE_ERROR1 ("nfa_hci_brcm_handle_nv_read: unable to allocate buffer to send VSC 0x%02x HCI network command",block|0xF0);
            /* Inform all applications on top */
            nfa_hci_brcm_init_complete (NFA_STATUS_FAILED);
        }
    }
    else if (  (status == NFA_STATUS_OK)
             &&(block == HC_DH_NV_BLOCK) 
             &&(size <= sizeof (tNCI_HCI_NETWK_DH))
             &&(nfa_brcm_cb.p_hci_netwk_dh_info_buf) )
    {
        /* We are in the middle of HCI Network configuration, Valid data read from NVRAM so send HCI Network ntf command */
        if ((p_buf = (UINT8 *) GKI_getbuf ((UINT16) (BT_HDR_SIZE + NCI_VSC_MSG_HDR_SIZE + size))) != NULL)
        {
            p_data          = (BT_HDR *) p_buf;
            p_data->offset  = NCI_VSC_MSG_HDR_SIZE;
            p_data->len     = size;
            p               = (UINT8 *) (p_data + 1) + p_data->offset;

            memcpy (p,(UINT8 *) nfa_brcm_cb.p_hci_netwk_dh_info_buf, size);
            /* Send HCI Network command for UICC */
            NFC_SendVsCommand ((UINT8) NCI_MSG_HCI_NETWK, (BT_HDR *)p_data, nfa_hci_vsc_cback);
            nfa_brcm_cb.hci_netwk_config_block = block;
            nfa_sys_start_timer (&nfa_brcm_cb.hci_timer, NFA_HCI_VSC_TIMEOUT_EVT, NFA_HCI_BRCM_NFCC_RSP_TIMEOUT_VAL);
        }
        else
        {
            /* HCI Network Initialization failed */
            NFA_TRACE_ERROR1 ("nfa_hci_brcm_handle_nv_read: unable to allocate buffer to send VSC 0x%02x HCI network command",block|0xF0);
            /* Inform all applications on top */
            nfa_hci_brcm_init_complete (NFA_STATUS_FAILED);
        }
    }
    else
    {
        /* We are in the middle of HCI Network configuration, Move to next configuration step */
        nfa_hci_set_next_hci_netwk_config (block);
    }
}

/*******************************************************************************
**
** Function         nfa_hci_brcm_init_complete
**
** Description      Notify VSC initialization is complete
**                  
** Returns          None
**
*******************************************************************************/
void nfa_hci_brcm_init_complete (tNFA_STATUS status)
{
    tNFA_HCI_EVENT_DATA *p_msg;

    if (nfa_brcm_cb.p_hci_netwk_dh_info_buf)
    {
        GKI_freebuf (nfa_brcm_cb.p_hci_netwk_dh_info_buf);
        nfa_brcm_cb.p_hci_netwk_dh_info_buf = NULL;
    }

    if (nfa_brcm_cb.p_hci_netwk_info_buf)
    {
        GKI_freebuf (nfa_brcm_cb.p_hci_netwk_info_buf);
        nfa_brcm_cb.p_hci_netwk_info_buf = NULL;
    }

    if (nfa_hci_cb.w4_vsc_init)
    {
        nfa_hci_cb.w4_vsc_init = FALSE;
        if ((p_msg = (tNFA_HCI_EVENT_DATA *) GKI_getbuf (sizeof (tNFA_HCI_EVENT_DATA))) != NULL)
        {
            p_msg->vsc_init.hdr.event = NFA_HCI_VSC_INIT_EVT;
            p_msg->vsc_init.status = status;
            nfa_sys_sendmsg (p_msg);
        }
    }
}

/*******************************************************************************
**
** Function         nfa_hci_set_next_hci_netwk_config
**
** Description      set next hci network configuration
**                  
** Returns          None
**
*******************************************************************************/
void nfa_hci_set_next_hci_netwk_config (UINT8 block)
{
    switch (block)
    {
    case HC_F3_NV_BLOCK:
        if (nfa_brcm_cb.p_hci_netwk_info_buf)
        {
            /* Send command to read nvram data for 0xF4 */
            memset (nfa_brcm_cb.p_hci_netwk_info_buf,0,sizeof (tNCI_HCI_NETWK));
            nfa_nv_co_read ((UINT8 *) nfa_brcm_cb.p_hci_netwk_info_buf, sizeof (tNCI_HCI_NETWK), HC_F4_NV_BLOCK);
            nfa_sys_start_timer (&nfa_brcm_cb.hci_timer, NFA_HCI_VSC_TIMEOUT_EVT, NFA_HCI_BRCM_NV_READ_TIMEOUT_VAL);
        }
        break;

    case HC_F4_NV_BLOCK:
        if ((nfa_brcm_cb.p_hci_netwk_dh_info_buf = (tNCI_HCI_NETWK_DH *) GKI_getbuf (sizeof (tNCI_HCI_NETWK_DH))) == NULL)
        {
            NFA_TRACE_ERROR0 ("nfa_hci_conn_cback: unable to allocate buffer for reading hci network info from nvram");
            nfa_hci_brcm_init_complete (NFA_STATUS_FAILED);
        }
        else
        {
            /* Send command to read nvram data for 0xF2 */
            memset (nfa_brcm_cb.p_hci_netwk_dh_info_buf,0,sizeof (tNCI_HCI_NETWK_DH));
            nfa_nv_co_read ((UINT8 *) nfa_brcm_cb.p_hci_netwk_dh_info_buf, sizeof (tNCI_HCI_NETWK_DH), HC_DH_NV_BLOCK);
            nfa_sys_start_timer (&nfa_brcm_cb.hci_timer, NFA_HCI_VSC_TIMEOUT_EVT, NFA_HCI_BRCM_NV_READ_TIMEOUT_VAL);
        }
        break;

    case HC_DH_NV_BLOCK:
        nfa_hci_brcm_init_complete (NFA_STATUS_OK);
        break;

    default:
        NFA_TRACE_ERROR1 ("nfa_hci_set_next_hci_netwk_config: unable to allocate buffer to send VSC 0x%02x", block);
        /* Brcm initialization failed */
        nfa_hci_brcm_init_complete (NFA_STATUS_FAILED);
        break;
    }
}

/*******************************************************************************
**
** Function         nfa_hci_vsc_cback
**
** Description      process VS callback event from stack  
**
** Returns          none
**
*******************************************************************************/
void nfa_hci_vsc_cback (tNFC_VS_EVT event, UINT16 data_len, UINT8 *p_data)
{
    UINT8 *p_ret = NULL;
    UINT8 status;
    
    p_ret  = p_data + NCI_MSG_HDR_SIZE;
    status = *p_ret;

    if (  (status != NFA_STATUS_OK)
        ||(event  != (NFC_VS_HCI_NETWK_RSP)) )
        return;

    switch (nfa_brcm_cb.hci_netwk_config_block)
    {
    case HC_F3_NV_BLOCK:
    case HC_F4_NV_BLOCK:
    case HC_DH_NV_BLOCK:
        nfa_sys_stop_timer (&nfa_brcm_cb.hci_timer);
        nfa_hci_set_next_hci_netwk_config (nfa_brcm_cb.hci_netwk_config_block);
        break;

    default:
        /* Ignore the event */
        break;
    }
}

/*******************************************************************************
**
** Function         NFA_HciAddStaticPipe
**
** Description      This function is called to add a static pipe for sending 
**                  7816 APDUs.
**
** Returns          NFA_STATUS_OK if successfully added
**                  NFA_STATUS_FAILED otherwise
**
*******************************************************************************/
tNFA_STATUS NFA_HciAddStaticPipe (tNFA_HANDLE hci_handle, UINT8 pipe)
{
    tNFA_HCI_DYN_GATE   *pg;
    tNFA_HCI_DYN_PIPE   *pp;
    UINT8               dest_host = (pipe == NFA_HCI_STATIC_PIPE_UICC0) ? NFA_HCI_HOST_ID_UICC0:NFA_HCI_HOST_ID_UICC1;
    tNFA_HCI_EVT_DATA   evt_data;
    tNFA_HANDLE         prev_owner;
    UINT8               dest_gate = (pipe == NFA_HCI_STATIC_PIPE_UICC0) ? NFA_HCI_PROPRIETARY_GATE0:NFA_HCI_PROPRIETARY_GATE1;

    NFA_TRACE_API2 ("NFA_HciAddStaticPipe (): hci_handle:0x%04x, pipe:0x%02x", hci_handle, pipe);


    /* Allocate a proprietary gate */
    if ((pg = nfa_hciu_alloc_gate (dest_gate, hci_handle)) != NULL)
    {
        prev_owner = pg->gate_owner;
        
        if (  (hci_handle != prev_owner)
            &&(pipe == NFA_HCI_STATIC_PIPE_UICC0) )
        {
            /* If any other application owned this gate, release its pipe on this gate and notify the application */
            if (nfa_hciu_release_pipe (NFA_HCI_STATIC_PIPE_UICC0) == NFA_HCI_ANY_OK)
            {
                evt_data.deleted.status = NFA_STATUS_OK;
                evt_data.deleted.pipe   = NFA_HCI_STATIC_PIPE_UICC0;
                nfa_hciu_send_to_app (NFA_HCI_DELETE_PIPE_EVT, &evt_data, prev_owner);
            }
        }
        
        if (  (hci_handle != prev_owner)
            &&(pipe == NFA_HCI_STATIC_PIPE_UICC1) )
        {
            /* If any other application owned this gate, release its pipe on this gate and notify the application */
            if (nfa_hciu_release_pipe (NFA_HCI_STATIC_PIPE_UICC1) == NFA_HCI_ANY_OK)
            {
                evt_data.deleted.status = NFA_STATUS_OK;
                evt_data.deleted.pipe   = NFA_HCI_STATIC_PIPE_UICC1;
                nfa_hciu_send_to_app (NFA_HCI_DELETE_PIPE_EVT, &evt_data, prev_owner);
            }
        }
        /* Assign new owner to the gate */
        pg->gate_owner = hci_handle;

        /* Add the dynamic pipe to the proprietary gate */
        if (nfa_hciu_add_pipe_to_gate (pipe,pg->gate_id,dest_host,dest_gate) != NFA_HCI_ANY_OK)
        {
            /* Unable to add the dynamic pipe, so release the gate */
            nfa_hciu_release_gate (pg->gate_id);
            return (NFA_STATUS_FAILED);
        }
        if ((pp = nfa_hciu_find_pipe_by_pid (pipe)) != NULL)
        {
            /* This pipe is always opened */
            pp->pipe_state = NFA_HCI_PIPE_OPENED;
            return (NFA_STATUS_OK);
        }
    }
    /* Unable to add static pipe */
    return (NFA_STATUS_FAILED);
}

#endif
