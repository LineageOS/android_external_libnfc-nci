/*****************************************************************************
**
**  Name:           nfa_hci_main.c
**
**  Description:    This is the main implementation file for the NFA HCI.
**
**  Copyright (c) 2010-2011, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#include <string.h>
#include "nfc_api.h"
#include "nfa_sys.h"
#include "nfa_sys_int.h"
#include "nfa_dm_int.h"
#include "nfa_hci_api.h"
#include "nfa_hci_int.h"
#include "nfa_ee_api.h"
#include "nfa_ee_int.h"
#include "nfa_nv_co.h"
#include "nfa_mem_co.h"
#include "nfa_hci_defs.h"
#include "trace_api.h"


/*****************************************************************************
**  Global Variables
*****************************************************************************/

tNFA_HCI_CB nfa_hci_cb;

#ifndef NFA_HCI_NV_READ_TIMEOUT_VAL
#define NFA_HCI_NV_READ_TIMEOUT_VAL    1000
#endif

#ifndef NFA_HCI_CON_CREATE_TIMEOUT_VAL
#define NFA_HCI_CON_CREATE_TIMEOUT_VAL 1000
#endif

/*****************************************************************************
**  Static Functions
*****************************************************************************/

/* event handler function type */
static BOOLEAN nfa_hci_evt_hdlr (BT_HDR *p_msg);

static void nfa_hci_sys_enable (void);
static void nfa_hci_sys_disable (void);
static void nfa_hci_rsp_timeout (tNFA_HCI_EVENT_DATA *p_evt_data);
static void nfa_hci_conn_cback (UINT8 conn_id, tNFC_CONN_EVT event, tNFC_CONN *p_data);
static void nfa_hci_set_reassemble_buf (UINT8 pipe);
static tNFA_STATUS nfa_hci_assemble_msg (UINT8 pipe, UINT8 *p_data, UINT16 data_len);
static void nfa_hci_handle_nv_read (UINT8 block, tNFA_STATUS status);

/*****************************************************************************
**  Constants
*****************************************************************************/
static const tNFA_SYS_REG nfa_hci_sys_reg = 
{
    nfa_hci_sys_enable,
    nfa_hci_evt_hdlr, 
    nfa_hci_sys_disable,
    nfa_hci_proc_nfcc_power_mode
};

/*******************************************************************************
**
** Function         nfa_hci_ee_info_cback
**
** Description      Callback function
**
** Returns          None
**
*******************************************************************************/
void nfa_hci_ee_info_cback (BOOLEAN disable_discover)
{
    tNFA_HCI_EVENT_DATA *p_msg;

    NFA_TRACE_DEBUG0 ("nfa_hci_ee_info_cback ()");

    memset (nfa_hci_cb.inactive_host, 0, NFA_HCI_MAX_HOST_IN_NETWORK);

    /* Notify EE Discovery is complete */
    if ((p_msg = (tNFA_HCI_EVENT_DATA *) GKI_getbuf (sizeof (tNFA_HCI_EVENT_DATA))) != NULL)
    {
        p_msg->ee_disc_cmplt.hdr.event    = NFA_HCI_EE_DISC_CMPLT_EVT;
        p_msg->ee_disc_cmplt.b_disc_cmplt = disable_discover;
        p_msg->ee_disc_cmplt.status       = NFA_STATUS_OK;
        nfa_hci_evt_hdlr ((BT_HDR *) p_msg);
    }
}

/*******************************************************************************
**
** Function         nfa_hci_init
**
** Description      Initialize NFA HCI
**
** Returns          None
**
*******************************************************************************/
void nfa_hci_init (void)
{
    NFA_TRACE_DEBUG0 ("nfa_hci_init ()");

    /* initialize control block */
    memset (&nfa_hci_cb, 0, sizeof (tNFA_HCI_CB));

    nfa_hci_cb.hci_state = NFA_HCI_STATE_STARTUP;

    /* By Default, do not wait for any Vendor specific initialization or Vendor specific handling of EE discovery */
    nfa_hci_cb.w4_vsc_init = FALSE;

    /* register message handler on NFA SYS */
    nfa_sys_register (NFA_ID_HCI, &nfa_hci_sys_reg);
}

/*******************************************************************************
**
** Function         nfa_hci_is_valid_cfg
**
** Description      Validate hci control block config parameters
**
** Returns          None
**
*******************************************************************************/
BOOLEAN nfa_hci_is_valid_cfg (void)
{
    UINT8       xx,yy,zz;
    tNFA_HANDLE reg_app[NFA_HCI_MAX_APP_CB];
    UINT8       valid_gate[NFA_HCI_MAX_GATE_CB];
    UINT8       app_count       = 0;
    UINT8       gate_count      = 0;
    UINT32      pipe_inx_mask   = 0;

    /* First, see if valid values are stored in app names, send connectivity events flag */
    for (xx = 0; xx < NFA_HCI_MAX_APP_CB; xx++)
    {
        /* Check if app name is valid with null terminated string */
        if (strlen (&nfa_hci_cb.cfg.reg_app_names[xx][0]) > NFA_MAX_HCI_APP_NAME_LEN)
            return FALSE;

        /* Send Connectivity event flag can be either TRUE or FALSE */
        if (  (nfa_hci_cb.cfg.b_send_conn_evts[xx] != TRUE)
            &&(nfa_hci_cb.cfg.b_send_conn_evts[xx] != FALSE))
            return FALSE;

        if (nfa_hci_cb.cfg.reg_app_names[xx][0] != 0)
        {
            /* Check if the app name is present more than one time in the control block */
            for (yy = xx + 1; yy < NFA_HCI_MAX_APP_CB; yy++)
            {
                if (  (nfa_hci_cb.cfg.reg_app_names[yy][0] != 0)
                    &&(!strncmp (&nfa_hci_cb.cfg.reg_app_names[xx][0], &nfa_hci_cb.cfg.reg_app_names[yy][0], strlen (nfa_hci_cb.cfg.reg_app_names[xx]))) )
                {
                    /* Two app cannot have the same name , NVRAM is corrupted */
                    NFA_TRACE_EVENT2 ("nfa_hci_is_valid_cfg (%s)  Reusing: %u", &nfa_hci_cb.cfg.reg_app_names[xx][0], xx);
                    return FALSE;
                }
            }
            /* Collect list of hci handle */
            reg_app[app_count++] = (tNFA_HANDLE) (xx | NFA_HANDLE_GROUP_HCI);
        }
    }

    /* Validate Gate Control block */
    for (xx = 0; xx < NFA_HCI_MAX_GATE_CB; xx++)
    {
        if (nfa_hci_cb.cfg.dyn_gates[xx].gate_id != 0)
        {
            if (  (  (nfa_hci_cb.cfg.dyn_gates[xx].gate_id != NFA_HCI_LOOP_BACK_GATE)
                   &&(nfa_hci_cb.cfg.dyn_gates[xx].gate_id != NFA_HCI_IDENTITY_MANAGEMENT_GATE)
                   &&(nfa_hci_cb.cfg.dyn_gates[xx].gate_id < NFA_HCI_FIRST_DYNAMIC_GATE))
                ||(nfa_hci_cb.cfg.dyn_gates[xx].gate_id > NFA_HCI_LAST_DYNAMIC_GATE))
                return FALSE;

            /* Check if the same gate id is present more than once in the control block */
            for (yy = xx + 1; yy < NFA_HCI_MAX_GATE_CB; yy++)
            {
                if (  (nfa_hci_cb.cfg.dyn_gates[yy].gate_id != 0)
                    &&(nfa_hci_cb.cfg.dyn_gates[xx].gate_id == nfa_hci_cb.cfg.dyn_gates[yy].gate_id) )
                {
                    NFA_TRACE_EVENT1 ("nfa_hci_is_valid_cfg  Reusing: %u", nfa_hci_cb.cfg.dyn_gates[xx].gate_id);
                    return FALSE;
                }
            }
            if ((nfa_hci_cb.cfg.dyn_gates[xx].gate_owner & (~NFA_HANDLE_GROUP_HCI)) >= NFA_HCI_MAX_APP_CB)
            {
                NFA_TRACE_EVENT1 ("nfa_hci_is_valid_cfg  Invalid Gate owner: %u", nfa_hci_cb.cfg.dyn_gates[xx].gate_owner);
                return FALSE;
            }
            if (nfa_hci_cb.cfg.dyn_gates[xx].gate_id != NFA_HCI_CONNECTIVITY_GATE)
            {
                /* The gate owner should be one of the registered application */
                for (zz = 0; zz < app_count; zz++)
                {
                    if (nfa_hci_cb.cfg.dyn_gates[xx].gate_owner == reg_app[zz])
                        break;
                }
                if (zz == app_count)
                {
                    NFA_TRACE_EVENT1 ("nfa_hci_is_valid_cfg  Invalid Gate owner: %u", nfa_hci_cb.cfg.dyn_gates[xx].gate_owner);
                    return FALSE;
                }
            }
            /* Collect list of allocated gates */
            valid_gate[gate_count++] = nfa_hci_cb.cfg.dyn_gates[xx].gate_id;

            /* No two gates can own a same pipe */
            if ((pipe_inx_mask & nfa_hci_cb.cfg.dyn_gates[xx].pipe_inx_mask) != 0)
                return FALSE;
            /* Collect the list of pipes on this gate */
            pipe_inx_mask |= nfa_hci_cb.cfg.dyn_gates[xx].pipe_inx_mask;
        }
    }

    for (xx = 0; (pipe_inx_mask && (xx < NFA_HCI_MAX_PIPE_CB)); xx++,pipe_inx_mask >>= 1)
    {
        /* Every bit set in pipe increment mask indicates a valid pipe */
        if (pipe_inx_mask & 1)
        {
            /* Check if the pipe is valid one */
            if (nfa_hci_cb.cfg.dyn_pipes[xx].pipe_id < NFA_HCI_FIRST_DYNAMIC_PIPE)               
                return FALSE;
        }
    }

    if (xx == NFA_HCI_MAX_PIPE_CB)
        return FALSE;

    /* Validate Gate Control block */
    for (xx = 0; xx < NFA_HCI_MAX_PIPE_CB; xx++)
    {
        if (nfa_hci_cb.cfg.dyn_pipes[xx].pipe_id != 0)
        {
            /* Check if pipe id is valid */
            if (nfa_hci_cb.cfg.dyn_pipes[xx].pipe_id < NFA_HCI_FIRST_DYNAMIC_PIPE)
                return FALSE;

            /* Check if pipe state is valid */
            if (  (nfa_hci_cb.cfg.dyn_pipes[xx].pipe_state != NFA_HCI_PIPE_OPENED)
                &&(nfa_hci_cb.cfg.dyn_pipes[xx].pipe_state != NFA_HCI_PIPE_CLOSED))
                return FALSE;

            /* Check if local gate on which the pipe is created is valid */
            if (  (((nfa_hci_cb.cfg.dyn_pipes[xx].local_gate != NFA_HCI_LOOP_BACK_GATE) && (nfa_hci_cb.cfg.dyn_pipes[xx].local_gate != NFA_HCI_IDENTITY_MANAGEMENT_GATE)) && (nfa_hci_cb.cfg.dyn_pipes[xx].local_gate < NFA_HCI_FIRST_DYNAMIC_GATE))
                ||(nfa_hci_cb.cfg.dyn_pipes[xx].local_gate > NFA_HCI_LAST_DYNAMIC_GATE))
                return FALSE;

            /* Check if the peer gate on which the pipe is created is valid */
            if (  (((nfa_hci_cb.cfg.dyn_pipes[xx].dest_gate != NFA_HCI_LOOP_BACK_GATE) && (nfa_hci_cb.cfg.dyn_pipes[xx].dest_gate != NFA_HCI_IDENTITY_MANAGEMENT_GATE)) && (nfa_hci_cb.cfg.dyn_pipes[xx].dest_gate < NFA_HCI_FIRST_DYNAMIC_GATE))
                ||(nfa_hci_cb.cfg.dyn_pipes[xx].dest_gate > NFA_HCI_LAST_DYNAMIC_GATE))
                return FALSE;

            /* Check if the same pipe is present more than once in the control block */
            for (yy = xx + 1; yy < NFA_HCI_MAX_PIPE_CB; yy++)
            {
                if (  (nfa_hci_cb.cfg.dyn_pipes[yy].pipe_id != 0)
                    &&(nfa_hci_cb.cfg.dyn_pipes[xx].pipe_id == nfa_hci_cb.cfg.dyn_pipes[yy].pipe_id) )
                {
                    NFA_TRACE_EVENT1 ("nfa_hci_is_valid_cfg  Reusing: %u", nfa_hci_cb.cfg.dyn_pipes[xx].pipe_id);
                    return FALSE;
                }
            }
            /* The local gate should be one of the element in gate control block */
            for (zz = 0; zz < gate_count; zz++)
            {
                if (nfa_hci_cb.cfg.dyn_pipes[xx].local_gate == valid_gate[zz])
                    break;
            }
            if (zz == gate_count)
            {
                NFA_TRACE_EVENT1 ("nfa_hci_is_valid_cfg  Invalid Gate: %u", nfa_hci_cb.cfg.dyn_pipes[xx].local_gate);
                return FALSE;
            }
        }
    }

    /* Check if admin pipe state is valid */
    if (  (nfa_hci_cb.cfg.admin_gate.pipe01_state != NFA_HCI_PIPE_OPENED)
        &&(nfa_hci_cb.cfg.admin_gate.pipe01_state != NFA_HCI_PIPE_CLOSED))
        return FALSE;

    /* Check if link management pipe state is valid */
    if (  (nfa_hci_cb.cfg.link_mgmt_gate.pipe00_state != NFA_HCI_PIPE_OPENED)
        &&(nfa_hci_cb.cfg.link_mgmt_gate.pipe00_state != NFA_HCI_PIPE_CLOSED))
        return FALSE;

    pipe_inx_mask = nfa_hci_cb.cfg.id_mgmt_gate.pipe_inx_mask;
    for (xx = 0; (pipe_inx_mask && (xx < NFA_HCI_MAX_PIPE_CB)); xx++,pipe_inx_mask >>= 1)
    {
        /* Every bit set in pipe increment mask indicates a valid pipe */
        if (pipe_inx_mask & 1)
        {
            /* Check if the pipe is valid one */
            if (nfa_hci_cb.cfg.dyn_pipes[xx].pipe_id < NFA_HCI_FIRST_DYNAMIC_PIPE)
                return FALSE;
            /* Check if the pipe is connected to Identity management gate */
            if (nfa_hci_cb.cfg.dyn_pipes[xx].local_gate != NFA_HCI_IDENTITY_MANAGEMENT_GATE)
                return FALSE;
        }
    }
    if (xx == NFA_HCI_MAX_PIPE_CB)
        return FALSE;

    return TRUE;
}

/*******************************************************************************
**
** Function         nfa_hci_cfg_default
**
** Description      Configure default values for hci control block
**
** Returns          None
**
*******************************************************************************/
void nfa_hci_restore_default_config (UINT64 session_id)
{
    memset (&nfa_hci_cb.cfg, 0, sizeof (nfa_hci_cb.cfg));
    nfa_hci_cb.cfg.admin_gate.session_id = session_id;
    nfa_hci_cb.nv_write_needed = TRUE;
}

/*******************************************************************************
**
** Function         nfa_hci_proc_nfcc_power_mode
**
** Description      Restore NFA HCI sub-module
**                  
** Returns          None
**
*******************************************************************************/
void nfa_hci_proc_nfcc_power_mode (UINT8 nfcc_power_mode)
{
    NFA_TRACE_DEBUG1 ("nfa_hci_proc_nfcc_power_mode () nfcc_power_mode=%d", nfcc_power_mode);

    /* if NFCC power mode is change to full power */
    if (nfcc_power_mode == NFA_DM_PWR_MODE_FULL)
    {
        nfa_hci_cb.b_low_power_mode = FALSE;
        if (nfa_hci_cb.hci_state == NFA_HCI_STATE_IDLE)
        {
            nfa_hci_cb.hci_state     = NFA_HCI_STATE_RESTORE;
            nfa_hci_cb.ee_disc_cmplt = FALSE;
        }
        else
        {
            NFA_TRACE_ERROR0 ("nfa_hci_proc_nfcc_power_mode (): Cannot restore now");
            nfa_sys_cback_notify_nfcc_power_mode_proc_complete (NFA_ID_HCI);
        }
    }
    else
    {
        nfa_sys_stop_timer (&nfa_hci_cb.timer);
        nfa_hci_cb.b_low_power_mode = TRUE;
        nfa_sys_cback_notify_nfcc_power_mode_proc_complete (NFA_ID_HCI);
    }
}

/*******************************************************************************
**
** Function         nfa_hci_dh_startup_complete
**
** Description      Initialization of terminal host in HCI Network is completed
**                  Wait for other host in the network to initialize
**                  
** Returns          None
**
*******************************************************************************/
void nfa_hci_dh_startup_complete (void)
{
    if (nfa_hci_cb.w4_hci_netwk_init)
    {
        nfa_hci_cb.hci_state = NFA_HCI_STATE_WAIT_NETWK_ENABLE;
        /* No HCP packet to DH for a specified period of time indicates all host in the network is initialized */
        nfa_sys_start_timer (&nfa_hci_cb.timer, NFA_HCI_RSP_TIMEOUT_EVT, NFA_HCI_NETWK_INIT_TIMEOUT);
    }
    else
        nfa_hci_startup_complete (NFA_STATUS_OK);
}

/*******************************************************************************
**
** Function         nfa_hci_startup_complete
**
** Description      HCI network initialization is completed
**                  
** Returns          None
**
*******************************************************************************/
void nfa_hci_startup_complete (tNFA_STATUS status)
{
    tNFA_HCI_EVT_DATA   evt_data; 

    NFA_TRACE_EVENT1 ("nfa_hci_startup_complete (): Status: %u", status);

    nfa_sys_stop_timer (&nfa_hci_cb.timer);

    if (nfa_hci_cb.hci_state == NFA_HCI_STATE_RESTORE)
    {
        nfa_ee_proc_hci_info_cback ();
        nfa_sys_cback_notify_nfcc_power_mode_proc_complete (NFA_ID_HCI);
    }
    else
    {
        evt_data.hci_init.status = status;

        nfa_hciu_send_to_all_apps (NFA_HCI_INIT_EVT, &evt_data);
        nfa_sys_cback_notify_enable_complete (NFA_ID_HCI);
    }

    if (status == NFA_STATUS_OK) 
        nfa_hci_cb.hci_state = NFA_HCI_STATE_IDLE;

    else
        nfa_hci_cb.hci_state = NFA_HCI_STATE_DISABLED;
}

/*******************************************************************************
**
** Function         nfa_hci_startup
**
** Description      Perform HCI startup
**
** Returns          None
**
*******************************************************************************/
void nfa_hci_startup (void)
{
    tNFA_STATUS     status = NFA_STATUS_FAILED;
    tNFA_EE_INFO    ee_info[2];
    UINT8           num_nfcee = 2;
    UINT8           target_handle;
    UINT8           count = 0;
    BOOLEAN         found = FALSE;

    if (HCI_LOOPBACK_DEBUG)
    {
        /* First step in initialization is to open the admin pipe */
        nfa_hciu_send_open_pipe_cmd (NFA_HCI_ADMIN_PIPE);
        return;
    }

    /* We can only start up if NV Ram is read and EE discovery is complete */
    if (nfa_hci_cb.nv_read_cmplt && nfa_hci_cb.ee_disc_cmplt && !nfa_hci_cb.w4_vsc_init)
    {
        NFA_EeGetInfo (&num_nfcee, ee_info);
        nfa_hci_cb.num_nfcee = num_nfcee;

        while ((count < num_nfcee) && (!found))
        {
            target_handle = (UINT8) ee_info[count].ee_handle;

            if(ee_info[count].ee_interface[0] == NFA_EE_INTERFACE_HCI_ACCESS)
            {
                found = TRUE;

                if (ee_info[count].ee_status == NFA_EE_STATUS_INACTIVE)
                {
                    NFC_NfceeModeSet (target_handle, NFC_MODE_ACTIVATE);
                }
                if ((status = NFC_ConnCreate (NCI_DEST_TYPE_NFCEE, target_handle, NFA_EE_INTERFACE_HCI_ACCESS, nfa_hci_conn_cback)) == NFA_STATUS_OK)
                    nfa_sys_start_timer (&nfa_hci_cb.timer, NFA_HCI_RSP_TIMEOUT_EVT, NFA_HCI_CON_CREATE_TIMEOUT_VAL);
                else
                {
                    nfa_hci_cb.hci_state = NFA_HCI_STATE_DISABLED;
                    NFA_TRACE_ERROR0 ("nfa_hci_startup - Failed to Create Logical connection. HCI Initialization/Restore failed");
                    nfa_hci_startup_complete (NFA_STATUS_FAILED);
                }
            }
            count++;
        }
        if (!found)
        {
            NFA_TRACE_ERROR0 ("nfa_hci_startup - HCI ACCESS Interface not discovered. HCI Initialization/Restore failed");
            nfa_hci_startup_complete (NFA_STATUS_FAILED);
        }
    }
}

/*******************************************************************************
**
** Function         nfa_hci_sys_enable
**
** Description      Enable NFA HCI
**
** Returns          None
**
*******************************************************************************/
static void nfa_hci_sys_enable (void)
{
    NFA_TRACE_DEBUG0 ("nfa_hci_sys_enable ()");
    nfa_ee_reg_cback_enable_done (&nfa_hci_ee_info_cback);

    nfa_nv_co_read ((UINT8 *)&nfa_hci_cb.cfg, sizeof (nfa_hci_cb.cfg),DH_NV_BLOCK);
    nfa_sys_start_timer (&nfa_hci_cb.timer, NFA_HCI_RSP_TIMEOUT_EVT, NFA_HCI_NV_READ_TIMEOUT_VAL);
}

/*******************************************************************************
**
** Function         nfa_hci_sys_disable
**
** Description      Disable NFA HCI
**
** Returns          None
**
*******************************************************************************/
static void nfa_hci_sys_disable (void)
{
    tNFA_HCI_EVT_DATA   evt_data;

    nfa_sys_stop_timer (&nfa_hci_cb.timer);

    if (nfa_hci_cb.conn_id)
    {
        if (nfa_sys_is_graceful_disable ())
        {
            /* Tell all applications stack is down */
            nfa_hciu_send_to_all_apps (NFA_HCI_EXIT_EVT, &evt_data);
            NFC_ConnClose (nfa_hci_cb.conn_id);
            return;
        }
        nfa_hci_cb.conn_id = 0;
    }

    nfa_hci_cb.hci_state = NFA_HCI_STATE_DISABLED;
    /* deregister message handler on NFA SYS */
    nfa_sys_deregister (NFA_ID_HCI);
}

/*******************************************************************************
**
** Function         nfa_hci_conn_cback
**
** Description      This function Process event from NCI
**                  
** Returns          None
**
*******************************************************************************/
static void nfa_hci_conn_cback (UINT8 conn_id, tNFC_CONN_EVT event, tNFC_CONN *p_data)
{
    UINT8   *p;
    BT_HDR  *p_pkt = (BT_HDR *) p_data->data.p_data;
    UINT8   chaining_bit;
    UINT8   pipe;
    UINT16  pkt_len;
    tNFA_HCI_EVENT_DATA *p_msg;

    if (event == NFC_CONN_CREATE_CEVT)
    {
        nfa_hci_cb.conn_id   = conn_id;
        nfa_hci_cb.buff_size = p_data->conn_create.buff_size;

        /* Notify HCP connection is created */
        if ((p_msg = (tNFA_HCI_EVENT_DATA *) GKI_getbuf (sizeof (tNFA_HCI_EVENT_DATA))) != NULL)
        {
            p_msg->conn_create.hdr.event    = NFA_HCI_HCP_CONN_CREATE_EVT;
            p_msg->conn_create.status       = NFA_STATUS_OK;
            nfa_hci_evt_hdlr ((BT_HDR *) p_msg);
        }

        if (nfa_hci_cb.hci_state == NFA_HCI_STATE_STARTUP)
        {
            nfa_hci_cb.w4_hci_netwk_init = TRUE;
            nfa_hciu_alloc_gate (NFA_HCI_CONNECTIVITY_GATE,0);
        }

        if (nfa_hci_cb.cfg.admin_gate.pipe01_state == NFA_HCI_PIPE_CLOSED)
        {
            /* First step in initialization/restore is to open the admin pipe */
            nfa_hciu_send_open_pipe_cmd (NFA_HCI_ADMIN_PIPE);
        }
        else
        {
            /* Read session id, to know DH session id is correct */
            nfa_hciu_send_get_param_cmd (NFA_HCI_ADMIN_PIPE, NFA_HCI_SESSION_IDENTITY_INDEX);
        }
    }
    else if (event == NFC_CONN_CLOSE_CEVT)
    {
        nfa_hci_cb.conn_id   = 0;
        nfa_hci_cb.hci_state = NFA_HCI_STATE_DISABLED;
        /* deregister message handler on NFA SYS */
        nfa_sys_deregister (NFA_ID_HCI);
    }

    if ((event != NFC_DATA_CEVT) || (p_pkt == NULL))
            return;

    if (nfa_hci_cb.hci_state == NFA_HCI_STATE_WAIT_NETWK_ENABLE)
    {
        /* Received HCP Packet before timeout, Other Host initialization is not complete */
        nfa_sys_stop_timer (&nfa_hci_cb.timer);
        nfa_sys_start_timer (&nfa_hci_cb.timer, NFA_HCI_RSP_TIMEOUT_EVT, NFA_HCI_NETWK_INIT_TIMEOUT);
    }

    p       = (UINT8 *) (p_pkt + 1) + p_pkt->offset;
    pkt_len = p_pkt->len;

#if (BT_TRACE_PROTOCOL == TRUE)
    DispHcp (p, pkt_len, TRUE, (BOOLEAN) !nfa_hci_cb.assembling);
#endif

    chaining_bit = ((*p) >> 0x07) & 0x01;
    pipe = (*p++) & 0x7F;
    if (pkt_len != 0)
        pkt_len--;

    if (nfa_hci_cb.assembling == FALSE)
    {
        nfa_hci_cb.type       = ((*p) >> 0x06) & 0x03;
        nfa_hci_cb.inst       = (*p & 0x3F);

        if (chaining_bit == NFA_HCI_MESSAGE_FRAGMENTATION)
        {
            nfa_hci_cb.assembling = TRUE;
            nfa_hci_cb.msg_len    = 0;
            nfa_hci_set_reassemble_buf (pipe);

            nfa_hci_assemble_msg (pipe, p, pkt_len);
        }
    }
    else
    {
        nfa_hci_assemble_msg (pipe, p, pkt_len);

        if (chaining_bit == NFA_HCI_NO_MESSAGE_FRAGMENTATION)
        {
            /* Just added the last segment in the chain. Reset pointers */
            nfa_hci_cb.assembling = FALSE;
            p                     = nfa_hci_cb.p_msg_data;
            pkt_len               = nfa_hci_cb.msg_len;
        }
    }

#if (BT_TRACE_VERBOSE == TRUE)
    NFA_TRACE_EVENT5 ("nfa_hci_conn_cback Recvd data pipe:%d  %s  chain:%d  assmbl:%d  len:%d",
                      (UINT8)pipe, nfa_hciu_get_type_inst_names (pipe, nfa_hci_cb.type, nfa_hci_cb.inst),
                      (UINT8)chaining_bit, (UINT8)nfa_hci_cb.assembling, p_pkt->len);
#else
    NFA_TRACE_EVENT6 ("nfa_hci_conn_cback Recvd data pipe:%d  Type: %u  Inst: %u  chain:%d reassm:%d len:%d",
                      pipe, nfa_hci_cb.type, nfa_hci_cb.inst, chaining_bit, nfa_hci_cb.assembling, p_pkt->len);
#endif


    /* If still reassembling fragments, just return */
    if (nfa_hci_cb.assembling)
    {
        GKI_freebuf (p_pkt);
        return;
    }

    /* If we got a response, cancel the response timer. Also, if waiting for */
    /* a single response, we can go back to idle state                       */
    if (nfa_hci_cb.type == NFA_HCI_RESPONSE_TYPE)
    {
        nfa_sys_stop_timer (&nfa_hci_cb.timer);

        if (nfa_hci_cb.hci_state == NFA_HCI_STATE_WAIT_RSP)
            nfa_hci_cb.hci_state = NFA_HCI_STATE_IDLE;
    }

    switch (pipe)
    {
    case NFA_HCI_ADMIN_PIPE:
        /* Check if data packet is a command, response or event */
        if (nfa_hci_cb.type == NFA_HCI_COMMAND_TYPE)
        {
            nfa_hci_handle_admin_gate_cmd (p, (UINT8) pkt_len);
        }
            else if (nfa_hci_cb.type == NFA_HCI_RESPONSE_TYPE)
        {
            nfa_hci_handle_admin_gate_rsp (p, (UINT8) pkt_len);
        }
        else if (nfa_hci_cb.type == NFA_HCI_EVENT_TYPE)
        {
            nfa_hci_handle_admin_gate_evt (p, (UINT8) pkt_len);
        }
        break;

    case NFA_HCI_LINK_MANAGEMENT_PIPE:
        /* We don't send Link Management commands, we only get them */
        if (nfa_hci_cb.type == NFA_HCI_COMMAND_TYPE)
            nfa_hci_handle_link_mgm_gate_cmd (p, (UINT8) pkt_len);
        break;

    default:
        if (pipe >= NFA_HCI_FIRST_DYNAMIC_PIPE)
            nfa_hci_handle_dyn_pipe_pkt (pipe, p, nfa_hci_cb.type, pkt_len);
        break;
    }

    /* Send a message to ouselves to check for anything to do */
    p_pkt->event = NFA_HCI_CHECK_QUEUE_EVT;
    p_pkt->len   = 0;
    nfa_sys_sendmsg (p_pkt);
}

/*******************************************************************************
**
** Function         nfa_hci_handle_nv_read
**
** Description      handler function for nv read complete event
**                  
** Returns          None
**
*******************************************************************************/
void nfa_hci_handle_nv_read (UINT8 block, tNFA_STATUS status)
{
    UINT64 session_id;

    if (block == DH_NV_BLOCK)
    {
        /* Stop timer as NVDATA Read Completed */
        nfa_sys_stop_timer (&nfa_hci_cb.timer);
        nfa_hci_cb.nv_read_cmplt = TRUE;
        if (  (status != NFA_STATUS_OK)
            ||(!nfa_hci_is_valid_cfg ())
            ||(nfa_hci_cb.cfg.admin_gate.session_id == (UINT64) NFA_HCI_DEFAULT_SESSION)
            ||(nfa_hci_cb.cfg.admin_gate.session_id == 0) )
        {
            /* Set a new session id so that we clear all pipes later after seeing a difference with the HC Session ID */
            session_id  = nfa_hci_cb.cfg.admin_gate.session_id << 32;
            session_id |= GKI_get_os_tick_count ();
            nfa_hci_restore_default_config (session_id);
        }
        nfa_hci_startup ();
    }
}

/*******************************************************************************
**
** Function         nfa_hci_rsp_timeout
**
** Description      action function to process timeout
**                  
** Returns          None
**
*******************************************************************************/
void nfa_hci_rsp_timeout (tNFA_HCI_EVENT_DATA *p_evt_data)
{
    tNFA_HCI_EVT        evt = 0;
    tNFA_HCI_EVT_DATA   evt_data;

    NFA_TRACE_EVENT2 ("nfa_hci_rsp_timeout () State: %u  Cmd: %u", nfa_hci_cb.hci_state, nfa_hci_cb.cmd_sent);

    evt_data.status      = NFA_STATUS_FAILED;

    switch (nfa_hci_cb.hci_state)
    {
    case NFA_HCI_STATE_STARTUP:
    case NFA_HCI_STATE_RESTORE:
        NFA_TRACE_ERROR0 ("nfa_hci_rsp_timeout - Initialization failed!");
        nfa_hci_startup_complete (NFA_STATUS_TIMEOUT);
        break;

    case NFA_HCI_STATE_WAIT_NETWK_ENABLE:
        /* HCI Network is enabled */
        nfa_hci_cb.w4_hci_netwk_init = FALSE;
        nfa_hci_startup_complete (NFA_STATUS_OK);
        break;

    case NFA_HCI_STATE_REMOVE_GATE:
        /* Something wrong, NVRAM data could be corrupt */
        if (nfa_hci_cb.cmd_sent == NFA_HCI_ADM_DELETE_PIPE)
        {
            nfa_hciu_send_clear_all_pipe_cmd ();
        }
        else
        {
            nfa_hciu_remove_all_pipes_from_host (0);
            nfa_hci_api_dealloc_gate (NULL);
        }
        break;

    case NFA_HCI_STATE_APP_DEREGISTER:
        /* Something wrong, NVRAM data could be corrupt */
        if (nfa_hci_cb.cmd_sent == NFA_HCI_ADM_DELETE_PIPE)
        {
            nfa_hciu_send_clear_all_pipe_cmd ();
        }
        else
        {
            nfa_hciu_remove_all_pipes_from_host (0);
            nfa_hci_api_deregister (NULL);
        }
        break;
 
    case NFA_HCI_STATE_WAIT_RSP:
        nfa_hci_cb.hci_state = NFA_HCI_STATE_IDLE;
        

        switch (nfa_hci_cb.cmd_sent)
        {
        case NFA_HCI_ANY_SET_PARAMETER:
            /* 
             * As no response to the command sent on this pipe, we may assume the pipe is 
             * deleted already and release the pipe. But still send delete pipe command to be safe.
             */
            nfa_hciu_send_delete_pipe_cmd (nfa_hci_cb.pipe_in_use);
            nfa_hciu_release_pipe (nfa_hci_cb.pipe_in_use);
            evt_data.registry.pipe     = nfa_hci_cb.pipe_in_use;
            evt_data.registry.data_len = 0;
            evt_data.registry.index    = nfa_hci_cb.param_in_use;
            evt                        = NFA_HCI_SET_REG_CMD_EVT;
            break;

        case NFA_HCI_ANY_GET_PARAMETER:
            /* 
             * As no response to the command sent on this pipe, we may assume the pipe is 
             * deleted already and release the pipe. But still send delete pipe command to be safe.
             */
            nfa_hciu_send_delete_pipe_cmd (nfa_hci_cb.pipe_in_use);
            nfa_hciu_release_pipe (nfa_hci_cb.pipe_in_use);
            evt_data.registry.pipe     = nfa_hci_cb.pipe_in_use;
            evt_data.registry.data_len = 0;
            evt_data.registry.index    = nfa_hci_cb.param_in_use;
            evt                        = NFA_HCI_GET_REG_CMD_EVT;
            break;

        case NFA_HCI_ANY_OPEN_PIPE:
            /* 
             * As no response to the command sent on this pipe, we may assume the pipe is 
             * deleted already and release the pipe. But still send delete pipe command to be safe.
             */
            nfa_hciu_send_delete_pipe_cmd (nfa_hci_cb.pipe_in_use);
            nfa_hciu_release_pipe (nfa_hci_cb.pipe_in_use);
            evt_data.opened.pipe = nfa_hci_cb.pipe_in_use;
            evt                  = NFA_HCI_OPEN_PIPE_EVT;
            break;

        case NFA_HCI_ANY_CLOSE_PIPE:
            /* 
             * As no response to the command sent on this pipe, we may assume the pipe is 
             * deleted already and release the pipe. But still send delete pipe command to be safe.
             */
            nfa_hciu_send_delete_pipe_cmd (nfa_hci_cb.pipe_in_use);
            nfa_hciu_release_pipe (nfa_hci_cb.pipe_in_use);
            evt_data.closed.pipe = nfa_hci_cb.pipe_in_use;
            evt                  = NFA_HCI_CLOSE_PIPE_EVT;
            break;

        case NFA_HCI_ADM_CREATE_PIPE:
            evt_data.created.pipe        = nfa_hci_cb.pipe_in_use;
            evt_data.created.source_gate = nfa_hci_cb.local_gate_in_use;
            evt_data.created.dest_host   = nfa_hci_cb.remote_host_in_use;
            evt_data.created.dest_gate   = nfa_hci_cb.remote_gate_in_use;
            evt                          = NFA_HCI_CREATE_PIPE_EVT;
            break;

        case NFA_HCI_ADM_DELETE_PIPE:
            nfa_hciu_release_pipe (nfa_hci_cb.pipe_in_use);
            evt_data.deleted.pipe = nfa_hci_cb.pipe_in_use;
            evt                   = NFA_HCI_DELETE_PIPE_EVT;
            /* 
             * As no response to the command sent on this pipe, we may assume the pipe is 
             * deleted already. Just release the pipe.
             */
            nfa_hciu_release_pipe (nfa_hci_cb.pipe_in_use);
            break;

        default:
            /* 
             * As no response to the command sent on this pipe, we may assume the pipe is 
             * deleted already and release the pipe. But still send delete pipe command to be safe.
             */
            nfa_hciu_send_delete_pipe_cmd (nfa_hci_cb.pipe_in_use);
            nfa_hciu_release_pipe (nfa_hci_cb.pipe_in_use);
            break;
        }
        break;
    case NFA_HCI_STATE_DISABLED:
    default:
        NFA_TRACE_DEBUG0 ("nfa_hci_rsp_timeout () Timeout in DISABLED/ Invalid state");
        break;
    }
    if (evt != 0)
        nfa_hciu_send_to_app (evt, &evt_data, nfa_hci_cb.app_in_use);
}

/*******************************************************************************
**
** Function         nfa_hci_set_reassemble_buf
**
** Description      Set reassembly buffer for incoming message
**
** Returns          status
**
*******************************************************************************/
static void nfa_hci_set_reassemble_buf (UINT8 pipe)
{
    tNFA_HANDLE app_handle;
    UINT8       app_inx;

    if (  (pipe >= NFA_HCI_FIRST_DYNAMIC_PIPE)
        &&(nfa_hci_cb.type == NFA_HCI_EVENT_TYPE)  )
    {
        app_handle = nfa_hciu_get_pipe_owner (pipe);

        if (app_handle != NFA_HANDLE_INVALID)
        {
            app_inx    = app_handle & NFA_HANDLE_MASK;

            if (  (nfa_hci_cb.app_info[app_inx].buf_size)
                &&(nfa_hci_cb.app_info[app_inx].p_evt_buf != NULL)  )
            {
                nfa_hci_cb.p_msg_data  = nfa_hci_cb.app_info[app_inx].p_evt_buf;
                nfa_hci_cb.max_msg_len = nfa_hci_cb.app_info[app_inx].buf_size;
                return;
            }
        }
    }
    nfa_hci_cb.p_msg_data  = nfa_hci_cb.msg_data;
    nfa_hci_cb.max_msg_len = NFA_MAX_HCI_EVENT_LEN;
}

/*******************************************************************************
**
** Function         nfa_hci_assemble_msg
**
** Description      Reassemble the incoming message
**
** Returns          status
**
*******************************************************************************/
static tNFA_STATUS nfa_hci_assemble_msg (UINT8 pipe, UINT8 *p_data, UINT16 data_len)
{
    if ((nfa_hci_cb.msg_len + data_len) >= nfa_hci_cb.max_msg_len)
        return NFA_STATUS_FAILED;

    memcpy (&nfa_hci_cb.p_msg_data[nfa_hci_cb.msg_len], p_data, data_len);

    nfa_hci_cb.msg_len += data_len;

    return NFA_STATUS_OK;
}

/*******************************************************************************
**
** Function         nfa_hci_evt_hdlr
**
** Description      Processing all event for NFA HCI
**
** Returns          TRUE if p_msg needs to be deallocated
**
*******************************************************************************/
static BOOLEAN nfa_hci_evt_hdlr (BT_HDR *p_msg)
{
    tNFA_HCI_EVENT_DATA *p_evt_data = (tNFA_HCI_EVENT_DATA *)p_msg;


#if (BT_TRACE_VERBOSE == TRUE)
    NFA_TRACE_EVENT4 ("nfa_hci_evt_hdlr state: %s (%d) event: %s (0x%04x)",
                      nfa_hciu_get_state_name (nfa_hci_cb.hci_state), nfa_hci_cb.hci_state, 
                      nfa_hciu_get_event_name (p_evt_data->hdr.event), p_evt_data->hdr.event);
#else
    NFA_TRACE_EVENT2 ("nfa_hci_evt_hdlr state: %d event: 0x%04x", nfa_hci_cb.hci_state, p_evt_data->hdr.event);
#endif

    /* If this is an API request, queue it up */
    if ((p_msg->event >= NFA_HCI_FIRST_API_EVENT) && (p_msg->event <= NFA_HCI_LAST_API_EVENT))
    {
        GKI_enqueue (&nfa_hci_cb.hci_api_q, p_msg);
    }
    else
    {
        switch (p_msg->event)
        {
        case NFA_HCI_VSC_INIT_EVT:
            if (p_evt_data->vsc_init.status == NFA_STATUS_OK)
                nfa_hci_startup ();
            else
                nfa_hci_startup_complete (p_evt_data->vsc_init.status);
            break;

        case NFA_HCI_EE_DISC_CMPLT_EVT:
            if (!p_evt_data->ee_disc_cmplt.b_disc_cmplt)
            {
                nfa_hci_cb.ee_disc_cmplt = TRUE;
                nfa_hci_startup ();
            }
            break;

        case NFA_HCI_RSP_NV_READ_EVT:
            nfa_hci_handle_nv_read (p_evt_data->nv_read.block, p_evt_data->nv_read.status);
            break;

        case NFA_HCI_RSP_NV_WRITE_EVT:
            /* NV Ram write completed - nothing to do... */
            break;

        case NFA_HCI_RSP_TIMEOUT_EVT:
            nfa_hci_rsp_timeout ((tNFA_HCI_EVENT_DATA *)p_msg);
            break;

        case NFA_HCI_HCP_CONN_CREATE_EVT:
            break;

        case NFA_HCI_CHECK_QUEUE_EVT:
            if (HCI_LOOPBACK_DEBUG)
            {
                if (p_msg->len != 0)
                {
                    tNFC_DATA_CEVT   xx;
                    xx.p_data = p_msg;
                    nfa_hci_conn_cback (0, NFC_DATA_CEVT, (tNFC_CONN *)&xx);
                    return FALSE;
                }
            }
            break;
        }        
    }

    if (nfa_hci_cb.p_vs_evt_hdlr)
    {
        (*(nfa_hci_cb.p_vs_evt_hdlr)) (p_evt_data);
    }

    if ((p_msg->event > NFA_HCI_LAST_API_EVENT))
        GKI_freebuf (p_msg);

    nfa_hci_check_api_requests ();

    if ((nfa_hci_cb.hci_state == NFA_HCI_STATE_IDLE) && (nfa_hci_cb.nv_write_needed))
    {
        nfa_hci_cb.nv_write_needed = FALSE;
        nfa_nv_co_write ((UINT8 *)&nfa_hci_cb.cfg, sizeof (nfa_hci_cb.cfg),DH_NV_BLOCK);
    }

    return FALSE;
}

