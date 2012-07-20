/*****************************************************************************
**
**  Name:           nfa_cho_main.c
**
**  Description:    This is the main implementation file for the NFA Connection
**                  Handover.
**
**  Copyright (c) 2010, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#include <string.h>
#include "nfc_api.h"
#include "nfa_sys.h"
#include "nfa_cho_api.h"
#include "nfa_cho_int.h"
#include "trace_api.h"

/*****************************************************************************
**  Global Variables
*****************************************************************************/

/* system manager control block definition */
#if NFA_DYNAMIC_MEMORY == FALSE
tNFA_CHO_CB nfa_cho_cb;
#endif

/*****************************************************************************
**  Static Functions
*****************************************************************************/

/* event handler function type */
static BOOLEAN nfa_cho_evt_hdlr (BT_HDR *p_msg);

/* disable function type */
static void nfa_cho_sys_disable (void);

/*****************************************************************************
**  Constants
*****************************************************************************/
static const tNFA_SYS_REG nfa_cho_sys_reg = 
{
    NULL,
    nfa_cho_evt_hdlr, 
    nfa_cho_sys_disable,
    NULL
};

/*******************************************************************************
**
** Function         nfa_cho_timer_cback
**
** Description      Process timeout event when timer expires
**                  
**
** Returns          None
**
*******************************************************************************/
static void nfa_cho_timer_cback (void *p_tle)
{
    nfa_cho_sm_execute (NFA_CHO_TIMEOUT_EVT, NULL);
}

/*******************************************************************************
**
** Function         nfa_cho_init
**
** Description      Initialize NFA Connection Handover
**                  
**
** Returns          None
**
*******************************************************************************/
void nfa_cho_init (void)
{
    CHO_TRACE_DEBUG0 ("nfa_cho_init ()");

    /* initialize control block */
    memset (&nfa_cho_cb, 0, sizeof (tNFA_CHO_CB));

    nfa_cho_cb.server_sap = LLCP_INVALID_SAP;
    nfa_cho_cb.client_sap = LLCP_INVALID_SAP;
    nfa_cho_cb.remote_sap = LLCP_INVALID_SAP;

    nfa_cho_cb.hs_ndef_type_handle   = NFA_HANDLE_INVALID;
    nfa_cho_cb.bt_ndef_type_handle   = NFA_HANDLE_INVALID;
    nfa_cho_cb.wifi_ndef_type_handle = NFA_HANDLE_INVALID;

    nfa_cho_cb.trace_level    = APPL_INITIAL_TRACE_LEVEL;

    /* register message handler on NFA SYS */
    nfa_sys_register ( NFA_ID_CHO,  &nfa_cho_sys_reg);

    /* initialize timer callback */
    nfa_cho_cb.timer.p_cback = nfa_cho_timer_cback;
}

/*******************************************************************************
**
** Function         nfa_cho_sys_disable
**
** Description      Deregister NFA Connection Handover from NFA SYS/DM
**                  
**
** Returns          None
**
*******************************************************************************/
static void nfa_cho_sys_disable (void)
{
    CHO_TRACE_DEBUG0 ("nfa_cho_sys_disable ()");

    /* clean up if application is still registered */
    if (nfa_cho_cb.p_cback)
    {
        nfa_cho_proc_api_dereg ();
    }

    /* deregister message handler on NFA SYS */
    nfa_sys_deregister (NFA_ID_CHO);
}

/*******************************************************************************
**
** Function         nfa_cho_evt_hdlr
**
** Description      Processing event for NFA Connection Handover
**                  
**
** Returns          TRUE if p_msg needs to be deallocated
**
*******************************************************************************/
static BOOLEAN nfa_cho_evt_hdlr (BT_HDR *p_msg)
{
    tNFA_CHO_INT_EVENT_DATA *p_data = (tNFA_CHO_INT_EVENT_DATA *) p_msg;

    nfa_cho_sm_execute (p_data->hdr.event, p_data);

    return TRUE;
}


