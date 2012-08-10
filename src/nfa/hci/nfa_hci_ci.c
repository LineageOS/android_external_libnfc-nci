/*****************************************************************************
**
**  Name:           nfa_hci_ci.c
**
**  Description:    This file contains the call-in functions for NFA HCI
**
**  Copyright (c) 2010-2011, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#include <string.h>
#include "nfa_sys.h"
#include "nfa_hci_api.h"
#include "nfa_hci_int.h"
#include "nfa_nv_co.h"


/*******************************************************************************
**
** Function         nfa_nv_ci_read
**
** Description      call-in function for non volatile memory read acess
**
** Returns          none
**
*******************************************************************************/
void nfa_nv_ci_read (UINT16 num_bytes_read, tNFA_NV_CO_STATUS status, UINT8 block)
{
    tNFA_HCI_EVENT_DATA *p_msg;

    if ((p_msg = (tNFA_HCI_EVENT_DATA *) GKI_getbuf (sizeof (tNFA_HCI_EVENT_DATA))) != NULL)
    {
        p_msg->nv_read.hdr.event = NFA_HCI_RSP_NV_READ_EVT;

        if (  (status == NFA_STATUS_OK)
            &&(num_bytes_read != 0) )
        {
            p_msg->nv_read.status = NFA_STATUS_OK;
            p_msg->nv_read.size   = num_bytes_read;
        }
        else
            p_msg->nv_read.status = NFA_STATUS_FAILED;

        p_msg->nv_read.block = block;
        nfa_sys_sendmsg (p_msg);
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
void nfa_nv_ci_write (tNFA_NV_CO_STATUS status)
{
    tNFA_HCI_EVENT_DATA *p_msg;

    if ((p_msg = (tNFA_HCI_EVENT_DATA *) GKI_getbuf (sizeof (tNFA_HCI_EVENT_DATA))) != NULL)
    {
        p_msg->nv_write.hdr.event = NFA_HCI_RSP_NV_WRITE_EVT;
        p_msg->nv_write.status = 0;
        nfa_sys_sendmsg (p_msg);
    }
}

