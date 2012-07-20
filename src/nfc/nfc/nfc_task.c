/*****************************************************************************
**
**  Name:          nfc_task.c
**
**  Description:   Entry point for NFC_TASK
**
**  Copyright (c) 2010-2012, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
******************************************************************************/
#include <string.h>
#include "gki.h"
#include "nfc_target.h"
#include "bt_types.h"

#if (NFC_INCLUDED == TRUE)
#include "nfc_api.h"
#include "nfc_int.h"
#include "nci_int.h"
#include "nci_hmsgs.h"
#include "rw_int.h"
#include "ce_int.h"
#include "llcp_int.h"

#if (defined(NFA_INCLUDED) && NFA_INCLUDED == TRUE)
#include "nfa_sys.h"
#include "nfa_dm_int.h"
#endif

/*******************************************************************************
**
** Function         nfc_notify_shared_transport_ready
**
** Description      Internal function - called by BT stack to notify NFC_TASK
**                  that the shared HCI/NCI transport is ready.
**
** Returns          The new or current trace level
**
*******************************************************************************/
void nfc_notify_shared_transport_ready(void)
{
    if (nci_cfg.shared_transport)
        GKI_send_event(NFC_TASK, NFC_TASK_EVT_TRANSPORT_READY);
}

/*******************************************************************************
**
** Function         nfc_start_timer
**
** Description      Start a timer for the specified amount of time.
**                  NOTE: The timeout resolution is in SECONDS! (Even
**                          though the timer structure field is ticks)
**
** Returns          void
**
*******************************************************************************/
void nfc_start_timer (TIMER_LIST_ENT *p_tle, UINT16 type, UINT32 timeout)
{
    BT_HDR *p_msg;

    /* if timer list is currently empty, start periodic GKI timer */
    if (nfc_cb.timer_queue.p_first == NULL)
    {
        /* if timer starts on other than NFC task (scritp wrapper) */
        if(GKI_get_taskid() != NFC_TASK)
        {
            /* post event to start timer in NFC task */
            if ((p_msg = (BT_HDR *)GKI_getbuf(BT_HDR_SIZE)) != NULL)
            {
                p_msg->event = BT_EVT_TO_START_TIMER;
                GKI_send_msg (NFC_TASK, NFC_MBOX_ID, p_msg);
            }
        }
        else
        {
            /* Start nfc_task 1-sec resolution timer */
            GKI_start_timer (NFC_TIMER_ID, GKI_SECS_TO_TICKS (1), TRUE);
        }
    }
    
    GKI_remove_from_timer_list (&nfc_cb.timer_queue, p_tle);

    p_tle->event = type;
    p_tle->ticks = timeout;         /* Save the number of seconds for the timer */

    GKI_add_to_timer_list (&nfc_cb.timer_queue, p_tle);
}

/*******************************************************************************
**
** Function         nfc_remaining_time
**
** Description      Return amount of time to expire
**
** Returns          time in second
**
*******************************************************************************/
UINT32 nfc_remaining_time (TIMER_LIST_ENT *p_tle)
{
    return (GKI_get_remaining_ticks (&nfc_cb.timer_queue, p_tle));
}

/*******************************************************************************
**
** Function         nfc_process_timer_evt
**
** Description      Process nfc GKI timer event
**
** Returns          void
**
*******************************************************************************/
void nfc_process_timer_evt(void)
{
    TIMER_LIST_ENT  *p_tle;

    GKI_update_timer_list (&nfc_cb.timer_queue, 1);

    while ((nfc_cb.timer_queue.p_first) && (!nfc_cb.timer_queue.p_first->ticks))
    {
        p_tle = nfc_cb.timer_queue.p_first;
        GKI_remove_from_timer_list (&nfc_cb.timer_queue, p_tle);

        switch (p_tle->event)
        {
        case NFC_TTYPE_NCI_CMD_CMPL:
            nfc_ncif_cmd_timeout();
            break;
        case NFC_TTYPE_WAIT_2_DEACTIVATE:
            nfc_wait_2_deactivate_timeout();
            break;

        default:
            NFC_TRACE_DEBUG1("nfc_process_timer_evt: unhandled timer event (0x%04x)", p_tle->event);
            break;
        }
    }

    /* if timer list is empty stop periodic GKI timer */
    if (nfc_cb.timer_queue.p_first == NULL)
    {
        GKI_stop_timer(NFC_TIMER_ID);
    }
}

/*******************************************************************************
**
** Function         nfc_stop_timer
**
** Description      Stop a timer.
**
** Returns          void
**
*******************************************************************************/
void nfc_stop_timer (TIMER_LIST_ENT *p_tle)
{
    GKI_remove_from_timer_list (&nfc_cb.timer_queue, p_tle);

    /* if timer list is empty stop periodic GKI timer */
    if (nfc_cb.timer_queue.p_first == NULL)
    {
        GKI_stop_timer(NFC_TIMER_ID);
    }
}

#if defined(QUICK_TIMER_TICKS_PER_SEC) && (QUICK_TIMER_TICKS_PER_SEC > 0)
/*******************************************************************************
**
** Function         nfc_start_quick_timer
**
** Description      Start a timer for the specified amount of time.
**                  NOTE: The timeout resolution depends on including modules.
**                  QUICK_TIMER_TICKS_PER_SEC should be used to convert from
**                  time to ticks.
**
**
** Returns          void
**
*******************************************************************************/
void nfc_start_quick_timer (TIMER_LIST_ENT *p_tle, UINT16 type, UINT32 timeout)
{
    BT_HDR *p_msg;

    /* if timer list is currently empty, start periodic GKI timer */
    if (nfc_cb.quick_timer_queue.p_first == NULL)
    {
        /* if timer starts on other than NFC task (scritp wrapper) */
        if(GKI_get_taskid() != NFC_TASK)
        {
            /* post event to start timer in NFC task */
            if ((p_msg = (BT_HDR *)GKI_getbuf(BT_HDR_SIZE)) != NULL)
            {
                p_msg->event = BT_EVT_TO_START_QUICK_TIMER;
                GKI_send_msg (NFC_TASK, NFC_MBOX_ID, p_msg);
            }
        }
        else
        {
            /* Quick-timer is required for LLCP */
            GKI_start_timer (NFC_QUICK_TIMER_ID, ((GKI_SECS_TO_TICKS (1)/QUICK_TIMER_TICKS_PER_SEC)), TRUE);
        }
    }
    
    GKI_remove_from_timer_list (&nfc_cb.quick_timer_queue, p_tle);

    p_tle->event = type;
    p_tle->ticks = timeout; /* Save the number of ticks for the timer */

    GKI_add_to_timer_list (&nfc_cb.quick_timer_queue, p_tle);
}




/*******************************************************************************
**
** Function         nfc_stop_quick_timer
**
** Description      Stop a timer.
**
** Returns          void
**
*******************************************************************************/
void nfc_stop_quick_timer (TIMER_LIST_ENT *p_tle)
{
    GKI_remove_from_timer_list (&nfc_cb.quick_timer_queue, p_tle);

    /* if timer list is empty stop periodic GKI timer */
    if (nfc_cb.quick_timer_queue.p_first == NULL)
    {
        GKI_stop_timer(NFC_QUICK_TIMER_ID);
    }
}

/*******************************************************************************
**
** Function         nfc_process_quick_timer_evt
**
** Description      Process quick timer event
**
** Returns          void
**
*******************************************************************************/
void nfc_process_quick_timer_evt(void)
{
    TIMER_LIST_ENT  *p_tle;

    GKI_update_timer_list (&nfc_cb.quick_timer_queue, 1);

    while ((nfc_cb.quick_timer_queue.p_first) && (!nfc_cb.quick_timer_queue.p_first->ticks))
    {
        p_tle = nfc_cb.quick_timer_queue.p_first;
        GKI_remove_from_timer_list (&nfc_cb.quick_timer_queue, p_tle);

        switch (p_tle->event)
        {
#if (NFC_RW_ONLY == FALSE)
        case NFC_TTYPE_LLCP_LINK_MANAGER:
        case NFC_TTYPE_LLCP_LINK_INACT:
        case NFC_TTYPE_LLCP_DATA_LINK:
        case NFC_TTYPE_LLCP_DELAY_FIRST_PDU:
            llcp_process_timeout (p_tle);
            break;
#endif
        case NFC_TTYPE_RW_T1T_RESPONSE:
            rw_t1t_process_timeout (p_tle);
            break;
        case NFC_TTYPE_RW_T2T_RESPONSE:
            rw_t2t_process_timeout (p_tle);
            break;
        case NFC_TTYPE_RW_T3T_RESPONSE:
            rw_t3t_process_timeout (p_tle);
            break;
        case NFC_TTYPE_RW_T4T_RESPONSE:
            rw_t4t_process_timeout (p_tle);
            break;
        case NFC_TTYPE_RW_I93_RESPONSE:
            rw_i93_process_timeout (p_tle);
            break;
#if (NFC_RW_ONLY == FALSE)
        case NFC_TTYPE_CE_T4T_UPDATE:
            ce_t4t_process_timeout (p_tle);
            break;
#endif
        default:
            if (nfc_cb.p_vs_evt_hdlr)
                (*nfc_cb.p_vs_evt_hdlr)(NFC_INT_TIMEOUT_EVT, p_tle);
            break;
        }
    }

    /* if timer list is empty stop periodic GKI timer */
    if (nfc_cb.quick_timer_queue.p_first == NULL)
    {
        GKI_stop_timer(NFC_QUICK_TIMER_ID);
    }
}
#endif /* defined(QUICK_TIMER_TICKS_PER_SEC) && (QUICK_TIMER_TICKS_PER_SEC > 0) */




/*******************************************************************************
**
** Function         nfc_task_enable_nfc
**
** Description      Enable NFC
**
** Returns          nothing
**
*******************************************************************************/
void nfc_task_enable_nfc(void)
{
    nfc_cb.flags |= NFC_FL_ENABLED;

    /* Reset the NFC controller. */
    nci_snd_core_reset(NCI_RESET_TYPE_RESET_CFG);
}

/*******************************************************************************
**
** Function         nfc_task_handle_terminate
**
** Description      Handle NFC shutdown
**
** Returns          nothing
**
*******************************************************************************/
void nfc_task_handle_terminate(void)
{
    BT_HDR        *p_msg;
    tNFC_RESPONSE  evt_data;

    /* Free any messages still in the mbox */
    while ((p_msg = (BT_HDR *) GKI_read_mbox (NFC_MBOX_ID)) != NULL)
    {
        GKI_freebuf(p_msg);
    }

    if (nfc_cb.nfc_state == NFC_STATE_NFCC_POWER_OFF_SLEEP)
    {
        /* clear flags for transport */
        nfc_cb.flags &= ~NFC_FL_NCI_TRANSPORT_ENABLED;

        nfc_gen_cleanup();

        /* if it's power cycle NFCC */
        if (nfc_cb.flags & NFC_FL_POWER_CYCLE_NFCC)
        {
            nfc_cb.flags |= NFC_FL_W4_TRANSPORT_READY;
            GKI_send_event(NCI_TASK, NCI_TASK_EVT_POWER_CYCLE);
        }
        else
        {
            /* if it is not shared transport, close port */
            if (!nci_cfg.shared_transport)
            {
                GKI_send_event(NCI_TASK, NCI_TASK_EVT_TERMINATE);
            }

            if (nfc_cb.p_resp_cback)
            {
                evt_data.status = NFC_STATUS_OK;
                (*nfc_cb.p_resp_cback)(NFC_NFCC_POWER_OFF_REVT, &evt_data);
            }
        }
    }
    else
    {
        /* Perform final clean up */
        nfc_main_cleanup();

        /* Stop the timers */
        GKI_stop_timer(NFC_TIMER_ID);
        GKI_stop_timer(NFC_QUICK_TIMER_ID);
#if (defined(NFA_INCLUDED) && NFA_INCLUDED == TRUE)
        GKI_stop_timer(NFA_TIMER_ID);
#endif
    }
}

/*******************************************************************************
**
** Function         nfc_task
**
** Description      NFC event processing task
**
** Returns          nothing
**
*******************************************************************************/
UINT32 nfc_task (UINT32 param)
{
    UINT16  event;
    BT_HDR  *p_msg;
    BOOLEAN free_buf, ret;

    /* Initialize the nfc control block */
    memset (&nfc_cb, 0, sizeof(tNFC_CB));
    nfc_cb.trace_level = NFC_INITIAL_TRACE_LEVEL;

#if (defined(NFA_INCLUDED) && NFA_INCLUDED == TRUE)
    memset(&nfa_dm_cb, 0, sizeof(tNFA_DM_CB));
#endif

    NFC_TRACE_DEBUG0("NFC_TASK started.");

    /* main loop */
    while (TRUE)
    {
        event = GKI_wait (0xFFFF, 0);

        /* Handle NFC_TASK_EVT_ENABLE generated by NFC_Enable */
        if (event & NFC_TASK_EVT_ENABLE)
        {
            nfc_cb.flags |= NFC_FL_ENABLE_PENDING;

            /* If transport enabled then proceed with enabling NFC */
            if (nfc_cb.flags & NFC_FL_NCI_TRANSPORT_ENABLED)
            {
                NFC_TRACE_DEBUG0("NFC_TASK got NFC_TASK_EVT_ENABLE. Proceeding with NFC start up.");
                nfc_task_enable_nfc();
            }
            else
            {
                /* Transport not ready. Wait for transport to be ready before enabling */
                nfc_cb.flags |= NFC_FL_W4_TRANSPORT_READY;
                NFC_TRACE_DEBUG0("NFC_TASK got NFC_TASK_EVT_ENABLE. Waiting for NCI transport..");
            }
        }

        /* Handle NFC_TASK_EVT_TRANSPORT_READY from NCI_TASK (for dedicated transport) or BTU_TASK (for shared BT/NFC transport) */
        if (event & NFC_TASK_EVT_TRANSPORT_READY)
        {
            NFC_TRACE_DEBUG0("NFC_TASK got NFC_TASK_EVT_TRANSPORT_READY.");
            nfc_cb.flags |= NFC_FL_NCI_TRANSPORT_ENABLED;

            /* if enable is pending, then enable now */
            if (nfc_cb.flags & NFC_FL_W4_TRANSPORT_READY)
            {
                NFC_TRACE_DEBUG0("Proceeding with NFC start up.");
                nfc_cb.flags &= ~NFC_FL_W4_TRANSPORT_READY;
                nfc_cb.flags &= ~NFC_FL_POWER_CYCLE_NFCC;

                if (nfc_cb.nfc_state == NFC_STATE_NFCC_POWER_OFF_SLEEP)
                {
                    nfc_cb.nfc_state = NFC_STATE_RESTARTING;
                    nci_snd_core_reset(NCI_RESET_TYPE_RESET_CFG);
                }
                else
                    nfc_task_enable_nfc();
            }
        }

        /* Handle termination signal */
        if (event & NFC_TASK_EVT_TERMINATE)
        {
            NFC_TRACE_DEBUG0("NFC_TASK got NFC_TASK_EVT_TERMINATE. Shutting down NFC...");
            nfc_task_handle_terminate();

            continue;
        }


        if (event & NFC_MBOX_EVT_MASK)
        {
            /* Process all incoming NCI messages */
            while ((p_msg = (BT_HDR *) GKI_read_mbox (NFC_MBOX_ID)) != NULL)
            {
                ret         = FALSE;
                free_buf    = TRUE;

                /* Determine the input message type. */
                switch (p_msg->event & BT_EVT_MASK)
                {
                    case BT_EVT_TO_NFC_NCI:
                        free_buf = nfc_ncif_process_event (p_msg);
                        break;

                    case BT_EVT_TO_START_TIMER :
                        /* Start nfc_task 1-sec resolution timer */
                        GKI_start_timer (NFC_TIMER_ID, GKI_SECS_TO_TICKS (1), TRUE);
                        break;

#if defined(QUICK_TIMER_TICKS_PER_SEC) && (QUICK_TIMER_TICKS_PER_SEC > 0)
                    case BT_EVT_TO_START_QUICK_TIMER :
                        /* Quick-timer is required for LLCP */
                        GKI_start_timer (NFC_QUICK_TIMER_ID, ((GKI_SECS_TO_TICKS (1)/QUICK_TIMER_TICKS_PER_SEC)), TRUE);
                        break;
#endif
                    case BT_EVT_TO_NFC_ERR:
                        nfc_main_handle_err(p_msg);
                        free_buf = FALSE;
                        break;

                    default:
                        NFC_TRACE_DEBUG1("nfc_task: unhandle mbox message, event=%04x", p_msg->event);
                        break;
                }

                if (nfc_cb.p_vs_evt_hdlr)
                    ret = (*nfc_cb.p_vs_evt_hdlr)((UINT16)(NFC_INT_MBOX_EVT| (p_msg->event & BT_EVT_MASK)), p_msg);

                if (free_buf && ret == FALSE)
                {
                    GKI_freebuf (p_msg);                        
                }
            }
        }

        /* Process gki timer tick */
        if (event & NFC_TIMER_EVT_MASK)
        {
            nfc_process_timer_evt();
        }

        /* Process quick timer tick */
        if (event & NFC_QUICK_TIMER_EVT_MASK)
        {
            nfc_process_quick_timer_evt();
        }

#if (defined(NFA_INCLUDED) && NFA_INCLUDED == TRUE)
        if (event & NFA_MBOX_EVT_MASK)
        {
            while ((p_msg = (BT_HDR *) GKI_read_mbox(NFA_MBOX_ID)) != NULL)
            {
                nfa_sys_event(p_msg);
            }
        }

        if (event & NFA_TIMER_EVT_MASK)
        {
            nfa_sys_timer_update();
        }
#endif

    }


    NFC_TRACE_DEBUG0("nfc_task terminated");

    GKI_exit_task(GKI_get_taskid());
    return 0;
}

#endif /* NFC_INCLUDED == TRUE */
