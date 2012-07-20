/*****************************************************************************
**
**  Name:          nci_main.c
**
**  Description:   Functions for handling NCI Transport events
**
**  Copyright (c) 2010, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
******************************************************************************/
#include <string.h>
#include "gki.h"
#include "nfc_target.h"
#include "bt_types.h"
#include "userial.h"
#include "upio.h"

#include "hcidefs.h"

#if (NFC_INCLUDED == TRUE)
#include "nfc_api.h"
#include "nfc_int.h"
#include "nci_int.h"

/****************************************************************************
** Definitions
****************************************************************************/

/* Default NCI port configuration  */
NCI_CFG_QUALIFIER tNCI_CFG nci_cfg = 
{
    NFC_SHARED_TRANSPORT_ENABLED,   /* bSharedTransport */
    USERIAL_BAUD_115200,            /* Baud rate */
    USERIAL_FC_HW                   /* Flow control */
};

/* Control block for NCI transport */
#if NFC_DYNAMIC_MEMORY == FALSE
tNCI_CB nci_cb;
#endif

/****************************************************************************
** Internal function prototypes
****************************************************************************/
static void nci_cback (tUSERIAL_PORT port, tUSERIAL_EVT evt, tUSERIAL_EVT_DATA *p_data);

/*******************************************************************************
**
** Function         nci_init
**
** Description      This function initializes control block for NCI
**
** Returns          nothing
**
*******************************************************************************/
void nci_init (void)
{
    /* Clear nci control block */
    memset(&nci_cb, 0, sizeof (tNCI_CB));

    nci_cb.trace_level = NFC_INITIAL_TRACE_LEVEL;
}

/*******************************************************************************
**
** Function         nci_open_transport
**
** Description      Open transport and reset transport control block; prepare for
**                  new incoming message;
**
** Returns          nothing
**
*******************************************************************************/
static void nci_open_transport (void)
{
    tUSERIAL_OPEN_CFG open_cfg;

    /* Initialize control block */
    nci_cb.rcv_state = nci_cb.init_rcv_state;

    if (nci_cb.p_rcv_msg)
    {
        GKI_freebuf (nci_cb.p_rcv_msg);
        nci_cb.p_rcv_msg = NULL;
    }

    /* open transport */
    open_cfg.fmt    = (USERIAL_DATABITS_8|USERIAL_PARITY_NONE|USERIAL_STOPBITS_1);
    open_cfg.baud   = nci_cfg.userial_baud;
    open_cfg.fc     = nci_cfg.userial_fc;
    open_cfg.buf    = USERIAL_BUF_BYTE;

    USERIAL_Open (USERIAL_NFC_PORT, &open_cfg, nci_cback);

    /* notify transport openned */
    if (nci_cb.p_vs_evt_hdlr)
    {
        /* start VS intialization */
        /* VS module must call nci_task_vs_init_done() when complete of VS initilaization */
        (*nci_cb.p_vs_evt_hdlr) (NCI_INT_VS_INIT_EVT, NULL);
    }
    else
    {
        /* Notify NFC Task that transport is ready */
        GKI_send_event (NFC_TASK, NFC_TASK_EVT_TRANSPORT_READY);
    }
}

/*****************************************************************************
**
** Function         nci_send
**
** Description      Send message to NCI task
**
** Returns          nothing
**
*****************************************************************************/
void nci_send (BT_HDR *p_msg)
{
    GKI_send_msg (NCI_TASK, NCI_TASK_MBOX, p_msg);
}

/*****************************************************************************
**
** Function         nci_receive_msg
**
** Description
**      Handle incoming data (NCI events) from the serial port.
**
**      If there is data waiting from the serial port, this funciton reads the
**      data and parses it. Once an entire NCI message has been read, it sends
**      the message the the NFC_TASK for processing
**
*****************************************************************************/
static BOOLEAN nci_receive_msg (tNCI_CB *p_cb, UINT8 byte)
{
    UINT16      len;
    BOOLEAN     msg_received = FALSE;

    switch (p_cb->rcv_state)
    {
    case NCI_RCV_IDLE_ST:

        /* Initialize rx parameters */
        p_cb->rcv_state = NCI_RCV_NFC_HDR_ST;
        p_cb->rcv_len   = NCI_MSG_HDR_SIZE;

        /* Start of new message. Allocate a buffer for message */
        if ((p_cb->p_rcv_msg = (BT_HDR *) GKI_getpoolbuf (NFC_NCI_POOL_ID)) != NULL)
        {
            /* Initialize BT_HDR */
            p_cb->p_rcv_msg->len    = 0;
            p_cb->p_rcv_msg->event  = BT_EVT_TO_NFC_NCI;
            p_cb->p_rcv_msg->offset = 0;

            *((UINT8 *)(p_cb->p_rcv_msg + 1) + p_cb->p_rcv_msg->offset + p_cb->p_rcv_msg->len++) = byte;
        }
        else
        {
            NCI_TRACE_ERROR0 ("Unable to allocate buffer for incoming NCI message."); 
        }
        p_cb->rcv_len--;
        break;

    case NCI_RCV_NFC_HDR_ST:

        if (p_cb->p_rcv_msg)
        {
            *((UINT8 *)(p_cb->p_rcv_msg + 1) + p_cb->p_rcv_msg->offset + p_cb->p_rcv_msg->len++) = byte;
        }

        p_cb->rcv_len--;

        /* Check if we read in entire NFC message header yet */
        if (p_cb->rcv_len == 0)
        {
            p_cb->rcv_len       = byte;

            /* If non-zero payload, then go to receive-data state */
            if (byte > 0)
            {
                p_cb->rcv_state = NCI_RCV_DATA_ST;
            }
            else
            {
                msg_received    = TRUE;
                p_cb->rcv_state = p_cb->init_rcv_state;
            }
        }
        break;

    case NCI_RCV_DATA_ST:

        p_cb->rcv_len--;
        if (p_cb->p_rcv_msg)
        {
            *((UINT8 *)(p_cb->p_rcv_msg + 1) + p_cb->p_rcv_msg->offset + p_cb->p_rcv_msg->len++) = byte;

            if (p_cb->rcv_len > 0)
            {
                /* Read in the rest of the message */
                len = USERIAL_Read (USERIAL_NFC_PORT, ((UINT8 *)(p_cb->p_rcv_msg + 1) + p_cb->p_rcv_msg->offset + p_cb->p_rcv_msg->len),  p_cb->rcv_len);
                p_cb->p_rcv_msg->len    += len;
                p_cb->rcv_len           -= len;
            }
        }

        /* Check if we read in entire message yet */
        if (p_cb->rcv_len == 0)
        {
            msg_received    = TRUE;
            p_cb->rcv_state = p_cb->init_rcv_state;  /* wait for next message */
        }
        break;
    }

    return (msg_received);
}

/*******************************************************************************
**
** Function         nci_cback
**
** Description      USERIAL callback for NFC transport
**
** Returns          nothing
**
*******************************************************************************/
static void nci_cback (tUSERIAL_PORT port, tUSERIAL_EVT evt, tUSERIAL_EVT_DATA *p_data)
{
    BT_HDR *p_msg;

    if (evt == USERIAL_RX_READY_EVT)
    {
        /* Notify transport task of serial port event */
        GKI_send_event (NCI_TASK, NCI_TASK_EVT_DATA_RDY);
    }
    else if (evt == USERIAL_TX_DONE_EVT)
    {
        /* Serial driver has finshed sending data from USERIAL_Write */
        /* Currently, no action is needed for this event */
    }
    else if (evt == USERIAL_ERR_EVT)
    {
        NCI_TRACE_ERROR0("nci_cback: USERIAL_ERR_EVT. Notifying NFC_TASK of transport error");
        if ((p_msg = (BT_HDR *) GKI_getbuf (BT_HDR_SIZE)) != NULL)
        {
            /* Initialize BT_HDR */
            p_msg->len    = 0;
            p_msg->event  = BT_EVT_TO_NFC_ERR;
            p_msg->offset = 0;
            p_msg->layer_specific = NFC_ERR_TRANSPORT;
            GKI_send_msg (NFC_TASK, NFC_MBOX_ID, p_msg);
        }
        else
        {
            NCI_TRACE_ERROR0 ("nci_cback: USERIAL_ERR_EVT. No buffers.");
        }
    }
    else if (evt == USERIAL_WAKEUP_EVT)
    {
        NCI_TRACE_DEBUG1 ("nci_cback: USERIAL_WAKEUP_EVT: %d", p_data->sigs);
    }
    else
    {
        NCI_TRACE_DEBUG1 ("nci_cback: unhandled userial evt: %i", evt);
    }
}

/*******************************************************************************
**
** Function         nci_task_vs_init_done
**
** Description      notify complete of vendor specific initialization
**
** Returns          nothing
**
*******************************************************************************/
void nci_task_vs_init_done (void)
{
    NCI_TRACE_DEBUG0 ("nci_task_vs_init_done ()");

    /* Notify NFC Task that transport is ready */
    GKI_send_event (NFC_TASK, NFC_TASK_EVT_TRANSPORT_READY);
}

/*******************************************************************************
**
** Function         nci_task_timeout_cback
**
** Description      callback function for power cycle timeout
**
** Returns          void
**
*******************************************************************************/
static void nci_task_timeout_cback (void *p_tle)
{
    NCI_TRACE_DEBUG0 ("nci_task_timeout_cback ()");

    nci_open_transport ();
}

/*******************************************************************************
**
** Function         nci_task_handle_terminate
**
** Description      Handle NFI transport shutdown
**
** Returns          nothing
**
*******************************************************************************/
static void nci_task_handle_terminate (void)
{
    BT_HDR *p_msg;

    /* Free unsent nfc rx buffer */
    if (nci_cb.p_rcv_msg)
        GKI_freebuf (nci_cb.p_rcv_msg);

    /* Free buffers in the tx mbox */
    while ((p_msg = (BT_HDR *)GKI_read_mbox (NCI_TASK_MBOX)) != NULL)
    {
        GKI_freebuf(p_msg);
    }
}

#if defined(QUICK_TIMER_TICKS_PER_SEC) && (QUICK_TIMER_TICKS_PER_SEC > 0)
/*******************************************************************************
**
** Function         nci_start_quick_timer
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
void nci_start_quick_timer (TIMER_LIST_ENT *p_tle, UINT16 type, UINT32 timeout)
{
    BT_HDR *p_msg;

    /* if timer list is currently empty, start periodic GKI timer */
    if (nci_cb.quick_timer_queue.p_first == NULL)
    {
        /* if timer starts on other than NFC task (scritp wrapper) */
        if(GKI_get_taskid() != NCI_TASK)
        {
            /* post event to start timer in NFC task */
            if ((p_msg = (BT_HDR *)GKI_getbuf(BT_HDR_SIZE)) != NULL)
            {
                p_msg->event = BT_EVT_TO_START_QUICK_TIMER;
                GKI_send_msg (NCI_TASK, NCI_TASK_MBOX, p_msg);
            }
        }
        else
        {
            GKI_start_timer (NCI_QUICK_TIMER_ID, ((GKI_SECS_TO_TICKS (1)/QUICK_TIMER_TICKS_PER_SEC)), TRUE);
        }
    }
    
    GKI_remove_from_timer_list (&nci_cb.quick_timer_queue, p_tle);

    p_tle->event = type;
    p_tle->ticks = timeout; /* Save the number of ticks for the timer */

    GKI_add_to_timer_list (&nci_cb.quick_timer_queue, p_tle);
}

/*******************************************************************************
**
** Function         nci_stop_quick_timer
**
** Description      Stop a timer.
**
** Returns          void
**
*******************************************************************************/
void nci_stop_quick_timer (TIMER_LIST_ENT *p_tle)
{
    GKI_remove_from_timer_list (&nci_cb.quick_timer_queue, p_tle);

    /* if timer list is empty stop periodic GKI timer */
    if (nci_cb.quick_timer_queue.p_first == NULL)
    {
        GKI_stop_timer (NCI_QUICK_TIMER_ID);
    }
}

/*******************************************************************************
**
** Function         nci_process_quick_timer_evt
**
** Description      Process quick timer event
**
** Returns          void
**
*******************************************************************************/
static void nci_process_quick_timer_evt(void)
{
    TIMER_LIST_ENT  *p_tle;

    GKI_update_timer_list (&nci_cb.quick_timer_queue, 1);

    while ((nci_cb.quick_timer_queue.p_first) && (!nci_cb.quick_timer_queue.p_first->ticks))
    {
        p_tle = nci_cb.quick_timer_queue.p_first;
        GKI_remove_from_timer_list (&nci_cb.quick_timer_queue, p_tle);

        if (p_tle->p_cback)
        {
            (*p_tle->p_cback) (p_tle);
        }
    }

    /* if timer list is empty stop periodic GKI timer */
    if (nci_cb.quick_timer_queue.p_first == NULL)
    {
        GKI_stop_timer (NCI_QUICK_TIMER_ID);
    }
}
#endif /* defined(QUICK_TIMER_TICKS_PER_SEC) && (QUICK_TIMER_TICKS_PER_SEC > 0) */

/*******************************************************************************
**
** Function         nci_task
**
** Description      NFC event processing task
**
** Returns          0
**
*******************************************************************************/
UINT32 nci_task (UINT32 param)
{
    UINT16  event;
    BT_HDR  *p_msg;
    UINT8   *p, byte;
    BOOLEAN continue_to_process;

    NFC_TRACE_DEBUG0 ("NCI_TASK started");

    /* Main loop */
    while (TRUE)
    {
        event = GKI_wait (0xFFFF, 0);

        /* Handle NCI_TASK_EVT_INITIALIZE (for initializing NCI transport) */
        if (event & NCI_TASK_EVT_INITIALIZE)
        {
            NCI_TRACE_DEBUG0 ("NCI_TASK got NCI_TASK_EVT_INITIALIZE signal. Opening NFC transport...");

            nci_open_transport ();
        }

        /* Check for terminate event */
        if (event & NCI_TASK_EVT_TERMINATE)
        {
            NCI_TRACE_DEBUG0 ("NCI_TASK got NCI_TASK_EVT_TERMINATE");
            nci_task_handle_terminate ();

            /* notify closing transport */
            if (nci_cb.p_vs_evt_hdlr)
            {
                (*nci_cb.p_vs_evt_hdlr) (NCI_INT_TERMINATE_EVT, NULL);
            }

            /* Close uart */
            USERIAL_Close (USERIAL_NFC_PORT);

            continue;
        }

        /* Check for power cycle event */
        if (event & NCI_TASK_EVT_POWER_CYCLE)
        {
            NCI_TRACE_DEBUG0 ("NCI_TASK got NCI_TASK_EVT_POWER_CYCLE");
            nci_task_handle_terminate ();

            /* Close uart */
            USERIAL_Close (USERIAL_NFC_PORT);

            nci_cb.timer.p_cback = nci_task_timeout_cback;
            nci_start_quick_timer (&nci_cb.timer, 0x00, 
                                   (NCI_POWER_CYCLE_DELAY*QUICK_TIMER_TICKS_PER_SEC)/1000);

            continue;
        }

        /* NCI message ready to be sent to NFCC */
        if (event & NCI_TASK_EVT_MBOX)
        {
            while ((p_msg = (BT_HDR *)GKI_read_mbox (NCI_TASK_MBOX)) != NULL)
            {
                if ((p_msg->event & BT_EVT_MASK) == BT_EVT_TO_NFC_NCI)
                {
                    /* adding vendor specific header at NCI message before sending to NFCC */
                    if (nci_cb.p_vs_evt_hdlr)
                        continue_to_process = (*nci_cb.p_vs_evt_hdlr) (NCI_INT_SEND_NCI_MSG_EVT, p_msg);
                    else
                        continue_to_process = TRUE;

                    /* if NFCC is ready to receive NCI command */
                    if (continue_to_process)
                    {
                        p = (UINT8 *)(p_msg + 1) + p_msg->offset;
                        USERIAL_Write (USERIAL_NFC_PORT, p, p_msg->len);
                    }
                    else
                    {
                        /* do not free buffer */
                        continue;
                    }
                }
#if defined(QUICK_TIMER_TICKS_PER_SEC) && (QUICK_TIMER_TICKS_PER_SEC > 0)
                else if ((p_msg->event & BT_EVT_MASK) == BT_EVT_TO_START_QUICK_TIMER)
                {
                    GKI_start_timer (NCI_QUICK_TIMER_ID, ((GKI_SECS_TO_TICKS (1)/QUICK_TIMER_TICKS_PER_SEC)), TRUE);
                }
#endif
                else if ((p_msg->event & BT_EVT_MASK) == BT_EVT_TO_NFC_NCI_VS)
                {
                    /* sending vendor specific message to NFCC */
                    if (nci_cb.p_vs_evt_hdlr)
                    {
                        (*nci_cb.p_vs_evt_hdlr) (NCI_INT_VS_MSG_EVT, p_msg);
                    }
                }

                GKI_freebuf (p_msg);
            }
        }

        /* Data waiting to be read from serial port */
        if (event & NCI_TASK_EVT_DATA_RDY)
        {
            while (TRUE)
            {
                /* Read one byte to see if there is anything waiting to be read */
                if (USERIAL_Read (USERIAL_NFC_PORT, &byte, 1) == 0)
                {
                    break;
                }

                /* if need to process vendor specific header in front of NCI header */
                if (nci_cb.rcv_state == NCI_RCV_VS_PREAMBLE_ST)
                {
                    if (  (nci_cb.p_vs_evt_hdlr)
                        &&((*nci_cb.p_vs_evt_hdlr) (NCI_INT_DATA_RDY_EVT, &byte) == FALSE)  )
                    {
                        /* no more data after vendor specific header */
                        break;
                    }
                }

                /* if it is not vendor specific message */
                if (nci_cb.rcv_state != NCI_RCV_VS_MSG_ST)
                {
                    if (nci_receive_msg (&nci_cb, byte))
                    {
                        /* complete of receiving NCI message */
                        if (nci_cb.p_rcv_msg)
                        {
                            if (nci_cb.p_vs_evt_hdlr)
                                continue_to_process = (*nci_cb.p_vs_evt_hdlr) (NCI_INT_RX_NCI_MSG_EVT, nci_cb.p_rcv_msg);
                            else
                                continue_to_process = TRUE;

                            /* if this is not vendor specific transport message */
                            if (continue_to_process)
                            {
                                /* Send message to NFC_TASK for processing by the stack */
                                GKI_send_msg (NFC_TASK, NFC_MBOX_ID, nci_cb.p_rcv_msg);
                            }
                            else
                            {
                                /* Message has been processed by the vs_evt_hdlr */
                                GKI_freebuf(nci_cb.p_rcv_msg);
                            }

                            nci_cb.p_rcv_msg = NULL;
                        }
                    }
                }
                else
                {
                    /* process vendor specific message (other than NCI format) */
                    if (nci_cb.p_vs_evt_hdlr)
                    {
                        (*nci_cb.p_vs_evt_hdlr) (NCI_INT_DATA_RDY_EVT, &byte);
                    }
                }
            } /* while (TRUE) */
        }

        /* Process quick timer tick */
        if (event & NCI_QUICK_TIMER_EVT_MASK)
        {
            nci_process_quick_timer_evt ();
        }
    }

    NCI_TRACE_DEBUG0("nci_task terminated");

    GKI_exit_task (GKI_get_taskid ());
    return 0;
}

/*******************************************************************************
**
** Function         NCI_SetTraceLevel
**
** Description      This function sets the trace level for NCI.  If called with
**                  a value of 0xFF, it simply returns the current trace level.
**
** Returns          The new or current trace level
**
*******************************************************************************/
UINT8 NCI_SetTraceLevel (UINT8 new_level)
{
    if (new_level != 0xFF)
        nci_cb.trace_level = new_level;

    return (nci_cb.trace_level);
}

#endif /* NFC_INCLUDED == TRUE */
