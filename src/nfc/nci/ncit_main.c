/*****************************************************************************
**
**  Name:          nci_main.c
**
**  Description:   Functions for handling NCI Transport events
**
**  Copyright (c) 2010-2012, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
******************************************************************************/
#include <string.h>
#include "gki.h"
#include "nfc_target.h"
#include "bt_types.h"
#include "userial.h"
#include "upio.h"


#if (NFC_INCLUDED == TRUE)
#include "nfc_api.h"
#include "nfc_int.h"
#include "nci_int.h"

/****************************************************************************
** Definitions
****************************************************************************/

/* Default NCI port configuration  */
NCIT_CFG_QUALIFIER tNCIT_CFG ncit_cfg =
{
    NFC_SHARED_TRANSPORT_ENABLED,   /* bSharedTransport */
    USERIAL_BAUD_115200,            /* Baud rate */
    USERIAL_FC_HW                   /* Flow control */
};

/* Control block for NCI transport */
#if NFC_DYNAMIC_MEMORY == FALSE
tNCIT_CB ncit_cb;
#endif

/****************************************************************************
** Internal function prototypes
****************************************************************************/
static void ncit_userial_cback (tUSERIAL_PORT port, tUSERIAL_EVT evt, tUSERIAL_EVT_DATA *p_data);

/*******************************************************************************
**
** Function         ncit_init
**
** Description      This function initializes control block for NCIT
**
** Returns          nothing
**
*******************************************************************************/
void ncit_init (void)
{
    /* Clear nci control block */
    memset (&ncit_cb, 0, sizeof (tNCIT_CB));

    ncit_cb.trace_level                  = NFC_INITIAL_TRACE_LEVEL;
}

/*******************************************************************************
**
** Function         ncit_open_transport
**
** Description      Open transport and reset transport control block; prepare for
**                  new incoming message;
**
** Returns          nothing
**
*******************************************************************************/
static void ncit_open_transport (void)
{
    tUSERIAL_OPEN_CFG open_cfg;

    /* Initialize control block */
    ncit_cb.rcv_state = ncit_cb.init_rcv_state;

    if (ncit_cb.p_rcv_msg)
    {
        GKI_freebuf (ncit_cb.p_rcv_msg);
        ncit_cb.p_rcv_msg = NULL;
    }
    ncit_cb.p_vs_rcv_cback  = NULL;

    /* open transport */
    open_cfg.fmt    = (USERIAL_DATABITS_8 | USERIAL_PARITY_NONE | USERIAL_STOPBITS_1);
    open_cfg.baud   = ncit_cfg.userial_baud;
    open_cfg.fc     = ncit_cfg.userial_fc;
    open_cfg.buf    = USERIAL_BUF_BYTE;

    USERIAL_Open (USERIAL_NFC_PORT, &open_cfg, ncit_userial_cback);

    /* notify transport openned */
    if (ncit_cb.p_vs_evt_hdlr)
    {
        /* start VS intialization */
        /* VS module must call ncit_task_vs_init_done () when complete of VS initilaization */
        (*ncit_cb.p_vs_evt_hdlr) (NCIT_INT_VS_INIT_EVT, NULL);
    }
    else
    {
        /* Notify NFC Task that transport is ready */
        ncit_task_vs_init_done ();
    }
}

/*****************************************************************************
**
** Function         ncit_send_msg
**
** Description      Send message to NCIT task
**
** Returns          nothing
**
*****************************************************************************/
void ncit_send_msg (BT_HDR *p_msg)
{
    GKI_send_msg (NCIT_TASK, NCIT_TASK_MBOX, p_msg);
}

/*******************************************************************************
**
** Function         ncit_assemble_msg
**
** Description      This function is called to reassemble the received NCI
**                  response/notification packet, if required.
**                  (The data packets are posted to NFC task for reassembly)
**
** Returns          void.
**
*******************************************************************************/
static void ncit_assemble_msg (void)
{
    BT_HDR *p_msg = ncit_cb.p_rcv_msg;
    UINT8 u8;
    UINT8 *p, *pp;
    UINT8 hdr[2];
    UINT8   *ps, *pd;
    UINT16  size, needed;
    BOOLEAN disp_again = FALSE;

    if ((p_msg == NULL) || (p_msg->len < NCI_MSG_HDR_SIZE))
        return;

#ifdef DISP_NCI
    DISP_NCI ((UINT8 *) (p_msg + 1) + p_msg->offset, (UINT16) (p_msg->len), TRUE);
#endif

    p       = (UINT8 *) (p_msg + 1) + p_msg->offset;
    u8      = *p++;
    /* remove the PBF bit for potential reassembly later */
    hdr[0]  = u8 & ~NCI_PBF_MASK;
    if ((u8 & NCI_MT_MASK) == NCI_MT_DATA)
    {
        /* clear the RFU in octet1 */
        *(p) = 0;
        /* data packet reassembly is performed in NFC task */
        return;
    }
    else
    {
        *(p) &= NCI_OID_MASK;
    }

    hdr[1]  = *p;
    pp = hdr;
    /* save octet0 and octet1 of an NCI header in layer_specific for the received packet */
    STREAM_TO_UINT16 (p_msg->layer_specific, pp);

    if (ncit_cb.p_frag_msg)
    {
        if (ncit_cb.p_frag_msg->layer_specific != p_msg->layer_specific)
        {
            /* check if these fragments are of the same NCI message */
            NFC_TRACE_ERROR2 ("ncit_assemble_msg - different messages 0x%x, 0x%x!!", ncit_cb.p_frag_msg->layer_specific, p_msg->layer_specific);
            ncit_cb.nci_ras  |= NCI_RAS_ERROR;
        }
        else if (ncit_cb.nci_ras == 0)
        {
            disp_again = TRUE;
            /* if not previous reassembly error, append the new fragment */
            p_msg->offset   += NCI_MSG_HDR_SIZE;
            p_msg->len      -= NCI_MSG_HDR_SIZE;
            size    = GKI_get_buf_size (ncit_cb.p_frag_msg);
            needed  = (BT_HDR_SIZE + ncit_cb.p_frag_msg->len + ncit_cb.p_frag_msg->offset + p_msg->len);
            if (size >= needed)
            {
                /* the buffer for reassembly is big enough to append the new fragment */
                ps   = (UINT8 *) (p_msg + 1) + p_msg->offset;
                pd   = (UINT8 *) (ncit_cb.p_frag_msg + 1) + ncit_cb.p_frag_msg->offset + ncit_cb.p_frag_msg->len;
                memcpy (pd, ps, p_msg->len);
                ncit_cb.p_frag_msg->len  += p_msg->len;
                /* adjust the NCI packet length */
                pd   = (UINT8 *) (ncit_cb.p_frag_msg + 1) + ncit_cb.p_frag_msg->offset + 2;
                *pd  = (UINT8) (ncit_cb.p_frag_msg->len - NCI_MSG_HDR_SIZE);
            }
            else
            {
                ncit_cb.nci_ras  |= NCI_RAS_TOO_BIG;
                NFC_TRACE_ERROR2 ("ncit_assemble_msg buffer overrun (%d + %d)!!", ncit_cb.p_frag_msg->len, p_msg->len);
            }
        }
        /* we are done with this new fragment, free it */
        GKI_freebuf (p_msg);
    }
    else
    {
        ncit_cb.p_frag_msg = p_msg;
    }


    if ((u8 & NCI_PBF_MASK) == NCI_PBF_NO_OR_LAST)
    {
        /* last fragment */
        p_msg               = ncit_cb.p_frag_msg;
        p                   = (UINT8 *) (p_msg + 1) + p_msg->offset;
        *p                  = u8; /* this should make the PBF flag as Last Fragment */
        ncit_cb.p_frag_msg   = NULL;

        p_msg->layer_specific = ncit_cb.nci_ras;
        /* still report the data packet, if the incoming packet is too big */
        if (ncit_cb.nci_ras & NCI_RAS_ERROR)
        {
            /* NFCC reported NCI fragments for different NCI messages and this is the last fragment - drop it */
            NFC_TRACE_ERROR0 ("ncit_assemble_msg clearing NCI_RAS_ERROR");
            GKI_freebuf (p_msg);
            p_msg = NULL;
        }
#ifdef DISP_NCI
        if ((ncit_cb.nci_ras == 0) && (disp_again))
        {
            DISP_NCI ((UINT8 *) (p_msg + 1) + p_msg->offset, (UINT16) (p_msg->len), TRUE);
        }
#endif
        /* clear the error flags, so the next NCI packet is clean */
        ncit_cb.nci_ras = 0;
    }
    else
    {
        /* still reassembling */
        p_msg               = NULL;
    }

    ncit_cb.p_rcv_msg = p_msg;
}

/*******************************************************************************
**
** Function         ncit_send_error
**
** Description      send an Error event to NFC task
**
** Returns          nothing
**
*******************************************************************************/
void ncit_send_error (UINT16 error_code)
{
    BT_HDR *p_msg;

    if ((p_msg = (BT_HDR *) GKI_getbuf (BT_HDR_SIZE)) != NULL)
    {
        /* Initialize BT_HDR */
        p_msg->len    = 0;
        p_msg->event  = BT_EVT_TO_NFC_ERR;
        p_msg->offset = 0;
        p_msg->layer_specific = error_code;
        GKI_send_msg (NFC_TASK, NFC_MBOX_ID, p_msg);
    }
    else
    {
        NCI_TRACE_ERROR1 ("ncit_send_error: %d. No buffers.", error_code);
    }
}

/*******************************************************************************
**
** Function         ncit_send_cmd
**
** Description      Send NCI command to the transport
**
** Returns          void
**
*******************************************************************************/
void ncit_send_cmd (BT_HDR *p_buf)
{
    BOOLEAN continue_to_process = TRUE;
    UINT8   *ps, *pd;
    UINT16  max_len;
    UINT16  buf_len, offset;
    UINT8   *p;
    UINT8           hdr[NCI_VSC_MSG_HDR_SIZE];
    UINT8   nci_ctrl_size = nfc_cb.nci_ctrl_size;
    UINT8   delta = 0;

    if (ncit_cb.p_vs_evt_hdlr)
    {
        /* if NFCC is ready to receive NCI command */
        continue_to_process = (*ncit_cb.p_vs_evt_hdlr) (NCIT_INT_CHECK_NFCC_STATE_EVT, p_buf);
    }

    if (!continue_to_process)
    {
        /* save the command to be sent until NFCC is free. */
        ncit_cb.p_pend_cmd   = p_buf;
        return;
    }

    max_len = nci_ctrl_size + NCI_MSG_HDR_SIZE;
    buf_len = p_buf->len;
    offset  = p_buf->offset;
#ifdef DISP_NCI
    if (buf_len > max_len)
    {
        /* this command needs to be fragmented. display the complete packet first */
        DISP_NCI ((UINT8 *) (p_buf + 1) + p_buf->offset, p_buf->len, FALSE);
    }
#endif
    ps      = (UINT8 *) (p_buf + 1) + p_buf->offset;
    memcpy (hdr, ps, NCI_MSG_HDR_SIZE);
    while (buf_len > max_len)
    {
        NFC_TRACE_DEBUG2 ("buf_len (%d) > max_len (%d)", buf_len, max_len);
        /* the NCI command is bigger than the NFCC Max Control Packet Payload Length
         * fragment the command */

        p_buf->len  = max_len;
        ps   = (UINT8 *) (p_buf + 1) + p_buf->offset;
        /* mark the control packet as fragmented */
        *ps |= NCI_PBF_ST_CONT;
        /* adjust the length of this fragment */
        ps  += 2;
        *ps  = nci_ctrl_size;

        /* check if extra header for this packet */
        if (ncit_cb.p_vs_evt_hdlr)
            (*ncit_cb.p_vs_evt_hdlr) (NCIT_INT_SEND_NCI_MSG_EVT, p_buf);

        /* send this fragment to transport */
        p = (UINT8 *) (p_buf + 1) + p_buf->offset;

#ifdef DISP_NCI
        delta = p_buf->len - max_len;
        DISP_NCI (p + delta, (UINT16) (p_buf->len - delta), FALSE);
#endif
        USERIAL_Write (USERIAL_NFC_PORT, p, p_buf->len);

        /* adjust the len and offset to reflect that part of the command is already sent */
        buf_len -= nci_ctrl_size;
        offset  += nci_ctrl_size;
        NFC_TRACE_DEBUG2 ("p_buf->len: %d buf_len (%d)", p_buf->len, buf_len);
        p_buf->len      = buf_len;
        p_buf->offset   = offset;
        pd   = (UINT8 *) (p_buf + 1) + p_buf->offset;
        /* restore the NCI header */
        memcpy (pd, hdr, NCI_MSG_HDR_SIZE);
        pd  += 2;
        *pd  = (UINT8) (p_buf->len - NCI_MSG_HDR_SIZE);
    }

    NFC_TRACE_DEBUG1 ("p_buf->len: %d", p_buf->len);
    /* check if extra header for this packet */
    if (ncit_cb.p_vs_evt_hdlr)
        (*ncit_cb.p_vs_evt_hdlr) (NCIT_INT_SEND_NCI_MSG_EVT, p_buf);

    /* send this fragment to transport */
    p = (UINT8 *) (p_buf + 1) + p_buf->offset;

#ifdef DISP_NCI
    delta = p_buf->len - buf_len;
    DISP_NCI (p + delta, (UINT16) (p_buf->len - delta), FALSE);
#endif
    USERIAL_Write (USERIAL_NFC_PORT, p, p_buf->len);

    GKI_freebuf (p_buf);
}


/*****************************************************************************
**
** Function         ncit_receive_msg
**
** Description
**      Handle incoming data (NCI events) from the serial port.
**
**      If there is data waiting from the serial port, this funciton reads the
**      data and parses it. Once an entire NCI message has been read, it sends
**      the message the the NFC_TASK for processing
**
*****************************************************************************/
static BOOLEAN ncit_receive_msg (tNCIT_CB *p_cb, UINT8 byte)
{
    UINT16      len;
    BOOLEAN     msg_received = FALSE;

    switch (p_cb->rcv_state)
    {
    case NCIT_RCV_IDLE_ST:

        /* Initialize rx parameters */
        p_cb->rcv_state = NCIT_RCV_NCI_HDR_ST;
        p_cb->rcv_len   = NCI_MSG_HDR_SIZE;

        /* Start of new message. Allocate a buffer for message */
        if ((p_cb->p_rcv_msg = (BT_HDR *) GKI_getpoolbuf (NFC_NCI_POOL_ID)) != NULL)
        {
            /* Initialize BT_HDR */
            p_cb->p_rcv_msg->len    = 0;
            p_cb->p_rcv_msg->event  = BT_EVT_TO_NFC_NCI;
            p_cb->p_rcv_msg->offset = NFC_RECEIVE_MSGS_OFFSET;

            *((UINT8 *) (p_cb->p_rcv_msg + 1) + p_cb->p_rcv_msg->offset + p_cb->p_rcv_msg->len++) = byte;
        }
        else
        {
            NCI_TRACE_ERROR0 ("Unable to allocate buffer for incoming NCI message.");
        }
        p_cb->rcv_len--;
        break;

    case NCIT_RCV_NCI_HDR_ST:

        if (p_cb->p_rcv_msg)
        {
            *((UINT8 *) (p_cb->p_rcv_msg + 1) + p_cb->p_rcv_msg->offset + p_cb->p_rcv_msg->len++) = byte;
        }

        p_cb->rcv_len--;

        /* Check if we read in entire NFC message header yet */
        if (p_cb->rcv_len == 0)
        {
            p_cb->rcv_len       = byte;

            /* If non-zero payload, then go to receive-data state */
            if (byte > 0)
            {
                p_cb->rcv_state = NCIT_RCV_NCI_PAYLOAD_ST;
            }
            else
            {
                msg_received    = TRUE;
                p_cb->rcv_state = p_cb->init_rcv_state;
            }
        }
        break;

    case NCIT_RCV_NCI_PAYLOAD_ST:

        p_cb->rcv_len--;
        if (p_cb->p_rcv_msg)
        {
            *((UINT8 *) (p_cb->p_rcv_msg + 1) + p_cb->p_rcv_msg->offset + p_cb->p_rcv_msg->len++) = byte;

            if (p_cb->rcv_len > 0)
            {
                /* Read in the rest of the message */
                len = USERIAL_Read (USERIAL_NFC_PORT, ((UINT8 *) (p_cb->p_rcv_msg + 1) + p_cb->p_rcv_msg->offset + p_cb->p_rcv_msg->len),  p_cb->rcv_len);
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
** Function         ncit_userial_cback
**
** Description      USERIAL callback for NCI transport
**
** Returns          nothing
**
*******************************************************************************/
static void ncit_userial_cback (tUSERIAL_PORT port, tUSERIAL_EVT evt, tUSERIAL_EVT_DATA *p_data)
{
    if (evt == USERIAL_RX_READY_EVT)
    {
        /* Notify transport task of serial port event */
        GKI_send_event (NCIT_TASK, NCIT_TASK_EVT_DATA_RDY);
    }
    else if (evt == USERIAL_TX_DONE_EVT)
    {
        /* Serial driver has finshed sending data from USERIAL_Write */
        /* Currently, no action is needed for this event */
    }
    else if (evt == USERIAL_ERR_EVT)
    {
        NCI_TRACE_ERROR0 ("ncit_userial_cback: USERIAL_ERR_EVT. Notifying NFC_TASK of transport error");
        ncit_send_error (NFC_ERR_TRANSPORT);
    }
    else if (evt == USERIAL_WAKEUP_EVT)
    {
        NCI_TRACE_DEBUG1 ("ncit_userial_cback: USERIAL_WAKEUP_EVT: %d", p_data->sigs);
    }
    else
    {
        NCI_TRACE_DEBUG1 ("ncit_userial_cback: unhandled userial evt: %i", evt);
    }
}

/*******************************************************************************
**
** Function         ncit_task_vs_init_done
**
** Description      notify complete of vendor specific initialization
**
** Returns          nothing
**
*******************************************************************************/
void ncit_task_vs_init_done (void)
{
    NCI_TRACE_DEBUG0 ("ncit_task_vs_init_done ()");

    /* Notify NFC Task that transport is ready */
    GKI_send_event (NFC_TASK, NFC_TASK_EVT_TRANSPORT_READY);
}

/*******************************************************************************
**
** Function         ncit_task_timeout_cback
**
** Description      callback function for timeout
**
** Returns          void
**
*******************************************************************************/
void ncit_task_timeout_cback (void *p_tle)
{
    TIMER_LIST_ENT  *p_tlent = (TIMER_LIST_ENT *) p_tle;

    NCI_TRACE_DEBUG0 ("ncit_task_timeout_cback ()");

    switch (p_tlent->event)
    {
    case NCIT_TTYPE_POWER_CYCLE:
        ncit_open_transport ();
        break;

    default:
        NFC_TRACE_DEBUG1 ("ncit_task_timeout_cback: unhandled timer event (0x%04x)", p_tlent->event);
        break;
    }
}

/*******************************************************************************
**
** Function         ncit_task_handle_terminate
**
** Description      Handle NFI transport shutdown
**
** Returns          nothing
**
*******************************************************************************/
static void ncit_task_handle_terminate (void)
{
    BT_HDR *p_msg;

    /* dequeue and free buffer */
    if (ncit_cb.p_pend_cmd != NULL)
    {
        GKI_freebuf (ncit_cb.p_pend_cmd);
        ncit_cb.p_pend_cmd = NULL;
    }

    /* Free unsent nfc rx buffer */
    if (ncit_cb.p_rcv_msg)
    {
        GKI_freebuf (ncit_cb.p_rcv_msg);
        ncit_cb.p_rcv_msg  = NULL;
    }

    /* Free buffer for pending fragmented response/notification */
    if (ncit_cb.p_frag_msg)
    {
        GKI_freebuf (ncit_cb.p_frag_msg);
        ncit_cb.p_frag_msg = NULL;
    }

    /* Free buffers in the tx mbox */
    while ((p_msg = (BT_HDR *) GKI_read_mbox (NCIT_TASK_MBOX)) != NULL)
    {
        GKI_freebuf (p_msg);
    }

    /* notify closing transport */
    if (ncit_cb.p_vs_evt_hdlr)
    {
        (*ncit_cb.p_vs_evt_hdlr) (NCIT_INT_TERMINATE_EVT, NULL);
    }
}

/*******************************************************************************
**
** Function         ncit_start_quick_timer
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
void ncit_start_quick_timer (TIMER_LIST_ENT *p_tle, UINT16 type, UINT32 timeout)
{
    BT_HDR *p_msg;

    /* if timer list is currently empty, start periodic GKI timer */
    if (ncit_cb.quick_timer_queue.p_first == NULL)
    {
        /* if timer starts on other than NCIT task (script wrapper) */
        if(GKI_get_taskid () != NCIT_TASK)
        {
            /* post event to start timer in NCIT task */
            if ((p_msg = (BT_HDR *) GKI_getbuf (BT_HDR_SIZE)) != NULL)
            {
                p_msg->event = BT_EVT_TO_START_QUICK_TIMER;
                GKI_send_msg (NCIT_TASK, NCIT_TASK_MBOX, p_msg);
            }
        }
        else
        {
            GKI_start_timer (NCIT_QUICK_TIMER_ID, ((GKI_SECS_TO_TICKS (1) / QUICK_TIMER_TICKS_PER_SEC)), TRUE);
        }
    }

    GKI_remove_from_timer_list (&ncit_cb.quick_timer_queue, p_tle);

    p_tle->event = type;
    p_tle->ticks = timeout; /* Save the number of ticks for the timer */

    GKI_add_to_timer_list (&ncit_cb.quick_timer_queue, p_tle);
}

/*******************************************************************************
**
** Function         ncit_stop_quick_timer
**
** Description      Stop a timer.
**
** Returns          void
**
*******************************************************************************/
void ncit_stop_quick_timer (TIMER_LIST_ENT *p_tle)
{
    GKI_remove_from_timer_list (&ncit_cb.quick_timer_queue, p_tle);

    /* if timer list is empty stop periodic GKI timer */
    if (ncit_cb.quick_timer_queue.p_first == NULL)
    {
        GKI_stop_timer (NCIT_QUICK_TIMER_ID);
    }
}

/*******************************************************************************
**
** Function         ncit_process_quick_timer_evt
**
** Description      Process quick timer event
**
** Returns          void
**
*******************************************************************************/
static void ncit_process_quick_timer_evt (void)
{
    TIMER_LIST_ENT  *p_tle;

    GKI_update_timer_list (&ncit_cb.quick_timer_queue, 1);

    while ((ncit_cb.quick_timer_queue.p_first) && (!ncit_cb.quick_timer_queue.p_first->ticks))
    {
        p_tle = ncit_cb.quick_timer_queue.p_first;
        GKI_remove_from_timer_list (&ncit_cb.quick_timer_queue, p_tle);

        if (p_tle->p_cback)
        {
            (*p_tle->p_cback) (p_tle);
        }
    }

    /* if timer list is empty stop periodic GKI timer */
    if (ncit_cb.quick_timer_queue.p_first == NULL)
    {
        GKI_stop_timer (NCIT_QUICK_TIMER_ID);
    }
}

/*******************************************************************************
**
** Function         ncit_send_message
**
** Description      This function is calledto send an NCI message.
**
** Returns          void
**
*******************************************************************************/
static void ncit_send_message (BT_HDR *p_msg)
{
    UINT8   *ps;
    UINT8   delta;
    UINT16  len = p_msg->len;

    NFC_TRACE_DEBUG1 ("ncit_send_message ls:0x%x", p_msg->layer_specific);
    if (  (p_msg->layer_specific == NCI_WAIT_RSP_CMD)
        ||(p_msg->layer_specific == NCI_WAIT_RSP_VSC)  )
    {
        ncit_send_cmd (p_msg);
    }
    else
    {
        /* NFC task has fragmented the data packet to the appropriate size
         * and data credit is available; just send it */

        /* check if extra header for this packet */
        if (ncit_cb.p_vs_evt_hdlr)
            (*ncit_cb.p_vs_evt_hdlr) (NCIT_INT_SEND_NCI_MSG_EVT, p_msg);

        /* send this packet to transport */
        ps = (UINT8 *) (p_msg + 1) + p_msg->offset;
#ifdef DISP_NCI
        delta = p_msg->len - len;
        DISP_NCI (ps + delta, (UINT16) (p_msg->len - delta), FALSE);
#endif
        USERIAL_Write (USERIAL_NFC_PORT, ps, p_msg->len);
        GKI_freebuf (p_msg);
    }
}





/*******************************************************************************
**
** Function         ncit_task
**
** Description      NCI transport event processing task
**
** Returns          0
**
*******************************************************************************/
UINT32 ncit_task (UINT32 param)
{
    UINT16  event;
    BT_HDR  *p_msg;
    UINT8   byte;

    NFC_TRACE_DEBUG0 ("NCIT_TASK started");

    /* Main loop */
    while (TRUE)
    {
        event = GKI_wait (0xFFFF, 0);

        /* Handle NCIT_TASK_EVT_INITIALIZE (for initializing NCI transport) */
        if (event & NCIT_TASK_EVT_INITIALIZE)
        {
            NCI_TRACE_DEBUG0 ("NCIT_TASK got NCIT_TASK_EVT_INITIALIZE signal. Opening NFC transport...");

            ncit_open_transport ();
        }

        /* Check for terminate event */
        if (event & NCIT_TASK_EVT_TERMINATE)
        {
            NCI_TRACE_DEBUG0 ("NCIT_TASK got NCIT_TASK_EVT_TERMINATE");
            ncit_task_handle_terminate ();

            /* Close uart */
            USERIAL_Close (USERIAL_NFC_PORT);

            continue;
        }

        /* Check for power cycle event */
        if (event & NCIT_TASK_EVT_POWER_CYCLE)
        {
            NCI_TRACE_DEBUG0 ("NCIT_TASK got NCIT_TASK_EVT_POWER_CYCLE");
            ncit_task_handle_terminate ();

            /* Close uart */
            USERIAL_Close (USERIAL_NFC_PORT);

            /* power cycle timeout */
            ncit_cb.timer.p_cback = ncit_task_timeout_cback;
            ncit_start_quick_timer (&ncit_cb.timer, NCIT_TTYPE_POWER_CYCLE,
                                   (NCI_POWER_CYCLE_DELAY*QUICK_TIMER_TICKS_PER_SEC)/1000);
            continue;
        }

        /* NCI message ready to be sent to NFCC */
        if (event & NCIT_TASK_EVT_MBOX)
        {
            while ((p_msg = (BT_HDR *) GKI_read_mbox (NCIT_TASK_MBOX)) != NULL)
            {
                switch (p_msg->event & BT_EVT_MASK)
                {
                case BT_EVT_TO_NFC_NCI:
                    ncit_send_message (p_msg);
                    /* do not free buffer. NCI VS code may keep it for processing later */
                    break;

                case BT_EVT_TO_START_QUICK_TIMER:
                    GKI_start_timer (NCIT_QUICK_TIMER_ID, ((GKI_SECS_TO_TICKS (1) / QUICK_TIMER_TICKS_PER_SEC)), TRUE);
                    GKI_freebuf (p_msg);
                    break;

                default:
                    /* processing vendor specific event */
                    if (ncit_cb.p_vs_evt_hdlr)
                    {
                        (*ncit_cb.p_vs_evt_hdlr) (NCIT_INT_VS_MSG_EVT, p_msg);
                        /* VS function must free buffer after processing */
                    }
                    else
                    {
                        GKI_freebuf (p_msg);
                    }
                    break;
                }

            }
        }

        /* Data waiting to be read from serial port */
        if (event & NCIT_TASK_EVT_DATA_RDY)
        {
            while (TRUE)
            {
                /* Read one byte to see if there is anything waiting to be read */
                if (USERIAL_Read (USERIAL_NFC_PORT, &byte, 1) == 0)
                {
                    break;
                }

                /* if it is not proprietary message (NCI formated message) */
                if (ncit_cb.rcv_state != NCIT_RCV_PROP_MSG_ST)
                {
                    if (ncit_receive_msg (&ncit_cb, byte))
                    {
                        /* complete of receiving NCI message */
                        ncit_assemble_msg ();
                        if (ncit_cb.p_rcv_msg)
                        {
                            if (ncit_cb.p_vs_rcv_cback)
                                (*ncit_cb.p_vs_rcv_cback) (ncit_cb.p_rcv_msg);
                            else
                                GKI_send_msg (NFC_TASK, NFC_MBOX_ID, ncit_cb.p_rcv_msg);
                            ncit_cb.p_rcv_msg = NULL;
                        }
                    }
                }
                else
                {
                    /* process proprietary message (other than NCI format) */
                    if (ncit_cb.p_vs_evt_hdlr)
                    {
                        (*ncit_cb.p_vs_evt_hdlr) (NCIT_INT_DATA_RDY_EVT, &byte);
                    }
                }
            } /* while (TRUE) */
        }

        /* Process quick timer tick */
        if (event & NCIT_QUICK_TIMER_EVT_MASK)
        {
            ncit_process_quick_timer_evt ();
        }
    }

    NCI_TRACE_DEBUG0 ("ncit_task terminated");

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
        ncit_cb.trace_level = new_level;

    return (ncit_cb.trace_level);
}

#endif /* NFC_INCLUDED == TRUE */
