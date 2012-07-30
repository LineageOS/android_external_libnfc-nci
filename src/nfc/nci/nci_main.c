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
static tNCI_CONN_CB * nci_alloc_conn_cb (UINT8 conn_id);

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
    int xx;

    /* Clear nci control block */
    memset(&nci_cb, 0, sizeof (tNCI_CB));
    nci_cb.nci_cmd_cplt_tout= NFC_CMD_CMPL_TIMEOUT * QUICK_TIMER_TICKS_PER_SEC;
    nci_cb.nci_cmd_window   = NCI_MAX_CMD_WINDOW;
    /* Reset the nfc control block */
    for (xx = 0; xx < NCI_MAX_CONN_CBS; xx++)
    {
        nci_cb.conn_cb[xx].conn_id = NFC_ILLEGAL_CONN_ID;
    }

    nci_cb.trace_level = NFC_INITIAL_TRACE_LEVEL;
    nci_cb.nci_cmd_cmpl_timer.p_cback = nci_task_timeout_cback;
}

/*******************************************************************************
**
** Function         nci_set_cmd_timeout_val
**
** Description      This function initializes control block for NCI
**
** Returns          nothing
**
*******************************************************************************/
void nci_set_cmd_timeout_val (UINT16 timeout_val)
{
    NCI_TRACE_DEBUG2 ("nci_set_cmd_timeout_val () %d->%d", nci_cb.nci_cmd_cplt_tout, timeout_val);
    nci_cb.nci_cmd_cplt_tout = timeout_val;
}

/*******************************************************************************
**
** Function         nci_find_conn_cb_by_conn_id
**
** Description      This function is called to locate the control block for
**                  the given connection id
**
** Returns          The control block or NULL
**
*******************************************************************************/
static tNCI_CONN_CB * nci_find_conn_cb_by_conn_id(UINT8 conn_id)
{
    tNCI_CONN_CB *p_conn_cb = NULL;
    UINT8   handle;
    UINT8   id;

    id         = conn_id & NFC_CONN_ID_ID_MASK;
    if (id < NFC_MAX_CONN_ID)
    {
        handle = nci_cb.conn_id[id];
        if (handle > 0)
            p_conn_cb = &nci_cb.conn_cb[handle - 1];
    }

    return p_conn_cb;
}
/*******************************************************************************
**
** Function         nci_release_conn_cb_buffers
**
** Description      This function is called to free the resources of the given
**                  control block
**
** Returns          void
**
*******************************************************************************/
static void nci_release_conn_cb_buffers (tNCI_CONN_CB *p_cb)
{
    void    *p_buf;

    while ( (p_buf = GKI_dequeue(&p_cb->tx_q)) != NULL)
        GKI_freebuf (p_buf);
    while ( (p_buf = GKI_dequeue(&p_cb->rx_q)) != NULL)
        GKI_freebuf (p_buf);
}

/*******************************************************************************
**
** Function         nci_free_conn_cb
**
** Description      This function is called to free the control block and
**                  resources and id mapping table
**
** Returns          void
**
*******************************************************************************/
static void nci_free_conn_cb (tNCI_CONN_CB *p_cb)
{
    if (p_cb == NULL)
        return;

    nci_release_conn_cb_buffers (p_cb);
    nci_cb.conn_id[p_cb->conn_id]   = 0;
    p_cb->conn_id                   = NFC_ILLEGAL_CONN_ID;
}

/*******************************************************************************
**
** Function         nci_alloc_conn_cb
**
** Description      This function is called to allocation a control block for
**                  NCI logical connection
**
** Returns          The allocated control block or NULL
**
*******************************************************************************/
static tNCI_CONN_CB * nci_alloc_conn_cb (UINT8 conn_id)
{
    int xx, max = NCI_MAX_CONN_CBS;
    tNCI_CONN_CB *p_conn_cb = NULL;
    UINT8   handle;

    NFC_CHECK_MAX_CONN();
    p_conn_cb = nci_find_conn_cb_by_conn_id (conn_id);
    if (p_conn_cb)
    {
        nci_release_conn_cb_buffers (p_conn_cb);
    }
    else
    {
        for (xx = 0; xx < max; xx++)
        {
            if (nci_cb.conn_cb[xx].conn_id == NFC_ILLEGAL_CONN_ID)
            {
                p_conn_cb                   = &nci_cb.conn_cb[xx];
                break;
            }
        }
    }

    if (p_conn_cb)
    {
        p_conn_cb->conn_id      = conn_id;
        handle                  = (UINT8)(p_conn_cb - nci_cb.conn_cb + 1);
        nci_cb.conn_id[conn_id] = handle;
    }
    return p_conn_cb;
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

/*******************************************************************************
**
** Function         nci_copy_msg
**
** Description      This function is called to copy the message held in p_src
**                  to p_dst
**
** Returns          void
**
*******************************************************************************/
static void nci_copy_msg(BT_HDR *p_dst, BT_HDR *p_src, UINT16 len)
{
    UINT8   *ps, *pd;
    p_dst->event            = p_src->event;
    p_dst->len              = len;
    p_dst->offset           = NFC_RECEIVE_MSGS_OFFSET;
    p_dst->layer_specific   = p_src->layer_specific;
    pd                      = (UINT8 *)(p_dst + 1) + p_dst->offset;
    ps                      = (UINT8 *)(p_src + 1) + p_src->offset;
    memcpy(pd, ps, len);
}

/*******************************************************************************
**
** Function         nci_reassemble_msg
**
** Description      This function is called to add the given fragment
**                  to nci_cb.p_frag_msg
**
** Returns          void
**
*******************************************************************************/
static void nci_reassemble_msg(BT_HDR *p_msg)
{
    BT_HDR  *p = nci_cb.p_frag_msg;
    BT_HDR  *p_max = NULL;
    UINT8   *ps, *pd;
    UINT16  len;
    UINT16  size;

    if (nci_cb.nci_ras)
    {
        /* drop the new packet on either incoming packet too big or reassembly error */
        NFC_TRACE_ERROR2 ("nci_reassemble_msg nci_ras:0x%x, len=0x%x!!", nci_cb.nci_ras, p_msg->len);
        GKI_freebuf (p_msg);
        return;
    }

    ps   = (UINT8 *)(p_msg + 1) + p_msg->offset;
    if (nci_cb.p_frag_msg == NULL)
    {
        NFC_TRACE_DEBUG0 ("First Fragment!!");
        p                   = NCI_GET_CMD_BUF(NCI_MAX_CTRL_SIZE);
        if (p == NULL)
        {
            NFC_TRACE_WARNING0 ("nci_reassemble_msg - No Buffer for reassembly!!");
            nci_cb.p_frag_msg = p_msg;
            return;
        }
        /* copy only the NCI header */
        nci_copy_msg (p, p_msg, NCI_MSG_HDR_SIZE);
        nci_cb.p_frag_msg   = p;
        pd                  = (UINT8 *)(p + 1) + p->offset;
        /* clear the PBF flag */
        *pd                &= ~NCI_PBF_MASK;
    }
    else if (nci_cb.p_frag_msg->layer_specific != p_msg->layer_specific)
    {
        /* check if these fragments are of the same NCI message */
        NFC_TRACE_ERROR2 ("nci_reassemble_msg - different messages 0x%x, 0x%x!!", nci_cb.p_frag_msg->layer_specific, p_msg->layer_specific);
        nci_cb.nci_ras  |= NCI_RAS_ERROR;
    }
    ps   += NCI_MSG_HDR_SIZE;

    /* copy the rest of the fragment */
    if (p_msg->len > NCI_MSG_HDR_SIZE)
    {
        size = GKI_get_buf_size(p);
        len  = p_msg->len - NCI_MSG_HDR_SIZE;
        if (size < (BT_HDR_SIZE + p->len + p->offset + len))
        {
            /* the current size of p_frag_msg is not big enough to hold the new fragment, p_msg */
            if (size != GKI_MAX_BUF_SIZE)
            {
                /* try the biggest GKI pool */
                p_max = (BT_HDR *)GKI_getpoolbuf (GKI_MAX_BUF_SIZE_POOL_ID);
                if (p_max)
                {
                    nci_copy_msg(p_max, p, p->len);
                    GKI_freebuf (p);
                    p       = p_max;
                    nci_cb.p_frag_msg = p;
                }
            }
            if (p_max == NULL)
            {
                nci_cb.nci_ras  |= NCI_RAS_TOO_BIG;
                NFC_TRACE_ERROR1 ("nci_reassemble_msg buffer overrun(%d)!!", len);
            }
        }

        if ((nci_cb.nci_ras & NCI_RAS_TOO_BIG) == 0)
        {
            pd   = (UINT8 *)(p + 1) + p->offset + p->len;
            memcpy(pd, ps, len);
            p->len  += len;
            /* adjust the NCI packet length */
            pd   = (UINT8 *)(p + 1) + p->offset + 2;
            *pd  = (UINT8)(p->len - NCI_MSG_HDR_SIZE);
            NFC_TRACE_DEBUG1 ("nci_reassemble_msg len:%d", p->len);
        }
    }

    GKI_freebuf (p_msg);
}


/*******************************************************************************
**
** Function         nci_check_fragmentation
**
** Description      This function is called to check if the received NCI
**                  response/notification/data packet is fragmented.
**
** Returns          void.
**
*******************************************************************************/
static void nci_check_fragmentation (void)
{
    BT_HDR *p_msg = nci_cb.p_rcv_msg;
    UINT8 u8;
    UINT8 *p, *pp;
    UINT8 hdr[2];

    if (p_msg == NULL)
        return;

    p       = (UINT8 *)(p_msg + 1) + p_msg->offset;
    u8      = *p++;
    /* remove the PBF bit for potential reassembly later */
    hdr[0]  = u8 & ~NCI_PBF_MASK;
    /* clear the RFU in octet1 */
    if ((u8 & NCI_MT_MASK) == NCI_MT_DATA)
        *(p) = 0;
    else
        *(p) &= NCI_OID_MASK;
    hdr[1]  = *p;
    pp = hdr;
    /* save octet0 and octet1 of an NCI header in layer_specific for the received packet */
    STREAM_TO_UINT16(p_msg->layer_specific, pp);

    if ((u8 & NCI_PBF_MASK) == NCI_PBF_ST_CONT)
    {
        NFC_TRACE_DEBUG4 ("Fragmented NCI message(len:%d: 0x%02x %02x %02x)!!", p_msg->len, u8, *p, *(p+1));
        nci_reassemble_msg (p_msg);
        p_msg = NULL;
    }
    else
    {
        if (nci_cb.p_frag_msg)
        {
            /* last fragment */
            nci_reassemble_msg (p_msg);
            p_msg               = nci_cb.p_frag_msg;
            p                   = (UINT8 *)(p_msg + 1) + p_msg->offset;
            *p                  = u8; /* this should make the PBF flag as Last Fragment */
            nci_cb.p_frag_msg   = NULL;
        }
        /* else just use the original p_msg */
    }

    if (p_msg)
    {
        p_msg->layer_specific = nci_cb.nci_ras;
        /* still report the data packet, if the incoming packet is too big */
        if (nci_cb.nci_ras & NCI_RAS_ERROR)
        {
            /* NFCC reported NCI fragments for different NCI messages and this is the last fragment - drop it */
            NFC_TRACE_ERROR0 ("nci_check_fragmentation clearing NCI_RAS_ERROR");
            p_msg = NULL;
        }
        /* clear the error flags, so the next NCI packet is clean */
        nci_cb.nci_ras = 0;
    }
    nci_cb.p_rcv_msg = p_msg;
}

/*******************************************************************************
**
** Function         nci_send_error
**
** Description      send an Error event to NFC task
**
** Returns          nothing
**
*******************************************************************************/
static void nci_send_error (UINT16 error_code)
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
        NCI_TRACE_ERROR1 ("nci_cback: nci_send_error: %d. No buffers.", error_code);
    }
}

/*******************************************************************************
**
** Function         nci_update_window 
**
** Description      Update tx cmd window to indicate that NFCC can received
**                  is_rsp=TRUE if receiving valid NCI rsp from NFCC
**                  is_rsp=FALSE if timeout waiting for NCI rsp from NFCC
**
** Returns          void
**
*********************************************************************************/
void nci_update_window(BOOLEAN is_rsp)
{
    NFC_TRACE_DEBUG2 ("nci_update_window is_rsp:0x%x, win:%d", is_rsp, nci_cb.nci_cmd_window);
    /* Sanity check - see if we were expecting a update_window */
    if (nci_cb.nci_cmd_window == NCI_MAX_CMD_WINDOW)
    {
        NFC_TRACE_ERROR0("nci_update_window: Unexpected call");
        return;
    }

    /* Stop command-pending timer */
    nci_stop_quick_timer(&nci_cb.nci_cmd_cmpl_timer);

    if (is_rsp)
    {
        nci_cb.nci_cmd_window++;
        nci_cb.p_last_cback = NULL;
        nci_cb.nci_last_ls  = NCI_MSGS_LS_INVALID;
    }

    /* Check if there were any commands waiting to be sent */
    nci_send_cmd (NULL);
}

/*******************************************************************************
**
** Function         nci_cmd_timeout 
**
** Description      Handle a command timeout
**
** Returns          void
**
*******************************************************************************/
void nci_cmd_timeout (void)
{
    NFC_TRACE_ERROR0("nci_cmd_timeout");
    /* report an error */    
    nci_send_error (NFC_ERR_CMD_TIMEOUT);
    /* Send next command in the xmit_q */
    nci_update_window(FALSE);
}

/*******************************************************************************
**
** Function         nci_store_n_to_lower 
**
** Description      Copy part of the command and
**                  send the command to transport 
**
** Returns          TRUE if the buffer can be sent to transport
**
*******************************************************************************/
static BOOLEAN nci_store_n_to_lower (BT_HDR *p_buf)
{
    UINT8   *ps, *pd;
    UINT16  max_len;
    UINT16  buf_len, offset;
    BOOLEAN continue_to_process = TRUE;
    UINT8   *p;
    tNCI_INT_EVT    int_event;
    UINT8           hdr[NCI_VSC_MSG_HDR_SIZE];
    UINT8   nci_ctrl_size = nfc_cb.nci_ctrl_size;
    UINT8   delta;

    if (nci_cb.p_vs_evt_hdlr)
    {
        /* if NFCC is ready to receive NCI command */
        continue_to_process = (*nci_cb.p_vs_evt_hdlr) (NCI_INT_CHECK_STATE_EVT, p_buf);
    }

    if (!continue_to_process)
        return continue_to_process;

    nci_cb.nci_last_ls = (UINT8)(p_buf->layer_specific);
    if (p_buf->event == BT_EVT_TO_NFC_NCI)
    {
        /* save the message header to double check the response */
        ps   = (UINT8 *)(p_buf + 1) + p_buf->offset;
        memcpy(nci_cb.last_hdr, ps, NCI_SAVED_HDR_SIZE);
        memcpy(nci_cb.last_cmd, ps + NCI_MSG_HDR_SIZE, NFC_SAVED_CMD_SIZE);
        if (p_buf->layer_specific == NCI_MSGS_LS_NCI_VSC)
        {
            /* save the callback for NCI VSCs)  */
            nci_cb.p_last_cback = ((tNFC_NCI_VS_MSG *)p_buf)->p_cback;
        }

        int_event = NCI_INT_SEND_NCI_MSG_EVT;
    }
    else
    {
        int_event = p_buf->event;
    }


    max_len = nci_ctrl_size + NCI_MSG_HDR_SIZE;
    buf_len = p_buf->len;
    offset  = p_buf->offset;
    ps      = (UINT8 *)(p_buf + 1) + p_buf->offset;
    memcpy (hdr, ps, NCI_MSG_HDR_SIZE);
    while (buf_len > max_len)
    {
        NFC_TRACE_DEBUG2 ("buf_len(%d) > max_len(%d)", buf_len, max_len);
        /* the NCI command is bigger than the NFCC Max Control Packet Payload Length
         * fragment the command */

        p_buf->len  = max_len;
        ps   = (UINT8 *)(p_buf + 1) + p_buf->offset;
        /* mark the control packet as fragmented */
        *ps |= NCI_PBF_ST_CONT;
        /* adjust the length of this fragment */
        ps  += 2;
        *ps  = nci_ctrl_size;

        /* check if extra header for this packet */
        if (nci_cb.p_vs_evt_hdlr)
            (*nci_cb.p_vs_evt_hdlr) (int_event, p_buf);

        /* send this fragment to transport */
        p = (UINT8 *)(p_buf + 1) + p_buf->offset;
#ifdef DISP_NCI
        delta = p_buf->len - max_len;
        DISP_NCI (p + delta, (UINT16)(p_buf->len - delta), FALSE);
#endif
        USERIAL_Write (USERIAL_NFC_PORT, p, p_buf->len);

        /* adjust the len and offset to reflect that part of the command is already sent */
        buf_len -= nci_ctrl_size;
        offset  += nci_ctrl_size;
        NFC_TRACE_DEBUG2 ("p_buf->len: %d buf_len(%d)", p_buf->len, buf_len);
        p_buf->len      = buf_len;
        p_buf->offset   = offset;
        pd   = (UINT8 *)(p_buf + 1) + p_buf->offset;
        /* restore the NCI header */
        memcpy(pd, hdr, NCI_MSG_HDR_SIZE);
        pd  += 2;
        *pd  = (UINT8)(p_buf->len - NCI_MSG_HDR_SIZE);
    }

    NFC_TRACE_DEBUG1 ("p_buf->len: %d", p_buf->len);
    /* check if extra header for this packet */
    if (nci_cb.p_vs_evt_hdlr)
        (*nci_cb.p_vs_evt_hdlr) (int_event, p_buf);

    /* send this fragment to transport */
    p = (UINT8 *)(p_buf + 1) + p_buf->offset;
#ifdef DISP_NCI
    delta = p_buf->len - buf_len;
    DISP_NCI (p + delta, (UINT16)(p_buf->len - delta), FALSE);
#endif
    USERIAL_Write (USERIAL_NFC_PORT, p, p_buf->len);

    return continue_to_process;
}

/*******************************************************************************
**
** Function         nci_confirm_deactivate
**
** Description      This function is called to post event to NFC task that
**                  all outstanding data credit ON RF connection is received.
**                  It's OK to proceed to deactivate the RF channel.
**
** Returns          void
**
*******************************************************************************/
static void nci_confirm_deactivate (void)
{
    BT_HDR       *p_msgs;

    nci_cb.nci_flags    &= ~NCI_FLAGS_DEACTIVATING;
    p_msgs = (BT_HDR *) GKI_getbuf(sizeof (tNFC_ACTIVATE_MSGS));
    if (p_msgs)
    {
        p_msgs->event           = BT_EVT_TO_NFC_MSGS;
        p_msgs->layer_specific  = NFC_MSGS_RF_DEACT_CONFIRM;
        GKI_send_msg (NFC_TASK, NFC_MBOX_ID, p_msgs);
    }
}

/*******************************************************************************
**
** Function         nci_send_data
**
** Description      This function is called to check if it can send data
**                  to the NFCC. It may be passed the address of
**                  a packet to send.
**
** Returns          void
**
*******************************************************************************/
void nci_send_data (tNCI_CONN_CB *p_cb, BT_HDR *p_data)
{
    UINT8 *ps;
    UINT8   payload_max;
    UINT16  data_max;
    UINT16  buf_len, frag_len;
    UINT8   hdr[NCI_VSC_MSG_HDR_SIZE];
    UINT8   delta;
    UINT16  offset;

    if (p_cb == NULL)
        return;

    if ((p_cb->conn_id == NFC_RF_CONN_ID) && (nci_cb.nci_flags & NCI_FLAGS_DEACTIVATING))
    {
        if (p_data == NULL)/* called because credit from NFCC */
        {
            if (p_cb->init_credits == p_cb->num_buff)
            {
                /* all the credits are back */
                nci_confirm_deactivate();
            }
        }
        else
        {
            /* drop outgoing data packets when deactivating */
            GKI_freebuf (p_data);
        }
        return;
    }

    if (p_data)
        GKI_enqueue (&p_cb->tx_q, p_data);

    /* get the buffer reference of the first buffer in the queue.
     * the buffer is dequeued when it's completely sent */
    p_data = GKI_getfirst (&p_cb->tx_q);

    payload_max = p_cb->buff_size;
    data_max    = payload_max + NCI_MSG_HDR_SIZE;
    if (p_data)
    {
        ps   = (UINT8 *)(p_data + 1) + p_data->offset;
        memcpy (hdr, ps, NCI_MSG_HDR_SIZE);
        buf_len = p_data->len;
        offset  = p_data->offset;
        while ((p_cb->num_buff > 0) && (buf_len > 0))
        {
            frag_len = buf_len;
            if (buf_len > data_max)
            {
                frag_len = p_data->len = data_max;
                NFC_TRACE_DEBUG2 ("buf_len(%d) > data_max(%d)", buf_len, data_max);
                /* need to fragment the data */
                ps   = (UINT8 *)(p_data + 1) + p_data->offset;
                /* adjust the fragment len in NCI header and set pbf */
                *ps |= NCI_PBF_ST_CONT;
                ps  += 2; /* skip to len byte */
                *ps  = payload_max;
            }

            if (p_cb->num_buff != NFC_CONN_NO_FC)
                p_cb->num_buff--;

            /* check if extra header for this packet */
            if (nci_cb.p_vs_evt_hdlr)
                (*nci_cb.p_vs_evt_hdlr) (NCI_INT_SEND_NCI_MSG_EVT, p_data);

            /* send this fragment to transport */
            ps = (UINT8 *)(p_data + 1) + p_data->offset;
#ifdef DISP_NCI
            delta = p_data->len - frag_len;
            DISP_NCI (ps + delta, (UINT16)(p_data->len - delta), FALSE);
#endif
            USERIAL_Write (USERIAL_NFC_PORT, ps, p_data->len);

            /* check if there's more data to be sent */
            if (buf_len > frag_len)
            {
                /* more data fragments in this buffer: adjust the buffer to reflect the sent fragment */
                buf_len        -= payload_max;
                offset         += payload_max;
                p_data->len     = buf_len;
                p_data->offset  = offset;
                ps      = (UINT8 *)(p_data + 1) + p_data->offset;
                memcpy (ps, hdr, NCI_MSG_HDR_SIZE);
                ps     += 2; /* skip to len byte */
                *ps     = (p_data->len - NCI_MSG_HDR_SIZE);
            }
            else
            {
                /* done with this data buffer, remove from the queue */
                buf_len = 0;
                GKI_dequeue (&p_cb->tx_q);
                GKI_freebuf (p_data);
                /* check if more data buffers in the tx queue */
                p_data = GKI_getfirst (&p_cb->tx_q);
                if (p_data)
                {
                    ps      = (UINT8 *)(p_data + 1) + p_data->offset;
                    memcpy (hdr, ps, NCI_MSG_HDR_SIZE);
                    buf_len = p_data->len;
                    offset  = p_data->offset;
                }
            }
        }
    }
}

/*******************************************************************************
**
** Function         nci_send_cmd
**
** Description      Send NCI command to the transport
**
** Returns          void
**
*******************************************************************************/
void nci_send_cmd (BT_HDR *p_buf)
{
    NFC_TRACE_DEBUG2 ("nci_send_cmd win:%d, qc:%d", nci_cb.nci_cmd_window, nci_cb.nci_cmd_xmit_q.count);
    if (p_buf)
    {
        /* put the new command in the queue */
        GKI_enqueue (&nci_cb.nci_cmd_xmit_q, p_buf);
    }

    /* If controller can accept another command, then send the next command */
    if (nci_cb.nci_cmd_window > 0)
    {
        /* get the first command from the queue */
        p_buf = (BT_HDR *)GKI_dequeue (&nci_cb.nci_cmd_xmit_q);

        if (p_buf)
        {
            /* Keep a copy of the command and send to NCI_TASK (or HCISU_TASK if using shared transport) */
            if (nci_store_n_to_lower (p_buf))
            {
                GKI_freebuf (p_buf);

                /* Indicate command is pending */
                nci_cb.nci_cmd_window--;

                /* start NFC command-timeout timer */
                nci_start_quick_timer (&nci_cb.nci_cmd_cmpl_timer, (UINT16)(NCI_TTYPE_NCI_CMD_CMPL), nci_cb.nci_cmd_cplt_tout);
            }
            else
            {
                /* can not send the buffer to transport now. put the buffer back to the queue */
                GKI_enqueue_head (&nci_cb.nci_cmd_xmit_q, p_buf);
            }
        }
    }
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
            p_cb->p_rcv_msg->offset = NFC_RECEIVE_MSGS_OFFSET;

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
        nci_send_error (NFC_ERR_TRANSPORT);
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
** Description      callback function for timeout
**
** Returns          void
**
*******************************************************************************/
void nci_task_timeout_cback (void *p_tle)
{
    TIMER_LIST_ENT  *p_tlent = (TIMER_LIST_ENT *)p_tle;

    NCI_TRACE_DEBUG0 ("nci_task_timeout_cback ()");

    switch (p_tlent->event)
    {
    case NCI_TTYPE_POWER_CYCLE:
        nci_open_transport ();
        break;

    case NCI_TTYPE_NCI_CMD_CMPL:
        nci_cmd_timeout();
        break;

    default:
        NFC_TRACE_DEBUG1("nci_task_timeout_cback: unhandled timer event (0x%04x)", p_tlent->event);
        break;
    }
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
    int xx;
    tNCI_CONN_CB *p_conn_cb = &nci_cb.conn_cb[0];

    /* free all connection control blocks */
    for (xx=0; xx<NCI_MAX_CONN_CBS; xx++,p_conn_cb++)
    {
        if (p_conn_cb->conn_id != NFC_ILLEGAL_CONN_ID)
        {
            nci_free_conn_cb(p_conn_cb);
        }
    }

    /* initialize command window */
    nci_cb.nci_cmd_window = NCI_MAX_CMD_WINDOW;

    /* Stop command-pending timer */
    nci_stop_quick_timer(&nci_cb.nci_cmd_cmpl_timer);

    /* dequeue and free buffer */
    while ((p_msg = (BT_HDR *)GKI_dequeue (&nci_cb.nci_cmd_xmit_q)) != NULL)
    {
        GKI_freebuf (p_msg);
    }

    /* Free unsent nfc rx buffer */
    if (nci_cb.p_rcv_msg)
        GKI_freebuf (nci_cb.p_rcv_msg);

    /* Free buffer for pending fragmented response/notification */
    if (nci_cb.p_frag_msg)
    {
        GKI_freebuf(nci_cb.p_frag_msg);
        nci_cb.p_frag_msg = NULL;
    }

    /* Free buffers in the tx mbox */
    while ((p_msg = (BT_HDR *)GKI_read_mbox (NCI_TASK_MBOX)) != NULL)
    {
        GKI_freebuf(p_msg);
    }
}

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

/*******************************************************************************
**
** Function         nci_send_message
**
** Description      This function is calledto send an NCI message. 
**
** Returns          void
**
*******************************************************************************/
static void nci_send_message (BT_HDR *p_msg)
{
    tNCI_CONN_CB    *p_conn_cb;

    if ((p_msg->layer_specific == NCI_MSGS_LS_CONTROL) ||
        (p_msg->layer_specific == NCI_MSGS_LS_NCI_VSC))
    {
        nci_send_cmd (p_msg);
    }
    else
    {
        p_conn_cb = nci_find_conn_cb_by_conn_id ((UINT8)(p_msg->layer_specific & 0xFF));
        nci_send_data(p_conn_cb, p_msg);
    }
}

/*******************************************************************************
**
** Function         nci_proc_credits
**
** Description      This function is called to process data credits
**
** Returns          void
**
*******************************************************************************/
static void nci_proc_credits(UINT8 *p, UINT16 plen)
{
    UINT8   num, xx;
    tNCI_CONN_CB * p_cb;

    num = *p++;
    for (xx = 0; xx < num; xx++)
    {
        p_cb = nci_find_conn_cb_by_conn_id(*p++);
        if (p_cb && p_cb->num_buff != NFC_CONN_NO_FC)
        {
            p_cb->num_buff += (*p);
            if (p_cb->num_buff > p_cb->init_credits)
            {
                p_cb->num_buff = p_cb->init_credits;
            }
            /* check if there's any data in tx q to be sent */
            nci_send_data(p_cb, NULL);
        }
        p++;
    }
}

/*******************************************************************************
**
** Function         nci_update_credit
**
** Description      This function is called to update the command window or
**                  credit for NCI data connection.
**
** Returns          void
**
*******************************************************************************/
static void nci_update_credit(void)
{
    BT_HDR *p_msg = nci_cb.p_rcv_msg;
    tNFC_NCI_MSG    *p_nci_msg;
    UINT8   mt, pbf, gid, *p, *pp, oid;
    UINT8   *p_old, old_gid, old_oid, len;
    BOOLEAN update = FALSE;
    UINT8   size, num, conn_id;
    UINT8   handle;
    tNCI_CONN_CB * p_cb = &nci_cb.conn_cb[NFC_RF_CONN_ID];

    p = (UINT8 *)(p_msg + 1) + p_msg->offset;

    pp = p;
    NCI_MSG_PRS_HDR0(pp, mt, pbf, gid);
    NCI_MSG_PRS_HDR1(pp, oid);
    len     = *pp++;
    if ((nci_cb.nci_last_ls == NCI_MSGS_LS_CONTROL) ||
        (nci_cb.nci_last_ls == NCI_MSGS_LS_NCI_VSC) )
    {
        if (mt == NCI_MT_RSP)
        {
            p_old   = nci_cb.last_hdr;
            NCI_MSG_PRS_HDR0(p_old, mt, pbf, old_gid);
            old_oid = ((*p_old) & NCI_OID_MASK);
            /* make sure this is the RSP we are waiting for before updating the command window */
            if ((old_gid == gid) && (old_oid == oid))
            {
                update = TRUE;
                if (gid == NCI_GID_CORE)
                {
                    switch (oid)
                    {
                    case NCI_MSG_CORE_INIT:
                        /* initializint the DH: reset control block */
                        if (*pp++ == NCI_STATUS_OK)
                        {
                            nci_release_conn_cb_buffers (p_cb);
                            p_cb->conn_id       = NFC_RF_CONN_ID;
                            p_cb->buff_size     = 0;
                            p_cb->init_credits  = 0;
                            p_cb->num_buff      = p_cb->init_credits;
                            handle              = (UINT8)(p_cb - nci_cb.conn_cb + 1);
                            nci_cb.conn_id[NFC_RF_CONN_ID] = handle;
                        }
                        break;

                    case NCI_MSG_CORE_CONN_CREATE:
                        if (*pp++ == NCI_STATUS_OK)
                        {
                            /* an NCI logical connection is created: initialize its control block */
                            size    = *pp++;
                            num     = *pp++;
                            conn_id = *pp++;
                            p_cb = nci_alloc_conn_cb (conn_id);
                            if (p_cb)
                            {
                                p_cb->buff_size    = size;
                                p_cb->num_buff     = num;
                                p_cb->init_credits = p_cb->num_buff;
                            }
                        }
                        break;

                    case NCI_MSG_CORE_CONN_CLOSE:
                        /* an NCI logical connection is closed: release its resources */
                        p_cb = nci_find_conn_cb_by_conn_id(nci_cb.last_cmd[0]);
                        nci_free_conn_cb(p_cb);
                        break;

                    default:
                        break;
                    }
                }
            }
        }
    }

    if (mt == NCI_MT_NTF)
    {
        if (gid == NCI_GID_CORE)
        {
            if (oid == NCI_MSG_CORE_CONN_CREDITS)
                nci_proc_credits(pp, len);
        }
        else if (gid == NCI_GID_RF_MANAGE)
        {
            if (oid == NCI_MSG_RF_DEACTIVATE)
            {
                /* RF connection is deactivated: release its resources */
                nci_release_conn_cb_buffers (p_cb);
                p_cb->buff_size     = 0;
                p_cb->init_credits  = 0;
                p_cb->num_buff      = p_cb->init_credits;
            }
        }
    }

    NFC_TRACE_DEBUG3 ("nci_update_credit last_ls:0x%x, update:%d, mt:0x%x", nci_cb.nci_last_ls, update, mt);
    p_nci_msg           = (tNFC_NCI_MSG *)p_msg;
    p_nci_msg->p_cback  = NULL;

#ifdef DISP_NCI
    DISP_NCI ((UINT8 *)(p_msg + 1) + p_msg->offset, p_msg->len, TRUE);
#endif
    if (update)
    {
        if (p_msg->offset >= NFC_RECEIVE_MSGS_OFFSET)
        {
            /* copy the saved NFC_SAVED_CMD_SIZE & VSC cback to p_msg */
            p_nci_msg->p_cback  = nci_cb.p_last_cback;
            memcpy(p_nci_msg->cmd, nci_cb.last_cmd, NFC_SAVED_CMD_SIZE);
        }
        nci_update_window(TRUE);
    }
}

/*******************************************************************************
**
** Function         nci_proc_nfc_msgs
**
** Description      This function is called to process a message from NFC task
**
** Returns          void
**
*******************************************************************************/
static void nci_proc_nfc_msgs (BT_HDR *p_msg)
{
    tNFC_ACTIVATE_MSGS  *p_act = (tNFC_ACTIVATE_MSGS *)p_msg;
    tNCI_CONN_CB        *p_cb  = &nci_cb.conn_cb[NFC_RF_CONN_ID];

    switch (p_msg->layer_specific)
    {
    case NFC_MSGS_RF_ACT_CREDITS:
        /* RF is activated. remember the data credit for this RF interface */
        p_cb->buff_size     = p_act->buff_size;
        p_cb->init_credits  = p_act->num_buff;
        p_cb->num_buff      = p_cb->init_credits;
        break;

    case NFC_MSGS_RF_DEACT_CHECK:
        /* deactivate API is called. Check there are outstanding data credits */
        nci_release_conn_cb_buffers (p_cb);
        nci_cb.nci_flags    |= NCI_FLAGS_DEACTIVATING;
        if (p_cb->init_credits == p_cb->num_buff)
        {
            nci_confirm_deactivate();
        }
        break;

    default:
        break;
    }
}

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
    UINT8   byte;
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

            nci_cb.nci_cmd_cmpl_timer.p_cback = nci_task_timeout_cback;
            nci_start_quick_timer (&nci_cb.nci_cmd_cmpl_timer, NCI_TTYPE_POWER_CYCLE, 
                                   (NCI_POWER_CYCLE_DELAY*QUICK_TIMER_TICKS_PER_SEC)/1000);

            continue;
        }

        /* NCI message ready to be sent to NFCC */
        if (event & NCI_TASK_EVT_MBOX)
        {
            while ((p_msg = (BT_HDR *)GKI_read_mbox (NCI_TASK_MBOX)) != NULL)
            {
                switch (p_msg->event & BT_EVT_MASK)
                {
                case BT_EVT_TO_NFC_NCI:
                    nci_send_message (p_msg);
                    /* do not free buffer. NCI VS code may keep it for processing later */
                    continue;

                case BT_EVT_TO_START_QUICK_TIMER:
                    GKI_start_timer (NCI_QUICK_TIMER_ID, ((GKI_SECS_TO_TICKS (1)/QUICK_TIMER_TICKS_PER_SEC)), TRUE);
                    break;

                case BT_EVT_TO_NFC_NCI_VS:
                    /* sending vendor specific message to NFCC */
                    if (nci_cb.p_vs_evt_hdlr)
                    {
                        (*nci_cb.p_vs_evt_hdlr) (NCI_INT_VS_MSG_EVT, p_msg);
                    }
                    break;

                case BT_EVT_TO_NFC_MSGS:
                    nci_proc_nfc_msgs (p_msg);
                    break;

                default:
                    break;
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
                        nci_check_fragmentation ();
                        if (nci_cb.p_rcv_msg)
                        {
                            nci_update_credit();

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
