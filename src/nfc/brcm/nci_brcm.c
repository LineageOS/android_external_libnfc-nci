/*****************************************************************************
**
**  Name:          nci_brcm.c
**
**  Description:   This file contains function of the NFC unit to
**                 receive/process NCI VS commands.
**
**
**  Copyright (c) 2010-2011, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
******************************************************************************/
#include <string.h>
#include "nfc_target.h"
#include "bt_types.h"
#include "gki.h"
#include "trace_api.h"
#include "userial.h"
#include "upio.h"

#if NFC_INCLUDED == TRUE
#include "nci_defs.h"
#include "nci_hmsgs.h"
#include "nfc_api.h"
#include "nfc_int.h"
#include "nci_int.h"

#if (NFC_BRCM_VS_INCLUDED == TRUE)
#include "hcidefs.h"
#include "nfc_brcm_api.h"
#include "nfc_brcm_int.h"

/*****************************************************************************
** Constants and types
*****************************************************************************/

const UINT8 nci_brcm_core_reset_cmd[NCI_MSG_HDR_SIZE + NCI_CORE_PARAM_SIZE_RESET] =
{
    NCI_MTS_CMD|NCI_GID_CORE,
    NCI_MSG_CORE_RESET,
    NCI_CORE_PARAM_SIZE_RESET,
    NCI_RESET_TYPE_RESET_CFG
};

#define NCI_PROP_PARAM_SIZE_XTAL_INDEX      3       /* length of parameters in XTAL_INDEX CMD */

const UINT8 nci_brcm_prop_build_info_cmd[NCI_MSG_HDR_SIZE] =
{
    NCI_MTS_CMD|NCI_GID_PROP,
    NCI_MSG_GET_BUILD_INFO,
    0x00
};
#define NCI_BUILD_INFO_OFFSET_HWID  25  /* HW ID offset in build info RSP */

const UINT8 nci_brcm_prop_get_patch_version_cmd [NCI_MSG_HDR_SIZE] =
{
    NCI_MTS_CMD|NCI_GID_PROP,
    NCI_MSG_GET_PATCH_VERSION,
    0x00
};
#define NCI_PATCH_INFO_OFFSET_NVMTYPE  35  /* NVM Type offset in patch info RSP */

/*****************************************************************************
** Local function prototypes
*****************************************************************************/

#if (BT_TRACE_VERBOSE == TRUE)
static char *nci_brcm_evt_2_str (UINT16 event);
#endif

tNCI_BRCM_CB nci_brcm_cb;

extern void nfc_brcm_prm_nci_command_complete_cback (tNFC_VS_EVT event, UINT16 data_len, UINT8 *p_data);

/*******************************************************************************
**
** Function         nci_brcm_set_xtal_freq_index
**
** Description      Set crystal frequency index
**
** Returns          void
**
*******************************************************************************/
void nci_brcm_set_xtal_freq_index (void)
{
    UINT8 nci_brcm_xtal_index_cmd[NCI_MSG_HDR_SIZE + NCI_PROP_PARAM_SIZE_XTAL_INDEX];
    UINT8 *p, xtal_index;

    NCI_TRACE_DEBUG1 ("nci_brcm_set_xtal_freq_index (): xtal_freq = %d", nci_brcm_cb.dev_init_config.xtal_freq);

    switch (nci_brcm_cb.dev_init_config.xtal_freq)
    {
    case  9600: xtal_index = BRCM_XTAL_INDEX_9600;  break;
    case 13000: xtal_index = BRCM_XTAL_INDEX_13000; break;
    case 16200: xtal_index = BRCM_XTAL_INDEX_16200; break;
    case 19200: xtal_index = BRCM_XTAL_INDEX_19200; break;
    case 24000: xtal_index = BRCM_XTAL_INDEX_24000; break;
    case 26000: xtal_index = BRCM_XTAL_INDEX_26000; break;
    case 38400: xtal_index = BRCM_XTAL_INDEX_38400; break;
    case 52000: xtal_index = BRCM_XTAL_INDEX_52000; break;
    case 37400: xtal_index = BRCM_XTAL_INDEX_37400; break;
    default :   xtal_index = BRCM_XTAL_INDEX_MAX;   break;  /* use out of range if no matched frequency */
    }

    p = nci_brcm_xtal_index_cmd;
    UINT8_TO_STREAM  (p, (NCI_MTS_CMD|NCI_GID_PROP));
    UINT8_TO_STREAM  (p, NCI_MSG_GET_XTAL_INDEX_FROM_DH);
    UINT8_TO_STREAM  (p, NCI_PROP_PARAM_SIZE_XTAL_INDEX);
    UINT8_TO_STREAM  (p, xtal_index);
    UINT16_TO_STREAM (p, nci_brcm_cb.dev_init_config.xtal_freq);

    NFC_BRCM_NCI_STATE (NCI_BRCM_INIT_STATE_W4_XTAL_SET);

    nci_brcm_send_nci_cmd (nci_brcm_xtal_index_cmd, NCI_MSG_HDR_SIZE + NCI_PROP_PARAM_SIZE_XTAL_INDEX, NULL);
}

/*****************************************************************************
**
** Function         nci_brcm_receive_msg
**
** Description
**      Handle incoming BRCM specific data from the serial port.
**
**      If there is data waiting from the serial port, this funciton reads the
**      data and parses it. Once an entire message has been read, it returns
**      TRUE.
**
*****************************************************************************/
static BOOLEAN nci_brcm_receive_msg (tNCI_BRCM_CB *p_cb, UINT8 byte)
{
    UINT16  len;
    BOOLEAN msg_received = FALSE;

    switch (p_cb->rcv_state)
    {
    case NCI_BRCM_RCV_PKT_TYPE_ST:

        /* if this is NCI message */
        if (byte == HCIT_TYPE_NFC)
        {
            /* let NCIT task process NCI message */
            ncit_cb.rcv_state = NCIT_RCV_IDLE_ST;
        }
        /* if this is BT message */
        else if (byte == HCIT_TYPE_EVENT)
        {
            /* continue to use BRCM specific to read BT message */
            p_cb->rcv_state = NCI_BRCM_RCV_BT_IDLE_ST;
        }
        else
        {
            NCI_TRACE_ERROR1 ("Unknown packet type drop this byte 0x%x", byte);
        }
        break;

    case NCI_BRCM_RCV_BT_IDLE_ST:
        /* Initialize rx parameters */
        p_cb->rcv_state = NCI_BRCM_RCV_BT_HDR_ST;
        p_cb->rcv_len   = HCIE_PREAMBLE_SIZE;

        if ((p_cb->p_rcv_msg = (BT_HDR *) GKI_getpoolbuf (NFC_NCI_POOL_ID)) != NULL)
        {
            /* Initialize BT_HDR */
            p_cb->p_rcv_msg->len    = 0;
            p_cb->p_rcv_msg->event  = BT_EVT_TO_NFC_NCI_VS;
            p_cb->p_rcv_msg->offset = NFC_RECEIVE_MSGS_OFFSET;

            *((UINT8 *) (p_cb->p_rcv_msg + 1) + p_cb->p_rcv_msg->offset + p_cb->p_rcv_msg->len++) = byte;
        }
        else
        {
            NCI_TRACE_ERROR0 ("[nfc] Unable to allocate buffer for incoming NCI message.");
        }
        p_cb->rcv_len--;
        break;

    case NCI_BRCM_RCV_BT_HDR_ST:
        if (p_cb->p_rcv_msg)
        {
            *((UINT8 *) (p_cb->p_rcv_msg + 1) + p_cb->p_rcv_msg->offset + p_cb->p_rcv_msg->len++) = byte;
        }
        p_cb->rcv_len--;

        /* Check if we received entire preamble yet */
        if (p_cb->rcv_len == 0)
        {
            /* Received entire preamble. Length is in the last byte(s) of the preamble */
            p_cb->rcv_len = byte;

            /* Verify that buffer is big enough to fit message */
            if ((sizeof (BT_HDR) + HCIE_PREAMBLE_SIZE + byte) > GKI_get_buf_size (p_cb->p_rcv_msg))
            {
                /* Message cannot fit into buffer */
                GKI_freebuf (p_cb->p_rcv_msg);
                p_cb->p_rcv_msg     = NULL;

                NCI_TRACE_ERROR0 ("HCIS: Invalid length for incoming HCI message.");
            }

            /* Message length is valid */
            if (byte)
            {
                /* Read rest of message */
                p_cb->rcv_state = NCI_BRCM_RCV_BT_PAYLOAD_ST;
            }
            else
            {
                /* Message has no additional parameters. (Entire message has been received) */
                msg_received    = TRUE;
                p_cb->rcv_state = NCI_BRCM_RCV_PKT_TYPE_ST;  /* Next, wait for packet type of next message */
            }
        }
        break;

    case NCI_BRCM_RCV_BT_PAYLOAD_ST:
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
            msg_received        = TRUE;
            p_cb->rcv_state     = NCI_BRCM_RCV_PKT_TYPE_ST;      /* Next, wait for packet type of next message */
        }
        break;
    }

    /* If we received entire message */
    if (msg_received)
    {
        /* Display protocol trace message */
#if (BT_TRACE_PROTOCOL == TRUE)
        DispHciEvt (p_cb->p_rcv_msg);
#endif
    }

    return (msg_received);
}

/*******************************************************************************
**
** Function         nci_brcm_set_nfc_wake
**
** Description      Set NFC_WAKE line
**
** Returns          void
**
*******************************************************************************/
void nci_brcm_set_nfc_wake (UINT8 cmd)
{
    NCI_TRACE_DEBUG1 ("nci_brcm_set_nfc_wake () %s",
                      (cmd == NCI_ASSERT_NFC_WAKE ? "ASSERT" : "DEASSERT"));

    /*
    **  nfc_wake_active_mode             cmd              result of voltage on NFC_WAKE
    **
    **  NFC_LP_ACTIVE_LOW (0)    NCI_ASSERT_NFC_WAKE (0)    pull down NFC_WAKE (GND)
    **  NFC_LP_ACTIVE_LOW (0)    NCI_DEASSERT_NFC_WAKE (1)  pull up NFC_WAKE (VCC)
    **  NFC_LP_ACTIVE_HIGH (1)   NCI_ASSERT_NFC_WAKE (0)    pull up NFC_WAKE (VCC)
    **  NFC_LP_ACTIVE_HIGH (1)   NCI_DEASSERT_NFC_WAKE (1)  pull down NFC_WAKE (GND)
    */

    if (cmd == nci_brcm_cb.nfc_wake_active_mode)
        UPIO_Set (UPIO_GENERAL, NCILP_NFC_WAKE_GPIO, UPIO_OFF); /* pull down NFC_WAKE */
    else
        UPIO_Set (UPIO_GENERAL, NCILP_NFC_WAKE_GPIO, UPIO_ON);  /* pull up NFC_WAKE */
}

/*******************************************************************************
**
** Function         nci_brcm_power_mode_execute
**
** Description      If snooze mode is enabled in full power mode,
**                     Assert NFC_WAKE before sending data
**                     Deassert NFC_WAKE when idle timer expires
**
** Returns          TRUE if DH can send data to NFCC
**
*******************************************************************************/
static BOOLEAN nci_brcm_power_mode_execute (UINT8 event)
{
    BOOLEAN send_to_nfcc = FALSE;

    NCI_TRACE_DEBUG1 ("nci_brcm_power_mode_execute () event = %d", event);

    if (nci_brcm_cb.power_mode == NCI_BRCM_POWER_MODE_FULL)
    {
        if (nci_brcm_cb.snooze_mode != NFC_LP_SNOOZE_MODE_NONE)
        {
            /* if any transport activity */
            if (  (event == NCI_BRCM_LP_TX_DATA_EVT)
                ||(event == NCI_BRCM_LP_RX_DATA_EVT)  )
            {
                /* if idle timer is not running */
                if (nci_brcm_cb.lp_timer.in_use == FALSE)
                {
                    nci_brcm_set_nfc_wake (NCI_ASSERT_NFC_WAKE);
                }

                /* start or extend idle timer */
                ncit_start_quick_timer (&nci_brcm_cb.lp_timer, 0x00,
                                       ((UINT32) NCI_LP_IDLE_TIMEOUT) * QUICK_TIMER_TICKS_PER_SEC / 1000);
            }
            else if (event == NCI_BRCM_LP_TIMEOUT_EVT)
            {
                /* let NFCC go to snooze mode */
                nci_brcm_set_nfc_wake (NCI_DEASSERT_NFC_WAKE);
            }
        }

        send_to_nfcc = TRUE;
    }
    else /* VS power mode */
    {
        if (nci_brcm_cb.p_vs_brcm_pwr_hdlr)
        {
            send_to_nfcc = (*nci_brcm_cb.p_vs_brcm_pwr_hdlr) (event);
        }
    }

    return (send_to_nfcc);
}

/*******************************************************************************
**
** Function         nci_brcm_lp_timeout_cback
**
** Description      callback function for low power timeout
**
** Returns          void
**
*******************************************************************************/
static void nci_brcm_lp_timeout_cback (void *p_tle)
{
    NCI_TRACE_DEBUG0 ("nci_brcm_lp_timeout_cback ()");

    nci_brcm_power_mode_execute (NCI_BRCM_LP_TIMEOUT_EVT);
}

/*******************************************************************************
**
** Function         nci_brcm_tx_nci_hcit
**
** Description      Add packet type (HCIT_TYPE_NFC)
**
** Returns          TRUE, if NFCC can receive NCI message
**
*******************************************************************************/
static BOOLEAN nci_brcm_tx_nci_hcit (BT_HDR *p_msg)
{
    UINT8   *p;
    BOOLEAN send_to_nfcc = TRUE;
    UINT8   hcit;

    /* add packet type in front of NCI header */
    if (p_msg->offset > 0)
    {
        p_msg->offset--;
        p_msg->len++;

        p  = (UINT8 *) (p_msg + 1) + p_msg->offset;
        *p = HCIT_TYPE_NFC;
    }
    else
    {
        NCI_TRACE_ERROR0 ("nci_brcm_tx_nci_hcit () : No space for packet type");
        hcit = HCIT_TYPE_NFC;
        USERIAL_Write (USERIAL_NFC_PORT, &hcit, 1);
    }

    return (send_to_nfcc);
}

/*******************************************************************************
**
** Function         nci_brcm_proc_rx_nci_msg
**
** Description      NFCC sends NCI message to DH while initializing NFCC
**
** Returns          TRUE, if NFC task need to receive NCI message
**
*******************************************************************************/
static BOOLEAN nci_brcm_proc_rx_nci_msg (BT_HDR *p_msg)
{
    UINT8 *p;
    UINT8 mt, pbf, gid, op_code;
    UINT8 reset_reason, reset_type;
    tNFC_VS_CBACK *p_cback   = NULL;
    UINT8   *p_old, old_gid, old_oid, old_mt;

    p = (UINT8 *) (p_msg + 1) + p_msg->offset;

    NFC_TRACE_DEBUG3 ("nci_brcm_proc_rx_nci_msg 0x%02x %02x, init sta:%d", *p, *(p + 1), nci_brcm_cb.initializing_state);
    NCI_MSG_PRS_HDR0 (p, mt, pbf, gid);
    NCI_MSG_PRS_HDR1 (p, op_code);

    /* check if waiting for this response */
    if (  (nci_brcm_cb.nci_wait_rsp == NCI_WAIT_RSP_CMD)
        ||(nci_brcm_cb.nci_wait_rsp == NCI_WAIT_RSP_VSC)  )
    {
        if (mt == NCI_MT_RSP)
        {
            p_old   = nci_brcm_cb.last_hdr;
            NCI_MSG_PRS_HDR0(p_old, old_mt, pbf, old_gid);
            old_oid = ((*p_old) & NCI_OID_MASK);
            /* make sure this is the RSP we are waiting for before updating the command window */
            if ((old_gid == gid) && (old_oid == op_code))
            {
                nci_brcm_cb.nci_wait_rsp = NCI_WAIT_RSP_NONE;
                p_cback                  = nci_brcm_cb.p_vsc_cback;
                nci_brcm_cb.p_vsc_cback  = NULL;
                ncit_stop_quick_timer (&nci_brcm_cb.nci_wait_rsp_timer);
            }
        }
    }

    /* if initializing BRCM NFCC */
    if (nci_brcm_cb.initializing_state != NCI_BRCM_INIT_STATE_IDLE)
    {
        if (gid == NCI_GID_CORE)
        {
            if (op_code == NCI_MSG_CORE_RESET)
            {
                if (mt == NCI_MT_RSP)
                {
                    NFC_BRCM_NCI_STATE (NCI_BRCM_INIT_STATE_W4_BUILD_INFO);

                    /* get build information to find out HW */
                    nci_brcm_send_nci_cmd (nci_brcm_prop_build_info_cmd, NCI_MSG_HDR_SIZE, NULL);
                }
                else
                {
                    /* Call reset notification callback */
                    p++;                                /* Skip over param len */
                    STREAM_TO_UINT8 (reset_reason, p);
                    STREAM_TO_UINT8 (reset_type, p);
                    nfc_brcm_prm_spd_reset_ntf (reset_reason, reset_type);
                }
            }
        }
        else if (gid == NCI_GID_PROP) /* this is for download patch */
        {
            if (mt == NCI_MT_NTF)
                op_code |= NCI_NTF_BIT;
            else
                op_code |= NCI_RSP_BIT;

            if (nci_brcm_cb.initializing_state == NCI_BRCM_INIT_STATE_W4_XTAL_SET)
            {
                if (op_code == (NCI_RSP_BIT|NCI_MSG_GET_XTAL_INDEX_FROM_DH))
                {
                    /* wait for crystal setting in NFCC */
                    GKI_delay (100);

                    /* Crytal frequency configured. Proceed with start up sequence: send CORE_RESET_CMD */
                    NFC_BRCM_NCI_STATE (NCI_BRCM_INIT_STATE_W4_RESET);

                    nci_brcm_send_nci_cmd (nci_brcm_core_reset_cmd, NCI_MSG_HDR_SIZE + NCI_CORE_PARAM_SIZE_RESET, NULL);
                }
            }
            else if (  (op_code == NFC_VS_GET_BUILD_INFO_EVT)
                     &&(nci_brcm_cb.initializing_state == NCI_BRCM_INIT_STATE_W4_BUILD_INFO)  )
            {
                p += NCI_BUILD_INFO_OFFSET_HWID;

                STREAM_TO_UINT32 (nci_brcm_cb.brcm_hw_id, p);

                NFC_BRCM_NCI_STATE (NCI_BRCM_INIT_STATE_W4_PATCH_INFO);

                nci_brcm_send_nci_cmd (nci_brcm_prop_get_patch_version_cmd, NCI_MSG_HDR_SIZE, NULL);
            }
            else if (  (op_code == NFC_VS_GET_PATCH_VERSION_EVT)
                     &&(nci_brcm_cb.initializing_state == NCI_BRCM_INIT_STATE_W4_PATCH_INFO)  )
            {
                p += NCI_PATCH_INFO_OFFSET_NVMTYPE;

                NFC_BRCM_NCI_STATE (NCI_BRCM_INIT_STATE_W4_APP_COMPLETE);

                /* let application update baudrate or download patch */
                nci_brcm_cb.p_dev_init_cback (nci_brcm_cb.brcm_hw_id, *p);
            }
            else if (p_cback)
            {
                (*p_cback) ((tNFC_VS_EVT) (op_code),
                                           p_msg->len, (UINT8 *) (p_msg + 1) + p_msg->offset);
            }
            else if (op_code == NFC_VS_SEC_PATCH_AUTH_EVT)
            {
                NFC_TRACE_DEBUG0 ("signature!!");
                nfc_brcm_prm_nci_command_complete_cback ((tNFC_VS_EVT) (op_code),
                                           p_msg->len, (UINT8 *) (p_msg + 1) + p_msg->offset);
            }
        }

        /* do not send message to NFC task while initializing NFCC */
        return (FALSE);
    }

    if (nci_brcm_cb.power_mode == NCI_BRCM_POWER_MODE_FULL)
    {
        if (nci_brcm_cb.snooze_mode != NFC_LP_SNOOZE_MODE_NONE)
        {
            /* extend idle timer */
            nci_brcm_power_mode_execute (NCI_BRCM_LP_RX_DATA_EVT);
        }
    }
    else
    {
        if (nci_brcm_cb.p_vs_brcm_evt_hdlr)
        {
            return ((*nci_brcm_cb.p_vs_brcm_evt_hdlr) (p_msg));
        }
    }

    return (TRUE);
}

/*******************************************************************************
**
** Function         nci_brcm_proc_bt_msg
**
** Description      Received BT message from NFCC
**
**                  Notify command complete if initializing NFCC
**                  Forward BT message to NFC task
**
** Returns          void
**
*******************************************************************************/
static void nci_brcm_proc_bt_msg (void)
{
    tNFC_BTVSC_CPLT vcs_cplt_params;
    UINT8           *p;
    BT_HDR          *p_msg;
    UINT16          opcode, old_opcode;
    tNFC_BTVSC_CPLT_CBACK *p_cback = NULL;

        /* if complete BT message is received successfully */
        if (nci_brcm_cb.p_rcv_msg)
        {
            p_msg   = nci_brcm_cb.p_rcv_msg;
            NFC_TRACE_DEBUG1 ("nci_brcm_proc_bt_msg GOT an BT msgs init_sta:%d", nci_brcm_cb.initializing_state);
            NFC_TRACE_DEBUG2 ("event: 0x%x, wait_rsp:0x%x", p_msg->event, nci_brcm_cb.nci_wait_rsp);
            /* increase the cmd window here */
            if (p_msg->event == BT_EVT_TO_NFC_NCI_VS && nci_brcm_cb.nci_wait_rsp == NCI_WAIT_RSP_PROP)
            {
                p = (UINT8 *) (p_msg + 1) + p_msg->offset;
                if (*p == HCI_COMMAND_COMPLETE_EVT)
                {
                    p  += 3; /* code, len, cmd window */
                    STREAM_TO_UINT16 (opcode, p);
                    p   = nci_brcm_cb.last_hdr;
                    STREAM_TO_UINT16 (old_opcode, p);
                    if (opcode == old_opcode)
                    {
                        nci_brcm_cb.nci_wait_rsp = NCI_WAIT_RSP_NONE;
                        p_cback                  = nci_brcm_cb.p_vsc_cback;
                        nci_brcm_cb.p_vsc_cback  = NULL;
                        ncit_stop_quick_timer (&nci_brcm_cb.nci_wait_rsp_timer);
                    }
                }
            }

            /* if initializing BRCM NFCC */
            if (nci_brcm_cb.initializing_state == NCI_BRCM_INIT_STATE_W4_AUTOBAUD)
            {
                /* stop timer for auto-baud */
                ncit_stop_quick_timer (&nci_brcm_cb.auto_baud_timer);

                /* NFCC auto-baud detection is done. */

                if (nci_brcm_cb.dev_init_config.flags & BRCM_DEV_INIT_FLAGS_SET_XTAL_FREQ)
                {
                    /* configure crystal frequency */
                    nci_brcm_set_xtal_freq_index ();
                }
                else
                {
                    /* Proceed with start up sequence: send CORE_RESET_CMD */
                    NFC_BRCM_NCI_STATE (NCI_BRCM_INIT_STATE_W4_RESET);

                    nci_brcm_send_nci_cmd (nci_brcm_core_reset_cmd, NCI_MSG_HDR_SIZE + NCI_CORE_PARAM_SIZE_RESET, NULL);
                }

                /* do not send message to NFC task while initializing NFCC */
                GKI_freebuf (nci_brcm_cb.p_rcv_msg);
            }
            else if (nci_brcm_cb.initializing_state == NCI_BRCM_INIT_STATE_W4_APP_COMPLETE)
            {
                /* this is command complete event for baud rate update or download patch */
                p = (UINT8 *) (p_msg + 1) + p_msg->offset;

                p += 1;    /* skip opcode */
                STREAM_TO_UINT8  (vcs_cplt_params.param_len, p);

                p += 1;    /* skip num command packets */
                STREAM_TO_UINT16 (vcs_cplt_params.opcode, p);

                vcs_cplt_params.param_len -= 3;
                vcs_cplt_params.p_param_buf = p;

                if (p_cback)
                {
                    nci_brcm_cb.p_vsc_cback = NULL;
                    (*p_cback) (&vcs_cplt_params);
                }

                /* do not send message to NFC task while initializing NFCC */
                GKI_freebuf (p_msg);
            }
            else
            {
                /* send BT message to NFC task */
                GKI_send_msg (NFC_TASK, NFC_MBOX_ID, p_msg);
            }
            nci_brcm_cb.p_rcv_msg = NULL;
        }

}

/*******************************************************************************
**
** Function         nci_brcm_rcv_msg_cback
**
** Description      The callback function to send the message received from NFCC
**                  to NFC task for processing
**
** Returns          nothing
**
*******************************************************************************/
void nci_brcm_rcv_msg_cback (BT_HDR *p_msg)
{
    /* if this is not vendor specific transport message */
    if (nci_brcm_proc_rx_nci_msg (p_msg))
    {
        /* Send message to NFC_TASK for processing by the stack */
        GKI_send_msg (NFC_TASK, NFC_MBOX_ID, p_msg);
    }
    else
    {
        /* Message has been processed by the vs_evt_hdlr */
        GKI_freebuf(p_msg);
    }
}

/*******************************************************************************
**
** Function         nci_brcm_vs_init_done
**
** Description      The function sets ncit_cb.p_vs_rcv_cback back to NULL
**                  can calls ncit_task_vs_init_done() to let NCIT know that the init
**                  process is complete
**
** Returns          nothing
**
*******************************************************************************/
void nci_brcm_vs_init_done (void)
{
    /* use nci_brcm_rcv_msg_cback() to process NCI VS RSP/NTF */
    ncit_task_vs_init_done ();
}

/*******************************************************************************
**
** Function         nci_brcm_evt_hdlr
**
** Description      This function processes events for Broadcom specific features
**                  in NCI transport layer.
**
** Returns          TRUE, if need to process after returning this function
**
*******************************************************************************/
static BOOLEAN nci_brcm_evt_hdlr (tNCIT_INT_EVT event, void *p_data)
{
    BOOLEAN continue_to_proc = TRUE;
    BT_HDR  *p_msg;

#if (BT_TRACE_VERBOSE == TRUE)
    NCI_TRACE_EVENT2 ("nci_brcm_evt_hdlr ():%s (0x%04x)", nci_brcm_evt_2_str (event), event);
#else
    NCI_TRACE_EVENT1 ("nci_brcm_evt_hdlr ():0x%04x", event);
#endif

    switch (event)
    {
    case NCIT_INT_CHECK_NFCC_STATE_EVT:   /* Checks if the NFCC can accept messages     */
        continue_to_proc = nci_brcm_power_mode_execute (NCI_BRCM_LP_TX_DATA_EVT);
        break;

    case NCIT_INT_SEND_NCI_MSG_EVT:  /* NFC task sends NCI command to NFCC */
        continue_to_proc = nci_brcm_tx_nci_hcit ((BT_HDR*) p_data);
        break;

    case NCIT_INT_DATA_RDY_EVT:      /* transport get data from NFCC */
        if (nci_brcm_receive_msg (&nci_brcm_cb, *(UINT8 *)p_data))
        {
            /* received BT message */
            nci_brcm_proc_bt_msg ();
        }
        break;

    case NCIT_INT_VS_MSG_EVT:    /* BRCM NCI APIs are called and events are posted to process */

        p_msg = (BT_HDR*) p_data;

        if (nci_brcm_cb.p_vs_brcm_evt_hdlr)
        {
            (*nci_brcm_cb.p_vs_brcm_evt_hdlr) (p_msg);
        }
        GKI_freebuf (p_msg);
        break;

    case NCIT_INT_VS_INIT_EVT:   /* NFCC is turned on */

        /* process received message before posting to NFC task */
        ncit_cb.p_vs_rcv_cback = nci_brcm_rcv_msg_cback;

        /* if application registered callback for device initialization */
        if (nci_brcm_cb.p_dev_init_cback)
        {
            if (nci_brcm_cb.dev_init_config.flags & BRCM_DEV_INIT_FLAGS_AUTO_BAUD)
            {
                nci_brcm_start_auto_baud_detection ();
            }
            else if (nci_brcm_cb.dev_init_config.flags & BRCM_DEV_INIT_FLAGS_SET_XTAL_FREQ)
            {
                nci_brcm_set_xtal_freq_index ();
            }
            else
            {
                NFC_BRCM_NCI_STATE (NCI_BRCM_INIT_STATE_W4_RESET);

                nci_brcm_send_nci_cmd (nci_brcm_core_reset_cmd, NCI_MSG_HDR_SIZE + NCI_CORE_PARAM_SIZE_RESET, NULL);
            }
        }
        else
        {
            /* notify complete of BRCM initialization */
            nci_brcm_vs_init_done ();
        }
        break;

    case NCIT_INT_TERMINATE_EVT: /* NFCC is going to shut down */

        /* reset low power mode variables */
        if (  (nci_brcm_cb.power_mode == NCI_BRCM_POWER_MODE_FULL)
            &&(nci_brcm_cb.snooze_mode != NFC_LP_SNOOZE_MODE_NONE)  )
        {
            nci_brcm_set_nfc_wake (NCI_ASSERT_NFC_WAKE);
        }

        nci_brcm_cb.power_mode  = NCI_BRCM_POWER_MODE_FULL;
        nci_brcm_cb.snooze_mode = NFC_LP_SNOOZE_MODE_NONE;

        /* Stop command-pending timer */
        ncit_stop_quick_timer (&nci_brcm_cb.nci_wait_rsp_timer);
        ncit_stop_quick_timer (&nci_brcm_cb.lp_timer);

        nci_brcm_reset_baud_on_shutdown ();
        break;

    default:
        break;
    }

    return (continue_to_proc);
}

/*******************************************************************************
**
** Function         nci_brcm_cmd_timeout_cback
**
** Description      callback function for timeout
**
** Returns          void
**
*******************************************************************************/
void nci_brcm_cmd_timeout_cback (void *p_tle)
{
    TIMER_LIST_ENT  *p_tlent = (TIMER_LIST_ENT *)p_tle;

    NCI_TRACE_DEBUG0 ("nci_brcm_cmd_timeout_cback ()");

    if (p_tlent->event == NCIT_TTYPE_NCI_WAIT_RSP)
    {
        /* report an error */
        ncit_send_error (NFC_ERR_CMD_TIMEOUT);
    }
}


/*******************************************************************************
**
** Function         nci_brcm_init
**
** Description      This function initializes Broadcom specific control blocks for
**                  NCI transport
**
** Returns          void
**
*******************************************************************************/
void nci_brcm_init (tBRCM_DEV_INIT_CONFIG *p_dev_init_config,
                    tBRCM_DEV_INIT_CBACK  *p_dev_init_cback)
{
    NCI_TRACE_DEBUG0 ("nci_brcm_init ()");

    ncit_cb.p_vs_evt_hdlr  = nci_brcm_evt_hdlr;
    ncit_cb.init_rcv_state = NCIT_RCV_PROP_MSG_ST; /* to process packet type */

    /* Clear nci brcm control block */
    memset (&nci_brcm_cb, 0, sizeof (tNCI_BRCM_CB));

    nci_brcm_cb.lp_timer.p_cback  = nci_brcm_lp_timeout_cback;

    nci_brcm_cb.p_dev_init_cback  = p_dev_init_cback;

    if (p_dev_init_config)
    {
        nci_brcm_cb.dev_init_config.flags     = p_dev_init_config->flags;
        nci_brcm_cb.dev_init_config.xtal_freq = p_dev_init_config->xtal_freq;
    }
    nci_brcm_cb.nci_wait_rsp_tout             = (BRCM_PRM_SPD_TOUT*QUICK_TIMER_TICKS_PER_SEC)/1000;
    nci_brcm_cb.nci_wait_rsp_timer.p_cback    = nci_brcm_cmd_timeout_cback;

    nci_vs_brcm_init ();
}

/*******************************************************************************
**
** Function         nci_brcm_send_nci_cmd
**
** Description      Send NCI command to NFCC while initializing BRCM NFCC
**
** Returns          void
**
*******************************************************************************/
void nci_brcm_send_nci_cmd (const UINT8 *p_data, UINT16 len, tNFC_VS_CBACK *p_cback)
{
    BT_HDR *p_buf;
    UINT8  *ps;

    NFC_TRACE_DEBUG1 ("nci_brcm_send_nci_cmd (): nci_wait_rsp = 0x%x", nci_brcm_cb.nci_wait_rsp);

    if (nci_brcm_cb.nci_wait_rsp != NCI_WAIT_RSP_NONE)
    {
        NFC_TRACE_ERROR0 ("nci_brcm_send_nci_cmd(): no command window");
        return;
    }

    if ((p_buf = (BT_HDR *) GKI_getbuf ((UINT16) (BT_HDR_SIZE + NCI_VSC_MSG_HDR_SIZE + len))) != NULL)
    {
        nci_brcm_cb.nci_wait_rsp = NCI_WAIT_RSP_VSC;

        p_buf->offset           = NCI_VSC_MSG_HDR_SIZE;
        p_buf->event            = BT_EVT_TO_NFC_NCI;
        p_buf->len    = len;

        memcpy ((UINT8*) (p_buf + 1) + p_buf->offset, p_data, len);

        /* Keep a copy of the command and send to NCI transport */

        /* save the message header to double check the response */
        ps   = (UINT8 *)(p_buf + 1) + p_buf->offset;
        memcpy(nci_brcm_cb.last_hdr, ps, NCI_SAVED_HDR_SIZE);
        memcpy(nci_brcm_cb.last_cmd, ps + NCI_MSG_HDR_SIZE, NFC_SAVED_CMD_SIZE);

        /* save the callback for NCI VSCs */
        nci_brcm_cb.p_vsc_cback = p_cback;

        ncit_send_cmd (p_buf);

        /* start NFC command-timeout timer */
        ncit_start_quick_timer (&nci_brcm_cb.nci_wait_rsp_timer, (UINT16)(NCIT_TTYPE_NCI_WAIT_RSP), nci_brcm_cb.nci_wait_rsp_tout);
    }
}

/*******************************************************************************
**
** Function         nci_brcm_send_bt_cmd
**
** Description      Send BT message to NFCC while initializing BRCM NFCC
**
** Returns          void
**
*******************************************************************************/
void nci_brcm_send_bt_cmd (const UINT8 *p_data, UINT16 len, tNFC_BTVSC_CPLT_CBACK *p_cback)
{
    BT_HDR *p_buf;
    UINT8  *p;

    NFC_TRACE_DEBUG1 ("nci_brcm_send_bt_cmd (): nci_wait_rsp = 0x%x", nci_brcm_cb.nci_wait_rsp);

    if (nci_brcm_cb.nci_wait_rsp != NCI_WAIT_RSP_NONE)
    {
        NFC_TRACE_ERROR0 ("nci_brcm_send_bt_cmd(): no command window");
        return;
    }

    if ((p_buf = (BT_HDR *) GKI_getbuf ((UINT16) (BT_HDR_SIZE + NCI_VSC_MSG_HDR_SIZE + len))) != NULL)
    {
        nci_brcm_cb.nci_wait_rsp = NCI_WAIT_RSP_PROP;

        p_buf->offset = NCI_VSC_MSG_HDR_SIZE;
        p_buf->len    = len;

        memcpy ((UINT8*) (p_buf + 1) + p_buf->offset, p_data, len);

#if (BT_TRACE_PROTOCOL == TRUE)
        DispHciCmd (p_buf);
#endif

        /* save the message header to double check the response */
        p = (UINT8 *)(p_buf + 1) + p_buf->offset;
        memcpy(nci_brcm_cb.last_hdr, p, NCI_SAVED_HDR_SIZE);

        /* save the callback for NCI VSCs)  */
        nci_brcm_cb.p_vsc_cback = p_cback;

        /* add packet type for BT message */
        p_buf->offset--;
        p_buf->len++;

        p  = (UINT8 *) (p_buf + 1) + p_buf->offset;
        *p = HCIT_TYPE_COMMAND;

        USERIAL_Write (USERIAL_NFC_PORT, p, p_buf->len);

        GKI_freebuf (p_buf);

        /* start NFC command-timeout timer */
        ncit_start_quick_timer (&nci_brcm_cb.nci_wait_rsp_timer, (UINT16)(NCIT_TTYPE_NCI_WAIT_RSP), nci_brcm_cb.nci_wait_rsp_tout);
    }
}


/*******************************************************************************
**
** Function         NCI_BrcmDevInitDone
**
** Description      Notify NCI transport that device is initialized
**
** Returns          void
**
*******************************************************************************/
void NCI_BrcmDevInitDone (void)
{
    NCI_TRACE_DEBUG0 ("NCI_BrcmDevInitDone ()");

    if (nci_brcm_cb.initializing_state == NCI_BRCM_INIT_STATE_W4_APP_COMPLETE)
    {
        NFC_BRCM_NCI_STATE (NCI_BRCM_INIT_STATE_IDLE);
        nci_brcm_cb.nci_wait_rsp_timer.p_cback   = nci_brcm_cmd_timeout_cback;

        nci_brcm_vs_init_done ();
    }
}

/*******************************************************************************
**
** Function         nci_brcm_set_snooze_mode_cback
**
** Description      This is baud rate update complete callback.
**
** Returns          void
**
*******************************************************************************/
static void nci_brcm_set_snooze_mode_cback (tNFC_BTVSC_CPLT *pData)
{
    tNFC_STATUS status = pData->p_param_buf[0];
    tNFC_STATUS_CBACK *p_cback;

    /* if it is completed */
    if (status == HCI_SUCCESS)
    {

        nci_brcm_set_nfc_wake (NCI_ASSERT_NFC_WAKE);

        if ( nci_brcm_cb.snooze_mode != NFC_LP_SNOOZE_MODE_NONE)
        {
            /* start idle timer */
            ncit_start_quick_timer (&nci_brcm_cb.lp_timer, 0x00,
                                   ((UINT32) NCI_LP_IDLE_TIMEOUT) * QUICK_TIMER_TICKS_PER_SEC / 1000);
        }
        else
        {
            ncit_stop_quick_timer (&nci_brcm_cb.lp_timer);
        }
    }

    if (nci_brcm_cb.p_prop_cback)
    {
        p_cback = nci_brcm_cb.p_prop_cback;
        nci_brcm_cb.p_prop_cback = NULL;
        (p_cback) (status);
    }
}

/*******************************************************************************
**
** Function         NCI_BrcmSetSnoozeMode
**
** Description      Set snooze mode
**                  snooze_mode
**                      NFC_LP_SNOOZE_MODE_NONE - Snooze mode disabled
**                      NFC_LP_SNOOZE_MODE_UART - Snooze mode for UART
**                      NFC_LP_SNOOZE_MODE_SPI_I2C - Snooze mode for SPI/I2C
**
**                  idle_threshold_dh/idle_threshold_nfcc
**                      Idle Threshold Host in 100ms unit
**
**                  nfc_wake_active_mode/dh_wake_active_mode
**                      NFC_LP_ACTIVE_LOW - high to low voltage is asserting
**                      NFC_LP_ACTIVE_HIGH - low to high voltage is asserting
**
**                  p_snooze_cback
**                      Notify status of operation
**
** Returns          tNFC_STATUS
**
*******************************************************************************/
tNFC_STATUS NCI_BrcmSetSnoozeMode (UINT8 snooze_mode,
                                   UINT8 idle_threshold_dh,
                                   UINT8 idle_threshold_nfcc,
                                   UINT8 nfc_wake_active_mode,
                                   UINT8 dh_wake_active_mode,
                                   tNFC_STATUS_CBACK *p_snooze_cback)
{
    UINT8 cmd[BRCM_BT_HCI_CMD_HDR_SIZE + HCI_BRCM_WRITE_SLEEP_MODE_LENGTH];
    UINT8 *p;

    NCI_TRACE_API1 ("NCI_BrcmSetSnoozeMode (): snooze_mode = %d", snooze_mode);

    nci_brcm_cb.snooze_mode          = snooze_mode;
    nci_brcm_cb.nfc_wake_active_mode = nfc_wake_active_mode;
    nci_brcm_cb.p_prop_cback         = p_snooze_cback;

    p = cmd;

    /* Add the HCI command */
    UINT16_TO_STREAM (p, HCI_BRCM_WRITE_SLEEP_MODE);
    UINT8_TO_STREAM  (p, HCI_BRCM_WRITE_SLEEP_MODE_LENGTH);

    memset (p, 0x00, HCI_BRCM_WRITE_SLEEP_MODE_LENGTH);

    UINT8_TO_STREAM  (p, snooze_mode);          /* Sleep Mode               */

    UINT8_TO_STREAM  (p, idle_threshold_dh);    /* Idle Threshold Host      */
    UINT8_TO_STREAM  (p, idle_threshold_nfcc);  /* Idle Threshold HC        */
    UINT8_TO_STREAM  (p, nfc_wake_active_mode); /* BT Wake Active Mode      */
    UINT8_TO_STREAM  (p, dh_wake_active_mode);  /* Host Wake Active Mode    */

    nci_brcm_send_bt_cmd (cmd,
                          BRCM_BT_HCI_CMD_HDR_SIZE + HCI_BRCM_WRITE_SLEEP_MODE_LENGTH,
                          nci_brcm_set_snooze_mode_cback);
    return (NFC_STATUS_OK);
}

#if (BT_TRACE_VERBOSE == TRUE)
/*******************************************************************************
**
** Function         nci_brcm_evt_2_str
**
** Description      convert nci brcm evt to string
**
*******************************************************************************/
static char *nci_brcm_evt_2_str (UINT16 event)
{
    switch (event)
    {
    case NCIT_INT_CHECK_NFCC_STATE_EVT:
        return "NCIT_INT_CHECK_NFCC_STATE_EVT";
    case NCIT_INT_SEND_NCI_MSG_EVT:
        return "NCIT_INT_SEND_NCI_MSG_EVT";
    case NCIT_INT_DATA_RDY_EVT:
        return "NCIT_INT_DATA_RDY_EVT";
    case NCIT_INT_VS_MSG_EVT:
        return "NCIT_INT_VS_MSG_EVT";
    case NCIT_INT_VS_INIT_EVT:
        return "NCIT_INT_VS_INIT_EVT";
    case NCIT_INT_TERMINATE_EVT:
        return "NCIT_INT_TERMINATE_EVT";
    }
    return "Unknown";
}
#endif

#endif

#endif /* NFC_INCLUDED == TRUE*/
