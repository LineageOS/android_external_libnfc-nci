/*****************************************************************************
**
**  Name:          nfc_btvscif_brcm.c
**
**  Description:   APIs for handling BRCM Bluetooth VSCs & Events that are
**                 supported by the NFCC.
**
**
**  Copyright (c) 2011, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
******************************************************************************/
#include <string.h>
#include "gki.h"
#include "nfc_target.h"
#include "bt_types.h"
#include "nfc_api.h"
#include "nfc_int.h"
#include "nci_int.h"
#include "hcidefs.h"
#include "userial.h"
#include "nfc_brcm_api.h"
#include "nfc_brcm_int.h"

#if (defined(NFC_SHARED_TRANSPORT_ENABLED) && (NFC_SHARED_TRANSPORT_ENABLED==TRUE))
#include "btm_api.h"
#endif

#if (NFC_BRCM_VS_INCLUDED == TRUE)

/****************************************************************************
** Declarations
****************************************************************************/

/* Length of message type ID for H4 transport */
#define BTVSC_H4_MSG_TYPE_LEN    1


/*******************************************************************************
**
** Function         nfc_brcm_btvscif_command_complete_evt 
**
** Description      Process event HCI_COMMAND_COMPLETE_EVT 
**
** Returns          void
**
*******************************************************************************/
static void nfc_brcm_btvscif_command_complete_evt (UINT8 *p_evt, UINT16 evt_len)
{
    UINT8           *p;
    UINT16          cmd_opcode, cc_opcode;

    tNFC_BTVSC_CPLT        vcs_cplt_params;
    tNFC_BTVSC_CPLT_CBACK *p_cback;
     
    if ((nfc_cb.p_nci_last_cmd)
     &&((nfc_cb.p_nci_last_cmd->event & BT_EVT_MASK) == BT_EVT_TO_NFC_NCI_VS))
    {
        /* get opcode from last command */
        p = (UINT8 *)(nfc_cb.p_nci_last_cmd + 1) + nfc_cb.p_nci_last_cmd->offset;
        STREAM_TO_UINT16(cmd_opcode, p);

        /* get opcode from received event */
        p_evt++;    /* BT command window */
        STREAM_TO_UINT16 (cc_opcode, p_evt);
        evt_len -= 3;

        /* Verify opcode matches the last command sent */
        if (cc_opcode != cmd_opcode)
        {
            NFC_TRACE_ERROR2("NFC_TASK: expecting command complete for opcode 0x%04X, but got opcode  0x%04X instead", 
                              cmd_opcode, cc_opcode);
        }
        else
        {
            /* get callback function from last command */
            p_cback = ((BT_HDR_BTVSC *)nfc_cb.p_nci_last_cmd)->p_cback;

            /* Call vsc cplt callback, if any */
            if (p_cback)
            {
                vcs_cplt_params.opcode      = cc_opcode;
                vcs_cplt_params.param_len   = evt_len;
                vcs_cplt_params.p_param_buf = p_evt;
                (p_cback)(&vcs_cplt_params);
            }
        }
    }

    /* see if we can send more commands */
    nfc_ncif_update_window (TRUE);
}


/*******************************************************************************
**
** Function         nfc_brcm_btvscif_process_event 
**
** Description      Process Bluetooth HCI events
**                  (currently, only HCI_COMMAND_COMPLETE needs to be handled)
**
** Returns          void
**
*******************************************************************************/
void nfc_brcm_btvscif_process_event(BT_HDR *p_msg)
{
    UINT8   *p = (UINT8 *)(p_msg + 1) + p_msg->offset;
    UINT8   btvsc_evt_code, btvsc_evt_len;
    
    STREAM_TO_UINT8  (btvsc_evt_code, p);
    STREAM_TO_UINT8  (btvsc_evt_len, p);

    NFC_TRACE_EVENT2 ("NFC_TASK: received HCI event: 0x%02x (len=%i)", btvsc_evt_code, btvsc_evt_len);

    /* Process btvsc event */
    if (btvsc_evt_code == HCI_COMMAND_COMPLETE_EVT)
    {
        nfc_brcm_btvscif_command_complete_evt (p, btvsc_evt_len);
    }
    else
    {
        NFC_TRACE_ERROR1 ("NFC_TASK: unhandled BT VS event: 0x%02x", btvsc_evt_code);
    }


    GKI_freebuf(p_msg);
}

/*******************************************************************************
**
** Function         nfc_brcm_btvscif_send
**
** Description      Send BT vendor-specific command to the NFC controller
**
** Returns          tNFC_STATUS
**
*******************************************************************************/
static tNFC_STATUS nfc_brcm_btvscif_send (UINT16 opcode,
                                          UINT8 param_len, UINT8 *p_param_buf,
                                          tNFC_BTVSC_CPLT_CBACK *p_cback)
{
    BT_HDR_BTVSC *p_buf;
    UINT8 *p;

    NFC_TRACE_EVENT2 ("NFC_BtVendorSpecificCommand: Opcode: 0x%04X, ParamLen: %i.",
                      opcode, param_len);

#if (defined(NFC_SHARED_TRANSPORT_ENABLED) && (NFC_SHARED_TRANSPORT_ENABLED==TRUE))
    /* If shared tranport, then send the Bluetooth VSC using the BT stack */
    if (nci_cfg.shared_transport)
    {
        tBTM_STATUS btm_status;
        btm_status = BTM_VendorSpecificCommand(opcode, param_len, p_param_buf, (tBTM_VSC_CMPL_CB *)p_cback);
        return ((btm_status==BTM_CMD_STARTED) ? NFC_STATUS_OK : NFC_STATUS_FAILED);
    }
#endif

    /* Allocate a buffer to hold BT VS command plus the callback function */
    if ((p_buf = (BT_HDR_BTVSC *)GKI_getbuf((UINT16)(sizeof(BT_HDR_BTVSC) +
                            BTVSC_H4_MSG_TYPE_LEN + BRCM_BT_HCI_CMD_HDR_SIZE + param_len))) != NULL)
    {
        p_buf->bt_hdr.len    = NCI_MSG_HDR_SIZE + param_len;
        p_buf->bt_hdr.offset = sizeof(void *);  /* Skip over pointer for VSC callback */
        p_buf->bt_hdr.layer_specific = 0;
        p_buf->bt_hdr.event = BT_EVT_TO_NFC_NCI_VS | NCI_BRCM_HCI_CMD_EVT;

        /* Store command complete callback*/
        p_buf->p_cback = p_cback;

        /* Point to HCI message area after BT_HDR_BTVSC */
        p = (UINT8 *)(p_buf + 1);

        /* Reserve space in case H4 message type byte is needed */
        p++;
        p_buf->bt_hdr.offset++;

        /* Add the HCI command */
        UINT16_TO_STREAM (p, opcode);
        UINT8_TO_STREAM  (p, param_len);
        ARRAY_TO_STREAM  (p, p_param_buf, param_len);

        /* Send the buffer */
        nfc_ncif_send_cmd ((BT_HDR *)p_buf);

        return (NFC_STATUS_OK);

    }
    else
        return (NFC_STATUS_NO_BUFFERS);

}

/*******************************************************************************
**
** Function         nfc_brcm_baudrate_command_cplt_cback
**
** Description      Callback for HCI_BRCM_UPDATE_BAUDRATE_CMD command-cplt-evt
**
*******************************************************************************/
static void nfc_brcm_baudrate_command_cplt_cback (tNFC_BTVSC_CPLT *p_vsc_cplt_info)
{
    UINT8 btvsc_hci_status;
    UINT8 *p = p_vsc_cplt_info->p_param_buf;

    /* Call baud rate change callback if any */
    if (nfc_brcm_cb.p_update_baud_cback)
    {
        STREAM_TO_UINT8(btvsc_hci_status, p);

        if (btvsc_hci_status == HCI_SUCCESS)
        {
            /* update baud rate in local device */
            nci_brcm_set_local_baud_rate (nci_brcm_cb.userial_baud_rate);

            (*nfc_brcm_cb.p_update_baud_cback)(NFC_STATUS_OK);
        }
        else
            (*nfc_brcm_cb.p_update_baud_cback)(NFC_STATUS_FAILED);
    }
}

/*******************************************************************************
**
** Function         NFC_UpdateBaudRate
**
** Description      Reconfigure controller's NCI transport baud rate.
**                  Only used for dedicated transport; for shared BT/NFC 
**                  transport, the baud rate is controlled by BT.
**
**                  Upon success notification, the host must reconfigure
**                  its baud rate to match the controller baud rate.  
**
** Returns          tNFC_STATUS
**
*******************************************************************************/
tNFC_STATUS NFC_UpdateBaudRate (UINT32               baud,
                                tNFC_STATUS_CBACK   *p_update_baud_cback)
{
    tNFC_STATUS result;
    UINT8 btvsc_data[HCI_BRCM_UPDATE_BAUD_RATE_UNENCODED_LENGTH];
    UINT8 *p = btvsc_data;

    /* Verify baud */
    if (USERIAL_GetBaud(baud) == USERIAL_BAUD_AUTO)
    {
        /* baud not supported by USERIAL */
        if (p_update_baud_cback)
        {
            (*p_update_baud_cback)(NFC_STATUS_FAILED);
        }

        return (NFC_STATUS_FAILED);
    }

    nfc_brcm_cb.p_update_baud_cback = p_update_baud_cback;

    /* Baudrate is loaded LittleEndian */
    UINT8_TO_STREAM(p, 0);
    UINT8_TO_STREAM(p, 0);
    UINT32_TO_STREAM(p, baud);

    /* Send the command to the host controller */
    result = nfc_brcm_btvscif_send( HCI_BRCM_UPDATE_BAUDRATE_CMD,
                                    HCI_BRCM_UPDATE_BAUD_RATE_UNENCODED_LENGTH,
                                    (UINT8 *)btvsc_data,
                                    (tNFC_BTVSC_CPLT_CBACK*) nfc_brcm_baudrate_command_cplt_cback);

    if ((result != NFC_STATUS_OK) && (p_update_baud_cback))
    {
        (*p_update_baud_cback)(NFC_STATUS_FAILED);
    }

    nci_brcm_cb.userial_baud_rate = USERIAL_GetBaud(baud);

    return (result);
}

/*******************************************************************************
**
** Function         NFC_SendBtVsCommand
**
** Description      Send BT Vendor Specific Command
**
** Returns          tNFC_STATUS
**
*******************************************************************************/
tNFC_STATUS NFC_SendBtVsCommand (UINT16 opcode, 
                                 UINT8  param_len, 
                                 UINT8 *p_param,
                                 tNFC_BTVSC_CPLT_CBACK *p_cback)
{
    tNFC_STATUS result;

    /* Send the command to the host controller */
    result = nfc_brcm_btvscif_send (opcode, param_len, p_param, p_cback);

    return (result);
}

#endif /* (NFC_BRCM_VS_INCLUDED == TRUE) */

