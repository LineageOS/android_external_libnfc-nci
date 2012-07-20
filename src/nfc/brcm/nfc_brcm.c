/*****************************************************************************
**
**  Name:          nfc_brcm.c
**
**  Description:   This file contains function of the NFC unit to
**                 receive/process NCI commands.
**
**
**  Copyright (c) 2010-2012, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
******************************************************************************/
#include <string.h>
#include "nfc_target.h"
#include "bt_types.h"
#include "gki.h"

#if NFC_INCLUDED == TRUE
#include "nci_defs.h"
#include "nci_hmsgs.h"
#include "nfc_api.h"
#include "nfc_int.h"

#if (NFC_BRCM_VS_INCLUDED == TRUE)
#include "nfc_brcm_api.h"
#include "nfc_brcm_int.h"

#if (NFC_RW_ONLY == TRUE)
#define NFC_NUM_BRCM_INTERFACE_MAP   1
#else
#define NFC_NUM_BRCM_INTERFACE_MAP   3
#endif

static const tNCI_DISCOVER_MAPS nfc_brcm_interface_mapping[NFC_NUM_BRCM_INTERFACE_MAP] =
{
    /* Protocols that use Frame Interface do not need to be included in the interface mapping */
    {
        NCI_PROTOCOL_ISO_DEP,
        NCI_INTERFACE_MODE_POLL_N_LISTEN,
        NCI_INTERFACE_ISO_DEP
    }
#if (NFC_RW_ONLY == FALSE)
    ,
    /* this can not be set here due to 2079xB0 NFCC issues */
    {
        NCI_PROTOCOL_NFC_DEP,
        NCI_INTERFACE_MODE_POLL_N_LISTEN,
        NCI_INTERFACE_NFC_DEP
    }
    ,
    {
        NCI_PROTOCOL_T2T,
        NCI_INTERFACE_MODE_LISTEN,
        NCI_INTERFACE_VS_T2T_CE
    }
#endif
};
tNFC_BRCM_CB    nfc_brcm_cb;


#if (BT_TRACE_VERBOSE == TRUE)
/*******************************************************************************
**
** Function         nfc_brcm_evt_2_str
**
** Description      convert nfc brcm evt to string
**
*******************************************************************************/
static char *nfc_brcm_evt_2_str(UINT16 event)
{
    event &= BT_SUB_EVT_MASK;
    switch (event)
    {
    case NFC_INT_ENABLE_END_EVT:
        return "NFC_INT_ENABLE_END_EVT";
    case NFC_INT_MBOX_EVT:
        return "NFC_INT_MBOX_EVT";
    case NFC_INT_NCI_VS_RSP_EVT:
        return "NFC_INT_NCI_VS_RSP_EVT";
    case NFC_INT_NCI_VS_NTF_EVT:
        return "NFC_INT_NCI_VS_NTF_EVT";
    case NFC_INT_TIMEOUT_EVT:
        return "NFC_INT_TIMEOUT_EVT";
    case NFC_INT_DISABLE_EVT:
        return "NFC_INT_DISABLE_EVT";
    }
    return "Unknown";
}
#endif


/*******************************************************************************
**
** Function         nfc_brcm_get_bld_info_vs_cback
**
** Description      Handle response for NCI_MSG_GET_BUILD_INFO
**
** Returns          nothing
**
*******************************************************************************/
void nfc_brcm_get_bld_info_vs_cback(tNFC_VS_EVT event, UINT16 data_len, UINT8 *p_data)
{
    tNFC_RESPONSE revt_data;
    tNFC_BRCM_RESPONSE brcm_revt_data;
    UINT8 *p, u8;

    /* Verify that for response is for GET_BUILD_INFO */
    if (event == (NFC_VS_GET_BUILD_INFO_EVT))
    {
        /* Clear build info structure */
        memset(&brcm_revt_data, 0, sizeof(tNFC_BRCM_RESPONSE));

        /* Skip over NCI header */
        p = p_data + NCI_MSG_HDR_SIZE;

        /* Check status */
        STREAM_TO_UINT8(u8, p);
        brcm_revt_data.fw_build_info.status = u8;

        if (u8 == NCI_STATUS_OK)
        {
            /* Parse NCI_MSG_GET_BUILD_INFO */
            STREAM_TO_ARRAY(brcm_revt_data.fw_build_info.date, p, BRCM_FW_BUILD_INFO_DATE_LEN);
            STREAM_TO_ARRAY(brcm_revt_data.fw_build_info.time, p, BRCM_FW_BUILD_INFO_TIME_LEN);
            STREAM_TO_UINT8(brcm_revt_data.fw_build_info.ver_major, p);
            STREAM_TO_UINT8(brcm_revt_data.fw_build_info.ver_minor, p);
            STREAM_TO_UINT8(brcm_revt_data.fw_build_info.build_major, p);
            STREAM_TO_UINT8(brcm_revt_data.fw_build_info.build_minor, p);
            STREAM_TO_UINT32(brcm_revt_data.fw_build_info.hwt, p);

            /* Get Chip ID string */
            STREAM_TO_UINT8(u8, p);
            if (u8 > BRCM_FW_BUILD_INFO_CHIP_ID_MAXLEN)
                u8 = BRCM_FW_BUILD_INFO_CHIP_ID_MAXLEN;
            STREAM_TO_ARRAY(brcm_revt_data.fw_build_info.chip_id, p, u8);
        }

        /* Notify app */
        if (nfc_cb.p_resp_cback)
        {
            revt_data.p_vs_evt_data = &brcm_revt_data;
            (*nfc_cb.p_resp_cback)(NFC_FIRMWARE_BUILD_INFO_REVT, (tNFC_RESPONSE *)&revt_data);
        }
    }

}

/*******************************************************************************
**
** Function         nfc_brcm_evt_hdlr
**
** Description      This function processes events for Broadcom specific features.
**
** Returns          TRUE, if the event is processed
**
*******************************************************************************/
BOOLEAN nfc_brcm_evt_hdlr (tNFC_INT_EVT event, void *p)
{
    BOOLEAN ret = TRUE;
    TIMER_LIST_ENT  *p_tle;
    BT_HDR          *p_msg;
    UINT8           *pp, num, xx, yy;

#if (BT_TRACE_VERBOSE == TRUE)
    NFC_TRACE_EVENT2("nfc_brcm_evt_hdlr :%s(0x%04x)", nfc_brcm_evt_2_str(event), event);
#else
    NFC_TRACE_EVENT1("nfc_brcm_evt_hdlr :0x%04x", event);
#endif
    switch (event & BT_SUB_EVT_MASK)
    {
    case NFC_INT_ENABLE_END_EVT:
        p_msg   = (BT_HDR *)p;
        pp      = (UINT8 *)(p_msg + 1) + p_msg->offset + NCI_MSG_HDR_SIZE + 5; /* 1/status, 4/NFCC features */
        num     = *pp++;
        yy      = 0;
        if (num > NFC_NFCC_MAX_NUM_VS_INTERFACE)
            num = NFC_NFCC_MAX_NUM_VS_INTERFACE;
        for (xx = 0; xx < num; xx++)
        {
           if ((*pp) >= NCI_INTERFACE_FIRST_VS)
           {
               nfc_cb.vs_interface[yy++] = *pp;
           }
           pp++;
        }
        pp += 6; /* 1/num conn, 2/lmrt, 1/ctrl, 2/max size, next is company ID */
        /* If this is not Broadcom NFCC, do not use any VS interface */
        if (*pp != NCI_BRCM_CO_ID)
            memset (nfc_cb.vs_interface, 0, NFC_NFCC_MAX_NUM_VS_INTERFACE);
        /* Get firwmare build info (invalidate old info) */
        NFC_SendVsCommand(NCI_MSG_GET_BUILD_INFO, NULL, NULL);
        break;

    case NFC_INT_NCI_VS_RSP_EVT:
        nci_brcm_proc_prop_rsp((BT_HDR *)p);
        break;

    case NFC_INT_NCI_VS_NTF_EVT:
        nci_brcm_proc_prop_ntf((BT_HDR *)p);
        break;

    case NFC_INT_MBOX_EVT:
        ret = FALSE;
        switch (event & BT_EVT_MASK)
        {
        case BT_EVT_TO_NFC_NCI:
            break;
        case BT_EVT_TO_NFC_NCI_VS:
            nfc_brcm_btvscif_process_event ((BT_HDR *)p);
            ret = TRUE;
            break;
        }
        break;

    case NFC_INT_TIMEOUT_EVT:
        p_tle = (TIMER_LIST_ENT *)p;
        switch (p_tle->event)
        {
        }
        break;

    case NFC_INT_DISABLE_EVT:
        break;

    default:
        ret = FALSE;
    }
    return ret;
}

/*******************************************************************************
**
** Function         NFC_BrcmInit
**
** Description      This function initializes Broadcom specific control blocks for NFC
**                  and registers a callback to be called after the NFCC is reset, 
**                  to perform any platform-specific initialization (e.g. patch download).
**
** Returns          void
**
*******************************************************************************/
void NFC_BrcmInit (tBRCM_DEV_INIT_CBACK *p_dev_init_cback)
{
    NFC_TRACE_API0("NFC_BrcmInit");
    memset (&nfc_brcm_cb, 0x00, sizeof(tNFC_BRCM_CB));

    ce_brcm_init ();
    nci_brcm_init (p_dev_init_cback);

    nfc_cb.p_disc_maps      = nfc_brcm_interface_mapping;
    nfc_cb.num_disc_maps    = NFC_NUM_BRCM_INTERFACE_MAP;
    nfc_cb.p_vs_evt_hdlr    = nfc_brcm_evt_hdlr;

}

/*******************************************************************************
**
** Function         NFC_BrcmGetFirmwareBuildInfo
**
** Description      Get firmware build info
**                  NFA_DM_FIRMWARE_BUILD_INFO_EVT will indicate the result
**
** Returns          NFC_STATUS_OK if operation successfully started
**                  NFC_STATUS_FAILED otherwise
**
*******************************************************************************/
tNFC_STATUS NFC_BrcmGetFirmwareBuildInfo(void)
{
    NFC_TRACE_API0("NFC_BrcmGetFirmwareBuildInfo");

    return(NFC_SendVsCommand(NCI_MSG_GET_BUILD_INFO, NULL, nfc_brcm_get_bld_info_vs_cback));
}


#endif

#endif /* NFC_INCLUDED == TRUE*/

