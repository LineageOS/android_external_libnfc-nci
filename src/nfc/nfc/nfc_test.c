/*****************************************************************************
**
**  Name:          nfc_main.c
**
**  Description:   This file contains functions that interface with the NFC NCI
**                 transport. On the receive side, it routes events to
**                 the appropriate handler (callback). On the
**                 transmit side, it manages the command transmission.
**
**
**  Copyright (c) 2010-2011, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
******************************************************************************/
#include <string.h>
#include "gki.h"
#include "nfc_target.h"
#include "bt_types.h"

#if (NFC_INCLUDED == TRUE)
#include "nfc_int.h"
#include "nci_hmsgs.h"

/****************************************************************************
** Declarations
****************************************************************************/

/*******************************************************************************
**
** Function         NFC_TestLoopback
**
** Description      This function is called to send the given data packet
**                  to NFCC for loopback test.
**                  When loopback data is received from NFCC, tNFC_TEST_CBACK . 
**                  reports a NFC_LOOPBACK_TEVT.
**
** Parameters       p_data - the data packet
**
** Returns          tNFC_STATUS
**
*******************************************************************************/
tNFC_STATUS NFC_TestLoopback(BT_HDR *p_data)
{
    tNFC_STATUS     status  = NFC_STATUS_FAILED;
    tNFC_CONN_CB    *p_cb   = nfc_find_conn_cb_by_handle(NCI_TEST_ID);

    if (p_data && p_cb && (p_data->offset >= (NCI_MSG_OFFSET_SIZE + NCI_DATA_HDR_SIZE)))
    {
        status = nfc_ncif_send_data (p_cb, p_data);
    }

    if(status != NFC_STATUS_OK)
        GKI_freebuf (p_data);

    return status;
}




#endif /* NFC_INCLUDED == TRUE */
