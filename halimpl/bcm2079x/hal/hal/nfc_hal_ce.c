/*****************************************************************************
**
**  Name:           nfc_hal_ce.c
**
**  Description:    Vendor-specific handler for CE events
**
**  Copyright (c) 2012, Broadcom Corp., All Rights Reserved.
**
*****************************************************************************/
#include "gki.h"
#include "nfc_hal_api.h"
#include "nfc_brcm_api.h"
#include "nfc_brcm_int.h"
#include "nfc_hal_int.h"

/*******************************************************************************
**
** Function         nfc_hal_ce_evt_handler
**
** Description      Vendor-specific handler for CE events
**
**                  NOTE: p_msg buffer may not have orginiated from HAL's
**                        memory pool (do not free here)
**
** Returns          void
**
*******************************************************************************/
void nfc_hal_ce_evt_handler(NFC_HDR *p_msg)
{
    /* TODO */
}
