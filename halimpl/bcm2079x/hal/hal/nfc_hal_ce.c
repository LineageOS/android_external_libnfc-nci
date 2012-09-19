/******************************************************************************
 *
 *  Copyright (C) 2012 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/******************************************************************************
 *
 *  Vendor-specific handler for CE events
 *
 ******************************************************************************/
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
