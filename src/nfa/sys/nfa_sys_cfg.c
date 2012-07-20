/*****************************************************************************
**
**  Name:           nfa_sys_cfg.c
**
**  Description:    This file contains compile-time configurable constants
**                  for the NFA system manager.
**
**  Copyright (c) 2010, Widcomm Inc., All Rights Reserved.
**  Widcomm Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/

#include "nfc_target.h"
#include "gki.h"
#include "nfa_sys.h"

const tNFA_SYS_CFG nfa_sys_cfg =
{
    NFA_MBOX_EVT_MASK,          /* GKI mailbox event */
    NFA_MBOX_ID,                /* GKI mailbox id */
    NFA_TIMER_ID,               /* GKI timer id */
    APPL_INITIAL_TRACE_LEVEL    /* initial trace level */
};

tNFA_SYS_CFG *p_nfa_sys_cfg = (tNFA_SYS_CFG *) &nfa_sys_cfg;


