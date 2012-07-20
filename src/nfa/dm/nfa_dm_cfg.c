/*****************************************************************************
**
**  Name:           nfa_dm_cfg.c
**
**  Description:    This file contains compile-time configurable constants
**                  for NFA modules
**
**  Copyright (c) 2011, Broadcom Corp., All Rights Reserved.
**
*****************************************************************************/
#include "nfa_api.h"

/* the SetConfig for CE T3T/T4T */
const UINT8 nfa_dm_ce_cfg[] =
{
    13,                         /* total length */
    NFC_PMID_LF_T3T_PMM,        /* Type-3 tag default PMM */
    NCI_PARAM_LEN_LF_T3T_PMM,
    0x20, 
    0x79, 
    0xFF, 
    0xFF, 
    0xFF, 
    0xFF, 
    0xFF, 
    0xFF,
    NFC_PMID_FWI,               /* FWI for ISO-DEP */
    1,
    CE_T4T_ISO_DEP_FWI
};

UINT8 *p_nfa_dm_ce_cfg = (UINT8 *) nfa_dm_ce_cfg;

const tNCI_DISCOVER_MAPS nfa_dm_interface_mapping[NFA_DM_NUM_INTERFACE_MAP] =
{
    /* Protocols that use Frame Interface do not need to be included in the interface mapping */
    {
        NCI_PROTOCOL_ISO_DEP,
        NCI_INTERFACE_MODE_POLL_N_LISTEN,
        NCI_INTERFACE_ISO_DEP
    },
    {
        NCI_PROTOCOL_NFC_DEP,
        NCI_INTERFACE_MODE_POLL_N_LISTEN,
        NCI_INTERFACE_NFC_DEP
    }
};
/* set to NULL to use the default mapping set by stack */
tNCI_DISCOVER_MAPS *p_nfa_dm_interface_mapping = NULL;


const tNFA_DM_CFG nfa_dm_cfg =
{
    NFA_DM_AUTO_DETECT_NDEF,                /* Automatic NDEF detection (when not in exclusive RF mode) */
    NFA_DM_AUTO_READ_NDEF                   /* Automatic NDEF read (when not in exclusive RF mode)      */

};

tNFA_DM_CFG *p_nfa_dm_cfg = (tNFA_DM_CFG *) &nfa_dm_cfg;




