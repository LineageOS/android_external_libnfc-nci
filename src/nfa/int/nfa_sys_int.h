/*****************************************************************************
**
**  Name:           nfa_sys_int.h
**
**  Description:    This is the private interface file for the NFA system
**                  manager.
**
**  Copyright (c) 2010, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#ifndef NFA_SYS_INT_H
#define NFA_SYS_INT_H

#include "nfa_sys_ptim.h"

/*****************************************************************************
**  Constants and data types
*****************************************************************************/


/* nfa_sys flags */
#define NFA_SYS_FL_INITIALIZED  0x00000001          /* nfa_sys initialized */

/*****************************************************************************
**  state table
*****************************************************************************/

/* system manager control block */
typedef struct
{
    UINT32                  flags;                  /* nfa_sys flags (must be first element of structure) */
    tNFA_SYS_REG            *reg[NFA_ID_MAX];       /* registration structures */
    BOOLEAN                 is_reg[NFA_ID_MAX];     /* registration structures */
    tPTIM_CB                ptim_cb;                /* protocol timer list */
    tNFA_SYS_ENABLE_CBACK   *p_enable_cback;
    UINT16                  enable_cplt_flags;
    UINT16                  enable_cplt_mask;

    tNFA_SYS_PROC_NFCC_PWR_MODE_CMPL  *p_proc_nfcc_pwr_mode_cmpl_cback;
    UINT16                  proc_nfcc_pwr_mode_cplt_flags;
    UINT16                  proc_nfcc_pwr_mode_cplt_mask;

    BOOLEAN                 graceful_disable;       /* TRUE if NFA_Disable () is called with TRUE */
    BOOLEAN                 timers_disabled;        /* TRUE if sys timers disabled */
    UINT8                   trace_level;            /* Trace level */
} tNFA_SYS_CB;



/*****************************************************************************
**  Global variables
*****************************************************************************/

/* system manager control block */
#if NFA_DYNAMIC_MEMORY == FALSE
extern tNFA_SYS_CB nfa_sys_cb;
#else
extern tNFA_SYS_CB *nfa_sys_cb_ptr;
#define nfa_sys_cb (*nfa_sys_cb_ptr)
#endif


/* system manager configuration structure */
extern tNFA_SYS_CFG *p_nfa_sys_cfg;

BOOLEAN nfa_sys_sm_execute (BT_HDR *p_msg);

#endif /* NFA_SYS_INT_H */
