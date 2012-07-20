/*****************************************************************************
**
**  Name:           nfa_sys_cback.c
**
**  Description:    Registration/deregistration functions for inter-module callbacks
**
**  Copyright (c) 2010-2011, Broadcom Corp., All Rights Reserved.
**  Widcomm Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#include "nfa_sys.h"
#include "nfa_sys_int.h"




/*******************************************************************************
**
** Function         nfa_sys_cback_reg_enable_complete
**
** Description      Called to register an initialization complete callback function
**
** Returns          void
**
*******************************************************************************/
void nfa_sys_cback_reg_enable_complete (tNFA_SYS_ENABLE_CBACK *p_cback)
{
    nfa_sys_cb.p_enable_cback = p_cback;
    nfa_sys_cb.enable_cplt_flags = 0;
}

/*******************************************************************************
**
** Function         nfa_sys_cback_notify_enable_complete
**
** Description      Called by other NFA subsystems to notify initialization is
**                  complete
**
** Returns          void
**
*******************************************************************************/
void nfa_sys_cback_notify_enable_complete (UINT8 id)
{
    nfa_sys_cb.enable_cplt_flags |= (0x0001 << id);

    NFA_TRACE_DEBUG2 ("nfa_sys_cback_notify_enable_complete () enable_cplt_flags=0x%x, enable_cplt_mask=0x%x",
                       nfa_sys_cb.enable_cplt_flags, nfa_sys_cb.enable_cplt_mask);

    if (  (nfa_sys_cb.enable_cplt_flags == nfa_sys_cb.enable_cplt_mask) /* except SYS, DM and DTA */
        &&(nfa_sys_cb.p_enable_cback)  )
    {
        nfa_sys_cb.p_enable_cback ();
        nfa_sys_cb.p_enable_cback = NULL;
    }
}

/*******************************************************************************
**
** Function         nfa_sys_cback_reg_nfcc_power_mode_proc_complete
**
** Description      Called to register a callback function for complete of processing
**                  NFCC power mode change from NFA sub-systems
**
** Returns          void
**
*******************************************************************************/
void nfa_sys_cback_reg_nfcc_power_mode_proc_complete (tNFA_SYS_PROC_NFCC_PWR_MODE_CMPL *p_cback)
{
    nfa_sys_cb.p_proc_nfcc_pwr_mode_cmpl_cback = p_cback;
    nfa_sys_cb.proc_nfcc_pwr_mode_cplt_flags   = 0;
}

/*******************************************************************************
**
** Function         nfa_sys_cback_notify_nfcc_power_mode_proc_complete
**
** Description      Called by other NFA subsystems to notify processing NFCC power 
**                  mode is complete
**
** Returns          void
**
*******************************************************************************/
void nfa_sys_cback_notify_nfcc_power_mode_proc_complete (UINT8 id)
{
    nfa_sys_cb.proc_nfcc_pwr_mode_cplt_flags |= (0x0001 << id);

    NFA_TRACE_DEBUG2 ("nfa_sys_cback_notify_nfcc_power_mode_proc_complete () flags=0x%x, mask=0x%x",
                       nfa_sys_cb.proc_nfcc_pwr_mode_cplt_flags,
                       nfa_sys_cb.proc_nfcc_pwr_mode_cplt_mask);

    if (  (nfa_sys_cb.proc_nfcc_pwr_mode_cplt_flags == nfa_sys_cb.proc_nfcc_pwr_mode_cplt_mask) /* except SYS */
        &&(nfa_sys_cb.p_proc_nfcc_pwr_mode_cmpl_cback)  )
    {
        nfa_sys_cb.p_proc_nfcc_pwr_mode_cmpl_cback ();
        nfa_sys_cb.p_proc_nfcc_pwr_mode_cmpl_cback = NULL;
    }
}
