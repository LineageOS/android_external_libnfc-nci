/*****************************************************************************
**
**  Name:           nfa_brcm_api.h
**
**  Description:    This is the public interface file for NFA, Broadcom's 
**                  NFC application layer for mobile phones.
**
**  Copyright (c) 2010-2011, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#ifndef NFA_BRCM_API_H
#define NFA_BRCM_API_H

#include "nfa_api.h"
#include "nfc_brcm_api.h"
#include "nfa_ee_api.h"

/*****************************************************************************
**  Constants and data types
*****************************************************************************/
typedef tNFC_BRCM_FW_BUILD_INFO tNFA_BRCM_FW_BUILD_INFO;    /* BRCM firmware build info                 */

/* Broadcom-specific NFA_DM events */
enum 
{
    NFA_DM_LPTD_EVT = (NFA_DM_VS_EVT_BASE + 1),             /* False detect in LPTD mode                */
    NFA_DM_FIRMWARE_BUILD_INFO_EVT,                         /* Result of  NFA_BrcmGetFirmwareBuildInfo  */
    NFA_DM_SNOOZE_ENABLED_EVT,                              /* Result of  NFA_EnableSnoozeMode          */
    NFA_DM_SNOOZE_DISABLED_EVT,                             /* Result of  NFA_DisableSnoozeMode         */
    NFA_DM_VS_LAST_EVT
};

/* Data types for BRCM-specific NFA_DM event notification */
/* Dereference using tNFA_DM_CBACK_DATA->p_vs_data        */
typedef union 
{
    tNFA_STATUS             status;                         /* Data for NFA_DM_SNOOZE_ENABLED_EVT       */
                                                            /* Data for NFA_DM_SNOOZE_DISABLED_EVT      */
    tNFA_BRCM_FW_BUILD_INFO fw_build_info;                  /* Data for NFA_DM_FIRMWARE_BUILD_INFO_EVT  */
} tNFA_BRCM_DM_CBACK_DATA;

/* compile-time configuration structure */
typedef struct
{
    UINT8   snooze_mode;                /* Snooze Mode */
    UINT8   idle_threshold_dh;          /* Idle Threshold Host */
    UINT8   idle_threshold_nfcc;        /* Idle Threshold HC   */
    UINT8   nfc_wake_active_mode;       /* NFC_LP_ACTIVE_LOW or NFC_LP_ACTIVE_HIGH */
    UINT8   dh_wake_active_mode;        /* NFC_LP_ACTIVE_LOW or NFC_LP_ACTIVE_HIGH */
    BOOLEAN power_cycle_to_full;        /* Power cycle to full power mode from CEx */
} tNFA_DM_LP_CFG;

extern tNFA_DM_LP_CFG *p_nfa_dm_lp_cfg;

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
**
** Function         NFA_BrcmInit
**
** Description      This function initializes Broadcom specific control blocks for NFA
**                  and registers a callback to be called after the NFCC is reset, 
**                  to perform any platform-specific initialization (e.g. patch download).
**                  
** Returns          none
**
*******************************************************************************/
NFC_API extern void NFA_BrcmInit (tBRCM_DEV_INIT_CBACK *p_dev_init_cback);

/*******************************************************************************
**
** Function         NFA_BrcmGetFirmwareBuildInfo
**
** Description      Get firmware build info from the NFCC.
**                  NFA_DM_FIRMWARE_BUILD_INFO_EVT will indicate the result.
**
** Returns          NFA_STATUS_OK if successfully started
**                  NFA_STATUS_FAILED otherwise
**
*******************************************************************************/
NFC_API extern tNFA_STATUS NFA_BrcmGetFirmwareBuildInfo (void);

/*******************************************************************************
**
** Function         NFA_EnableSnoozeMode
**
** Description      This function is called to enable Snooze Mode as configured
**                  in nfa_dm_brcm_cfg.c.
**                  NFA_DM_SNOOZE_ENABLED_EVT will be sent to indicate status.
**                  
** Returns          NFA_STATUS_OK if successfully initiated
**                  NFA_STATUS_FAILED otherwise
**
*******************************************************************************/
NFC_API extern tNFA_STATUS NFA_EnableSnoozeMode (void);

/*******************************************************************************
**
** Function         NFA_DisableSnoozeMode
**
** Description      This function is called to disable Snooze Mode
**                  NFA_DM_SNOOZE_DISABLED_EVT will be sent to indicate status.
**                  
** Returns          NFA_STATUS_OK if successfully initiated
**                  NFA_STATUS_FAILED otherwise
**
*******************************************************************************/
NFC_API extern tNFA_STATUS NFA_DisableSnoozeMode (void);

/*******************************************************************************
**
** Function         NFA_HciAddStaticPipe
**
** Description      This function is called to add a static pipe for sending 
**                  7816 APDUs.
**
** Returns          NFA_STATUS_OK if successfully added
**                  NFA_STATUS_FAILED otherwise
**
*******************************************************************************/
NFC_API extern tNFA_STATUS NFA_HciAddStaticPipe (tNFA_HANDLE hci_handle, UINT8 pipe);

/*******************************************************************************
**
** Function         NFA_SetMultiTechRsp
**
** Description      Enable or disable NFCC responding more than one technology
**                  during listen discovry. No returning event.
**                  
** Returns          NFA_STATUS_OK if successfully initiated
**                  NFA_STATUS_FAILED otherwise
**
*******************************************************************************/
NFC_API extern tNFA_STATUS NFA_SetMultiTechRsp (BOOLEAN enable);

#ifdef __cplusplus
}
#endif

#endif /* NFA_BRCM_API_H */

