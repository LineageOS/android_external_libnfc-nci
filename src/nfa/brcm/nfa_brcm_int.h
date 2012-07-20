/*****************************************************************************
**
**  Name:           nfa_brcm_int.h
**
**  Description:    This is the private interface file for the NFA Broadcom
**                  vendor specific implementation.
**
**  Copyright (c) 2003-2012, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#ifndef NFA_BRCM_INT_H
#define NFA_BRCM_INT_H

#include "nfc_api.h"
#include "nfa_api.h"
#include "nfa_dm_int.h"

/*****************************************************************************
**  Constants and data types
*****************************************************************************/

#define NFA_HCI_STATIC_PIPE_UICC0           0x70
#define NFA_HCI_STATIC_PIPE_UICC1           0x71
#define NFA_HCI_HOST_ID_UICC0               0x02
#define NFA_HCI_HOST_ID_UICC1               0x03
#define NFA_HCI_PROPRIETARY_GATE0           0xF0
#define NFA_HCI_PROPRIETARY_GATE1           0xF1

#define NFA_HCI_DH_TARGET_HANDLE            0xF2
#define NFA_HCI_UICC0_TARGET_HANDLE         0xF3
#define NFA_HCI_UICC1_TARGET_HANDLE         0xF4

#if (NFC_NFCEE_INCLUDED == TRUE)
#ifdef NFA_EE_NUM_VS_SM_EVT
#undef NFA_EE_NUM_VS_SM_EVT
#endif
#define NFA_EE_NUM_VS_SM_EVT       3
#define NFA_EE_API_SET_PPSE_RSP_EVT         (NFA_EE_FIRST_VS_SM_EVT)
#define NFA_EE_API_ADD_PPSE_ENTRY_EVT       (NFA_EE_FIRST_VS_SM_EVT + 1)
#define NFA_EE_NCI_VS_EVT                   (NFA_EE_FIRST_VS_SM_EVT + 2)
#define NFA_EE_MAX_BRCM_EVT                 NFA_EE_NCI_VS_EVT

#define NFA_EE_AE_PPSE                  NFA_EE_AE_VS    /* for the SELECT PPSE response */
#define NFA_EE_AE_PPSE_PRIORITY         0x0F            /* the PPSE priority            */
#define NFA_EE_ECB_FLAGS_PPSE           NFA_EE_ECB_FLAGS_VS /* PPSE RSP changed                   */
#define NFA_EE_STS_CHANGED_PPSE         NFA_EE_STS_CHANGED_VS        
#define NFA_EE_STS_CHANGED_LP_PPSE      0x04 
#define NFA_EE_STS_CHANGED_CANNED_PPSE  0x08 
#define NFA_EE_STS_PREV_API_PPSE        0x80       
#define NFA_EE_STS_PREV_PPSE            0x40       
#define NFA_EE_FLAG_FIRST_PPSE          0x02    /* to clear PPSE in NV */
#endif

/* data type for NFA_EE_API_SET_PPSE_RSP_EVT */
typedef struct
{
    BT_HDR              hdr;
    UINT8               *p_ppse_rsp;
    UINT16              ppse_rsp_len;
    UINT16              dh_aid_entries_len;
    BOOLEAN             internal;
    BOOLEAN             canned;
} tNFA_EE_API_SET_PPSE_RSP;

#define NFA_EE_API_ADD_PPSE_ENTRY_EXTRA_LEN        10
/* data type for NFA_EE_API_ADD_PPSE_ENTRY_EVT */
typedef struct
{
    BT_HDR              hdr;
    void                *p_cb;
    UINT8               nfcee_id;
    UINT8               cfged_mask;
    UINT8               aid_len;
    UINT8               *p_aid;
    UINT8               priority;
    UINT8               label_len;
    UINT8               extra_len;
    UINT8               *p_extra;
} tNFA_EE_API_ADD_PPSE_ENTRY;


/* data type for NFA_EE_NCI_VS_EVT */
typedef struct
{
    BT_HDR                      hdr;
    tNFC_STATUS                 status;                 /* The event status. */
    tNFC_VS_EVT                 event;
} tNFA_EE_NCI_VS;


#define NFA_DM_BRCM_FLAGS_SNOOZE_ENABLED   0x0001     /* Snooze mode enabled             */
#define NFA_DM_BRCM_FLAGS_SNOOZE_API       0x0002     /* Snooze API is called            */
#define NFA_DM_BRCM_FLAGS_VS_CBACK_REG     0x0004     /* VS Callback has been registered */
#define NFA_DM_BRCM_FLAGS_SNOOZE_ENABLING  0x0008     /* Snooze mode enabling            */
#define NFA_DM_BRCM_FLAGS_RESERVED         0xFF00     /* Reserved for BRCM VS            */

typedef UINT16 tNFA_DM_BRCM_FLAGS;

/* BRCM DM Events */
enum
{
    /* device manager local device API events */
    NFA_DM_BRCM_API_ENABLE_SNOOZE_EVT   = NFA_DM_MAX_EVT,
    NFA_DM_BRCM_API_DISABLE_SNOOZE_EVT,
    NFA_DM_BRCM_API_MULTI_TECH_RSP_EVT,
    NFA_DM_BRCM_API_GET_BUILD_INFO_EVT,
    NFA_DM_BRCM_VS_1_EVT,
    NFA_DM_BRCM_MAX_EVT
};

/* type for action functions */
typedef BOOLEAN (*tNFA_DM_BRCM_ACTION) (BT_HDR *p_msg);

/*****************************************************************************
** NFA BRCM Control block
*****************************************************************************/
typedef struct
{
    /* data members for NFA-DM */
    tNFA_DM_BRCM_FLAGS  dm_flags;               /* flags for BRCM DM                */
    BOOLEAN             dm_enable_multi_resp;   /* TRUE if enable multiple response */
    UINT8               new_nfcc_pwr_mode;      /* configuring NFCC power mode      */

    /* data members for NFA-CE */
    UINT8               ce_flags;               /* flags for various status         */
    tNFA_PROTOCOL_MASK  ce_protocol_mask;       /* Mask of protocols                */
    tNFA_HANDLE         ce_rf_disc_handle;      /* RF Discover handle               */
    UINT8               *p_ce_scratch_buf;      /* Scratch buffer for write requests */
    UINT8               uid_len;                /* size of UID                      */
    UINT8               uid[NFA_MAX_UID_LEN];   /* UID of the Tag                   */

#if (NFC_NFCEE_INCLUDED == TRUE)
    /* data members for NFA-EE */
    UINT8               *p_ee_canned_ppse;      /* PPSE RSP by NFA_EeSetPpseResponse*/
    UINT16              ee_canned_ppse_len;     /* Canned PPSE RSP length           */
    UINT8               ee_mem_type;            /* The memory type VSC Access App   */
    UINT8               ee_num_vsc;             /* the number of vsc sent           */

    /* data members for NFA-HCI */
    TIMER_LIST_ENT      hci_timer;                          /* Timer to avoid indefinitely waiting for response */
    tNCI_HCI_NETWK      *p_hci_netwk_info_buf;              /* Buffer for reading HCI Network information */
    tNCI_HCI_NETWK_DH   *p_hci_netwk_dh_info_buf;           /* Buffer for reading HCI Network DH information */
    BOOLEAN             ee_disc_cmplt;                      /* EE Discovery operation completed */
    UINT8               hci_netwk_config_block;             /* Rsp awaiting for hci network configuration block */
#endif
} tNFA_BRCM_CB;

/* NFA control block for BRCM */
#if NFA_DYNAMIC_MEMORY == FALSE
extern tNFA_BRCM_CB nfa_brcm_cb;
#else
extern tNFA_BRCM_CB *nfa_brcm_cb_ptr;
#define nfa_brcm_cb (*nfa_brcm_cb_ptr)
#endif

/* VS functions from NFA-DM */
extern void nfa_dm_brcm_restore (void);
extern void nfa_dm_brcm_set_fw_fsm (BOOLEAN enable);

#define nfa_ee_brcm_init()
#define nfa_ee_brcm_enable()
#define nfa_ee_brcm_disable()

#if (NFC_NFCEE_INCLUDED == TRUE)
/* VS functions from NFA-HCI */
extern void nfa_hci_brcm_init (void);
extern void nfa_hci_brcm_enable (void);
extern void nfa_hci_brcm_proc_nfcc_power_mode (UINT8 nfcc_power_mode);
extern void nfa_hci_handle_hci_netwk_info (UINT8 *p_data);
#else
#define nfa_hci_brcm_init()
#define nfa_hci_brcm_enable()
#define nfa_hci_brcm_proc_nfcc_power_mode(p)
#define nfa_hci_brcm_ee_discover_req(p)
#define nfa_hci_handle_hci_netwk_info(p)
#endif

/* VS definitions for DTA */
extern UINT8 *p_nfa_dta_brcm_start_up_cfg;
extern UINT8 nfa_dta_brcm_start_up_cfg_len;
extern UINT8 *p_nfa_dm_lptd_cfg;
extern UINT8 *p_nfa_dm_start_up_cfg;
extern UINT8 *p_nfa_dm_start_up_vsc_cfg;
extern UINT8 *p_nfa_lp_ce_power_cfg;
extern UINT8 *p_nfa_power_bitmap_full;
extern UINT8 *p_nfa_power_bitmap_ce_lp;

#define nfa_dm_vs_brcm_init()

#endif /* NFA_BRCM_INT_H */

