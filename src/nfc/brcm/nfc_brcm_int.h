/****************************************************************************
** 
**  File:        nfc_brcm_int.h
**
**  Description:   this file contains the BRCM specific
**                 internal definitions and functions.                   
**
**  Copyright (c) 2011, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
****************************************************************************/

#ifndef NFC_BRCM_INT_H
#define NFC_BRCM_INT_H


#include "nfc_target.h"
#include "gki.h"
#include "ce_api.h"

#include "nfc_brcm_api.h"
#include "nci_int.h"
#include "userial.h" 


#if (defined(NFC_SHARED_TRANSPORT_ENABLED) && (NFC_SHARED_TRANSPORT_ENABLED==TRUE))
#include "btm_api.h"
#endif

#define NFC_TTYPE_CE_T1T_CONT_RSP           (NFC_TTYPE_VS_BASE + 1)
#define NFC_TTYPE_CE_T2T_CONT_RSP           (NFC_TTYPE_VS_BASE + 2)

enum 
{
    NCI_BRCM_POWER_MODE_FULL,           /* NFCC is full power mode      */
    NCI_BRCM_POWER_MODE_LAST
};

enum 
{
    NCI_BRCM_LP_TX_DATA_EVT,            /* DH is sending data to NFCC   */
    NCI_BRCM_LP_RX_DATA_EVT,            /* DH received data from NFCC   */
    NCI_BRCM_LP_TIMEOUT_EVT,            /* Timeout                      */
    NCI_BRCM_LP_LAST_EVT
};

#define NCI_ASSERT_NFC_WAKE      0x00   /* assert NFC_WAKE      */
#define NCI_DEASSERT_NFC_WAKE    0x01   /* deassert NFC_WAKE    */


#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
** Internal constants and definitions
****************************************************************************/
#define BRCM_BT_HCI_CMD_HDR_SIZE     3   /* opcode (2) +  length (1) */

/*****************************************************************************
** BRCM Patch RAM Download Control block
*****************************************************************************/

/* PRM states */
enum
{
    BRCM_PRM_ST_IDLE,
    BRCM_PRM_ST_LOADING_MINIDRV,
    BRCM_PRM_ST_LOADING_MINIDRV_DONE,
    BRCM_PRM_ST_LOADING_DATA,
    BRCM_PRM_ST_LOADING_DATA_DONE,
    BRCM_PRM_ST_LAUNCHING_RAM,
    BRCM_PRM_ST_LAUNCHING_RAM_DONE,

    /* Secure patch download stated */
    BRCM_PRM_ST_SPD_GET_VERSION,
    BRCM_PRM_ST_SPD_COMPARE_VERSION,
    BRCM_PRM_ST_SPD_GET_PATCH_HEADER,
    BRCM_PRM_ST_SPD_DOWNLOADING,
    BRCM_PRM_ST_SPD_AUTHENTICATING,
    BRCM_PRM_ST_SPD_AUTH_DONE
};
typedef UINT8 tBRCM_PRM_STATE;

/* Maximum number of patches (currently 2: LPM and FPM) */
#define BRCM_PRM_MAX_PATCH_COUNT    2
#define BRCM_PRM_PATCH_MASK_ALL     0xFFFFFFFF

/* Structures for PRM Control Block */
typedef struct
{
    UINT8               power_mode;
    UINT16              len;
} tBRCM_PRM_PATCHDESC;

typedef struct
{
    tBRCM_PRM_STATE     state;                  /* download state */
    UINT32              flags;                  /* internal flags */
    UINT16              cur_patch_len_remaining;/* bytes remaining in patchfile to process     */
    const UINT8*        p_cur_patch_data;       /* pointer to patch currently being downloaded */
    UINT16              cur_patch_offset;       /* offset of next byte to process              */
    UINT32              dest_ram;

    /* Secure Patch Download */
    TIMER_LIST_ENT      timer;
    UINT32              spd_patch_needed_mask;  /* Mask of patches that need to be downloaded */
    UINT32              spd_nvm_patch_mask;     /* Mask of patches currently in NVM */
    UINT16              spd_project_id;         /* Current project_id of patch in nvm */
    UINT16              spd_nvm_max_size;
    UINT16              spd_patch_max_size;
    UINT16              spd_fpm_patch_size;
    UINT16              spd_lpm_patch_size;

    UINT8               spd_patch_count;        /* Number of patches left to download */
    UINT8               spd_cur_patch_idx;      /* Current patch being downloaded */
    UINT16              spd_ver_major;          /* Current major version of patch in nvm */
    UINT16              spd_ver_minor;          /* Current minor version of patch in nvm */
    tBRCM_PRM_PATCHDESC spd_patch_desc[BRCM_PRM_MAX_PATCH_COUNT];

    /* I2C-patch */
    UINT8               *p_spd_patch;           /* pointer to spd patch             */
    UINT16              spd_patch_len_remaining;/* patch length                     */
    UINT16              spd_patch_offset;       /* offset of next byte to process   */

    tBRCM_PRM_FORMAT    format;
    tBRCM_PRM_CBACK     *p_cback;               /* Callback for download status notifications */
} tBRCM_PRM_CB;

/* Patch for I2C fix */
typedef struct
{
    UINT8               *p_patch;               /* patch for i2c fix                */
    UINT16              len;                    /* i2c patch length                 */
} tBRCM_PRM_I2C_FIX;

#if (NFC_RW_ONLY == FALSE)
/*****************************************************************************
** CE Type 1 Tag control block
*****************************************************************************/

typedef struct
{
    tCE_CBACK       *p_cback;           /* Pointer to the callback function to notify events  */
    UINT8           uid[T1T_UID_LEN];   /* Unique Identifier of the tag                       */
    UINT8           wait_rsp;           /* Expected response                                  */
    UINT8           cur_op;             /* Current operation                                  */
    UINT8           state;              /* Current state of the T1 card emulation i/f module  */
    TIMER_LIST_ENT  timer;              /* T1 Card emulation timer                            */
    UINT8           *p_ndef_msg;        /* Pointer to NDEF Message content                    */
    UINT16          ndef_msg_offset;    /* Offset of the NDEF Msg sent/read from controller   */
    UINT16          ndef_msg_len;       /* Length of the NDEF Msg after rw updation           */
    UINT16          init_ndef_msg_len;  /* Length of the NDEF Msg before rw updation          */
    UINT16          ndef_msg_max;       /* Size of the scratch buffer                         */
    UINT8           *p_scratch_buf;     /* Buffer to hold the updated NDEF Msg                */
    BOOLEAN         b_dynamic_mem;      /* Flag to indicate Tag has dynamic memory structure  */
    BOOLEAN         b_readonly;         /* Flag to indicate Tag is read only or not           */
} tCE_T1T_MEM;

/*****************************************************************************
** CE Type 1 Tag control block
*****************************************************************************/

typedef struct
{
    tCE_CBACK       *p_cback;           /* Pointer to the callback function to notify events  */
    UINT8           uid[TAG_MAX_UID_LEN]; /* Unique Identifier of the tag                     */
    UINT8           wait_rsp;           /* Expected response                                  */
    UINT8           cur_op;             /* Current operation                                  */
    UINT8           state;              /* Current state of the T2 card emulation i/f module  */
    TIMER_LIST_ENT  timer;              /* T2 Card emulation timer                            */
    UINT8           *p_ndef_msg;        /* Pointer to NDEF Message content                    */
    UINT16          ndef_msg_offset;    /* Offset of the NDEF Msg sent/read from controller   */
    UINT16          ndef_msg_len;       /* Length of the NDEF Msg after rw updation           */
    UINT16          init_ndef_msg_len;  /* Length of the NDEF Msg before rw updation          */
    UINT16          ndef_msg_max;       /* Size of the scratch buffer                         */
    UINT8           *p_scratch_buf;     /* Buffer to hold the updated NDEF Msg                */
    BOOLEAN         b_dynamic_mem;      /* Flag to indicate Tag has dynamic memory structure  */
    BOOLEAN         b_readonly;         /* Flag to indicate Tag is read only or not           */
} tCE_T2T_MEM;

/* CE memory control blocks */
typedef struct
{
    tCE_T1T_MEM         t1t;
    tCE_T2T_MEM         t2t;
} tCE_BRCM_MEM;

typedef struct
{
    tCE_BRCM_MEM        mem;
    tCE_CBACK           *p_cback;
    UINT8               *p_ndef;     /* the memory starting from NDEF */
    UINT16              ndef_max;    /* max size of p_ndef */
    UINT16              ndef_cur;    /* current size of p_ndef */
    tNFC_RF_TECH        tech;
    UINT8               trace_level;

} tCE_BRCM_CB;

#endif /* (NFC_RW_ONLY == FALSE) */
/*****************************************************************************
** BRCM Control block
*****************************************************************************/
typedef struct 
{
    tBRCM_PRM_CB            prm;
    tBRCM_PRM_I2C_FIX       prm_i2c;
    tNFC_STATUS_CBACK       *p_update_baud_cback;       /* Application callback for pending baud-rate change */
} tNFC_BRCM_CB;

/* BRCM NCI rcv states */
enum 
{
    NCI_BRCM_RCV_IDLE_ST,       /* wating for new message */
    NCI_BRCM_RCV_HCI_HDR_ST,    /* reading HCI header     */
    NCI_BRCM_RCV_DATA_ST        /* reading whole message  */
};

/* BRCM NCI events */
enum
{
    NCI_BRCM_HCI_CMD_EVT,               /* BT message is sent from DH */
    NCI_BRCM_API_ENABLE_SNOOZE_EVT,     /* NCI_BrcmEnableSnoozeMode () is called    */
    NCI_BRCM_API_DISABLE_SNOOZE_EVT,    /* NCI_BrcmDisableSnoozeMode () is called   */
    NCI_BRCM_API_ENTER_CE_LOW_PWR_EVT,  /* NCI_BrcmEnterCELowPowerMode () is called */
    NCI_BRCM_API_ENTER_FULL_PWR_EVT     /* NCI_BrcmEnterFullPowerMode () is called  */
};

typedef UINT8 tNCI_BRCM_EVT;

typedef BOOLEAN (tNFC_VS_BRCM_PWR_HDLR) (UINT8 event);
typedef BOOLEAN (tNFC_VS_BRCM_EVT_HDLR) (BT_HDR *p_msg);


/* BRCM NFCC initializing state */
enum
{
    NCI_BRCM_INIT_STATE_IDLE,               /* Initialization is done                */
    NCI_BRCM_INIT_STATE_W4_RESET,           /* Waiting for reset rsp                 */
    NCI_BRCM_INIT_STATE_W4_BUILD_INFO,      /* Waiting for build info rsp            */
    NCI_BRCM_INIT_STATE_W4_APP_COMPLETE     /* Waiting for complete from application */
};

typedef UINT8 tNCI_BRCM_INIT_STATE;

/* Control block for NCI transport */
typedef struct 
{
    UINT8                   rcv_state;              /* current rx state          */
    BT_HDR                  *p_rcv_msg;             /* buffer to receive message */
    UINT16                  rcv_len;                /* bytes remaining to be received in current rx state */   
    
    UINT8                   power_mode;             /* NFCC power mode                          */
    UINT8                   power_state;            /* NFCC power state in a power mode         */
    BOOLEAN                 snooze_mode_enabled;    /* TRUE if snooze mode enabled              */
    UINT8                   nfc_wake_active_mode;   /* NFC_LP_ACTIVE_LOW or NFC_LP_ACTIVE_HIGH  */

    tNCI_BRCM_INIT_STATE    initializing_state;     /* state of initializing NFCC               */
    tNFC_VS_CBACK           *p_vs_cback;            /* callback for NCI vendor specific events  */
    tNFC_BTVSC_CPLT_CBACK   *p_btvs_cback;          /* callback for BT vendor specific events   */

    TIMER_LIST_ENT          lp_timer;               /* timer for low power mode                 */
    BUFFER_Q                tx_data_q;              /* NCI command pending queue until NFCC wakes up */

    UINT8                   userial_baud_rate;      /* reconfigured baud rate                   */
    tNFC_STATUS_CBACK       *p_update_baud_cback;   /* callback to notify complete of update    */

    tNFC_VS_BRCM_PWR_HDLR   *p_vs_brcm_pwr_hdlr;    /* VS low power mode handler */
    tNFC_VS_BRCM_EVT_HDLR   *p_vs_brcm_evt_hdlr;    /* VS event handler          */

    tBRCM_DEV_INIT_CBACK    *p_dev_init_cback;      /* called for app to start initialization */
} tNCI_BRCM_CB;

/*****************************************************************************
**  EXTERNAL FUNCTION DECLARATIONS
*****************************************************************************/

/* Global NFC data */
#if NFC_DYNAMIC_MEMORY == FALSE
NFC_API extern tNFC_BRCM_CB  nfc_brcm_cb;
NFC_API extern tNCI_BRCM_CB  nci_brcm_cb;
#if (NFC_RW_ONLY == FALSE)
NFC_API extern tCE_BRCM_CB   ce_brcm_cb;
#endif
#else
#define ce_brcm_cb (*ce_brcm_cb_ptr)
NFC_API extern tNFC_BRCM_CB *nfc_brcm_cb_ptr;
#define nfc_brcm_cb (*nfc_brcm_cb_ptr)
NFC_API extern tNCI_BRCM_CB *nci_brcm_cb_ptr;
#define nci_brcm_cb (*nci_brcm_cb_ptr)
#if (NFC_RW_ONLY == FALSE)
NFC_API extern tCE_BRCM_CB *ce_brcm_cb_ptr;
#endif
#endif


/****************************************************************************
** Internal functions 
****************************************************************************/
/* nci_brcm.c */
void nci_brcm_proc_prop_rsp(BT_HDR *p_msg);
void nci_brcm_proc_prop_ntf(BT_HDR *p_msg);
void nci_brcm_init (tBRCM_DEV_INIT_CBACK *p_dev_init_cback);
void nci_brcm_send_nci_data (UINT8 *p_data, UINT16 len, tNFC_VS_CBACK *p_cback);
void nci_brcm_send_bt_data (UINT8 *p_data, UINT16 len, tNFC_BTVSC_CPLT_CBACK *p_cback);
void nci_brcm_set_local_baud_rate (UINT8 baud);
void nci_brcm_set_nfc_wake (UINT8 cmd);

/* nfc_btvscif_brcm.c */
extern void nfc_brcm_btvscif_process_event (BT_HDR *p_msg);

/* nfc_prm_brcm.c */
extern void DispNci (UINT8 *p, UINT16 len, BOOLEAN is_recv);
void nfc_brcm_prm_spd_reset_ntf(UINT8 reset_reason, UINT8 reset_type);

#define ce_brcm_init()

#define nci_vs_brcm_init();

#ifdef __cplusplus
}
#endif


#endif /* NFC_BRCM_INT_H */
