/****************************************************************************
** 
**  File:        nfc_int.h
**
**  Description:   this file contains the main NFC Upper Layer
**                 internal definitions and functions.                   
**
**  Copyright (c) 2009-2012, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
****************************************************************************/

#ifndef NFC_INT_H_
#define NFC_INT_H_


#include "nfc_target.h"
#include "gki.h"
#include "nci_defs.h"
#include "nfc_api.h"
#include "btu_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
** Internal NFC constants and definitions
****************************************************************************/

/****************************************************************************
** NFC_TASK definitions
****************************************************************************/

/* NFC_TASK event masks */
#define NFC_TASK_EVT_TRANSPORT_READY        EVENT_MASK(APPL_EVT_0)
#define NFC_TASK_EVT_ENABLE                 EVENT_MASK(APPL_EVT_1)
#define NFC_TASK_EVT_TERMINATE              EVENT_MASK(APPL_EVT_7)

/* Error codes for BT_EVT_TO_NFC_ERR event */
#define NFC_ERR_TRANSPORT                   0   /* fatal error in NCI transport */


/* NFC Timer events */
#define NFC_TTYPE_NCI_CMD_CMPL              0
#define NFC_TTYPE_WAIT_2_DEACTIVATE         1

#define NFC_TTYPE_LLCP_LINK_MANAGER         100
#define NFC_TTYPE_LLCP_LINK_INACT           101
#define NFC_TTYPE_LLCP_DATA_LINK            102
#define NFC_TTYPE_LLCP_DELAY_FIRST_PDU      103
#define NFC_TTYPE_RW_T1T_RESPONSE           104
#define NFC_TTYPE_RW_T2T_RESPONSE           105
#define NFC_TTYPE_RW_T3T_RESPONSE           106
#define NFC_TTYPE_RW_T4T_RESPONSE           107
#define NFC_TTYPE_RW_I93_RESPONSE           108
#define NFC_TTYPE_CE_T4T_UPDATE             109
#define NFC_TTYPE_VS_BASE                   200


/* NFC Task event messages */

enum
{
    NFC_STATE_NONE,     /* not start up yet         */
    NFC_STATE_IDLE,     /* reset/init               */
    NFC_STATE_OPEN,     /* NFC link is activated    */
    NFC_STATE_CLOSING,  /* de-activating            */
    NFC_STATE_RESTARTING,           /* restarting NFCC after reboot */
    NFC_STATE_NFCC_POWER_OFF_SLEEP  /* NFCC is power-off sleep mode */
};
typedef UINT8 tNFC_STATE;

/* NFC control block flags */
#define NFC_FL_ENABLED                  0x0001  /* NFC enabled                                  */
#define NFC_FL_ENABLE_PENDING           0x0002  /* NFC is being enabled (NFC_ENABLE_EVT pending)*/
#define NFC_FL_NCI_TRANSPORT_ENABLED    0x0004  /* Transport enabled                            */
#define NFC_FL_DEACTIVATING             0x0008  /* NFC_Deactivate() is called and the NCI cmd is not sent   */
#define NFC_FL_W4_TRANSPORT_READY       0x0010  /* Waiting for NCI transport to be ready before enabling NFC*/
#define NFC_FL_POWER_CYCLE_NFCC         0x0020  /* Power cycle NFCC                             */

#define NFC_PEND_CONN_ID               0xFE
#define NFC_CONN_ID_INT_MASK           0xF0
#define NFC_CONN_ID_ID_MASK            NCI_CID_MASK
#define NFC_CONN_NO_FC                 0xFF /* set num_buff to this for no flow control */
#define NFC_NCI_CONN_NO_FC             0xFF

typedef struct
{
    tNFC_CONN_CBACK *p_cback;   /* the callback function to receive the data        */
    BUFFER_Q    tx_q;           /* transmit queue                                   */
    BUFFER_Q    rx_q;           /* receive queue                                    */
    UINT8       id;             /* NFCEE ID or RF Discovery ID or NFC_TEST_ID       */
    UINT8       act_protocol;   /* the active protocol on this logical connection   */
    UINT8       buff_size;      /* the max buffer size for this connection.     .   */
    UINT8       num_buff;       /* num of buffers left to send on this connection   */
    UINT8       init_credits;   /* initial num of buffer credits                    */
    UINT8       conn_id;        /* the connection id assigned by NFCC for this conn */
} tNFC_CONN_CB;

#define NFC_SAVED_CMD_SIZE      2

enum
{
    NFC_INT_ENABLE_END_EVT,     /* Right before reporting NFC_ENABLE_EVT */
    NFC_INT_MBOX_EVT,           /* Received mailbox event */
    NFC_INT_NCI_VS_RSP_EVT,     /* Received NCI VS response */
    NFC_INT_NCI_VS_NTF_EVT,     /* Received NCI VS notification */
    NFC_INT_TIMEOUT_EVT,        /* timeout event */
    NFC_INT_DISABLE_EVT         /* Disabling the stack */
};
typedef UINT16   tNFC_INT_EVT;
typedef BOOLEAN (tNFC_VS_EVT_HDLR) (tNFC_INT_EVT event, void *p);

/* NFCC power state change pending callback */
typedef void (tNFC_PWR_ST_CBACK) (void);

/* NFC control blocks */
typedef struct
{
    UINT16              flags;                      /* NFC control block flags - NFC_FL_* */
    UINT8               last_cmd[NFC_SAVED_CMD_SIZE];/* the first few bytes of last NCI command(in case out of GKI buffer) */
    tNFC_CONN_CB        conn_cb[NCI_MAX_CONN_CBS];
    UINT8               conn_id[NFC_MAX_CONN_ID+1]; /* index: conn_id; conn_id[]: index(1 based) to conn_cb[] */
    tNFC_DISCOVER_CBACK *p_discv_cback;
    tNFC_RESPONSE_CBACK *p_resp_cback;
    tNFC_TEST_CBACK     *p_test_cback;
    tNFC_VS_CBACK       *p_vs_cb[NFC_NUM_VS_CBACKS];/* Register for vendor specific events  */

#if (NFC_RW_ONLY == FALSE)
    /* NFCC information at init rsp */
    UINT32              nci_features;               /* the NCI features supported by NFCC */
    UINT16              max_ce_table;               /* the max routing table size       */
    UINT8               max_conn;                   /* the num of connections supported by NFCC */
#endif

    /* the data members in this section may be changed by NFC_<VendorSpecific>Init() */
    tNFC_VS_EVT_HDLR    *p_vs_evt_hdlr;             /* VS processing for internal events */
    const tNCI_DISCOVER_MAPS  *p_disc_maps;         /* NCI RF Discovery interface mapping */
    UINT8               num_disc_maps;              /* number of RF Discovery interface mappings */
    UINT16              nci_interfaces;             /* the NCI interfaces of NFCC       */
    UINT8               vs_interface[NFC_NFCC_MAX_NUM_VS_INTERFACE];  /* the NCI VS interfaces of NFCC    */

    /* NFC_TASK timer management */
    TIMER_LIST_Q        timer_queue;                /* 1-sec timer event queue */
    TIMER_LIST_Q        quick_timer_queue;

    BUFFER_Q            nci_cmd_xmit_q;             /* NCI command queue */
    BT_HDR              *p_nci_last_cmd;            /* the last command sent to NFCC; waiting for rsp */
    BT_HDR              *p_frag_msg;                /* fragmented NCI message; waiting for last fragment */
    TIMER_LIST_ENT      nci_cmd_cmpl_timer;         /* Timer for waiting for nci command response */
    TIMER_LIST_ENT      deactivate_timer;           /* Timer to wait for deactivation */
    UINT16              nci_cmd_cplt_tout;          /* NCI command timeout (in seconds) */
    UINT8               nci_ctrl_size;              /* Max Control Packet Payload Size */
    UINT8               nci_cmd_window;             /* Number of commands the controller can accecpt without waiting for response */
    UINT8               nci_num_timeout;            /* number of NCI cmd timeout */

    tNFC_STATE          nfc_state;
    UINT8               trace_level;
} tNFC_CB;


/*****************************************************************************
**  EXTERNAL FUNCTION DECLARATIONS
*****************************************************************************/

/* Global NFC data */
#if NFC_DYNAMIC_MEMORY == FALSE
NFC_API extern tNFC_CB  nfc_cb;
#else
NFC_API extern tNFC_CB *nfc_cb_ptr;
#define nfc_cb (*nfc_cb_ptr)
#endif

/****************************************************************************
** Internal nfc functions 
****************************************************************************/

NFC_API extern void nfc_init(void);

/* from nfc_utils.c */
NFC_API extern tNFC_CONN_CB * nfc_alloc_conn_cb( tNFC_CONN_CBACK *p_cback);
NFC_API extern tNFC_CONN_CB * nfc_find_conn_cb_by_conn_id(UINT8 conn_id);
NFC_API extern tNFC_CONN_CB * nfc_find_conn_cb_by_handle(UINT8 target_handle);
NFC_API extern void nfc_set_conn_id(tNFC_CONN_CB * p_cb, UINT8 conn_id);
NFC_API extern void nfc_free_conn_cb( tNFC_CONN_CB *p_cb);
NFC_API extern void nfc_reset_all_conn_cbs( void);
NFC_API extern void nfc_data_event(tNFC_CONN_CB * p_cb);

void nfc_ncif_send (BT_HDR *p_buf, BOOLEAN is_cmd);
extern UINT8 nfc_ncif_send_data (tNFC_CONN_CB *p_cb, BT_HDR *p_data);
NFC_API extern void nfc_ncif_cmd_timeout (void);
NFC_API extern void nfc_wait_2_deactivate_timeout (void);
NFC_API extern void nfc_ncif_update_window(BOOLEAN is_rsp);

NFC_API extern BOOLEAN nfc_ncif_process_event (BT_HDR *p_msg);
NFC_API extern void nfc_ncif_send_cmd (BT_HDR *p_buf);
NFC_API extern void nfc_ncif_proc_discover_ntf(UINT8 *p, UINT16 plen);
NFC_API extern void nfc_ncif_rf_management_status(tNFC_DISCOVER_EVT event, UINT8 status);
NFC_API extern void nfc_ncif_set_config_status (UINT8 *p, UINT8 len);
NFC_API extern void nfc_ncif_event_status(tNFC_RESPONSE_EVT event, UINT8 status);
NFC_API extern void nfc_ncif_error_status(UINT8 conn_id, UINT8 status);
NFC_API extern void nfc_ncif_proc_credits(UINT8 *p, UINT16 plen);
NFC_API extern void nfc_ncif_proc_activate(UINT8 *p, UINT8 len);
NFC_API extern void nfc_ncif_proc_deactivate(UINT8 status, UINT8 deact_type, BOOLEAN is_ntf);
#if ((NFC_NFCEE_INCLUDED == TRUE) && (NFC_RW_ONLY == FALSE))
NFC_API extern void nfc_ncif_proc_ee_action(UINT8 *p, UINT16 plen);
NFC_API extern void nfc_ncif_proc_ee_discover_req(UINT8 *p, UINT16 plen);
NFC_API extern void nfc_ncif_proc_get_routing(UINT8 *p, UINT8 len);
#endif
NFC_API extern void nfc_ncif_proc_conn_create_rsp (UINT8 *p, UINT16 plen, UINT8 dest_type);
NFC_API extern void nfc_ncif_report_conn_close_evt (UINT8 conn_id, tNFC_STATUS status);
NFC_API extern void nfc_ncif_proc_t3t_polling_ntf(UINT8 *p, UINT16 plen);
NFC_API extern void nfc_ncif_proc_reset_rsp (UINT8 *p, BOOLEAN is_ntf);
NFC_API extern void nfc_ncif_proc_init_rsp (BT_HDR *p_msg);
NFC_API extern void nfc_ncif_proc_get_config_rsp (BT_HDR *p_msg);
NFC_API extern void nfc_ncif_proc_data (BT_HDR *p_msg);
extern void nfc_main_disable_complete(tNFC_STATUS status);

#if (NFC_RW_ONLY == FALSE)
NFC_API extern void nfc_ncif_proc_rf_field_ntf(UINT8 rf_status);
#else
#define nfc_ncif_proc_rf_field_ntf(rf_status)
#endif

/* From nci_main.c */
NFC_API extern void nfc_notify_shared_transport_ready(void);

/* From nfc_main.c */
void nfc_enabled (tNFC_STATUS nfc_status, BT_HDR *p_init_rsp_msg);
void nfc_set_state (tNFC_STATE nfc_state);
void nfc_gen_cleanup(void);
void nfc_main_cleanup(void);
void nfc_main_handle_err(BT_HDR *p_err_msg);
void nfc_main_flush_cmd_queue (void);

/* Timer functions */
void nfc_start_timer (TIMER_LIST_ENT *p_tle, UINT16 type, UINT32 timeout);
UINT32 nfc_remaining_time (TIMER_LIST_ENT *p_tle);
void nfc_stop_timer (TIMER_LIST_ENT *p_tle);

void nfc_start_quick_timer (TIMER_LIST_ENT *p_tle, UINT16 type, UINT32 timeout); 
void nfc_stop_quick_timer (TIMER_LIST_ENT *p_tle);
void nfc_process_quick_timer_evt (void);
#ifdef __cplusplus
}
#endif

#endif /* NFC_INT_H_ */
