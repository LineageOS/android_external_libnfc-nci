/****************************************************************************
** 
**  File:        nci_int.h
**
**  Description:   this file contains the NCI transport
**                 internal definitions and functions.                   
**
**  Copyright (c) 2009-2012, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
****************************************************************************/

#ifndef NCI_INT_H_
#define NCI_INT_H_


#include "gki.h"
#include "nci_defs.h"


#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
** NCI_TASK transport definitions
****************************************************************************/
/* NCI_TASK event masks */
#define NCI_TASK_EVT_DATA_RDY               EVENT_MASK(APPL_EVT_0)
#define NCI_TASK_EVT_INITIALIZE             EVENT_MASK(APPL_EVT_5)
#define NCI_TASK_EVT_TERMINATE              EVENT_MASK(APPL_EVT_6)
#define NCI_TASK_EVT_POWER_CYCLE            EVENT_MASK(APPL_EVT_7)

#define NCI_TASK_EVT_MBOX                   (TASK_MBOX_0_EVT_MASK)

/* NCI_TASK mailbox definitions */
#define NCI_TASK_MBOX                       (TASK_MBOX_0)

#ifndef NCI_QUICK_TIMER_EVT_MASK
#define NCI_QUICK_TIMER_EVT_MASK            (TIMER_0_EVT_MASK)
#endif

#ifndef NCI_QUICK_TIMER_ID
#define NCI_QUICK_TIMER_ID                  (TIMER_0)
#endif

/* NCI Timer events */
#define NCI_TTYPE_NCI_CMD_CMPL              0
#define NCI_TTYPE_POWER_CYCLE               1


/* NCI message layer specific */
#define NCI_MSGS_LS_CONTROL                 0x80    /* control message (cmd/rsp/ntf)                    */
#define NCI_MSGS_LS_NCI_VSC                 0x40    /* NCI VS control message (cmd/rsp/ntf)             */
#define NCI_MSGS_LS_VSC                     0x20    /* Other VS control message (cmd/rsp/ntf)           */
#define NCI_MSGS_LS_DATA                    0x10    /* data message (connection ID is not specified)    */
#define NCI_MSGS_LS_DATA_MASK               0x0F    /* data message for connection ID as indicated      */
#define NCI_MSGS_LS_INVALID                 0xFF    /* NCI is not waiting for any response  */

/* Maximum number of NCI commands that the NFCC accepts without needing to wait for response */
#define NCI_MAX_CMD_WINDOW          1


/* NCI configuration */
typedef struct 
{
    BOOLEAN         shared_transport;   /* TRUE if using shared HCI/NCI transport */
    UINT8           userial_baud;
    UINT8           userial_fc;
} tNCI_CFG;

#ifdef TESTER
#define NCI_CFG_QUALIFIER               /* For Insight, nci_cfg is runtime-configurable */
#else
#define NCI_CFG_QUALIFIER   const       /* For all other platforms, nci_cfg is constant */
#endif
extern NCI_CFG_QUALIFIER tNCI_CFG nci_cfg;

/* NCI rcv states */
enum 
{
    NCI_RCV_IDLE_ST,            /* waiting for new NCI message          */
    NCI_RCV_NFC_HDR_ST,         /* reading NCI header                   */
    NCI_RCV_DATA_ST,            /* reading NCI message                  */
    NCI_RCV_VS_PREAMBLE_ST,     /* waiting for vendor specific header   */
    NCI_RCV_VS_MSG_ST           /* reading vendor specific message      */
};

/* vendor specific initialization complete callback */
typedef void (tNCI_VS_INIT_DONE_CBACK) (void);

/* internal NCI events for vendor specific handler */
enum
{
    NCI_INT_CHECK_STATE_EVT,    /* Checks if the NFCC can accept messages     */
    NCI_INT_SEND_NCI_MSG_EVT,   /* NFC task sends NCI message to NCI task     */
    NCI_INT_SEND_VS_CMD_EVT,    /* a credit is available to send VS command   */
    NCI_INT_DATA_RDY_EVT,       /* Rx data is ready in serial port            */
    NCI_INT_RX_NCI_MSG_EVT,     /* Complete NCI message is received from NFCC */    
    NCI_INT_RX_VS_MSG_EVT,      /* Complete VS message is received from NFCC  */    
    NCI_INT_VS_MSG_EVT,         /* Vendor specific message for NFCC           */
    NCI_INT_VS_INIT_EVT,        /* serial port is openned, start VS init      */
    NCI_INT_TERMINATE_EVT       /* serial port is closing                     */
};
typedef UINT16 tNCI_INT_EVT;

/* vendor specific event handler */
typedef BOOLEAN (tNCI_VS_EVT_HDLR) (tNCI_INT_EVT event, void *p);

typedef struct
{
    BUFFER_Q    tx_q;           /* transmit queue                                   */
    BUFFER_Q    rx_q;           /* receive queue                                    */
    UINT8       conn_id;        /* the connection id assigned by NFCC for this conn */
    UINT8       buff_size;      /* the max buffer size for this connection.     .   */
    UINT8       num_buff;       /* num of buffers left to send on this connection   */
    UINT8       init_credits;   /* initial num of buffer credits                    */
} tNCI_CONN_CB;

#define NCI_SAVED_HDR_SIZE          (2)

/* errors during NCI packet reassembly process */
#define NCI_RAS_TOO_BIG             0x01
#define NCI_RAS_ERROR               0x02
typedef UINT8 tNCI_RAS;

#define NCI_FLAGS_DEACTIVATING      0x01
typedef UINT8 tNCI_FLAGS;

/* Control block for NCI transport */
typedef struct {
    UINT8               rcv_state;          /* current rx state                 */
    UINT8               init_rcv_state;     /* initial rx state NCI_RCV_IDLE_ST or NCI_RCV_VS_PREAMBLE_ST */
    UINT16              rcv_len;            /* bytes remaining to be received in current rx state */      
    BT_HDR              *p_rcv_msg;         /* buffer to receive NCI message    */
    tNCI_CONN_CB        conn_cb[NCI_MAX_CONN_CBS];
    UINT8               conn_id[NFC_MAX_CONN_ID+1]; /* index: conn_id; conn_id[]: index(1 based) to conn_cb[] */

    TIMER_LIST_Q        quick_timer_queue;  /* timer list queue                 */
    tNCI_VS_EVT_HDLR    *p_vs_evt_hdlr;     /* vendor specific processing for internal event */

    UINT8               last_hdr[NCI_SAVED_HDR_SIZE];/* part of last NCI command header */
    UINT8               last_cmd[NFC_SAVED_CMD_SIZE];/* part of last NCI command payload */
    void                *p_last_cback;      /* the callback function for last VSC command */
    BUFFER_Q            nci_cmd_xmit_q;     /* NCI command queue */
    BT_HDR              *p_frag_msg;        /* fragmented NCI message; waiting for last fragment */
    TIMER_LIST_ENT      nci_cmd_cmpl_timer; /* Timer for waiting for nci command response */
    UINT16              nci_cmd_cplt_tout;  /* NCI command timeout (in ms) */
    UINT8               nci_last_ls;        /* layer_specific for last NCI message */
    tNCI_RAS            nci_ras;            /* nci reassembly error status */
    tNCI_FLAGS          nci_flags;          /* nci various status */

    UINT8               nci_cmd_window;     /* Number of commands the controller can accecpt without waiting for response */
    UINT8               trace_level;        /* NCI trace level */
} tNCI_CB;

/* Global NCI data */
#if NFC_DYNAMIC_MEMORY == FALSE
NFC_API extern tNCI_CB   nci_cb;
#else
#define nci_cb (*nci_cb_ptr)
NFC_API extern tNCI_CB *nci_cb_ptr;
#endif

/****************************************************************************
** Internal nfc functions
****************************************************************************/

/* From nci_main.c */
void nci_init (void);
void nci_send (BT_HDR *p_msg);
void nci_task_vs_init_done (void);
void nci_send_cmd (BT_HDR *p_buf);
void nci_update_window(BOOLEAN is_rsp);
void nci_start_quick_timer (TIMER_LIST_ENT *p_tle, UINT16 type, UINT32 timeout);
void nci_stop_quick_timer (TIMER_LIST_ENT *p_tle);
void nci_task_timeout_cback (void *p_tle);
void nci_set_cmd_timeout_val (UINT16 timeout_val);

NFC_API extern UINT32 nfc_task (UINT32 param);
NFC_API extern UINT32 nci_task (UINT32 param);

/* Define default NCI protocol trace function (if protocol tracing is enabled) */
#if (!defined(DISP_NCI) && (BT_TRACE_PROTOCOL == TRUE))
#define DISP_NCI    (DispNci)
extern void DispNci (UINT8 *p, UINT16 len, BOOLEAN is_recv);
#endif  /* DISP_NCI */

#ifdef __cplusplus
}
#endif

#endif /* NCI_INT_H_ */
