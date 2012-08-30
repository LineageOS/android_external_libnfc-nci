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
** NCIT_TASK transport definitions
****************************************************************************/
/* NCIT_TASK event masks */
#define NCIT_TASK_EVT_DATA_RDY               EVENT_MASK (APPL_EVT_0)
#define NCIT_TASK_EVT_INITIALIZE             EVENT_MASK (APPL_EVT_5)
#define NCIT_TASK_EVT_TERMINATE              EVENT_MASK (APPL_EVT_6)
#define NCIT_TASK_EVT_POWER_CYCLE            EVENT_MASK (APPL_EVT_7)

#define NCIT_TASK_EVT_MBOX                   (TASK_MBOX_0_EVT_MASK)

/* NCIT_TASK mailbox definitions */
#define NCIT_TASK_MBOX                       (TASK_MBOX_0)

#ifndef NCIT_QUICK_TIMER_EVT_MASK
#define NCIT_QUICK_TIMER_EVT_MASK            (TIMER_0_EVT_MASK)
#endif

#ifndef NCIT_QUICK_TIMER_ID
#define NCIT_QUICK_TIMER_ID                  (TIMER_0)
#endif

/* NCI Timer events */
#define NCIT_TTYPE_NCI_WAIT_RSP              0
#define NCIT_TTYPE_POWER_CYCLE               1

/* NCI Wait Response flag */
#define NCI_WAIT_RSP_CMD                    0x10    /* wait response on an NCI command                  */
#define NCI_WAIT_RSP_VSC                    0x20    /* wait response on an NCI vendor specific command  */
#define NCI_WAIT_RSP_PROP                   0x40    /* wait response on a proprietary command           */
#define NCI_WAIT_RSP_NONE                   0x00    /* not waiting for anything                         */
typedef UINT8 tNCI_WAIT_RSP;

/* Maximum number of NCI commands that the NFCC accepts without needing to wait for response */
#define NCI_MAX_CMD_WINDOW          1


/* NCI configuration */
typedef struct
{
    BOOLEAN         shared_transport;   /* TRUE if using shared HCI/NCI transport */
    UINT8           userial_baud;
    UINT8           userial_fc;
} tNCIT_CFG;

#ifdef TESTER
#define NCIT_CFG_QUALIFIER               /* For Insight, ncit_cfg is runtime-configurable */
#else
#define NCIT_CFG_QUALIFIER   const       /* For all other platforms, ncit_cfg is constant */
#endif
extern NCIT_CFG_QUALIFIER tNCIT_CFG ncit_cfg;

/* NCIT rcv states */
enum
{
    NCIT_RCV_IDLE_ST,            /* waiting for new NCI message              */
    NCIT_RCV_NCI_HDR_ST,         /* reading NCI header                       */
    NCIT_RCV_NCI_PAYLOAD_ST,     /* reading NCI payload                      */
    NCIT_RCV_PROP_MSG_ST         /* reading proprietary message              */
};

/* internal NCIT events for vendor specific handler */
enum
{
    NCIT_INT_CHECK_NFCC_STATE_EVT,   /* Checks if the NFCC can accept messages     */
    NCIT_INT_SEND_NCI_MSG_EVT,       /* NFC task sends NCI message to NCIT task    */
    NCIT_INT_DATA_RDY_EVT,           /* Rx data is ready in serial port            */
    NCIT_INT_VS_MSG_EVT,             /* Vendor specific message                    */
    NCIT_INT_VS_INIT_EVT,            /* serial port is opened, start VS init       */
    NCIT_INT_TERMINATE_EVT           /* serial port is closing                     */
};
typedef UINT16 tNCIT_INT_EVT;

/* vendor specific event handler */
typedef BOOLEAN (tNCIT_VS_EVT_HDLR) (tNCIT_INT_EVT event, void *p);

/* callback function to process an NCI message received from NFCC */
typedef void (tNCI_RCVD_MSG_CBACK) (BT_HDR *);

/* errors during NCI packet reassembly process */
#define NCI_RAS_TOO_BIG             0x01
#define NCI_RAS_ERROR               0x02
typedef UINT8 tNCI_RAS;


/* Control block for NCI transport */
typedef struct {
    UINT8               rcv_state;          /* current rx state                 */
    UINT8               init_rcv_state;     /* initial rx state NCIT_RCV_IDLE_ST  or NCI_RCV_PROP_MSG_ST */
    UINT16              rcv_len;            /* bytes remaining to be received in current rx state */
    BT_HDR              *p_rcv_msg;         /* buffer to receive NCI message    */
    BT_HDR              *p_frag_msg;        /* fragmented NCI message; waiting for last fragment */
    tNCI_RCVD_MSG_CBACK *p_vs_rcv_cback;    /* vendor specific callback function to process tranport messages */
    tNCIT_VS_EVT_HDLR   *p_vs_evt_hdlr;     /* vendor specific processing for internal event */

    BT_HDR              *p_pend_cmd;        /* pending NCI message; waiting for NFCC state to be free */
    TIMER_LIST_Q        quick_timer_queue;  /* timer list queue                 */
    TIMER_LIST_ENT      timer;              /* timer for NCI transport task     */
    tNCI_RAS            nci_ras;            /* nci reassembly error status */

    UINT8               trace_level;        /* NCI trace level */
} tNCIT_CB;

/* Global NCI data */
#if NFC_DYNAMIC_MEMORY == FALSE
NFC_API extern tNCIT_CB   ncit_cb;
#else
#define ncit_cb (*ncit_cb_ptr)
NFC_API extern tNCIT_CB *ncit_cb_ptr;
#endif

/****************************************************************************
** Internal nfc functions
****************************************************************************/

/* From ncit_main.c */
void ncit_init (void);
void ncit_send_msg (BT_HDR *p_msg);
void ncit_task_vs_init_done (void);
void ncit_send_cmd (BT_HDR *p_buf);
void ncit_start_quick_timer (TIMER_LIST_ENT *p_tle, UINT16 type, UINT32 timeout);
void ncit_stop_quick_timer (TIMER_LIST_ENT *p_tle);
void ncit_task_timeout_cback (void *p_tle);
void ncit_send_error (UINT16 error_code);

NFC_API extern UINT32 nfc_task (UINT32 param);
NFC_API extern UINT32 ncit_task (UINT32 param);

/* Define default NCI protocol trace function (if protocol tracing is enabled) */
#if (!defined (DISP_NCI) && (BT_TRACE_PROTOCOL == TRUE))
#define DISP_NCI    (DispNci)
extern void DispNci (UINT8 *p, UINT16 len, BOOLEAN is_recv);
#endif  /* DISP_NCI */

#ifdef __cplusplus
}
#endif

#endif /* NCI_INT_H_ */
