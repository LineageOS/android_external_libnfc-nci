/****************************************************************************
** 
**  File:        nci_int.h
**
**  Description:   this file contains the NCI transport
**                 internal definitions and functions.                   
**
**  Copyright (c) 2009-2010, Broadcom Corp., All Rights Reserved.
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
    NCI_INT_SEND_NCI_MSG_EVT,   /* NFC task sends NCI message to NCI task     */
    NCI_INT_DATA_RDY_EVT,       /* Rx data is ready in serial port            */
    NCI_INT_RX_NCI_MSG_EVT,     /* Complete NCI message is received from NFCC */    
    NCI_INT_VS_MSG_EVT,         /* Vendor specific message is sending to NFCC */
    NCI_INT_VS_INIT_EVT,        /* serial port is openned, start VS init      */
    NCI_INT_TERMINATE_EVT       /* serial port is closing                     */
};
typedef UINT16 tNCI_INT_EVT;

/* vendor specific event handler */
typedef BOOLEAN (tNCI_VS_EVT_HDLR) (tNCI_INT_EVT event, void *p);

/* Control block for NCI transport */
typedef struct {
    UINT8               rcv_state;          /* current rx state                 */
    BT_HDR              *p_rcv_msg;         /* buffer to receive NCI message    */
    UINT16              rcv_len;            /* bytes remaining to be received in current rx state */      

    TIMER_LIST_ENT      timer;              /* timer for power cylce NFCC       */

    UINT8               init_rcv_state;     /* initial rx state NCI_RCV_IDLE_ST or NCI_RCV_VS_PREAMBLE_ST */
    TIMER_LIST_Q        quick_timer_queue;  /* timer list queue                 */
    tNCI_VS_EVT_HDLR    *p_vs_evt_hdlr;     /* vendor specific processing for internal event */

    UINT8   trace_level;        /* NCI trace level */
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

#if defined(QUICK_TIMER_TICKS_PER_SEC) && (QUICK_TIMER_TICKS_PER_SEC > 0)
void nci_start_quick_timer (TIMER_LIST_ENT *p_tle, UINT16 type, UINT32 timeout);
void nci_stop_quick_timer (TIMER_LIST_ENT *p_tle);
#endif

NFC_API extern UINT32 nfc_task (UINT32 param);
NFC_API extern UINT32 nci_task (UINT32 param);

#ifdef __cplusplus
}
#endif

#endif /* NCI_INT_H_ */
