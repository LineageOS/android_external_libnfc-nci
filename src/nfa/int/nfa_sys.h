/*****************************************************************************
**
**  Name:           nfa_sys.h
**
**  Description:    This is the public interface file for the BTA system
**                  manager.
**
**  Copyright (c) 2003-2010, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#ifndef NFA_SYS_H
#define NFA_SYS_H

#include "nfc_target.h"
#include "gki.h"
#include "nfa_api.h"

/*****************************************************************************
**  Constants and data types
*****************************************************************************/

/* SW sub-systems */
#define NFA_ID_SYS          0            /* system manager                      */

#define NFA_ID_VS_PRE       1            /* Vendor Specific before standard NFA */
#define NFA_ID_DM           2            /* device manager                      */
#define NFA_ID_EE           3            /* NFCEE sub-system                    */
#define NFA_ID_P2P          4            /* Peer-to-Peer sub-system             */
#define NFA_ID_CHO          5            /* Connection Handover sub-system      */
#define NFA_ID_SNEP         6            /* SNEP sub-system                     */
#define NFA_ID_RW           7            /* Reader/writer sub-system            */
#define NFA_ID_CE           8            /* Card-emulation sub-system           */
#define NFA_ID_HCI          9            /* Host controller interface sub-system*/
#define NFA_ID_DTA          10           /* Device Test Application sub-system  */
#define NFA_ID_VS_POST      11           /* Vendor Specific after standard NFA  */

#define NFA_ID_MAX          12

typedef UINT8 tNFA_SYS_ID;
/* enable function type */
typedef void (tNFA_SYS_ENABLE) (void);

/* event handler function type */
typedef BOOLEAN (tNFA_SYS_EVT_HDLR) (BT_HDR *p_msg);

/* disable function type */
typedef void (tNFA_SYS_DISABLE) (void);

/* function type for processing the change of NFCC power mode */
typedef void (tNFA_SYS_PROC_NFCC_PWR_MODE) (UINT8 nfcc_power_mode);

typedef void (tNFA_SYS_CBACK) (void);
typedef void (tNFA_SYS_ENABLE_CBACK) (void);
typedef void (tNFA_SYS_PROC_NFCC_PWR_MODE_CMPL) (void);

/* registration structure */
typedef struct
{
    tNFA_SYS_ENABLE                 *enable;
    tNFA_SYS_EVT_HDLR               *evt_hdlr;
    tNFA_SYS_DISABLE                *disable;
    tNFA_SYS_PROC_NFCC_PWR_MODE     *proc_nfcc_pwr_mode;
} tNFA_SYS_REG;

/* system manager configuration structure */
typedef struct
{
    UINT16          mbox_evt;                       /* GKI mailbox event */
    UINT8           mbox;                           /* GKI mailbox id */
    UINT8           timer;                          /* GKI timer id */
    UINT8           trace_level;                    /* initial trace level */
} tNFA_SYS_CFG;


/*****************************************************************************
**  Global data
*****************************************************************************/

/*****************************************************************************
**  Macros
*****************************************************************************/

/* Calculate start of event enumeration; id is top 8 bits of event */
#define NFA_SYS_EVT_START(id)       ((id) << 8)


/*****************************************************************************
**  Function declarations
*****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

NFC_API extern void nfa_sys_init (void);
NFC_API extern void nfa_sys_event (BT_HDR *p_msg);
NFC_API extern void nfa_sys_timer_update (void);
NFC_API extern void nfa_sys_disable_timers (void);
NFC_API extern void nfa_sys_set_trace_level (UINT8 level);

extern void nfa_sys_register (UINT8 id, const tNFA_SYS_REG *p_reg);
extern void nfa_sys_deregister (UINT8 id);
extern BOOLEAN nfa_sys_is_register (UINT8 id);
extern void nfa_sys_disable_subsystems (BOOLEAN graceful);
extern void nfa_sys_enable_subsystems (void);

extern BOOLEAN nfa_sys_is_graceful_disable (void);
extern void nfa_sys_sendmsg (void *p_msg);
extern void nfa_sys_start_timer (TIMER_LIST_ENT *p_tle, UINT16 type, INT32 timeout);
extern void nfa_sys_stop_timer (TIMER_LIST_ENT *p_tle);

extern void nfa_sys_cback_reg_enable_complete (tNFA_SYS_ENABLE_CBACK *p_cback);
extern void nfa_sys_cback_notify_enable_complete (UINT8 id);

extern void nfa_sys_notify_nfcc_power_mode (UINT8 nfcc_power_mode);
extern void nfa_sys_cback_reg_nfcc_power_mode_proc_complete (tNFA_SYS_PROC_NFCC_PWR_MODE_CMPL *p_cback);
extern void nfa_sys_cback_notify_nfcc_power_mode_proc_complete (UINT8 id);

#ifdef __cplusplus
}
#endif

#endif /* NFA_SYS_H */
