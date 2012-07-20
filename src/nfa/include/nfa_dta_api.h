/*****************************************************************************
**
**  Name:           nfa_dta_api.h
**
**  Description:    NFA Device Test Application (DTA) API functions
**
**  Copyright (c) 2011, Broadcom Corp., All Rights Reserved.
**  Broadcom NFC Core. Proprietary and confidential.
**
*****************************************************************************/
#ifndef NFA_DTA_API_H
#define NFA_DTA_API_H

#include "nfc_target.h"
#include "nfa_api.h"

/*****************************************************************************
**  Constants and data types
*****************************************************************************/


/****************************************************************************/
/* Definitions for configuring DTA                                          */

/* Flags for poll/listen */
#define NFA_DTA_FL_P_A          0x01    /* Poll for NFC-A */
#define NFA_DTA_FL_P_B          0x02    /* Poll for NFC-B */
#define NFA_DTA_FL_P_F          0x04    /* Poll for NFC-F */
#define NFA_DTA_FL_L_NFCDEP_A   0x08    /* Listen for NFC-A NFCDEP */
#define NFA_DTA_FL_L_NFCDEP_F   0x10    /* Listen for NFC-F NFCDEP */
#define NFA_DTA_FL_L_T3T        0x20    /* Listen for T3T */
#define NFA_DTA_FL_L_T4T_A      0x40    /* Listen for NFC-A T4T */
#define NFA_DTA_FL_L_T4T_B      0x80    /* Listen for NFC-B T4T */
typedef UINT8 tNFA_DTA_FL_POLL_LISTEN;

/* Structure for specifying historical bytes */
typedef struct
{
    UINT8 hsby_len;
    UINT8 hsby[15];
} tDTA_DTA_CFG_HSBY;

/* Configuration parameters for NFA_DtaConfig */
#define NFA_DTA_CFG_POLL_LISTEN             0   /* default: 0xFF (all technologies/protocols enabled) */
#define NFA_DTA_CFG_TP_CONTINUE             1   /* default: TRUE (continuous mode) */
#define NFA_DTA_CFG_T4AT_NFCDEP_PRIORITY    2   /* default: FALSE (NFCDEP-A is prefered over T4AT) */
#define NFA_DTA_CFG_REACTIVATION            3   /* default: TRUE */
#define NFA_DTA_CFG_TOTAL_DURATION          4   /* default: 1000ms */
#define NFA_DTA_CFG_LLCP                    5   /* default: FALSE */
#define NFA_DTA_CFG_SNEP                    6   /* default: FALSE */
typedef UINT8 tNFA_DTA_CFG_ITEM;

/* Structures for NFA_DtaConfig configuration parameters */
typedef union
{
    tNFA_DTA_FL_POLL_LISTEN poll_listen;    /* NFA_DTA_CFG_POLL_LISTEN  */
    BOOLEAN tp_continue;                    /* NFA_DTA_CFG_TP_CONTINUE */
    BOOLEAN t4at_nfcdep_priority;           /* NFA_DTA_CFG_T4AT_NFCDEP_PRIORITY */
    BOOLEAN reactivation;                   /* NFA_DTA_CFG_REACTIVATION */
    UINT16  total_duration;                 /* NFA_DTA_CFG_TOTAL_DURATION */
    BOOLEAN enable_dta_llcp;                /* NFA_DTA_CFG_LLCP */
    BOOLEAN enable_dta_snep;                /* NFA_DTA_CFG_SNEP */
} tNFA_DTA_CFG;

#define NFA_DTA_SNEP_CLIENT_TEST_MODE_DISABLE                   0
#define NFA_DTA_SNEP_CLIENT_TEST_MODE_DEFAULT_PUT_SHORT_NDEF    1
#define NFA_DTA_SNEP_CLIENT_TEST_MODE_DEFAULT_PUT_LONG_NDEF     2
#define NFA_DTA_SNEP_CLIENT_TEST_MODE_EXTENDED_PUT_SHORT_NDEF   3
#define NFA_DTA_SNEP_CLIENT_TEST_MODE_EXTENDED_PUT_LONG_NDEF    4
#define NFA_DTA_SNEP_CLIENT_TEST_MODE_EXTENDED_GET              5
typedef UINT8 tNFA_DTA_SNEP_CLIENT_TEST_MODE;

/* tNFA_DTA_CBACK events */
#define NFA_DTA_ENABLE_EVT              0   /* DTA Mode enabled (status) */
#define NFA_DTA_DISABLE_EVT	            1   /* DTA Mode disabled */
#define NFA_DTA_START_EVT	            2   /* Main DTA loop started (status) */
#define NFA_DTA_STOP_EVT	            3   /* Main DTA loop stopped */
#define NFA_DTA_NFCC_TIMEOUT_EVT	    4   /* NFCC is not responding */
#define NFA_DTA_NFCC_TRANSPORT_ERR_EVT  5   /* NCI Transport error    */

/* tNFA_DTA_CBACK data types */
typedef union
{
    tNFA_STATUS status;
} tNFA_DTA_CBACK_DATA;

/* tNFA_DTA_CBACK: Notification of DTA events. */
typedef void (tNFA_DTA_CBACK) (UINT8 event, tNFA_DTA_CBACK_DATA *p_data);

/*****************************************************************************
**  External Function Declarations
*****************************************************************************/
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
**
** Function         NFA_DtaInit
**
** Description      This function initializes control blocks for NFA_DTA
**                  
** Returns          none
**
*******************************************************************************/
NFC_API void NFA_DtaInit (void);

/*******************************************************************************
**
** Function         NFA_DtaEnable
**
** Description      Enable DTA mode.
**
**                  This function will initialize NFC, and download patchram to
**                  the controller (if needed).
**
**                  Status of the operation will be reported with a
**                  NFA_DTA_ENABLE_EVT event.
**
**                  Note: NFA_DtaEnable will disable any prior connections
**                  before entering DTA mode.
**
** Returns:
**                  NFA_STATUS_OK if successfully initiated
**                  NFA_STATUS_FAILED otherwise
**
*******************************************************************************/
NFC_API tNFA_STATUS NFA_DtaEnable (BOOLEAN auto_start, tNFA_DTA_CBACK *p_cback);

/*******************************************************************************
**
** Function         NFA_DtaDisable
**
** Description      Disable DTA mode.
**
**                  When the NFC shutdown procedure is completed, an
**                  NFC_DTA_DISABLE_EVT will be reported.
**
** Returns:
**                  NFA_STATUS_OK if successfully initiated
**                  NFA_STATUS_FAILED otherwise
**
*******************************************************************************/
NFC_API tNFA_STATUS NFA_DtaDisable (void);


/*******************************************************************************
**
** Function         NFA_DtaConfig
**
** Description      Configures a DTA parameter
**
**
** Returns:
**                  NFA_STATUS_OK if successfully initiated
**                  NFA_STATUS_FAILED otherwise
**
*******************************************************************************/
NFC_API tNFA_STATUS NFA_DtaConfig (tNFA_DTA_CFG_ITEM config_item, tNFA_DTA_CFG *p_config);


/*******************************************************************************
**
** Function         NFA_DtaStart
**
** Description      Start the Test Profile and Listen Mode Execution Main loop
**                  (as described in [DTA] $4.1)
**
**                  The result of this operation is reported with the
**                  NFA_DTA_START_EVT.
**
** Returns:
**                  NFA_STATUS_OK if successfully initiated
**                  NFA_STATUS_FAILED otherwise
**
*******************************************************************************/
NFC_API tNFA_STATUS NFA_DtaStart (UINT8 pattern_number, UINT8 tlv_size, UINT8 *p_param_tlvs);

/*******************************************************************************
**
** Function         NFA_DtaStop
**
** Description      Stop the test. This API will deactivate any ongoing
**                  connection, stop polling/listening, and go into idle mode.
**
**                  The result of this operation is reported with the
**                  NFA_DTA_STOP_EVT.
**
** Returns:
**                  NFA_STATUS_OK if successfully initiated
**                  NFA_STATUS_FAILED otherwise
**
*******************************************************************************/
NFC_API tNFA_STATUS NFA_DtaStop (void);

/*******************************************************************************
**
** Function         NFA_DtaSnepSetClientTestMode
**
** Description      Set SNEP client test mode.
**
** Returns:
**                  NFA_STATUS_OK if successfully initiated
**                  NFA_STATUS_FAILED otherwise
**
*******************************************************************************/
NFC_API tNFA_STATUS NFA_DtaSnepSetClientTestMode (tNFA_DTA_SNEP_CLIENT_TEST_MODE mode);


#ifdef __cplusplus
}
#endif

#endif /* NFA_DTA_API_H */

