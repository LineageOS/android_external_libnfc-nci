/****************************************************************************
**
**  Name:       nfc_target.h
**
**  Function    This file contains platform-specific NFC run-time parameters
**
**  NOTE        This file should always be included first.
**
**
**  Copyright (c) 1999-2012, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/

#ifndef NFC_TARGET_H
#define NFC_TARGET_H

#include "data_types.h"

#ifdef BUILDCFG
#include "buildcfg.h"
#endif

/* Include common GKI definitions used by this platform */
#include "gki_target.h"

#include "bt_types.h"   /* This must be defined AFTER buildcfg.h */
#include "dyn_mem.h"    /* defines static and/or dynamic memory for components */
#ifndef LMP_TEST
#include "bt_trace.h"
#endif


/* API macros for DLL (needed to export API functions from DLLs) */
#define NFC_API         EXPORT_API
#define LLCP_API        EXPORT_API

/******************************************************************************
**
** GKI Mail Box and Timer
**
******************************************************************************/

/* Mailbox event mask for NFC stack */
#ifndef NFC_MBOX_EVT_MASK
#define NFC_MBOX_EVT_MASK           (TASK_MBOX_0_EVT_MASK)
#endif

/* Mailbox ID for NFC stack */
#ifndef NFC_MBOX_ID
#define NFC_MBOX_ID                 (TASK_MBOX_0)
#endif

/* Mailbox event mask for NFA */
#ifndef NFA_MBOX_EVT_MASK
#define NFA_MBOX_EVT_MASK           (TASK_MBOX_2_EVT_MASK)
#endif

/* Mailbox ID for NFA */
#ifndef NFA_MBOX_ID
#define NFA_MBOX_ID                 (TASK_MBOX_2)
#endif

/* GKI timer id used for protocol timer in NFC stack */
#ifndef NFC_TIMER_ID
#define NFC_TIMER_ID                (TIMER_0)
#endif

/* GKI timer event mask used for protocol timer in NFC stack */
#ifndef NFC_TIMER_EVT_MASK
#define NFC_TIMER_EVT_MASK          (TIMER_0_EVT_MASK)
#endif

/* GKI timer id used for quick timer in NFC stack */
#ifndef NFC_QUICK_TIMER_ID
#define NFC_QUICK_TIMER_ID          (TIMER_1)
#endif

/* GKI timer event mask used for quick timer in NFC stack */
#ifndef NFC_QUICK_TIMER_EVT_MASK
#define NFC_QUICK_TIMER_EVT_MASK    (TIMER_1_EVT_MASK)
#endif

/* GKI timer id used for protocol timer in NFA */
#ifndef NFA_TIMER_ID
#define NFA_TIMER_ID                (TIMER_2)
#endif

/* GKI timer event mask used for protocol timer in NFA */
#ifndef NFA_TIMER_EVT_MASK
#define NFA_TIMER_EVT_MASK          (TIMER_2_EVT_MASK)
#endif

/******************************************************************************
**
** GKI Buffer Pools
**
******************************************************************************/

/* NCI command/notification/data */
#ifndef NFC_NCI_POOL_ID
#define NFC_NCI_POOL_ID             GKI_POOL_ID_2
#endif

#ifndef NFC_NCI_POOL_BUF_SIZE
#define NFC_NCI_POOL_BUF_SIZE       GKI_BUF2_SIZE
#endif

/* Reader/Write commands (NCI data payload) */
#ifndef NFC_RW_POOL_ID
#define NFC_RW_POOL_ID             GKI_POOL_ID_2
#endif

#ifndef NFC_RW_POOL_BUF_SIZE
#define NFC_RW_POOL_BUF_SIZE       GKI_BUF2_SIZE
#endif

/* Card Emulation responses (NCI data payload) */
#ifndef NFC_CE_POOL_ID
#define NFC_CE_POOL_ID             GKI_POOL_ID_2
#endif

#ifndef NFC_CE_POOL_BUF_SIZE
#define NFC_CE_POOL_BUF_SIZE       GKI_BUF2_SIZE
#endif


/******************************************************************************
**
** NCI Transport definitions
**
******************************************************************************/
/* offset of the first NCI packet in buffer for outgoing */
#ifndef NCI_MSG_OFFSET_SIZE
#define NCI_MSG_OFFSET_SIZE             1
#endif

/* Sends an NCI command received from the upper stack to the NFC NCI transport. */
#ifndef NCI_CMD_TO_NFCC
#define NCI_CMD_TO_NFCC(p)         bte_hcisu_send((BT_HDR *)(p), BT_EVT_TO_NFC_NCI);
#endif

/* Restore NFCC baud rate to default on shutdown if NFC_UpdateBaudRate was called */
#ifndef NFC_RESTORE_BAUD_ON_SHUTDOWN
#define NFC_RESTORE_BAUD_ON_SHUTDOWN    TRUE
#endif

/* Default NFCC power-up baud rate */
#ifndef NFC_DEFAULT_BAUD
#define NFC_DEFAULT_BAUD                USERIAL_BAUD_115200
#endif

/******************************************************************************
**
** NCI
**
******************************************************************************/

#define NCI_VERSION_20791B2         0x0F

#ifndef NCI_VERSION
#define NCI_VERSION                 NCI_VERSION_20791B2
#endif

/* TRUE I2C patch is needed */
#ifndef NFC_I2C_PATCH_INCLUDED
#define NFC_I2C_PATCH_INCLUDED          TRUE     /* NFC-Android uses this!!! */
#endif

/******************************************************************************
**
** NFC
**
******************************************************************************/
#ifndef NFC_INCLUDED
#define NFC_INCLUDED            TRUE
#endif

/* Define to TRUE to include Broadcom Vendor Specific implementation */
#ifndef NFC_BRCM_VS_INCLUDED
#define NFC_BRCM_VS_INCLUDED    TRUE
#endif

/* Define to TRUE to include not openned Broadcom Vendor Specific implementation */
#ifndef NFC_BRCM_NOT_OPEN_INCLUDED
#define NFC_BRCM_NOT_OPEN_INCLUDED  FALSE  //Android requires FALSE
#endif

/* Define to TRUE if compling for NFC Reader/Writer Only mode */
#ifndef NFC_RW_ONLY
#define NFC_RW_ONLY         FALSE
#endif

/* Define to TRUE to use dynamic memory allocation.
 * The default is FALSE - to use static memory allocations.
*/
#ifndef NFC_DYNAMIC_MEMORY
#define NFC_DYNAMIC_MEMORY              FALSE
#endif

/* Timeout for receiving response to NCI command */
#ifndef NFC_CMD_CMPL_TIMEOUT
#define NFC_CMD_CMPL_TIMEOUT        2
#endif

/* Timeout for waiting on data credit/NFC-DEP */
#ifndef NFC_DEACTIVATE_TIMEOUT
#define NFC_DEACTIVATE_TIMEOUT      2
#endif

/* the maximum number of Vendor Specific callback functions allowed to be registered. 1-14 */
#ifndef NFC_NUM_VS_CBACKS
#define NFC_NUM_VS_CBACKS       3
#endif

/* the maximum number of NCI connections allowed. 1-14 */
#ifndef NCI_MAX_CONN_CBS
#define NCI_MAX_CONN_CBS        4
#endif

/* the maximum number of NCI command timeout before reporting NFC_NFCC_TIMEOUT_REVT */
#ifndef NCI_MAX_CMD_TIMEOUT
#define NCI_MAX_CMD_TIMEOUT         1
#endif

/* Define to TRUE to include the NFCEE related functionalities */
#ifndef NFC_NFCEE_INCLUDED
#define NFC_NFCEE_INCLUDED          TRUE
#endif

/* the maximum number of NFCEE interface supported */
#ifndef NFC_MAX_EE_INTERFACE
#define NFC_MAX_EE_INTERFACE        3
#endif

/* the maximum number of NFCEE information supported. */
#ifndef NFC_MAX_EE_INFO
#define NFC_MAX_EE_INFO        8
#endif

/* the maximum number of NFCEE TLVs supported */
#ifndef NFC_MAX_EE_TLVS
#define NFC_MAX_EE_TLVS        1
#endif

/* the maximum size of NFCEE TLV list supported */
#ifndef NFC_MAX_EE_TLV_SIZE
#define NFC_MAX_EE_TLV_SIZE        150
#endif

/* Number of times reader/writer should attempt to resend a command on failure */
#ifndef RW_MAX_RETRIES
#define RW_MAX_RETRIES              5
#endif

/* RW NDEF Support */
#ifndef RW_NDEF_INCLUDED
#define RW_NDEF_INCLUDED            TRUE
#endif

/* RW Type 1 Tag timeout for each API call, in ms */
#ifndef RW_T1T_TOUT_RESP
#define RW_T1T_TOUT_RESP            100
#endif

/* CE Type 1 Tag timeout for controller command, in ms */
#ifndef CE_T1T_TOUT_RESP
#define CE_T1T_TOUT_RESP            1000
#endif

/* CE Type 2 Tag timeout for controller command, in ms */
#ifndef CE_T2T_TOUT_RESP
#define CE_T2T_TOUT_RESP            1000
#endif

/* RW Type 2 Tag timeout for each API call, in ms */
#ifndef RW_T2T_TOUT_RESP
#define RW_T2T_TOUT_RESP            100
#endif

/* RW Type 2 Tag timeout for each API call, in ms */
#ifndef RW_T2T_SEC_SEL_TOUT_RESP
#define RW_T2T_SEC_SEL_TOUT_RESP    10
#endif

/* RW Type 3 Tag timeout for each API call, in ms */
#ifndef RW_T3T_TOUT_RESP
#define RW_T3T_TOUT_RESP            100         /* NFC-Android will use 100 instead of 75 for T3t presence-check */
#endif

/* CE Type 3 Tag maximum response timeout index (for check and update, used in SENSF_RES) */
#ifndef CE_T3T_MRTI_C
#define CE_T3T_MRTI_C               0xFF
#endif
#ifndef CE_T3T_MRTI_U
#define CE_T3T_MRTI_U               0xFF
#endif

/* Default maxblocks for CE_T3T UPDATE/CHECK operations */
#ifndef CE_T3T_DEFAULT_UPDATE_MAXBLOCKS
#define CE_T3T_DEFAULT_UPDATE_MAXBLOCKS 3
#endif

#ifndef CE_T3T_DEFAULT_CHECK_MAXBLOCKS
#define CE_T3T_DEFAULT_CHECK_MAXBLOCKS  3
#endif

/* CE Type 4 Tag, Frame Waiting time Integer */
#ifndef CE_T4T_ISO_DEP_FWI
#define CE_T4T_ISO_DEP_FWI          7
#endif

/* RW Type 4 Tag timeout for each API call, in ms */
#ifndef RW_T4T_TOUT_RESP
#define RW_T4T_TOUT_RESP            1000
#endif

/* CE Type 4 Tag timeout for update file, in ms */
#ifndef CE_T4T_TOUT_UPDATE
#define CE_T4T_TOUT_UPDATE          1000
#endif

/* CE Type 4 Tag, mandatory NDEF File ID */
#ifndef CE_T4T_MANDATORY_NDEF_FILE_ID
#define CE_T4T_MANDATORY_NDEF_FILE_ID    0x1000
#endif

/* CE Type 4 Tag, max number of AID supported */
#ifndef CE_T4T_MAX_REG_AID
#define CE_T4T_MAX_REG_AID         4
#endif

/* TRUE, to include Card Emulation related test commands */
#ifndef CE_TEST_INCLUDED
#define CE_TEST_INCLUDED            FALSE
#endif

/* Map NFC serial port to USERIAL_PORT_6 by default */
#ifndef USERIAL_NFC_PORT
#define USERIAL_NFC_PORT            (USERIAL_PORT_6)
#endif

/* NFC snooze mode */
#ifndef NFC_LP_SNOOZE_MODE
#define NFC_LP_SNOOZE_MODE          NFC_LP_SNOOZE_MODE_SPI_I2C  /* NFC-Android will use I2C by default */
#endif

/* Idle Threshold Host in 100ms unit */
#ifndef NFC_LP_IDLE_THRESHOLD_HOST
#define NFC_LP_IDLE_THRESHOLD_HOST  0
#endif

/* Idle Threshold HC in 100ms unit */
#ifndef NFC_LP_IDLE_THRESHOLD_HC
#define NFC_LP_IDLE_THRESHOLD_HC    0
#endif

/* Power cycle NFCC to move full power mode from CE low power mode */
#ifndef NFC_LP_POWER_CYCLE_TO_FULL
#define NFC_LP_POWER_CYCLE_TO_FULL  TRUE
#endif

/* time (in ms) between power off and on NFCC */
#ifndef NCI_POWER_CYCLE_DELAY
#define NCI_POWER_CYCLE_DELAY       100
#endif

/* NFCC snooze mode idle timeout before deassert NFC_WAKE in ms */
#ifndef NCI_LP_IDLE_TIMEOUT
#define NCI_LP_IDLE_TIMEOUT         100
#endif

/* NFCC boot-up delay before sending command in CE low power mode in ms */
#ifndef NCI_LP_BOOTUP_DEALY
#define NCI_LP_BOOTUP_DEALY         50
#endif

/* NFC-WAKE */
#ifndef NCILP_NFC_WAKE_GPIO
#define NCILP_NFC_WAKE_GPIO         UPIO_GENERAL3
#endif

/* Quick Timer */
#ifndef QUICK_TIMER_TICKS_PER_SEC
#define QUICK_TIMER_TICKS_PER_SEC   100       /* 10ms timer */
#endif

/* Send to lower layer */
#ifndef NCI_TO_LOWER

#ifdef TESTER
/* For BTE Insight, NCI_TO_LOWER is runtime configurable (depends on whether */
/*                     user selected 'shared' or 'dedicated' nci transport). */
/*                     Call Insight's btstk_nfc_send to forward NCI messages.*/
extern void btstk_nfc_send(BT_HDR *p_msg);
#define NCI_TO_LOWER(p)         btstk_nfc_send ((BT_HDR *)(p));

#elif defined (NFC_SHARED_TRANSPORT)
/* For embedded platforms, configured for Shared Transport:                 */
/*                      NCI_TO_LOWER calls bte_hcisu_send to send NCI       */
/*                      messages to the HCI_TASK                            */
#define NCI_TO_LOWER(p)         bte_hcisu_send((BT_HDR *)(p), ((BT_HDR *)(p))->event);

#else
/* For embedded platforms, configured for Dedicated Transport:              */
/*                      NCI_TO_LOWER calls nci_send to send NCI messages    */
/*                      to the NCI_TASK                                     */
#define NCI_TO_LOWER(p)         nci_send((BT_HDR *)(p));
#endif

#endif /* NCI_TO_LOWER */

/******************************************************************************
**
** LLCP
**
******************************************************************************/

#ifndef LLCP_TEST_INCLUDED
#define LLCP_TEST_INCLUDED          FALSE
#endif

#ifndef LLCP_POOL_ID
#define LLCP_POOL_ID                GKI_POOL_ID_3
#endif

#ifndef LLCP_POOL_BUF_SIZE
#define LLCP_POOL_BUF_SIZE          GKI_BUF3_SIZE
#endif

/* LLCP Maximum Information Unit (between LLCP_DEFAULT_MIU(128) and LLCP_MAX_MIU (2175)*/
#ifndef LLCP_MIU
#define LLCP_MIU                    (LLCP_POOL_BUF_SIZE - BT_HDR_SIZE - NCI_MSG_OFFSET_SIZE - NCI_DATA_HDR_SIZE - LLCP_PDU_HEADER_SIZE)
#endif

/* Link Timeout, LTO */
#ifndef LLCP_LTO_VALUE
#define LLCP_LTO_VALUE              1000    /* Default is 100ms. It should be sufficiently larger than RWT */
#endif

/* 
** LTO is max time interval between the last bit received and the first bit sent over the air.
** Link timeout must be delayed as much as time between the packet sent from LLCP and the last bit transmitted at NFCC.
**  - 200ms, max OTA transmitting time between the first bit and the last bit at NFCC 
**    Largest MIU(2175bytes) of LLCP must be fragmented and sent on NFC-DEP over the air. 
**    8 * (DEP_REQ/RES+ACK) + DEP_REQ/RES for 2175 MIU at 106kbps bit rate.
**  - 10ms, processing time
*/
#ifndef LLCP_INTERNAL_TX_DELAY
#define LLCP_INTERNAL_TX_DELAY      210
#endif

/* 
** LTO is max time interval between the last bit received and the first bit sent over the air.
** Link timeout must be delayed as much as time between the first bit received at NFCC and the packet received at LLCP.
**  - 200ms, max OTA transmitting time between the first bit and the last bit at NFCC 
**    LLCP cannot receive data packet until all bit are received and reassembled in NCI. 
**    8 * (DEP_REQ/RES+ACK) + DEP_REQ/RES for 2175 MIU at 106kbps bit rate.
**  - 10ms, processing time
*/
#ifndef LLCP_INTERNAL_RX_DELAY
#define LLCP_INTERNAL_RX_DELAY      210
#endif

/* Wait for application layer sending data before sending SYMM */
#ifndef LLCP_DELAY_RESP_TIME
#define LLCP_DELAY_RESP_TIME        20      /* in ms */
#endif

/* LLCP inactivity timeout for initiator */
#ifndef LLCP_INIT_INACTIVITY_TIMEOUT
#define LLCP_INIT_INACTIVITY_TIMEOUT            0    /* in ms */
#endif

/* LLCP inactivity timeout for target */
#ifndef LLCP_TARGET_INACTIVITY_TIMEOUT
#define LLCP_TARGET_INACTIVITY_TIMEOUT          0    /* in ms */
#endif

/* LLCP delay timeout to send the first PDU as initiator */
#ifndef LLCP_DELAY_TIME_TO_SEND_FIRST_PDU
#define LLCP_DELAY_TIME_TO_SEND_FIRST_PDU      50    /* in ms */
#endif

/* Response Waiting Time */
#ifndef LLCP_WAITING_TIME
#define LLCP_WAITING_TIME           7       /* its scaled value should be less than LTO */
#endif

/* Options Parameters */
#ifndef LLCP_OPT_VALUE
#define LLCP_OPT_VALUE              LLCP_LSC_3  /* Link Service Class 3 */
#endif

/* Data link connection timeout */
#ifndef LLCP_DATA_LINK_CONNECTION_TOUT
#define LLCP_DATA_LINK_CONNECTION_TOUT      1000
#endif

/* Max length of service name */
#ifndef LLCP_MAX_SN_LEN
#define LLCP_MAX_SN_LEN             255     /* max length of service name */
#endif

/* Max number of well-known services, at least 2 for LM and SDP and up to 16 */
#ifndef LLCP_MAX_WKS
#define LLCP_MAX_WKS                5
#endif

/* Max number of services advertised by local SDP, up to 16 */
#ifndef LLCP_MAX_SERVER
#define LLCP_MAX_SERVER             10
#endif

/* Max number of services not advertised by local SDP, up to 32 */
#ifndef LLCP_MAX_CLIENT
#define LLCP_MAX_CLIENT             20
#endif

/* Max number of data link connections */
#ifndef LLCP_MAX_DATA_LINK
#define LLCP_MAX_DATA_LINK          16
#endif

/* Max number of outstanding service discovery requests */
#ifndef LLCP_MAX_SDP_TRANSAC
#define LLCP_MAX_SDP_TRANSAC        16
#endif

/* Percentage of LLCP buffer pool for receiving data */
#ifndef LLCP_RX_BUFF_RATIO
#define LLCP_RX_BUFF_RATIO                  30
#endif

/* Rx congestion end threshold as percentage of receiving buffers */
#ifndef LLCP_RX_CONGEST_END
#define LLCP_RX_CONGEST_END                 50
#endif

/* Rx congestion start threshold as percentage of receiving buffers */
#ifndef LLCP_RX_CONGEST_START
#define LLCP_RX_CONGEST_START               70
#endif

/* limitation of rx UI PDU as percentage of receiving buffers */
#ifndef LLCP_LL_RX_BUFF_LIMIT
#define LLCP_LL_RX_BUFF_LIMIT               30
#endif

/* minimum rx congestion threshold (number of rx I PDU in queue) for data link connection */
#ifndef LLCP_DL_MIN_RX_CONGEST
#define LLCP_DL_MIN_RX_CONGEST              4
#endif

/* limitation of tx UI PDU as percentage of transmitting buffers */
#ifndef LLCP_LL_TX_BUFF_LIMIT
#define LLCP_LL_TX_BUFF_LIMIT               30
#endif

/******************************************************************************
**
** NFA
**
******************************************************************************/
#ifndef NFA_DYNAMIC_MEMORY
#define NFA_DYNAMIC_MEMORY          FALSE
#endif

#ifndef NFA_INCLUDED
#define NFA_INCLUDED                TRUE
#endif

#ifndef NFA_P2P_INCLUDED
#define NFA_P2P_INCLUDED            TRUE
#endif

/* Timeout for waiting on other host in HCI Network to initialize */
#ifndef NFA_HCI_NETWK_INIT_TIMEOUT
#define NFA_HCI_NETWK_INIT_TIMEOUT  400
#endif

#ifndef NFA_HCI_MAX_HOST_IN_NETWORK
#define NFA_HCI_MAX_HOST_IN_NETWORK 0x06
#endif

#ifndef NFA_HCI_MAX_GATE_IN_HOST
#define NFA_HCI_MAX_GATE_IN_HOST    0x0F
#endif

#ifndef NFA_HCI_MAX_MESSAGE_LEN
#define NFA_HCI_MAX_MESSAGE_LEN     0xFF
#endif

/* Max number of Application that can be registered to NFA-HCI */
#ifndef NFA_HCI_MAX_APP_CB
#define NFA_HCI_MAX_APP_CB          0x05
#endif

/* Max number of HCI gates that can be created */
#ifndef NFA_HCI_MAX_GATE_CB
#define NFA_HCI_MAX_GATE_CB         0x06
#endif

/* Max number of HCI pipes that can be created for the whole system */
#ifndef NFA_HCI_MAX_PIPE_CB
#define NFA_HCI_MAX_PIPE_CB         0x08
#endif

/* Max number of HCI pipes that can be created per gate */
#ifndef NFA_HCI_MAX_PIPES_PER_GATE
#define NFA_HCI_MAX_PIPES_PER_GATE  0x02
#endif

/* NFCC will respond to more than one technology during listen discovery  */
#ifndef NFA_DM_MULTI_TECH_RESP
#define NFA_DM_MULTI_TECH_RESP      FALSE    /* NFC-Android will enable/disable this by .conf file on startup */
#endif

/* Default poll duration (may be over-ridden using NFA_SetRfDiscoveryDuration) */
#ifndef NFA_DM_DISC_DURATION_POLL
#define NFA_DM_DISC_DURATION_POLL               1000
#endif

/* Automatic NDEF detection (when not in exclusive RF mode) */
#ifndef NFA_DM_AUTO_DETECT_NDEF
#define NFA_DM_AUTO_DETECT_NDEF      FALSE  /* !!!!! NFC-Android needs FALSE */
#endif

/* Automatic NDEF read (when not in exclusive RF mode) */
#ifndef NFA_DM_AUTO_READ_NDEF
#define NFA_DM_AUTO_READ_NDEF        FALSE  /* !!!!! NFC-Android needs FALSE */
#endif

/* Time to restart discovery after deactivated */
#ifndef NFA_DM_DISC_DELAY_DISCOVERY
#define NFA_DM_DISC_DELAY_DISCOVERY     1000
#endif

/* Max number of NDEF type handlers that can be registered (including the default handler) */
#ifndef NFA_NDEF_MAX_HANDLERS
#define NFA_NDEF_MAX_HANDLERS       8
#endif

/* Maximum number of listen entries configured/registered with NFA_CeConfigureUiccListenTech, */
/* NFA_CeRegisterFelicaSystemCodeOnDH, or NFA_CeRegisterT4tAidOnDH                            */
#ifndef NFA_CE_LISTEN_INFO_MAX
#define NFA_CE_LISTEN_INFO_MAX        5
#endif

#ifndef NFA_CHO_INCLUDED
#define NFA_CHO_INCLUDED            TRUE
#endif

/* MIU for CHO              */
#ifndef NFA_CHO_MIU
#define NFA_CHO_MIU                    499
#endif

/* Receiving Window for CHO */
#ifndef NFA_CHO_RW
#define NFA_CHO_RW                     4
#endif

/* Max number of alternative carrier information */
#ifndef NFA_CHO_MAX_AC_INFO
#define NFA_CHO_MAX_AC_INFO                 2
#endif

/* Max reference character length, it is up to 255 but it's RECOMMENDED short */
#ifndef NFA_CHO_MAX_REF_NAME_LEN
#define NFA_CHO_MAX_REF_NAME_LEN            8  
#endif

/* Max auxiliary data count */
#ifndef NFA_CHO_MAX_AUX_DATA_COUNT
#define NFA_CHO_MAX_AUX_DATA_COUNT          2
#endif

#ifndef NFA_CHO_TEST_INCLUDED
#define NFA_CHO_TEST_INCLUDED           FALSE
#endif

#ifndef NFA_SNEP_INCLUDED
#define NFA_SNEP_INCLUDED               TRUE
#endif

/* Max number of SNEP server/client and data link connection */
#ifndef NFA_SNEP_MAX_CONN
#define NFA_SNEP_MAX_CONN               6
#endif

/* Max number data link connection of SNEP default server*/
#ifndef NFA_SNEP_DEFAULT_MAX_CONN
#define NFA_SNEP_DEFAULT_MAX_CONN       3
#endif

/* MIU for SNEP              */
#ifndef NFA_SNEP_MIU
#define NFA_SNEP_MIU                    1980        /* Modified for NFC-A */
#endif

/* Receiving Window for SNEP */
#ifndef NFA_SNEP_RW
#define NFA_SNEP_RW                     2           /* Modified for NFC-A */
#endif

/* Max number of NFCEE supported */
#ifndef NFA_EE_MAX_EE_SUPPORTED
#define NFA_EE_MAX_EE_SUPPORTED         4
#endif

/* Maximum number of AID entries per target_handle  */
#ifndef NFA_EE_MAX_AID_ENTRIES
#define NFA_EE_MAX_AID_ENTRIES      (10)
#endif

/* Maximum number of callback functions can be registered through NFA_EeRegister() */
#ifndef NFA_EE_MAX_CBACKS
#define NFA_EE_MAX_CBACKS           (3)
#endif

#endif /* NFC_TARGET_H */



