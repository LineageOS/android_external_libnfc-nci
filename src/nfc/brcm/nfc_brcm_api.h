/****************************************************************************
** 
** Name:         nfc_brcm_api.h
** 
** Description:  Broadcom specific NFC API function external definitions.
**
** Copyright (c) 2009-2012, BROADCOM Inc., All Rights Reserved.
** Broadcom Bluetooth Core. Proprietary and confidential.
**
******************************************************************************/
#ifndef NFC_BRCM_API_H
#define NFC_BRCM_API_H

#include "bt_types.h"
#include "nfc_api.h"
#include "ce_api.h"

/*****************************************************************************
** Broadcom-specific events for tNFC_RESPONSE_CBACK
*****************************************************************************/
enum 
{
    NFC_FIRMWARE_BUILD_INFO_REVT = NFC_FIRST_VS_REVT
};

/* Definitions for NFC_FIRMWARE_BUILD_INFO_REVT */
#define BRCM_FW_BUILD_INFO_DATE_LEN         11
#define BRCM_FW_BUILD_INFO_TIME_LEN         8
#define BRCM_FW_BUILD_INFO_CHIP_ID_MAXLEN   64
typedef struct 
{
    tNFC_STATUS status;
    UINT8       date[BRCM_FW_BUILD_INFO_DATE_LEN+1];
    UINT8       time[BRCM_FW_BUILD_INFO_TIME_LEN+1];
    UINT8       ver_major;
    UINT8       ver_minor;
    UINT8       build_major;
    UINT8       build_minor;
    UINT32      hwt;
    UINT8       chip_id[BRCM_FW_BUILD_INFO_CHIP_ID_MAXLEN+1];
} tNFC_BRCM_FW_BUILD_INFO;

/* Data types for BRCM-specific tNFC_RESPONSE_CBACK event notification */
/* Dereference using tNFC_RESPONSE->p_vs_data                          */
typedef union 
{
    tNFC_BRCM_FW_BUILD_INFO fw_build_info;
} tNFC_BRCM_RESPONSE;

/*****************************************************************************
** Bluetooth Vendor specific command complete callback 
** Structure returned with Vendor Specific Command complete callback 
*****************************************************************************/
typedef struct
{
    UINT16  opcode;
    UINT16  param_len;
    UINT8   *p_param_buf;
} tNFC_BTVSC_CPLT;

typedef void (tNFC_BTVSC_CPLT_CBACK) (tNFC_BTVSC_CPLT *p1);

/* Header for HCI BT VSC */
typedef struct 
{
    BT_HDR                bt_hdr;
    tNFC_BTVSC_CPLT_CBACK *p_cback;     /* Callback for BT VSC complete */
} BT_HDR_BTVSC;

/**********************************************
 * NCI Message Proprietary  Group       - F
 **********************************************/
#define NCI_MSG_TAG_SET_MEM             0x00
#define NCI_MSG_TAG_GET_MEM             0x01
#define NCI_MSG_T1T_SET_HR              0x02
#define NCI_MSG_SET_CLF_REGISTERS       0x03
#define NCI_MSG_GET_BUILD_INFO          0x04
#define NCI_MSG_HCI_NETWK               0x05
#define NCI_MSG_SET_FWFSM               0x06
#define NCI_MSG_SET_UICCRDRF            0x07
#define NCI_MSG_POWER_LEVEL             0x08
#define NCI_MSG_FRAME_LOG               0x09
#define NCI_MSG_UICC_READER_ACTION      0x0A
#define NCI_MSG_SET_PPSE_RESPONSE       0x0B
#define NCI_MSG_PRBS_SET                0x0C
#define NCI_MSG_RESET_ALL_UICC_CFG      0x0D    /* reset HCI network/close all pipes (S,D) register */
#define NCI_MSG_GET_NFCEE_INFO          0x0E
#define NCI_MSG_DISABLE_INIT_CHECK      0x0F
#define NCI_MSG_ANTENNA_SELF_TEST       0x10
#define NCI_MSG_WI_COMM                 0x11
#define NCI_MSG_NCIP_CLK_REQ_OR_CAR_DET 0x12
#define NCI_MSG_NCIP_CONFIG_DBUART      0x13
#define NCI_MSG_NCIP_ENABLE_DVT_DRIVER  0x14
#define NCI_MSG_SET_ASWP                0x15
#define NCI_MSG_ENCAPSULATE_NCI         0x16
#define NCI_MSG_CONFIGURE_ARM_JTAG      0x17
#define NCI_MSG_STATISTICS              0x18
#define NCI_MSG_SET_DSP_TABLE           0x19
#define NCI_MSG_GET_DSP_TABLE           0x1a
#define NCI_MSG_READY_RX_CMD            0x1b
#define NCI_MSG_GET_VBAT                0x1c
#define NCI_MSG_GET_XTAL_INDEX_FROM_DH  0x1d
#define NCI_MSG_SWP_LOG                 0x1e
#define NCI_MSG_GET_PWRLEVEL            0x1f
#define NCI_MSG_SET_VBAT_MONITOR        0x20
#define NCI_MSG_SET_TINT_MODE           0x21
#define NCI_MSG_ACCESS_APP              0x22
#define NCI_MSG_SET_SECURE_MODE         0x23
#define NCI_MSG_GET_NV_DEVICE           0x24
#define NCI_MSG_LPTD                    0x25
#define NCI_MSG_SET_CE4_AS_SNOOZE       0x26
#define NCI_MSG_NFCC_SEND_HCI           0x27
#define NCI_MSG_CE4_PATCH_DOWNLOAD_DONE 0x28
#define NCI_MSG_EEPROM_RW               0x29
#define NCI_MSG_GET_CLF_REGISTERS       0x2A
#define NCI_MSG_RF_TEST                 0x2B
#define NCI_MSG_DEBUG_PRINT             0x2C
#define NCI_MSG_GET_PATCH_VERSION       0x2D
#define NCI_MSG_SECURE_PATCH_DOWNLOAD   0x2E
#define NCI_MSG_SPD_FORMAT_NVM          0x2F
#define NCI_MSG_SPD_READ_NVM            0x30

/**********************************************
 * Proprietary  NCI status codes
 **********************************************/
#define NCI_STATUS_SPD_ERROR_ORDER          0xE0
#define NCI_STATUS_SPD_ERROR_DEST           0xE1
#define NCI_STATUS_SPD_ERROR_PROJECTID      0xE2
#define NCI_STATUS_SPD_ERROR_CHIPVER        0xE3
#define NCI_STATUS_SPD_ERROR_MAJORVER       0xE4
#define NCI_STATUS_SPD_ERROR_INVALID_PARAM  0xE5
#define NCI_STATUS_SPD_ERROR_INVALID_SIG    0xE6
#define NCI_STATUS_SPD_ERROR_NVM_CORRUPTED  0xE7
#define NCI_STATUS_SPD_ERROR_PWR_MODE       0xE8
#define NCI_STATUS_SPD_ERROR_MSG_LEN        0xE9
#define NCI_STATUS_SPD_ERROR_PATCHSIZE      0xEA



#define NCI_NV_DEVICE_NONE              0x00
#define NCI_NV_DEVICE_EEPROM            0x08
#define NCI_NV_DEVICE_UICC1             0x10

/* The events reported on tNFC_VS_CBACK */
/* The event is (NCI_NTF_BIT|oid) or (NCI_RSP_BIT|oid) */
#define NFC_VS_HCI_NETWK_EVT            (NCI_NTF_BIT|NCI_MSG_HCI_NETWK)
#define NFC_VS_HCI_NETWK_RSP            (NCI_RSP_BIT|NCI_MSG_HCI_NETWK)
#define NFC_VS_UICC_READER_ACTION_EVT   (NCI_NTF_BIT|NCI_MSG_UICC_READER_ACTION)      
#define NFC_VS_POWER_LEVEL_RSP          (NCI_RSP_BIT|NCI_MSG_POWER_LEVEL)      
#define NFC_VS_GET_NV_DEVICE_EVT        (NCI_RSP_BIT|NCI_MSG_GET_NV_DEVICE)
#define NFC_VS_LPTD_EVT                 (NCI_NTF_BIT|NCI_MSG_LPTD)
#define NFC_VS_GET_BUILD_INFO_EVT       (NCI_RSP_BIT|NCI_MSG_GET_BUILD_INFO)
#define NFC_VS_GET_PATCH_VERSION_EVT    (NCI_RSP_BIT|NCI_MSG_GET_PATCH_VERSION)
#define NFC_VS_SEC_PATCH_DOWNLOAD_EVT   (NCI_RSP_BIT|NCI_MSG_SECURE_PATCH_DOWNLOAD)
#define NFC_VS_SEC_PATCH_AUTH_EVT       (NCI_NTF_BIT|NCI_MSG_SECURE_PATCH_DOWNLOAD)

/* Bit 0 - op */
#define NCI_AA_CTRL_OP_MASK             0x01
#define NCI_AA_CTRL_OP_READ             0x00
#define NCI_AA_CTRL_OP_WRITE            0x01

/* Bit 1~2 - destination */
#define NCI_AA_CTRL_TYPE_MASK           0x06
#define NCI_AA_CTRL_TYPE_AONMEM         0x02
#define NCI_AA_CTRL_TYPE_NV             0x04

/* Bit 3 - not used by DH */
/* Bit 4~7 - access type */
#define NCI_AA_CTRL_DATA_MASK           0xf0
#define NCI_AA_CTRL_DT_PPSE_LEN         0x00
#define NCI_AA_CTRL_DT_PPSE_RSP         0x10
#define NCI_AA_CTRL_DT_PATCH_LEN        0x20
#define NCI_AA_CTRL_DT_PATCH            0x30
#define NCI_AA_CTRL_DT_AID_LEN          0x40
#define NCI_AA_CTRL_DT_AID              0x50
#define NCI_AA_CTRL_DT_PSN_LEN          0x60
#define NCI_AA_CTRL_DT_PSN              0x70
#define NCI_AA_CTRL_DT_PIPE_INFO_LEN    0x80
#define NCI_AA_CTRL_DT_PIPE_INFO        0x90
#define NCI_AA_CTRL_DT_WORKAROUND       0xa0
#define NCI_AA_CTRL_DT_RESET_PSN        0xb0
#define NCI_AA_CTRL_DT_LOW_POWER        0x70 /* B3 uses this DT to trigger saving low pwoer related information to NV */
#define NCI_AA_HEADER_SIZE              0x03 /* control/1 + offset/2 */

#define NCI_NFCC_PIPE_INFO_NV_SIZE      24  /* Static and dynamic pipe id and status for each pipe to uicc0 and uicc1. */
#define NCI_PERSONALITY_SLOT_SIZE       19
#define NCI_DYNAMIC_PIPE_SIZE           8

#define NCI_SWP_INTERFACE_TYPE          0xFF    /* Type of TLV in NCI_MSG_HCI_NETWK */
#define NCI_HCI_GATE_TYPE               0xFE    /* Type of TLV in NCI_MSG_HCI_NETWK */

/* Secure Patch Download definitions (patch type definitions) */
#define NCI_SPD_TYPE_HEADER             0x00
#define NCI_SPD_TYPE_SRAM               0x01
#define NCI_SPD_TYPE_AON                0x02
#define NCI_SPD_TYPE_PATCH_TABLE        0x03
#define NCI_SPD_TYPE_SECURE_CONFIG      0x04
#define NCI_SPD_TYPE_CONTROLLED_CONFIG  0x05
#define NCI_SPD_TYPE_SIGNATURE          0x06
#define NCI_SPD_TYPE_SIGCHEK            0x07

/* Secure Patch Download definitions (NCI_SPD_TYPE_HEADER definitions) */
#define NCI_SPD_HEADER_OFFSET_CHIPVERLEN    0x18
#define NCI_SPD_HEADER_CHIPVER_LEN          16

/* NVM Type (in GET_PATCH_VERSION RSP) */
#define NCI_SPD_NVM_TYPE_NONE           0x00
#define NCI_SPD_NVM_TYPE_EEPROM         0x01
#define NCI_SPD_NVM_TYPE_UICC           0x02

/**********************************************
 * NCI NFCC proprietary features in octet 3
 **********************************************/
#define NCI_FEAT_SIGNED_PATCH           0x01000000

/**********************************************
 * NCI Interface Types
 **********************************************/
#define NCI_INTERFACE_VS_CALYPSO_CE     0x81
#define NCI_INTERFACE_VS_T2T_CE         0x82    /* for Card Emulation side */
#define NCI_INTERFACE_VS_15693          0x83    /* for both Reader/Writer and Card Emulation side */
#define NCI_INTERFACE_VS_T1T_CE         0x84    /* for Card Emulation side */

/**********************************************
 * NCI Proprietary Parameter IDs
 **********************************************/
#define NCI_PARAM_ID_LA_FSDI            0xA0
#define NCI_PARAM_ID_LB_FSDI            0xA1
#define NCI_PARAM_ID_HOST_LISTEN_MASK   0xA2
#define NCI_PARAM_ID_CHIP_TYPE          0xA3 /* NFCDEP */
#define NCI_PARAM_ID_PA_ANTICOLL        0xA4
#define NCI_PARAM_ID_CONTINUE_MODE      0xA5
#define NCI_PARAM_ID_LBP                0xA6
#define NCI_PARAM_ID_T1T_RDR_ONLY       0xA7
#define NCI_PARAM_ID_LA_SENS_RES        0xA8
#define NCI_PARAM_ID_PWR_SETTING_BITMAP 0xA9
#define NCI_PARAM_ID_WI_NTF_ENABLE      0xAA
#define NCI_PARAM_ID_LN_BITRATE         0xAB /* NFCDEP Listen Bitrate */
#define NCI_PARAM_ID_LF_BITRATE         0xAC /* FeliCa */
#define NCI_PARAM_ID_SWP_BITRATE_MASK   0xAD
#define NCI_PARAM_ID_KOVIO              0xAE
#define NCI_PARAM_ID_UICC_NTF_TO        0xAF
#define NCI_PARAM_ID_NFCDEP             0xB0
#define NCI_PARAM_ID_CLF_REGS_CFG       0xB1
#define NCI_PARAM_ID_NFCDEP_TRANS_TIME  0xB2
#define NCI_PARAM_ID_CREDIT_TIMER       0xB3
#define NCI_PARAM_ID_CORRUPT_RX         0xB4
#define NCI_PARAM_ID_ISODEP             0xB5
#define NCI_PARAM_ID_LF_CONFIG          0xB6
#define NCI_PARAM_ID_I93_DATARATE       0xB7
#define NCI_PARAM_ID_CREDITS_THRESHOLD  0xB8
#define NCI_PARAM_ID_TAGSNIFF_CFG       0xB9
#define NCI_PARAM_ID_PA_FSDI            0xBA /* ISODEP */
#define NCI_PARAM_ID_PB_FSDI            0xBB /* ISODEP */
#define NCI_PARAM_ID_FRAME_INTF_RETXN   0xBC

#define NCI_PARAM_ID_UICC_RDR_PRIORITY  0xBD
#define NCI_PARAM_ID_GUARD_TIME         0xBE
#define NCI_PARAM_ID_STDCONFIG          0xBF /* dont not use this config item */
#define NCI_PARAM_ID_PROPCFG            0xC0 /* dont not use this config item  */
#define NCI_PARAM_ID_MAXTRY2ACTIVATE    0xC1
#define NCI_PARAM_ID_SWPCFG             0xC2
#define NCI_PARAM_ID_CLF_LPM_CFG        0xC3
#define NCI_PARAM_ID_DCLB               0xC4
#define NCI_PARAM_ID_ACT_ORDER          0xC5
#define NCI_PARAM_ID_DEP_DELAY_ACT      0xC6
#define NCI_PARAM_ID_DH_PARITY_CRC_CTL  0xC7
#define NCI_PARAM_ID_PREINIT_DSP_CFG    0xC8
#define NCI_PARAM_ID_FW_WORKAROUND      0xC9
#define NCI_PARAM_ID_RFU_CONFIG         0xCA
#define NCI_PARAM_ID_EMVCO_ENABLE       0xCB
#define NCI_PARAM_ID_ANTDRIVER_PARAM    0xCC
#define NCI_PARAM_ID_PLL325_CFG_PARAM   0xCD
#define NCI_PARAM_ID_OPNLP_ADPLL_ENABLE 0xCE
#define NCI_PARAM_ID_CONFORMANCE_MODE   0xCF

#define NCI_PARAM_ID_LPO_ON_OFF_ENABLE  0xD0
#define NCI_PARAM_ID_FORCE_VANT         0xD1
#define NCI_PARAM_ID_COEX_CONFIG        0xD2
#define NCI_PARAM_ID_INTEL_MODE         0xD3

#define NCI_PARAM_ID_AID                0xFF

/**********************************************
 * NCI Parameter ID Lens
 **********************************************/
#define NCI_PARAM_LEN_PWR_SETTING_BITMAP    3
#define NCI_PARAM_LEN_HOST_LISTEN_MASK      2

/*****************************************************************************
**  Low Power Mode definitions
*****************************************************************************/
#define NFC_LP_SNOOZE_MODE_NONE      0x00    /* Snooze mode disabled    */
#define NFC_LP_SNOOZE_MODE_UART      0x01    /* Snooze mode for UART    */
#define NFC_LP_SNOOZE_MODE_SPI_I2C   0x08    /* Snooze mode for SPI/I2C */

#define NFC_LP_ACTIVE_LOW   0x00             /* high to low voltage is asserting  */
#define NFC_LP_ACTIVE_HIGH  0x01             /* low to high voltage is asserting */

#define CE_T1T_FIRST_EVT    0x20
#define CE_T2T_FIRST_EVT    0x40

/*****************************************************************************
**  Patch RAM MANAGEMENT
*****************************************************************************/
/****************************
**  Patch RAM Constants
*****************************/
/* Events for tBRCM_PRM_CBACK */
enum
{
    BRCM_PRM_CONTINUE_EVT,
    BRCM_PRM_COMPLETE_EVT,
    BRCM_PRM_ABORT_EVT,
    BRCM_PRM_ABORT_INVALID_PATCH_EVT,       /* Patch is invalid (bad version, project id, or chip)  */
    BRCM_PRM_ABORT_BAD_SIGNATURE_EVT,       /* Patch has invalid signature                          */

    BRCM_PRM_SPD_GET_PATCHFILE_HDR_EVT,     /* Secure Patch Download: request for patchfile header  */
    BRCM_PRM_SPD_GET_NEXT_PATCH             /* Get first command of next patch in patchfile         */
};

/*************************************
**  Patch RAM Callback for event notificaton
**************************************/
typedef void (tBRCM_PRM_CBACK) (UINT8 event);

/* patch format type */
#define BRCM_PRM_FORMAT_BIN  0x00
#define BRCM_PRM_FORMAT_HCD  0x01
#define BRCM_PRM_FORMAT_NCD  0x02
typedef UINT8 tBRCM_PRM_FORMAT;

/* 
** Callback function for application to start device initialization 
** When platform-specific initialization is completed, 
** NCI_BrcmDevInitDone() must be called to proceed with stack start up.
*/

typedef void (tBRCM_DEV_INIT_CBACK) (UINT32 brcm_hw_id);

/****************************
**  CE T1/T2 Constants
*****************************/
enum
{
    /* Note: the order of these events can not be changed. 
     * If new events are needed, add them after CE_T1T_SETMEM_HR_EVT */
    CE_T1T_SETMEM_HR_EVT = CE_T1T_FIRST_EVT,
    CE_T1T_NDEF_UPDATE_START_EVT,
    CE_T1T_NDEF_UPDATE_CPLT_EVT,
    CE_T1T_MAX_EVT,

    /* Type 2 tag events */
    CE_T2T_SETMEM_EVT = CE_T2T_FIRST_EVT,
    CE_T2T_NDEF_UPDATE_START_EVT,
    CE_T2T_NDEF_UPDATE_CPLT_EVT,
    CE_T2T_MAX_EVT
};

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
**  Bluetooth VS Command MANAGEMENT FUNCTIONS
*****************************************************************************/
/*******************************************************************************
**
** Function         NFC_BrcmInit
**
** Description      This function initializes Broadcom specific control blocks for NFC
**                  and registers a callback to be called after the NFCC is reset, 
**                  to perform any platform-specific initialization (e.g. patch download).
**
** Returns          void
**
*******************************************************************************/
NFC_API extern void NFC_BrcmInit (tBRCM_DEV_INIT_CBACK *p_dev_init_cback);

/*******************************************************************************
**
** Function         NFC_BrcmGetFirmwareBuildInfo
**
** Description      Get firmware build info
**                  NFC_FIRMWARE_BUILD_INFO_REVT will indicate the result
**
** Returns          NFC_STATUS_OK if operation successfully started
**                  NFC_STATUS_FAILED otherwise
**
*******************************************************************************/
NFC_API extern tNFC_STATUS NFC_BrcmGetFirmwareBuildInfo(void);

/*******************************************************************************
**
** Function         NFC_UpdateBaudRate
**
** Description      Reconfigure controller's NCI transport baud rate.
**                  Only used for dedicated transport; for shared BT/NFC 
**                  transport, the baud rate is controlled by BT.
**
**                  Upon success notification, the host must reconfigure
**                  its baud rate to match the controller baud rate.  
**
** Returns          tNFC_STATUS
**
*******************************************************************************/
NFC_API extern tNFC_STATUS NFC_UpdateBaudRate (UINT32               baud,
                                               tNFC_STATUS_CBACK   *p_update_baud_cback);

/*******************************************************************************
**
** Function         NFC_SendBtVsCommand
**
** Description      Send BT Vendor Specific Command
**
** Returns          tNFC_STATUS
**
*******************************************************************************/
NFC_API extern tNFC_STATUS NFC_SendBtVsCommand (UINT16 opcode, 
                                                UINT8  param_len, 
                                                UINT8 *p_param,
                                                tNFC_BTVSC_CPLT_CBACK *p_cback);

/*******************************************************************************
**
** Function         CE_BrcmSetActivatedTagType
**
** Description      This function selects the tag type for Reader/Writer mode.  
**
** Returns          tNFC_STATUS
**
*******************************************************************************/
NFC_API extern tNFC_STATUS CE_BrcmSetActivatedTagType (tNFC_ACTIVATE_DEVT *p_activate_params, tCE_CBACK *p_cback);

/*****************************************************************************
**  Low power MANAGEMENT FUNCTIONS
*****************************************************************************/

/*******************************************************************************
**
** Function         NCI_BrcmEnableSnoozeMode
**
** Description      Notify NCI transport that snooze mode has been enabled. 
**
** Returns          tNFC_STATUS
**
*******************************************************************************/
NFC_API extern tNFC_STATUS NCI_BrcmEnableSnoozeMode (UINT8 nfc_wake_active_mode);

/*******************************************************************************
**
** Function         NCI_BrcmDisableSnoozeMode
**
** Description      Notify NCI transport that snooze mode has been disabled.  
**
** Returns          tNFC_STATUS
**
*******************************************************************************/
NFC_API extern tNFC_STATUS NCI_BrcmDisableSnoozeMode (void);

/*******************************************************************************
**
** Function         CE_T1tSetLocalNDEFMsg
**
** Description      Initialise CE Type 1 Tag with mandatory NDEF message
**
**                  The following event may be returned
**                      CE_T1T_NDEF_UPDATE_CPLT_EVT for complete update
**                  
**                  read_only:      TRUE if read only
**                  ndef_msg_max:   Max NDEF message size
**                  ndef_msg_len:   NDEF message size
**                  p_ndef_msg:     NDEF message (excluding NLEN)
**                  p_scratch_buf:  temp storage for update
**                  p_cback:        callback function to get notification
**                  uid_len:        UID size
**                  p_uid:          UID
**
**                  If the NDEF Message size is less than or equal to 86 bytes, 
**                  the tag is configured as static memory structure tag.
**                  However based on Max NDEF size, one or more static lock 
**                  bits will get set. 
**                  If the size is more than 86 bytes, the tag is configured as 
**                  dynamic memory structure tag. Based on Max NDEF size one or
**                  more Dynamic and static lock bits will get set.
**
**                  uid_len value will be ignored if p_uid is set to NULL
**                  
** Returns          NFC_STATUS_OK if successfully initiated to set NDEF
**                  NFC_STATUS_BUSY,if busy updating/reading NDEF from controller
**                  NFC_STATUS_FAILED otherwise
**
*******************************************************************************/
NFC_API extern tNFC_STATUS CE_T1tSetLocalNDEFMsg (BOOLEAN    read_only,
                                                  UINT16     ndef_msg_max,
                                                  UINT16     ndef_msg_len,
                                                  UINT8     *p_ndef_msg,
                                                  UINT8     *p_scratch_buf,
                                                  tCE_CBACK *p_cback,
                                                  UINT8      uid_len,
                                                  UINT8      *p_uid);

/*******************************************************************************
**
** Function         CE_T2tSetLocalNDEFMsg
**
** Description      Initialise CE Type 2 Tag with mandatory NDEF message
**
**                  The following event may be returned
**                      CE_T2T_NDEF_UPDATE_CPLT_EVT for complete update
**                  
**                  read_only:      TRUE if read only
**                  ndef_msg_max:   Max NDEF message size
**                  ndef_msg_len:   NDEF message size
**                  p_ndef_msg:     NDEF message (excluding NLEN)
**                  p_scratch_buf:  temp storage for update
**                  p_cback:        callback function to get notification
**                  uid_len:        UID size
**                  p_uid:          UID
**
**                  If the NDEF Message size is less than 46 bytes, the tag 
**                  is configured as static memory structure tag. 
**                  However based on Max NDEF size, one or more static lock 
**                  bits will get set. 
**                  If the size is more than 46 bytes, the tag is configured as 
**                  dynamic memory structure tag. Based on Max NDEF size one or
**                  more Dynamic and static lock bits will get set.
**
**                  uid_len value will be ignored if p_uid is set to NULL
**
** Returns          NFC_STATUS_OK if successfully initiated to set NDEF
**                  NFC_STATUS_BUSY,if busy updating/reading NDEF from controller
**                  NFC_STATUS_FAILED otherwise
**
*******************************************************************************/
NFC_API extern tNFC_STATUS CE_T2tSetLocalNDEFMsg (BOOLEAN    read_only,
                                                  UINT16     ndef_msg_max,
                                                  UINT16     ndef_msg_len,
                                                  UINT8     *p_ndef_msg,
                                                  UINT8     *p_scratch_buf,
                                                  tCE_CBACK *p_cback,
                                                  UINT8      uid_len,
                                                  UINT8      *p_uid);

/*******************************************************************************
**
** Function         NCI_BrcmSetBaudRate
**
** Description      Set UART baud rate
**
** Returns          void
**
*******************************************************************************/
NFC_API extern void NCI_BrcmSetBaudRate (UINT8             userial_baud_rate, 
                                         tNFC_STATUS_CBACK *p_update_baud_cback);

/*******************************************************************************
**
** Function         NCI_BrcmDevInitDone
**
** Description      Notify NCI transport that device is initialized 
**
** Returns          void
**
*******************************************************************************/
NFC_API extern void NCI_BrcmDevInitDone (void);

/*****************************************************************************
**  Patch RAM MANAGEMENT FUNCTIONS
*****************************************************************************/
/*******************************************************************************
**
** Function         PRM_DownloadStart
**
** Description      Initiate patch download
**
** Input Params
**                  format_type     patch format type (HCI or BIN) 
**
**                  dest_address    destination adderess (needed for BIN format only)
**
**                  p_patchram_buf  pointer to patchram buffer. If NULL, 
**                                  then app must call PRM_DownloadContinue when
**                                  PRM_CONTINUE_EVT is received, to send the next
**                                  segment of patchram
**
**                  patchram_len    size of p_patchram_buf (if non-NULL)
**
**                  p_cback         callback for download status
**
**
** Returns          TRUE if successful, otherwise FALSE
**                  
**
*******************************************************************************/
EXPORT_API extern BOOLEAN PRM_DownloadStart (tBRCM_PRM_FORMAT format_type,
                                             UINT32           dest_address,
                                             UINT8           *p_patchram_buf,
                                             UINT32           patchram_len,
                                             tBRCM_PRM_CBACK *p_cback);

/*******************************************************************************
**
** Function         PRM_DownloadContinue
**
** Description      Send next segment of patchram to controller. Called when
**                  PRM_CONTINUE_EVT is received.
**
**                  Only needed if PRM_DownloadStart was called with
**                  p_patchram_buf=NULL
**                  
** Input Params     p_patch_data    pointer to patch data
**                  patch_data_len  patch data len
**                  
** Returns          TRUE if successful, otherwise FALSE
**
*******************************************************************************/
EXPORT_API extern BOOLEAN PRM_DownloadContinue (UINT8 *p_patch_data,
                                                UINT16 patch_data_len);

/*******************************************************************************
**
** Function         PRM_LaunchRam
**
** Description      Launch patchram.
**
**                  Called after receiving PRM_CONTINUE_EVT/PRM_COMPLETE_EVT, when
**                  entire patchram has been downloaded.
**
**                  (Note: not needed if using .hcd patches with LAUNCH_RAM
**                  embedded in the .hcd file)
**                 
**
** Input Param      None
**
**
** Returns          TRUE if successful, otherwise FALSE
**
*******************************************************************************/
EXPORT_API extern BOOLEAN PRM_LaunchRam (void);

/*******************************************************************************
**
** Function         PRM_SetI2cPatch
**
** Description      Specify patchfile for BCM20791B3 I2C fix. This fix
**                  must be downloaded prior to initial patch download for I2C
**                  transport
**
** Input Params     p_i2c_patchfile_buf: pointer to patch for i2c fix
**                  i2c_patchfile_len: length of patch
**                  
**
** Returns          Nothing
**                  
**
*******************************************************************************/
EXPORT_API extern void PRM_SetI2cPatch (UINT8 *p_i2c_patchfile_buf, UINT16 i2c_patchfile_len);

#ifdef __cplusplus
}
#endif

#endif

