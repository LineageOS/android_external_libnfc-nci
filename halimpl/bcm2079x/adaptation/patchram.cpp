/****************************************************************************
**
**  Name:       patchram.c
**
**  Function:   this file contains functions related to firmware download
**
**
**  Copyright (c) 1999-2011, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#include "OverrideLog.h"
#include "config.h"
#include "nfc_hal_int.h"
#include "userial.h"
extern "C"
{
    #include "nfc_hal_post_reset.h"
}
#include <string>

#define LOG_TAG "NfcNciHal"

/* Location of patchfiles */
#ifndef NFCA_PATCHFILE_LOCATION
#define NFCA_PATCHFILE_LOCATION ("/system/vendor/firmware/")
#endif

#define FW_PRE_PATCH                        "FW_PRE_PATCH"
#define FW_PATCH                            "FW_PATCH"
#define NFA_CONFIG_FORMAT                   "NFA_CONFIG_FORMAT"

#define MAX_BUFFER      (512)
static char sPrePatchFn[MAX_BUFFER+1];
static char sPatchFn[MAX_BUFFER+1];
static void * sPrmBuf = NULL;
static void * sI2cFixPrmBuf = NULL;
extern "C" void setReadPacketSize(int);

#define NFA_DM_START_UP_CFG_PARAM_MAX_LEN   100
static UINT8 nfa_dm_start_up_cfg[NFA_DM_START_UP_CFG_PARAM_MAX_LEN];
extern UINT8 *p_nfc_hal_dm_start_up_cfg;
static UINT8 nfa_dm_start_up_vsc_cfg[NFA_DM_START_UP_CFG_PARAM_MAX_LEN];
extern UINT8 *p_nfc_hal_dm_start_up_vsc_cfg;
extern UINT8 *p_nfc_hal_dm_lptd_cfg;
static UINT8 nfa_dm_lptd_cfg[LPTD_PARAM_LEN];
extern tSNOOZE_MODE_CONFIG gSnoozeModeCfg;


tNFC_POST_RESET_CB nfc_post_reset_cb =
{
    "/vendor/firmware/bcm2079x_firmware.ncd",
    NULL,
    "/vendor/firmware/bcm2079x_pre_firmware.ncd",
    NULL,
    NFC_HAL_DEFAULT_BAUD,

    {0, 0},                     /* tBRCM_DEV_INIT_CONFIG dev_init_config */

    NFC_HAL_LP_SNOOZE_MODE_NONE,    /* Snooze Mode          */
    NFC_HAL_LP_IDLE_THRESHOLD_HOST, /* Idle Threshold Host  */
    NFC_HAL_LP_IDLE_THRESHOLD_HC,   /* Idle Threshold HC    */
    NFC_HAL_LP_ACTIVE_LOW,          /* NFC_WAKE Active Mode */
    NFC_HAL_LP_ACTIVE_HIGH          /* DH_WAKE Active Mode  */
};


/*******************************************************************************
**
** Function         getFileLength
**
** Description      return the size of a file
**
** Returns          file size in number of bytes
**
*******************************************************************************/
static long getFileLength(FILE* fp)
{
    long sz;
    fseek(fp, 0L, SEEK_END);
    sz = ftell(fp);
    fseek(fp, 0L, SEEK_SET);

    return sz;
}

/*******************************************************************************
**
** Function         isFileExist
**
** Description      Check if file name exists (android does not support fexists)
**
** Returns          TRUE if file exists
**
*******************************************************************************/
static BOOLEAN isFileExist(const char *pFilename)
{
    FILE *pf;

    if ((pf = fopen(pFilename, "r")) != NULL)
    {
        fclose(pf);
        return TRUE;
    }
    return FALSE;
}

/*******************************************************************************
**
** Function         findPatchramFile
**
** Description      Find the patchram file name specified in the .conf
**
** Returns          pointer to the file name
**
*******************************************************************************/
static const char* findPatchramFile(const char * pConfigName, char * pBuffer, int bufferLen)
{
    ALOGD("%s: config=%s", __FUNCTION__, pConfigName);

    if (pConfigName == NULL)
    {
        ALOGD("%s No patchfile defined\n", __FUNCTION__);
        return NULL;
    }

    if (GetStrValue(pConfigName, &pBuffer[0], bufferLen))
    {
        ALOGD("%s found patchfile %s\n", __FUNCTION__, pBuffer);
        return (pBuffer[0] == '\0') ? NULL : pBuffer;
    }

    ALOGD("%s Cannot find patchfile '%s'\n", __FUNCTION__, pConfigName);
    return NULL;
}

/*******************************************************************************
**
** Function:    continueAfterSetSnoozeMode
**
** Description: Called after Snooze Mode is enabled.
**
** Returns:     none
**
*******************************************************************************/
static void continueAfterSetSnoozeMode(tHAL_NFC_STATUS status)
{
    ALOGD("%s: status=%u", __FUNCTION__, status);
    if (status == NCI_STATUS_OK)
        HAL_NfcPreInitDone (HAL_NFC_STATUS_OK);
    else
        HAL_NfcPreInitDone (HAL_NFC_STATUS_FAILED);
}

/*******************************************************************************
**
** Function:    postDownloadPatchram
**
** Description: Called after patch download
**
** Returns:     none
**
*******************************************************************************/
extern "C" void postDownloadPatchram(tHAL_NFC_STATUS status)
{
    ALOGD("%s: status=%i", __FUNCTION__, status);
    setReadPacketSize(0);

    if (status != NCI_STATUS_OK)
    {
        ALOGE("%s: Patch download failed", __FUNCTION__);
        HAL_NfcPreInitDone(HAL_NFC_STATUS_FAILED);
        return;
    }

    /* Set snooze mode here */
    if (gSnoozeModeCfg.snooze_mode != NFC_HAL_LP_SNOOZE_MODE_NONE)
    {
        status = HAL_NfcSetSnoozeMode(gSnoozeModeCfg.snooze_mode,
                                       gSnoozeModeCfg.idle_threshold_dh,
                                       gSnoozeModeCfg.idle_threshold_nfcc,
                                       gSnoozeModeCfg.nfc_wake_active_mode,
                                       gSnoozeModeCfg.dh_wake_active_mode,
                                       continueAfterSetSnoozeMode);
        if (status != NCI_STATUS_OK)
        {
            ALOGE("%s: Setting snooze mode failed, status=%i", __FUNCTION__, status);
            HAL_NfcPreInitDone(HAL_NFC_STATUS_OK);
        }
    }
    else
    {
        ALOGD("%s: Not using Snooze Mode", __FUNCTION__);
        HAL_NfcPreInitDone(HAL_NFC_STATUS_OK);
    }
}


/*******************************************************************************
**
** Function:    prmCallback
**
** Description: Patchram callback (for static patchram mode)
**
** Returns:     none
**
*******************************************************************************/
void prmCallback(UINT8 event)
{
    ALOGD("%s: event=0x%x", __FUNCTION__, event);
    switch (event)
    {
    case NFC_HAL_PRM_CONTINUE_EVT:
        /* This event does not occur if static patchram buf is used */
        break;

    case NFC_HAL_PRM_COMPLETE_EVT:
        postDownloadPatchram(HAL_NFC_STATUS_OK);
        break;

    case NFC_HAL_PRM_ABORT_EVT:
        postDownloadPatchram(HAL_NFC_STATUS_FAILED);
        break;

    case NFC_HAL_PRM_ABORT_INVALID_PATCH_EVT:
        ALOGD("%s: invalid patch...skipping patch download", __FUNCTION__);
        postDownloadPatchram(HAL_NFC_STATUS_FAILED);
        break;

    case NFC_HAL_PRM_ABORT_BAD_SIGNATURE_EVT:
        ALOGD("%s: patch authentication failed", __FUNCTION__);
        postDownloadPatchram(HAL_NFC_STATUS_REFUSED);
        break;

    default:
        ALOGD("%s: not handled event=0x%x", __FUNCTION__, event);
        break;
    }
}


/*******************************************************************************
**
** Function         getNfaValues
**
** Description      Get configuration values needed by NFA layer
**
** Returns:         None
**
*******************************************************************************/
static void getNfaValues()
{
    unsigned long num;

    if ( GetStrValue ( NAME_NFA_DM_START_UP_CFG, (char*)nfa_dm_start_up_cfg, sizeof ( nfa_dm_start_up_cfg ) ) )
    {
        p_nfc_hal_dm_start_up_cfg = &nfa_dm_start_up_cfg[0];
        ALOGD ( "START_UP_CFG[0] = %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
                                                                            nfa_dm_start_up_cfg[0],
                                                                            nfa_dm_start_up_cfg[1],
                                                                            nfa_dm_start_up_cfg[2],
                                                                            nfa_dm_start_up_cfg[3],
                                                                            nfa_dm_start_up_cfg[4],
                                                                            nfa_dm_start_up_cfg[5],
                                                                            nfa_dm_start_up_cfg[6],
                                                                            nfa_dm_start_up_cfg[7] );
    }
    if ( GetStrValue ( NAME_NFA_DM_START_UP_VSC_CFG, (char*)nfa_dm_start_up_vsc_cfg, sizeof (nfa_dm_start_up_vsc_cfg) ) )
    {
        p_nfc_hal_dm_start_up_vsc_cfg = &nfa_dm_start_up_vsc_cfg[0];
        ALOGD ( "START_UP_VSC_CFG[0] = %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
                                                                            nfa_dm_start_up_vsc_cfg[0],
                                                                            nfa_dm_start_up_vsc_cfg[1],
                                                                            nfa_dm_start_up_vsc_cfg[2],
                                                                            nfa_dm_start_up_vsc_cfg[3],
                                                                            nfa_dm_start_up_vsc_cfg[4],
                                                                            nfa_dm_start_up_vsc_cfg[5],
                                                                            nfa_dm_start_up_vsc_cfg[6],
                                                                            nfa_dm_start_up_vsc_cfg[7] );
    }
    if ( GetStrValue ( NAME_LPTD_CFG, (char*)&nfa_dm_lptd_cfg[0], sizeof( nfa_dm_lptd_cfg ) ) )
        p_nfc_hal_dm_lptd_cfg = &nfa_dm_lptd_cfg[0];
}

static void ReadConfigFile(UINT32 chipid)
{
    ALOGD ("%s: chipid=%lx",__FUNCTION__, chipid);

    char chipID[30];
    sprintf(chipID, "%lx", chipid);
    ALOGD ("%s: chidId=%s", __FUNCTION__, chipID);

    readOptionalConfig(chipID);     // Read optional chip specific settings
    readOptionalConfig("fime");     // Read optional FIME specific settings
    getNfaValues();                 // Get NFA configuration values into variables


    findPatchramFile(FW_PATCH, sPatchFn, sizeof(sPatchFn));
    findPatchramFile(FW_PRE_PATCH, sPrePatchFn, sizeof(sPatchFn));

    {
        FILE *fd;
        /* If an I2C fix patch file was specified, then tell the stack about it */
        if (sPrePatchFn[0] != '\0')
        {
            if ((fd = fopen(sPrePatchFn, "rb")) != NULL)
            {
                UINT32 lenPrmBuffer = getFileLength(fd);

                if ((sI2cFixPrmBuf = malloc(lenPrmBuffer)) != NULL)
                {
                    fread(sI2cFixPrmBuf, lenPrmBuffer, 1, fd);

                    ALOGD("%s Setting I2C fix to %s (size: %lu)", __FUNCTION__, sPrePatchFn, lenPrmBuffer);
                    HAL_NfcPrmSetI2cPatch((UINT8*)sI2cFixPrmBuf, (UINT16)lenPrmBuffer, 0);
                }
                else
                {
                    ALOGE("%s Unable to get buffer to i2c fix (%lu bytes)", __FUNCTION__, lenPrmBuffer);
                }

                fclose(fd);
            }
            else
            {
                ALOGE("%s Unable to open i2c fix patchfile %s", __FUNCTION__, sPrePatchFn);
            }
        }
    }

    {
        FILE *fd;

        /* If a patch file was specified, then download it now */
        if (sPatchFn[0] != '\0')
        {
            /* open patchfile, read it into a buffer */
            if ((fd = fopen(sPatchFn, "rb")) != NULL)
            {
                UINT32 lenPrmBuffer = getFileLength(fd);
                tNFC_HAL_PRM_FORMAT patch_format = NFC_HAL_PRM_FORMAT_NCD;

                GetNumValue((char*)NFA_CONFIG_FORMAT, &patch_format, sizeof(patch_format));

                ALOGD("%s Downloading patchfile %s (size: %lu) format=%u", __FUNCTION__, sPatchFn, lenPrmBuffer, patch_format);
                if ((sPrmBuf = malloc(lenPrmBuffer)) != NULL)
                {
                    fread(sPrmBuf, lenPrmBuffer, 1, fd);

                    // Only use custom read size when not doing .ncd.  The stack does extra commands for NCD
                    // and the download will take place only once per new patch file.
                    if (patch_format != NFC_HAL_PRM_FORMAT_NCD)
                        setReadPacketSize(7);

                    /* Download patch using static memeory mode */
                    HAL_NfcPrmDownloadStart(patch_format, 0, (UINT8*)sPrmBuf, lenPrmBuffer, 0, prmCallback);
                }
                else
                    ALOGE("%s Unable to buffer to hold patchram (%lu bytes)", __FUNCTION__, lenPrmBuffer);

                fclose(fd);
            }
            else
                ALOGE("%s Unable to open patchfile %s", __FUNCTION__, sPatchFn);
        }
        else
        {
            ALOGE("%s: No patchfile specified or disabled. Proceeding to post-download procedure...", __FUNCTION__);
            postDownloadPatchram(HAL_NFC_STATUS_OK);
        }
    }

    ALOGD ("%s: exit", __FUNCTION__);
}

/*******************************************************************************
**
** Function:    nfc_hal_post_reset_init
**
** Description: Called by the NFC HAL after controller has been reset.
**              Begin to download firmware patch files.
**
** Returns:     none
**
*******************************************************************************/
void nfc_hal_post_reset_init (UINT32 brcm_hw_id, UINT8 nvm_type)
{
    ALOGD("%s", __FUNCTION__);
    ReadConfigFile(brcm_hw_id);
}
