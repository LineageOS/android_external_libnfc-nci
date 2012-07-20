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
#include "NfcAdaptation.h"
#include "config.h"
extern "C" {
    #include "gki.h"
    #include "nfa_api.h"
    #include "nci_int.h"
    #include "nfc_int.h"
    #include <cutils/log.h>
    #include "userial.h"
    #include "nfc_brcm_api.h"
};

#define LOG_TAG "BrcmNfcNfa"

/* Location of patchfiles */
#ifndef NFCA_PATCHFILE_LOCATION
#define NFCA_PATCHFILE_LOCATION ("/system/vendor/firmware/")
#endif

#ifndef NFCA_DEFAULT_PATCHFILE_CONTAINER
#define NFCA_DEFAULT_PATCHFILE_CONTAINER    ("/vendor/firmware/nfcfirmware.txt")
#endif
#define NFCA_PATCHFILE_EXTENSION            (".hcd")
#define NFCA_PATCHFILE_EXTENSION_LEN        (4)
#define NFA_CONFIG_DL_CFG                   "NFA_CONFIG_DL_CFG"
#define NFA_CONFIG_I2C_FIX_CFG              "NFA_CONFIG_I2C_FIX_CFG"
#define NFA_CONFIG_FORMAT                   "NFA_CONFIG_FORMAT"
#define NFA_CONFIG_XTAL_INDEX               "NFA_CONFIG_XTAL_INDEX"

#define MAX_BUFFER      (512)

static char sBuffer[MAX_BUFFER+1];
static void * sPrmBuf = NULL;
static void * sI2cFixPrmBuf = NULL;

extern "C" void setReadPacketSize(int);

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
static const char* findPatchramFile(char * pConfigName)
{
    if (GetStrValue(pConfigName, &sBuffer[0], MAX_BUFFER))
    {
        ALOGD("%s From .conf found patchfile %s\n", __func__, sBuffer);
        return sBuffer;
    }

    ALOGD("%s Cannot find patchfile '%s'\n", __func__, pConfigName);
    return NULL;
}

/*******************************************************************************
**
** Function         continueAfterSetXtalIndex
**
** Description      continue the reset process after set_xtalIndex is called.
**
** Returns          none
**
*******************************************************************************/
extern "C" void continueAfterSetXtalIndex (tNFC_VS_EVT event, UINT16 len, UINT8* p_data)
{
    ALOGD("%s :complete", __func__);

    NCI_BrcmDevInitDone();
}

/*******************************************************************************
**
** Function         readXtalIndex
**
** Description      continue the reset process after set_xtalIndex is called.
**
** Returns          none
**
*******************************************************************************/
static void readXtalIndex(UINT8 xtalIndex)
{
    BT_HDR  *pMsg = (BT_HDR *)GKI_getbuf((UINT16)(BT_HDR_SIZE + NCI_VSC_MSG_HDR_SIZE + 1));

    ALOGD("%s: index=0x%02x",__func__, xtalIndex);

    if (pMsg)
    {
        pMsg->offset = NCI_VSC_MSG_HDR_SIZE;
        UINT8 *p = (UINT8*)(pMsg + 1) + pMsg->offset;
        *p = xtalIndex;    /* xtalIndex 0x04 for 24Mhz */
        pMsg->len = 1;
        NFC_SendVsCommand(NCI_MSG_GET_XTAL_INDEX_FROM_DH, pMsg, continueAfterSetXtalIndex);
    }
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
extern "C" void postDownloadPatchram(tNFC_STATUS status)
{
    UINT8 xtalIndex = 0;

    ALOGD("postDownloadPatchram: status=%i", status);
    setReadPacketSize(0);

#if (defined(NFA_APP_DOWNLOAD_NFC_PATCHRAM) && (NFA_APP_DOWNLOAD_NFC_PATCHRAM == TRUE))
    /* If a buffers were allocated for patch download, free them now */
    if (sI2cFixPrmBuf)
    {
        free(sI2cFixPrmBuf);
        sI2cFixPrmBuf = NULL;
    }
    if (sPrmBuf)
    {
        free(sPrmBuf);
        sPrmBuf = NULL;
    }
#endif

    if (status != NFC_STATUS_OK)
        ALOGE("Patch download failed");

    if (GetNumValue((char*)NFA_CONFIG_XTAL_INDEX, &xtalIndex, sizeof(xtalIndex)))
    {
        ALOGD("%s: setting XTAL index=%d", __func__, xtalIndex);

        /* Proceed with setting XTAL, then next step of NFC startup sequence */
        readXtalIndex(xtalIndex);
    }
    else /* Proceed with next step of NFC startup sequence */
        NCI_BrcmDevInitDone();
}

#if (defined(NFA_APP_DOWNLOAD_NFC_PATCHRAM) && (NFA_APP_DOWNLOAD_NFC_PATCHRAM == TRUE))
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
    ALOGD("prmCallback evt: 0x%x", event);

    switch (event)
    {
    case BRCM_PRM_CONTINUE_EVT:
        /* This event does not occur if static patchram buf is used */
        break;
        
    case BRCM_PRM_COMPLETE_EVT:
        postDownloadPatchram(NFC_STATUS_OK);
        break;
        
    case BRCM_PRM_ABORT_EVT:
        postDownloadPatchram(NFC_STATUS_FAILED);
        break;
        
    case BRCM_PRM_ABORT_INVALID_PATCH_EVT:
        ALOGD("prmCallback: invalid patch...skipping patch download");
        postDownloadPatchram(NFC_STATUS_FAILED);
        break;
        
    case BRCM_PRM_ABORT_BAD_SIGNATURE_EVT:
        ALOGD("prmCallback: patch authentication failed");
        postDownloadPatchram(NFC_STATUS_REFUSED);
        break;
        
    default:
        ALOGD("prmCallback unhandled evt: 0x%x", event);
        break;
        }
}
#endif  /* NFA_APP_DOWNLOAD_NFC_PATCHRAM */

/*******************************************************************************
**
** Function:    postBaudUpdate
**
** Description: Called after baud rate udate
**
** Returns:     none
**
*******************************************************************************/
extern "C" void postBaudUpdate(tNFC_STATUS status)
{
    ALOGD("postBaudUpdate");

#if (defined(NFA_APP_RECONFIG_BAUD) && (NFA_APP_RECONFIG_BAUD != 0))
    ALOGD("%s calling USERIAL_GetBaud()\n", __func__);
    /* Update host baud rate */
    if (status == NFC_STATUS_OK)
    {
        ALOGD("%s calling userial GetBaud\n", __func__);
        tUSERIAL_IOCTL_DATA data;
        data.baud = USERIAL_GetBaud(NFA_APP_RECONFIG_BAUD);
        USERIAL_Ioctl(USERIAL_NFC_PORT, USERIAL_OP_BAUD_WR, &data);
    }
#endif


#if (defined(NFA_APP_DOWNLOAD_NFC_PATCHRAM) && (NFA_APP_DOWNLOAD_NFC_PATCHRAM == TRUE))
    
    {
        FILE *fd;
        const char* pfilename = findPatchramFile(NFA_CONFIG_I2C_FIX_CFG);

        /* If an I2C fix patch file was specified, then tell the stack about it */
        if (pfilename != NULL && pfilename[0] != '\0')
        {    
            if ((fd = fopen(pfilename, "rb")) != NULL)
            {
                UINT32 lenPrmBuffer = getFileLength(fd);

                if ((sI2cFixPrmBuf = malloc(lenPrmBuffer)) != NULL)
                {
                    fread(sI2cFixPrmBuf, lenPrmBuffer, 1, fd);

                    ALOGD("%s Setting I2C fix to %s (size: %d)", __func__, pfilename, lenPrmBuffer);
                    PRM_SetI2cPatch((UINT8*)sI2cFixPrmBuf, (UINT16)lenPrmBuffer);
                }
                else
                {
                    ALOGE("%s Unable to get buffer to i2c fix (%l bytes)", __func__, lenPrmBuffer);
                }

                fclose(fd);
            }
            else
            {
                ALOGE("%s Unable to open i2c fix patchfile %s", __func__, pfilename);
            }
        }
    }
    
    {
        FILE *fd;
        const char* pfilename = findPatchramFile(NFA_CONFIG_DL_CFG);

        /* If a patch file was specified, then download it now */
        if (pfilename != NULL && pfilename[0] != '\0')
        {
            /* open patchfile, read it into a buffer */
            if ((fd = fopen(pfilename, "rb")) != NULL)
            {
                UINT32 lenPrmBuffer = getFileLength(fd);
                tBRCM_PRM_FORMAT patch_format = BRCM_PRM_FORMAT_NCD;

                GetNumValue((char*)NFA_CONFIG_FORMAT, &patch_format, sizeof(patch_format));

                ALOGD("%s Downloading patchfile %s (size: %d) format=%d", __func__, pfilename, lenPrmBuffer, patch_format);
                if ((sPrmBuf = malloc(lenPrmBuffer)) != NULL)
                {
                    fread(sPrmBuf, lenPrmBuffer, 1, fd);
                    
                    // Only use custom read size when not doing .ncd.  The stack does extra commands for NCD
                    // and the download will take place only once per new patch file.
                    if (patch_format != BRCM_PRM_FORMAT_NCD)
                        setReadPacketSize(7);

                    /* Download patch using static memeory mode */
                    PRM_DownloadStart(patch_format, 0, (UINT8*)sPrmBuf, lenPrmBuffer, prmCallback);
                }
                else
                    ALOGE("%s Unable to buffer to hold patchram (%l bytes)", __func__, lenPrmBuffer);

                fclose(fd);
            }
            else
                ALOGE("%s Unable to open patchfile %s", __func__, pfilename);

        }
        else
        {
            ALOGE("No patchfile specified or disabled. Proceeding to post-download procedure...");
            postDownloadPatchram(NFC_STATUS_OK);
        }
    }

#else
    postDownloadPatchram(NFC_STATUS_OK);
#endif
}

/*******************************************************************************
**
** Function:    nfa_app_post_nci_reset
**
** Description: Perform any post-NCI reset routines
**
** Returns:     none
**
*******************************************************************************/
extern "C" void nfa_app_post_nci_reset (void)
{
    ALOGD("nfa_app_post_nci_reset");

#if (defined(NFA_APP_RECONFIG_BAUD) && (NFA_APP_RECONFIG_BAUD != 0))
    NFC_UpdateBaudRate (NFA_APP_RECONFIG_BAUD, postBaudUpdate);
#else
    postBaudUpdate(NFC_STATUS_OK);
#endif
}

