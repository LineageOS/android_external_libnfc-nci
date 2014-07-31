/*
 * Copyright (C) 2012-2014 NXP Semiconductors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <sys/stat.h>
#include <phNxpNciHal.h>
#include <phNxpNciHal_ext.h>
#include <phNxpNciHal_Dnld.h>
#include <phNxpNciHal_Adaptation.h>
#include <phTmlNfc.h>
#include <phDnldNfc.h>
#include <phDal4Nfc_messageQueueLib.h>
#include <phNxpLog.h>
#include <phNxpConfig.h>
#include <phNxpNciHal_NfcDepSWPrio.h>
#include <phNxpNciHal_Kovio.h>
/*********************** Global Variables *************************************/
#define PN547C2_CLOCK_SETTING
#undef  PN547C2_FACTORY_RESET_DEBUG
#define CORE_RES_STATUS_BYTE 3
/* Processing of ISO 15693 EOF */
extern uint8_t icode_send_eof;
static uint8_t cmd_icode_eof[] = { 0x00, 0x00, 0x00 };

/* FW download success flag */
static uint8_t fw_download_success = 0;

/* NCI HAL Control structure */
phNxpNciHal_Control_t nxpncihal_ctrl;

/* NXP Poll Profile structure */
phNxpNciProfile_Control_t nxpprofile_ctrl;

/* TML Context */
extern phTmlNfc_Context_t *gpphTmlNfc_Context;
extern void phTmlNfc_set_fragmentation_enabled(phTmlNfc_i2cfragmentation_t result);
/* global variable to get FW version from NCI response*/
uint32_t wFwVerRsp;
/* External global variable to get FW version */
extern uint16_t wFwVer;
extern int send_to_upper_kovio;
extern int kovio_detected;
extern int disable_kovio;
static uint8_t Rx_data[NCI_MAX_DATA_LEN];

uint32_t timeoutTimerId = 0;

/**************** local methods used in this file only ************************/
static NFCSTATUS phNxpNciHal_fw_download(void);
static void phNxpNciHal_open_complete(NFCSTATUS status);
static void phNxpNciHal_write_complete(void *pContext, phTmlNfc_TransactInfo_t *pInfo);
static void phNxpNciHal_read_complete(void *pContext, phTmlNfc_TransactInfo_t *pInfo);
static void phNxpNciHal_close_complete(NFCSTATUS status);
static void phNxpNciHal_core_initialized_complete(NFCSTATUS status);
static void phNxpNciHal_pre_discover_complete(NFCSTATUS status);
static void phNxpNciHal_power_cycle_complete(NFCSTATUS status);
static void phNxpNciHal_kill_client_thread(phNxpNciHal_Control_t *p_nxpncihal_ctrl);
static void *phNxpNciHal_client_thread(void *arg);
static void phNxpNciHal_get_clk_freq(void);
static void phNxpNciHal_set_clock(void);
static void phNxpNciHal_check_factory_reset(void);
static void phNxpNciHal_print_res_status( uint8_t *p_rx_data);
static NFCSTATUS phNxpNciHal_CheckValidFwVersion(void);
static void phNxpNciHal_enable_i2c_fragmentation();
/******************************************************************************
 * Function         phNxpNciHal_client_thread
 *
 * Description      This function is a thread handler which handles all TML and
 *                  NCI messages.
 *
 * Returns          void
 *
 ******************************************************************************/
static void *phNxpNciHal_client_thread(void *arg)
{
    phNxpNciHal_Control_t *p_nxpncihal_ctrl = (phNxpNciHal_Control_t *) arg;
    phLibNfc_Message_t msg;

    NXPLOG_NCIHAL_D("thread started");

    p_nxpncihal_ctrl->thread_running = 1;

    while (p_nxpncihal_ctrl->thread_running == 1)
    {
        /* Fetch next message from the NFC stack message queue */
        if (phDal4Nfc_msgrcv(p_nxpncihal_ctrl->gDrvCfg.nClientId,
                &msg, 0, 0) == -1)
        {
            NXPLOG_NCIHAL_E("NFC client received bad message");
            continue;
        }

        if(p_nxpncihal_ctrl->thread_running == 0){
            break;
        }

        switch (msg.eMsgType)
        {
            case PH_LIBNFC_DEFERREDCALL_MSG:
            {
                phLibNfc_DeferredCall_t *deferCall =
                        (phLibNfc_DeferredCall_t *) (msg.pMsgData);

                REENTRANCE_LOCK();
                deferCall->pCallback(deferCall->pParameter);
                REENTRANCE_UNLOCK();

            break;
        }

        case NCI_HAL_OPEN_CPLT_MSG:
        {
            REENTRANCE_LOCK();
            if (nxpncihal_ctrl.p_nfc_stack_cback != NULL)
            {
                /* Send the event */
                (*nxpncihal_ctrl.p_nfc_stack_cback)(HAL_NFC_OPEN_CPLT_EVT,
                        HAL_NFC_STATUS_OK);
            }
            REENTRANCE_UNLOCK();
            break;
        }

        case NCI_HAL_CLOSE_CPLT_MSG:
        {
            REENTRANCE_LOCK();
            if (nxpncihal_ctrl.p_nfc_stack_cback != NULL)
            {
                /* Send the event */
                (*nxpncihal_ctrl.p_nfc_stack_cback)(HAL_NFC_CLOSE_CPLT_EVT,
                        HAL_NFC_STATUS_OK);
                phNxpNciHal_kill_client_thread(&nxpncihal_ctrl);
            }
            REENTRANCE_UNLOCK();
            break;
        }

        case NCI_HAL_POST_INIT_CPLT_MSG:
        {
            REENTRANCE_LOCK();
            if (nxpncihal_ctrl.p_nfc_stack_cback != NULL)
            {
                /* Send the event */
                (*nxpncihal_ctrl.p_nfc_stack_cback)(HAL_NFC_POST_INIT_CPLT_EVT,
                        HAL_NFC_STATUS_OK);
            }
            REENTRANCE_UNLOCK();
            break;
        }

        case NCI_HAL_PRE_DISCOVER_CPLT_MSG:
        {
            REENTRANCE_LOCK();
            if (nxpncihal_ctrl.p_nfc_stack_cback != NULL)
            {
                /* Send the event */
                (*nxpncihal_ctrl.p_nfc_stack_cback)(
                        HAL_NFC_PRE_DISCOVER_CPLT_EVT, HAL_NFC_STATUS_OK);
            }
            REENTRANCE_UNLOCK();
            break;
        }

        case NCI_HAL_ERROR_MSG:
        {
            REENTRANCE_LOCK();
            if (nxpncihal_ctrl.p_nfc_stack_cback != NULL)
            {
                /* Send the event */
                (*nxpncihal_ctrl.p_nfc_stack_cback)(HAL_NFC_ERROR_EVT,
                        HAL_NFC_STATUS_FAILED);
            }
            REENTRANCE_UNLOCK();
            break;
        }

        case NCI_HAL_RX_MSG:
        {
            REENTRANCE_LOCK();
            if (nxpncihal_ctrl.p_nfc_stack_data_cback != NULL)
            {
                (*nxpncihal_ctrl.p_nfc_stack_data_cback)(
                        nxpncihal_ctrl.rsp_len, nxpncihal_ctrl.p_rsp_data);
            }
            REENTRANCE_UNLOCK();
            break;
        }
        }
    }

    NXPLOG_NCIHAL_D("NxpNciHal thread stopped");

    return NULL;
}

/******************************************************************************
 * Function         phNxpNciHal_kill_client_thread
 *
 * Description      This function safely kill the client thread and clean all
 *                  resources.
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpNciHal_kill_client_thread(phNxpNciHal_Control_t *p_nxpncihal_ctrl)
{
    NXPLOG_NCIHAL_D("Terminating phNxpNciHal client thread...");

    p_nxpncihal_ctrl->p_nfc_stack_cback = NULL;
    p_nxpncihal_ctrl->p_nfc_stack_data_cback = NULL;
    p_nxpncihal_ctrl->thread_running = 0;

    return;
}

/******************************************************************************
 * Function         phNxpNciHal_fw_download
 *
 * Description      This function download the PN547 secure firmware to IC. If
 *                  firmware version in Android filesystem and firmware in the
 *                  IC is same then firmware download will return with success
 *                  without downloading the firmware.
 *
 * Returns          NFCSTATUS_SUCCESS if firmware download successful
 *                  NFCSTATUS_FAILED in case of failure
 *
 ******************************************************************************/
static NFCSTATUS phNxpNciHal_fw_download(void)
{
    NFCSTATUS status = NFCSTATUS_FAILED;

    phNxpNciHal_get_clk_freq();
    status = phTmlNfc_IoCtl(phTmlNfc_e_EnableDownloadMode);
    if (NFCSTATUS_SUCCESS == status)
    {
        /* Set the obtained device handle to download module */
        phDnldNfc_SetHwDevHandle();
        NXPLOG_NCIHAL_D("Calling Seq handler for FW Download \n");
        status = phNxpNciHal_fw_download_seq(nxpprofile_ctrl.bClkSrcVal, nxpprofile_ctrl.bClkFreqVal);
        phDnldNfc_ReSetHwDevHandle();
    }
    else
    {
        status = NFCSTATUS_FAILED;
    }

    return status;
}

/******************************************************************************
 * Function         phNxpNciHal_CheckValidFwVersion
 *
 * Description      This function checks the valid FW for Mobile device.
 *                  If the FW doesn't belong the Mobile device it further
 *                  checks nxp config file to override.
 *
 * Returns          NFCSTATUS_SUCCESS if valid fw version found
 *                  NFCSTATUS_NOT_ALLOWED in case of FW not valid for mobile
 *                  device
 *
 ******************************************************************************/
static NFCSTATUS phNxpNciHal_CheckValidFwVersion(void)
{
    NFCSTATUS status = NFCSTATUS_NOT_ALLOWED;
    const unsigned char sfw_mobile_major_no = 0x01;
    const unsigned char sfw_infra_major_no = 0x02;
    unsigned char ufw_current_major_no = 0x00;
    unsigned long num = 0;
    int isfound = 0;

    /* extract the firmware's major no */
    ufw_current_major_no = ((0x00FF) & (wFwVer >> 8U));

    NXPLOG_NCIHAL_D("%s current_major_no = 0x%x", __FUNCTION__,ufw_current_major_no );
    if ( ufw_current_major_no == sfw_mobile_major_no)
    {
        status = NFCSTATUS_SUCCESS;
    }
    else if (ufw_current_major_no == sfw_infra_major_no)
    {
        /* Check the nxp config file if still want to go for download */
        /* By default NAME_NXP_FW_PROTECION_OVERRIDE will not be defined in config file.
           If user really want to override the Infra firmware over mobile firmware, please
           put "NXP_FW_PROTECION_OVERRIDE=0x01" in libnfc-nxp.conf file.
           Please note once Infra firmware downloaded to Mobile device, The device
           can never be updated to Mobile firmware*/
        isfound = GetNxpNumValue(NAME_NXP_FW_PROTECION_OVERRIDE, &num, sizeof(num));
        if (isfound > 0)
        {
            if (num == 0x01)
            {
                NXPLOG_NCIHAL_D("Override Infra FW over Mobile");
                status = NFCSTATUS_SUCCESS;
            }
            else
            {
                NXPLOG_NCIHAL_D("Firmware download not allowed (NXP_FW_PROTECION_OVERRIDE invalid value)");
            }
        }
        else
        {
            NXPLOG_NCIHAL_D("Firmware download not allowed (NXP_FW_PROTECION_OVERRIDE not defiend)");
        }
    }
    else
    {
        NXPLOG_NCIHAL_E("Wrong FW Version >>> Firmware download not allowed");
    }

    return status;
}

static void phNxpNciHal_get_clk_freq(void)
{
    unsigned long num = 0;
    int isfound = 0;

    nxpprofile_ctrl.bClkSrcVal = 0;
    nxpprofile_ctrl.bClkFreqVal = 0;
    nxpprofile_ctrl.bTimeout = 0;

    isfound = GetNxpNumValue(NAME_NXP_SYS_CLK_SRC_SEL, &num, sizeof(num));
    if (isfound > 0)
    {
        nxpprofile_ctrl.bClkSrcVal = num;
    }

    num = 0;
    isfound = 0;
    isfound = GetNxpNumValue(NAME_NXP_SYS_CLK_FREQ_SEL, &num, sizeof(num));
    if (isfound > 0)
    {
        nxpprofile_ctrl.bClkFreqVal = num;
    }

    num = 0;
    isfound = 0;
    isfound = GetNxpNumValue(NAME_NXP_SYS_CLOCK_TO_CFG, &num, sizeof(num));
    if (isfound > 0)
    {
        nxpprofile_ctrl.bTimeout = num;
    }

    NXPLOG_FWDNLD_D("gphNxpNciHal_fw_IoctlCtx.bClkSrcVal = 0x%x", nxpprofile_ctrl.bClkSrcVal);
    NXPLOG_FWDNLD_D("gphNxpNciHal_fw_IoctlCtx.bClkFreqVal = 0x%x", nxpprofile_ctrl.bClkFreqVal);
    NXPLOG_FWDNLD_D("gphNxpNciHal_fw_IoctlCtx.bClkFreqVal = 0x%x", nxpprofile_ctrl.bTimeout);

    if ((nxpprofile_ctrl.bClkSrcVal < CLK_SRC_XTAL) ||
            (nxpprofile_ctrl.bClkSrcVal > CLK_SRC_PLL))
    {
        NXPLOG_FWDNLD_E("Clock source value is wrong in config file, setting it as default");
        nxpprofile_ctrl.bClkSrcVal = NXP_SYS_CLK_SRC_SEL;
    }
    if ((nxpprofile_ctrl.bClkFreqVal < CLK_FREQ_13MHZ) ||
            (nxpprofile_ctrl.bClkFreqVal > CLK_FREQ_52MHZ))
    {
        NXPLOG_FWDNLD_E("Clock frequency value is wrong in config file, setting it as default");
        nxpprofile_ctrl.bClkFreqVal = NXP_SYS_CLK_FREQ_SEL;
    }
    if ((nxpprofile_ctrl.bTimeout < CLK_TO_CFG_DEF) || (nxpprofile_ctrl.bTimeout > CLK_TO_CFG_MAX))
    {
        NXPLOG_FWDNLD_E("Clock timeout value is wrong in config file, setting it as default");
        nxpprofile_ctrl.bTimeout = CLK_TO_CFG_DEF;
    }

}

/******************************************************************************
 * Function         phNxpNciHal_open
 *
 * Description      This function is called by libnfc-nci during the
 *                  initialization of the NFCC. It opens the physical connection
 *                  with NFCC (pn547) and creates required client thread for
 *                  operation.
 *                  After open is complete, status is informed to libnfc-nci
 *                  through callback function.
 *
 * Returns          This function return NFCSTATUS_SUCCES (0) in case of success
 *                  In case of failure returns other failure value.
 *
 ******************************************************************************/
int phNxpNciHal_open(nfc_stack_callback_t *p_cback, nfc_stack_data_callback_t *p_data_cback)
{
    phOsalNfc_Config_t tOsalConfig;
    phTmlNfc_Config_t tTmlConfig;
    NFCSTATUS wConfigStatus = NFCSTATUS_SUCCESS;
    NFCSTATUS status = NFCSTATUS_SUCCESS;
    /*NCI_INIT_CMD*/
    static uint8_t cmd_init_nci[] = {0x20,0x01,0x00};
    /*NCI_RESET_CMD*/
    static uint8_t cmd_reset_nci[] = {0x20,0x00,0x01,0x01};
    /* reset config cache */
    resetNxpConfig();

    /* initialize trace level */
    phNxpLog_InitializeLogLevel();

    /*Create the timer for extns write response*/
    timeoutTimerId = phOsalNfc_Timer_Create();

    if (phNxpNciHal_init_monitor() == NULL)
    {
        NXPLOG_NCIHAL_E("Init monitor failed");
        return NFCSTATUS_FAILED;
    }

    CONCURRENCY_LOCK();

    memset(&nxpncihal_ctrl, 0x00, sizeof(nxpncihal_ctrl));
    memset(&tOsalConfig, 0x00, sizeof(tOsalConfig));
    memset(&tTmlConfig, 0x00, sizeof(tTmlConfig));
    memset (&nxpprofile_ctrl, 0, sizeof(phNxpNciProfile_Control_t));

    /* By default HAL status is HAL_STATUS_OPEN */
    nxpncihal_ctrl.halStatus = HAL_STATUS_OPEN;

    nxpncihal_ctrl.p_nfc_stack_cback = p_cback;
    nxpncihal_ctrl.p_nfc_stack_data_cback = p_data_cback;

    /* Configure hardware link */
    nxpncihal_ctrl.gDrvCfg.nClientId = phDal4Nfc_msgget(0, 0600);
    nxpncihal_ctrl.gDrvCfg.nLinkType = ENUM_LINK_TYPE_I2C;/* For PN547 */
    tTmlConfig.pDevName = (int8_t *) "/dev/pn544";
    tOsalConfig.dwCallbackThreadId
    = (uint32_t) nxpncihal_ctrl.gDrvCfg.nClientId;
    tOsalConfig.pLogFile = NULL;
    tTmlConfig.dwGetMsgThreadId = (uint32_t) nxpncihal_ctrl.gDrvCfg.nClientId;

    /* Initialize TML layer */
    wConfigStatus = phTmlNfc_Init(&tTmlConfig);
    if (wConfigStatus != NFCSTATUS_SUCCESS)
    {
        NXPLOG_NCIHAL_E("phTmlNfc_Init Failed");
        goto clean_and_return;
    }

    /* Create the client thread */
    if (pthread_create(&nxpncihal_ctrl.client_thread, NULL,
            phNxpNciHal_client_thread, &nxpncihal_ctrl) != 0)
    {
        NXPLOG_NCIHAL_E("pthread_create failed");
        wConfigStatus = phTmlNfc_Shutdown();
        goto clean_and_return;
    }

    CONCURRENCY_UNLOCK();

    /* call read pending */
    status = phTmlNfc_Read(
            nxpncihal_ctrl.p_cmd_data,
            NCI_MAX_DATA_LEN,
            (pphTmlNfc_TransactCompletionCb_t) &phNxpNciHal_read_complete,
            NULL);
    if (status != NFCSTATUS_PENDING)
    {
        NXPLOG_NCIHAL_E("TML Read status error status = %x", status);
        wConfigStatus = phTmlNfc_Shutdown();
        wConfigStatus = NFCSTATUS_FAILED;
        goto clean_and_return;
    }

    phNxpNciHal_ext_init();

    status = phNxpNciHal_send_ext_cmd(sizeof(cmd_reset_nci),cmd_reset_nci);
    if((status != NFCSTATUS_SUCCESS) && (nxpncihal_ctrl.retry_cnt >= MAX_RETRY_COUNT))
    {
        NXPLOG_NCIHAL_E("Force FW Download, NFCC not coming out from Standby");
        wConfigStatus = NFCSTATUS_FAILED;
        goto force_download;
    }
    else if(status != NFCSTATUS_SUCCESS)
    {
        NXPLOG_NCIHAL_E ("NCI_CORE_RESET: Failed");
        wConfigStatus = phTmlNfc_Shutdown();
        wConfigStatus = NFCSTATUS_FAILED;
        goto clean_and_return;
    }

    status = phNxpNciHal_send_ext_cmd(sizeof(cmd_init_nci),cmd_init_nci);
    if(status != NFCSTATUS_SUCCESS)
    {
        NXPLOG_NCIHAL_E ("NCI_CORE_INIT : Failed");
        wConfigStatus = phTmlNfc_Shutdown();
        wConfigStatus = NFCSTATUS_FAILED;
        goto clean_and_return;
    }
    phNxpNciHal_enable_i2c_fragmentation();
    /*Get FW version from device*/
    status = phDnldNfc_InitImgInfo();
    NXPLOG_NCIHAL_D ("FW version for FW file = 0x%x", wFwVer);
    NXPLOG_NCIHAL_D ("FW version from device = 0x%x", wFwVerRsp);
    if ((wFwVerRsp & 0x0000FFFF) == wFwVer)
    {
        NXPLOG_NCIHAL_D ("FW uptodate not required");
        phDnldNfc_ReSetHwDevHandle();
    }
    else if (wFwVer != 0 && (wFwVerRsp & 0x0000FFFF) > wFwVer)
    {
        NXPLOG_NCIHAL_D ("FW image older than device's, skip update");
        phDnldNfc_ReSetHwDevHandle();
    }
    else
    {
force_download:
        if (NFCSTATUS_SUCCESS == phNxpNciHal_CheckValidFwVersion())
        {
            NXPLOG_NCIHAL_D ("FW update required");
            fw_download_success = 0;
            status = phNxpNciHal_fw_download();
            if (status != NFCSTATUS_SUCCESS)
            {
                NXPLOG_NCIHAL_E ("FW Download failed - NFCC init will continue");
            }
            else
            {
                wConfigStatus = NFCSTATUS_SUCCESS;
                fw_download_success = 1;
                /* call read pending */
                status = phTmlNfc_Read(
                        nxpncihal_ctrl.p_cmd_data,
                        NCI_MAX_DATA_LEN,
                        (pphTmlNfc_TransactCompletionCb_t) &phNxpNciHal_read_complete,
                        NULL);
                if (status != NFCSTATUS_PENDING)
                {
                    NXPLOG_NCIHAL_E("TML Read status error status = %x", status);
                    wConfigStatus = phTmlNfc_Shutdown();
                    wConfigStatus = NFCSTATUS_FAILED;
                    goto clean_and_return;
                }
            }
        }
    }
    /* Call open complete */
    phNxpNciHal_open_complete(wConfigStatus);

    return wConfigStatus;

    clean_and_return:
    CONCURRENCY_UNLOCK();
    /* Report error status */
    (*nxpncihal_ctrl.p_nfc_stack_cback)(HAL_NFC_OPEN_CPLT_EVT,
            HAL_NFC_STATUS_FAILED);

    nxpncihal_ctrl.p_nfc_stack_cback = NULL;
    nxpncihal_ctrl.p_nfc_stack_data_cback = NULL;
    phNxpNciHal_cleanup_monitor();
    nxpncihal_ctrl.halStatus = HAL_STATUS_CLOSE;
    return NFCSTATUS_FAILED;
}

/******************************************************************************
 * Function         phNxpNciHal_open_complete
 *
 * Description      This function inform the status of phNxpNciHal_open
 *                  function to libnfc-nci.
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpNciHal_open_complete(NFCSTATUS status)
{
    static phLibNfc_Message_t msg;

    if (status == NFCSTATUS_SUCCESS)
    {
        msg.eMsgType = NCI_HAL_OPEN_CPLT_MSG;
        nxpncihal_ctrl.hal_open_status = TRUE;
    }
    else
    {
        msg.eMsgType = NCI_HAL_ERROR_MSG;
    }

    msg.pMsgData = NULL;
    msg.Size = 0;

    phTmlNfc_DeferredCall(gpphTmlNfc_Context->dwCallbackThreadId,
            (phLibNfc_Message_t *) &msg);

    return;
}

/******************************************************************************
 * Function         phNxpNciHal_write
 *
 * Description      This function write the data to NFCC through physical
 *                  interface (e.g. I2C) using the pn547 driver interface.
 *                  Before sending the data to NFCC, phNxpNciHal_write_ext
 *                  is called to check if there is any extension processing
 *                  is required for the NCI packet being sent out.
 *
 * Returns          It returns number of bytes successfully written to NFCC.
 *
 ******************************************************************************/
int phNxpNciHal_write(uint16_t data_len, const uint8_t *p_data)
{
    NFCSTATUS status = NFCSTATUS_FAILED;
    static phLibNfc_Message_t msg;

    /* Create local copy of cmd_data */
    memcpy(nxpncihal_ctrl.p_cmd_data, p_data, data_len);
    nxpncihal_ctrl.cmd_len = data_len;

#ifdef P2P_PRIO_LOGIC_HAL_IMP
    /* Specific logic to block RF disable when P2P priority logic is busy */
    if (p_data[0] == 0x21&&
        p_data[1] == 0x06 &&
        p_data[2] == 0x01 &&
        EnableP2P_PrioLogic == TRUE)
    {
        NXPLOG_NCIHAL_D ("P2P priority logic busy: Disable it.");
        phNxpNciHal_clean_P2P_Prio();
    }
#endif
    /* Specific logic to block RF disable when Kovio detection logic is active */
    if (p_data[0] == 0x21&&
        p_data[1] == 0x06 &&
        p_data[2] == 0x01 &&
        kovio_detected == TRUE)
    {
        NXPLOG_NCIHAL_D ("Kovio detection logic is active: Set Flag to disable it.");
        disable_kovio=0x01;
    }

    /* Check for NXP ext before sending write */
    status = phNxpNciHal_write_ext(&nxpncihal_ctrl.cmd_len,
            nxpncihal_ctrl.p_cmd_data, &nxpncihal_ctrl.rsp_len,
            nxpncihal_ctrl.p_rsp_data);
    if (status != NFCSTATUS_SUCCESS)
    {
        /* Do not send packet to PN547, send response directly */
        msg.eMsgType = NCI_HAL_RX_MSG;
        msg.pMsgData = NULL;
        msg.Size = 0;

        phTmlNfc_DeferredCall(gpphTmlNfc_Context->dwCallbackThreadId,
                (phLibNfc_Message_t *) &msg);
        goto clean_and_return;
    }

    CONCURRENCY_LOCK();
    data_len = phNxpNciHal_write_unlocked(nxpncihal_ctrl.cmd_len,
            nxpncihal_ctrl.p_cmd_data);
    CONCURRENCY_UNLOCK();

    if (icode_send_eof == 1)
    {
        usleep (10000);
        icode_send_eof = 2;
        phNxpNciHal_send_ext_cmd (3, cmd_icode_eof);
    }

    clean_and_return:
    /* No data written */
    return data_len;
}

/******************************************************************************
 * Function         phNxpNciHal_write_unlocked
 *
 * Description      This is the actual function which is being called by
 *                  phNxpNciHal_write. This function writes the data to NFCC.
 *                  It waits till write callback provide the result of write
 *                  process.
 *
 * Returns          It returns number of bytes successfully written to NFCC.
 *
 ******************************************************************************/
int phNxpNciHal_write_unlocked(uint16_t data_len, const uint8_t *p_data)
{
    NFCSTATUS status = NFCSTATUS_INVALID_PARAMETER;
    phNxpNciHal_Sem_t cb_data;
    nxpncihal_ctrl.retry_cnt = 0;
    static uint8_t reset_ntf[] = {0x60, 0x00, 0x06, 0xA0, 0x00, 0xC7, 0xD4, 0x00, 0x00};

    /* Create the local semaphore */
    if (phNxpNciHal_init_cb_data(&cb_data, NULL) != NFCSTATUS_SUCCESS)
    {
        NXPLOG_NCIHAL_D("phNxpNciHal_write_unlocked Create cb data failed");
        data_len = 0;
        goto clean_and_return;
    }

    /* Create local copy of cmd_data */
    memcpy(nxpncihal_ctrl.p_cmd_data, p_data, data_len);
    nxpncihal_ctrl.cmd_len = data_len;

    retry:

    data_len = nxpncihal_ctrl.cmd_len;

    status = phTmlNfc_Write( (uint8_t *) nxpncihal_ctrl.p_cmd_data,
            (uint16_t) nxpncihal_ctrl.cmd_len,
            (pphTmlNfc_TransactCompletionCb_t) &phNxpNciHal_write_complete,
            (void *) &cb_data);
    if (status != NFCSTATUS_PENDING)
    {
        NXPLOG_NCIHAL_E("write_unlocked status error");
        data_len = 0;
        goto clean_and_return;
    }

    /* Wait for callback response */
    if (SEM_WAIT(cb_data))
    {
        NXPLOG_NCIHAL_E("write_unlocked semaphore error");
        data_len = 0;
        goto clean_and_return;
    }

    if (cb_data.status != NFCSTATUS_SUCCESS)
    {
        data_len = 0;
        if(nxpncihal_ctrl.retry_cnt++ < MAX_RETRY_COUNT)
        {
            NXPLOG_NCIHAL_E("write_unlocked failed - PN547 Maybe in Standby Mode - Retry");
            /* 1ms delay to give NFCC wake up delay */
            usleep(1000);
            goto retry;
        }
        else
        {

            NXPLOG_NCIHAL_E("write_unlocked failed - PN547 Maybe in Standby Mode (max count = 0x%x)", nxpncihal_ctrl.retry_cnt);

            status = phTmlNfc_IoCtl(phTmlNfc_e_ResetDevice);

            if(NFCSTATUS_SUCCESS == status)
            {
                NXPLOG_NCIHAL_D("PN547 Reset - SUCCESS\n");
            }
            else
            {
                NXPLOG_NCIHAL_D("PN547 Reset - FAILED\n");
            }
            if (nxpncihal_ctrl.p_nfc_stack_data_cback!= NULL &&
                nxpncihal_ctrl.p_rx_data!= NULL &&
                nxpncihal_ctrl.hal_open_status == TRUE)
            {
                NXPLOG_NCIHAL_D("Send the Core Reset NTF to upper layer, which will trigger the recovery\n");
                //Send the Core Reset NTF to upper layer, which will trigger the recovery.
                nxpncihal_ctrl.rx_data_len = sizeof(reset_ntf);
                memcpy(nxpncihal_ctrl.p_rx_data, reset_ntf, sizeof(reset_ntf));
                (*nxpncihal_ctrl.p_nfc_stack_data_cback)(nxpncihal_ctrl.rx_data_len, nxpncihal_ctrl.p_rx_data);
            }
        }
    }

    clean_and_return:
    phNxpNciHal_cleanup_cb_data(&cb_data);
    return data_len;
}

/******************************************************************************
 * Function         phNxpNciHal_write_complete
 *
 * Description      This function handles write callback.
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpNciHal_write_complete(void *pContext, phTmlNfc_TransactInfo_t *pInfo)
{
    phNxpNciHal_Sem_t *p_cb_data = (phNxpNciHal_Sem_t*) pContext;

    if (pInfo->wStatus == NFCSTATUS_SUCCESS)
    {
        NXPLOG_NCIHAL_D("write successful status = 0x%x", pInfo->wStatus);
    }
    else
    {
        NXPLOG_NCIHAL_E("write error status = 0x%x", pInfo->wStatus);
    }

    p_cb_data->status = pInfo->wStatus;

    SEM_POST(p_cb_data);

    return;
}

/******************************************************************************
 * Function         phNxpNciHal_read_complete
 *
 * Description      This function is called whenever there is an NCI packet
 *                  received from NFCC. It could be RSP or NTF packet. This
 *                  function provide the received NCI packet to libnfc-nci
 *                  using data callback of libnfc-nci.
 *                  There is a pending read called from each
 *                  phNxpNciHal_read_complete so each a packet received from
 *                  NFCC can be provide to libnfc-nci.
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpNciHal_read_complete(void *pContext, phTmlNfc_TransactInfo_t *pInfo)
{
    NFCSTATUS status = NFCSTATUS_FAILED;
    if(nxpncihal_ctrl.read_retry_cnt == 1)
    {
        nxpncihal_ctrl.read_retry_cnt = 0;
    }

    if (pInfo->wStatus == NFCSTATUS_SUCCESS)
    {
        NXPLOG_NCIHAL_D("read successful status = 0x%x", pInfo->wStatus);

        nxpncihal_ctrl.p_rx_data = pInfo->pBuff;
        nxpncihal_ctrl.rx_data_len = pInfo->wLength;

        status = phNxpNciHal_process_ext_rsp (nxpncihal_ctrl.p_rx_data, &nxpncihal_ctrl.rx_data_len);

        phNxpNciHal_print_res_status(nxpncihal_ctrl.p_rx_data);
        /* Check if response should go to hal module only */
        if (nxpncihal_ctrl.hal_ext_enabled == 1
                && (nxpncihal_ctrl.p_rx_data[0x00] & 0x40) == 0x40)
        {
            /* Unlock semaphore */
            SEM_POST(&(nxpncihal_ctrl.ext_cb_data));
        }
        /* Read successful send the event to higher layer */
        else if ((nxpncihal_ctrl.p_nfc_stack_data_cback != NULL) &&
                (status == NFCSTATUS_SUCCESS)&&(send_to_upper_kovio==1))
        {
            (*nxpncihal_ctrl.p_nfc_stack_data_cback)(
                    nxpncihal_ctrl.rx_data_len, nxpncihal_ctrl.p_rx_data);
        }
    }
    else
    {
        NXPLOG_NCIHAL_E("read error status = 0x%x", pInfo->wStatus);
    }

    if(nxpncihal_ctrl.halStatus == HAL_STATUS_CLOSE)
    {
        return;
    }
    /* Read again because read must be pending always.*/
    status = phTmlNfc_Read(
            Rx_data,
            NCI_MAX_DATA_LEN,
            (pphTmlNfc_TransactCompletionCb_t) &phNxpNciHal_read_complete,
            NULL);
    if (status != NFCSTATUS_PENDING)
    {
        NXPLOG_NCIHAL_E("read status error status = %x", status);
        /* TODO: Not sure how to handle this ? */
    }

    return;
}

void read_retry()
{
    /* Read again because read must be pending always.*/
    NFCSTATUS status = phTmlNfc_Read(
            Rx_data,
            NCI_MAX_DATA_LEN,
            (pphTmlNfc_TransactCompletionCb_t) &phNxpNciHal_read_complete,
            NULL);
    if (status != NFCSTATUS_PENDING)
    {
        NXPLOG_NCIHAL_E("read status error status = %x", status);
        /* TODO: Not sure how to handle this ? */
    }
}
/******************************************************************************
 * Function         phNxpNciHal_core_initialized
 *
 * Description      This function is called by libnfc-nci after successful open
 *                  of NFCC. All proprietary setting for PN547 are done here.
 *                  After completion of proprietary settings notification is
 *                  provided to libnfc-nci through callback function.
 *
 * Returns          Always returns NFCSTATUS_SUCCESS (0).
 *
 ******************************************************************************/
int phNxpNciHal_core_initialized(uint8_t* p_core_init_rsp_params)
{
    NFCSTATUS status = NFCSTATUS_SUCCESS;
    static uint8_t p2p_listen_mode_routing_cmd[] = { 0x21, 0x01, 0x07, 0x00, 0x01,
                                                0x01, 0x03, 0x00, 0x01, 0x05 };

    uint8_t swp_full_pwr_mode_on_cmd[] = { 0x20, 0x02, 0x05, 0x01, 0xA0,
                                           0xF1,0x01,0x01 };

    uint8_t *buffer = NULL;
    long bufflen = 260;
    long retlen = 0;
    int isfound;

    buffer = (uint8_t*) malloc(bufflen*sizeof(uint8_t));
    if(NULL == buffer)
    {
        return NFCSTATUS_FAILED;
    }

    retlen = 0;
    isfound = GetNxpByteArrayValue(NAME_NXP_ACT_PROP_EXTN, (char *) buffer,
            bufflen, &retlen);
    if (retlen > 0) {
        /* NXP ACT Proprietary Ext */
        status = phNxpNciHal_send_ext_cmd(retlen, buffer);
        if (status != NFCSTATUS_SUCCESS) {
            NXPLOG_NCIHAL_E("NXP ACT Proprietary Ext failed");
        }
    }
#ifdef PN547C2_CLOCK_SETTING
    if (isNxpConfigModified() || (fw_download_success == 1))
    {
        phNxpNciHal_get_clk_freq();  // Read the new values from Config file
        phNxpNciHal_set_clock();
    }
#endif
    phNxpNciHal_check_factory_reset();
    retlen = 0;
    isfound = GetNxpByteArrayValue(NAME_NXP_NFC_PROFILE_EXTN, (char *) buffer,
            bufflen, &retlen);
    if (retlen > 0) {
        /* NXP ACT Proprietary Ext */
        status = phNxpNciHal_send_ext_cmd(retlen, buffer);
        if (status != NFCSTATUS_SUCCESS) {
            NXPLOG_NCIHAL_E("NXP ACT Proprietary Ext failed");
        }
    }

    if(isNxpConfigModified() || (fw_download_success == 1))
    {
        retlen = 0;
        fw_download_success = 0;
        NXPLOG_NCIHAL_D ("Performing RF Settings BLK 1");
        isfound = GetNxpByteArrayValue(NAME_NXP_RF_CONF_BLK_1, (char *) buffer,
                bufflen, &retlen);
        if (retlen > 0) {
            status = phNxpNciHal_send_ext_cmd(retlen, buffer);
            if (status != NFCSTATUS_SUCCESS) {
                NXPLOG_NCIHAL_E("RF Settings BLK 1 failed");
            }
        }
        retlen = 0;

        NXPLOG_NCIHAL_D ("Performing RF Settings BLK 2");
        isfound = GetNxpByteArrayValue(NAME_NXP_RF_CONF_BLK_2, (char *) buffer,
                bufflen, &retlen);
        if (retlen > 0) {
            status = phNxpNciHal_send_ext_cmd(retlen, buffer);
            if (status != NFCSTATUS_SUCCESS) {
                NXPLOG_NCIHAL_E("RF Settings BLK 2 failed");
            }
        }
        retlen = 0;

        NXPLOG_NCIHAL_D ("Performing RF Settings BLK 3");
        isfound = GetNxpByteArrayValue(NAME_NXP_RF_CONF_BLK_3, (char *) buffer,
                bufflen, &retlen);
        if (retlen > 0) {
            status = phNxpNciHal_send_ext_cmd(retlen, buffer);
            if (status != NFCSTATUS_SUCCESS) {
                NXPLOG_NCIHAL_E("RF Settings BLK 3 failed");
            }
        }
        retlen = 0;

        NXPLOG_NCIHAL_D ("Performing RF Settings BLK 4");
        isfound = GetNxpByteArrayValue(NAME_NXP_RF_CONF_BLK_4, (char *) buffer,
                bufflen, &retlen);
        if (retlen > 0) {
            status = phNxpNciHal_send_ext_cmd(retlen, buffer);
            if (status != NFCSTATUS_SUCCESS) {
                NXPLOG_NCIHAL_E("RF Settings BLK 4 failed");
            }
        }
        retlen = 0;

        NXPLOG_NCIHAL_D ("Performing RF Settings BLK 5");
        isfound = GetNxpByteArrayValue(NAME_NXP_RF_CONF_BLK_5, (char *) buffer,
                bufflen, &retlen);
        if (retlen > 0) {
            status = phNxpNciHal_send_ext_cmd(retlen, buffer);
            if (status != NFCSTATUS_SUCCESS) {
                NXPLOG_NCIHAL_E("RF Settings BLK 5 failed");
            }
        }
        retlen = 0;

        NXPLOG_NCIHAL_D ("Performing RF Settings BLK 6");
        isfound = GetNxpByteArrayValue(NAME_NXP_RF_CONF_BLK_6, (char *) buffer,
                bufflen, &retlen);
        if (retlen > 0) {
            status = phNxpNciHal_send_ext_cmd(retlen, buffer);
            if (status != NFCSTATUS_SUCCESS) {
                NXPLOG_NCIHAL_E("RF Settings BLK 6 failed");
            }
        }
        retlen = 0;

        NXPLOG_NCIHAL_D ("Performing NAME_NXP_CORE_CONF_EXTN Settings");
        isfound = GetNxpByteArrayValue(NAME_NXP_CORE_CONF_EXTN,
                (char *) buffer, bufflen, &retlen);
        if (retlen > 0) {
            /* NXP ACT Proprietary Ext */
            status = phNxpNciHal_send_ext_cmd(retlen, buffer);
            if (status != NFCSTATUS_SUCCESS) {
                NXPLOG_NCIHAL_E("NXP Core configuration failed");
            }
        }

        retlen = 0;

        isfound = GetNxpByteArrayValue(NAME_NXP_CORE_MFCKEY_SETTING,
                (char *) buffer, bufflen, &retlen);
        if (retlen > 0) {
            /* NXP ACT Proprietary Ext */
            status = phNxpNciHal_send_ext_cmd(retlen, buffer);
            if (status != NFCSTATUS_SUCCESS) {
                NXPLOG_NCIHAL_E("Setting mifare keys failed");
            }
        }

        retlen = 0;

        isfound = GetNxpByteArrayValue(NAME_NXP_CORE_RF_FIELD,
                (char *) buffer, bufflen, &retlen);
        if (retlen > 0) {
            /* NXP ACT Proprietary Ext */
            status = phNxpNciHal_send_ext_cmd(retlen, buffer);
            if (status != NFCSTATUS_SUCCESS) {
                NXPLOG_NCIHAL_E("Setting NXP_CORE_RF_FIELD status failed");
            }
        }
    }

    retlen = 0;

    isfound = GetNxpByteArrayValue(NAME_NXP_CORE_STANDBY, (char *) buffer,bufflen, &retlen);
    if (retlen > 0) {
        /* NXP ACT Proprietary Ext */
        status = phNxpNciHal_send_ext_cmd(retlen, buffer);
        if (status != NFCSTATUS_SUCCESS) {
            NXPLOG_NCIHAL_E("Stand by mode enable failed");
        }
    }
    retlen = 0;

    isfound =  GetNxpByteArrayValue(NAME_NXP_CORE_CONF,(char *)buffer,bufflen,&retlen);
    if(retlen > 0)
    {
        /* NXP ACT Proprietary Ext */
        status = phNxpNciHal_send_ext_cmd(retlen,buffer);
        if (status != NFCSTATUS_SUCCESS)
        {
            NXPLOG_NCIHAL_E("Core Set Config failed");
        }
    }

    /* P2P listen mode routing */
    status = phNxpNciHal_send_ext_cmd (sizeof (p2p_listen_mode_routing_cmd), p2p_listen_mode_routing_cmd);
    if (status != NFCSTATUS_SUCCESS)
    {
        NXPLOG_NCIHAL_E("P2P listen mode routing failed");
    }

    retlen = 0;

    /* SWP FULL PWR MODE SETTING ON */
    if(GetNxpNumValue(NAME_NXP_SWP_FULL_PWR_ON, (void *)&retlen, sizeof(retlen)))
    {
        if(1 == retlen)
        {
            status = phNxpNciHal_send_ext_cmd (sizeof(swp_full_pwr_mode_on_cmd),
                                                      swp_full_pwr_mode_on_cmd);
            if (status != NFCSTATUS_SUCCESS)
            {
               NXPLOG_NCIHAL_E("SWP FULL PWR MODE SETTING ON CMD FAILED");
            }
        }
        else
        {
            swp_full_pwr_mode_on_cmd[7]=0x00;
            status = phNxpNciHal_send_ext_cmd (sizeof(swp_full_pwr_mode_on_cmd),
                                                      swp_full_pwr_mode_on_cmd);
            if (status != NFCSTATUS_SUCCESS)
            {
                NXPLOG_NCIHAL_E("SWP FULL PWR MODE SETTING OFF CMD FAILED");
            }
        }
    }
    if(buffer)
    {
        free(buffer);
    }
    phNxpNciHal_core_initialized_complete(status);

    updateNxpConfigTimestamp();
    return NFCSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpNciHal_core_initialized_complete
 *
 * Description      This function is called when phNxpNciHal_core_initialized
 *                  complete all proprietary command exchanges. This function
 *                  informs libnfc-nci about completion of core initialize
 *                  and result of that through callback.
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpNciHal_core_initialized_complete(NFCSTATUS status)
{
    static phLibNfc_Message_t msg;

    if (status == NFCSTATUS_SUCCESS)
    {
        msg.eMsgType = NCI_HAL_POST_INIT_CPLT_MSG;
    }
    else
    {
        msg.eMsgType = NCI_HAL_ERROR_MSG;
    }
    msg.pMsgData = NULL;
    msg.Size = 0;

    phTmlNfc_DeferredCall(gpphTmlNfc_Context->dwCallbackThreadId,
            (phLibNfc_Message_t *) &msg);

    return;
}

/******************************************************************************
 * Function         phNxpNciHal_pre_discover
 *
 * Description      This function is called by libnfc-nci to perform any
 *                  proprietary exchange before RF discovery. When proprietary
 *                  exchange is over completion is informed to libnfc-nci
 *                  through phNxpNciHal_pre_discover_complete function.
 *
 * Returns          It always returns NFCSTATUS_SUCCESS (0).
 *
 ******************************************************************************/
int phNxpNciHal_pre_discover(void)
{
    /* Nothing to do here for initial version */
    return NFCSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpNciHal_pre_discover_complete
 *
 * Description      This function informs libnfc-nci about completion and
 *                  status of phNxpNciHal_pre_discover through callback.
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpNciHal_pre_discover_complete(NFCSTATUS status)
{
    static phLibNfc_Message_t msg;

    if (status == NFCSTATUS_SUCCESS)
    {
        msg.eMsgType = NCI_HAL_PRE_DISCOVER_CPLT_MSG;
    }
    else
    {
        msg.eMsgType = NCI_HAL_ERROR_MSG;
    }
    msg.pMsgData = NULL;
    msg.Size = 0;

    phTmlNfc_DeferredCall(gpphTmlNfc_Context->dwCallbackThreadId,
            &msg);

    return;
}

/******************************************************************************
 * Function         phNxpNciHal_close
 *
 * Description      This function close the NFCC interface and free all
 *                  resources.This is called by libnfc-nci on NFC service stop.
 *
 * Returns          Always return NFCSTATUS_SUCCESS (0).
 *
 ******************************************************************************/
int phNxpNciHal_close(void)
{
    NFCSTATUS status;
    /*NCI_RESET_CMD*/
    static uint8_t cmd_reset_nci[] = {0x20,0x00,0x01,0x00};

    static uint8_t cmd_ce_disc_nci[] = {0x21,0x03,0x07,0x03,0x80,0x01,0x81,0x01,0x82,0x01};

    CONCURRENCY_LOCK();

    status = phNxpNciHal_send_ext_cmd(sizeof(cmd_ce_disc_nci),cmd_ce_disc_nci);
    if(status != NFCSTATUS_SUCCESS)
    {
        NXPLOG_NCIHAL_E ("CMD_CE_DISC_NCI: Failed");
    }

    nxpncihal_ctrl.halStatus = HAL_STATUS_CLOSE;

    status = phNxpNciHal_send_ext_cmd(sizeof(cmd_reset_nci),cmd_reset_nci);
    if(status != NFCSTATUS_SUCCESS)
    {
        NXPLOG_NCIHAL_E ("NCI_CORE_RESET: Failed");
    }

    if (NULL != gpphTmlNfc_Context->pDevHandle)
    {
        phNxpNciHal_close_complete(NFCSTATUS_SUCCESS);
        /* Abort any pending read and write */
        status = phTmlNfc_ReadAbort();
        status = phTmlNfc_WriteAbort();

        phOsalNfc_Timer_Cleanup();

        status = phTmlNfc_Shutdown();

        phDal4Nfc_msgrelease(nxpncihal_ctrl.gDrvCfg.nClientId);


        memset (&nxpncihal_ctrl, 0x00, sizeof (nxpncihal_ctrl));

        NXPLOG_NCIHAL_D("phNxpNciHal_close - phOsalNfc_DeInit completed");
    }

    CONCURRENCY_UNLOCK();

    phNxpNciHal_cleanup_monitor();

    /* Return success always */
    return NFCSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpNciHal_close_complete
 *
 * Description      This function inform libnfc-nci about result of
 *                  phNxpNciHal_close.
 *
 * Returns          void.
 *
 ******************************************************************************/
void phNxpNciHal_close_complete(NFCSTATUS status)
{
    static phLibNfc_Message_t msg;

    if (status == NFCSTATUS_SUCCESS)
    {
        msg.eMsgType = NCI_HAL_CLOSE_CPLT_MSG;
    }
    else
    {
        msg.eMsgType = NCI_HAL_ERROR_MSG;
    }
    msg.pMsgData = NULL;
    msg.Size = 0;

    phTmlNfc_DeferredCall(gpphTmlNfc_Context->dwCallbackThreadId,
            &msg);

    return;
}
/******************************************************************************
 * Function         phNxpNciHal_notify_i2c_fragmentation
 *
 * Description      This function can be used by HAL to inform
 *                 libnfc-nci that i2c fragmentation is enabled/disabled
 *
 * Returns          void.
 *
 ******************************************************************************/
void phNxpNciHal_notify_i2c_fragmentation(void)
{
    if (nxpncihal_ctrl.p_nfc_stack_cback != NULL)
    {
        /*inform libnfc-nci that i2c fragmentation is enabled/disabled */
        (*nxpncihal_ctrl.p_nfc_stack_cback)(HAL_NFC_ENABLE_I2C_FRAGMENTATION_EVT,
                HAL_NFC_STATUS_OK);
    }


}
/******************************************************************************
 * Function         phNxpNciHal_control_granted
 *
 * Description      Called by libnfc-nci when NFCC control is granted to HAL.
 *
 * Returns          Always returns NFCSTATUS_SUCCESS (0).
 *
 ******************************************************************************/
int phNxpNciHal_control_granted(void)
{
    /* Take the concurrency lock so no other calls from upper layer
     * will be allowed
     */
    CONCURRENCY_LOCK();

    if(NULL != nxpncihal_ctrl.p_control_granted_cback)
    {
        (*nxpncihal_ctrl.p_control_granted_cback)();
    }
    /* At the end concurrency unlock so calls from upper layer will
     * be allowed
     */
    CONCURRENCY_UNLOCK();
    return NFCSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpNciHal_request_control
 *
 * Description      This function can be used by HAL to request control of
 *                  NFCC to libnfc-nci. When control is provided to HAL it is
 *                  notified through phNxpNciHal_control_granted.
 *
 * Returns          void.
 *
 ******************************************************************************/
void phNxpNciHal_request_control(void)
{
    if (nxpncihal_ctrl.p_nfc_stack_cback != NULL)
    {
        /* Request Control of NCI Controller from NCI NFC Stack */
        (*nxpncihal_ctrl.p_nfc_stack_cback)(HAL_NFC_REQUEST_CONTROL_EVT,
                HAL_NFC_STATUS_OK);
    }

    return;
}

/******************************************************************************
 * Function         phNxpNciHal_release_control
 *
 * Description      This function can be used by HAL to release the control of
 *                  NFCC back to libnfc-nci.
 *
 * Returns          void.
 *
 ******************************************************************************/
void phNxpNciHal_release_control(void)
{
    if (nxpncihal_ctrl.p_nfc_stack_cback != NULL)
    {
        /* Release Control of NCI Controller to NCI NFC Stack */
        (*nxpncihal_ctrl.p_nfc_stack_cback)(HAL_NFC_RELEASE_CONTROL_EVT,
                HAL_NFC_STATUS_OK);
    }

    return;
}

/******************************************************************************
 * Function         phNxpNciHal_power_cycle
 *
 * Description      This function is called by libnfc-nci when power cycling is
 *                  performed. When processing is complete it is notified to
 *                  libnfc-nci through phNxpNciHal_power_cycle_complete.
 *
 * Returns          Always return NFCSTATUS_SUCCESS (0).
 *
 ******************************************************************************/
int phNxpNciHal_power_cycle(void)
{
    NXPLOG_NCIHAL_D("Power Cycle");

    NFCSTATUS status = NFCSTATUS_FAILED;

    status = phTmlNfc_IoCtl(phTmlNfc_e_ResetDevice);

    if(NFCSTATUS_SUCCESS == status)
    {
        NXPLOG_NCIHAL_D("PN547 Reset - SUCCESS\n");
    }
    else
    {
        NXPLOG_NCIHAL_D("PN547 Reset - FAILED\n");
    }

    phNxpNciHal_power_cycle_complete(NFCSTATUS_SUCCESS);

    return NFCSTATUS_SUCCESS;
}

/******************************************************************************
 * Function         phNxpNciHal_power_cycle_complete
 *
 * Description      This function is called to provide the status of
 *                  phNxpNciHal_power_cycle to libnfc-nci through callback.
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpNciHal_power_cycle_complete(NFCSTATUS status)
{
    static phLibNfc_Message_t msg;

    if (status == NFCSTATUS_SUCCESS)
    {
        msg.eMsgType = NCI_HAL_OPEN_CPLT_MSG;
    }
    else
    {
        msg.eMsgType = NCI_HAL_ERROR_MSG;
    }
    msg.pMsgData = NULL;
    msg.Size = 0;

    phTmlNfc_DeferredCall(gpphTmlNfc_Context->dwCallbackThreadId,
            &msg);

    return;
}

/******************************************************************************
 * Function         phNxpNciHal_set_clock
 *
 * Description      This function is called after successfull download
 *                  to apply the clock setting provided in config file
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpNciHal_set_clock(void)
{
    NFCSTATUS status = NFCSTATUS_FAILED;
    if (nxpprofile_ctrl.bClkSrcVal == CLK_SRC_PLL)
    {
        static uint8_t set_clock_cmd[] = {0x20, 0x02,0x09, 0x02, 0xA0, 0x03, 0x01, 0x11,
                                                               0xA0, 0x04, 0x01, 0x01};
        uint8_t param_clock_src = CLK_SRC_PLL;
        param_clock_src = param_clock_src << 3;

        if (nxpprofile_ctrl.bClkFreqVal == CLK_FREQ_13MHZ)
        {
            param_clock_src |= 0x00;
        }
        else if (nxpprofile_ctrl.bClkFreqVal == CLK_FREQ_19_2MHZ)
        {
            param_clock_src |= 0x01;
        }
        else if (nxpprofile_ctrl.bClkFreqVal == CLK_FREQ_24MHZ)
        {
            param_clock_src |= 0x02;
        }
        else if (nxpprofile_ctrl.bClkFreqVal == CLK_FREQ_26MHZ)
        {
            param_clock_src |= 0x03;
        }
        else if (nxpprofile_ctrl.bClkFreqVal == CLK_FREQ_38_4MHZ)
        {
            param_clock_src |= 0x04;
        }
        else if (nxpprofile_ctrl.bClkFreqVal == CLK_FREQ_52MHZ)
        {
            param_clock_src |= 0x05;
        }
        else
        {
            NXPLOG_NCIHAL_E("Wrong clock freq, send default PLL@19.2MHz");
            param_clock_src = 0x11;
        }

        set_clock_cmd[7] = param_clock_src;
        set_clock_cmd[11] = nxpprofile_ctrl.bTimeout;
        status = phNxpNciHal_send_ext_cmd(sizeof(set_clock_cmd), set_clock_cmd);
        if (status != NFCSTATUS_SUCCESS)
        {
            NXPLOG_NCIHAL_E("PLL colck setting failed !!");
        }
    }
    else if(nxpprofile_ctrl.bClkSrcVal == CLK_SRC_XTAL)
    {
        static uint8_t set_clock_cmd[] = {0x20, 0x02, 0x05, 0x01, 0xA0, 0x03, 0x01, 0x08};
        status = phNxpNciHal_send_ext_cmd(sizeof(set_clock_cmd), set_clock_cmd);
        if (status != NFCSTATUS_SUCCESS)
        {
            NXPLOG_NCIHAL_E("XTAL colck setting failed !!");
        }

    }
    else
    {
        NXPLOG_NCIHAL_E("Wrong clock source. Dont apply any modification")
    }
}
/******************************************************************************
 * Function         phNxpNciHal_enable_i2c_fragmentation
 *
 * Description      This function is called to process the response status
 *                  and print the status byte.
 *
 * Returns          void.
 *
 ******************************************************************************/
void phNxpNciHal_enable_i2c_fragmentation()
{
    NFCSTATUS status = NFCSTATUS_FAILED;
    static uint8_t fragmentation_enable_config_cmd[] = { 0x20, 0x02, 0x05, 0x01, 0xA0, 0x05, 0x01, 0x10};
    int isfound = 0;
    long i2c_status = 0x00;
    long config_i2c_vlaue = 0xff;
    /*NCI_RESET_CMD*/
    static uint8_t cmd_reset_nci[] = {0x20,0x00,0x01,0x01};
    /*NCI_INIT_CMD*/
    static uint8_t cmd_init_nci[] = {0x20,0x01,0x00};
    static uint8_t get_i2c_fragmentation_cmd[] = {0x20, 0x03, 0x03, 0x01 ,0xA0 ,0x05};
    isfound = (GetNxpNumValue(NAME_NXP_I2C_FRAGMENTATION_ENABLED, (void *)&i2c_status, sizeof(i2c_status)));
    status = phNxpNciHal_send_ext_cmd(sizeof(get_i2c_fragmentation_cmd),get_i2c_fragmentation_cmd);
    if(status != NFCSTATUS_SUCCESS)
    {
        NXPLOG_NCIHAL_E("unable to retrieve  get_i2c_fragmentation_cmd");
    }else
    {
        if(nxpncihal_ctrl.p_rx_data[8] == 0x10)
        {
            config_i2c_vlaue = 0x01;
            phNxpNciHal_notify_i2c_fragmentation();
            phTmlNfc_set_fragmentation_enabled(I2C_FRAGMENTATION_ENABLED);
        }else if(nxpncihal_ctrl.p_rx_data[8] == 0x00)
        {
            config_i2c_vlaue = 0x00;
        }
        if( config_i2c_vlaue == i2c_status)
        {
            NXPLOG_NCIHAL_E("i2c_fragmentation_status existing");
        }else {
            if (i2c_status == 0x01) {
                /* NXP I2C fragmenation enabled*/
                status = phNxpNciHal_send_ext_cmd(sizeof(fragmentation_enable_config_cmd), fragmentation_enable_config_cmd);
                if (status != NFCSTATUS_SUCCESS) {
                    NXPLOG_NCIHAL_E("NXP fragmentation enable failed");
                }
            }
            else if (i2c_status == 0x00 || config_i2c_vlaue == 0xff) {
                fragmentation_enable_config_cmd[7] = 0x00;
                /* NXP I2C fragmentation disabled*/
                status = phNxpNciHal_send_ext_cmd(sizeof(fragmentation_enable_config_cmd), fragmentation_enable_config_cmd);
                if (status != NFCSTATUS_SUCCESS) {
                    NXPLOG_NCIHAL_E("NXP fragmentation disable failed");
                }
            }
            status = phNxpNciHal_send_ext_cmd(sizeof(cmd_reset_nci),cmd_reset_nci);
            if(status != NFCSTATUS_SUCCESS)
            {
                NXPLOG_NCIHAL_E ("NCI_CORE_RESET: Failed");
            }

            status = phNxpNciHal_send_ext_cmd(sizeof(cmd_init_nci),cmd_init_nci);
            if(status != NFCSTATUS_SUCCESS)
            {
                NXPLOG_NCIHAL_E ("NCI_CORE_INIT : Failed");
            }else if(i2c_status == 0x01)
            {
                phNxpNciHal_notify_i2c_fragmentation();
                phTmlNfc_set_fragmentation_enabled(I2C_FRAGMENTATION_ENABLED);
            }
        }
    }
}
/******************************************************************************
 * Function         phNxpNciHal_check_factory_reset
 *
 * Description      This function is called at init time to check
 *                  the presence of ese related info. If file are not
 *                  present set the SWP_INT_SESSION_ID_CFG to FF to
 *                  force the NFCEE to re-run its initialization sequence.
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpNciHal_check_factory_reset(void)
{
    struct stat st;
    int ret = 0;
    NFCSTATUS status = NFCSTATUS_FAILED;
    const char config_eseinfo_path[] = "/data/nfc/nfaStorage.bin1";
    static uint8_t reset_ese_session_identity_set[] = { 0x20, 0x02, 0x17, 0x02,
                                      0xA0, 0xEA, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                                      0xA0, 0xEB, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
#ifdef PN547C2_FACTORY_RESET_DEBUG
    static uint8_t reset_ese_session_identity[] = { 0x20, 0x03, 0x05, 0x02,
                                          0xA0, 0xEA, 0xA0, 0xEB};
#endif
    if (stat(config_eseinfo_path, &st) == -1)
    {
        NXPLOG_NCIHAL_D("%s file not present = %s", __FUNCTION__, config_eseinfo_path);
        ret = -1;
    }
    else
    {
        ret = 0;
    }

    if(ret == -1)
    {
#ifdef PN547C2_FACTORY_RESET_DEBUG
        /* NXP ACT Proprietary Ext */
        status = phNxpNciHal_send_ext_cmd(sizeof(reset_ese_session_identity),
                                           reset_ese_session_identity);
        if (status != NFCSTATUS_SUCCESS) {
            NXPLOG_NCIHAL_E("NXP reset_ese_session_identity command failed");
        }
#endif
        status = phNxpNciHal_send_ext_cmd(sizeof(reset_ese_session_identity_set),
                                           reset_ese_session_identity_set);
        if (status != NFCSTATUS_SUCCESS) {
            NXPLOG_NCIHAL_E("NXP reset_ese_session_identity_set command failed");
        }
#ifdef PN547C2_FACTORY_RESET_DEBUG
        /* NXP ACT Proprietary Ext */
        status = phNxpNciHal_send_ext_cmd(sizeof(reset_ese_session_identity),
                                           reset_ese_session_identity);
        if (status != NFCSTATUS_SUCCESS) {
            NXPLOG_NCIHAL_E("NXP reset_ese_session_identity command failed");
        }
#endif

    }
}

/******************************************************************************
 * Function         phNxpNciHal_print_res_status
 *
 * Description      This function is called to process the response status
 *                  and print the status byte.
 *
 * Returns          void.
 *
 ******************************************************************************/
static void phNxpNciHal_print_res_status( uint8_t *p_rx_data)
{
    static uint8_t response_buf[][30] = {"STATUS_OK",
                                     "STATUS_REJECTED",
                                     "STATUS_RF_FRAME_CORRUPTED" ,
                                     "STATUS_FAILED" ,
                                     "STATUS_NOT_INITIALIZED" ,
                                     "STATUS_SYNTAX_ERROR",
                                     "STATUS_SEMANTIC_ERROR",
                                     "RFU",
                                     "RFU",
                                     "STATUS_INVALID_PARAM",
                                     "STATUS_MESSAGE_SIZE_EXCEEDED",
                                     "STATUS_UNDEFINED"};
    int status_byte;
    if(p_rx_data[0] == 0x40 && (p_rx_data[1] == 0x02 || p_rx_data[1] == 0x03))
    {
        if(p_rx_data[2] <= 10)
        {
            status_byte = p_rx_data[CORE_RES_STATUS_BYTE];
            NXPLOG_NCIHAL_D("%s: response status =%s",__FUNCTION__,response_buf[status_byte]);
        }
        else
        {
            NXPLOG_NCIHAL_D("%s: response status =%s",__FUNCTION__,response_buf[11]);
        }
    }
}
