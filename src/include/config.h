/****************************************************************************
**
**  Name:       config.h
**
**  Function:   this file contains constant definitions for config settings
**
**
**  Copyright (c) 1999-2011, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef __cplusplus
extern "C"
{
#endif

void readDefaultConfig();
int GetStrValue(const char* name, char* p_value, unsigned long len);
int GetNumValue(const char* name, void* p_value, unsigned long len);

#ifdef __cplusplus
};
#endif

#define NAME_NFA_DM_MULTI_TECH_RESP     "NFA_DM_MULTI_TECH_RESP"
#define NAME_USE_NXP_P2P_RC_WORKAROUND  "USE_NXP_P2P_RC_WORKAROUND"
#define NAME_JNI_VERSION                "JNI_VERSION"
#define NAME_NFA_DM_ENABLE_SLEEP        "NFA_DM_ENABLE_SLEEP"
#define NAME_ENABLE_BRCM_EXTRAS_API     "ENABLE_BRCM_EXTRAS_API"
#define NAME_POLLING_TECH_MASK          "POLLING_TECH_MASK"
#define NAME_REGISTER_VIRTUAL_SE        "REGISTER_VIRTUAL_SE"
#define NAME_APPL_TRACE_LEVEL           "APPL_TRACE_LEVEL"
#define NAME_LPTD_CFG                   "LPTD_CFG"
#define NAME_SCREEN_OFF_POWER_STATE     "SCREEN_OFF_POWER_STATE"
#define NAME_USE_VBAT_MONITOR_API       "SEND_VBAT_MONITOR_CMD"
#define NAME_VBAT_MONITOR_API_PARAM     "VBAT_MONITOR_THRESHOLD"
#define NAME_UICC_IDLE_TIMEOUT          "UICC_IDLE_TIMEOUT"
#define NAME_PREINIT_DSP_CFG            "PREINIT_DSP_CFG"
#define NAME_DTA_START_CFG              "DTA_START_CFG"

struct tUART_CONFIG {
    int     m_iBaudrate;            // 115200
    int     m_iDatabits;            // 8
    int     m_iParity;              // 0 - none, 1 = odd, 2 = even
    int     m_iStopbits;
};

extern struct tUART_CONFIG  uartConfig;

#endif
