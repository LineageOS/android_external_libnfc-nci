/****************************************************************************
**
**  Name:       android_logmsg.cpp
**
**  Function    This file contains helper functions for android logging
**
**  Copyright (c) 2011-2012, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#include "buildcfg.h"
#include <cutils/log.h>


#ifndef BTE_LOG_BUF_SIZE
    #define BTE_LOG_BUF_SIZE  1024
#endif
#define BTE_LOG_MAX_SIZE  (BTE_LOG_BUF_SIZE - 12)


extern "C"
{
    void LogMsg (UINT32 trace_set_mask, const char *fmt_str, ...);
}

/*******************************************************************************
**
** Function:    ScrLog
**
** Description: log a message
**
** Returns:     none
**
*******************************************************************************/
void ScrLog (UINT32 trace_set_mask, const char *fmt_str, ...)
{
    static char buffer[BTE_LOG_BUF_SIZE];
    va_list ap;

    va_start(ap, fmt_str);
    vsnprintf(buffer, BTE_LOG_MAX_SIZE, fmt_str, ap);
    va_end(ap);
    __android_log_write(ANDROID_LOG_INFO, "BrcmNci", buffer);
}


/*******************************************************************************
**
** Function:    LogMsg
**
** Description: log a message
**
** Returns:     none
**
*******************************************************************************/
void LogMsg (UINT32 trace_set_mask, const char *fmt_str, ...)
{
    static char buffer[BTE_LOG_BUF_SIZE];
    va_list ap;

    va_start(ap, fmt_str);
    vsnprintf(buffer, BTE_LOG_MAX_SIZE, fmt_str, ap);
    va_end(ap);
    __android_log_write(ANDROID_LOG_INFO, "BrcmNfcNfa", buffer);
}

