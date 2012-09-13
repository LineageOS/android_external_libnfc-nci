/****************************************************************************
**
**  Name:       bbuildcfg.h
**
**  Function:   this file contains constant definitions for customizing NFA
**
**
**  Copyright (c) 1999-2011, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#pragma once
#include <string.h>
#include <memory.h>
#include <stdio.h>
#include "data_types.h"
#include "btdisp_lock.h"


#define BTE_APPL_MAX_USERIAL_DEV_NAME           (256)

#ifdef	__cplusplus
extern "C" {
#endif


extern UINT8 *scru_dump_hex (UINT8 *p, char *p_title, UINT32 len, UINT32 trace_layer, UINT32 trace_type);


#ifdef	__cplusplus
};
#endif

