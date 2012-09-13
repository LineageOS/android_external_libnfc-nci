/*****************************************************************************
**
**  Name:           HalAdaptation.h
**
**  Description:    HAL Adaptation Interface (HAI).
**                  This interface regulates the interaction between standard Android HAL
**                  and Broadcom-specific HAL.  It adapts Broadcom-specific features
**                  to the Android framework.
**
**  Copyright (c) 2012, Broadcom Corp., All Rights Reserved.
**  Proprietary and confidential.
**
*****************************************************************************/
#pragma once
#include <hardware/hardware.h>
#include <hardware/nfc.h>


#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
    struct nfc_nci_device nci_device;
    //below declarations are private variables within Broadcom HAL
    void* data;
}
bcm2079x_dev_t;


/*
All functions return POSIX error codes (see errno):
  0 means success.
  non-zero means failure; for example EACCES.
*/


extern int HaiInitializeLibrary (const bcm2079x_dev_t* device);
extern int HaiTerminateLibrary ();
extern int HaiOpen (const bcm2079x_dev_t* device, nfc_stack_callback_t* halCallbackFunc);
extern int HaiClose (const bcm2079x_dev_t* device);
extern int HaiCoreInitialized (const bcm2079x_dev_t* device, uint8_t* coreInitResponseParams);
extern int HaiWrite (const bcm2079x_dev_t* dev, uint16_t dataLen, const uint8_t* data);
extern int HaiPreDiscover (const bcm2079x_dev_t* device);
extern int HaiControlGranted (const bcm2079x_dev_t* device);
extern int HaiPowerCycle (const bcm2079x_dev_t* device);


#ifdef __cplusplus
}
#endif
