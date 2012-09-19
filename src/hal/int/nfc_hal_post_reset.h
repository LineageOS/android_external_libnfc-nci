/*****************************************************************************
**
**  Name:          nfc_hal_post_reset.h
**
**  Description:   Post NCI reset routines
**
**  Copyright (c) 2009-2012, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
****************************************************************************/
#ifndef NFC_HAL_POST_RESET_H
#define NFC_HAL_POST_RESET_H


/*****************************************************************************
** Application control block definitions
******************************************************************************/
#define NFA_APP_PATCHFILE_MAX_PATH          255

typedef struct
{
    UINT8 prm_file[NFA_APP_PATCHFILE_MAX_PATH+1];   /* Filename of patchram */
    UINT8 *p_prm_buf;                               /* Pointer to buffer for holding patchram data */

    /* Patchfile for I2C fix */
    UINT8 prm_i2c_patchfile[NFA_APP_PATCHFILE_MAX_PATH+1];
    UINT8 *p_prm_i2c_buf;

    UINT8 userial_baud;

    tNFC_HAL_DEV_INIT_CFG dev_init_config;

    /* snooze mode setting */
    UINT8 snooze_mode;
    UINT8 idle_threshold_dh;
    UINT8 idle_threshold_nfcc;
    UINT8 nfc_wake_active_mode;
    UINT8 dh_wake_active_mode;

} tNFC_POST_RESET_CB;
extern tNFC_POST_RESET_CB nfc_post_reset_cb;

/*
** Post NCI reset handler
**
** This function is called to start device pre-initialization after NCI CORE-RESET.
** When pre-initialization is completed,
** HAL_NfcPreInitDone() must be called to proceed with stack start up.
*/
void nfc_hal_post_reset_init (UINT32 brcm_hw_id, UINT8 nvm_type);


#endif  /* NFC_HAL_POST_RESET_H */
