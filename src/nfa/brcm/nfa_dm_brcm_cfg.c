/*****************************************************************************
**
**  Name:           nfa_dm_brcm_cfg.c
**
**  Description:    This file contains compile-time configurable constants
**                  for NFA modules
**
**  Copyright (c) 2011-2012, Broadcom Corp., All Rights Reserved.
**
*****************************************************************************/
#include "nfa_cho_api.h"
#include "nfa_brcm_api.h"



/* the SetConfig at start up*/
UINT8 nfa_start_up_cfg[] = {
    /* TLV len */   28,
    /* B0 */        NCI_PARAM_ID_EMVCO_ENABLE,
    /* B1 */        1,
    /* B2 */        1,     /* (1 = enable emvco mode, 0 = disable emvco mode) Default = 0.*/
    /* B3 */        NCI_PARAM_ID_CONTINUE_MODE, /* NFCC will restart discovery after deactivated */
    /* B4 */        1,
    /* B5 */        1,     /* (1 = enable, 0 = disable) Default = 0.*/
    /* B6 */        NCI_PARAM_ID_RFU_CONFIG,
    /* B7 */        0x14,
    /* B8 */        0x00,
    /* B9 */        0x00,
    /* B10*/        0x00,
    /* B11*/        0x00,
    /* B12*/        0x02,
    /* B13*/        0xE8,
    /* B14*/        0x03,
    /* B15*/        0x00,
    /* B16*/        0x00,
    /* B17*/        0x00,
    /* B18*/        0x00,
    /* B19*/        0x00,
    /* B20*/        0x00,
    /* B21*/        0x00,
    /* B22*/        0x00,
    /* B23*/        0x00,
    /* B24*/        0x00,
    /* B25*/        0x00,
    /* B26*/        0x00,
    /* B27*/        0x00
};

UINT8 *p_nfa_dm_start_up_cfg = (UINT8 *) nfa_start_up_cfg;

/* the VSCs at start up:
 * The VSCs are specified in TLV format similar to nfa_start_up_cfg[]
 * first byte is the TLV total len.
 * B0 is the first T; i.e. the opcode for the VSC
 * B1 is the len of the VSC parameters/payload
 * */
UINT8 nfa_dm_start_up_vsc_cfg[] = {
    /* TLV len */   4,
    /* B0 */        NCI_MSG_FRAME_LOG,
    /* B1 */        2,
    /* B2 */        0,  /* 1 to enable RF frames */
    /* B3 */        1   /* 1 to enable SWP frames */
};

UINT8 *p_nfa_dm_start_up_vsc_cfg = NULL;


const tNCI_DISCOVER_MAPS nfa_dm_brcm_interface_mapping[NFA_DM_NUM_INTERFACE_MAP] =
{
    /* Protocols that use Frame Interface do not need to be included in the interface mapping */
    {
        NCI_PROTOCOL_ISO_DEP,
        NCI_INTERFACE_MODE_POLL_N_LISTEN,
        NCI_INTERFACE_ISO_DEP
    },
    {
        NCI_PROTOCOL_NFC_DEP,
        NCI_INTERFACE_MODE_POLL_N_LISTEN,
        NCI_INTERFACE_NFC_DEP
    },
    {
        NCI_PROTOCOL_T2T,
        NCI_INTERFACE_MODE_LISTEN,
        NCI_INTERFACE_VS_T2T_CE
    }
};


const tNFA_DM_LP_CFG nfa_dm_lp_cfg =
{
    NFC_LP_SNOOZE_MODE,         /* Snooze Mode          */
    NFC_LP_IDLE_THRESHOLD_HOST, /* Idle Threshold Host  */
    NFC_LP_IDLE_THRESHOLD_HC,   /* Idle Threshold HC    */
    NFC_LP_ACTIVE_LOW,          /* NFC_WAKE Active Mode */
    NFC_LP_ACTIVE_HIGH,         /* DH_WAKE Active Mode  */
    NFC_LP_POWER_CYCLE_TO_FULL  /* Power cycle to full power mode from CEx */
};

tNFA_DM_LP_CFG *p_nfa_dm_lp_cfg = (tNFA_DM_LP_CFG *) &nfa_dm_lp_cfg;

/* LPTD parameters (LowPowerTagDetection) 
 * This is typical values for 20791B2
 * The timing and threshold parameters used for a customer handset/hardware may vary
 * depending on antenna and should be verified during a customer testing phase. 
 * the data fields without comments are too complicated. Please see ""
 * */
const UINT8 nfa_dm_lptd_cfg[] =
{
    19,             /* length */
    0x01,           /* B0 enable: 0/disable, 1/enable*/
    0x02,           /* B1 poll count: number of full power poll before starting lptd poll */
    0xFF,           /* B2 sniff count lsb: number of lptd poll before switching to full power poll */
    0xFF,           /* B3 sniff count msb */
    0x80,           /* B4 threshold: Bigger thresholds give a smaller LPTD range but more immunity to false detections. Smaller thresholds increase LPTD range at the cost of greater likelihood of false detections. */
    0x40,           /* B5 delay lsb: delay (us) to sampling power */
    0x00,           /* B6 delay msb */
    0x40,           /* B7 carrier threshold lsb */
    0x00,           /* B8 carrier threshold msb */
    0x80,           /* B9 mode: Bitwise variable used to enable various algorithm modes.*/
    0x80,           /* B10 0-offset lsb */
    0x00,           /* B11 0-offset msb */
    0x10,           /* B12 field sense time lsb */
    0x00,           /* B13 field sense time msb */
    0x00,           /* B14 false detect threshold lsb: 0x00 to disable LPTD NTF. The number of false tag detections to resport LPTD NTF. */
    0x00,           /* B15 false detect threshold msb. A false tag detect - full poll results in no tag being detected.*/
    0x75,           /* B16 mode1; Bitwise variable used to enable various algorithm modes. */
    0x0D,           /* B17 lptd ant cfg rx */
    0x30,           /* B18 lptd rdr cfg ve */
};

UINT8 *p_nfa_dm_lptd_cfg = (UINT8 *) &nfa_dm_lptd_cfg[0];

/* the SetConfig for full power mode */
const UINT8 nfa_power_bitmap_full[] = 
{
    /* TLV len */   5,
    /* B0 */        NCI_PARAM_ID_PWR_SETTING_BITMAP,
    /* B1 */        NCI_PARAM_LEN_PWR_SETTING_BITMAP,
    /* B2 */        0x00,
    /* B3 */        0x00,
    /* B4 */        0x00
};

UINT8 *p_nfa_power_bitmap_full = (UINT8 *) nfa_power_bitmap_full;

/* the SetConfig for CE low power mode */
const UINT8 nfa_power_bitmap_ce_lp[] = 
{
    /* TLV len */   5,
    /* B0 */        NCI_PARAM_ID_PWR_SETTING_BITMAP,
    /* B1 */        NCI_PARAM_LEN_PWR_SETTING_BITMAP,
    /* B2 */        0x00,
    /* B3 */        0x02, /* 0x02: 2.5V VDDIO CAP for CE4 */
    /* B4 */        0x00
};

UINT8 *p_nfa_power_bitmap_ce_lp = (UINT8 *) nfa_power_bitmap_ce_lp;

