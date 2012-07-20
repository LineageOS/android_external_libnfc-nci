/****************************************************************************
**
**  Name:       libmain.h
**
**  Function    This file contains helper functions for reading configuration
**              settings form libnfc-brcm.conf file.
**
**  Copyright (c) 2011-2012, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#include "gki.h"
#include "nfa_api.h"
#include "nci_int.h"
#include "nfc_int.h"
#include <cutils/log.h>
#include "userial.h"
#include "config.h"
#include "nfa_api.h"
#include "nfa_brcm_api.h"
#include "nfa_ce_api.h"

#define LOG_TAG "NfcAdaptation"

#define MAX_PATH            (120)

#define NAME_TRANSPORT_DRIVER   "TRANSPORT_DRIVER"
#define NAME_POWER_CONTROL_DRIVER "POWER_CONTROL_DRIVER"
#define NAME_PROTOCOL_TRACE     "PROTOCOL_TRACE_LEVEL"
#define NAME_APPL_TRACE         "APPL_TRACE_LEVEL"
#define NAME_UART_PORT          "UART_PORT"
#define NAME_UART_BAUD          "UART_BAUD"
#define NAME_UART_PARITY        "UART_PARITY"
#define NAME_UART_STOPBITS      "UART_STOPBITS"
#define NAME_UART_DATABITS      "UART_DATABITS"
#define NAME_CLIENT_ADDRESS     "BCMI2CNFC_ADDRESS"
#define NAME_NFA_DM_START_UP_CFG "NFA_DM_START_UP_CFG"
#define NAME_NFA_DM_CFG         "NFA_DM_CFG"
#define NAME_NFA_DM_LP_CFG      "NFA_DM_LP_CFG"
#define NAME_LOW_SPEED_TRANSPORT "LOW_SPEED_TRANSPORT"
#define NAME_NFC_WAKE_DELAY     "NFC_WAKE_DELAY"
#define NAME_NFC_WRITE_DELAY    "NFC_WRITE_DELAY"
#define NAME_PERF_MEASURE_FREQ  "REPORT_PERFORMANCE_MEASURE"
#define NAME_READ_MULTI_PACKETS "READ_MULTIPLE_PACKETS"
#define NAME_POWER_ON_DELAY     "POWER_ON_DELAY"
#define NAME_NFA_STORAGE        "NFA_STORAGE"
#define NAME_NFA_DM_START_UP_VSC_CFG "NFA_DM_START_UP_VSC_CFG"

// default configuration
#define default_transport       "/dev/bcm2079x"
#define default_storage_location "/data/bcmnfc"
int uart_port  = 0;
int isLowSpeedTransport = 0;
int nfc_wake_delay = 0;
int nfc_write_delay = 0;
int gPowerOnDelay = 300;
char userial_dev[BTE_APPL_MAX_USERIAL_DEV_NAME+1];
char power_control_dev[BTE_APPL_MAX_USERIAL_DEV_NAME+1];
char bcm_nfc_location[MAX_PATH+1];

UINT32 ScrProtocolTraceFlag = SCR_PROTO_TRACE_ALL; //0x017F00;
UINT8 appl_trace_level = 0xff;

struct tUART_CONFIG  uartConfig = {
    .m_iBaudrate = USERIAL_BAUD_115200,
    .m_iDatabits = USERIAL_DATABITS_8,
    .m_iParity = USERIAL_PARITY_NONE,
    .m_iStopbits = USERIAL_STOPBITS_1
};

UINT8 bcmi2cnfc_client_addr = 0;
UINT8 bcmi2cnfc_read_multi_packets = 0;
static UINT8 nfa_dm_cfg[sizeof ( tNFA_DM_CFG ) ];
static UINT8 nfa_dm_lp_cfg[sizeof ( tNFA_DM_LP_CFG ) ];

/* Startup Configuration
 * Note we allocate a little extra space just incase
 * more TLVs are desired.
 * */
#define NFA_DM_START_UP_CFG_PARAM_MAX_LEN   100
static UINT8 nfa_dm_start_up_cfg[NFA_DM_START_UP_CFG_PARAM_MAX_LEN];
extern UINT8 *p_nfa_dm_start_up_cfg;
extern tNFA_DM_CFG *p_nfa_dm_cfg;
extern int read_config ( char* name );
extern int perf_log_every_count;
static UINT8 nfa_dm_start_up_vsc_cfg[NFA_DM_START_UP_CFG_PARAM_MAX_LEN];
extern UINT8 *p_nfa_dm_start_up_vsc_cfg;

/*******************************************************************************
**
** Function         readDefaultConfig
**
** Description      read dewfault configuration settings
**
** Returns:
**                  Nothing
**
*******************************************************************************/
void readDefaultConfig()
{
    char   temp[128];
    unsigned long num;
    struct tUART_CONFIG uart;
    memcpy ( &uart, &uartConfig, sizeof ( struct tUART_CONFIG ) );
    if ( !GetStrValue ( NAME_NFA_STORAGE, bcm_nfc_location, sizeof ( bcm_nfc_location ) ) )
        strcpy ( bcm_nfc_location, default_storage_location );

    if ( !GetStrValue ( NAME_TRANSPORT_DRIVER, userial_dev, sizeof ( userial_dev ) ) )
    {
        strcpy ( userial_dev, default_transport );
    }
    ALOGD ( "%s userial_dev=%s\n", __func__, userial_dev );
    if ( !GetStrValue ( NAME_POWER_CONTROL_DRIVER, power_control_dev, sizeof ( power_control_dev ) ) )
    {
        // do not use any default for power control
    }
    ALOGD ( "%s power_control_dev=%s\n", __func__, power_control_dev );

    if ( GetNumValue ( NAME_UART_PORT, &num, sizeof ( num ) ) )
        uart_port = num;
    if ( GetStrValue ( NAME_UART_PARITY, temp, sizeof ( temp ) ) )
    {
        if ( strcmp ( temp, "even" ) == 0 )
            uart.m_iParity = USERIAL_PARITY_EVEN;
        else if ( strcmp ( temp, "odd" ) == 0 )
            uart.m_iParity = USERIAL_PARITY_ODD;
        else if ( strcmp ( temp, "none" ) == 0 )
            uart.m_iParity = USERIAL_PARITY_NONE;
    }

    if ( GetStrValue ( NAME_UART_STOPBITS, temp, sizeof ( temp ) ) )
    {
        if ( strcmp ( temp, "1" ) == 0 )
            uart.m_iStopbits = USERIAL_STOPBITS_1;
        else if ( strcmp ( temp, "2" ) == 0 )
            uart.m_iStopbits = USERIAL_STOPBITS_2;
        else if ( strcmp ( temp, "1.5" ) == 0 )
            uart.m_iStopbits = USERIAL_STOPBITS_1_5;
    }
    else if ( GetNumValue ( NAME_UART_STOPBITS, &num, sizeof ( num ) ) )
    {
        if ( num == 1 )
            uart.m_iStopbits = USERIAL_STOPBITS_1;
        else if ( num == 2 )
            uart.m_iStopbits = USERIAL_STOPBITS_2;
    }

    if ( GetNumValue ( NAME_UART_DATABITS, &num, sizeof ( num ) ) )
    {
        if ( 5 <= num && num <= 8 )
            uart.m_iDatabits = ( 1 << ( num + 1 ) );
    }

    if ( GetNumValue ( NAME_UART_BAUD, &num, sizeof ( num ) ) )
    {
        if ( num == 300 ) uart.m_iBaudrate = USERIAL_BAUD_300;
        else if ( num == 600 ) uart.m_iBaudrate = USERIAL_BAUD_600;
        else if ( num == 1200 ) uart.m_iBaudrate = USERIAL_BAUD_1200;
        else if ( num == 2400 ) uart.m_iBaudrate = USERIAL_BAUD_2400;
        else if ( num == 9600 ) uart.m_iBaudrate = USERIAL_BAUD_9600;
        else if ( num == 19200 ) uart.m_iBaudrate = USERIAL_BAUD_19200;
        else if ( num == 57600 ) uart.m_iBaudrate = USERIAL_BAUD_57600;
        else if ( num == 115200 ) uart.m_iBaudrate = USERIAL_BAUD_115200;
        else if ( num == 230400 ) uart.m_iBaudrate = USERIAL_BAUD_230400;
        else if ( num == 460800 ) uart.m_iBaudrate = USERIAL_BAUD_460800;
        else if ( num == 921600 ) uart.m_iBaudrate = USERIAL_BAUD_921600;
    }
    else if ( GetStrValue ( NAME_UART_BAUD, temp, sizeof ( temp ) ) )
    {
        if ( strcmp ( temp, "auto" ) == 0 )
            uart.m_iBaudrate = USERIAL_BAUD_AUTO;
    }
    if ( GetNumValue ( NAME_PROTOCOL_TRACE, &num, sizeof ( num ) ) )
        ScrProtocolTraceFlag = num;
    if ( GetNumValue ( NAME_APPL_TRACE, &num, sizeof ( num ) ) )
        appl_trace_level = num % 256;
    if ( GetNumValue ( NAME_CLIENT_ADDRESS, &num, sizeof ( num ) ) )
        bcmi2cnfc_client_addr = num & 0xFF;
    if ( GetNumValue ( NAME_LOW_SPEED_TRANSPORT, &num, sizeof ( num ) ) )
        isLowSpeedTransport = num;
    if ( GetNumValue ( NAME_READ_MULTI_PACKETS, &num, sizeof ( num ) ) )
        bcmi2cnfc_read_multi_packets = num;
    if ( GetNumValue ( NAME_POWER_ON_DELAY, &num, sizeof ( num ) ) )
        gPowerOnDelay = num;

    if ( GetStrValue ( NAME_NFA_DM_START_UP_CFG, (char*)nfa_dm_start_up_cfg, sizeof ( nfa_dm_start_up_cfg ) ) )
    {
        p_nfa_dm_start_up_cfg = &nfa_dm_start_up_cfg[0];
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
        p_nfa_dm_start_up_vsc_cfg = &nfa_dm_start_up_vsc_cfg[0];
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
    if ( GetStrValue ( NAME_NFA_DM_CFG, (char*)nfa_dm_cfg, sizeof ( nfa_dm_cfg ) ) )
        p_nfa_dm_cfg = ( tNFA_DM_CFG * ) &nfa_dm_cfg[0];
    if ( GetStrValue ( NAME_NFA_DM_LP_CFG, (char*)nfa_dm_lp_cfg, sizeof ( nfa_dm_lp_cfg ) ) )
        p_nfa_dm_lp_cfg = ( tNFA_DM_LP_CFG * ) &nfa_dm_lp_cfg[0];
    if ( GetNumValue ( NAME_NFC_WAKE_DELAY, &num, sizeof ( num ) ) )
        nfc_wake_delay = num;
    if ( GetNumValue ( NAME_NFC_WRITE_DELAY, &num, sizeof ( num ) ) )
        nfc_write_delay = num;
    if ( GetNumValue ( NAME_PERF_MEASURE_FREQ, &num, sizeof ( num ) ) )
        perf_log_every_count = num;
    ALOGD ( "%s done\nPOWER_ON_DELAY=%d\n", __func__ , gPowerOnDelay);
}

