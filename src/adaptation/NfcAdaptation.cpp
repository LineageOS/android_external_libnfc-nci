/****************************************************************************
**
**  Name:       patchram.c
**
**  Function:   this file contains functions related to NfcAdaptation
**
**
**  Copyright (c) 1999-2011, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#include "OverrideLog.h"
#include "NfcAdaptation.h"
extern "C"
{
    #include "gki.h"
    #include "nfa_api.h"
    #include "nci_int.h"
    #include "nfc_int.h"
    #include "userial.h"
}
#include "config.h"

#define LOG_TAG "NfcAdaptation"

extern "C" void GKI_shutdown();
extern void resetConfig();

NfcAdaptation* NfcAdaptation::mpInstance = NULL;
ThreadMutex NfcAdaptation::sLock;

UINT32 ScrProtocolTraceFlag = SCR_PROTO_TRACE_ALL; //0x017F00;
UINT8 appl_trace_level = 0xff;
char bcm_nfc_location[120];

/*******************************************************************************
**
** Function:    NfcAdaptation::NfcAdaptation()
**
** Description: class constructor
**
** Returns:     none
**
*******************************************************************************/
NfcAdaptation::NfcAdaptation()
{
}

/*******************************************************************************
**
** Function:    NfcAdaptation::~NfcAdaptation()
**
** Description: class destructor
**
** Returns:     none
**
*******************************************************************************/
NfcAdaptation::~NfcAdaptation()
{
    mpInstance = NULL;
}

/*******************************************************************************
**
** Function:    NfcAdaptation::GetInstance()
**
** Description: access class singleton
**
** Returns:     pointer to the singleton object
**
*******************************************************************************/
NfcAdaptation& NfcAdaptation::GetInstance()
{
    AutoThreadMutex  a(sLock);

    if (!mpInstance)
        mpInstance = new NfcAdaptation;
    return *mpInstance;
}

/*******************************************************************************
**
** Function:    NfcAdaptation::Initialize()
**
** Description: class initializer
**
** Returns:     none
**
*******************************************************************************/
void NfcAdaptation::Initialize ()
{
    const char* func = "NfcAdaptation::Initialize";
    ALOGD("%s: enter\n", func);
    unsigned long num;

    if ( !GetStrValue ( NAME_NFA_STORAGE, bcm_nfc_location, sizeof ( bcm_nfc_location ) ) )
        strcpy ( bcm_nfc_location, "/data/bcmnfc" );
    if ( GetNumValue ( NAME_PROTOCOL_TRACE_LEVEL, &num, sizeof ( num ) ) )
        ScrProtocolTraceFlag = num;
    initializeGlobalAppLogLevel ();

    GKI_init ();
    GKI_enable ();
    GKI_create_task ((TASKPTR)NFCA_TASK, BTU_TASK, (INT8*)"NFCA_TASK", 0, 0, (pthread_cond_t*)NULL, NULL);
    {
        AutoThreadMutex guard(mCondVar);
        GKI_create_task ((TASKPTR)Thread, MMI_TASK, (INT8*)"NFCA_THREAD", 0, 0, (pthread_cond_t*)NULL, NULL);
        mCondVar.wait();
    }
    ALOGD ("%s: exit", func);
}

/*******************************************************************************
**
** Function:    NfcAdaptation::Finalize()
**
** Description: class finalizer
**
** Returns:     none
**
*******************************************************************************/
void NfcAdaptation::Finalize()
{
    AutoThreadMutex  a(sLock);

    ALOGD ("%s: enter", __func__);
    GKI_shutdown ();

    resetConfig();
    ALOGD ("%s: exit", __func__);
    delete this;
}

/*******************************************************************************
**
** Function:    NfcAdaptation::signal()
**
** Description: signal the CondVar to release the thread that is waiting
**
** Returns:     none
**
*******************************************************************************/
void NfcAdaptation::signal ()
{
    mCondVar.signal();
}

/*******************************************************************************
**
** Function:    NfcAdaptation::NFCA_TASK()
**
** Description: NFCA_TASK runs the GKI main task
**
** Returns:     none
**
*******************************************************************************/
UINT32 NfcAdaptation::NFCA_TASK (UINT32 arg)
{
    ALOGD ("%s: enter", __func__);
    GKI_run (0);
    ALOGD ("%s: exit", __func__);
    return NULL;
}

/*******************************************************************************
**
** Function:    NfcAdaptation::Thread()
**
** Description: Creates work threads
**
** Returns:     none
**
*******************************************************************************/
UINT32 NfcAdaptation::Thread (UINT32 arg)
{
    unsigned long num;
    char temp[120];
    ALOGD ("%s: enter", __func__);
    tUSERIAL_OPEN_CFG cfg;
    struct tUART_CONFIG  uart;

    if ( GetStrValue ( NAME_UART_PARITY, temp, sizeof ( temp ) ) )
    {
        if ( strcmp ( temp, "even" ) == 0 )
            uart.m_iParity = USERIAL_PARITY_EVEN;
        else if ( strcmp ( temp, "odd" ) == 0 )
            uart.m_iParity = USERIAL_PARITY_ODD;
        else if ( strcmp ( temp, "none" ) == 0 )
            uart.m_iParity = USERIAL_PARITY_NONE;
    }
    else
        uart.m_iParity = USERIAL_PARITY_NONE;

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
    else
        uart.m_iStopbits = USERIAL_STOPBITS_1;

    if ( GetNumValue ( NAME_UART_DATABITS, &num, sizeof ( num ) ) )
    {
        if ( 5 <= num && num <= 8 )
            uart.m_iDatabits = ( 1 << ( num + 1 ) );
    }
    else
        uart.m_iDatabits = USERIAL_DATABITS_8;

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
    else
        uart.m_iBaudrate = USERIAL_BAUD_115200;

    memset (&cfg, 0, sizeof(tUSERIAL_OPEN_CFG));
    cfg.fmt = uart.m_iDatabits | uart.m_iParity | uart.m_iStopbits;
    cfg.baud = uart.m_iBaudrate;

    ALOGD ("%s: uart config=0x%04x, %d\n", __func__, cfg.fmt, cfg.baud);
    USERIAL_Init(&cfg);
    {
        ThreadCondVar    CondVar;
        AutoThreadMutex  guard(CondVar);
        GKI_create_task ((TASKPTR)ncit_task, NCIT_TASK, (INT8*)"NCIT_TASK", 0, 0, (pthread_cond_t*)CondVar, (pthread_mutex_t*)CondVar);
        CondVar.wait();
    }
    {
        ThreadCondVar    CondVar;
        AutoThreadMutex  guard(CondVar);
        GKI_create_task ((TASKPTR)nfc_task, NFC_TASK, (INT8*)"NFC_TASK", 0, 0, (pthread_cond_t*)CondVar, (pthread_mutex_t*)CondVar);
        CondVar.wait();
    }

    NfcAdaptation::GetInstance().signal();

    ALOGD ("%s: exit", __func__);
    return NULL;
}

/*******************************************************************************
**
** Function:    ThreadMutex::ThreadMutex()
**
** Description: class constructor
**
** Returns:     none
**
*******************************************************************************/
ThreadMutex::ThreadMutex()
{
    pthread_mutexattr_t mutexAttr;

    pthread_mutexattr_init(&mutexAttr);
    pthread_mutex_init(&mMutex, &mutexAttr);
    pthread_mutexattr_destroy(&mutexAttr);
}

/*******************************************************************************
**
** Function:    ThreadMutex::~ThreadMutex()
**
** Description: class destructor
**
** Returns:     none
**
*******************************************************************************/
ThreadMutex::~ThreadMutex()
{
    pthread_mutex_destroy(&mMutex);
}

/*******************************************************************************
**
** Function:    ThreadMutex::lock()
**
** Description: lock kthe mutex
**
** Returns:     none
**
*******************************************************************************/
void ThreadMutex::lock()
{
    pthread_mutex_lock(&mMutex);
}

/*******************************************************************************
**
** Function:    ThreadMutex::unblock()
**
** Description: unlock the mutex
**
** Returns:     none
**
*******************************************************************************/
void ThreadMutex::unlock()
{
    pthread_mutex_unlock(&mMutex);
}

/*******************************************************************************
**
** Function:    ThreadCondVar::ThreadCondVar()
**
** Description: class constructor
**
** Returns:     none
**
*******************************************************************************/
ThreadCondVar::ThreadCondVar()
{
    pthread_condattr_t CondAttr;

    pthread_condattr_init(&CondAttr);
    pthread_cond_init(&mCondVar, &CondAttr);

    pthread_condattr_destroy(&CondAttr);
}

/*******************************************************************************
**
** Function:    ThreadCondVar::~ThreadCondVar()
**
** Description: class destructor
**
** Returns:     none
**
*******************************************************************************/
ThreadCondVar::~ThreadCondVar()
{
    pthread_cond_destroy(&mCondVar);
}

/*******************************************************************************
**
** Function:    ThreadCondVar::wait()
**
** Description: wait on the mCondVar
**
** Returns:     none
**
*******************************************************************************/
void ThreadCondVar::wait()
{
    pthread_cond_wait(&mCondVar, *this);
    pthread_mutex_unlock(*this);
}

/*******************************************************************************
**
** Function:    ThreadCondVar::signal()
**
** Description: signal the mCondVar
**
** Returns:     none
**
*******************************************************************************/
void ThreadCondVar::signal()
{
    AutoThreadMutex  a(*this);
    pthread_cond_signal(&mCondVar);
}

/*******************************************************************************
**
** Function:    AutoThreadMutex::AutoThreadMutex()
**
** Description: class constructor, automatically lock the mutex
**
** Returns:     none
**
*******************************************************************************/
AutoThreadMutex::AutoThreadMutex(ThreadMutex &m)
    : mm(m)
{
    mm.lock();
}

/*******************************************************************************
**
** Function:    AutoThreadMutex::~AutoThreadMutex()
**
** Description: class destructor, automatically unlock the mutex
**
** Returns:     none
**
*******************************************************************************/
AutoThreadMutex::~AutoThreadMutex()
{
    mm.unlock();
}
