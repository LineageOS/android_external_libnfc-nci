/****************************************************************************
**
**  Name:       libmain.h
**
**  Function    This file contains helper functions for NFA
**
**  Copyright (c) 2011-2012, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include "buildcfg.h"
#include "nfa_mem_co.h"
#include "nfa_nv_co.h"
#include "nfa_nv_ci.h"
#include <cutils/log.h>
#include "config.h"

#define LOG_TAG "BrcmNfcNfa"
#define PRINT(s) __android_log_write(ANDROID_LOG_DEBUG, "BrcmNci", s)

extern UINT32 ScrProtocolTraceFlag;         // = SCR_PROTO_TRACE_ALL; // 0x017F;
static const char* sTable = "0123456789abcdef";
extern char bcm_nfc_location[];

/*******************************************************************************
**
** Function         nfa_mem_co_alloc
**
** Description      allocate a buffer from platform's memory pool
**
** Returns:
**                  pointer to buffer if successful
**                  NULL otherwise
**
*******************************************************************************/
NFC_API extern void *nfa_mem_co_alloc(UINT32 num_bytes)
{
    return malloc(num_bytes);
}


/*******************************************************************************
**
** Function         nfa_mem_co_free
**
** Description      free buffer previously allocated using nfa_mem_co_alloc
**
** Returns:
**                  Nothing
**
*******************************************************************************/
NFC_API extern void nfa_mem_co_free(void *pBuffer)
{
    free(pBuffer);
}

/*******************************************************************************
**
** Function         nfa_nv_co_init
**
** Description      This function is executed as a part of the start up sequence
**                  to make sure the control block is initialized.
**
** Parameters       void.
**
** Returns          void
**
**
*******************************************************************************/
NFC_API extern void nfa_nv_co_init(void)
{
    ALOGD ("%s: enter/exit ", __FUNCTION__);
}

/*******************************************************************************
**
** Function         nfa_nv_co_read
**
** Description      This function is called by NFA to read in data from the
**                  previously opened file.
**
** Parameters       pBuffer   - buffer to read the data into.
**                  nbytes  - number of bytes to read into the buffer.
**
** Returns          void
**                  
**                  Note: Upon completion of the request, nfa_nv_ci_read() is
**                        called with the buffer of data, along with the number
**                        of bytes read into the buffer, and a status.  The
**                        call-in function should only be called when ALL requested
**                        bytes have been read, the end of file has been detected,
**                        or an error has occurred.
**
*******************************************************************************/
NFC_API extern void nfa_nv_co_read(UINT8 *pBuffer, UINT16 nbytes, UINT8 block)
{
    char filename[256], filename2[256];
    strcpy(filename2, bcm_nfc_location);
    strcat(filename2, "/nfaStorage.bin");
    if (strlen(filename2) > 200)
    {
        ALOGE ("%s: filename too long", __FUNCTION__);
        return;
    }
    sprintf (filename, "%s%u", filename2, block);

    ALOGD ("%s: buffer len=%u; file=%s", __FUNCTION__, nbytes, filename);
    int fileStream = open (filename, O_RDONLY);
    if (fileStream > 0)
    {
        size_t actualRead = read (fileStream, pBuffer, nbytes);
        if (actualRead > 0)
        {
            ALOGD ("%s: read bytes=%u", __FUNCTION__, actualRead);
            nfa_nv_ci_read (actualRead, NFA_NV_CO_OK, block);
        }
        else
        {
            ALOGE ("%s: fail to read", __FUNCTION__);
            nfa_nv_ci_read (actualRead, NFA_NV_CO_FAIL, block);
        }
        close (fileStream);
    }
    else
    {
        ALOGE ("%s: fail to open", __FUNCTION__);
        nfa_nv_ci_read (0, NFA_NV_CO_FAIL, block);
    }
}

/*******************************************************************************
**
** Function         nfa_nv_co_write
**
** Description      This function is called by io to send file data to the
**                  phone.
**
** Parameters       pBuffer   - buffer to read the data from.
**                  nbytes  - number of bytes to write out to the file.
**
** Returns          void
**                  
**                  Note: Upon completion of the request, nfa_nv_ci_write() is
**                        called with the file descriptor and the status.  The
**                        call-in function should only be called when ALL requested
**                        bytes have been written, or an error has been detected,
**
*******************************************************************************/
NFC_API extern void nfa_nv_co_write(const UINT8 *pBuffer, UINT16 nbytes, UINT8 block)
{
    char filename[256], filename2[256];
    strcpy(filename2, bcm_nfc_location);
    strcat(filename2, "/nfaStorage.bin");
    if (strlen(filename2) > 200)
    {
        ALOGE ("%s: filename too long", __FUNCTION__);
        return;
    }
    sprintf (filename, "%s%u", filename2, block);
    ALOGD ("%s: bytes=%u; file=%s", __FUNCTION__, nbytes, filename);
    
    int fileStream = 0;
    
    fileStream = open (filename, O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR);
    if (fileStream > 0)
    {
        size_t actualWritten = write (fileStream, pBuffer, nbytes);
        ALOGD ("%s: %d bytes written", __FUNCTION__, actualWritten);
        if (actualWritten > 0) {
            nfa_nv_ci_write (NFA_NV_CO_OK);
        }
        else
        {
            ALOGE ("%s: fail to write", __FUNCTION__);
            nfa_nv_ci_write (NFA_NV_CO_FAIL);
        }
        close (fileStream);
    }
    else
    {
        ALOGE ("%s: fail to open, error = %d", __FUNCTION__, errno);
        nfa_nv_ci_write (NFA_NV_CO_FAIL);
    }
}

/*******************************************************************************
**
** Function         byte2hex
**
** Description      convert a byte array to hexadecimal string
**
** Returns:
**                  Nothing
**
*******************************************************************************/
static inline void byte2hex(const char* data, char** str)
{
    **str = sTable[(*data >> 4) & 0xf];
    ++*str;
    **str = sTable[*data & 0xf];
    ++*str;
}

/*******************************************************************************
**
** Function         byte2char
**
** Description      convert a byte array to displayable text string
**
** Returns:
**                  Nothing
**
*******************************************************************************/
static inline void byte2char(const char* data, char** str)
{
    **str = *data < ' ' ? '.' : *data > '~' ? '.' : *data;
    ++(*str);
}

/*******************************************************************************
**
** Function         word2hex
**
** Description      Convert a two byte into text string as little-endian WORD
**
** Returns:
**                  Nothing
**
*******************************************************************************/
static inline void word2hex(const char* data, char** hex)
{
    byte2hex(&data[1], hex);
    byte2hex(&data[0], hex);
}

/*******************************************************************************
**
** Function         dumpbin
**
** Description      convert a byte array to a blob of text string for logging
**
** Returns:
**                  Nothing
**
*******************************************************************************/
void dumpbin(const char* data, int size, UINT32 trace_layer, UINT32 trace_type)
{
    char line_buff[256];
    char *line;
    int i, j, addr;
    const int width = 16;
    if(size <= 0)
        return;
#ifdef __RAW_HEADER
    //write offset
    line = line_buff;
    *line++ = ' ';
    *line++ = ' ';
    *line++ = ' ';
    *line++ = ' ';
    *line++ = ' ';
    *line++ = ' ';
    for(j = 0; j < width; j++)
    {
        byte2hex((const char*)&j, &line);
        *line++ = ' ';
    }
    *line = 0;
    PRINT(line_buff);
#endif
    for(i = 0; i < size / width; i++)
    {
        line = line_buff;
        //write address:
        addr = i*width;
        word2hex((const char*)&addr, &line);
        *line++ = ':'; *line++ = ' ';
        //write hex of data
        for(j = 0; j < width; j++)
        {
            byte2hex(&data[j], &line);
            *line++ = ' ';
        }
        //write char of data
        for(j = 0; j < width; j++)
            byte2char(data++, &line);
        //wirte the end of line
        *line = 0;
        //output the line
        PRINT(line_buff);
    }
    //last line of left over if any
    int leftover = size % width;
    if(leftover > 0)
    {
        line = line_buff;
        //write address:
        addr = i*width;
        word2hex((const char*)&addr, &line);
        *line++ = ':'; *line++ = ' ';
        //write hex of data
        for(j = 0; j < leftover; j++)
        {
            byte2hex(&data[j], &line);
            *line++ = ' ';
        }
        //write hex padding
        for(; j < width; j++)
        {
            *line++ = ' ';
            *line++ = ' ';
            *line++ = ' ';
        }
        //write char of data
        for(j = 0; j < leftover; j++)
            byte2char(data++, &line);
        //write the end of line
        *line = 0;
        //output the line
        PRINT(line_buff);
    }
}

/*******************************************************************************
**
** Function         scru_dump_hex
**
** Description      print a text string to log
**
** Returns:
**                  text string
**
*******************************************************************************/
UINT8 *scru_dump_hex (UINT8 *p, char *pTitle, UINT32 len, UINT32 layer, UINT32 type)
{
    if(pTitle && *pTitle)
        PRINT(pTitle);
    dumpbin(p, len, layer, type);
    return p;
}

/*******************************************************************************
**
** Function         DispHciCmd
**
** Description      Display a HCI command string
**
** Returns:
**                  Nothing
**
*******************************************************************************/
void DispHciCmd (BT_HDR *pBuffer)
{
    UINT8   *p = (UINT8 *)(pBuffer + 1) + pBuffer->offset;

    if (!(ScrProtocolTraceFlag & SCR_PROTO_TRACE_HCI_SUMMARY))
        return;

    BTDISP_LOCK_LOG();

    BTDISP_UNLOCK_LOG();
}


/*******************************************************************************
**
** Function         DispHciEvt
**
** Description      display a NCI event
**
** Returns:
**                  Nothing
**
*******************************************************************************/
void DispHciEvt (BT_HDR *pBuffer)
{
    if (!(ScrProtocolTraceFlag & SCR_PROTO_TRACE_HCI_SUMMARY))
        return;

    BTDISP_LOCK_LOG();

    BTDISP_UNLOCK_LOG();
}


