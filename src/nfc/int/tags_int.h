/******************************************************************************
**
** File:         tags_int.h
**
**  Description:   This file contains the common data types shared by 
**                 Reader/Writer mode and Card Emulation.
**
** Copyright (c) 2009-2010  Broadcom Corp.  All Rights Reserved.
** Broadcom Bluetooth Core. Proprietary and confidential.
**
******************************************************************************/

#ifndef TAGS_INT_H
#define TAGS_INT_H

/******************************************************************************
// T1T command and response definitions
******************************************************************************/

typedef struct
{
    UINT8   opcode;
    UINT8   cmd_len;
    UINT8   uid_offset;
    UINT8   rsp_len;
} tT1T_CMD_RSP_INFO;

typedef struct
{
    UINT8   tag_model;
    UINT8   tms;
    UINT8   b_dynamic;
    UINT8   lock_tlv[3];
    UINT8   mem_tlv[3];
} tT1T_INIT_TAG;

typedef struct
{
    UINT8   manufacturer_id;
    BOOLEAN b_multi_version;
    UINT8   version_block;
    UINT16  version_no;
    UINT16  version_bmask;
    UINT8   b_calc_cc;
    UINT8   tms;
    BOOLEAN b_otp;
} tT2T_INIT_TAG;

typedef struct
{
    UINT8   opcode;
    UINT8   cmd_len;
    UINT8   rsp_len;
    UINT8   nack_rsp_len;
} tT2T_CMD_RSP_INFO;

extern const UINT8 t4t_v10_ndef_tag_aid[];  /* V1.0 Type 4 Tag Applicaiton ID */
extern const UINT8 t4t_v20_ndef_tag_aid[];  /* V2.0 Type 4 Tag Applicaiton ID */

extern const tT1T_CMD_RSP_INFO t1t_cmd_rsp_infos[];
extern const tT1T_INIT_TAG t1t_init_content[];
extern const tT1T_CMD_RSP_INFO * t1t_cmd_to_rsp_info(UINT8 opcode);
extern const tT1T_INIT_TAG * t1t_tag_init_data (UINT8 tag_model);
extern UINT8 t1t_info_to_evt (const tT1T_CMD_RSP_INFO * p_info);

extern const tT2T_INIT_TAG * t2t_tag_init_data (UINT8 manufacturer_id, BOOLEAN b_valid_ver, UINT16 version_no);
extern const tT2T_CMD_RSP_INFO t2t_cmd_rsp_infos[];
extern const tT2T_CMD_RSP_INFO * t2t_cmd_to_rsp_info(UINT8 opcode);
extern UINT8 t2t_info_to_evt (const tT2T_CMD_RSP_INFO * p_info);


#if (BT_TRACE_PROTOCOL == TRUE)
extern const char * t1t_info_to_str (const tT1T_CMD_RSP_INFO * p_info);
extern const char * t2t_info_to_str (const tT2T_CMD_RSP_INFO * p_info);
#else
#define t1t_info_to_str(x) ""
#define t2t_info_to_str(x) ""
#endif
extern int tags_pow(int x, int y);
extern unsigned int tags_log2(register unsigned int x);


#endif /* TAGS_INT_H */
