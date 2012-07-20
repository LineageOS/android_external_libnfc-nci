/*******************************************************************************
**
**  File:         nci_hmsgs.h
**
**  Description:  defines NCI interface messages (for DH)
**
**  Copyright (c) 2009-2010, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*******************************************************************************/
#ifndef NFC_NCI_HMSGS_H
#define NFC_NCI_HMSGS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nci_defs.h"


BOOLEAN nci_proc_core_rsp(BT_HDR *p_msg);
void nci_proc_rf_management_rsp(BT_HDR *p_msg);
void nci_proc_ee_management_rsp(BT_HDR *p_msg);
void nci_proc_core_ntf(BT_HDR *p_msg);
void nci_proc_rf_management_ntf(BT_HDR *p_msg);
void nci_proc_ee_management_ntf(BT_HDR *p_msg);
void nci_proc_prop_rsp(BT_HDR *p_msg);
void nci_proc_prop_ntf(BT_HDR *p_msg);


UINT8 nci_snd_core_reset (UINT8 reset_type);
UINT8 nci_snd_core_init (void);
UINT8 nci_snd_core_get_config (UINT8 *param_ids, UINT8 num_ids);
UINT8 nci_snd_core_set_config (UINT8 *p_param_tlvs, UINT8 tlv_size);

UINT8 nci_snd_core_conn_create (UINT8 dest_type, UINT8 num_tlv, UINT8 tlv_size, UINT8 *p_param_tlvs);
UINT8 nci_snd_core_conn_close (UINT8 conn_id);



UINT8 nci_snd_discover_cmd (UINT8 num, tNCI_DISCOVER_PARAMS *p_param);

UINT8 nci_snd_discover_select_cmd (UINT8 rf_disc_id, UINT8 protocol, UINT8 rf_interface);
UINT8 nci_snd_deactivate_cmd (UINT8 de_act_type );
UINT8 nci_snd_discover_map_cmd (UINT8 num, tNCI_DISCOVER_MAPS *p_maps);
UINT8 nci_snd_t3t_polling (UINT16 system_code, UINT8 rc, UINT8 tsn);
UINT8 nci_snd_parameter_update_cmd (UINT8 *p_param_tlvs, UINT8 tlv_size);
#if ((NFC_NFCEE_INCLUDED == TRUE) && (NFC_RW_ONLY == FALSE))
UINT8 nci_snd_nfcee_discover (UINT8 discover_action);
UINT8 nci_snd_nfcee_mode_set (UINT8 nfcee_id, UINT8 nfcee_mode);
UINT8 nci_snd_set_routing_cmd (BOOLEAN more, UINT8 target_handle, UINT8 num_tlv, UINT8 tlv_size, UINT8 *p_param_tlvs);
UINT8 nci_snd_get_routing_cmd(void);
#endif

#ifdef __cplusplus
}
#endif

#endif  /* NFC_NCI_MSGS_H */
