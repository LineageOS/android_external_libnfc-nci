/*****************************************************************************
**
**  Name:           nfa_hci_int.h
**
**  Description:    This is the private interface file for the NFA HCI.
**
**  Copyright (c) 2010-2011, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
*****************************************************************************/
#ifndef NFA_HCI_INT_H
#define NFA_HCI_INT_H

#include "nfa_hci_api.h"

extern BOOLEAN HCI_LOOPBACK_DEBUG;

/*****************************************************************************
**  Constants and data types
*****************************************************************************/
#define NFA_HCI_SESSION_ID_LEN          8           /* HCI Session ID length */
#define NFA_MAX_PIPES_IN_GENERIC_GATE   0x0F        /* Maximum pipes that can be created on a generic pipe  */

#define NFA_HCI_VERSION_SW              0x090000    /* HCI SW Version number                       */
#define NFA_HCI_VERSION_HW              0x000000    /* HCI HW Version number                       */
#define NFA_HCI_VENDOR_NAME             "HCI"       /* Vendor Name                                 */
#define NFA_HCI_MODEL_ID                00          /* Model ID                                    */
#define NFA_HCI_VERSION                 90          /* HCI Version                                 */

/* NFA HCI states */
#define NFA_HCI_STATE_DISABLED          0x00     /* HCI is disabled  */
#define NFA_HCI_STATE_STARTUP           0x01     /* HCI performing Initialization sequence */
#define NFA_HCI_STATE_WAIT_NETWK_ENABLE 0x02     /* HCI is waiting for initialization of other host in the network */
#define NFA_HCI_STATE_IDLE              0x03     /* HCI is waiting to handle api commands  */
#define NFA_HCI_STATE_WAIT_RSP          0x04     /* HCI is performing api command request  */
#define NFA_HCI_STATE_REMOVE_GATE       0x05     /* Removing all pipes prior to removing the gate */
#define NFA_HCI_STATE_APP_DEREGISTER    0x06     /* Removing all pipes and gates prior to deregistering the app */
#define NFA_HCI_STATE_RESTORE           0x07     /* HCI restore */

typedef UINT8 tNFA_HCI_STATE;

/* NFA HCI PIPE states */
#define NFA_HCI_PIPE_CLOSED             0x00     /* Pipe is closed */
#define NFA_HCI_PIPE_OPENED             0x01     /* Pipe is opened */

#define NFA_HCI_INVALID_INX             0xFF


typedef UINT8 tNFA_HCI_COMMAND;
typedef UINT8 tNFA_HCI_RESPONSE;


/* NFA HCI Internal events */
enum
{
    NFA_HCI_API_REGISTER_APP_EVT = NFA_SYS_EVT_START (NFA_ID_HCI),/* Register APP with HCI */
    NFA_HCI_API_DEREGISTER_APP_EVT,                               /* Deregister an app from HCI */
    NFA_HCI_API_GET_APP_GATE_PIPE_EVT,                            /* Get the list of gate and pipe associated to the application */
    NFA_HCI_API_ALLOC_GATE_EVT,                                   /* Allocate a dyanmic gate for the application */ 
    NFA_HCI_API_DEALLOC_GATE_EVT,                                 /* Deallocate a previously allocated gate to the application */
    NFA_HCI_API_GET_HOST_LIST_EVT,                                /* Get the list of Host in the network */
    NFA_HCI_API_GET_REGISTRY_EVT,                                 /* Get a registry entry from a host */
    NFA_HCI_API_SET_REGISTRY_EVT,                                 /* Set a registry entry on a host */
    NFA_HCI_API_CREATE_PIPE_EVT,                                  /* Create a pipe between two gates */
    NFA_HCI_API_OPEN_PIPE_EVT,                                    /* Open a pipe */
    NFA_HCI_API_CLOSE_PIPE_EVT,                                   /* Close a pipe */
    NFA_HCI_API_DELETE_PIPE_EVT,                                  /* Delete a pipe */
    NFA_HCI_API_SEND_CMD_EVT,                                     /* Send command via pipe */
    NFA_HCI_API_SEND_RSP_EVT,                                     /* Application Response to a command */
    NFA_HCI_API_SEND_EVENT_EVT,                                   /* Send event via pipe */

    NFA_HCI_RSP_NV_READ_EVT,                                      /* Non volatile read complete event */                                 
    NFA_HCI_RSP_NV_WRITE_EVT,                                     /* Non volatile write complete event */
    NFA_HCI_VSC_INIT_EVT,                                         /* Vendor specific initialization is completed */
    NFA_HCI_EE_DISC_CMPLT_EVT,
    NFA_HCI_RSP_TIMEOUT_EVT,
    NFA_HCI_HCP_CONN_CREATE_EVT,
    NFA_HCI_VSC_TIMEOUT_EVT,                                      /* Timeout to Vendor specific operation */
    NFA_HCI_CHECK_QUEUE_EVT
};

#define NFA_HCI_FIRST_API_EVENT     NFA_HCI_API_REGISTER_APP_EVT
#define NFA_HCI_LAST_API_EVENT      NFA_HCI_API_SEND_EVENT_EVT

typedef UINT16 tNFA_HCI_INT_EVT;

/* Internal event structures.
**
** Note, every internal structure starts with a BT_HDR and an app handle
*/

/* data type for NFA_HCI_API_REGISTER_APP_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
    char                app_name[NFA_MAX_HCI_APP_NAME_LEN + 1];
    tNFA_HCI_CBACK      *p_cback;
    BOOLEAN             b_send_conn_evts;
    UINT16              buf_size;
    UINT8               *p_evt_buf;
} tNFA_HCI_API_REGISTER_APP;

/* data type for NFA_HCI_API_DEREGISTER_APP_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
    char                app_name[NFA_MAX_HCI_APP_NAME_LEN + 1];
} tNFA_HCI_API_DEREGISTER_APP;

/* data type for NFA_HCI_API_GET_APP_GATE_PIPE_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
} tNFA_HCI_API_GET_APP_GATE_PIPE;

/* data type for NFA_HCI_API_ALLOC_GATE_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
} tNFA_HCI_API_ALLOC_GATE;


/* data type for NFA_HCI_API_DEALLOC_GATE_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
    UINT8               gate;
} tNFA_HCI_API_DEALLOC_GATE;

/* data type for NFA_HCI_API_GET_HOST_LIST_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
    tNFA_STATUS         status;
} tNFA_HCI_API_GET_HOST_LIST;

/* data type for NFA_HCI_API_GET_REGISTRY_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
    UINT8               pipe;
    UINT8               reg_inx;
} tNFA_HCI_API_GET_REGISTRY;

/* data type for NFA_HCI_API_SET_REGISTRY_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
    UINT8               pipe;
    UINT8               reg_inx;
    UINT8               size;
    UINT8               data[255];
} tNFA_HCI_API_SET_REGISTRY;

/* data type for NFA_HCI_API_CREATE_PIPE_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
    tNFA_STATUS         status;
    UINT8               source_gate;
    UINT8               dest_host;
    UINT8               dest_gate;
} tNFA_HCI_API_CREATE_PIPE_EVT;

/* data type for NFA_HCI_API_OPEN_PIPE_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
    tNFA_STATUS         status;
    UINT8               pipe;
} tNFA_HCI_API_OPEN_PIPE_EVT;

/* data type for NFA_HCI_API_CLOSE_PIPE_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
    tNFA_STATUS         status;
    UINT8               pipe;
} tNFA_HCI_API_CLOSE_PIPE_EVT;

/* data type for NFA_HCI_API_DELETE_PIPE_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
    tNFA_STATUS         status;
    UINT8               pipe;
} tNFA_HCI_API_DELETE_PIPE_EVT;

/* data type for NFA_HCI_API_SEND_EVENT_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
    UINT8               pipe;
    UINT8               evt_code;
    UINT16              evt_len;
    UINT8               data[NFA_MAX_HCI_EVENT_LEN];
} tNFA_HCI_API_SEND_EVENT_EVT;

/* data type for NFA_HCI_API_SEND_CMD_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
    UINT8               pipe;
    UINT8               cmd_code;
    UINT16              cmd_len;
    UINT8               data[NFA_MAX_HCI_CMD_LEN];
} tNFA_HCI_API_SEND_CMD_EVT;

/* data type for NFA_HCI_RSP_NV_READ_EVT */
typedef struct
{
    BT_HDR              hdr;
    UINT8               block;
    UINT16              size;
    tNFA_STATUS         status;
} tNFA_HCI_RSP_NV_READ_EVT;

/* data type for NFA_HCI_RSP_NV_WRITE_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_STATUS         status;
} tNFA_HCI_RSP_NV_WRITE_EVT;

/* data type for NFA_HCI_VSC_INIT_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_STATUS         status;
} tNFA_HCI_VSC_INIT_EVT;

/* data type for NFA_HCI_EE_DISC_CMPLT_EVT */
typedef struct
{
    BT_HDR              hdr;
    BOOLEAN             b_disc_cmplt;
    tNFA_STATUS         status;
} tNFA_HCI_EE_DISC_CMPLT_EVT;

/* data type for NFA_HCI_HCP_CONN_CREATE_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_STATUS         status;
} tNFA_HCI_HCP_CONN_CREATE_EVT;

/* data type for NFA_HCI_API_SEND_RSP_EVT */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
    UINT8               pipe;
    UINT8               response;
    UINT8               size;
    UINT8               data[255];
} tNFA_HCI_API_SEND_RSP_EVT;

/* common data type for internal events */
typedef struct
{
    BT_HDR              hdr;
    tNFA_HANDLE         hci_handle;
} tNFA_HCI_COMM_DATA;

/* union of all event data types */
typedef union
{
    BT_HDR                              hdr;
    tNFA_HCI_COMM_DATA                  comm;

    /* API events */
    tNFA_HCI_API_REGISTER_APP           app_info;                       /* Register/Deregister an application */
    tNFA_HCI_API_GET_APP_GATE_PIPE      get_gate_pipe_list;             /* Get the list of gates and pipes created for the application */
    tNFA_HCI_API_ALLOC_GATE             gate_info;                      /* Allocate a dynamic gate to the application */
    tNFA_HCI_API_DEALLOC_GATE           gate_dealloc;                   /* Deallocate the gate allocated to the application */
    tNFA_HCI_API_CREATE_PIPE_EVT        create_pipe;                    /* Create a pipe */
    tNFA_HCI_API_OPEN_PIPE_EVT          open_pipe;                      /* Open a pipe */
    tNFA_HCI_API_CLOSE_PIPE_EVT         close_pipe;                     /* Close a pipe */
    tNFA_HCI_API_DELETE_PIPE_EVT        delete_pipe;                    /* Delete a pipe */
    tNFA_HCI_API_GET_HOST_LIST          get_host_list;                  /* Get the list of Host in the network */
    tNFA_HCI_API_GET_REGISTRY           get_registry;                   /* Get a registry entry on a host */
    tNFA_HCI_API_SET_REGISTRY           set_registry;                   /* Set a registry entry on a host */
    tNFA_HCI_API_SEND_CMD_EVT           send_cmd;                       /* Send a event on a pipe to a host */
    tNFA_HCI_API_SEND_RSP_EVT           send_rsp;                       /* Response to a command sent on a pipe to a host */
    tNFA_HCI_API_SEND_EVENT_EVT         send_evt;                       /* Send a command on a pipe to a host */

    /* Internal events */
    tNFA_HCI_RSP_NV_READ_EVT            nv_read;
    tNFA_HCI_RSP_NV_WRITE_EVT           nv_write;
    tNFA_HCI_VSC_INIT_EVT               vsc_init;
    tNFA_HCI_EE_DISC_CMPLT_EVT          ee_disc_cmplt;
    tNFA_HCI_HCP_CONN_CREATE_EVT        conn_create;
} tNFA_HCI_EVENT_DATA;

/* type for State Machine (SM) action functions */
typedef void (*tNFA_HCI_SM_ACT) (tNFA_HCI_EVENT_DATA *p_data);
/*****************************************************************************
**  control block
*****************************************************************************/

/* Dynamic pipe control block */
typedef struct
{
    UINT8                   pipe_id;                /* Pipe ID */
    tNFA_HCI_PIPE_STATE     pipe_state;             /* State of the Pipe */
    UINT8                   local_gate;             /* local gate id */
    UINT8                   dest_host;              /* Peer host to which this pipe is connected */
    UINT8                   dest_gate;              /* Peer gate to which this pipe is connected */
} tNFA_HCI_DYN_PIPE;

/* Dynamic gate control block */
typedef struct
{
    UINT8                   gate_id;                /* local gate id */
    tNFA_HANDLE             gate_owner;             /* NFA-HCI handle assigned to the application which owns the gate */ 
    UINT32                  pipe_inx_mask;          /* Bit 0 == pipe inx 0, etc */
} tNFA_HCI_DYN_GATE;

/* Admin gate control block */
typedef struct
{
    tNFA_HCI_PIPE_STATE pipe01_state;               /* State of Pipe '01' */
    UINT64              session_id;                 /* Session ID of the host network */
} tNFA_ADMIN_GATE_INFO;

/* Link management gate control block */
typedef struct
{
    tNFA_HCI_PIPE_STATE pipe00_state;               /* State of Pipe '00' */
    UINT16              rec_errors;                 /* Receive errors */
} tNFA_LINK_MGMT_GATE_INFO;

/* Identity management gate control block */
typedef struct
{
    UINT32              pipe_inx_mask;                  /* Bit 0 == pipe inx 0, etc */
    UINT16              version_sw;                     /* Software version number */
    UINT16              version_hw;                     /* Hardware version number */
    UINT8               vendor_name[20];                /* Vendor name */
    UINT8               model_id;                       /* Model ID */
    UINT8               hci_version;                    /* HCI Version */
} tNFA_ID_MGMT_GATE_INFO;

/* Register application control block */
typedef struct
{
    tNFA_HCI_CBACK      *p_app_cback;                   /* Callback function registered by the application */
    UINT16              buf_size;                       /* Maximum size of APDU buffer */
    UINT8               *p_evt_buf;                     /* Buffer to hold reassembled event */
} tNFA_HCI_APP_INFO;

/* Internal flags */
#define NFA_HCI_FL_DISABLING        0x01                /* sub system is being disabled */
#define NFA_HCI_FL_NV_CHANGED       0x02                /* NV Ram changed */


/* NFA HCI control block */
typedef struct
{
    tNFA_HCI_STATE                  hci_state;                          /* state of the HCI */
    UINT8                           num_nfcee;
    UINT8                           inactive_host[NFA_HCI_MAX_HOST_IN_NETWORK]; /* Inactive host in the host network */
    BOOLEAN                         b_low_power_mode;                   /* Host controller in low power mode */
    BOOLEAN                         b_hci_netwk_reset;                  /* Command sent to reset HCI Network */
    BOOLEAN                         w4_hci_netwk_init;                  /* Wait for other host in network to initialize */
    TIMER_LIST_ENT                  timer;                              /* Timer to avoid indefinitely waiting for response */
    UINT8                           conn_id;                            /* Connection ID */
    UINT8                           buff_size;                          /* Connection buffer size */
    BOOLEAN                         nv_read_cmplt;                      /* NV Read completed */
    BOOLEAN                         w4_vsc_init;                        /* Wait for VSC initialization */
    BOOLEAN                         nv_write_needed;                    /* Something changed - NV write is needed */
    BOOLEAN                         assembling;                         /* Set true if in process of assembling a message  */
    tNFA_HANDLE                     app_in_use;                         /* ??? Index of the application that is waiting for response */
    UINT8                           local_gate_in_use;                  /* Local gate currently working with */
    UINT8                           remote_gate_in_use;                 /* Remote gate currently working with */
    UINT8                           remote_host_in_use;                 /* The remote host to which a command is sent */
    UINT8                           pipe_in_use;                        /* The pipe currently working with */
    UINT8                           param_in_use;                       /* The registry parameter currently working with */
    tNFA_HCI_COMMAND                cmd_sent;                           /* The last command sent */
    BOOLEAN                         ee_disc_cmplt;                      /* EE Discovery operation completed */
    UINT16                          msg_len;                            /* For segmentation - length of the combined message */
    UINT16                          max_msg_len;                        /* Maximum reassembled message size */
    UINT8                           msg_data[NFA_MAX_HCI_EVENT_LEN];    /* For segmentation - the combined message data */
    UINT8                           *p_msg_data;                        /* For segmentation - reassembled message */
    UINT8                           type;                               /* Instruction type of incoming message */
    UINT8                           inst;                               /* Instruction of incoming message */

    BUFFER_Q                        hci_api_q;                          /* Buffer Q to hold incoming API commands */
    tNFA_HCI_SM_ACT                 p_vs_evt_hdlr;                      /* vendor specific event handler    */
    tNFA_HCI_APP_INFO               app_info[NFA_HCI_MAX_APP_CB];       /* Application specific information */
    struct
    {
        char                        reg_app_names[NFA_HCI_MAX_APP_CB][NFA_MAX_HCI_APP_NAME_LEN + 1];

        tNFA_HCI_DYN_GATE           dyn_gates[NFA_HCI_MAX_GATE_CB];
        tNFA_HCI_DYN_PIPE           dyn_pipes[NFA_HCI_MAX_PIPE_CB];

        BOOLEAN                     b_send_conn_evts[NFA_HCI_MAX_APP_CB];
        tNFA_ADMIN_GATE_INFO        admin_gate;
        tNFA_LINK_MGMT_GATE_INFO    link_mgmt_gate;
        tNFA_ID_MGMT_GATE_INFO      id_mgmt_gate;
    } cfg;

} tNFA_HCI_CB;


/*****************************************************************************
**  External variables
*****************************************************************************/

/* NFA HCI control block */
#if NFA_DYNAMIC_MEMORY == FALSE
extern tNFA_HCI_CB nfa_hci_cb;
#else
extern tNFA_HCI_CB *nfa_hci_cb_ptr;
#define nfa_hci_cb (*nfa_hci_cb_ptr)
#endif


/*****************************************************************************
**  External functions
*****************************************************************************/

/* Functions in nfa_hci_main.c
*/
extern void nfa_hci_init (void);
extern void nfa_hci_proc_nfcc_power_mode (UINT8 nfcc_power_mode);
extern void nfa_hci_dh_startup_complete (void);
extern void nfa_hci_startup_complete (tNFA_STATUS status);
extern void nfa_hci_startup (void);
extern void nfa_hci_restore_default_config (UINT64 session_id);
extern void nfa_hci_vsc_cback (tNFC_VS_EVT event, UINT16 data_len, UINT8 *p_data);

/* Action functions in nfa_hci_act.c
*/
extern void nfa_hci_check_api_requests (void);
extern void nfa_hci_handle_admin_gate_cmd (UINT8 *p_data, UINT8 data_len);
extern void nfa_hci_handle_admin_gate_rsp (UINT8 *p_data, UINT8 data_len);
extern void nfa_hci_handle_admin_gate_evt (UINT8 *p_data, UINT8 data_len);
extern void nfa_hci_handle_link_mgm_gate_cmd (UINT8 *p_data, UINT8 data_len);
extern void nfa_hci_handle_dyn_pipe_pkt (UINT8 pipe, UINT8  *p_data, UINT8 type, UINT16 data_len);
extern void nfa_hci_handle_pipe_open_close_cmd (UINT8 instruction, tNFA_HCI_DYN_PIPE *p_pipe);
extern void nfa_hci_api_dealloc_gate (tNFA_HCI_EVENT_DATA *p_evt_data);
extern void nfa_hci_api_deregister (tNFA_HCI_EVENT_DATA *p_evt_data);

/* Utility functions in nfa_hci_utils.c
*/
extern tNFA_HCI_DYN_GATE  *nfa_hciu_alloc_gate (UINT8 gate_id, tNFA_HANDLE app_handle);
extern tNFA_HCI_DYN_GATE  *nfa_hciu_find_gate_by_gid (UINT8 gate_id);
extern tNFA_HCI_DYN_GATE  *nfa_hciu_find_gate_by_owner (tNFA_HANDLE app_handle);
extern tNFA_HCI_DYN_GATE  *nfa_hciu_find_gate_with_nopipes_by_owner (tNFA_HANDLE app_handle);
extern tNFA_HCI_DYN_PIPE  *nfa_hciu_find_pipe_by_pid (UINT8 pipe_id);
extern tNFA_HCI_DYN_PIPE  *nfa_hciu_find_pipe_by_owner (tNFA_HANDLE app_handle);
extern tNFA_HCI_DYN_PIPE  *nfa_hciu_find_active_pipe_by_owner (tNFA_HANDLE app_handle);
extern tNFA_HCI_DYN_PIPE  *nfa_hciu_find_pipe_on_gate (UINT8 gate_id);
extern tNFA_HANDLE         nfa_hciu_get_gate_owner (UINT8 gate_id);
extern BOOLEAN             nfa_hciu_is_active_host (UINT8 host_id);
extern tNFA_HCI_DYN_PIPE  *nfa_hciu_find_active_pipe_on_gate (UINT8 gate_id);
extern tNFA_HANDLE         nfa_hciu_get_pipe_owner (UINT8 pipe_id);
extern UINT8               nfa_hciu_count_open_pipes_on_gate (tNFA_HCI_DYN_GATE *p_gate);
extern UINT8               nfa_hciu_count_pipes_on_gate (tNFA_HCI_DYN_GATE *p_gate);
extern tNFA_STATUS         nfa_hciu_asmbl_dyn_pipe_pkt (UINT8 *p_data, UINT8 data_len);

extern tNFA_HCI_RESPONSE   nfa_hciu_add_pipe_to_gate (UINT8 pipe, UINT8 local_gate, UINT8 dest_host, UINT8 dest_gate);
extern tNFA_HCI_RESPONSE   nfa_hciu_add_pipe_to_static_gate (UINT8 local_gate, UINT8 pipe_id, UINT8 dest_host, UINT8 dest_gate);

extern tNFA_HCI_RESPONSE   nfa_hciu_release_pipe (UINT8 pipe_id);
extern void                nfa_hciu_release_gate (UINT8 gate);
extern void                nfa_hciu_remove_all_pipes_from_host (UINT8 host);
extern UINT8               nfa_hciu_get_allocated_gate_list (UINT8 *p_gate_list);

extern void                nfa_hciu_send_to_app (tNFA_HCI_EVT event, tNFA_HCI_EVT_DATA *p_evt, tNFA_HANDLE app_handle);
extern void                nfa_hciu_send_to_all_apps (tNFA_HCI_EVT event, tNFA_HCI_EVT_DATA *p_evt);
extern void                nfa_hciu_send_to_apps_handling_connectivity_evts (tNFA_HCI_EVT event, tNFA_HCI_EVT_DATA *p_evt);

extern tNFA_STATUS nfa_hciu_send_close_pipe_cmd (UINT8 pipe);
extern tNFA_STATUS nfa_hciu_send_delete_pipe_cmd (UINT8 pipe);
extern tNFA_STATUS nfa_hciu_send_clear_all_pipe_cmd (void);
extern tNFA_STATUS nfa_hciu_send_open_pipe_cmd (UINT8 pipe);
extern tNFA_STATUS nfa_hciu_send_get_param_cmd (UINT8 pipe, UINT8 index);
extern tNFA_STATUS nfa_hciu_send_create_pipe_cmd (UINT8 source_gate, UINT8 dest_host, UINT8 dest_gate);
extern tNFA_STATUS nfa_hciu_send_set_param_cmd (UINT8 pipe, UINT8 index, UINT8 length, UINT8 *p_data);
extern tNFA_STATUS nfa_hciu_send_msg (UINT8 pipe_id, UINT8 type, UINT8 instruction, UINT16 pkt_len, UINT8 *p_pkt);



#if (BT_TRACE_VERBOSE == TRUE)
extern char *nfa_hciu_type_2_str (UINT8 type);
extern char *nfa_hciu_instr_2_str (UINT8 type);
extern char *nfa_hciu_get_event_name (UINT16 event);
extern char *nfa_hciu_get_response_name (UINT8 rsp_code);
extern char *nfa_hciu_get_state_name (UINT8 state);
extern char *nfa_hciu_get_type_inst_names (UINT8 pipe, UINT8 type, UINT8 inst);
extern char *nfa_hciu_evt_2_str (UINT8 pipe_id, UINT8 evt);
#endif


#endif /* NFA_HCI_INT_H */
