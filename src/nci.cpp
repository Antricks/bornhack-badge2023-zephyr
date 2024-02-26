#include <string.h>

#include "config.h"
#include "nci.h"
#include "util.h"

void print_rf_technology(const uint8_t technology) {
    switch (technology) {
    case NFC_RF_TECHNOLOGY_A: printf("NFC-A"); break;
    case NFC_RF_TECHNOLOGY_B: printf("NFC-B"); break;
    case NFC_RF_TECHNOLOGY_F: printf("NFC-F"); break;
    case NFC_RF_TECHNOLOGY_V: printf("NFC-V"); break;
    default: printf("[unknown RF technology 0x%02x]", technology); break;
    }
}

void print_technology_mode(const uint8_t technology_mode) {
    switch (technology_mode) {
    case NFC_A_PASSIVE_POLL_MODE: printf("NFC-A passive poll mode"); break;
    case NFC_B_PASSIVE_POLL_MODE: printf("NFC-B passive poll mode"); break;
    case NFC_F_PASSIVE_POLL_MODE: printf("NFC-F passive poll mode"); break;
    case NFC_ACTIVE_POLL_MODE: printf("NFC active poll mode"); break;
    case NFC_V_PASSIVE_POLL_MODE: printf("NFC-V passivle poll mode"); break;
    case NFC_A_PASSIVE_LISTEN_MODE: printf("NFC-A passive listen mode"); break;
    case NFC_B_PASSIVE_LISTEN_MODE: printf("NFC-B passive listen mode"); break;
    case NFC_F_PASSIVE_LISTEN_MODE: printf("NFC-F passive listen mode"); break;
    case NFC_ACTIVE_LISTEN_MODE: printf("NFC active listen mode"); break;
    default: printf("[unknown NFC technology / mode 0x%02x]", technology_mode); break;
    }
}

void print_bitrate(const uint8_t bitrate) {
    switch (bitrate) {
    case NFC_BIT_RATE_106: printf("106 kbit/s"); break;
    case NFC_BIT_RATE_212: printf("212 kbit/s"); break;
    case NFC_BIT_RATE_424: printf("424 kbit/s"); break;
    case NFC_BIT_RATE_848: printf("848 kbit/s"); break;
    case NFC_BIT_RATE_1695: printf("1695 kbit/s"); break;
    case NFC_BIT_RATE_3390: printf("3390 kbit/s"); break;
    case NFC_BIT_RATE_6780: printf("6780 kbit/s"); break;
    case NFC_BIT_RATE_26: printf("26 kbit/s"); break;
    default: printf("[unknown bitrate 0x%02x]", bitrate); break;
    }
}

void print_rf_protocol(const uint8_t protocol) {
    switch (protocol) {
    case RF_PROTO_UNDETERMINED: printf("RF_PROTO_UNDETERMINED"); break;
    case RF_PROTO_T1T_DEPRECATED: printf("T1T (deprecated)"); break;
    case RF_PROTO_T2T: printf("T2T"); break;
    case RF_PROTO_T3T: printf("T3T"); break;
    case RF_PROTO_ISO_DEP: printf("ISO-DEP"); break;
    case RF_PROTO_NFC_DEP: printf("NFC-DEP"); break;
    case RF_PROTO_T5T: printf("T5T"); break;
    case RF_PROTO_NDEF: printf("NDEF"); break;
    case RF_PROTO_WLC: printf("WLC"); break;
    default: printf("[unknown protocol 0x%02x]", protocol); break;
    }
}

void print_rf_interface(const uint8_t intf) {
    switch (intf) {
    case RF_INTF_NFCEE_DIRECT: printf("NFCEE direct"); break;
    case RF_INTF_FRAME: printf("Frame RF"); break;
    case RF_INTF_ISO_DEP: printf("ISO-DEP"); break;
    case RF_INTF_NFC_DEP: printf("NFC-DEP"); break;
    case RF_INTF_NDEF: printf("NDEF"); break;
    case RF_INTF_WLC_P_AUTONOMOUS: printf("WLC-P autonomous"); break;
    default: printf("[unknown RF interface 0x%02x]", intf); break;
    }
}

void print_status(const uint8_t status) {
    switch (status) {
    case STATUS_OK: printf("OK"); break;
    case STATUS_REJECTED: printf("REJECTED"); break;
    case STATUS_FAILED: printf("FAILED"); break;
    case STATUS_NOT_INITIALIZED: printf("NOT_INITIALIZED"); break;
    case STATUS_SYNTAX_ERROR: printf("SYNTAX_ERROR"); break;
    case STATUS_SEMANTIC_ERROR: printf("SEMANTIC_ERROR"); break;
    case STATUS_INVALID_PARAM: printf("INVALID_PARAM"); break;
    case STATUS_MESSAGE_SIZE_EXCEEDED: printf("MESSAGE_SIZE_EXCEEDED"); break;
    case STATUS_OK_1_BIT: printf("OK_1_BIT"); break;
    case STATUS_OK_2_BIT: printf("OK_2_BIT"); break;
    case STATUS_OK_3_BIT: printf("OK_3_BIT"); break;
    case STATUS_OK_4_BIT: printf("OK_4_BIT"); break;
    case STATUS_OK_5_BIT: printf("OK_5_BIT"); break;
    case STATUS_OK_6_BIT: printf("OK_6_BIT"); break;
    case STATUS_OK_7_BIT: printf("OK_7_BIT"); break;
    case STATUS_DISCOVERY_ALREADY_STARTED: printf("DISCOVERY_ALREADY_STARTED"); break;
    case STATUS_DISCOVERY_TARGET_ACTIVATION_FAILED: printf("DISCOVERY_TARGET_ACTIVATION_FAILED"); break;
    case STATUS_DISCOVERY_TEAR_DOWN: printf("DISCOVERY_TEAR_DOWN"); break;
    case STATUS_RF_FRAME_CORRUPTED: printf("RF_FRAME_CORRUPTED"); break;
    case STATUS_RF_TRANSMISSION_EXCEPTION: printf("RF_TRANSMISSION_EXCEPTION"); break;
    case STATUS_RF_PROTOCOL_EXCEPTION: printf("RF_PROTOCOL_EXCEPTION"); break;
    case STATUS_RF_TIMEOUT_EXCEPTION: printf("RF_TIMEOUT_EXCEPTION"); break;
    case STATUS_RF_UNEXPECTED_DATA: printf("RF_UNEXPECTED_DATA"); break;
    case STATUS_NFCEE_INTERFACE_ACTIVATION_FAILED: printf("NFCEE_INTERFACE_ACTIVATION_FAILED"); break;
    case STATUS_NFCEE_TRANSMISSION_ERROR: printf("NFCEE_TRANSMISSION_ERROR"); break;
    case STATUS_NFCEE_PROTOCOL_ERROR: printf("NFCEE_PROTOCOL_ERROR"); break;
    case STATUS_NFCEE_TIMEOUT_ERROR: printf("NFCEE_TIMEOUT_ERROR"); break;
    default: printf("[unknown status 0x%02x]", status); break;
    }
}

struct nci_control_msg {
    uint8_t mt;
    bool pkg_boundary_flag;
    uint8_t gid;
    uint8_t oid;
    uint8_t payload_len;
    uint8_t payload[256]; // TODO this is not really exactly nice but I don't know how else to do this...
    struct nci_control_msg *next;
};

struct nci_data_msg {
    bool pkg_boundary_flag;
    uint8_t conn_id;
    uint8_t credits;
    uint8_t payload_len;
    uint8_t payload[256]; // TODO this is not really exactly nice but I don't know how else to do this...
    struct nci_data_msg *next;
};

// returns the expected overall length of an NCI packet according to packet header
size_t expected_packet_length(const uint8_t *packet) {
    return 3 + packet[2];
}

Nci::Nci() {}
Nci::~Nci() {}

struct nci_control_msg Nci::nci_parse_control_msg_standalone(const uint8_t *msg_buf) {
    uint8_t payload_len = msg_buf[2];
    struct nci_control_msg msg = (struct nci_control_msg){
        .pkg_boundary_flag = (msg_buf[0] & 0x10) != 0,
        .gid = (uint8_t)(msg_buf[0] & 0x0f),
        .oid = msg_buf[1],
        .payload_len = payload_len,
        .payload = {0},
        .next = nullptr,
    };
    memcpy(&msg.payload[0], &msg_buf[3], payload_len);
    return msg;
}

struct nci_data_msg Nci::nci_parse_data_msg_standalone(const uint8_t *msg_buf) {
    uint8_t payload_len = msg_buf[2];
    struct nci_data_msg msg = (struct nci_data_msg){
        .pkg_boundary_flag = (msg_buf[0] & 0x10) != 0,
        .conn_id = (uint8_t)(msg_buf[0] & 0x0f),
        .credits = (uint8_t)(msg_buf[1] & 0x03),
        .payload_len = payload_len,
        .payload = {0},
        .next = nullptr,
    };
    memcpy(&msg.payload[0], &msg_buf[3], payload_len);
    return msg;
}

int Nci::nci_write(const uint8_t *cmd) {
    int ret = transport_write(cmd, expected_packet_length(cmd));
    if (ret) {
        printf("transport_write: %i\n", ret);
        return ret;
    }
#if DEBUG_NCI_HEXDUMP
    // hexdump("> ", cmd, expected_packet_length(cmd));
    hexdump("> ", cmd, expected_packet_length(cmd));
    puts("");
#endif
#if DEBUG_NCI_ANALYSIS
    nci_debug(cmd);
#endif
    return 0;
}

int Nci::nci_read() {
    int ret = transport_read();
    if (ret) {
        printf("transport_read: %i\n", ret);
        return ret;
    }
#if DEBUG_NCI_HEXDUMP
    // hexdump("< ", this->read_buf, expected_packet_length(this->read_buf));
    hexdump("< ", this->read_buf, expected_packet_length(this->read_buf));
    puts("");
#endif
#if DEBUG_NCI_ANALYSIS
    nci_debug(this->read_buf);
#endif
    return 0;
}

int Nci::nci_write_read(const uint8_t *cmd) {
    int ret = 0;

    ret = this->nci_write(cmd);
    if (ret) {
        return ret;
    }

    while (!transport_ready_to_read()) {
        k_sleep(K_MSEC(50));
    }

    ret = nci_read();
    if (ret)
        return ret;

    return 0;
}

// NOTE This might become a full on handler function later.
// Theoretically it should not need it but a flag to deactivate debug messages & actual handling
// could be nice in the future.
// This should also take a representation of current context (state machine, parameters, ...)
void Nci::nci_debug(const uint8_t *msg_buf) {
    uint8_t mt = msg_buf[0] & 0xe0; // this should also maybe be abstracted.
    if (mt == MT_CMD || mt == MT_RSP || mt == MT_NTF) {
        struct nci_control_msg msg = nci_parse_control_msg_standalone(msg_buf);
        if (msg.pkg_boundary_flag) {
            printf("[PBF] ");
        }
        if (mt == MT_CMD) {
            // control message
            switch (msg.gid) {
            case CMD_GID_CORE:
                switch (msg.oid) {
                case CORE_RESET: printf("CORE_RESET_CMD\n"); break;                             // TODO details
                case CORE_INIT: printf("CORE_INIT_CMD\n"); break;                               // TODO details
                case CORE_SET_CONFIG: printf("CORE_SET_CONFIG_CMD\n"); break;                   // TODO details
                case CORE_GET_CONFIG: printf("CORE_GET_CONFIG_CMD\n"); break;                   // TODO details
                case CORE_CONN_CREATE: printf("CORE_CONN_CREATE_CMD\n"); break;                 // TODO details
                case CORE_CONN_CLOSE: printf("CORE_CONN_CLOSE_CMD\n"); break;                   // TODO details
                case CORE_SET_POWER_SUB_STATE: printf("CORE_SET_POWER_SUB_STATE_CMD\n"); break; // TODO details
                default: printf("[WARN] Command unknown for Core OID: 0x%02x\n", msg.oid); break;
                }
                break;
            case CMD_GID_RF:
                switch (msg.oid) {
                case RF_DISCOVER_MAP: printf("RF_DISCOVER_MAP_CMD\n"); break;                         // TODO details
                case RF_SET_LISTEN_MODE_ROUTING: printf("RF_SET_LISTEN_MODE_ROUTING_CMD\n"); break;   // TODO details
                case RF_GET_LISTEN_MODE_ROUTING: printf("RF_GET_LISTEN_MODE_ROUTING_CMD\n"); break;   // TODO details
                case RF_DISCOVER: printf("RF_DISCOVER_CMD\n"); break;                                 // TODO details
                case RF_DISCOVER_SELECT: printf("RF_DISCOVER_SELECT_CMD\n"); break;                   // TODO details
                case RF_DEACTIVATE: printf("RF_DEACTIVATE_CMD\n"); break;                             // TODO details
                case RF_T3T_POLLING: printf("RF_T3T_POLLING_CMD\n"); break;                           // TODO details
                case RF_PARAMETER_UPDATE: printf("RF_PARAMETER_UPDATE_CMD\n"); break;                 // TODO details
                case RF_INTF_EXT_START: printf("RF_INTF_EXT_START_CMD\n"); break;                     // TODO details
                case RF_INTF_EXT_STOP: printf("RF_INTF_EXT_STOP_CMD\n"); break;                       // TODO details
                case RF_EXT_AGG_ABORT: printf("RF_EXT_AGG_ABORT_CMD\n"); break;                       // TODO details
                case RF_NDEF_ABORT: printf("RF_NDEF_ABORT_CMD\n"); break;                             // TODO details
                case RF_ISO_DEP_NAK_PRESENCE: printf("RF_ISO_DEP_NAK_PRESENCE_CMD\n"); break;         // TODO details
                case RF_SET_FORCED_NFCEE_ROUTING: printf("RF_SET_FORCED_NFCEE_ROUTING_CMD\n"); break; // TODO details
                case RF_REMOVAL_DETECTION: printf("RF_REMOVAL_DETECTION_CMD\n"); break;               // TODO details
                case GET_WLCP_INFO_PARAM: printf("RF_GET_WLCP_INFO_PARAM_CMD\n"); break;              // TODO details
                case WPT_START: printf("RF_WPT_START_CMD\n"); break;                                  // TODO details
                default: printf("[WARN] Command unknown for RF management OID: 0x%02x\n", msg.oid); break;
                }
                break;
            case CMD_GID_NFCEE_MGMT:
                switch (msg.oid) {
                case NFCEE_DISCOVER: printf("NFCEE_DISCOVER_CMD\n"); break;                         // TODO details
                case NFCEE_MODE_SET: printf("NFCEE_MODE_SET_CMD\n"); break;                         // TODO details
                case NFCEE_STATUS: printf("NFCEE_STATUS_CMD\n"); break;                             // TODO details
                case NFCEE_POWER_AND_LINK_CNTRL: printf("NFCEE_POWER_AND_LINK_CNTRL_CMD\n"); break; // TODO details
                default: printf("[WARN] Command unknown for NFCEE management OID: 0x%02x\n", msg.oid); break;
                }
                break;
            case CMD_GID_NFCC_MGMT: printf("No NFCC management commands known.\n"); break;
            case CMD_GID_TEST_MGMT: printf("No test management commands known.\n"); break;
            default: printf("[WARN] NCI control GID 0x%01x unknown.\n", msg_buf[0] & 0xf); break;
            }
        } else if (mt == MT_RSP) {
            switch (msg.gid) {
            case CMD_GID_CORE:
                switch (msg.oid) {
                case CORE_RESET:
                    printf("CORE_RESET_RSP\n");
                    break; // TODO details (has different format than specified in practice... weird...)
                case CORE_INIT:
                    printf("CORE_INIT_RSP (");
                    print_status(msg.payload[0]);
                    printf(")\n");
                    if (msg.payload[0] != STATUS_OK)
                        break;
                    printf("\tNFCC Features: ");
                    hexdump(&msg.payload[1], 4); // TODO make human readable
                    printf("\n");
                    printf("\tMax Logical Connections: %i\n", msg.payload[5]);
                    printf("\tMax Routing Table Size: %i\n", *((uint16_t *)(&msg.payload[6])));
                    printf("\tMax Control Packet Payload Size: %i\n", msg.payload[8]);
                    printf("\tMax Data Packet Payload Size: %i\n", msg.payload[9]);
                    printf("\tHCI Credits: %i\n", msg.payload[10]);
                    // NOTE values bigger than one octet are encoded in little endian.
                    printf("\tMax NFC-V RF Frame Size: %i\n", *((uint16_t *)(&msg.payload[11])));
                    printf("\tNumber of Supported RF Interfaces: %i\n", msg.payload[13]);
                    printf("\tSupported RF Interfaces: ");
                    hexdump(&msg.payload[14], msg.payload_len - 14); // TODO make human readable
                    printf("\n");
                    break; // TODO the data coming from this in practice doesn't seem right. But I think this code
                           // behaves as specified...
                case CORE_SET_CONFIG:
                    printf("CORE_SET_CONFIG_RSP (");
                    print_status(msg.payload[0]);
                    printf(")\n");
                    if (msg.payload[0] == STATUS_OK)
                        break;
                    printf("\tInvalid parameters (%i): ", msg.payload[1]);
                    hexdump(&msg.payload[2], msg.payload[1]); // TODO make human readable
                    printf("\n");
                    break;
                case CORE_GET_CONFIG: printf("CORE_GET_CONFIG_RSP\n"); break;                   // TODO details
                case CORE_CONN_CREATE: printf("CORE_CONN_CREATE_RSP\n"); break;                 // TODO details
                case CORE_CONN_CLOSE: printf("CORE_CONN_CLOSE_RSP\n"); break;                   // TODO details
                case CORE_SET_POWER_SUB_STATE: printf("CORE_SET_POWER_SUB_STATE_RSP\n"); break; // TODO details
                default: printf("[WARN] Response unknown for Core OID: 0x%02x\n", msg.oid); break;
                }
                break;
            case CMD_GID_RF:
                switch (msg.oid) {
                case RF_DISCOVER_MAP:
                    printf("RF_DISCOVER_MAP_RSP (");
                    print_status(msg.payload[0]);
                    printf(")\n");
                    break;
                case RF_SET_LISTEN_MODE_ROUTING:
                    printf("RF_SET_LISTEN_MODE_ROUTING_RSP (");
                    print_status(msg.payload[0]);
                    printf(")\n");
                    break;
                case RF_GET_LISTEN_MODE_ROUTING:
                    printf("RF_GET_LISTEN_MODE_ROUTING_RSP (");
                    print_status(msg.payload[0]);
                    printf(")\n");
                    break;
                case RF_DISCOVER:
                    printf("RF_DISCOVER_RSP (");
                    print_status(msg.payload[0]);
                    printf(")\n");
                    break;
                case RF_DISCOVER_SELECT: printf("RF_DISCOVER_SELECT_RSP\n"); break;                   // TODO details
                case RF_DEACTIVATE: printf("RF_DEACTIVATE_RSP\n"); break;                             // TODO details
                case RF_T3T_POLLING: printf("RF_T3T_POLLING_RSP\n"); break;                           // TODO details
                case RF_PARAMETER_UPDATE: printf("RF_PARAMETER_UPDATE_RSP\n"); break;                 // TODO details
                case RF_INTF_EXT_START: printf("RF_INTF_EXT_START_RSP\n"); break;                     // TODO details
                case RF_INTF_EXT_STOP: printf("RF_INTF_EXT_STOP_RSP\n"); break;                       // TODO details
                case RF_EXT_AGG_ABORT: printf("RF_EXT_AGG_ABORT_RSP\n"); break;                       // TODO details
                case RF_NDEF_ABORT: printf("RF_NDEF_ABORT_RSP\n"); break;                             // TODO details
                case RF_ISO_DEP_NAK_PRESENCE: printf("RF_ISO_DEP_NAK_PRESENCE_RSP\n"); break;         // TODO details
                case RF_SET_FORCED_NFCEE_ROUTING: printf("RF_SET_FORCED_NFCEE_ROUTING_RSP\n"); break; // TODO details
                case RF_REMOVAL_DETECTION: printf("RF_REMOVAL_DETECTION_RSP\n"); break;               // TODO details
                case GET_WLCP_INFO_PARAM: printf("RF_GET_WLCP_INFO_PARAM_RSP\n"); break;              // TODO details
                case WPT_START: printf("RF_WPT_START_RSP\n"); break;                                  // TODO details
                default: printf("[WARN] Response unknown for RF management OID: 0x%02x\n", msg.oid); break;
                }
                break;
            case CMD_GID_NFCEE_MGMT:
                switch (msg.oid) {
                case NFCEE_DISCOVER: printf("NFCEE_DISCOVER_RSP\n"); break;                         // TODO details
                case NFCEE_MODE_SET: printf("NFCEE_MODE_SET_RSP\n"); break;                         // TODO details
                case NFCEE_STATUS: printf("NFCEE_STATUS_RSP\n"); break;                             // TODO details
                case NFCEE_POWER_AND_LINK_CNTRL: printf("NFCEE_POWER_AND_LINK_CNTRL_RSP\n"); break; // TODO details
                default: printf("[WARN] Response unknown for NFCEE management OID: 0x%02x\n", msg.oid); break;
                }
                break;
            case CMD_GID_NFCC_MGMT: printf("No NFCC management responses known.\n"); break;
            case CMD_GID_TEST_MGMT: printf("No test management responses known.\n"); break;
            default: printf("[WARN] NCI control GID 0x%01x unknown.\n", msg_buf[0] & 0xf); break;
            }
        } else if (mt == MT_NTF) {
            switch (msg.gid) {
            case CMD_GID_CORE:
                switch (msg.oid) {
                case CORE_RESET: printf("CORE_RESET_NTF\n"); break;                 // TODO details
                case CORE_CONN_CREDITS: printf("CORE_CONN_CREDITS_NTF\n"); break;   // TODO details
                case CORE_GENERIC_ERROR: printf("CORE_GENERIC_ERROR_NTF\n"); break; // TODO details
                case CORE_INTERFACE_ERROR:
                    printf("CORE_INTERFACE_ERROR_NTF (");
                    print_status(msg.payload[0]);
                    printf(") - Connection ID: %i\n", msg.payload[1] & 0x0f);
                    break;
                default: printf("[WARN] Notification unknown for Core OID: 0x%02x\n", msg.oid); break;
                }
                break;
            case CMD_GID_RF:
                switch (msg.oid) {
                case RF_GET_LISTEN_MODE_ROUTING: printf("RF_GET_LISTEN_MODE_ROUTING_NTF\n"); break; // TODO details
                case RF_DISCOVER: printf("RF_DISCOVER_NTF\n"); break;                               // TODO details
                case RF_INTF_ACTIVATED: {
                    printf("RF_INTF_ACTIVATED_NTF\n");
                    printf("\tRF Discovery ID: %02x\n", msg.payload[0]);
                    printf("\tRF Interface: ");
                    uint8_t rf_intf = msg.payload[1];
                    print_rf_interface(rf_intf);
                    printf("\n");
                    printf("\tRF Protocol: ");
                    uint8_t rf_proto = msg.payload[2];
                    print_rf_protocol(rf_proto);
                    printf("\n");
                    printf("\tActivation RF Technology and Mode: ");
                    print_technology_mode(msg.payload[3]);
                    printf("\n");
                    printf("\tMax Data Packet Payload Size: %i\n", msg.payload[4]);
                    printf("\tInitial Number of Credits: %i\n", msg.payload[5]);
                    uint8_t techno_params_len = msg.payload[6];
                    if (techno_params_len > 0) {
                        printf("\tTechnology specific params (%i bytes):\n\t\t", msg.payload[5]);
                        hexdump(&msg.payload[7], techno_params_len); // TODO make human readble
                        printf("\n");
                    } else {
                        printf("\tNo technology specific params.\n");
                    }
                    printf("\tData Exchange RF Technology and Mode: ");
                    uint8_t rf_techno_mode = msg.payload[7 + techno_params_len];
                    print_technology_mode(rf_techno_mode);
                    printf("\n");
                    printf("\tData Exchange Transmit Rate: ");
                    print_bitrate(msg.payload[8 + techno_params_len]);
                    printf("\n");
                    printf("\tData Exchange Receive Rate: ");
                    print_bitrate(msg.payload[9 + techno_params_len]);
                    printf("\n");
                    uint8_t activation_params_len = msg.payload[10 + techno_params_len];
                    if (activation_params_len > 0) {
                        printf("\tActivation Parameters (%i bytes):\n", activation_params_len);
                        if (rf_intf == RF_INTF_ISO_DEP) {
                            if (rf_techno_mode == NFC_A_PASSIVE_POLL_MODE) {
                                printf("\t\tRATS response (%i bytes): ", msg.payload[11 + techno_params_len]);
                                hexdump(&msg.payload[12 + techno_params_len], activation_params_len - 1);
                                printf("\n");
                            } else if (rf_techno_mode == NFC_B_PASSIVE_POLL_MODE) {
                                printf("\t\tATTRIB response (%i bytes): ", msg.payload[11 + techno_params_len]);
                                hexdump(&msg.payload[12 + techno_params_len],
                                        activation_params_len - 1); // TODO make human readable
                                printf("\n");
                            } else if (rf_techno_mode == NFC_A_PASSIVE_LISTEN_MODE) {
                                printf("\t\tRATS PARAM: %02x\n",
                                       msg.payload[11 + techno_params_len]); // TODO? make human readable
                            } else if (rf_techno_mode == NFC_B_PASSIVE_LISTEN_MODE) {
                                printf("\t\tATTRIB Command (%i bytes): ", msg.payload[11 + techno_params_len]);
                                hexdump(&msg.payload[12 + techno_params_len],
                                        activation_params_len - 1); // TODO make params human readable
                                const uint8_t *params = &msg.payload[16 + techno_params_len];
                                // TODO check for matching NFCID0
                                printf("\n\t\t\tParam 1 (0x%02x): ", params[0]);
                                printf("TR0: ");
                                switch ((params[0] & 0xc0) >> 6) {
                                case 0: printf("default"); break;
                                case 1: printf("48 * 16/fc"); break;
                                case 2: printf("16 * 16/fc"); break;
                                default: printf("unknown"); break;
                                }
                                printf(", TR1: ");
                                switch ((params[0] & 0x30) >> 4) {
                                case 0: printf("default"); break;
                                case 1: printf("48 * 16/fc"); break;
                                case 2: printf("16 * 16/fc"); break;
                                default: printf("unknown"); break;
                                }
                                if (params[0] & 0x08) {
                                    printf(", EoS supressed");
                                } else {
                                    printf(", EoS required");
                                }
                                if (params[0] & 0x04) {
                                    printf(", SoS supressed");
                                } else {
                                    printf(", SoS required");
                                }
                                printf("\n\t\t\tParam 2 (0x%02x): ", params[1]);
                                printf("bit rate D_listen->poll: ");
                                switch ((params[1] & 0xc0) >> 6) {
                                case 0: printf("1"); break;
                                case 1: printf("2"); break;
                                case 2: printf("4"); break;
                                case 3: printf("8"); break;
                                }
                                printf(", bit rate D_poll->listen: ");
                                switch ((params[1] & 0x30) >> 4) {
                                case 0: printf("1"); break;
                                case 1: printf("2"); break;
                                case 2: printf("4"); break;
                                case 3: printf("8"); break;
                                }
                                uint8_t fsdi = params[1] & 0x0f;
                                int fsd = 0;
                                switch (fsdi) {
                                case 0: fsd = 16; break;
                                case 1: fsd = 24; break;
                                case 2: fsd = 32; break;
                                case 3: fsd = 40; break;
                                case 4: fsd = 48; break;
                                case 5: fsd = 64; break;
                                case 6: fsd = 96; break;
                                case 7: fsd = 128; break;
                                case 8: fsd = 256; break;
                                }
                                printf(",\n\t\t\t\t maximum frame size (FSD): %i bytes", fsd);
                                printf("\n\t\t\tParam 3 (0x%02x): ", params[2]);
                                printf("Minimum TR2: ");
                                switch ((params[2] & 0x06) >> 1) {
                                case 0: printf("default"); break;
                                case 1: printf("48 * 16/fc"); break;
                                case 2: printf("16 * 16/fc"); break;
                                default: printf("unknown"); break;
                                }
                                if (params[2] & 0x01) {
                                    printf(",\n\t\t\t\t Device in Listen Mode ISO/IEC 14443 compliant.");
                                } else {
                                    printf(",\n\t\t\t\t Device in Listen Mode *NOT* ISO/IEC 14443 compliant.");
                                }
                                printf("\n\t\t\tParam 4 (0x%02x): DID: 0x%01x", params[3], params[3] & 0x0f);
                                printf("\n");
                            }
                        } else if (rf_intf == RF_INTF_NFC_DEP) {
                            if (rf_techno_mode == NFC_A_PASSIVE_POLL_MODE ||
                                rf_techno_mode == NFC_F_PASSIVE_POLL_MODE || rf_techno_mode == NFC_ACTIVE_POLL_MODE) {
                                uint8_t atr_res_len = msg.payload[11 + techno_params_len];
                                printf("\t\tATR_RES Response (%i bytes): ", atr_res_len);
                                hexdump(&msg.payload[12 + techno_params_len], atr_res_len);
                                printf("\n");
                                printf("\t\tData Exchange Length Reduction: %02x",
                                       msg.payload[12 + techno_params_len + atr_res_len]); // TODO make human readable
                            } else if (rf_techno_mode == NFC_A_PASSIVE_LISTEN_MODE ||
                                       rf_techno_mode == NFC_F_PASSIVE_LISTEN_MODE ||
                                       rf_techno_mode == NFC_ACTIVE_LISTEN_MODE) {
                                printf("\t\t"); // TODO implement
                            }
                        }
                    } else {
                        printf("\tNo activation parameters.");
                    }
                } break;
                case RF_DEACTIVATE: printf("RF_DEACTIVATE_NTF\n"); break;
                case RF_FIELD_INFO:
                    printf("RF_FIELD_INFO_NTF ");
                    if (msg.payload[0] & 0x01) {
                        printf("(Operating Field generated by Remote NFC Endpoint)\n");
                    } else {
                        printf("(No Operating Field generated by Remote NFC Endpoint)\n");
                    }
                    break;
                case RF_T3T_POLLING: printf("RF_T3T_POLLING_NTF\n"); break;                   // TODO details
                case RF_NFCEE_ACTION: printf("RF_NFCEE_ACTION_NTF\n"); break;                 // TODO details
                case RF_NFCEE_DISCOVERY_REQ: printf("RF_NFCEE_DISCOVERY_REQ_NTF\n"); break;   // TODO details
                case RF_ISO_DEP_NAK_PRESENCE: printf("RF_ISO_DEP_NAK_PRESENCE_NTF\n"); break; // TODO details
                case RF_REMOVAL_DETECTION: printf("RF_REMOVAL_DETECTION_NTF\n"); break;       // TODO details
                case RF_WLC_STATUS: printf("RF_WLC_STATUS_NTF\n"); break;                     // TODO details
                case GET_WLCP_INFO_PARAM: printf("GET_WLCP_INFO_PARAM_NTF\n"); break;         // TODO details
                case WPT_START: printf("WPT_START_NTF\n"); break;                             // TODO details
                default: printf("[WARN] Notification unknown for RF management OID: 0x%02x\n", msg.oid); break;
                }
                break;
            case CMD_GID_NFCEE_MGMT:
                switch (msg.oid) {
                case NFCEE_DISCOVER: printf("NFCEE_DISCOVER_NTF\n"); break; // TODO details
                case NFCEE_MODE_SET: printf("NFCEE_MODE_SET_NTF\n"); break; // TODO details
                case NFCEE_STATUS: printf("NFCEE_STATUS_NTF\n"); break;     // TODO details
                default: printf("[WARN] Notification unknown for NFCEE management OID: 0x%02x\n", msg.oid); break;
                }
                break;
            case CMD_GID_NFCC_MGMT: printf("No NFCC management notifications known.\n"); break;
            case CMD_GID_TEST_MGMT: printf("No test management notifications known.\n"); break;
            default: printf("[WARN] NCI control GID 0x%01x unknown.\n", msg_buf[0] & 0xf); break;
            }
        }
    } else {
        struct nci_data_msg msg = nci_parse_data_msg_standalone(msg_buf);
        if (msg.pkg_boundary_flag) {
            printf("[PBF] ");
        }
        printf("Data message (Conn ID: 0x%01x; %i credits)\n", msg.conn_id, msg.credits);
    }
}
