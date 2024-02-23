#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include "config.h"
#include "nci.h"
#include "util.h"

Nci::Nci(const struct i2c_dt_spec &i2c, const struct gpio_dt_spec &irq_gpio) : i2c(i2c), irq(irq_gpio) {}
Nci::~Nci() {}

struct nci_control_msg {
    uint8_t message_type;
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

int Nci::nci_write(const uint8_t *cmd) {
    puts("juhuu 1.1.1.1");
    int ret = i2c_write_dt(&this->i2c, cmd, expected_packet_length(cmd));
    if (ret) {
        printk("i2c_write_dt: %i\n", ret);
        return ret;
    }
    puts("juhuu 1.1.1.2");
#if DEBUG_NCI_HEXDUMP
    hexdump("> ", cmd, expected_packet_length(cmd));
    puts("");
#endif
    puts("juhuu 1.1.1.3");
#if DEBUG_NCI_ANALYSIS
    nci_debug(cmd);
#endif
    puts("juhuu 1.1.1.4");
    return 0;
}

int Nci::nci_read() {
    // TODO this currently enforces read_buf to be public which is not good...
    int ret = i2c_read_dt(&this->i2c, this->read_buf, this->read_buf_len);
    if (ret) {
        printk("i2c_read_dt: %i\n", ret);
        return ret;
    }
#if DEBUG_NCI_HEXDUMP
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

    puts("juhuu 1.1.1");
    ret = this->nci_write(cmd);
    if (ret) {
        return ret;
    }

    puts("juhuu 1.1.2");
    while (gpio_pin_get_dt(&this->irq) == 0) {
        k_sleep(K_MSEC(50));
    }
    puts("juhuu 1.1.3");

    ret = nci_read();
    if (ret)
        return ret;

    puts("juhuu 1.1.4");
    return 0;
}

struct nci_control_msg Nci::nci_parse_control_msg_standalone(const uint8_t *msg_buf) {
    uint8_t payload_len = msg_buf[2];
    struct nci_control_msg msg = (struct nci_control_msg){
        .pkg_boundary_flag = (msg_buf[0] & 0x10) != 0,
        .gid = (uint8_t)(msg_buf[0] & 0x0f),
        .oid = msg_buf[1],
        .payload_len = payload_len,
        .next = nullptr,
    };
    memcpy(msg.payload, &msg_buf[3], payload_len);
    return msg;
}

struct nci_data_msg Nci::nci_parse_data_msg_standalone(const uint8_t *msg_buf) {
    uint8_t payload_len = msg_buf[2];
    struct nci_data_msg msg = (struct nci_data_msg){
        .pkg_boundary_flag = (msg_buf[0] & 0x10) != 0,
        .conn_id = (uint8_t)(msg_buf[0] & 0x0f),
        .credits = (uint8_t)(msg_buf[1] & 0x03),
        .payload_len = payload_len,
        .next = nullptr,
    };
    memcpy(msg.payload, &msg_buf[3], payload_len);
    return msg;
}

void print_rf_technology(const uint8_t technology) {
    switch (technology) {
    case NFC_RF_TECHNOLOGY_A: printk("NFC-A"); break;
    case NFC_RF_TECHNOLOGY_B: printk("NFC-B"); break;
    case NFC_RF_TECHNOLOGY_F: printk("NFC-F"); break;
    case NFC_RF_TECHNOLOGY_V: printk("NFC-V"); break;
    default: printk("[unknown RF technology 0x%02x]", technology); break;
    }
}

void print_technology_mode(const uint8_t technology_mode) {
    switch (technology_mode) {
    case NFC_A_PASSIVE_POLL_MODE: printk("NFC-A passive poll mode"); break;
    case NFC_B_PASSIVE_POLL_MODE: printk("NFC-B passive poll mode"); break;
    case NFC_F_PASSIVE_POLL_MODE: printk("NFC-F passive poll mode"); break;
    case NFC_ACTIVE_POLL_MODE: printk("NFC active poll mode"); break;
    case NFC_V_PASSIVE_POLL_MODE: printk("NFC-V passivle poll mode"); break;
    case NFC_A_PASSIVE_LISTEN_MODE: printk("NFC-A passive listen mode"); break;
    case NFC_B_PASSIVE_LISTEN_MODE: printk("NFC-B passive listen mode"); break;
    case NFC_F_PASSIVE_LISTEN_MODE: printk("NFC-F passive listen mode"); break;
    case NFC_ACTIVE_LISTEN_MODE: printk("NFC active listen mode"); break;
    default: printk("[unknown NFC technology / mode 0x%02x]", technology_mode); break;
    }
}

void print_bitrate(const uint8_t bitrate) {
    switch (bitrate) {
    case NFC_BIT_RATE_106: printk("106 kbit/s"); break;
    case NFC_BIT_RATE_212: printk("212 kbit/s"); break;
    case NFC_BIT_RATE_424: printk("424 kbit/s"); break;
    case NFC_BIT_RATE_848: printk("848 kbit/s"); break;
    case NFC_BIT_RATE_1695: printk("1695 kbit/s"); break;
    case NFC_BIT_RATE_3390: printk("3390 kbit/s"); break;
    case NFC_BIT_RATE_6780: printk("6780 kbit/s"); break;
    case NFC_BIT_RATE_26: printk("26 kbit/s"); break;
    default: printk("[unknown bitrate 0x%02x]", bitrate); break;
    }
}

void print_rf_protocol(const uint8_t protocol) {
    switch (protocol) {
    case RF_PROTO_UNDETERMINED: printk("RF_PROTO_UNDETERMINED"); break;
    case RF_PROTO_T1T_DEPRECATED: printk("T1T (deprecated)"); break;
    case RF_PROTO_T2T: printk("T2T"); break;
    case RF_PROTO_T3T: printk("T3T"); break;
    case RF_PROTO_ISO_DEP: printk("ISO-DEP"); break;
    case RF_PROTO_NFC_DEP: printk("NFC-DEP"); break;
    case RF_PROTO_T5T: printk("T5T"); break;
    case RF_PROTO_NDEF: printk("NDEF"); break;
    case RF_PROTO_WLC: printk("WLC"); break;
    default: printk("[unknown protocol 0x%02x]", protocol); break;
    }
}

void print_rf_interface(const uint8_t intf) {
    switch (intf) {
    case RF_INTF_NFCEE_DIRECT: printk("NFCEE direct"); break;
    case RF_INTF_FRAME: printk("Frame RF"); break;
    case RF_INTF_ISO_DEP: printk("ISO-DEP"); break;
    case RF_INTF_NFC_DEP: printk("NFC-DEP"); break;
    case RF_INTF_NDEF: printk("NDEF"); break;
    case RF_INTF_WLC_P_AUTONOMOUS: printk("WLC-P autonomous"); break;
    default: printk("[unknown RF interface 0x%02x]", intf); break;
    }
}

void print_status(const uint8_t status) {
    switch (status) {
    case STATUS_OK: printk("OK"); break;
    case STATUS_REJECTED: printk("REJECTED"); break;
    case STATUS_FAILED: printk("FAILED"); break;
    case STATUS_NOT_INITIALIZED: printk("NOT_INITIALIZED"); break;
    case STATUS_SYNTAX_ERROR: printk("SYNTAX_ERROR"); break;
    case STATUS_SEMANTIC_ERROR: printk("SEMANTIC_ERROR"); break;
    case STATUS_INVALID_PARAM: printk("INVALID_PARAM"); break;
    case STATUS_MESSAGE_SIZE_EXCEEDED: printk("MESSAGE_SIZE_EXCEEDED"); break;
    case STATUS_OK_1_BIT: printk("OK_1_BIT"); break;
    case STATUS_OK_2_BIT: printk("OK_2_BIT"); break;
    case STATUS_OK_3_BIT: printk("OK_3_BIT"); break;
    case STATUS_OK_4_BIT: printk("OK_4_BIT"); break;
    case STATUS_OK_5_BIT: printk("OK_5_BIT"); break;
    case STATUS_OK_6_BIT: printk("OK_6_BIT"); break;
    case STATUS_OK_7_BIT: printk("OK_7_BIT"); break;
    case STATUS_DISCOVERY_ALREADY_STARTED: printk("DISCOVERY_ALREADY_STARTED"); break;
    case STATUS_DISCOVERY_TARGET_ACTIVATION_FAILED: printk("DISCOVERY_TARGET_ACTIVATION_FAILED"); break;
    case STATUS_DISCOVERY_TEAR_DOWN: printk("DISCOVERY_TEAR_DOWN"); break;
    case STATUS_RF_FRAME_CORRUPTED: printk("RF_FRAME_CORRUPTED"); break;
    case STATUS_RF_TRANSMISSION_EXCEPTION: printk("RF_TRANSMISSION_EXCEPTION"); break;
    case STATUS_RF_PROTOCOL_EXCEPTION: printk("RF_PROTOCOL_EXCEPTION"); break;
    case STATUS_RF_TIMEOUT_EXCEPTION: printk("RF_TIMEOUT_EXCEPTION"); break;
    case STATUS_RF_UNEXPECTED_DATA: printk("RF_UNEXPECTED_DATA"); break;
    case STATUS_NFCEE_INTERFACE_ACTIVATION_FAILED: printk("NFCEE_INTERFACE_ACTIVATION_FAILED"); break;
    case STATUS_NFCEE_TRANSMISSION_ERROR: printk("NFCEE_TRANSMISSION_ERROR"); break;
    case STATUS_NFCEE_PROTOCOL_ERROR: printk("NFCEE_PROTOCOL_ERROR"); break;
    case STATUS_NFCEE_TIMEOUT_ERROR: printk("NFCEE_TIMEOUT_ERROR"); break;
    default: printk("[unknown status 0x%02x]", status); break;
    }
}

// NOTE This might become a full on handler function later.
// Theoretically it should not need it but a flag to deactivate debug messages & actual handling
// could be nice in the future.
// This should also take a representation of current context (state machine, parameters, ...)
void Nci::nci_debug(const uint8_t *msg_buf) {
    uint8_t mt = msg_buf[0] & 0xe0; // this should also maybe be abstracted.
    if (mt == MT_CMD) {
        // control message
        struct nci_control_msg msg = nci_parse_control_msg_standalone(msg_buf);
        if (msg.pkg_boundary_flag) {
            printk("[PBF] ");
        }
        switch (msg.gid) {
        case CMD_GID_CORE:
            switch (msg.oid) {
            case CORE_RESET: printk("CORE_RESET_CMD\n"); break;                             // TODO details
            case CORE_INIT: printk("CORE_INIT_CMD\n"); break;                               // TODO details
            case CORE_SET_CONFIG: printk("CORE_SET_CONFIG_CMD\n"); break;                   // TODO details
            case CORE_GET_CONFIG: printk("CORE_GET_CONFIG_CMD\n"); break;                   // TODO details
            case CORE_CONN_CREATE: printk("CORE_CONN_CREATE_CMD\n"); break;                 // TODO details
            case CORE_CONN_CLOSE: printk("CORE_CONN_CLOSE_CMD\n"); break;                   // TODO details
            case CORE_SET_POWER_SUB_STATE: printk("CORE_SET_POWER_SUB_STATE_CMD\n"); break; // TODO details
            default: printk("[WARN] Command unknown for Core OID: 0x%02x\n", msg.oid); break;
            }
            break;
        case CMD_GID_RF:
            switch (msg.oid) {
            case RF_DISCOVER_MAP: printk("RF_DISCOVER_MAP_CMD\n"); break;                         // TODO details
            case RF_SET_LISTEN_MODE_ROUTING: printk("RF_SET_LISTEN_MODE_ROUTING_CMD\n"); break;   // TODO details
            case RF_GET_LISTEN_MODE_ROUTING: printk("RF_GET_LISTEN_MODE_ROUTING_CMD\n"); break;   // TODO details
            case RF_DISCOVER: printk("RF_DISCOVER_CMD\n"); break;                                 // TODO details
            case RF_DISCOVER_SELECT: printk("RF_DISCOVER_SELECT_CMD\n"); break;                   // TODO details
            case RF_DEACTIVATE: printk("RF_DEACTIVATE_CMD\n"); break;                             // TODO details
            case RF_T3T_POLLING: printk("RF_T3T_POLLING_CMD\n"); break;                           // TODO details
            case RF_PARAMETER_UPDATE: printk("RF_PARAMETER_UPDATE_CMD\n"); break;                 // TODO details
            case RF_INTF_EXT_START: printk("RF_INTF_EXT_START_CMD\n"); break;                     // TODO details
            case RF_INTF_EXT_STOP: printk("RF_INTF_EXT_STOP_CMD\n"); break;                       // TODO details
            case RF_EXT_AGG_ABORT: printk("RF_EXT_AGG_ABORT_CMD\n"); break;                       // TODO details
            case RF_NDEF_ABORT: printk("RF_NDEF_ABORT_CMD\n"); break;                             // TODO details
            case RF_ISO_DEP_NAK_PRESENCE: printk("RF_ISO_DEP_NAK_PRESENCE_CMD\n"); break;         // TODO details
            case RF_SET_FORCED_NFCEE_ROUTING: printk("RF_SET_FORCED_NFCEE_ROUTING_CMD\n"); break; // TODO details
            case RF_REMOVAL_DETECTION: printk("RF_REMOVAL_DETECTION_CMD\n"); break;               // TODO details
            case GET_WLCP_INFO_PARAM: printk("RF_GET_WLCP_INFO_PARAM_CMD\n"); break;              // TODO details
            case WPT_START: printk("RF_WPT_START_CMD\n"); break;                                  // TODO details
            default: printk("[WARN] Command unknown for RF management OID: 0x%02x\n", msg.oid); break;
            }
            break;
        case CMD_GID_NFCEE_MGMT:
            switch (msg.oid) {
            case NFCEE_DISCOVER: printk("NFCEE_DISCOVER_CMD\n"); break;                         // TODO details
            case NFCEE_MODE_SET: printk("NFCEE_MODE_SET_CMD\n"); break;                         // TODO details
            case NFCEE_STATUS: printk("NFCEE_STATUS_CMD\n"); break;                             // TODO details
            case NFCEE_POWER_AND_LINK_CNTRL: printk("NFCEE_POWER_AND_LINK_CNTRL_CMD\n"); break; // TODO details
            default: printk("[WARN] Command unknown for NFCEE management OID: 0x%02x\n", msg.oid); break;
            }
            break;
        case CMD_GID_NFCC_MGMT: printk("No NFCC management commands known.\n"); break;
        case CMD_GID_TEST_MGMT: printk("No test management commands known.\n"); break;
        default: printk("[WARN] NCI control GID 0x%01x unknown.\n", msg_buf[0] & 0xf); break;
        }
    } else if (mt == MT_RSP) {
        struct nci_control_msg msg = nci_parse_control_msg_standalone(msg_buf);
        if (msg.pkg_boundary_flag) {
            printk("[PBF] ");
        }
        switch (msg.gid) {
        case CMD_GID_CORE:
            switch (msg.oid) {
            case CORE_RESET:
                printk("CORE_RESET_RSP\n");
                break; // TODO details (has different format than specified in practice... weird...)
            case CORE_INIT:
                printk("CORE_INIT_RSP (");
                print_status(msg.payload[0]);
                printk(")\n");
                if (msg.payload[0] != STATUS_OK)
                    break;
                printk("\tNFCC Features: ");
                hexdump((uint8_t *)&(msg.payload) + 1, 4); // TODO make human readable
                printk("\n");
                printk("\tMax Logical Connections: %i\n", msg.payload[5]);
                printk("\tMax Routing Table Size: %i\n", *((uint16_t *)(msg.payload + 6)));
                printk("\tMax Control Packet Payload Size: %i\n", msg.payload[8]);
                printk("\tMax Data Packet Payload Size: %i\n", msg.payload[9]);
                printk("\tHCI Credits: %i\n", msg.payload[10]);
                // NOTE values bigger than one octet are encoded in little endian.
                printk("\tMax NFC-V RF Frame Size: %i\n", *((uint16_t *)(msg.payload + 11)));
                printk("\tNumber of Supported RF Interfaces: %i\n", msg.payload[13]);
                printk("\tSupported RF Interfaces: ");
                hexdump((uint8_t *)&(msg.payload) + 14, msg.payload_len - 14); // TODO make human readable
                printk("\n");
                break; // TODO the data coming from this in practice doesn't seem right. But I think this code behaves
                       // as specified...
            case CORE_SET_CONFIG:
                printk("CORE_SET_CONFIG_RSP (");
                print_status(msg.payload[0]);
                printk(")\n");
                if (msg.payload[0] == STATUS_OK)
                    break;
                printk("\tInvalid parameters (%i): ", msg.payload[1]);
                hexdump((uint8_t *)&(msg.payload) + 2, msg.payload[1]); // TODO make human readable
                printk("\n");
                break;
            case CORE_GET_CONFIG: printk("CORE_GET_CONFIG_RSP\n"); break;                   // TODO details
            case CORE_CONN_CREATE: printk("CORE_CONN_CREATE_RSP\n"); break;                 // TODO details
            case CORE_CONN_CLOSE: printk("CORE_CONN_CLOSE_RSP\n"); break;                   // TODO details
            case CORE_SET_POWER_SUB_STATE: printk("CORE_SET_POWER_SUB_STATE_RSP\n"); break; // TODO details
            default: printk("[WARN] Response unknown for Core OID: 0x%02x\n", msg.oid); break;
            }
            break;
        case CMD_GID_RF:
            switch (msg.oid) {
            case RF_DISCOVER_MAP:
                printk("RF_DISCOVER_MAP_RSP (");
                print_status(msg.payload[0]);
                printk(")\n");
                break;
            case RF_SET_LISTEN_MODE_ROUTING:
                printk("RF_SET_LISTEN_MODE_ROUTING_RSP (");
                print_status(msg.payload[0]);
                printk(")\n");
                break;
            case RF_GET_LISTEN_MODE_ROUTING:
                printk("RF_GET_LISTEN_MODE_ROUTING_RSP (");
                print_status(msg.payload[0]);
                printk(")\n");
                break;
            case RF_DISCOVER:
                printk("RF_DISCOVER_RSP (");
                print_status(msg.payload[0]);
                printk(")\n");
                break;
            case RF_DISCOVER_SELECT: printk("RF_DISCOVER_SELECT_RSP\n"); break;                   // TODO details
            case RF_DEACTIVATE: printk("RF_DEACTIVATE_RSP\n"); break;                             // TODO details
            case RF_T3T_POLLING: printk("RF_T3T_POLLING_RSP\n"); break;                           // TODO details
            case RF_PARAMETER_UPDATE: printk("RF_PARAMETER_UPDATE_RSP\n"); break;                 // TODO details
            case RF_INTF_EXT_START: printk("RF_INTF_EXT_START_RSP\n"); break;                     // TODO details
            case RF_INTF_EXT_STOP: printk("RF_INTF_EXT_STOP_RSP\n"); break;                       // TODO details
            case RF_EXT_AGG_ABORT: printk("RF_EXT_AGG_ABORT_RSP\n"); break;                       // TODO details
            case RF_NDEF_ABORT: printk("RF_NDEF_ABORT_RSP\n"); break;                             // TODO details
            case RF_ISO_DEP_NAK_PRESENCE: printk("RF_ISO_DEP_NAK_PRESENCE_RSP\n"); break;         // TODO details
            case RF_SET_FORCED_NFCEE_ROUTING: printk("RF_SET_FORCED_NFCEE_ROUTING_RSP\n"); break; // TODO details
            case RF_REMOVAL_DETECTION: printk("RF_REMOVAL_DETECTION_RSP\n"); break;               // TODO details
            case GET_WLCP_INFO_PARAM: printk("RF_GET_WLCP_INFO_PARAM_RSP\n"); break;              // TODO details
            case WPT_START: printk("RF_WPT_START_RSP\n"); break;                                  // TODO details
            default: printk("[WARN] Response unknown for RF management OID: 0x%02x\n", msg.oid); break;
            }
            break;
        case CMD_GID_NFCEE_MGMT:
            switch (msg.oid) {
            case NFCEE_DISCOVER: printk("NFCEE_DISCOVER_RSP\n"); break;                         // TODO details
            case NFCEE_MODE_SET: printk("NFCEE_MODE_SET_RSP\n"); break;                         // TODO details
            case NFCEE_STATUS: printk("NFCEE_STATUS_RSP\n"); break;                             // TODO details
            case NFCEE_POWER_AND_LINK_CNTRL: printk("NFCEE_POWER_AND_LINK_CNTRL_RSP\n"); break; // TODO details
            default: printk("[WARN] Response unknown for NFCEE management OID: 0x%02x\n", msg.oid); break;
            }
            break;
        case CMD_GID_NFCC_MGMT: printk("No NFCC management responses known.\n"); break;
        case CMD_GID_TEST_MGMT: printk("No test management responses known.\n"); break;
        default: printk("[WARN] NCI control GID 0x%01x unknown.\n", msg_buf[0] & 0xf); break;
        }
    } else if (mt == MT_NTF) {
        struct nci_control_msg msg = nci_parse_control_msg_standalone(msg_buf);
        if (msg.pkg_boundary_flag) {
            printk("[PBF] ");
        }
        switch (msg.gid) {
        case CMD_GID_CORE:
            switch (msg.oid) {
            case CORE_RESET: printk("CORE_RESET_NTF\n"); break;                     // TODO details
            case CORE_CONN_CREDITS: printk("CORE_CONN_CREDITS_NTF\n"); break;       // TODO details
            case CORE_GENERIC_ERROR: printk("CORE_GENERIC_ERROR_NTF\n"); break;     // TODO details
            case CORE_INTERFACE_ERROR: printk("CORE_INTERFACE_ERROR_NTF\n"); break; // TODO details
            default: printk("[WARN] Notification unknown for Core OID: 0x%02x\n", msg.oid); break;
            }
            break;
        case CMD_GID_RF:
            switch (msg.oid) {
            case RF_GET_LISTEN_MODE_ROUTING: printk("RF_GET_LISTEN_MODE_ROUTING_NTF\n"); break; // TODO details
            case RF_DISCOVER: printk("RF_DISCOVER_NTF\n"); break;                               // TODO details
            case RF_INTF_ACTIVATED: {
                printk("RF_INTF_ACTIVATED_NTF\n");
                printk("\tRF Discovery ID: %02x\n", msg.payload[0]);
                printk("\tRF Interface: ");
                uint8_t rf_intf = msg.payload[1];
                print_rf_interface(rf_intf);
                printk("\n");
                printk("\tRF Protocol: ");
                uint8_t rf_proto = msg.payload[2];
                print_rf_protocol(rf_proto);
                printk("\n");
                printk("\tActivation RF Technology and Mode: ");
                print_technology_mode(msg.payload[3]);
                printk("\n");
                printk("\tMax Data Packet Payload Size: %i\n", msg.payload[4]);
                printk("\tInitial Number of Credits: %i\n", msg.payload[5]);
                uint8_t techno_params_len = msg.payload[6];
                if (techno_params_len > 0) {
                    printk("\tTechnology specific params (%i bytes):\n\t\t", msg.payload[5]);
                    hexdump(&msg.payload[7], techno_params_len); // TODO make human readble
                    printk("\n");
                } else {
                    printk("\tNo technology specific params.\n");
                }
                printk("\tData Exchange RF Technology and Mode: ");
                uint8_t rf_techno_mode = msg.payload[7 + techno_params_len];
                print_technology_mode(rf_techno_mode);
                printk("\n");
                printk("\tData Exchange Transmit Rate: ");
                print_bitrate(msg.payload[8 + techno_params_len]);
                printk("\n");
                printk("\tData Exchange Receive Rate: ");
                print_bitrate(msg.payload[9 + techno_params_len]);
                printk("\n");
                uint8_t activation_params_len = msg.payload[10 + techno_params_len];
                if (activation_params_len > 0) {
                    printk("\tActivation Parameters (%i bytes):\n", activation_params_len);
                    if (rf_intf == RF_INTF_ISO_DEP) {
                        if (rf_techno_mode == NFC_A_PASSIVE_POLL_MODE) {
                            printk("\t\tRATS response (%i bytes): ", msg.payload[11 + techno_params_len]);
                            hexdump(&msg.payload[12 + techno_params_len], activation_params_len - 1);
                            printk("\n");
                        } else if (rf_techno_mode == NFC_B_PASSIVE_POLL_MODE) {
                            printk("\t\tATTRIB response (%i bytes): ", msg.payload[11 + techno_params_len]);
                            hexdump(&msg.payload[12 + techno_params_len],
                                    activation_params_len - 1); // TODO make human readable
                            printk("\n");
                        } else if (rf_techno_mode == NFC_A_PASSIVE_LISTEN_MODE) {
                            printk("\t\tRATS PARAM: %02x\n",
                                   msg.payload[11 + techno_params_len]); // TODO? make human readable
                        } else if (rf_techno_mode == NFC_B_PASSIVE_LISTEN_MODE) {
                            printk("\t\tATTRIB Command (%i bytes): ", msg.payload[11 + techno_params_len]);
                            hexdump(&msg.payload[12 + techno_params_len],
                                    activation_params_len - 1); // TODO make params human readable
                            const uint8_t *params = &msg.payload[16 + techno_params_len];
                            // TODO check for matching NFCID0
                            printk("\n\t\t\tParam 1 (0x%02x): ", params[0]);
                            printk("TR0: ");
                            switch ((params[0] & 0xc0) >> 6) {
                            case 0: printk("default"); break;
                            case 1: printk("48 * 16/fc"); break;
                            case 2: printk("16 * 16/fc"); break;
                            default: printk("unknown"); break;
                            }
                            printk(", TR1: ");
                            switch ((params[0] & 0x30) >> 4) {
                            case 0: printk("default"); break;
                            case 1: printk("48 * 16/fc"); break;
                            case 2: printk("16 * 16/fc"); break;
                            default: printk("unknown"); break;
                            }
                            if (params[0] & 0x08) {
                                printk(", EoS supressed");
                            } else {
                                printk(", EoS required");
                            }
                            if (params[0] & 0x04) {
                                printk(", SoS supressed");
                            } else {
                                printk(", SoS required");
                            }
                            printk("\n\t\t\tParam 2 (0x%02x): ", params[1]);
                            printk("bit rate D_listen->poll: ");
                            switch ((params[1] & 0xc0) >> 6) {
                            case 0: printk("1"); break;
                            case 1: printk("2"); break;
                            case 2: printk("4"); break;
                            case 3: printk("8"); break;
                            }
                            printk(", bit rate D_poll->listen: ");
                            switch ((params[1] & 0x30) >> 4) {
                            case 0: printk("1"); break;
                            case 1: printk("2"); break;
                            case 2: printk("4"); break;
                            case 3: printk("8"); break;
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
                            printk(",\n\t\t\t\t maximum frame size (FSD): %i bytes", fsd);
                            printk("\n\t\t\tParam 3 (0x%02x): ", params[2]);
                            printk("Minimum TR2: ");
                            switch ((params[2] & 0x06) >> 1) {
                            case 0: printk("default"); break;
                            case 1: printk("48 * 16/fc"); break;
                            case 2: printk("16 * 16/fc"); break;
                            default: printk("unknown"); break;
                            }
                            if (params[2] & 0x01) {
                                printk(",\n\t\t\t\t Device in Listen Mode ISO/IEC 14443 compliant.");
                            } else {
                                printk(",\n\t\t\t\t Device in Listen Mode *NOT* ISO/IEC 14443 compliant.");
                            }
                            printk("\n\t\t\tParam 4 (0x%02x): DID: 0x%01x", params[3], params[3] & 0x0f);
                            printk("\n");
                        }
                    } else if (rf_intf == RF_INTF_NFC_DEP) {
                        if (rf_techno_mode == NFC_A_PASSIVE_POLL_MODE || rf_techno_mode == NFC_F_PASSIVE_POLL_MODE ||
                            rf_techno_mode == NFC_ACTIVE_POLL_MODE) {
                            uint8_t atr_res_len = msg.payload[11 + techno_params_len];
                            printk("\t\tATR_RES Response (%i bytes): ", atr_res_len);
                            hexdump(&msg.payload[12 + techno_params_len], atr_res_len);
                            printk("\n");
                            printk("\t\tData Exchange Length Reduction: %02x",
                                   msg.payload[12 + techno_params_len + atr_res_len]); // TODO make human readable
                        } else if (rf_techno_mode == NFC_A_PASSIVE_LISTEN_MODE ||
                                   rf_techno_mode == NFC_F_PASSIVE_LISTEN_MODE ||
                                   rf_techno_mode == NFC_ACTIVE_LISTEN_MODE) {
                            printk("\t\t"); // TODO implement
                        }
                    }
                } else {
                    printk("\tNo activation parameters.");
                }
            } break;
            case RF_DEACTIVATE: printk("RF_DEACTIVATE_NTF\n"); break;
            case RF_FIELD_INFO:
                printk("RF_FIELD_INFO_NTF ");
                if (msg.payload[0] & 0x01) {
                    printk("(Operating Field generated by Remote NFC Endpoint)\n");
                } else {
                    printk("(No Operating Field generated by Remote NFC Endpoint)\n");
                }
                break;
            case RF_T3T_POLLING: printk("RF_T3T_POLLING_NTF\n"); break;                   // TODO details
            case RF_NFCEE_ACTION: printk("RF_NFCEE_ACTION_NTF\n"); break;                 // TODO details
            case RF_NFCEE_DISCOVERY_REQ: printk("RF_NFCEE_DISCOVERY_REQ_NTF\n"); break;   // TODO details
            case RF_ISO_DEP_NAK_PRESENCE: printk("RF_ISO_DEP_NAK_PRESENCE_NTF\n"); break; // TODO details
            case RF_REMOVAL_DETECTION: printk("RF_REMOVAL_DETECTION_NTF\n"); break;       // TODO details
            case RF_WLC_STATUS: printk("RF_WLC_STATUS_NTF\n"); break;                     // TODO details
            case GET_WLCP_INFO_PARAM: printk("GET_WLCP_INFO_PARAM_NTF\n"); break;         // TODO details
            case WPT_START: printk("WPT_START_NTF\n"); break;                             // TODO details
            default: printk("[WARN] Notification unknown for RF management OID: 0x%02x\n", msg.oid); break;
            }
            break;
        case CMD_GID_NFCEE_MGMT:
            switch (msg.oid) {
            case NFCEE_DISCOVER: printk("NFCEE_DISCOVER_NTF\n"); break; // TODO details
            case NFCEE_MODE_SET: printk("NFCEE_MODE_SET_NTF\n"); break; // TODO details
            case NFCEE_STATUS: printk("NFCEE_STATUS_NTF\n"); break;     // TODO details
            default: printk("[WARN] Notification unknown for NFCEE management OID: 0x%02x\n", msg.oid); break;
            }
            break;
        case CMD_GID_NFCC_MGMT: printk("No NFCC management notifications known.\n"); break;
        case CMD_GID_TEST_MGMT: printk("No test management notifications known.\n"); break;
        default: printk("[WARN] NCI control GID 0x%01x unknown.\n", msg_buf[0] & 0xf); break;
        }
    } else {
        struct nci_data_msg msg = nci_parse_data_msg_standalone(msg_buf);
        if (msg.pkg_boundary_flag) {
            printk("[PBF] ");
        }
        printk("Data message (Conn ID: 0x%01x; %i credits)\n", msg.conn_id, msg.credits);
        // TODO this feature would theoretically also take an extra arg for the protocol
    }
}
