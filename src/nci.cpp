#include "nci.h"
#include "util.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

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

// TODO state machine and handler for incoming traffic

// returns the expected overall length of an NCI packet according to packet header
size_t expected_packet_length(const uint8_t *packet) {
    return 3 + packet[2];
}

int nci_write(const struct i2c_dt_spec &dev, const uint8_t *cmd) {
    int ret = i2c_write_dt(&dev, cmd, expected_packet_length(cmd));
    if (ret) {
        printf("i2c_write_dt: %i\n", ret);
        return ret;
    }
    hexdump("> ", cmd, expected_packet_length(cmd));
    puts("");
    nci_debug(cmd);
    return 0;
}

int nci_read(const struct i2c_dt_spec &dev, uint8_t *resp_buf, size_t resp_read_len) {
    int ret = i2c_read_dt(&dev, resp_buf, resp_read_len);
    if (ret) {
        printf("i2c_read_dt: %i\n", ret);
        return ret;
    }
    hexdump("< ", resp_buf, expected_packet_length(resp_buf));
    puts("");
    nci_debug(resp_buf);
    return 0;
}

int nci_write_read(const struct i2c_dt_spec &dev, const struct gpio_dt_spec *nfcc_irq, const uint8_t *cmd,
                   uint8_t *resp_buf, size_t resp_read_len) {
    int ret = 0;

    ret = nci_write(dev, cmd);
    if (ret)
        return ret;

    // TODO parametrize nfcc_irq
    while (gpio_pin_get_dt(nfcc_irq) == 0) {
        k_sleep(K_MSEC(50));
    }

    ret = nci_read(dev, resp_buf, resp_read_len);
    if (ret)
        return ret;

    return 0;
}

struct nci_control_msg nci_parse_control_msg_standalone(const uint8_t *msg_buf) {
    uint8_t payload_len = expected_packet_length(msg_buf); 
    struct nci_control_msg msg = (struct nci_control_msg){
        .pkg_boundary_flag = (msg_buf[0] & 0x10) != 0,
        .gid = (uint8_t)(msg_buf[0] & 0x0f),
        .oid = msg_buf[1],
        .payload_len = payload_len,
        .next = nullptr,
    };
    memcpy(&msg.payload, msg_buf+3, payload_len);
    return msg;
}

struct nci_data_msg nci_parse_data_msg_standalone(const uint8_t *msg_buf) {
    uint8_t payload_len = expected_packet_length(msg_buf); 
    struct nci_data_msg msg = (struct nci_data_msg){
        .pkg_boundary_flag = (msg_buf[0] & 0x10) != 0,
        .conn_id = (uint8_t)(msg_buf[0] & 0x0f),
        .credits = (uint8_t)(msg_buf[1] & 0x03),
        .payload_len = payload_len,
        .next = nullptr,
    };
    memcpy(&msg.payload, msg_buf+3, payload_len);
    return msg;
}

void nci_debug(const uint8_t *msg_buf) {
    uint8_t mt = msg_buf[0] & 0xe0; // this should also maybe be abstracted.
    if (mt == MT_CMD) {
        // control message
        struct nci_control_msg msg = nci_parse_control_msg_standalone(msg_buf);
        if(msg.pkg_boundary_flag) {
            printf("[PBF] ");
        }
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
            default: printf("[WARN] Command unknown for Core OID: 0x%02x", msg.oid); break;
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
            default: printf("[WARN] Command unknown for RF management OID: 0x%02x", msg.oid); break;
            }
            break;
        case CMD_GID_NFCEE_MGMT:
            switch (msg.oid) {
            case NFCEE_DISCOVER: printf("NFCEE_DISCOVER_CMD\n"); break;                         // TODO details
            case NFCEE_MODE_SET: printf("NFCEE_MODE_SET_CMD\n"); break;                         // TODO details
            case NFCEE_STATUS: printf("NFCEE_STATUS_CMD\n"); break;                             // TODO details
            case NFCEE_POWER_AND_LINK_CNTRL: printf("NFCEE_POWER_AND_LINK_CNTRL_CMD\n"); break; // TODO details
            default: printf("[WARN] Command unknown for NFCEE management OID: 0x%02x", msg.oid); break;
            }
            break;
        case CMD_GID_NFCC_MGMT: printf("No NFCC management commands known.\n"); break;
        case CMD_GID_TEST_MGMT: printf("No test management commands known.\n"); break;
        default: printf("[WARN] NCI control GID 0x%01x unknown.\n", msg_buf[0] & 0xf); break;
        }
    } else if (mt == MT_RSP) {
        struct nci_control_msg msg = nci_parse_control_msg_standalone(msg_buf);
        if(msg.pkg_boundary_flag) {
            printf("[PBF] ");
        }
        switch (msg.gid) {
        case CMD_GID_CORE:
            switch (msg.oid) {
            case CORE_RESET: printf("CORE_RESET_RSP\n"); break;                             // TODO details
            case CORE_INIT: printf("CORE_INIT_RSP\n"); break;                               // TODO details
            case CORE_SET_CONFIG: printf("CORE_SET_CONFIG_RSP\n"); break;                   // TODO details
            case CORE_GET_CONFIG: printf("CORE_GET_CONFIG_RSP\n"); break;                   // TODO details
            case CORE_CONN_CREATE: printf("CORE_CONN_CREATE_RSP\n"); break;                 // TODO details
            case CORE_CONN_CLOSE: printf("CORE_CONN_CLOSE_RSP\n"); break;                   // TODO details
            case CORE_SET_POWER_SUB_STATE: printf("CORE_SET_POWER_SUB_STATE_RSP\n"); break; // TODO details
            default: printf("[WARN] Response unknown for Core OID: 0x%02x", msg.oid); break;
            }
            break;
        case CMD_GID_RF:
            switch (msg.oid) {
            case RF_DISCOVER_MAP: printf("RF_DISCOVER_MAP_RSP\n"); break;                         // TODO details
            case RF_SET_LISTEN_MODE_ROUTING: printf("RF_SET_LISTEN_MODE_ROUTING_RSP\n"); break;   // TODO details
            case RF_GET_LISTEN_MODE_ROUTING: printf("RF_GET_LISTEN_MODE_ROUTING_RSP\n"); break;   // TODO details
            case RF_DISCOVER: printf("RF_DISCOVER_RSP\n"); break;                                 // TODO details
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
            default: printf("[WARN] Response unknown for RF management OID: 0x%02x", msg.oid); break;
            }
            break;
        case CMD_GID_NFCEE_MGMT:
            switch (msg.oid) {
            case NFCEE_DISCOVER: printf("NFCEE_DISCOVER_RSP\n"); break;                         // TODO details
            case NFCEE_MODE_SET: printf("NFCEE_MODE_SET_RSP\n"); break;                         // TODO details
            case NFCEE_STATUS: printf("NFCEE_STATUS_RSP\n"); break;                             // TODO details
            case NFCEE_POWER_AND_LINK_CNTRL: printf("NFCEE_POWER_AND_LINK_CNTRL_RSP\n"); break; // TODO details
            default: printf("[WARN] Response unknown for NFCEE management OID: 0x%02x", msg.oid); break;
            }
            break;
        case CMD_GID_NFCC_MGMT: printf("No NFCC management responses known.\n"); break;
        case CMD_GID_TEST_MGMT: printf("No test management responses known.\n"); break;
        default: printf("[WARN] NCI control GID 0x%01x unknown.\n", msg_buf[0] & 0xf); break;
        }
    } else if (mt == MT_NTF) {
        struct nci_control_msg msg = nci_parse_control_msg_standalone(msg_buf);
        if(msg.pkg_boundary_flag) {
            printf("[PBF] ");
        }
        switch (msg.gid) {
        case CMD_GID_CORE:
            switch (msg.oid) {
            case CORE_RESET: printf("CORE_RESET_NTF\n"); break; // TODO details
            case CORE_CONN_CREDITS: printf("CORE_CONN_CREDITS_NTF\n"); break; // TODO details
            case CORE_GENERIC_ERROR: printf("CORE_GENERIC_ERROR_NTF\n"); break; // TODO details
            case CORE_INTERFACE_ERROR: printf("CORE_INTERFACE_ERROR_NTF\n"); break; // TODO details
            default: printf("[WARN] Notification unknown for Core OID: 0x%02x", msg.oid); break;
            }
            break;
        case CMD_GID_RF:
            switch (msg.oid) {
            case RF_GET_LISTEN_MODE_ROUTING: printf("RF_GET_LISTEN_MODE_ROUTING_NTF\n"); break; // TODO details
            case RF_DISCOVER: printf("RF_DISCOVER_NTF\n"); break; // TODO details
            case RF_INTF_ACTIVATED: printf("RF_INTF_ACTIVATED_NTF\n"); break; // TODO details
            case RF_DEACTIVATE: printf("RF_DEACTIVATE_NTF\n"); break; // TODO details
            case RF_T3T_POLLING: printf("RF_T3T_POLLING_NTF\n"); break; // TODO details
            case RF_NFCEE_ACTION: printf("RF_NFCEE_ACTION_NTF\n"); break; // TODO details
            case RF_NFCEE_DISCOVERY_REQ: printf("RF_NFCEE_DISCOVERY_REQ_NTF\n"); break; // TODO details
            case RF_ISO_DEP_NAK_PRESENCE: printf("RF_ISO_DEP_NAK_PRESENCE_NTF\n"); break; // TODO details
            case RF_REMOVAL_DETECTION: printf("RF_REMOVAL_DETECTION_NTF\n"); break; // TODO details
            case RF_WLC_STATUS: printf("RF_WLC_STATUS_NTF\n"); break; // TODO details
            case GET_WLCP_INFO_PARAM: printf("GET_WLCP_INFO_PARAM_NTF\n"); break; // TODO details
            case WPT_START: printf("WPT_START_NTF\n"); break; // TODO details
            default: printf("[WARN] Notification unknown for RF management OID: 0x%02x", msg.oid); break;
            }
            break;
        case CMD_GID_NFCEE_MGMT:
            switch (msg.oid) {
            case NFCEE_DISCOVER: printf("NFCEE_DISCOVER_NTF\n"); break; // TODO details
            case NFCEE_MODE_SET: printf("NFCEE_MODE_SET_NTF\n"); break; // TODO details
            case NFCEE_STATUS: printf("NFCEE_STATUS_NTF\n"); break; // TODO details
            default: printf("[WARN] Notification unknown for NFCEE management OID: 0x%02x", msg.oid); break;
            }
            break;
        case CMD_GID_NFCC_MGMT: printf("No NFCC management notifications known.\n"); break;
        case CMD_GID_TEST_MGMT: printf("No test management notifications known.\n"); break;
        default: printf("[WARN] NCI control GID 0x%01x unknown.\n", msg_buf[0] & 0xf); break;
        }
    } else {
        struct nci_data_msg msg = nci_parse_data_msg_standalone(msg_buf);
        if(msg.pkg_boundary_flag) {
            printf("[PBF] ");
        }
        printf("Data message (Conn ID: %02x; %i credits)\n",msg.conn_id, msg.credits);
        // TODO this feature would theoretically also take an extra arg for the protocol
    }
}
