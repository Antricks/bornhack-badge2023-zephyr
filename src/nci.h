#pragma once
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

// [NCI Table 2] Message Types
#define MT_DATA 0 << 5
#define MT_CMD 1 << 5
#define MT_RSP 2 << 5
#define MT_NTF 3 << 5

// [NCI Table 140] Status Codes
#define STATUS_OK 0
#define STATUS_REJECTED 1
#define STATUS_FAILED 3
#define STATUS_NOT_INITIALIZED 4
#define STATUS_SYNTAX_ERROR 5
#define STATUS_SEMANTIC_ERROR 6
#define STATUS_INVALID_PARAM 9
#define STATUS_MESSAGE_SIZE_EXCEEDED 0xa
#define STATUS_OK_1_BIT 0x11
#define STATUS_OK_2_BIT 0x12
#define STATUS_OK_3_BIT 0x13
#define STATUS_OK_4_BIT 0x14
#define STATUS_OK_5_BIT 0x15
#define STATUS_OK_6_BIT 0x16
#define STATUS_OK_7_BIT 0x17

#define STATUS_DISCOVERY_ALREADY_STARTED 0xa0
#define STATUS_DISCOVERY_TARGET_ACTIVATION_FAILED 0xa1
#define STATUS_DISCOVERY_TEAR_DOWN 0xa2

#define STATUS_RF_FRAME_CORRUPTED 2
#define STATUS_RF_TRANSMISSION_EXCEPTION 0xb0
#define STATUS_RF_PROTOCOL_EXCEPTION 0xb1
#define STATUS_RF_TIMEOUT_EXCEPTION 0xb2
#define STATUS_RF_UNEXPECTED_DATA 0xb3

#define STATUS_NFCEE_INTERFACE_ACTIVATION_FAILED 0xc0
#define STATUS_NFCEE_TRANSMISSION_ERROR 0xc1
#define STATUS_NFCEE_PROTOCOL_ERROR 0xc2
#define STATUS_NFCEE_TIMEOUT_ERROR 0xc3

// [NCI Table 141] RF Technologies
#define NFC_RF_TECHNOLOGY_A 0
#define NFC_RF_TECHNOLOGY_B 1
#define NFC_RF_TECHNOLOGY_F 2
#define NFC_RF_TECHNOLOGY_V 3

// [NCI Table 142] RF Technology and Mode
#define NFC_A_PASSIVE_POLL_MODE 0
#define NFC_B_PASSIVE_POLL_MODE 1
#define NFC_F_PASSIVE_POLL_MODE 2
#define NFC_ACTIVE_POLL_MODE 3
#define NFC_V_PASSIVE_POLL_MODE 6
#define NFC_A_PASSIVE_LISTEN_MODE 0x80
#define NFC_B_PASSIVE_LISTEN_MODE 0x81
#define NFC_F_PASSIVE_LISTEN_MODE 0x82
#define NFC_ACTIVE_LISTEN_MODE 0x83

// [NCI Table 143] Bit Rates
#define NFC_BIT_RATE_106 0
#define NFC_BIT_RATE_212 1
#define NFC_BIT_RATE_424 2
#define NFC_BIT_RATE_848 3
#define NFC_BIT_RATE_1695 4
#define NFC_BIT_RATE_3390 5
#define NFC_BIT_RATE_6780 6
#define NFC_BIT_RATE_26 0x20

// [NCI Table 144] RF Protocols
#define RF_PROTO_UNDETERMINED 0
#define RF_PROTO_T1T_DEPRECATED 1
#define RF_PROTO_T2T 2
#define RF_PROTO_T3T 3
#define RF_PROTO_ISO_DEP 4
#define RF_PROTO_NFC_DEP 5
#define RF_PROTO_T5T 6
#define RF_PROTO_NDEF 7
#define RF_PROTO_WLC 8

// [NCI Table 145] RF Interfaces
#define RF_INTF_NFCEE_DIRECT 0
#define RF_INTF_FRAME 1
#define RF_INTF_ISO_DEP 2
#define RF_INTF_NFC_DEP 3
#define RF_INTF_NDEF 6
#define RF_INTF_WLC_P_AUTONOMOUS 7

// [NCI Table 149] Configuration Parameter Tags
#define CFG_TOTAL_DURATION 0
#define CFG_CON_DISCOVERY_PARAM 2
#define CFG_POWER_STATE 3

#define CFG_PA_BAIL_OUT 8
#define CFG_PA_DEVICES_LIMIT 9

#define CFG_PB_AFI 0x10
#define CFG_PB_BAIL_OUT 0x11
#define CFG_PB_ATTRIB_PARAM1 0x12
#define CFG_PB_SENSB_REQ_PARAM 0x13
#define CFG_PB_DEVICES_LIMIT 0x14

#define CFG_PF_BIT_RATE 0x18
#define CFG_PF_BAIL_OUT 0x19
#define CFG_PF_DEVICES_LIMIT 0x1a

#define CFG_PI_B_H_INFO 0x20
#define CFG_PI_BIT_RATE 0x21

#define CFG_PN_NFC_DEP_PSL 0x28
#define CFG_PN_ATR_REQ_GEN_BYTES 0x29
#define CFG_PN_ATR_REQ_CONFIG 0x2a

#define CFG_PV_DEVICES_LIMIT 0x2f

#define CFG_LA_BIT_FRAME_SDD 0x30
#define CFG_LA_PLATFORM_CONFIG 0x31
#define CFG_LA_SEL_INFO 0x32
#define CFG_LA_NFCID1 0x33

#define CFG_LB_SENSB_INFO 0x38
#define CFG_LB_NFCID0 0x39
#define CFG_LB_APPLICATION_DATA 0x3a
#define CFG_LB_SFGI 0x3b
#define CFG_LB_FWI_ADC_FO 0x3c
#define CFG_LB_BIT_RATE 0x3e

#define CFG_LF_T3T_IDENTIFIERS_1 0x40
#define CFG_LF_T3T_IDENTIFIERS_2 0x41
#define CFG_LF_T3T_IDENTIFIERS_3 0x42
#define CFG_LF_T3T_IDENTIFIERS_4 0x43
#define CFG_LF_T3T_IDENTIFIERS_5 0x44
#define CFG_LF_T3T_IDENTIFIERS_6 0x45
#define CFG_LF_T3T_IDENTIFIERS_7 0x46
#define CFG_LF_T3T_IDENTIFIERS_8 0x47
#define CFG_LF_T3T_IDENTIFIERS_9 0x48
#define CFG_LF_T3T_IDENTIFIERS_10 0x49
#define CFG_LF_T3T_IDENTIFIERS_11 0x4a
#define CFG_LF_T3T_IDENTIFIERS_12 0x4b
#define CFG_LF_T3T_IDENTIFIERS_13 0x4c
#define CFG_LF_T3T_IDENTIFIERS_14 0x4d
#define CFG_LF_T3T_IDENTIFIERS_15 0x4e
#define CFG_LF_T3T_IDENTIFIERS_16 0x4f
#define CFG_LF_T3T_MAX 0x52
#define CFG_LF_T3T_FLAGS 0x53
#define CFG_LF_T3T_RD_ALLOWED 0x55
#define CFG_LF_PROTOCOL_TYPE 0x50

#define CFG_LI_A_RATS_TBI 0x58
#define CFG_LI_A_HIST_BY 0x59
#define CFG_LI_B_H_INFO_RESP 0x5a
#define CFG_LI_A_BIT_RATE 0x5b
#define CFG_LI_A_RATS_TC1 0x5c

#define CFG_LN_WT 0x60
#define CFG_LN_ATR_RES_GEN_BYTES 0x61
#define CFG_LN_ATR_RES_CONFIG 0x62

#define CFG_PACM_BIT_RATE 0x68

#define CFG_WLC_P_CAP_POWER_CLASS 0x69 // heh
#define CFG_TOT_POWER_STEPS 0x6a
#define CFG_WLC_AUTO_CAPABILITIES 0x6b

#define CFG_RF_FIELD_INFO 0x80
#define CFG_RF_NFCEE_ACTION 0x81
#define CFG_NFCDEP_OP 0x82
#define CFG_LLCP_VERSION 0x83
#define CFG_NFCC_CONFIG_CONTROL 0x85
#define CFG_RF_WLC_STATUS_CONFIG 0x86

// [NCI Table 150] GID and OID Definitions
#define CMD_GID_CORE 0
#define CMD_GID_RF 1
#define CMD_GID_NFCEE_MGMT 2
#define CMD_GID_NFCC_MGMT 3
#define CMD_GID_TEST_MGMT 4

#define CORE_RESET 0
#define CORE_INIT 1
#define CORE_SET_CONFIG 2
#define CORE_GET_CONFIG 3
#define CORE_CONN_CREATE 4
#define CORE_CONN_CLOSE 5
#define CORE_CONN_CREDITS 6
#define CORE_GENERIC_ERROR 7
#define CORE_INTERFACE_ERROR 8
#define CORE_SET_POWER_SUB_STATE 9

#define RF_DISCOVER_MAP 0
#define RF_SET_LISTEN_MODE_ROUTING 1
#define RF_GET_LISTEN_MODE_ROUTING 2
#define RF_DISCOVER 3
#define RF_DISCOVER_SELECT 4
#define RF_INTF_ACTIVATED 5
#define RF_DEACTIVATE 6
#define RF_FIELD_INFO 7
#define RF_T3T_POLLING 8
#define RF_NFCEE_ACTION 9
#define RF_NFCEE_DISCOVERY_REQ 10
#define RF_PARAMETER_UPDATE 11
#define RF_INTF_EXT_START 12
#define RF_INTF_EXT_STOP 13
#define RF_EXT_AGG_ABORT 14
#define RF_NDEF_ABORT 15
#define RF_ISO_DEP_NAK_PRESENCE 16
#define RF_SET_FORCED_NFCEE_ROUTING 17
#define RF_REMOVAL_DETECTION 18
#define RF_WLC_STATUS 19
#define GET_WLCP_INFO_PARAM 20
#define WPT_START 21

#define NFCEE_DISCOVER 0
#define NFCEE_MODE_SET 1
#define NFCEE_STATUS 2
#define NFCEE_POWER_AND_LINK_CNTRL 3

// some shortcuts for utility
#define CORE_CMD MT_CMD | CMD_GID_CORE
#define CORE_RSP MT_RSP | CMD_GID_CORE
#define CORE_NTF MT_NTF | CMD_GID_CORE

#define RF_CMD MT_CMD | CMD_GID_RF
#define RF_RSP MT_RSP | CMD_GID_RF
#define RF_NTF MT_NTF | CMD_GID_RF

#define NFCEE_CMD MT_CMD | CMD_GID_NFCEE_MGMT
#define NFCEE_RSP MT_RSP | CMD_GID_NFCEE_MGMT
#define NFCEE_NTF MT_NTF | CMD_GID_NFCEE_MGMT

enum rf_state {
    idle,
    discovery,
    listen_active,
    listen_sleep,
    poll_removal_detection,
    poll_active,
    w4_all_discoveries,
    w4_host_select
};

class Nci {
  public:
    Nci(const struct i2c_dt_spec &i2c, const struct gpio_dt_spec &irq_gpio);
    ~Nci();

    int nci_read();
    int nci_write(const uint8_t *cmd);
    int nci_write_read(const uint8_t *cmd);
    void nci_debug(const uint8_t *msg_buf);
    uint8_t read_buf[256];

  protected:
    struct nci_control_msg nci_parse_control_msg_standalone(const uint8_t *msg_buf);
    struct nci_data_msg nci_parse_data_msg_standalone(const uint8_t *msg_buf);

    size_t read_buf_len = 255;
    const struct i2c_dt_spec &i2c;
    const struct gpio_dt_spec &irq;

    rf_state state;
    uint8_t rf_intf;
    uint8_t rf_techno_mode;
    uint8_t discovery_id;
    uint8_t max_data_payload_size;
    uint8_t credits;
};

// returns the expected overall length of an NCI packet according to packet header
size_t expected_packet_length(const uint8_t *packet);

void nci_debug(const uint8_t *msg_buf);
