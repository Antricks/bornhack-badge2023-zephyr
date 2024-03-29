#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include "config.h"
#include "example-nci.hpp"
#include "nci.hpp"
#include "util.hpp"

ExampleNci::ExampleNci(const struct i2c_dt_spec i2c, const struct gpio_dt_spec irq) : i2c(i2c), irq(irq) {}
ExampleNci::~ExampleNci() {}

int ExampleNci::transport_read() {
    return i2c_read_dt(&this->i2c, &this->read_buf[0], this->read_buf_len);
}

// TODO fragment on output limit exceeded
int ExampleNci::transport_write(const uint8_t *buf, size_t buf_len) {
    return i2c_write_dt(&this->i2c, buf, buf_len);
}

bool ExampleNci::transport_ready_to_read() {
    return gpio_pin_get_dt(&irq) != 0;
}

int ExampleNci::nfca_iso_dep_setup() {
    // RF_DISCOVER_MAP_CMD 4 bytes, 1 mapping, 0x04: PROTOCOL_ISO_DEP, 0b10: map RF interface in listen mode,
    // 0x02: ISO-DEP RF Interface
    // -- according to chapter 7 in user manual
    const uint8_t rf_discover_map_cmd[] = {RF_CMD, RF_DISCOVER_MAP, 4, 1, RF_PROTO_ISO_DEP,
                                           0x02,   RF_INTF_ISO_DEP}; // 0x02 -> listen mode -> [NCI Table 51]
    nci_write_read(rf_discover_map_cmd);
    //TODO handle response

    uint8_t nfca_core_config[]={
        CORE_CMD, CORE_SET_CONFIG, 0, // NOTE: packet length is set dynamically!
        8, //
        CFG_LA_BIT_FRAME_SDD , 0x01, 0x08, //
        CFG_LA_PLATFORM_CONFIG, 0x01, 0x03, //
        CFG_LA_SEL_INFO, 0x01, 0x60, //
        //CFG_LB_SENSB_INFO, 0x01, 0x01,
        CFG_LA_NFCID1,7,0x13,0x37,0xc3,0xac,0xc0,0xff,0xee, //
        CFG_LI_A_BIT_RATE, 0x01, 0x00, //
        CFG_LI_A_HIST_BY, 0x09, 0x4d, 0x54, 0x43, 0x4f, 0x53, 0x73, 0x02, 0x02, 0x04, // NOTE: Copied from an example card for testing purposes, optional
        CFG_RF_FIELD_INFO, 0x01, 0x01, //
        CFG_RF_NFCEE_ACTION , 0x01, 0x01, //
    };
    nfca_core_config[2] = sizeof(nfca_core_config) - 3;
    nci_write_read(nfca_core_config);
    
    uint8_t nfca_routing_table[] = {
        RF_CMD, RF_SET_LISTEN_MODE_ROUTING, 0, 0x00, // NOTE: packet length is set dynamically!
        1, //
        // 0x02,0x0a,0x00,0x3f,0x07,0xd2,0x76,0x00,0x00,0x85,0x01,0x01, // Route: DH, Power state: all on (NOTE not sure if that's sensible), AID: 0xD2760000850101 (which stands for mapping version 2.0 NDEF Tag application)
        0x01,3,0x00,0x3f, RF_PROTO_ISO_DEP, // Proto: ISO-DEP -> DH
        // 0x00,3,0x00,0x3f, NFC_RF_TECHNOLOGY_A, // Techno: NFC-A -> DH 
    };
    nfca_routing_table[2] = sizeof(nfca_routing_table) - 3;
    nci_write_read(nfca_routing_table);
    // TODO handle response

    const uint8_t nfca_rf_discover_cmd[] = {RF_CMD, RF_DISCOVER, 3, 1, NFC_A_PASSIVE_LISTEN_MODE, 0x01};
    nci_write_read(nfca_rf_discover_cmd);
    // TODO handle response
    
    return 0;
}

int ExampleNci::nfca_nfc_dep_setup() {
    const uint8_t rf_discover_map_cmd[] = {RF_CMD, RF_DISCOVER_MAP, 4, 1, RF_PROTO_NFC_DEP,
                                           0x02,   RF_INTF_NFC_DEP}; // 0x02 -> listen mode -> [NCI Table 51]
    nci_write_read(rf_discover_map_cmd);
    //TODO handle response

    uint8_t nfca_core_config[] = {
        CORE_CMD, CORE_SET_CONFIG, 0, // NOTE: packet length is set dynamically!
        9,                            // number of config entries
        CFG_LA_BIT_FRAME_SDD,1,0x00, // LA_BIT_FRAME_SDD - 4 Byte ID1, 00000b 
        CFG_LA_PLATFORM_CONFIG,1,0x0c, // LA_PLATFORM_CONFIG - RFU part set to 0, rest set to 1100b 
        // Still don't know what that does but seems to kinda work in ISO-DEP
        CFG_LA_SEL_INFO,1,0x60, // 
        CFG_LA_NFCID1,7,0x13,0x37,0xc3,0xac,0xc0,0xff,0xee, //
        CFG_LN_WT,1,10, //
        CFG_LN_ATR_RES_GEN_BYTES,0, //
        CFG_LN_ATR_RES_CONFIG,1,0x30, //
        CFG_RF_FIELD_INFO,1,0x01, //
        CFG_RF_NFCEE_ACTION,1,0x01, //
    };
    nfca_core_config[2] = sizeof(nfca_core_config) - 3;
    nci_write_read(nfca_core_config);
    // TODO handle response

    uint8_t nfca_routing_table[] = {
        RF_CMD, RF_SET_LISTEN_MODE_ROUTING, 0, 0x00, // NOTE: packet length is set dynamically!
        1,
        // 0x02,0x0a,0x00,0x3f,0x07,0xd2,0x76,0x00,0x00,0x85,0x01,0x01, // Route: DH, Power state: all on (NOTE not sure if that's sensible), AID: 0xD2760000850101 (which stands for mapping version 2.0 NDEF Tag application)
        0x01,0x03,0x00,0x3f,RF_PROTO_NFC_DEP, //
        //0x00,0x03,0x00,0x3f,NFC_RF_TECHNOLOGY_A, // Techno: NFC-A 
    };
    nfca_routing_table[2] = sizeof(nfca_routing_table) - 3;
    nci_write_read(nfca_routing_table);
    // TODO handle response

    const uint8_t nfca_rf_discover_cmd[] = {RF_CMD, RF_DISCOVER, 3, 1, NFC_A_PASSIVE_LISTEN_MODE, 0x01};
    nci_write_read(nfca_rf_discover_cmd);
    // TODO handle response
    
    return 0;
}

int ExampleNci::nfcb_iso_dep_setup() {
    // RF_DISCOVER_MAP_CMD 4 bytes, 1 mapping, 0x04: PROTOCOL_ISO_DEP, 0b10: map RF interface in listen mode,
    // 0x02: ISO-DEP RF Interface
    // -- according to chapter 7 in user manual
    const uint8_t rf_discover_map_cmd[] = {RF_CMD, RF_DISCOVER_MAP, 4, 1, RF_PROTO_ISO_DEP,
                                           0x02,   RF_INTF_ISO_DEP}; // 0x02 -> listen mode -> [NCI Table 51]
    nci_write_read(rf_discover_map_cmd);
    // TODO handle response

    uint8_t nfcb_core_config[] = {
        CORE_CMD, CORE_SET_CONFIG, 0, // NOTE: packet length is set dynamically!
        3,                            // number of config entries
        // CFG_LB_SENS_INFO, 0x01, 0x00, // LB_SENSB_INFO - no support for both
        CFG_LB_NFCID0, 4, 0x13, 0x37, 0x70, 0x07, // LB_NFCID0
        // CFG_LB_APPLICATION_DATA, 0x04, 0x00, 0x00, 0x00, 0x00, // LB_APPLICATION_DATA
        // CFG_LB_SFGI, 0x01, 0x00, // LB_SFGI - default value 0
        // CFG_LB_FWI_ADC_FO, 0x01, 0x05, // LB_FWI_ADC_FO - default value 0x05
        // CFG_LB_BIT_RATE, 1, NFC_BIT_RATE_6780,
        CFG_RF_FIELD_INFO,1,0x01, // for some reason I don't get data messages from my mobile with this enabled
        CFG_RF_NFCEE_ACTION,1,0x01,
    };
    nfcb_core_config[2] = sizeof(nfcb_core_config) - 3;
    nci_write_read(nfcb_core_config);
    // TODO handle response

    uint8_t nfcb_routing_table[] = {
        RF_CMD, RF_SET_LISTEN_MODE_ROUTING, 0, // NOTE: packet length is set dynamically!
        0x00,
        1, // number of table entries
        //0x02, 9, 0x00, 0x3f, 0xd2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01,
        // Route: DH, Power state: all on (NOTE not sure if that's sensible),
        // AID: 0xD2760000850101 (which stands for mapping version 2.0 NDEF Tag application)
        0x01, 3, 0x00, 0x3f, 0x04 // Proto: ISO-DEP
    };
    nfcb_routing_table[2] = sizeof(nfcb_routing_table) - 3;
    nci_write_read(nfcb_routing_table);
    // TODO handle response

    const uint8_t nfcb_rf_discover_cmd[] = {RF_CMD, RF_DISCOVER, 3, 1, NFC_B_PASSIVE_LISTEN_MODE, 0x01};
    nci_write_read(nfcb_rf_discover_cmd);
    // TODO handle response

    return 0;
}

int ExampleNci::nfcc_setup() {
    const uint8_t core_reset_cmd[] = {CORE_CMD, CORE_RESET, 1, 1}; // CORE_RESET_CMD(0x01), reset config
    nci_write_read(core_reset_cmd);
    // TODO handle response

    const uint8_t core_init_cmd[] = {CORE_CMD, 0x01, 0};
    nci_write_read(core_init_cmd);
    // TODO handle response

    const uint8_t prop_act_cmd[] = {MT_CMD | 0xf, 0x02, 0}; // gid 0x0f seems to be some proprietary thing I just stole from the micropython demo
    nci_write_read(prop_act_cmd);
    // TODO handle response
    return 0;
}
