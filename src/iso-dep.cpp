#include <string.h>
#include <zephyr/kernel.h>

#include "dep.hpp"
#include "iso-dep.hpp"
#include "util.hpp"

static const uint8_t NDEF_AID[] = {0xd2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01};
static const uint8_t CC_ID[] = {0xe1, 0x03};

IsoDep::IsoDep(Nci *nci) : Dep(nci) {}
IsoDep::~IsoDep() {}

// TODO / NOTE: Just blindly following t4top - there's probably huge potential for tag type specific abstraction
int IsoDep::handle_apdu(const uint8_t *buf, size_t buf_len) {
    // TODO / NOTE: assuming buf_len is in a valid range right now
    // TODO / NOTE: not supporting extended length fields.
    uint8_t cla = buf[0];
    uint8_t ins = buf[1];
    uint8_t p1 = buf[2];
    uint8_t p2 = buf[3];
    printf("CLA: 0x%02x, INS: 0x%02x, P1: 0x%02x, P2: 0x%02x\n", cla, ins, p1, p2);
    switch (ins) {
    case APDU_INS_SELECT: {
        uint8_t lc = buf[4];
        const uint8_t *identifier = &buf[5];
        // TODO proper selection
        if (p2 == 0x00) {
            this->selected_application_len = lc;
            memcpy((void *)&selected_application, identifier, lc);
        } else if (p2 == 0x0c) {
            this->selected_ef_len = lc;
            memcpy((void *)&selected_ef, identifier, lc);
        }
        printf("SELECT (identifier=");
        hexdump(identifier, lc);
        printf(")\n");
        const uint8_t res[2] = {0x90, 0};
        this->nci->nci_send_data_msg(res, 2);
    } break; // TODO implement as specified (:
    case APDU_INS_READ_BINARY_b0: {
        uint16_t offset = (((uint16_t)p1) << 8) + p2; // TODO ignored right now
        uint8_t le = buf[4];
        printf("READ_BINARY (offset=%i; le=%i)\n", offset, le);
        if (this->selected_ef_len == 2 && memcmp(this->selected_ef, CC_ID, 2) == 0) {
            puts("[Capability container selected]");
            uint8_t res[17] = {0x00, 0x0f, 0x20, 0x00, 0x3b, 0x00, 0x34, 0x04, 0x06,
                               0xe1, 0x04, 0x00, 0x32, 0x00, 0x00, 0x90, 0x00}; // NOTE: Example Capability Container
                                                                                // content from [T4TOP] Table 25
            this->nci->nci_send_data_msg(res, 17);
        } else {
            printf("[File selected: ");
            hexdump(this->selected_ef, this->selected_ef_len);
            puts("]");
            if (le == 2) {
                uint8_t res[4] = {0x00, 0x20, 0x90, 0x00};
                this->nci->nci_send_data_msg(res, 4);
            } else {
                uint8_t res[34] = "\xd1\x00\x1d               Hello World :3\x90";
                this->nci->nci_send_data_msg(res, 34);
            }
        }
    } break; // TODO implement as specified 2.0 (:
    case APDU_INS_UPDATE_BINARY_d6: {
        uint16_t offset = (((uint16_t)p1) << 8) + p2; // TODO ignored right now
        uint8_t lc = buf[4];
        const uint8_t *data = &buf[5];
        printf("UPDATE_BINARY (offset=%i; lc=%i; data=", offset, lc);
        hexdump(data, lc);
        puts(")");
        const uint8_t res[2] = {0x90, 0};
        this->nci->nci_send_data_msg(res, 2);
    } break; // TODO implement as specified 3.0 ((:
    default: puts("Unsupported instruction."); break;
    }
    return 0;
}
