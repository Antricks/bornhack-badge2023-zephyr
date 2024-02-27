#include "iso-dep.hpp"
#include "dep.hpp"

static const uint8_t NDEF_AID[] = {0xd2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01};

IsoDep::IsoDep(Nci *nci) : Dep(nci) {}
IsoDep::~IsoDep() {}

int IsoDep::handle_apdu(const uint8_t *buf, size_t buf_len) const {
    // TODO / NOTE: assuming buf_len is in a valid range right now
    // TODO / NOTE: not supporting extended length fields.
    uint8_t cla = buf[0]; // TODO there must be something to be mindful of in there
    uint8_t ins = buf[1];
    uint8_t p1 = buf[2]; // TODO extract info
    uint8_t p2 = buf[3]; // TODO extract info
    // TODO consider encoding (N_c)
    uint8_t lc = buf[4];
    // TODO consider encoding (N_e)
    uint8_t le = buf[buf_len - 1];
    printf("CLA: 0x%02x, INS: 0x%02x, P1: 0x%02x, P2: 0x%02x, \"L_c\": 0x%02x, \"L_e\": 0x%02x\n", cla, ins, p1, p2, lc, le);
    switch (ins) {
    case APDU_INS_SELECT: {
        puts("SELECT");
        const uint8_t res[2] = {0x90, 0};
        this->nci->nci_send_data_msg(res, 2);
    } break; // TODO implement as specified (:
    case APDU_INS_READ_BINARY_b0: {
        puts("READ_BINARY (mandatory offset)");
        const uint8_t res[15] = "\x00\x0a\xd1\x01\x07Umeow :3\x90";
        this->nci->nci_send_data_msg(res, 15);
    } break; // TODO implement as specified 2.0 (:
    default: puts("Unsupported instruction."); break;
    }
    return 0;
}
