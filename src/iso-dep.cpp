#include "iso-dep.hpp"
#include "dep.hpp"

IsoDep::IsoDep(Nci *nci) : Dep(nci) {}
IsoDep::~IsoDep() {}

int IsoDep::handle_apdu(const uint8_t *buf, size_t buf_len) {
    puts("+++ handle apdu +++");

    // TODO / NOTE: assuming buf_len is in a valid range right now
    // TODO / NOTE: not supporting extended length fields.
    uint8_t cla = buf[0]; // TODO there must be something to be mindful of in there
    uint8_t ins = buf[1];
    uint8_t p1 = buf[2]; // TODO extract info
    uint8_t p2 = buf[3]; // TODO extract info
    printf("CLA: 0x%02x, INS: 0x%02x, P1: 0x%02x, P2: 0x%02x\n", cla, ins, p1, p2);
    switch (ins) {
    case APDU_INS_SELECT: {
        printf("SELECT\n");
        const uint8_t res[32] = {0};
        printf("sending response...\n");
        this->nci->nci_send_data_msg(res, 32);
    } break; // TODO
    }
    puts("juhuu");
    return 0;
}
