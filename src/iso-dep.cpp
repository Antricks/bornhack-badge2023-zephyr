#include <stdint.h>

struct iso_dep_block {
    uint8_t proto_ctrl_byte;
    uint8_t did;
    uint8_t payload_len;
    uint8_t *payload;
    uint16_t crc;
    struct iso_dep_block *next;
};

// TODO handle commands specified in ISO/IEC 7816-4
