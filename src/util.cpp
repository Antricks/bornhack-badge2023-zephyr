#include <zephyr/kernel.h>

void hexdump(const uint8_t *buf, size_t len) {
    for (int i = 0; i < len; i++) {
        printf("%02x", buf[i]);
        if (i < len - 1) {
            printf(":");
        }
    }
}

void hexdump(const char *prefix, const uint8_t *buf, size_t len) {
    fputs(prefix, stdout);
    hexdump(buf, len);
}
