#include <zephyr/kernel.h>

void hexdump(const uint8_t *buf, size_t len) {
    for (int i = 0; i < len; i++) {
        printk("%02x", buf[i]); 
        if (i < len - 1) {
            printk(":");
        }
    }
}

void hexdump(const char *prefix, const uint8_t *buf, size_t len) {
    printk("%s", prefix);
    hexdump(buf, len);
}
