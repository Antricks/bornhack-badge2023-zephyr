#pragma once
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include "util.h"

// returns the expected overall length of an NCI packet according to packet header
size_t expected_packet_length(const uint8_t *packet) {
    return 3 + packet[2];
}

int nci_write(const struct i2c_dt_spec *dev, const uint8_t *cmd) {
    int ret = i2c_write_dt(dev, cmd, expected_packet_length(cmd));
    if (ret) {
        printf("i2c_write_dt: %i\n", ret);
        return ret;
    }
    hexdump("> ", cmd, expected_packet_length(cmd));
    puts("");
    return 0;
}

int nci_read(const struct i2c_dt_spec *dev, uint8_t *resp_buf, size_t resp_read_len) {
    int ret = i2c_read_dt(dev, resp_buf, resp_read_len);
    if (ret) {
        printf("i2c_read_dt: %i\n", ret);
        return ret;
    }
    hexdump("< ", resp_buf, expected_packet_length(resp_buf));
    puts("");
    return 0;
}

int nci_write_read(const struct i2c_dt_spec *dev, const struct gpio_dt_spec* nfcc_irq, const uint8_t *cmd, uint8_t *resp_buf, size_t resp_read_len) {
    int ret = 0;

    ret = nci_write(dev, cmd);
    if (ret)
        return ret;

    // TODO parametrize nfcc_irq
    // TODO? read on interrupt instead of polling?
    while (gpio_pin_get_dt(nfcc_irq) == 0) {
        k_sleep(K_MSEC(50));
    }

    ret = nci_read(dev, resp_buf, resp_read_len);
    if (ret)
        return ret;

    return 0;
}
