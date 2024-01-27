#pragma once
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

// returns the expected overall length of an NCI packet according to packet header
size_t expected_packet_length(const uint8_t *packet);

int nci_write(const struct i2c_dt_spec *dev, const uint8_t *cmd);
int nci_read(const struct i2c_dt_spec *dev, uint8_t *resp_buf, size_t resp_read_len);
int nci_write_read(const struct i2c_dt_spec *dev, const struct gpio_dt_spec *nfcc_irq, const uint8_t *cmd,
                   uint8_t *resp_buf, size_t resp_read_len);
