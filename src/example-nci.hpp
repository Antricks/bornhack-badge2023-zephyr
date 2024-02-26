#pragma once

#include "nci.hpp"

class ExampleNci : public Nci {
  public:
    ExampleNci(const struct i2c_dt_spec i2c, const struct gpio_dt_spec irq_gpio);
    ~ExampleNci();
    int nfca_iso_dep_setup();
    int nfca_nfc_dep_setup();
    int nfcb_iso_dep_setup();
    int nfcc_setup();

  private:
    const struct i2c_dt_spec i2c;
    const struct gpio_dt_spec irq;

    int transport_read();
    int transport_write(const uint8_t *buf, size_t buf_len);
    bool transport_ready_to_read();
};
