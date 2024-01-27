#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>

#include "nci.h"
#include "util.h"

static const struct gpio_dt_spec btn_user1 = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_user1), gpios);
static const struct gpio_dt_spec btn_user2 = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_user2), gpios);

static const struct gpio_dt_spec led_user0 = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);
static const struct gpio_dt_spec led_user1 = GPIO_DT_SPEC_GET(DT_NODELABEL(led_user1), gpios);
static const struct gpio_dt_spec led_user2 = GPIO_DT_SPEC_GET(DT_NODELABEL(led_user2), gpios);

static const struct gpio_dt_spec nfcc_ven = GPIO_DT_SPEC_GET(DT_NODELABEL(nfcc_ven), gpios);
static const struct gpio_dt_spec nfcc_irq = GPIO_DT_SPEC_GET(DT_NODELABEL(nfcc_irq), gpios);

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static const struct i2c_dt_spec pn7150 = I2C_DT_SPEC_GET(DT_NODELABEL(pn7150));

static uint8_t read_buf[256] = {0}; // read buffer for i2c
static bool read_nci_on_irq = false;

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart), "Device is not ACM CDC UART device");

int serial_init() {
    /* Configure to set Console output to USB Serial */
    const struct device *usb_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint32_t dtr = 0;

    /* Check if USB can be initialised, bails out if fail is returned */
    if (usb_enable(NULL) != 0) {
        return 1;
    }

    /* Wait for a console connection, if the DTR flag was set to activate USB.
     * If you wish to start generating serial data immediately, you can simply
     * remove the while loop, to not wait until the control line is set.
     */
    while (!dtr) {
        uart_line_ctrl_get(usb_device, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }

    return 0;
}

void btn1_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    puts("User Button 1");
}

void btn2_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    puts("User Button 2");
}

void nfcc_irq_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    fputs("+++ IRQ", stdout);
    int irq_val = gpio_pin_get_dt(&nfcc_irq);
    printf("=%i +++\n", irq_val);
    gpio_pin_set_dt(&led_user2, irq_val);

    // TODO this should probably be a while loop
    if (read_nci_on_irq && irq_val) {
        puts("reading from interrupt...");
        // TODO this seems to fail (does not terminate)
        nci_read(&pn7150, read_buf, 255);
    }
}

int main(void) {
    int ret = 0; // buffer for function return values

    ret = serial_init();
    if (ret != 0) {
        return ret;
    }

    // BEGIN Buttons

    ret = (!device_is_ready(btn_user1.port));
    if (ret) {
        puts("Could not get User Button 1.");
        return ret;
    } else {
        puts("Got User Button 1.");
    }

    ret = (!device_is_ready(btn_user2.port));
    if (ret) {
        puts("Could not get User Button 2.");
        return ret;
    } else {
        puts("Got User Button 2.");
    }

    ret = gpio_pin_configure_dt(&btn_user1, GPIO_INPUT | GPIO_PULL_UP);
    ret = gpio_pin_configure_dt(&btn_user2, GPIO_INPUT | GPIO_PULL_UP);

    // END Buttons
    // BEGIN LEDs

    ret = (!device_is_ready(led_user0.port));
    if (ret) {
        puts("Could not get User LED 0.");
        return ret;
    } else {
        puts("Got User LED 0.");
    }

    ret = (!device_is_ready(led_user1.port));
    if (ret) {
        puts("Could not get User LED 1.");
        return ret;
    } else {
        puts("Got User LED 1.");
    }

    ret = (!device_is_ready(led_user2.port));
    if (ret) {
        puts("Could not get User LED 2.");
        return ret;
    } else {
        puts("Got User LED 2.");
    }

    ret = gpio_pin_configure(led_user0.port, led_user0.pin, GPIO_OUTPUT);
    ret = gpio_pin_configure(led_user1.port, led_user1.pin, GPIO_OUTPUT);
    ret = gpio_pin_configure(led_user2.port, led_user2.pin, GPIO_OUTPUT);

    // END LEDs
    // BEGIN NFCC flags

    ret = (!device_is_ready(nfcc_ven.port));
    if (ret) {
        puts("Could not get NFCC's VEN.");
        return ret;
    } else {
        puts("Got NFCC's VEN.");
    }

    ret = (!device_is_ready(nfcc_irq.port));
    if (ret) {
        puts("Could not get NFCC's IRQ.");
        return ret;
    } else {
        puts("Got NFCC's IRQ.");
    }

    ret = gpio_pin_configure(nfcc_ven.port, nfcc_ven.pin, GPIO_OUTPUT);
    if (ret) {
        printf("Could not configure NFCC's VEN (%i).\n", ret);
        return ret;
    } else {
        puts("Set NFCC's VEN config.");
    }

    ret = gpio_pin_configure(nfcc_irq.port, nfcc_irq.pin, GPIO_INPUT | GPIO_PULL_DOWN);
    if (ret) {
        printf("Could not configure NFCC's IRQ (%i).\n", ret);
        return ret;
    } else {
        puts("Set NFCC's IRQ config.");
    }

    ret = gpio_pin_set_dt(&nfcc_ven, 1);
    if (ret) {
        printf("Could not set NFCC's VEN (%i).\n", ret);
        return ret;
    } else {
        puts("Set NFCC's VEN to ACTIVE.");
    }
    k_sleep(K_MSEC(200));

    // END NFCC flags
    // BEGIN GPIO Interrupts

    struct gpio_callback irq_cb;
    gpio_pin_interrupt_configure_dt(&nfcc_irq, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&irq_cb, nfcc_irq_handler, BIT(nfcc_irq.pin));
    gpio_add_callback(nfcc_irq.port, &irq_cb);

    struct gpio_callback btn1_cb;
    gpio_pin_interrupt_configure_dt(&btn_user1, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&btn1_cb, btn1_handler, BIT(btn_user1.pin));
    gpio_add_callback(btn_user1.port, &btn1_cb);

    struct gpio_callback btn2_cb;
    gpio_pin_interrupt_configure_dt(&btn_user2, GPIO_INT_EDGE_BOTH);
    gpio_init_callback(&btn2_cb, btn2_handler, BIT(btn_user2.pin));
    gpio_add_callback(btn_user2.port, &btn2_cb);

    // END GPIO Interrupts
    // BEGIN I2C setup

    ret = (i2c_dev == NULL || !device_is_ready(i2c_dev));
    if (ret) {
        puts("Could not get I2C controller device.");
        return ret;
    } else {
        puts("Got I2C controller device.");
    }

    puts("Got PN7150 dt spec.");

    ret = i2c_is_ready_dt(&pn7150);
    if (ret) {
        puts("i2c_is_ready: true");
    } else {
        puts("i2c_is_ready: false");
        return 1;
    }

    uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_FAST) | I2C_MODE_CONTROLLER;
    uint32_t i2c_cfg_tmp = 0;

    ret = i2c_configure(i2c_dev, i2c_cfg);
    printf("i2c_configure: %i\n", ret);
    if (ret) {
        printf("i2c_configure failed: %i\n", ret);
    } else {
        puts("I2C config set successfully.");
    }

    ret = i2c_get_config(i2c_dev, &i2c_cfg_tmp);
    printf("i2c_get_config: %i\n", ret);
    if (ret == -88) {
        puts("Config get not supported. (ENOSYS returned)");
    } else if (ret) {
        printf("Could not get controller config. (%i)", ret);
        return ret;
    } else {
        if (i2c_cfg_tmp != i2c_cfg) {
            printf("Config was not set correctly.");
            hexdump("\nretreived: ", (uint8_t *)&i2c_cfg_tmp, 4);
            hexdump("\nintended: ", (uint8_t *)&i2c_cfg, 4);
            puts("");
            return 1;
        }
    }

    // END I2C setup

    // yippie, working i2c!

    const uint8_t CORE_RESET_CMD[] = {0x20, 0x00, 1, 1}; // CORE_RESET_CMD(0x01), reset config
    nci_write_read(&pn7150, &nfcc_irq, CORE_RESET_CMD, read_buf, 255);

    const uint8_t CORE_INIT_CMD[] = {0x20, 0x01, 0};
    nci_write_read(&pn7150, &nfcc_irq, CORE_INIT_CMD, read_buf, 255);

    const uint8_t PROP_ACT_CMD[] = {0x2f, 0x02, 0};
    nci_write_read(&pn7150, &nfcc_irq, PROP_ACT_CMD, read_buf, 255);

    // RF_DISCOVER_MAP_CMD 4 bytes, 1 mapping, 0x04: PROTOCOL_ISO_DEP, 0b10: map RF interface in listen mode,
    // 0x02: ISO-DEP RF Interface
    // -- according to chapter 7 in user manual
    const uint8_t RF_DISCOVER_MAP_CMD[] = "\x21\x00\x04\x01\x04\x02\x02";
    nci_write_read(&pn7150, &nfcc_irq, RF_DISCOVER_MAP_CMD, read_buf, 255);

    uint8_t NFCB_CORE_CONFIG[] = {
        0x20, 0x02, 0x1c,                   // CORE_SET_CONFIG_CMD NOTICE: packet length is set dynamically!
        7,                                  // number of config entries
        0x38, 0x01, 0x00,                   // LB_SENSB_INFO - no support for both
        0x39, 0x04, 0x13, 0x37, 0x70, 0x07, // LB_NFCID0
        0x3a, 0x04, 0x00, 0x00, 0x00, 0x00, // LB_APPLICATION_DATA
        0x3b, 0x01, 0x00,                   // LB_SFGI - default value 0
        0x3c, 0x01, 0x05,                   // LB_FWI_ADC_FO - default value 0x05
        0x3e, 0x01, 0x06,                   // LB_BIT_RATE
        0x5a, 0x00                          // LI_B_H_INFO_RESP
    };
    NFCB_CORE_CONFIG[2] = sizeof(NFCB_CORE_CONFIG) - 3;
    nci_write_read(&pn7150, &nfcc_irq, NFCB_CORE_CONFIG, read_buf, 255);

    uint8_t NFCB_ROUTING_TABLE[] = {
        0x21, 0x01, 0x00, 0x00, // TODO Command annotation NOTICE: packet length calculated dynamically
        2,                      // number of table entries
        0x02, 0x09, 0x00, 0x3f, 0xd2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01,
        // Route: DH, Power state: all on (NOTICE not sure if that's sensible),
        // AID: 0xD2760000850101 (which stands for mapping version 2.0 NDEF Tag application)
        0x01, 0x03, 0x00, 0x3f, 0x04 // Proto: ISO-DEP
    };
    NFCB_ROUTING_TABLE[2] = sizeof(NFCB_ROUTING_TABLE) - 3;
    nci_write_read(&pn7150, &nfcc_irq, NFCB_ROUTING_TABLE, read_buf, 255);

    const uint8_t RF_DISCOVER_CMD_NFCB[] = {0x21,0x03,0x03,0x01,0x81,0x01}; // # RF_DISCOVER_CMD in B mode
    nci_write_read(&pn7150, &nfcc_irq, RF_DISCOVER_CMD_NFCB, read_buf, 255);

    puts("Reached end.");
    while (true) {
        gpio_pin_toggle_dt(&led_user1);
        k_sleep(K_MSEC(1000));

        if (gpio_pin_get_dt(&nfcc_irq)) {
            nci_read(&pn7150, read_buf, 255);
        }
    }

    return 0;
}
