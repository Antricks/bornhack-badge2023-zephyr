#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>

#include "nci.h"
#include "util.h"

#define WAIT_FOR_SERIAL 1

static const struct gpio_dt_spec btn_user1 = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_user1), gpios);
static const struct gpio_dt_spec btn_user2 = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_user2), gpios);

static const struct pwm_dt_spec pwm_led_user0 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_led0));
static const struct pwm_dt_spec pwm_led_user1 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_led1));
static const struct pwm_dt_spec pwm_led_user2 = PWM_DT_SPEC_GET(DT_NODELABEL(pwm_led2));

static const struct gpio_dt_spec led_user0 = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);
static const struct gpio_dt_spec led_user1 = GPIO_DT_SPEC_GET(DT_NODELABEL(led_user1), gpios);
static const struct gpio_dt_spec led_user2 = GPIO_DT_SPEC_GET(DT_NODELABEL(led_user2), gpios);

static const struct gpio_dt_spec nfcc_ven = GPIO_DT_SPEC_GET(DT_NODELABEL(nfcc_ven), gpios);
static const struct gpio_dt_spec nfcc_irq = GPIO_DT_SPEC_GET(DT_NODELABEL(nfcc_irq), gpios);

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static const struct i2c_dt_spec pn7150 = I2C_DT_SPEC_GET(DT_NODELABEL(pn7150));

static uint8_t read_buf[256] = {0}; // read buffer for i2c

BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart), "Device is not ACM CDC UART device");

int serial_init() {
    /* Configure to set Console output to USB Serial */
    const struct device *usb_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint32_t dtr = 0;

    /* Check if USB can be initialised, bails out if fail is returned */
    if (usb_enable(NULL) != 0) {
        return 1;
    }

    uart_line_ctrl_get(usb_device, UART_LINE_CTRL_DTR, &dtr);
#if WAIT_FOR_SERIAL
    // TODO for some reason main does not print to stdout after some time if I don't do this. No clue why... ._.
    while (!dtr) {
        uart_line_ctrl_get(usb_device, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }
#endif

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
}

int nfca_iso_dep_setup() {
    // RF_DISCOVER_MAP_CMD 4 bytes, 1 mapping, 0x04: PROTOCOL_ISO_DEP, 0b10: map RF interface in listen mode,
    // 0x02: ISO-DEP RF Interface
    // -- according to chapter 7 in user manual
    const uint8_t rf_discover_map_cmd[] = {RF_CMD, RF_DISCOVER_MAP, 4, 1, RF_PROTO_ISO_DEP,
                                           0x02,   RF_INTF_ISO_DEP}; // 0x02 -> listen mode -> [NCI Table 51]
    nci_write_read(pn7150, &nfcc_irq, rf_discover_map_cmd, read_buf, 255);
    //TODO handle response

    uint8_t nfca_core_config[] = {
        CORE_CMD, CORE_SET_CONFIG, 0, // NOTICE: packet length is set dynamically!
        4,                            // number of config entries
        CFG_LA_BIT_FRAME_SDD,1,0x00, // LA_BIT_FRAME_SDD - 4 Byte ID1, 00000b 
        CFG_LA_PLATFORM_CONFIG,1,0x0c, // LA_PLATFORM_CONFIG - RFU part set to 0, rest set to 1100b 
        CFG_LA_SEL_INFO,1,0x60, // LA_SEL_INFO = ??
        CFG_LA_NFCID1,7,0xcc,0xca,0xc,0x13,0x37,0x37,0xc3, // LA_NFCID1 = 0x37c31337
        //0x59,0x01,0x00, //# LI_A_HIST_BY = 0
        //0x5b,0x01,0x01, //# LI_A_BIT_RATE = maximum available bitrate 
    };
    nfca_core_config[2] = sizeof(nfca_core_config) - 3;
    nci_write_read(pn7150, &nfcc_irq, nfca_core_config, read_buf, 255);
    // TODO handle response

    uint8_t nfca_routing_table[] = {
        RF_CMD, RF_SET_LISTEN_MODE_ROUTING, 0, 0x00, // NOTICE: packet length is set dynamically!
        3,
        //0x02,0x07,0x00,0x3f,0x04,0x37,0xc3,0x13,0x37, // Application ID: 0x37c31337
        0x02,0x0a,0x00,0x3f,0x07,0xd2,0x76,0x00,0x00,0x85,0x01,0x01, // Route: DH, Power state: all on (NOTICE not sure if that's sensible), AID: 0xD2760000850101 (which stands for mapping version 2.0 NDEF Tag application)
        0x01,0x03,0x00,0x3f,0x04, // Proto: ISO-DEP
        0x00,0x03,0x00,0x3f,0x00, // Techno: NFC-A 
    };
    nfca_routing_table[2] = sizeof(nfca_routing_table) - 3;
    nci_write_read(pn7150, &nfcc_irq, nfca_routing_table, read_buf, 255);
    // TODO handle response

    const uint8_t nfca_rf_discover_cmd[] = {RF_CMD, RF_DISCOVER, 3, 1, NFC_A_PASSIVE_LISTEN_MODE, 0x01};
    nci_write_read(pn7150, &nfcc_irq, nfca_rf_discover_cmd, read_buf, 255);
    // TODO handle response
    
    return 0;
}

int nfca_nfc_dep_setup() {
    const uint8_t rf_discover_map_cmd[] = {RF_CMD, RF_DISCOVER_MAP, 4, 1, RF_PROTO_NFC_DEP,
                                           0x02,   RF_INTF_NFC_DEP}; // 0x02 -> listen mode -> [NCI Table 51]
    nci_write_read(pn7150, &nfcc_irq, rf_discover_map_cmd, read_buf, 255);
    //TODO handle response

    uint8_t nfca_core_config[] = {
        CORE_CMD, CORE_SET_CONFIG, 0, // NOTICE: packet length is set dynamically!
        1,                            // number of config entries
        //CFG_LA_BIT_FRAME_SDD,1,0x00, // LA_BIT_FRAME_SDD - 4 Byte ID1, 00000b 
        //CFG_LA_PLATFORM_CONFIG,1,0x0c, // LA_PLATFORM_CONFIG - RFU part set to 0, rest set to 1100b 
        //CFG_LA_SEL_INFO,1,0x60, // Still don't know what that does but seems to kinda work in ISO-DEP
        CFG_LA_NFCID1,7,0xcc,0xca,0xcc,0x13,0x37,0x37,0xc3,
        //CFG_LN_WT,1,10,
        //CFG_LN_ATR_RES_GEN_BYTES,0,
        //CFG_LN_ATR_RES_CONFIG,1,0x30
    };
    nfca_core_config[2] = sizeof(nfca_core_config) - 3;
    nci_write_read(pn7150, &nfcc_irq, nfca_core_config, read_buf, 255);
    // TODO handle response

    uint8_t nfca_routing_table[] = {
        RF_CMD, RF_SET_LISTEN_MODE_ROUTING, 0, 0x00, // NOTICE: packet length is set dynamically!
        3,
        //0x02,0x07,0x00,0x3f,0x04,0x37,0xc3,0x13,0x37, // Application ID: 0x37c31337
        0x02,0x0a,0x00,0x3f,0x07,0xd2,0x76,0x00,0x00,0x85,0x01,0x01, // Route: DH, Power state: all on (NOTICE not sure if that's sensible), AID: 0xD2760000850101 (which stands for mapping version 2.0 NDEF Tag application)
        0x01,0x03,0x00,0x3f,RF_PROTO_NFC_DEP,
        0x00,0x03,0x00,0x3f,0x00, // Techno: NFC-A 
    };
    nfca_routing_table[2] = sizeof(nfca_routing_table) - 3;
    nci_write_read(pn7150, &nfcc_irq, nfca_routing_table, read_buf, 255);
    // TODO handle response

    const uint8_t nfca_rf_discover_cmd[] = {RF_CMD, RF_DISCOVER, 3, 1, NFC_A_PASSIVE_LISTEN_MODE, 0x01};
    nci_write_read(pn7150, &nfcc_irq, nfca_rf_discover_cmd, read_buf, 255);
    // TODO handle response
    
    return 0;
}

int nfcb_iso_dep_setup() {
    // RF_DISCOVER_MAP_CMD 4 bytes, 1 mapping, 0x04: PROTOCOL_ISO_DEP, 0b10: map RF interface in listen mode,
    // 0x02: ISO-DEP RF Interface
    // -- according to chapter 7 in user manual
    const uint8_t rf_discover_map_cmd[] = {RF_CMD, RF_DISCOVER_MAP, 4, 1, RF_PROTO_ISO_DEP,
                                           0x02,   RF_INTF_ISO_DEP}; // 0x02 -> listen mode -> [NCI Table 51]
    nci_write_read(pn7150, &nfcc_irq, rf_discover_map_cmd, read_buf, 255);
    // TODO handle response

    uint8_t nfcb_core_config[] = {
        CORE_CMD, CORE_SET_CONFIG, 0, // NOTICE: packet length is set dynamically!
        2,                            // number of config entries
        // CFG_LB_SENS_INFO, 0x01, 0x00,                   // LB_SENSB_INFO - no support for both
        CFG_LB_NFCID0, 4, 0x13, 0x37, 0x70, 0x07, // LB_NFCID0
        // CFG_LB_APPLICATION_DATA, 0x04, 0x00, 0x00, 0x00, 0x00, // LB_APPLICATION_DATA
        // CFG_LB_SFGI, 0x01, 0x00,                   // LB_SFGI - default value 0
        // CFG_LB_FWI_ADC_FO, 0x01, 0x05,                   // LB_FWI_ADC_FO - default value 0x05
        CFG_LB_BIT_RATE, 1, NFC_BIT_RATE_6780, CFG_LI_B_H_INFO_RESP, 0 // LI_B_H_INFO_RESP
    };
    nfcb_core_config[2] = sizeof(nfcb_core_config) - 3;
    nci_write_read(pn7150, &nfcc_irq, nfcb_core_config, read_buf, 255);
    // TODO handle response

    uint8_t nfcb_routing_table[] = {
        RF_CMD, RF_SET_LISTEN_MODE_ROUTING, 0, // NOTICE: packet length is set dynamically!
        0x00,
        2, // number of table entries
        0x02, 9, 0x00, 0x3f, 0xd2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01,
        // Route: DH, Power state: all on (NOTICE not sure if that's sensible),
        // AID: 0xD2760000850101 (which stands for mapping version 2.0 NDEF Tag application)
        0x01, 3, 0x00, 0x3f, 0x04 // Proto: ISO-DEP
    };
    nfcb_routing_table[2] = sizeof(nfcb_routing_table) - 3;
    nci_write_read(pn7150, &nfcc_irq, nfcb_routing_table, read_buf, 255);
    // TODO handle response

    const uint8_t nfcb_rf_discover_cmd[] = {RF_CMD, RF_DISCOVER, 3, 1, NFC_B_PASSIVE_LISTEN_MODE, 0x01};
    nci_write_read(pn7150, &nfcc_irq, nfcb_rf_discover_cmd, read_buf, 255);
    // TODO handle response

    return 0;
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
    // TODO handle errors here

    // END Buttons
    // BEGIN LEDs

    // ret = gpio_pin_configure(led_user0.port, led_user0.pin, GPIO_OUTPUT);
    ret = gpio_pin_configure(led_user1.port, led_user1.pin, GPIO_OUTPUT);
    ret = gpio_pin_configure(led_user2.port, led_user2.pin, GPIO_OUTPUT);
    // TODO handle errors here

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

    if (!pwm_is_ready_dt(&pwm_led_user0)) {
        printk("Error: PWM device %s is not ready\n", pwm_led_user0.dev->name);
        return 0;
    } else {
        puts("Got User LED 0 PWM.");
    }
    if (!pwm_is_ready_dt(&pwm_led_user1)) {
        printk("Error: PWM device %s is not ready\n", pwm_led_user1.dev->name);
        return 0;
    } else {
        puts("Got User LED 1 PWM.");
    }

    if (!pwm_is_ready_dt(&pwm_led_user2)) {
        printk("Error: PWM device %s is not ready\n", pwm_led_user2.dev->name);
        return 0;
    } else {
        puts("Got User LED 2 PWM.");
    }

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

    const uint8_t core_reset_cmd[] = {CORE_CMD, CORE_RESET, 1, 1}; // CORE_RESET_CMD(0x01), reset config
    nci_write_read(pn7150, &nfcc_irq, core_reset_cmd, read_buf, 255);
    // TODO handle response

    const uint8_t core_init_cmd[] = {CORE_CMD, 0x01, 0};
    nci_write_read(pn7150, &nfcc_irq, core_init_cmd, read_buf, 255);
    // TODO handle response

    const uint8_t prop_act_cmd[] = {
        MT_CMD | 0xf, 0x02, 0}; // gid 0x0f seems to be some proprietary thing I just stole from the micropython demo
    nci_write_read(pn7150, &nfcc_irq, prop_act_cmd, read_buf, 255);
    // TODO handle response

    nfca_nfc_dep_setup();

    printf("Reached end.");

    // TODO adjust dynamically based on period from dt
    uint32_t pwm_min_pulse = PWM_NSEC(5000);
    uint32_t pwm_max_pulse = PWM_MSEC(1);
    bool dir = true;
    uint32_t pulse = pwm_min_pulse;

    while (true) {
        pwm_set_pulse_dt(&pwm_led_user0, pulse);
        pulse = dir ? pulse / 1.1 : pulse * 1.1;
        if (dir && pulse < pwm_min_pulse || !dir && pulse > pwm_max_pulse) {
            dir = !dir;
        }

        k_sleep(K_MSEC(50));

        if (gpio_pin_get_dt(&nfcc_irq)) {
            nci_read(pn7150, read_buf, 255);
        }
    }

    return 0;
}
