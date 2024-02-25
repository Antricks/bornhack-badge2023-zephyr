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

#include "config.h"
#include "example-nci.h"
#include "util.h"

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
static const struct i2c_dt_spec pn7150_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(pn7150));

static uint8_t read_buf[256] = {0};

static ExampleNci nci = ExampleNci(pn7150_i2c, nfcc_irq, read_buf);

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
        k_msleep(100);
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
    int irq_val = gpio_pin_get_dt(&nfcc_irq);

#if DEBUG_NFCC_IRQ
    printf("+++ IRQ=%i +++\n", irq_val);
#endif

    gpio_pin_set_dt(&led_user2, irq_val);
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
        printf("Error: PWM device %s is not ready\n", pwm_led_user0.dev->name);
        return 0;
    } else {
        puts("Got User LED 0 PWM.");
    }
    if (!pwm_is_ready_dt(&pwm_led_user1)) {
        printf("Error: PWM device %s is not ready\n", pwm_led_user1.dev->name);
        return 0;
    } else {
        puts("Got User LED 1 PWM.");
    }

    if (!pwm_is_ready_dt(&pwm_led_user2)) {
        printf("Error: PWM device %s is not ready\n", pwm_led_user2.dev->name);
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
    k_msleep(200);

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

    ret = i2c_is_ready_dt(&pn7150_i2c);
    if (ret) {
        puts("i2c_is_ready: true");
    } else {
        puts("i2c_is_ready: false");
        return 1;
    }

    uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_FAST) | I2C_MODE_CONTROLLER;
    uint32_t i2c_cfg_tmp = 0;

    ret = i2c_configure(i2c_dev, i2c_cfg);
    if (ret) {
        printf("i2c_configure failed: %i\n", ret);
    } else {
        puts("I2C config set successfully.");
    }

    ret = i2c_get_config(i2c_dev, &i2c_cfg_tmp);
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

    nci.nfcc_setup();

    // nci.nfca_nfc_dep_setup();
    // nci.nfca_iso_dep_setup();
    nci.nfcb_iso_dep_setup();

    // TODO adjust dynamically based on period from dt
    uint32_t pwm_min_pulse = PWM_NSEC(5000);
    uint32_t pwm_max_pulse = PWM_MSEC(1);
    bool dir = true;
    uint32_t pulse = pwm_min_pulse;

    while (true) {
        pwm_set_pulse_dt(&pwm_led_user0, pulse);
        pulse = dir ? pulse / 1.05 : pulse * 1.05;
        if ((dir && pulse < pwm_min_pulse) || (!dir && pulse > pwm_max_pulse)) {
            dir = !dir;
        }

        k_msleep(30);

        if (gpio_pin_get_dt(&nfcc_irq)) {
            nci.nci_read();
        }
    }

    return 0;
}
