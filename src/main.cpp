#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/usb_device.h>

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

void hexdump(const uint8_t* buf, uint32_t len) {
    for (int i = 0; i < len; i++) {
        printf("%02x", buf[i]);    
        if(i < len-1) {
            printf(":");
        }
    }
}

void hexdump(const char* prefix, const uint8_t* buf, uint32_t len) {
    fputs(prefix, stdout);
    hexdump(buf, len);
}

int main(void) {
    uint8_t read_buf[256]; // read buffer for i2c
    int ret = 0; // buffer for function return values

    ret = serial_init();
    if (ret != 0) {
        return ret;
    }

    const struct gpio_dt_spec btn_user1 = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_user1), gpios);
    const struct gpio_dt_spec btn_user2 = GPIO_DT_SPEC_GET(DT_NODELABEL(btn_user2), gpios);

    ret = (!device_is_ready(btn_user1.port));
    if(ret) {
        puts("Could not get User Button 1.");
        return ret;
    } else {
        puts("Got User Button 1.");
    }
    
    ret = (!device_is_ready(btn_user2.port));
    if(ret) {
        puts("Could not get User Button 2.");
        return ret;
    } else {
        puts("Got User Button 2.");
    }
    
    const struct gpio_dt_spec led_user0 = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);
    const struct gpio_dt_spec led_user1 = GPIO_DT_SPEC_GET(DT_NODELABEL(led_user1), gpios);
    const struct gpio_dt_spec led_user2 = GPIO_DT_SPEC_GET(DT_NODELABEL(led_user2), gpios);

    ret = (!device_is_ready(led_user0.port));
    if(ret) {
        puts("Could not get User LED 0.");
        return ret;
    } else {
        puts("Got User LED 0.");
    }

    ret = (!device_is_ready(led_user1.port));
    if(ret) {
        puts("Could not get User LED 1.");
        return ret;
    } else {
        puts("Got User LED 1.");
    }
    
    ret = (!device_is_ready(led_user2.port));
    if(ret) {
        puts("Could not get User LED 2.");
        return ret;
    } else {
        puts("Got User LED 2.");
    }

    ret = gpio_pin_configure(led_user0.port, led_user0.pin, GPIO_OUTPUT);
    ret = gpio_pin_configure(led_user1.port, led_user1.pin, GPIO_OUTPUT);
    ret = gpio_pin_configure(led_user2.port, led_user2.pin, GPIO_OUTPUT);

    gpio_pin_set_dt(&led_user0, 1);
    
    const struct gpio_dt_spec nfcc_ven = GPIO_DT_SPEC_GET(DT_NODELABEL(nfcc_ven), gpios);
    const struct gpio_dt_spec nfcc_irq = GPIO_DT_SPEC_GET(DT_NODELABEL(nfcc_irq), gpios);
    
    ret = (!device_is_ready(nfcc_ven.port));
    if(ret) {
        puts("Could not get NFCC's VEN.");
        return ret;
    } else {
        puts("Got NFCC's VEN.");
    }
     
    ret = (!device_is_ready(nfcc_irq.port));
    if(ret) {
        puts("Could not get NFCC's IRQ.");
        return ret;
    } else {
        puts("Got NFCC's IRQ.");
    }
   
    ret = gpio_pin_configure(nfcc_ven.port, nfcc_ven.pin, GPIO_OUTPUT);
    if(ret) {
        printf("Could not configure NFCC's VEN (%i).\n", ret);
        return ret;
    } else {
        puts("Set NFCC's VEN config.");
    }
    
    ret = gpio_pin_configure(nfcc_irq.port, nfcc_irq.pin, GPIO_INPUT | GPIO_PULL_DOWN);
    if(ret) {
        printf("Could not configure NFCC's IRQ (%i).\n", ret);
        return ret;
    } else {
        puts("Set NFCC's IRQ config.");
    }

    ret = gpio_pin_set_dt(&nfcc_ven, 1);
    k_sleep(K_MSEC(500));
    if(ret) {
        printf("Could not set NFCC's VEN (%i).\n", ret);
        return ret;
    } else {
        puts("Set NFCC's VEN to ACTIVE.");
    }

    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

    ret = (i2c_dev == NULL || !device_is_ready(i2c_dev));
    if(ret) {
        puts("Could not get I2C controller device.");
        return ret;
    } else {
        puts("Got I2C controller device.");
    }

    const struct i2c_dt_spec pn7150 = I2C_DT_SPEC_GET(DT_NODELABEL(pn7150));
    puts("Got PN7150 dt spec.");

    ret = i2c_is_ready_dt(&pn7150);
    if(ret) {
        puts("i2c_is_ready: true");
    }
    else {
        puts("i2c_is_ready: false");
        return 1;
    }

    uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_FAST) | I2C_MODE_CONTROLLER;
	uint32_t i2c_cfg_tmp = 0;

    ret = i2c_configure(i2c_dev, i2c_cfg);
    printf("i2c_configure: %i\n", ret);
    if(ret) {
        printf("i2c_configure failed: %i\n", ret);
    }
    else {
        puts("I2C config set successfully.");
    }

    ret = i2c_get_config(i2c_dev, &i2c_cfg_tmp);
    printf("i2c_get_config: %i\n", ret);
    if(ret == -88) {
        puts("Config get not supported. (ENOSYS returned)");
    }
    else if(ret) {
        printf("Could not get controller config. (%i)", ret);
        return ret;
    }
    else {
        if(i2c_cfg_tmp != i2c_cfg) {
            printf("Config was not set correctly.");
            hexdump("\nretreived: ", (uint8_t*) &i2c_cfg_tmp, 4);
            hexdump("\nintended: ", (uint8_t*) &i2c_cfg, 4);
            puts("");
            return 1;
        }
    }
  
    ret = i2c_reg_write_byte_dt(&pn7150, 0, 69);
    printf("i2c_reg_write_byte_dt: %i\n", ret);

    uint8_t cmd[] = {0x20,0x00,0x01,0x01}; // CORE_RESET_CMD(0x01), reset config
    puts("Attempting reset command send...");
    ret = i2c_write_dt(&pn7150, cmd, sizeof(cmd));
    printf("i2c_write_dt: %i\n", ret);
    hexdump("> ", cmd, sizeof(cmd));
    puts("");

    // i2c_read(i2c_dev, read_buf, 255, 0x28);

    puts("Reached end.");

    return 0;
}
