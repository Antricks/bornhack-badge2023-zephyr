#include <string.h>
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
    uint8_t read_buf[256];
    int ret = 0;

    ret = serial_init();
    if (ret != 0) {
        return ret;
    }

    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(pico_i2c1));

    ret = (i2c_dev == NULL || !device_is_ready(i2c_dev));
    if(ret) {
        puts("Could not get I2C controller device.");
        return ret;
    } else {
        puts("Got I2C controller device.");
    }

    uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_FAST) | I2C_MODE_CONTROLLER;
	uint32_t i2c_cfg_tmp = 0;

    printf("i2c_configure: %i\n", i2c_configure(i2c_dev, i2c_cfg));
    
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
    
    uint8_t cmd[] = {0x20,0x00,0x01,0x01}; // CORE_RESET_CMD(0x01), reset config
    puts("Attempting reset command send...");
    printf("i2c_write: %i\n", i2c_write(i2c_dev, cmd, sizeof(cmd), 0x28));
    hexdump("> ", cmd, sizeof(cmd));
    puts("");

    // i2c_read(i2c_dev, read_buf, 255, 0x28);

    puts("Reached end.");

    return 0;
}
