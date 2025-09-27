#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"
#include "mpr121_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---- CONFIGURABLE ----
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define MPR121_I2C_ADDR             0x5A
#define POLL_DELAY_MS               100
// ----------------------

// ESP-IDF I2C write function for MPR121 driver
int espidf_i2c_write(void *context, uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    if (len > 0) {
        i2c_master_write(cmd, (uint8_t *)data, len, true);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? 0 : -1;
}

// ESP-IDF I2C read function for MPR121 driver
int espidf_i2c_read(void *context, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? 0 : -1;
}

static void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Electrode names for user-friendly output
static const char *electrode_names[12] = {
    NULL, NULL, NULL, // 0, 1, 2
    "start",         // 3
    "temperature down", // 4
    "temperature up",   // 5
    "finish",           // 6
    "keep warm",        // 7
    "stir up",          // 8
    "stir down",        // 9
    "connect",          // 10
    "dispense"          // 11
};

void app_main(void) {
    i2c_master_init();
    printf("I2C initialized.\n");

    mpr121_t mpr;
    mpr121_init(&mpr, NULL, espidf_i2c_write, espidf_i2c_read, MPR121_I2C_ADDR);
    // Optimized thresholds for through-surface detection: touch=12, release=6 (matches Python driver)
    if (mpr121_configure(&mpr, 12, 6) != 0) {
        printf("Failed to configure MPR121!\n");
        return;
    }
    printf("MPR121 configured. Monitoring touch electrodes...\n");
    // Re-initialize baseline once after configuration to align with current environment/cover
    (void)mpr121_reinitialize_baseline(&mpr);

    uint16_t last_touch = 0;
    while (1) {
        uint16_t touch = 0;
        if (mpr121_read_touch(&mpr, &touch) == 0) {
            if (touch != last_touch) {
                for (int i = 0; i < 12; ++i) {
                    bool now = (touch >> i) & 1;
                    bool before = (last_touch >> i) & 1;
                    if (now && !before) {
                        if (electrode_names[i]) {
                            printf("Electrode %d (%s) touched!\n", i, electrode_names[i]);
                        } else {
                            printf("Electrode %d touched!\n", i);
                        }
                    } else if (!now && before) {
                        if (electrode_names[i]) {
                            printf("Electrode %d (%s) released.\n", i, electrode_names[i]);
                        } else {
                            printf("Electrode %d released.\n", i);
                        }
                    }
                }
                last_touch = touch;
            }
        } else {
            printf("Error reading touch status!\n");
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));
    }
} 