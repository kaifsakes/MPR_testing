#include "mpr121_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// MPR121 register addresses (from datasheet)
#define MPR121_REG_TOUCH_STATUS_L 0x00
#define MPR121_REG_TOUCH_STATUS_H 0x01
#define MPR121_REG_ELE0_T 0x41
#define MPR121_REG_ELE0_R 0x42
#define MPR121_REG_MHDR 0x2B
#define MPR121_REG_NHDR 0x2C
#define MPR121_REG_NCLR 0x2D
#define MPR121_REG_FDLR 0x2E
#define MPR121_REG_MHDF 0x2F
#define MPR121_REG_NHDF 0x30
#define MPR121_REG_NCLF 0x31
#define MPR121_REG_FDLF 0x32
#define MPR121_REG_NHDT 0x33
#define MPR121_REG_NCLT 0x34
#define MPR121_REG_FDLT 0x35
#define MPR121_REG_DEBOUNCE 0x5B
#define MPR121_REG_CONFIG1 0x5C
#define MPR121_REG_CONFIG2 0x5D
#define MPR121_REG_ECR 0x5E
#define MPR121_REG_SOFTRESET 0x80
#define MPR121_REG_FILTERED_BASE 0x04
#define MPR121_REG_BASELINE_BASE 0x1E

#define MPR121_NUM_ELECTRODES 12

void mpr121_init(mpr121_t *dev, void *i2c_context, mpr121_i2c_write_fn write_fn, mpr121_i2c_read_fn read_fn, uint8_t address) {
    dev->i2c_context = i2c_context;
    dev->i2c_write = write_fn;
    dev->i2c_read = read_fn;
    dev->address = address;
    dev->last_touch = 0;
    dev->last_release = 0;
}

static int mpr121_write_reg(mpr121_t *dev, uint8_t reg, uint8_t val) {
    return dev->i2c_write(dev->i2c_context, dev->address, reg, &val, 1);
}

static int mpr121_read_reg(mpr121_t *dev, uint8_t reg, uint8_t *val) {
    return dev->i2c_read(dev->i2c_context, dev->address, reg, val, 1);
}

int mpr121_configure(mpr121_t *dev, uint8_t touch_thresh, uint8_t release_thresh) {
    // Soft reset
    int err = mpr121_write_reg(dev, MPR121_REG_SOFTRESET, 0x63);
    if (err) return err;
    // Stop mode
    err = mpr121_write_reg(dev, MPR121_REG_ECR, 0x00);
    if (err) return err;
    // Set thresholds for all electrodes
    for (uint8_t i = 0; i < MPR121_NUM_ELECTRODES; ++i) {
        err = mpr121_write_reg(dev, MPR121_REG_ELE0_T + i*2, touch_thresh);
        if (err) return err;
        err = mpr121_write_reg(dev, MPR121_REG_ELE0_R + i*2, release_thresh);
        if (err) return err;
    }
    // Filter and global config (typical values)
    err |= mpr121_write_reg(dev, MPR121_REG_MHDR, 0x01);
    err |= mpr121_write_reg(dev, MPR121_REG_NHDR, 0x01);
    err |= mpr121_write_reg(dev, MPR121_REG_NCLR, 0x04);
    err |= mpr121_write_reg(dev, MPR121_REG_FDLR, 0x00);
    err |= mpr121_write_reg(dev, MPR121_REG_MHDF, 0x01);
    err |= mpr121_write_reg(dev, MPR121_REG_NHDF, 0x05);
    err |= mpr121_write_reg(dev, MPR121_REG_NCLF, 0x01);
    err |= mpr121_write_reg(dev, MPR121_REG_FDLF, 0x00);
    err |= mpr121_write_reg(dev, MPR121_REG_NHDT, 0x00);
    err |= mpr121_write_reg(dev, MPR121_REG_NCLT, 0x00);
    err |= mpr121_write_reg(dev, MPR121_REG_FDLT, 0x00);
    // Debounce: DT=0, DR=0 -> 0x00 (fastest response for debug)
    err |= mpr121_write_reg(dev, MPR121_REG_DEBOUNCE, 0x00);
    // CONFIG1 (0x5C): FFI[7:6]=00, CDC[5:0]=0x3F (63uA max)
    err |= mpr121_write_reg(dev, MPR121_REG_CONFIG1, 0x3F);
    // CONFIG2 (0x5D): CDT=111 (32us), SFI=00 (min filter), ESI=001 (~1ms) -> 0xE1
    err |= mpr121_write_reg(dev, MPR121_REG_CONFIG2, 0xE1);
    if (err) return err;

    // Optional: per-electrode CDC/CDT boost (useful for through-cover)
    for (uint8_t ch = 0; ch < MPR121_NUM_ELECTRODES; ch++) {
        err |= mpr121_write_reg(dev, 0x5F + ch, 0x3F); // CDCx = 63uA
        err |= mpr121_write_reg(dev, 0x6C + ch, 0x07); // CDTx ≈ 32us encoding
    }
    if (err) return err;
    // Enable all electrodes, run mode
    // ECR: First, initialize baselines: CL=10, ELE_EN=12
    err = mpr121_write_reg(dev, MPR121_REG_ECR, 0x8C);
    if (err) return err;
    vTaskDelay(pdMS_TO_TICKS(1));
    // Then switch to normal baseline tracking: CL=01, ELE_EN=12
    err = mpr121_write_reg(dev, MPR121_REG_ECR, 0x4C);
    return err;
}

int mpr121_read_touch(mpr121_t *dev, uint16_t *touch_bits) {
    uint8_t buf[2];
    int err = dev->i2c_read(dev->i2c_context, dev->address, MPR121_REG_TOUCH_STATUS_L, buf, 2);
    if (err) return err;
    *touch_bits = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return 0;
}

int mpr121_set_thresholds(mpr121_t *dev, uint8_t touch_thresh, uint8_t release_thresh) {
    int err = 0;
    for (uint8_t i = 0; i < MPR121_NUM_ELECTRODES; ++i) {
        err |= mpr121_write_reg(dev, MPR121_REG_ELE0_T + i*2, touch_thresh);
        err |= mpr121_write_reg(dev, MPR121_REG_ELE0_R + i*2, release_thresh);
    }
    return err;
}

int mpr121_set_electrode_threshold(mpr121_t *dev, uint8_t electrode, uint8_t touch_thresh, uint8_t release_thresh) {
    if (electrode >= MPR121_NUM_ELECTRODES) return -1;
    int err = mpr121_write_reg(dev, MPR121_REG_ELE0_T + electrode*2, touch_thresh);
    if (err) return err;
    return mpr121_write_reg(dev, MPR121_REG_ELE0_R + electrode*2, release_thresh);
}

int mpr121_read_filtered(mpr121_t *dev, uint8_t electrode, uint16_t *value) {
    if (electrode >= MPR121_NUM_ELECTRODES) return -1;
    uint8_t buf[2];
    int err = dev->i2c_read(dev->i2c_context, dev->address, MPR121_REG_FILTERED_BASE + electrode*2, buf, 2);
    if (err) return err;
    *value = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return 0;
}

int mpr121_reinitialize_baseline(mpr121_t *dev) {
    // Set CL bits to 10 to reinitialize baseline to current value, then set to 01 for tracking
    int err = mpr121_write_reg(dev, MPR121_REG_ECR, 0x8C);
    if (err) return err;
    vTaskDelay(pdMS_TO_TICKS(1));
    return mpr121_write_reg(dev, MPR121_REG_ECR, 0x4C);
}

int mpr121_read_baseline(mpr121_t *dev, uint8_t electrode, uint16_t *value) {
    if (electrode >= MPR121_NUM_ELECTRODES) return -1;
    uint8_t val;
    int err = mpr121_read_reg(dev, MPR121_REG_BASELINE_BASE + electrode, &val);
    if (err) return err;
    *value = ((uint16_t)val) << 2; // baseline is upper 8 bits, lower 2 bits always 0
    return 0;
}
