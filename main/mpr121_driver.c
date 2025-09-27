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
    // Filter and global config (optimized for through-surface detection)
    err |= mpr121_write_reg(dev, MPR121_REG_MHDR, 0x01);   // MHDR (max half delta rising)
    err |= mpr121_write_reg(dev, MPR121_REG_NHDR, 0x01);   // NHDR (noise half delta rising)
    err |= mpr121_write_reg(dev, MPR121_REG_NCLR, 0x0E);   // NCLR (noise count limit rising) - INCREASED for better through-surface detection
    err |= mpr121_write_reg(dev, MPR121_REG_FDLR, 0x00);   // FDLR (filter delay count limit rising)
    err |= mpr121_write_reg(dev, MPR121_REG_MHDF, 0x01);   // MHDF (max half delta falling)
    err |= mpr121_write_reg(dev, MPR121_REG_NHDF, 0x05);   // NHDF (noise half delta falling)
    err |= mpr121_write_reg(dev, MPR121_REG_NCLF, 0x01);   // NCLF (noise count limit falling)
    err |= mpr121_write_reg(dev, MPR121_REG_FDLF, 0x00);   // FDLF (filter delay count limit falling)
    err |= mpr121_write_reg(dev, MPR121_REG_NHDT, 0x00);   // NHDT (noise half delta touched)
    err |= mpr121_write_reg(dev, MPR121_REG_NCLT, 0x00);   // NCLT (noise count limit touched)
    err |= mpr121_write_reg(dev, MPR121_REG_FDLT, 0x00);   // FDLT (filter delay count limit touched)
    
    // Proximity filter settings (same as electrode filters - matches Python driver)
    err |= mpr121_write_reg(dev, 0x36, 0x01);   // MPR121_MHDPROXR (max half delta proximity rising)
    err |= mpr121_write_reg(dev, 0x37, 0x01);   // MPR121_NHDPROXR (noise half delta proximity rising)
    err |= mpr121_write_reg(dev, 0x38, 0x0E);   // MPR121_NCLPROXR (noise count limit proximity rising)
    err |= mpr121_write_reg(dev, 0x39, 0x00);   // MPR121_FDLPROXR (filter delay count limit proximity rising)
    err |= mpr121_write_reg(dev, 0x3A, 0x01);   // MPR121_MHDPROXF (max half delta proximity falling)
    err |= mpr121_write_reg(dev, 0x3B, 0x05);   // MPR121_NHDPROXF (noise half delta proximity falling)
    err |= mpr121_write_reg(dev, 0x3C, 0x01);   // MPR121_NCLPROXF (noise count limit proximity falling)
    err |= mpr121_write_reg(dev, 0x3D, 0x00);   // MPR121_FDLPROXF (filter delay count limit proximity falling)
    err |= mpr121_write_reg(dev, 0x3E, 0x00);   // MPR121_NHDPROXT (noise half delta proximity touched)
    err |= mpr121_write_reg(dev, 0x3F, 0x00);   // MPR121_NCLPROXT (noise count limit proximity touched)
    err |= mpr121_write_reg(dev, 0x40, 0x00);   // MPR121_FDLPROXT (filter delay count limit proximity touched)
    
    // Debounce: DT=0, DR=0 -> 0x00 (fastest response for debug)
    err |= mpr121_write_reg(dev, MPR121_REG_DEBOUNCE, 0x00);
    // CONFIG1 (0x5C): FFI[7:6]=00, CDC[5:0]=0x10 (16uA - matches Python driver)
    err |= mpr121_write_reg(dev, MPR121_REG_CONFIG1, 0x10);
    // CONFIG2 (0x5D): CDT=100 (0.5us encoding), SFI=00 (min filter), ESI=000 (1ms period) -> 0x20
    err |= mpr121_write_reg(dev, MPR121_REG_CONFIG2, 0x20);
    if (err) return err;

    // Per-electrode CDC/CDT boost (optimized for through-surface detection)
    for (uint8_t ch = 0; ch < MPR121_NUM_ELECTRODES; ch++) {
        err |= mpr121_write_reg(dev, 0x5F + ch, 0x10); // CDCx = 16uA (matches global setting)
        // Note: CDT registers (0x6C+) are shared between electrodes, handled by global CONFIG2
    }
    if (err) return err;

    // Auto-configuration settings (matches Python driver exactly)
    // USL = int(((3.3 - 0.7) * 256) / 3.3) = 202
    err |= mpr121_write_reg(dev, 0x7D, 202);  // MPR121_AUTO_USL
    // TL = int(((3.3 - 0.7) * 256 * 0.9) / 3.3) = 182  
    err |= mpr121_write_reg(dev, 0x7F, 182);  // MPR121_AUTO_TL
    // LSL = int(((3.3 - 0.7) * 256 * 0.65) / 3.3) = 131
    err |= mpr121_write_reg(dev, 0x7E, 131);  // MPR121_AUTO_LSL
    // Auto-config control
    err |= mpr121_write_reg(dev, 0x7B, 0x0B); // MPR121_AUTO_CTRL0
    if (err) return err;

    // Enable all electrodes, run mode (matches Python driver exactly)
    // ECR: 0x8F = CL=10 (initialize baselines), ELE_EN=15 (all electrodes + proximity)
    err = mpr121_write_reg(dev, MPR121_REG_ECR, 0x8F);
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

int mpr121_set_proximity_mode(mpr121_t *dev, uint8_t mode) {
    // Set the proximity mode (virtual 13th electrode)
    // mode: 0 = disabled, 1 = EL0-EL1, 2 = EL0-EL3, 3 = EL0-EL11
    uint8_t ELEPROX_EN = (mode & 3) << 4;
    uint8_t ECR;
    
    // Read current ECR
    int err = mpr121_read_reg(dev, MPR121_REG_ECR, &ECR);
    if (err) return err;
    
    // Clear old proximity mode and set new one
    ECR = (ECR & 0xCF) | ELEPROX_EN;
    
    // Stop chip, update ECR, then start chip
    err = mpr121_write_reg(dev, MPR121_REG_ECR, 0x00);
    if (err) return err;
    err = mpr121_write_reg(dev, MPR121_REG_ECR, ECR);
    return err;
}
