#ifndef MPR121_DRIVER_H
#define MPR121_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// I2C abstraction: user must provide these functions for their platform
// Return 0 on success, nonzero on error
// context: user pointer for passing driver state
// addr: 7-bit I2C address
// reg: register address
// data: pointer to data buffer
// len: number of bytes
typedef int (*mpr121_i2c_write_fn)(void *context, uint8_t addr, uint8_t reg, const uint8_t *data, uint16_t len);
typedef int (*mpr121_i2c_read_fn)(void *context, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len);

// MPR121 driver state
typedef struct {
    void *i2c_context;
    mpr121_i2c_write_fn i2c_write;
    mpr121_i2c_read_fn i2c_read;
    uint8_t address;
    uint16_t last_touch;
    uint16_t last_release;
} mpr121_t;

// API
// Initialize driver struct (does not touch hardware)
void mpr121_init(mpr121_t *dev, void *i2c_context, mpr121_i2c_write_fn write_fn, mpr121_i2c_read_fn read_fn, uint8_t address);
// Reset and configure the MPR121 (returns 0 on success)
int mpr121_configure(mpr121_t *dev, uint8_t touch_thresh, uint8_t release_thresh);
// Read current touch status (bitmask, 1=touch)
int mpr121_read_touch(mpr121_t *dev, uint16_t *touch_bits);
// Set touch/release threshold for all electrodes
int mpr121_set_thresholds(mpr121_t *dev, uint8_t touch_thresh, uint8_t release_thresh);
// Set touch/release threshold for one electrode
int mpr121_set_electrode_threshold(mpr121_t *dev, uint8_t electrode, uint8_t touch_thresh, uint8_t release_thresh);
// Read filtered data for one electrode
int mpr121_read_filtered(mpr121_t *dev, uint8_t electrode, uint16_t *value);
// Read baseline data for one electrode
int mpr121_read_baseline(mpr121_t *dev, uint8_t electrode, uint16_t *value);

#ifdef __cplusplus
}
#endif

#endif // MPR121_DRIVER_H
