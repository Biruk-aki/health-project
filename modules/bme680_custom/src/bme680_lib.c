#include "bme680_lib.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

/**
 * Sends a single byte of data to a specific register on the sensor.
 */
static inline int bme_i2c_write(struct bme680_dev *dev, uint8_t reg, uint8_t val) {
    return i2c_reg_write_byte(dev->i2c_dev, dev->i2c_addr, reg, val);
}

int bme680_read_calibration_data(struct bme680_dev *dev) {
    uint8_t d[2];

    /* Reads two bytes starting at 0xE9 and stores them as a 16-bit unsigned integer in par_t1. */
    if (i2c_burst_read(dev->i2c_dev, dev->i2c_addr, 0xE9, d, 2) ||
        (dev->calib.par_t1 = (uint16_t)d[1] << 8 | d[0], 0)) return -EIO;

    /* Reads two bytes starting at 0x8A and stores them as a 16-bit signed integer in par_t2. */
    if (i2c_burst_read(dev->i2c_dev, dev->i2c_addr, 0x8A, d, 2) ||
        (dev->calib.par_t2 = (int16_t)(d[1] << 8 | d[0]), 0)) return -EIO;

    /* Reads a single byte from 0x8C and stores it as an 8-bit signed integer in par_t3. */
    return i2c_reg_read_byte(dev->i2c_dev, dev->i2c_addr, 0x8C, (uint8_t *)&dev->calib.par_t3);
}

int bme680_calculate_temperature(struct bme680_dev *dev, int32_t adc_raw, int32_t *temp_comp) {
    /* Copies calibration values from the device structure into local variables. */
    int32_t t1 = (int32_t)dev->calib.par_t1;
    int32_t t2 = (int32_t)dev->calib.par_t2;
    int32_t t3 = (int32_t)dev->calib.par_t3;

    /* Implements the Bosch datasheet formula using bit-shifts and integer multiplication. */
    int32_t v1 = (adc_raw >> 3) - (t1 << 1);
    int32_t v2 = (v1 * t2) >> 11;
    int32_t v3 = ((((v1 >> 1) * (v1 >> 1)) >> 12) * (t3 << 4)) >> 14;
    
    /* Sums partial results to create the t_fine variable used for multi-sensor compensation. */
    dev->calib.t_fine = v2 + v3;
    
    /* Scales the final value to Celsius multiplied by 100. */
    *temp_comp = ((dev->calib.t_fine * 5) + 128) >> 8;
    return 0;
}

int bme680_init(struct bme680_dev *dev) {
    uint8_t id;

    /* Reads the chip ID register and compares it against the expected value of 0x61. */
    if (i2c_reg_read_byte(dev->i2c_dev, dev->i2c_addr, BME680_REG_CHIP_ID, &id) || id != BME680_CHIP_ID_VAL)
        return -ENODEV;

    /* Writes the reset command 0xB6 to the reset register. */
    bme_i2c_write(dev, BME680_REG_RESET, 0xB6);
    /* Pauses execution for 10ms to allow the sensor to reboot. */
    k_msleep(10); 

    /* Calls the function to populate the calibration structure from sensor memory. */
    if (bme680_read_calibration_data(dev)) return -EIO;

    /* Writes 0x00 to the gas control register to turn off the heating element. */
    bme_i2c_write(dev, BME680_REG_CTRL_GAS_1, 0x00); 
    
    /* Sets the temperature oversampling to x16 in the measurement control register. */
    bme_i2c_write(dev, BME680_REG_CTRL_MEAS, (0x05 << 5)); 

    return 0;
}

int bme680_read_temperature(struct bme680_dev *dev, int32_t *temp_celsius) {
    uint8_t d[3];

    /* Writes to the control register to put the device into 'Forced Mode' for one reading. */
    bme_i2c_write(dev, BME680_REG_CTRL_MEAS, (0x05 << 5) | 0x01);
    
    /* Waits 20ms for the hardware to complete the analog-to-digital conversion. */
    k_msleep(20); 

    /* Reads 3 bytes of raw ADC data starting from the Temperature MSB register. */
    if (i2c_burst_read(dev->i2c_dev, dev->i2c_addr, BME680_REG_TEMP_MSB, d, 3)) return -EIO;

    /* Combines the 3 bytes into a single 20-bit integer. */
    int32_t adc_raw = (int32_t)(d[0] << 12 | d[1] << 4 | d[2] >> 4);
    
    /* Passes the raw value to the math function to get the compensated Celsius result. */
    return bme680_calculate_temperature(dev, adc_raw, temp_celsius);
}