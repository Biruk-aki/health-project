#ifndef BME680_LIB_H
#define BME680_LIB_H

#include <zephyr/types.h>
#include <zephyr/drivers/i2c.h>

/* I2C Hardware Addresses */
#define BME680_I2C_ADDR_LoW   0x76 
#define BME680_I2C_ADDR_HiGH  0x77 

/* Critical Registers */
#define BME680_REG_CHIP_ID    0xD0
#define BME680_CHIP_ID_VAL    0x61
#define BME680_REG_RESET      0xE0
#define BME680_REG_CTRL_MEAS  0x74
#define BME680_REG_CTRL_GAS_1 0x71
#define BME680_REG_TEMP_MSB   0x22

/**
 * Stores factory-set calibration constants. 
 * These are unique to every individual sensor.
 */
struct bme680_calib_data {
    uint16_t par_t1; /* Unsigned 16-bit */
    int16_t  par_t2; /* Signed 16-bit */
    int8_t   par_t3; /* Signed 8-bit */
    int32_t  t_fine; /* Calculated temperature fine-tuning factor */
};

/**
 * Main handle for the sensor instance.
 */
struct bme680_dev {
    const struct device *i2c_dev; /* The I2C hardware bus */
    uint8_t i2c_addr;             /* The sensor's address (0x76 or 0x77) */
    struct bme680_calib_data calib; 
};

/**
 * @brief Sets up the sensor, verifies identity, and reads calibration.
 * @return 0 on success, negative error code on failure.
 */
int bme680_init(struct bme680_dev *dev);

/**
 * @brief Triggers a measurement and returns temperature in Celsius * 100.
 * Example: 3650 = 36.50 C
 */
int bme680_read_temperature(struct bme680_dev *dev, int32_t *temp_celsius);

#endif /* BME680_LIB_H */