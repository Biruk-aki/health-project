// communication with BME680 sensor
#define BME680_I2C_ADDR_LoW 0x76 // SD0 coonected to GND
#define BME680_I2C_ADDR_HiGH 0x77 // SD0 coonected to VDDIO
#define BME680_CHIP_ID_VAL 0x61 // BME680 registers for every BME680 sensor

// BME680 ADDRESS where we send commands to pick up data
#define BME680_REG_CHIP_ID      0xD0  // Register to verify the sensor is present [cite: 682, 721]
#define BME680_REG_RESET        0xE0  // Write 0xB6 here to soft-reset [cite: 682, 717]
#define BME680_REG_CTRL_HUM     0x72  // Controls humidity oversampling [cite: 682, 724]
#define BME680_REG_CTRL_MEAS    0x74  // Controls Temp/Pressure oversampling & Mode [cite: 682, 705]
#define BME680_REG_CTRL_GAS_1   0x71  // We use this to disable the heater [cite: 682, 781]

//BME680 spreads the calibration data across several registers

#define BME680_REG_TEMP_MSB     0x22  // Most Significant Byte [cite: 682, 792]
#define BME680_REG_TEMP_LSB     0x23  // Least Significant Byte [cite: 682, 792]
#define BME680_REG_TEMP_XLSB    0x24  // Extra Least Significant Byte [cite: 682, 792]

#include <zephyr/types.h>
#include <zephyr/drivers/i2c.h>

/**
 * @brief Structure to hold factory calibration data for temperature.
 * These values are read once during sensor initialization.
 */
struct bme680_calib_data {
    uint16_t par_t1;  // Unsigned 16-bit 
    int16_t  par_t2;  // Signed 16-bit 
    int8_t   par_t3;  // Signed 8-bit 
    
    /* We also store t_fine, a global temperature factor used 
       by other sensors (like pressure) later. */
    int32_t t_fine; 
};

/**
 * @brief Main device structure for our standalone driver.
 */
struct bme680_dev {
    const struct device *i2c_dev;
    uint8_t i2c_addr;
    struct bme680_calib_data calib;
};

/**
 * @brief Initialize the BME680: Verifies Chip ID and reads calibration data.
 * @return 0 on success, negative errno on failure.
 */
int bme680_init(struct bme680_dev *dev);

/**
 * @brief Reads the raw ADC temperature and calculates the "pure" Celsius value.
 * @param temp Pointer to store the result (in degrees Celsius).
 * @return 0 on success.
 */
int bme680_read_temperature(struct bme680_dev *dev, double *temp);