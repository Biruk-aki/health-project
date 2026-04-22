#include "bme680_lib.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

// Function to read calibration data from the BME680 sensor
int bme680_read_calibration_data(struct bme680_dev *dev){
uint8_t data [2];
// read par_t1 (unsigned 16-bit) from 0xE9 (LCB) and 0xEA (MSB)

if(i2c_burst_read(dev->i2c_dev, dev->i2c_addr, 0xE9, data, 2) != 0){
    return -EIO; // I/O error

}
dev-> calib.par_t1= (uint16_t)data[1] << 8 | data[0]; // Combine MSB and LSB

// read par_t2 (signed 16-bit) from 0x8A (LSB) and 0x8B (MSB)
if(i2c_burst_read(dev->i2c_dev, dev->i2c_addr, 0x8A, data, 2) != 0){
    return -EIO; // I/O error

}
dev-> calib.par_t2= (int16_t)((data[1] << 8 | data[0])); // Combine MSB and LSB

// read par_t3 (signed 8-bit) from 0x8C
if(i2c_reg_read_byte(dev->i2c_dev, dev->i2c_addr, 0x8C, (uint8_t *)&dev->calib.par_t3) != 0){
    return -EIO; // I/O error

}return 0; // Success

}

// Function to calculate compensated temperature in Celsius (multiplied by 100 for precision)
int bme680_calculate_temperature(struct bme680_dev *dev, int32_t temp_adc, int32_t *temp_comp) {
    int32_t var1, var2, var3;

    // The formula from Bosch Datasheet [cite: 417, 418, 419, 420, 421]
    var1 = ((int32_t)temp_adc >> 3) - ((int32_t)dev->calib.par_t1 << 1);
    var2 = (var1 * (int32_t)dev->calib.par_t2) >> 11;
    var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t)dev->calib.par_t3 << 4)) >> 14;
    
    // Calculate t_fine (needed for other sensors later)
    dev->calib.t_fine = var2 + var3;
    
    // Final temperature in Celsius (multiplied by 100 for precision in integer)
    *temp_comp = ((dev->calib.t_fine * 5) + 128) >> 8;
    
    return 0;
}

// Initialize the BME680 sensor: Verify Chip ID, read calibration, disable heater, set oversampling

int bme680_init(struct bme680_dev *dev) {
    uint8_t chip_id;
    uint8_t reg_val;

    // 1. Verify Chip ID: Read register 0xD0 [cite: 721]
    if (i2c_reg_read_byte(dev->i2c_dev, dev->i2c_addr, BME680_REG_CHIP_ID, &chip_id) != 0) {
        return -EIO;
    }
    if (chip_id != BME680_CHIP_ID_VAL) {
        return -ENODEV; // Wrong chip detected! [cite: 721]
    }

    // 2. Soft Reset: Write 0xB6 to 0xE0 [cite: 719]
    reg_val = 0xB6;
    i2c_reg_write_byte(dev->i2c_dev, dev->i2c_addr, BME680_REG_RESET, reg_val);
    k_msleep(10); // Wait for the sensor to wake up again [cite: 218]

    // 3. Read Calibration Data [cite: 429]
    if (bme680_read_calibration(dev) != 0) {
        return -EIO;
    }

    // 4. Disable Gas Heater: Set run_gas bit (bit 4) to 0 in register 0x71 [cite: 360, 781]
    // Also sets nb_conv to 0. This ensures no self-heating [cite: 776]
    reg_val = 0x00; 
    i2c_reg_write_byte(dev->i2c_dev, dev->i2c_addr, BME680_REG_CTRL_GAS_1, reg_val);

    // 5. Set Oversampling: Temp x16 for "pure" results [cite: 347, 728]
    // Register 0x74: osrs_t is bits <7:5>. 0b101 = x16 [cite: 727, 728]
    // We keep Pressure (bits 4:2) and Mode (bits 1:0) at 0 for now [cite: 734, 706]
    reg_val = (0x05 << 5); 
    i2c_reg_write_byte(dev->i2c_dev, dev->i2c_addr, BME680_REG_CTRL_MEAS, reg_val);

    return 0;
}

// read function for temperature, returns temp in Celsius as an integer (multiplied by 100 for precision)
int bme680_read_temperature(struct bme680_dev *dev, int32_t *temp_celsius) {
    uint8_t data[3];
    int32_t adc_raw;

    // 1. Trigger Forced Mode: Write 0b01 to mode bits in 0x74 [cite: 367, 705]
    // We preserve our x16 oversampling (0x05 << 5) [cite: 728]
    uint8_t ctrl_meas = (0x05 << 5) | 0x01;
    i2c_reg_write_byte(dev->i2c_dev, dev->i2c_addr, BME680_REG_CTRL_MEAS, ctrl_meas);

    // 2. Wait for measurement to finish (typical time is small) [cite: 218]
    k_msleep(20); 

    // 3. Read Raw ADC: Registers 0x22, 0x23, 0x24 [cite: 682, 792]
    if (i2c_burst_read(dev->i2c_dev, dev->i2c_addr, BME680_REG_TEMP_MSB, data, 3) != 0) {
        return -EIO;
    }

    // 4. Assemble the 20-bit ADC value [cite: 792]
    adc_raw = (int32_t)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));

    // 5. Apply the math formula we wrote earlier!
    return bme680_calculate_temperature(dev, adc_raw, temp_celsius);
}
//sync test