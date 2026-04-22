#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "bme680_lib.h"

// Define the sensor device globally so other tasks can see it
struct bme680_dev body_temp_sensor;

int main(void) {
    int32_t current_temp;

    // 1. Point to the I2C peripheral defined in your overlay
    body_temp_sensor.i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    body_temp_sensor.i2c_addr = BME680_I2C_ADDR_LoW;

    if (!device_is_ready(body_temp_sensor.i2c_dev)) {
        return -1;
    }

    // 2. Initialize your driver
    if (bme680_init(&body_temp_sensor) != 0) {
        return -1;
    }

    while (1) {
        // 3. Read the data
        if (bme680_read_temperature(&body_temp_sensor, &current_temp) == 0) {
            // other sensors can now use current_temp for health analysis

            printk("Body Temp: %d.%02d C\n", 
                    current_temp / 100, current_temp % 100);
        }
        
        k_sleep(K_MSEC(1000)); // Sample every second
    }
}