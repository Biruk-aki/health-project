#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>

#define SENSOR_NODE_ADDR  0x60

#define CMD_HEART_RATE    0x01
#define CMD_BLOOD_OXYGEN  0x02
#define CMD_TEMPERATURE   0x03
#define CMD_PING          0xFF

int main(void)
{
    const struct device *i2c_dev  = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint32_t dtr = 0;
    uint8_t c = 0;

    /* Wait for USB terminal */
    while (!dtr) {
        uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }
uart_irq_rx_enable(uart_dev);
    /* Setup I2C controller */
    if (!device_is_ready(i2c_dev)) {
        printk("ERROR: I2C not ready\n");
        return 1;
    }

    printk("\n=== Health Monitor Base Node ===\n");

    /* Ping sensor node */
    uint8_t ping = CMD_PING;
    uint8_t id   = 0;
    if (i2c_write_read(i2c_dev, SENSOR_NODE_ADDR, &ping, 1, &id, 1) < 0
        || id != 0xEE) {
        printk("ERROR: Sensor node not found at 0x%02x\n", SENSOR_NODE_ADDR);
        return 1;
    }
    printk("Sensor node found at 0x%02x\n", SENSOR_NODE_ADDR);

    /* Main loop */
    while (1) {

        /* Prompt user for sensor reading mode */
        printk("\nSelect sensor:\n");
        printk("  1 = Heart Rate\n");
        printk("  2 = SpO2\n");
        printk("  3 = Temperature\n");
        printk("Enter choice: ");

        while (uart_poll_in(uart_dev, &c) != 0) {
            k_sleep(K_MSEC(10));
        }
        printk("%c\n", c);

        uint8_t cmd;
        switch (c) {
            case '1': cmd = CMD_HEART_RATE;   break;
            case '2': cmd = CMD_BLOOD_OXYGEN; break;
            case '3': cmd = CMD_TEMPERATURE;  break;
            default:
                printk("Invalid. Press 1, 2 or 3.\n");
                continue;
        }

        /* Poll and display sensor data from sensor node */
        printk("Polling... press any key to return to menu\n\n");

        while (1) {
            uint8_t raw[4] = {0};

            if (i2c_write_read(i2c_dev, SENSOR_NODE_ADDR,
                               &cmd, 1, raw, 4) < 0) {
                printk("ERROR: Failed to read sensor\n");
            } else {
                uint32_t value = (uint32_t)raw[0]          |
                                ((uint32_t)raw[1] << 8)   |
                                ((uint32_t)raw[2] << 16)  |
                                ((uint32_t)raw[3] << 24);

                if (cmd == CMD_TEMPERATURE) {
                    printk("Temperature: %d.%02d C\n",
                           value / 100, value % 100);
                } else if (cmd == CMD_HEART_RATE) {
                    printk("Heart Rate: %d BPM\n", value);
                } else {
                    printk("SpO2: %d%%\n", value);
                }
            }

            k_msleep(1000);

            /* Any key press returns to menu */
            if (uart_poll_in(uart_dev, &c) == 0) {
                break;
            }
        }
    }

    return 0;
}