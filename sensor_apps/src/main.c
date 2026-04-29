#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <stdint.h>

/* BME680 I2C address */
#define BME680_ADDR             0x77

/* BME680 control registers */
#define BME680_REG_CHIP_ID      0xD0
#define BME680_CHIP_ID_VAL      0x61
#define BME680_REG_RESET        0xE0
#define BME680_RESET_VAL        0xB6
#define BME680_REG_CTRL_HUM     0x72
#define BME680_REG_CTRL_MEAS    0x74

/* BME680 status register */
#define BME680_REG_STATUS       0x1D
#define BME680_STATUS_MEASURING (1 << 5)

/* BME680 temperature data registers */
#define BME680_REG_TEMP_MSB     0x22
#define BME680_REG_TEMP_LSB     0x23
#define BME680_REG_TEMP_XLSB    0x24

/* BME680 calibration registers */
#define BME680_REG_T1_LSB       0xE9
#define BME680_REG_T2_LSB       0x8A
#define BME680_REG_T3           0x8C

// calibration data structure for temperature compensation
struct bme680_calib {
    uint16_t T1;     
    int16_t  T2;     
    int8_t   T3;      
    int32_t  t_fine;  
};
/* Shared sensor data between BME680 thread and I2C target thread */
struct sensor_reading {
    struct k_mutex sensor_mtx;
    uint32_t heart_rate;
    uint32_t blood_oxygen;
    uint32_t temperature;    
};

#define SENS_HR_OFFSET    offsetof(struct sensor_reading, heart_rate)
#define SENS_BO_OFFSET    offsetof(struct sensor_reading, blood_oxygen)
#define SENS_TEMP_OFFSET  offsetof(struct sensor_reading, temperature)
#define SENS_BUFFER_SIZE  sizeof(uint32_t)

static struct sensor_reading sens_reading;
static struct bme680_calib   calib;
static uint8_t               sensor_mode;
static atomic_t              buffer_index;

int bme680_read_calib(const struct device *i2c_dev, struct bme680_calib *calib)
{
    uint8_t buf[2];
    uint8_t reg;

    /* T1 — unsigned 16-bit, LSB at 0xE9, MSB at 0xEA */
    reg = BME680_REG_T1_LSB;
    if (i2c_write_read(i2c_dev, BME680_ADDR, &reg, 1, buf, 2) != 0) {
        printk("ERROR: Failed to read T1\n");
        return -1;
    }
    calib->T1 = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));

    /* T2 — signed 16-bit, LSB at 0x8A, MSB at 0x8B */
    reg = BME680_REG_T2_LSB;
    if (i2c_write_read(i2c_dev, BME680_ADDR, &reg, 1, buf, 2) != 0) {
        printk("ERROR: Failed to read T2\n");
        return -1;
    }
    calib->T2 = (int16_t)(buf[0] | ((uint16_t)buf[1] << 8));

    /* T3 — signed 8-bit at 0x8C */
    reg = BME680_REG_T3;
    if (i2c_write_read(i2c_dev, BME680_ADDR, &reg, 1, buf, 1) != 0) {
        printk("ERROR: Failed to read T3\n");
        return -1;
    }
    calib->T3 = (int8_t)buf[0];

    printk("Calibration: T1=%u T2=%d T3=%d\n", calib->T1, calib->T2, calib->T3);
    return 0;
}

int32_t bme680_compensate_temp(struct bme680_calib *calib, uint32_t adc_temp)
{
    int32_t var1, var2, var3;

    var1 = ((int32_t)adc_temp >> 3) - ((int32_t)calib->T1 << 1);
    var2 = (var1 * (int32_t)calib->T2) >> 11;
    var3 = (((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t)calib->T3 << 4);
    var3 = var3 >> 14;

    calib->t_fine = var2 + var3;

    return ((calib->t_fine * 5) + 128) >> 8;
}

int bme680_init(const struct device *i2c_dev)
{
    uint8_t chip_id;
    uint8_t reg;
    uint8_t cmd[2];

    /*  — check chip ID */
    reg = BME680_REG_CHIP_ID;
    if (i2c_write_read(i2c_dev, BME680_ADDR, &reg, 1, &chip_id, 1) != 0) {
        printk("ERROR: Cannot reach BME680 on I2C\n");
        return -1;
    }
    if (chip_id != BME680_CHIP_ID_VAL) {
        printk("ERROR: Wrong chip ID 0x%02x, expected 0x61\n", chip_id);
        return -1;
    }
    printk("BME680 found. Chip ID: 0x%02x\n", chip_id);

    /*  — soft reset */
    cmd[0] = BME680_REG_RESET;
    cmd[1] = BME680_RESET_VAL;
    if (i2c_write(i2c_dev, cmd, 2, BME680_ADDR) != 0) {
        printk("ERROR: Failed to reset BME680\n");
        return -1;
    }
    k_msleep(10);

    /*  — read calibration data */
    if (bme680_read_calib(i2c_dev, &calib) != 0) {
        printk("ERROR: Failed to read calibration\n");
        return -1;
    }

    /*  — set humidity oversampling to 1x */
    cmd[0] = BME680_REG_CTRL_HUM;
    cmd[1] = 0x01;
    if (i2c_write(i2c_dev, cmd, 2, BME680_ADDR) != 0) {
        printk("ERROR: Failed to set humidity oversampling\n");
        return -1;
    }

    printk("BME680 initialized successfully\n");
    return 0;
}

int bme680_read_temp(const struct device *i2c_dev, int32_t *temp_out)
{
    uint8_t cmd[2];
    uint8_t reg;
    uint8_t status;
    uint8_t raw[3];
    uint32_t adc_temp;

    /*  — trigger a forced mode measurement */
    cmd[0] = BME680_REG_CTRL_MEAS;
    cmd[1] = 0x41;
    if (i2c_write(i2c_dev, cmd, 2, BME680_ADDR) != 0) {
        printk("ERROR: Failed to trigger measurement\n");
        return -1;
    }

    /*  — wait for measurement to finish */
    do {
        k_msleep(5);
        reg = BME680_REG_STATUS;
        if (i2c_write_read(i2c_dev, BME680_ADDR, &reg, 1, &status, 1) != 0) {
            printk("ERROR: Failed to read status\n");
            return -1;
        }
    } while (status & BME680_STATUS_MEASURING);

    /*  — read raw temperature registers */
    reg = BME680_REG_TEMP_MSB;
    if (i2c_write_read(i2c_dev, BME680_ADDR, &reg, 1, raw, 3) != 0) {
        printk("ERROR: Failed to read temperature\n");
        return -1;
    }

    /*  — combine into 20-bit ADC value */
    adc_temp = ((uint32_t)raw[0] << 12) |
               ((uint32_t)raw[1] << 4)  |
               ((uint32_t)raw[2] >> 4);

    /*  apply compensation formula */
    *temp_out = bme680_compensate_temp(&calib, adc_temp);

    return 0;
}

void bme680_thread_fn(void *p1, void *p2, void *p3)
{
    const struct device *i2c_dev  = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    uint32_t dtr = 0;

    /* Wait for USB terminal to connect */
    while (!dtr) {
        uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }

    /* Initialize mutex */
    k_mutex_init(&sens_reading.sensor_mtx);

    /* Check i2c0 is ready */
    if (!device_is_ready(i2c_dev)) {
        printk("ERROR: i2c0 not ready\n");
        return;
    }

    /* Initialize BME680 */
    if (bme680_init(i2c_dev) != 0) {
        printk("ERROR: BME680 init failed\n");
        return;
    }

    /* Main reading loop */
    while (1) {
        int32_t temp;

        if (bme680_read_temp(i2c_dev, &temp) == 0) {
            k_mutex_lock(&sens_reading.sensor_mtx, K_FOREVER);
            sens_reading.temperature = (uint32_t)temp;
            k_mutex_unlock(&sens_reading.sensor_mtx);

            printk("Temperature: %d.%02d C\n", temp / 100, temp % 100);
        } else {
            printk("ERROR: Failed to read temperature\n");
        }

        k_msleep(5000);
    }
}

// i2c target callbacks for sensor node
int sensor_node_write_received_cb(struct i2c_target_config *cfg, uint8_t val)
{
    switch (val) {
    case 0x01:
        sensor_mode = SENS_HR_OFFSET;
        break;
    case 0x02:
        sensor_mode = SENS_BO_OFFSET;
        break;
    case 0x03:
        sensor_mode = SENS_TEMP_OFFSET;
        break;
    case 0xFF:
        sensor_mode = 0xFF;
        break;
    default:
        sensor_mode = 0;
        break;
}
   return 0;
}

int sensor_node_read_requested_cb(struct i2c_target_config *cfg, uint8_t *val)
{
    if (sensor_mode == 0xFF) {
        *val = 0xEE;
        return 0;
    }

    buffer_index = ATOMIC_INIT(0);
    uint8_t *data = (uint8_t *)&sens_reading;
    *val = data[sensor_mode];

    return 0;
}

int sensor_node_read_processed_cb(struct i2c_target_config *cfg, uint8_t *val)
{
    atomic_inc(&buffer_index);

    if (buffer_index >= SENS_BUFFER_SIZE) {
        *val = 0xFF;
    } else {
        uint8_t *data = (uint8_t *)&sens_reading;
        *val = data[sensor_mode + buffer_index];
    }

    return 0;
}

int sensor_node_stop_cb(struct i2c_target_config *cfg)
{
    buffer_index = ATOMIC_INIT(0);
    return 0;
}

static struct i2c_target_callbacks sensor_node_callbacks = {
    .write_received  = sensor_node_write_received_cb,
    .read_requested  = sensor_node_read_requested_cb,
    .read_processed  = sensor_node_read_processed_cb,
    .stop            = sensor_node_stop_cb,
};

// sensor node I2C1 target thread
void i2c_target_thread_fn(void *p1, void *p2, void *p3)
{
    int err;
    const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

    struct i2c_target_config i2c_cfg = {
        .address   = 0x60,
        .callbacks = &sensor_node_callbacks,
    };

    if (!device_is_ready(i2c_dev)) {
        printk("ERROR: i2c1 not ready\n");
        return;
    }

    err = i2c_target_register(i2c_dev, &i2c_cfg);
    if (err < 0) {
        printk("ERROR: Failed to register I2C target (%d)\n", err);
        return;
    }

    printk("I2C target registered at address 0x62\n");
}

// Thread definitions
#define BME680_STACK_SIZE  1024
#define TARGET_STACK_SIZE  512
#define SENSOR_THREAD_PRIO 5
#define TARGET_THREAD_PRIO 5

K_THREAD_DEFINE(bme680_thread,
                BME680_STACK_SIZE,
                bme680_thread_fn,
                NULL, NULL, NULL,
                SENSOR_THREAD_PRIO, 0, 0);

K_THREAD_DEFINE(i2c_target_thread,
                TARGET_STACK_SIZE,
                i2c_target_thread_fn,
                NULL, NULL, NULL,
                TARGET_THREAD_PRIO, 0, 0);