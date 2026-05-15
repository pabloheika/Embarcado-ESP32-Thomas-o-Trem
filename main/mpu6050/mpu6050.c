#include "mpu6050.h"

#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "mpu6050";

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_GPIO 21
#define I2C_SCL_GPIO 22
#define I2C_FREQ_HZ 400000
#define I2C_TIMEOUT_MS 1000

#define MPU6050_ADDR_7BIT 0x68
#define MPU6050_REG_WHO_AM_I 0x75
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_WHO_AM_I_EXPECT 0x68

#define MPU6050_READ_STACK_WORDS 3072
#define MPU6050_READ_TASK_PRIO 5
#define MPU6050_LOG_EVERY_N_SAMPLES 25

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static SemaphoreHandle_t s_sample_mutex;
static mpu6050_sample_t s_last_sample;
static TaskHandle_t s_read_task;
static uint32_t s_read_period_ms;

static esp_err_t mpu6050_register_read(i2c_master_dev_handle_t dev, uint8_t reg_addr, uint8_t *data,
                                       size_t len)
{
    return i2c_master_transmit_receive(dev, &reg_addr, 1, data, len, I2C_TIMEOUT_MS);
}

static esp_err_t mpu6050_register_write_byte(i2c_master_dev_handle_t dev, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev, write_buf, sizeof(write_buf), I2C_TIMEOUT_MS);
}

static void mpu6050_parse_burst(const uint8_t *buf, mpu6050_sample_t *s)
{
    s->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    s->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    s->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    s->temp = (int16_t)((buf[6] << 8) | buf[7]);
    s->gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
    s->gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
    s->gyro_z = (int16_t)((buf[12] << 8) | buf[13]);
}

static void mpu6050_read_task(void *arg)
{
    (void)arg;
    uint8_t buf[14];
    uint32_t n = 0;

    for (;;) {
        esp_err_t err = mpu6050_register_read(s_dev, MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "falha na leitura I2C: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(s_read_period_ms ? s_read_period_ms : 20));
            continue;
        }

        mpu6050_sample_t sample;
        mpu6050_parse_burst(buf, &sample);

        if (xSemaphoreTake(s_sample_mutex, portMAX_DELAY) == pdTRUE) {
            s_last_sample = sample;
            xSemaphoreGive(s_sample_mutex);
        }

        n++;
        if (n % MPU6050_LOG_EVERY_N_SAMPLES == 0) {
            ESP_LOGI(TAG,
                     "accel [%d,%d,%d] gyro [%d,%d,%d] temp_raw %d",
                     (int)sample.accel_x,
                     (int)sample.accel_y,
                     (int)sample.accel_z,
                     (int)sample.gyro_x,
                     (int)sample.gyro_y,
                     (int)sample.gyro_z,
                     (int)sample.temp);
        }

        vTaskDelay(pdMS_TO_TICKS(s_read_period_ms ? s_read_period_ms : 20));
    }
}

esp_err_t mpu6050_init(void)
{
    if (s_bus != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_sample_mutex = xSemaphoreCreateMutex();
    if (s_sample_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        vSemaphoreDelete(s_sample_mutex);
        s_sample_mutex = NULL;
        ESP_LOGE(TAG, "i2c_new_master_bus: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR_7BIT,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
        vSemaphoreDelete(s_sample_mutex);
        s_sample_mutex = NULL;
        ESP_LOGE(TAG, "i2c_master_bus_add_device: %s", esp_err_to_name(err));
        return err;
    }

    err = mpu6050_register_write_byte(s_dev, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wake: %s", esp_err_to_name(err));
        goto fail_dev;
    }

    uint8_t who = 0;
    err = mpu6050_register_read(s_dev, MPU6050_REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "who_am_i: %s", esp_err_to_name(err));
        goto fail_dev;
    }
    if (who != MPU6050_WHO_AM_I_EXPECT) {
        ESP_LOGE(TAG, "WHO_AM_I inesperado: 0x%02x (esperado 0x%02x)", who, MPU6050_WHO_AM_I_EXPECT);
        err = ESP_ERR_INVALID_RESPONSE;
        goto fail_dev;
    }

    memset(&s_last_sample, 0, sizeof(s_last_sample));
    ESP_LOGI(TAG, "MPU6050 ok (I2C SDA=%d SCL=%d)", I2C_SDA_GPIO, I2C_SCL_GPIO);
    return ESP_OK;

fail_dev:
    i2c_master_bus_rm_device(s_dev);
    s_dev = NULL;
    i2c_del_master_bus(s_bus);
    s_bus = NULL;
    vSemaphoreDelete(s_sample_mutex);
    s_sample_mutex = NULL;
    return err;
}

esp_err_t mpu6050_start_read_task(uint32_t read_period_ms)
{
    if (s_bus == NULL || s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_read_task != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_read_period_ms = read_period_ms ? read_period_ms : 20;

    BaseType_t ok = xTaskCreate(mpu6050_read_task, "mpu6050_read", MPU6050_READ_STACK_WORDS, NULL,
                                MPU6050_READ_TASK_PRIO, &s_read_task);
    if (ok != pdPASS) {
        s_read_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "tarefa de leitura iniciada (%lu ms)", (unsigned long)s_read_period_ms);
    return ESP_OK;
}

esp_err_t mpu6050_get_last_sample(mpu6050_sample_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_sample_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(s_sample_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    *out = s_last_sample;
    xSemaphoreGive(s_sample_mutex);
    return ESP_OK;
}
