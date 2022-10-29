#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_err.h>
#include <esp_log.h>

#if CONFIG_FROG_I2C_ONBOARD_PULLUP_GPIO_OUTPUT > -1
#include <driver/gpio.h>
#endif

#include "lc709203f.h"
#include "scd30.h"

static const char *TAG = "Frog";

static esp_err_t initialize_lc709203f(i2c_dev_t *lc)
{
    if (!lc)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK(lc709203f_set_power_mode(lc, LC709203F_POWER_MODE_OPERATIONAL));
    // Using 2500mAh LiPo battery. Check Datasheet graph for APA values by battery type & mAh
    ESP_ERROR_CHECK(lc709203f_set_apa(lc, 0x32));
    ESP_ERROR_CHECK(lc709203f_set_battery_profile(lc, LC709203F_BATTERY_PROFILE_1));
    ESP_ERROR_CHECK(lc709203f_initial_rsoc(lc));
    ESP_ERROR_CHECK(lc709203f_set_temp_mode(lc, LC709203F_TEMP_MODE_I2C));
    ESP_ERROR_CHECK(lc709203f_set_cell_temperature_celsius(lc, 20));

    uint16_t value = 0;
    ESP_ERROR_CHECK(lc709203f_get_power_mode(lc, (lc709203f_power_mode_t *)&value));
    ESP_LOGI(TAG, "Power Mode: 0x%X", value);
    ESP_ERROR_CHECK(lc709203f_get_apa(lc, (uint8_t *)&value));
    ESP_LOGI(TAG, "APA: 0x%X", value);
    ESP_ERROR_CHECK(lc709203f_get_battery_profile(lc, (lc709203f_battery_profile_t *)&value));
    ESP_LOGI(TAG, "Battery Profile: 0x%X", value);
    ESP_ERROR_CHECK(lc709203f_get_temp_mode(lc, (lc709203f_temp_mode_t *)&value));
    ESP_LOGI(TAG, "Temp Mode: 0x%X", value);

    return ESP_OK;
}

static esp_err_t initialize_scd30(i2c_dev_t *scd)
{
    if (!scd)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t version, major_ver, minor_ver;
    ESP_ERROR_CHECK(scd30_read_firmware_version(scd, &version));

    major_ver = (version >> 8) & 0xf;
    minor_ver = version & 0xf;

    ESP_LOGI(TAG, "SCD30 Firmware Version: %d.%d", major_ver, minor_ver);

    ESP_LOGI(TAG, "Starting continuous measurement");
    ESP_ERROR_CHECK(scd30_trigger_continuous_measurement(scd, 0));

    return ESP_OK;
}

void task(void *pvParameters)
{
    i2c_dev_t lc;
    i2c_dev_t scd;

    // lc709203f
    uint16_t voltage = 0, rsoc = 0, ite = 0;
    float bat_temp = -274;

    // scd30
    float co2, temperature, humidity;
    bool data_ready;

    memset(&lc, 0, sizeof(lc));

    ESP_ERROR_CHECK(lc709203f_init_desc(&lc, 0, CONFIG_FROG_I2C_MASTER_SDA, CONFIG_FROG_I2C_MASTER_SCL));
    initialize_lc709203f(&lc);

    memset(&scd, 0, sizeof(scd));
    ESP_ERROR_CHECK(scd30_init_desc(&scd, 0, CONFIG_FROG_I2C_MASTER_SDA, CONFIG_FROG_I2C_MASTER_SCL));
    initialize_scd30(&scd);

    while (1)
    {
        ESP_ERROR_CHECK(lc709203f_get_cell_voltage(&lc, &voltage));
        ESP_ERROR_CHECK(lc709203f_get_rsoc(&lc, &rsoc));
        ESP_ERROR_CHECK(lc709203f_get_cell_ite(&lc, &ite));
        // Temperature in I2C mode. Temperature should be the same as configured.
        ESP_ERROR_CHECK(lc709203f_get_cell_temperature_celsius(&lc, &bat_temp));
        ESP_LOGI("Battery", "Temp: %.1f\tVoltage: %.2f\tRSOC: %d%%\tITE: %.1f%%", bat_temp, voltage / 1000.0, rsoc,
            ite / 10.0);

        scd30_get_data_ready_status(&scd, &data_ready);
        if (data_ready)
        {
            esp_err_t res = scd30_read_measurement(&scd, &co2, &temperature, &humidity);
            if (res != ESP_OK)
            {
                ESP_LOGE(TAG, "Error reading results %d (%s)", res, esp_err_to_name(res));
                // continue;
            }

            if (co2 == 0)
            {
                ESP_LOGW(TAG, "Invalid sample detected, skipping");
                // continue;
            }

            ESP_LOGI(TAG, "CO2: %.0f ppm", co2);
            ESP_LOGI(TAG, "Temperature: %.2f °C", temperature);
            ESP_LOGI(TAG, "Humidity: %.2f %%", humidity);
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// void scd30_test(void *pvParameters)
// {
//     i2c_dev_t dev = {0};

//     ESP_ERROR_CHECK(scd30_init_desc(&dev, 0, CONFIG_FROG_I2C_MASTER_SDA, CONFIG_FROG_I2C_MASTER_SCL));

//     uint16_t version, major_ver, minor_ver;
//     ESP_ERROR_CHECK(scd30_read_firmware_version(&dev, &version));

//     major_ver = (version >> 8) & 0xf;
//     minor_ver = version & 0xf;

//     ESP_LOGI(TAG, "SCD30 Firmware Version: %d.%d", major_ver, minor_ver);

//     ESP_LOGI(TAG, "Starting continuous measurement");
//     ESP_ERROR_CHECK(scd30_trigger_continuous_measurement(&dev, 0));

//     float co2, temperature, humidity;
//     bool data_ready;
//     // while (1)
//     // {
//     //     vTaskDelay(pdMS_TO_TICKS(2000));

//         scd30_get_data_ready_status(&dev, &data_ready);

//         if (data_ready)
//         {
//             esp_err_t res = scd30_read_measurement(&dev, &co2, &temperature, &humidity);
//             if (res != ESP_OK)
//             {
//                 ESP_LOGE(TAG, "Error reading results %d (%s)", res, esp_err_to_name(res));
//                 // continue;
//             }

//             if (co2 == 0)
//             {
//                 ESP_LOGW(TAG, "Invalid sample detected, skipping");
//                 // continue;
//             }

//             ESP_LOGI(TAG, "CO2: %.0f ppm", co2);
//             ESP_LOGI(TAG, "Temperature: %.2f °C", temperature);
//             ESP_LOGI(TAG, "Humidity: %.2f %%", humidity);
//         }
//     // }
// }

void app_main(void)
{
#if CONFIG_FROG_I2C_ONBOARD_PULLUP_GPIO_OUTPUT > -1
    /// Adafruit Feather esp32ss/s3 needs to set GPIO7 as HIGH level output to enable onboard I2C pull ups
    /// We needn't internal pull ups.
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1 << CONFIG_FROG_I2C_ONBOARD_PULLUP_GPIO_OUTPUT);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(CONFIG_FROG_I2C_ONBOARD_PULLUP_GPIO_OUTPUT, CONFIG_FROG_I2C_ONBOARD_PULLUP_GPIO_OUTPUT_LEVEL);
#endif

    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(task, "sense", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
    // xTaskCreate(scd30_test, "scd30_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}