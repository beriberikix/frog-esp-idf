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
#include "dps310.h"

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
    ESP_LOGI(TAG, "Power Mode (lc709203f): 0x%X", value);
    ESP_ERROR_CHECK(lc709203f_get_apa(lc, (uint8_t *)&value));
    ESP_LOGI(TAG, "APA (lc709203f): 0x%X", value);
    ESP_ERROR_CHECK(lc709203f_get_battery_profile(lc, (lc709203f_battery_profile_t *)&value));
    ESP_LOGI(TAG, "Battery Profile (lc709203f): 0x%X", value);
    ESP_ERROR_CHECK(lc709203f_get_temp_mode(lc, (lc709203f_temp_mode_t *)&value));
    ESP_LOGI(TAG, "Temp Mode (lc709203f): 0x%X", value);

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

static esp_err_t initialize_dps310(dps310_t *dps, dps310_config_t *config)
{
    bool sensor_ready = false;
    bool coef_ready = false;
    esp_err_t err = ESP_FAIL;

    /* Initialize the device. */
    ESP_LOGI(TAG, "Initializing the device");
    err = dps310_init(dps, config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_init(): %s", esp_err_to_name(err));
    }

    /* ensure the sensor is ready and coefficients, or COEF, are also ready.
     * */
    ESP_LOGI(TAG, "Waiting for the sensor to be ready for measurement");
    do
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (!sensor_ready)
        {
            err = dps310_is_ready_for_sensor(dps, &sensor_ready);
            if (err != ESP_OK)
            {
                // goto fail;
            }
        }

        if (!coef_ready)
        {
            err = dps310_is_ready_for_coef(dps, &coef_ready);
            if (err != ESP_OK)
            {
                // goto fail;
            }
        }
    } while (!sensor_ready || !coef_ready);

    /* read COEF once, which is used to compensate the raw value. The COEF
     * values are kept in the device descriptor.
     */
    err = dps310_get_coef(dps);
    if (err != ESP_OK)
    {
        // goto fail;
    }

    return ESP_OK;
}

void task(void *pvParameters)
{
    i2c_dev_t lc;
    i2c_dev_t scd;
    dps310_t dps;

    esp_err_t err = ESP_FAIL;

    // lc709203f
    uint16_t voltage = 0, rsoc = 0, ite = 0;
    float bat_temp = -274;

    // scd30
    float co2, temperature, humidity;
    bool data_ready;

    // dps210
    float temperature_dps = 0, pressure_dps = 0;
    bool temperature_ready = false, pressure_ready = false;

    memset(&lc, 0, sizeof(lc));
    ESP_ERROR_CHECK(lc709203f_init_desc(&lc, 0, CONFIG_FROG_I2C_MASTER_SDA, CONFIG_FROG_I2C_MASTER_SCL));
    initialize_lc709203f(&lc);

    memset(&scd, 0, sizeof(scd));
    ESP_ERROR_CHECK(scd30_init_desc(&scd, 0, CONFIG_FROG_I2C_MASTER_SDA, CONFIG_FROG_I2C_MASTER_SCL));
    initialize_scd30(&scd);

    dps310_config_t config = DPS310_CONFIG_DEFAULT();
    memset(&dps, 0, sizeof(dps310_t));
    config.tmp_oversampling = DPS310_TMP_PRC_128;
    config.pm_oversampling = DPS310_PM_PRC_128;
    err = dps310_init_desc(&dps, 0x77, 0, CONFIG_FROG_I2C_MASTER_SDA, CONFIG_FROG_I2C_MASTER_SCL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "dps310_init_desc(): %s", esp_err_to_name(err));
        // goto fail;
    }
    initialize_dps310(&dps, &config);

    while (1)
    {
        ESP_ERROR_CHECK(lc709203f_get_cell_voltage(&lc, &voltage));
        ESP_ERROR_CHECK(lc709203f_get_rsoc(&lc, &rsoc));
        ESP_ERROR_CHECK(lc709203f_get_cell_ite(&lc, &ite));
        // Temperature in I2C mode. Temperature should be the same as configured.
        ESP_ERROR_CHECK(lc709203f_get_cell_temperature_celsius(&lc, &bat_temp));
        // ESP_LOGI("Battery (lc709203f)", "Temp  (lc709203f): %.1f\tVoltage (lc709203f): %.2f\tRSOC (lc709203f): %d%%\tITE (lc709203f): %.1f%%", bat_temp, voltage / 1000.0, rsoc,
        //          ite / 10.0);
        ESP_LOGI(TAG, "Temp (lc709203f): %.1f", bat_temp);
        ESP_LOGI(TAG, "Voltage (lc709203f): %.2f", voltage / 1000.0);
        ESP_LOGI(TAG, "RSOC (lc709203f): %d%%", rsoc);
        ESP_LOGI(TAG, "ITE (lc709203f): %.1f%%", ite / 10.0);

        scd30_get_data_ready_status(&scd, &data_ready);
        if (data_ready)
        {
            esp_err_t res = scd30_read_measurement(&scd, &co2, &temperature, &humidity);
            if (res != ESP_OK)
            {
                ESP_LOGE(TAG, "Error reading results %d (%s)", res, esp_err_to_name(res));
                continue;
            }

            if (co2 == 0)
            {
                ESP_LOGW(TAG, "Invalid sample detected, skipping");
                continue;
            }

            ESP_LOGI(TAG, "CO2 (scd30): %.0f ppm", co2);
            ESP_LOGI(TAG, "Temperature (scd30): %.2f °C", temperature);
            ESP_LOGI(TAG, "Humidity (scd30): %.2f %%", humidity);
        }

        // dps310
        /* Temperature command mode.
         *
         * The sensor measures temperature once, stores the raw value in a
         * resister, and sets TMP_RDY bit. The sensor goes back to standby mode
         * after measurement.
         */
        // ESP_LOGI(TAG, "Setting manual temperature measurement mode");
        err = dps310_set_mode(&dps, DPS310_MODE_COMMAND_TEMPERATURE);
        if (err != ESP_OK)
        {
            // goto fail;
        }

        /* wait for the result by polling TMP_RDY bit */
        // ESP_LOGI(TAG, "Waiting for the temperature value to be ready");
        do
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            err = dps310_is_ready_for_temp(&dps, &temperature_ready);
            if (err != ESP_OK)
            {
                // goto fail;
            }
        } while (!temperature_ready);

        /* Read the result of temperature measurement */
        err = dps310_read_temp(&dps, &temperature_dps);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "dps310_read_temp(): %s", esp_err_to_name(err));
            // goto fail;
        }
        ESP_LOGI(TAG, "Temperature (dps310): %0.2f °C", temperature_dps);

        /* Pressure command mode
         *
         * The sensor measures pressure once, stores the raw value in a resister,
         * and sets PRS_RDY bit. The sensor goes back to standby mode after
         * measurement.
         */
        // ESP_LOGI(TAG, "Setting manual pressure measurement mode");
        err = dps310_set_mode(&dps, DPS310_MODE_COMMAND_PRESSURE);
        if (err != ESP_OK)
        {
            // goto fail;
        }

        /* wait for the result by polling PRS_RDY bit */
        // ESP_LOGI(TAG, "Waiting for the pressure value to be ready");
        do
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            err = dps310_is_ready_for_pressure(&dps, &pressure_ready);
            if (err != ESP_OK)
            {
                // goto fail;
            }
        } while (!pressure_ready);

        /* Read the result of pressure measurement, and compensate the result with
         * temperature and COEF.
         *
         * Note that dps310_read_pressure() reads temperature *and* pressure
         * values for compensation, including oversampling rates.
         *
         * This implies:
         *
         * * temperature measurement must be done brefore dps310_read_pressure()
         *   at least once.
         * * the function is slow.
         */
        err = dps310_read_pressure(&dps, &pressure_dps);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "dps310_read_pressure(): %s", esp_err_to_name(err));
            // goto fail;
        }
        ESP_LOGI(TAG, "Pressure (dps310): %0.2f Pa", pressure_dps);
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelay(pdMS_TO_TICKS(10000));
}

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