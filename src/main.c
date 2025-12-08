#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_err.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "minimal_wifi.h"
#include "driver/gpio.h"
#include "driver/adc.h"
// Configure the WiFi network settings here

#define WIFI_SSID      "Tufts_Wireless"
#define WIFI_PASS      ""

static const char *TAG = "wifi demo";


#define I2C_PORT I2C_NUM_0
#define I2C_FREQ_HZ 100000
#define STS31_ADDR 0x4A
#define CMD_MSB 0x24
#define CMD_LSB 0x16
#define MEAS_DELAY_MS 25

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t dev;


#define ADC_VREF            3.3f
#define ADC_SAMPLES         32

#define SDA_PIN 7
#define SCL_PIN 6
#define LED_GPIO 2
#define NTC_ADC_CH          ADC_CHANNEL_0

static void init_adc(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(NTC_ADC_CH, ADC_ATTEN_DB_11); // up to ~3.3V
}

float readvoltage(void) {
    int acc = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        acc += adc1_get_raw(NTC_ADC_CH);
    }
    float raw = (float)acc / (float)ADC_SAMPLES;
    float v = (raw / 4095.0f) * ADC_VREF;
    return v;
}

void i2cinit()
{
    // I2C init
    i2c_master_bus_config_t b = {
        .i2c_port = I2C_PORT,
        .scl_io_num = SCL_PIN,
        .sda_io_num = SDA_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags = {.enable_internal_pullup = 1},
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&b, &bus));
    i2c_device_config_t d = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = STS31_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &d, &dev));
}

// static uint8_t crc8_sens(const uint8_t *d, size_t n)
// {
//     uint8_t c = 0xFF;
//     for (size_t i = 0; i < n; i++)
//     {
//         c ^= d[i];
//         for (int b = 0; b < 8; b++)
//             c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x31) : (uint8_t)(c << 1);
//     }
//     return c;
// }

static float get_temp_ic(void)
{
    uint8_t cmd[2] = {CMD_MSB, CMD_LSB};
    uint8_t rx[3] = {0};

    // robust: one-call TxRx with retry once
    esp_err_t e = i2c_master_transmit_receive(dev, cmd, 2, rx, 3, 300);
    if (e != ESP_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(MEAS_DELAY_MS));
        e = i2c_master_transmit_receive(dev, cmd, 2, rx, 3, 300);
        if (e != ESP_OK)
            return -999.9;
    }
    else
    {
        vTaskDelay(pdMS_TO_TICKS(MEAS_DELAY_MS));
        if (i2c_master_receive(dev, rx, 3, 300) != ESP_OK)
            return -99.9; // some controllers prefer explicit read
    }

    uint16_t raw = ((uint16_t)rx[0] << 8) | rx[1];
    return -45.0f + 175.0f * ((float)raw / 65535.0f);
}
void nvsinit(){
    ESP_LOGI(TAG, "STARTING APPLICATION");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void wifiinit(){
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_connect(WIFI_SSID, WIFI_PASS);
}
void ledinit(){
        // Configure pin as output
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

}
void turnonled(void)
{

    // Set HIGH
    gpio_set_level(LED_GPIO, 1);
    printf("GPIO %d HIGH\n", LED_GPIO);

    // Wait 1 second
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // // Set LOW
    // gpio_set_level(LED_GPIO, 0);
    // printf("GPIO %d LOW\n", LED_GPIO);
}

void app_main(void)
{
    nvsinit();
    init_adc();
    ledinit();
    i2cinit();
    wifiinit();
    while(1) {
        float t_ic = get_temp_ic();
        printf("temperature is: %6.2f °C", t_ic);
        float batt_vol = readvoltage();
        printf("Battery percentage is is: %6.2f °C", batt_vol);
        turnonled();
        vTaskDelay(1000);
    }

}