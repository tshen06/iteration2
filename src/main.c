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
#include "mqtt_client.h"
#include "esp_sleep.h"
#include <math.h>  
#include "esp_sntp.h"
#include <time.h>


// Configure the WiFi network settings here


#define I2C_PORT I2C_NUM_0
#define I2C_FREQ_HZ 100000
#define TMP1075_ADDR  0x48
#define CMD_MSB 0x24
#define CMD_LSB 0x16
#define MEAS_DELAY_MS 25

#define ADC_VREF            3.3f
#define ADC_SAMPLES         32

#define SDA_PIN 5
#define SCL_PIN 4
#define LED_GPIO 1
#define NTC_ADC_CH          ADC_CHANNEL_3


#define WIFI_SSID      "Tufts_Wireless"
#define WIFI_PASS      ""
#define BROKER_URI "mqtt://bell-mqtt.eecs.tufts.edu"


static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t dev;

static const char *TAG = "Iteration 2, Group 1";

esp_mqtt_client_handle_t client;


static void init_adc(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(NTC_ADC_CH, ADC_ATTEN_DB_11); // up to ~3.3V
}

static void mqttinit(){
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = BROKER_URI,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}

//to do
static void publishmqtt(float temp, float battery, float rrsi){
    //esp_mqtt_client_publish(client, "tshen06/iteration1/thermistor_temp", t_ntc_buffer, 0, 0, 0);
}

#define R1 330000.0f   // top resistor (Vbat -> ADC)
#define R2 110000.0f   // bottom resistor (ADC -> GND)


float readvoltage(void) {
    int acc = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        acc += adc1_get_raw(NTC_ADC_CH);   // ADC_CHANNEL_3 for GPIO3
    }

    float raw = (float)acc / (float)ADC_SAMPLES;

    // ADC voltage at GPIO3
    float v_adc = (raw / 4095.0f) * ADC_VREF;   // ADC_VREF = 3.3f

    // Battery voltage (undo the divider)
    float v_bat = v_adc * ((R1 + R2) / R2);     // here this is v_adc * 4.0f

    return v_bat;
}


void i2cinit(void)
{
    i2c_master_bus_config_t b = {
        .i2c_port = I2C_PORT,
        .scl_io_num = SCL_PIN,
        .sda_io_num = SDA_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags = {.enable_internal_pullup = 1},   // still OK, but add externals on PCB
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&b, &bus));

    i2c_device_config_t d = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TMP1075_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &d, &dev));
}




static float get_temp_ic(void)
{
    // 1) Pointer to temperature register (0x00)
    uint8_t ptr = 0x00;
    uint8_t rx[2] = {0};

    // Combined write-then-read: [ADDR+W][0x00] + repeated START + [ADDR+R][2 bytes]
    esp_err_t e = i2c_master_transmit_receive(dev, &ptr, 1, rx, 2, 300);
    if (e != ESP_OK) {
        ESP_LOGE("TMP1075", "I2C read failed: %s", esp_err_to_name(e));
        return NAN;   // or -999.9f
    }

    // 2) Convert to 12-bit two's complement
    int16_t raw = ((int16_t)rx[0] << 8) | rx[1];
    raw >>= 4;  // top 12 bits are temperature

    // 3) Each LSB = 0.0625°C
    float temp_c = (float)raw * 0.0625f;
    return temp_c;
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

static void init_sntp(void)
{
    ESP_LOGI("NTP", "Initializing SNTP");

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");  // You can add more servers if needed
    sntp_init();
}

void ledinit(){
    // Configure pin as output
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1);
}

void ledflash(void)
{
    for (int i=0; i<5; i++){
        gpio_set_level(LED_GPIO, 0);
        printf("GPIO %d HIGH\n", LED_GPIO);
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

uint64_t get_epoch_time(void)
{
    // Initialize SNTP only once
    static bool sntp_initialized = false;
    if (!sntp_initialized) {
        init_sntp();
        sntp_initialized = true;
    }

    // Wait for time to be set
    int retry = 0;
    const int retry_count = 15;

    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && retry < retry_count) {
        ESP_LOGI("NTP", "Waiting for system time... (%d/%d)", retry+1, retry_count);
        vTaskDelay(pdMS_TO_TICKS(1000));
        retry++;
    }

    // Get current system time
    time_t now;
    time(&now);

    if (now < 100000) {
        ESP_LOGE("NTP", "Failed to get time from NTP!");
        return 0;  // 0 = failed
    }

    ESP_LOGI("NTP", "Epoch time: %lld", (long long)now);
    return (uint64_t)now;
}


void enterDeepSleep(){
    ESP_LOGI("SLEEP", "Device is going to sleep for 10 second...");

    // 1 hour = 3600 seconds
    const int64_t sleep_time_us = 10LL * 1000000LL;

    // Configure wakeup source: timer
    esp_sleep_enable_timer_wakeup(sleep_time_us);

    // Optional: flush logs
    vTaskDelay(pdMS_TO_TICKS(100));

    // Enter deep sleep
    esp_deep_sleep_start();
}


void mqttsend(char* topic, char* data){
    esp_mqtt_client_publish(client, topic, data, 0, 1, 0);
}

void app_main(void)
{
    nvsinit();
    init_adc();
    ledinit();
    i2cinit();
    wifiinit();
    init_sntp();
    uint64_t epoch = get_epoch_time();
    printf("Epoch = %llu\n", (unsigned long long)epoch);
    mqttinit();
    float t_ic = get_temp_ic();
    printf("temperature is: %6.2f °C", t_ic);
    float batt_vol = readvoltage();
    printf("Battery percentage is is: %6.2f", batt_vol);

    
    ledflash();
    enterDeepSleep();
}