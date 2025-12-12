#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_err.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"
#include "minimal_wifi.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "mqtt_client.h"
#include "esp_sleep.h"
#include <math.h>
#include "esp_sntp.h"
#include <time.h>
#include "esp_mac.h"
#include <string.h>

// --- 1. CONFIGURATION ---
#define I2C_PORT I2C_NUM_0
#define I2C_FREQ_HZ 100000
#define TMP1075_ADDR 0x48
#define MEAS_DELAY_MS 25

#define ADC_VREF 3.3f
#define ADC_SAMPLES 32
#define SDA_PIN 5
#define SCL_PIN 4
#define LED_GPIO 1 // Re-enabled LED GPIO
#define NTC_ADC_CH  ADC_CHANNEL_3 // Assuming GPIO3
#define R2 330000.0f // top resistor (Vbat -> ADC)
#define R1 110000.0f // bottom resistor (ADC -> GND)

#define WIFI_SSID "Tufts_Wireless"
#define WIFI_PASS ""
#define BROKER_URI "mqtt://bell-mqtt.eecs.tufts.edu/"

// Sleep interval in seconds (1 hour)
#define SLEEP_INTERVAL_S 3600 
#define MAX_MQTT_RETRY 10 // Max retry for MQTT connection
#include "freertos/event_groups.h" // Important: include the type definition


#define NVS_NAMESPACE "storage"
#define NVS_KEY_LAST_EPOCH "last_epoch"


// Add this extern declaration:
extern EventGroupHandle_t s_wifi_event_group; 

// Keep the function prototype:
esp_err_t wifi_connect(const char* ssid, const char* pass);
// --- 2. GLOBAL STATE & ERROR FLAGS ---
typedef enum {
    STATUS_OK = 0,
    STATUS_WIFI_FAIL = 1,
    STATUS_MQTT_FAIL = 2,
    STATUS_I2C_FAIL = 3,
    STATUS_NTP_FAIL = 4,
} system_status_t;

static system_status_t system_status = STATUS_OK; 


static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t dev;

static const char *TAG = "Iteration 2, Group 1";
static const char *MQTT_TAG = "MQTT";

esp_mqtt_client_handle_t client;
static bool mqtt_connected = false;
static bool wifi_connected = false;
static bool mqtt_published = false; // Flag to track successful publish


// --- 3. HELPER FUNCTIONS ---

static void init_adc(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(NTC_ADC_CH, ADC_ATTEN_DB_11);
}

void i2cinit(void)
{
    i2c_master_bus_config_t b = {
        .i2c_port = I2C_PORT,
        .scl_io_num = SCL_PIN,
        .sda_io_num = SDA_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags = {.enable_internal_pullup = 1}, 
    };
    if (ESP_OK != i2c_new_master_bus(&b, &bus)) {
        ESP_LOGE(TAG, "I2C Bus Init FAILED");
        system_status = STATUS_I2C_FAIL;
        return;
    }

    i2c_device_config_t d = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TMP1075_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    if (ESP_OK != i2c_master_bus_add_device(bus, &d, &dev)) {
        ESP_LOGE(TAG, "I2C Device Add FAILED");
        system_status = STATUS_I2C_FAIL;
    }
}

static float get_temp_ic(void)
{
    if (system_status == STATUS_I2C_FAIL) return -999.9f;

    uint8_t ptr = 0x00;
    uint8_t rx[2] = {0};

    esp_err_t e = i2c_master_transmit_receive(dev, &ptr, 1, rx, 2, 300);
    if (e != ESP_OK) {
        ESP_LOGE("TMP1075", "I2C read failed: %s", esp_err_to_name(e));
        system_status = STATUS_I2C_FAIL;
        return -999.9;
    }

    int16_t raw = ((int16_t)rx[0] << 8) | rx[1];
    raw >>= 4;
    float temp_c = (float)raw * 0.0625f;
    return temp_c;
}


float readvoltage(void) {
    int acc = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        acc += adc1_get_raw(NTC_ADC_CH);
    }
    float raw = (float)acc / (float)ADC_SAMPLES;
    float v_adc = (raw / 4095.0f) * ADC_VREF;
    float v_bat = v_adc * ((R1 + R2) / R2);

    ESP_LOGI("Bat", "Battery raw is %.2f", v_bat);
    return v_bat;
}

#define BATTERY_MAX_VOLTAGE 4.20f // 100% full charge
#define BATTERY_MIN_VOLTAGE .00f // 0% discharged (cutoff voltage)

/**
 * @brief Converts the measured battery voltage (Vbat) to a percentage (0-100%).
 * * Uses linear interpolation based on defined MAX and MIN voltage boundaries.
 * * @param vbat_voltage The battery voltage read by the ADC function.
 * @return float The battery percentage (0.0f to 100.0f).
 */
float voltage_to_percentage(float vbat_voltage) {
    
    // 1. Boundary Check (Clamp the voltage)
    
    // If voltage is above MAX, it's 100%
    if (vbat_voltage >= BATTERY_MAX_VOLTAGE) {
        return 100.0f;
    }
    
    // If voltage is below MIN, it's 0%
    if (vbat_voltage <= BATTERY_MIN_VOLTAGE) {
        return 0.0f;
    }

    // 2. Linear Interpolation Calculation
    
    // Calculate the range of voltages used for 0% to 100%
    float voltage_range = BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE;
    
    // Calculate the voltage above the minimum threshold
    float voltage_above_min = vbat_voltage - BATTERY_MIN_VOLTAGE;
    
    // Calculate percentage (scale the result to 0-100)
    float percentage = (voltage_above_min / voltage_range) * 100.0f;

    // Return the calculated percentage
    return percentage;
}

// Example usage in your app_main (assuming you call readvoltage() first):
/*
    float batt_vol = readvoltage();
    float batt_percent = voltage_to_percentage(batt_vol); // <-- New line
    
    // Pass batt_percent to the build_json function
    char *json = build_json(epoch, t_ic, rssi, batt_vol, batt_percent, system_status);
*/

uint64_t get_epoch_time(void)
{
    nvs_handle_t nvs_handle;
    uint64_t last_epoch = 0;
    esp_err_t err;
    uint64_t estimated_epoch = 0;

    // --- 1. Read last successful epoch from NVS ---
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
        err = nvs_get_u64(nvs_handle, NVS_KEY_LAST_EPOCH, &last_epoch);
        nvs_close(nvs_handle);
    }
    
    if (err == ESP_OK && last_epoch > 0) {
        // Estimate the current time based on the last reading and sleep duration
        estimated_epoch = last_epoch + SLEEP_INTERVAL_S;
        ESP_LOGI("NTP", "Last synchronized epoch: %llu. Estimated current epoch: %llu", 
                 (unsigned long long)last_epoch, (unsigned long long)estimated_epoch);
    } else {
        ESP_LOGW("NTP", "No valid last epoch found in NVS. Starting from 0.");
    }
    
    // --- 2. Attempt NTP Synchronization ---
    ESP_LOGI("NTP", "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    int retry = 0;
    const int retry_count = 15;
    
    // Wait for sync or timeout
    while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED && retry < retry_count) {
        ESP_LOGI("NTP", "Waiting for system time... (Status: %d) (%d/%d)", 
                 sntp_get_sync_status(), retry+1, retry_count);
        vTaskDelay(pdMS_TO_TICKS(1000));
        retry++;
    }

    time_t now_ntp;
    time(&now_ntp); // Get time from RTC, whether synced or not

    // --- 3. Process Result and Save to NVS ---
    if (retry < 15) {
        ESP_LOGI("NTP", "NTP synchronization SUCCESS. Epoch: %llu", (unsigned long long)now_ntp);
        
        // Save the new, valid time to NVS
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle) == ESP_OK) {
            nvs_set_u64(nvs_handle, NVS_KEY_LAST_EPOCH, (uint64_t)now_ntp);
            nvs_commit(nvs_handle);
            nvs_close(nvs_handle);
            ESP_LOGI("NTP", "New epoch saved to NVS.");
        } else {
            ESP_LOGE("NTP", "Failed to open NVS for writing!");
        }
        
        // Return the successfully synchronized time
        return (uint64_t)now_ntp;
    } 
    
    // --- 4. NTP Failed: Fallback to Estimated Time ---
    else {
        ESP_LOGW("NTP", "NTP synchronization FAILED after %d attempts. Status: %d", retry, sntp_get_sync_status());
        system_status = STATUS_NTP_FAIL;
        
        if (estimated_epoch > 0) {
            ESP_LOGW("NTP", "Falling back to estimated epoch time: %llu", (unsigned long long)estimated_epoch);
            
            // Note: We don't save this estimated time back to NVS, only successfully synced time.
            return estimated_epoch;
        } else {
            ESP_LOGE("NTP", "No estimated time available. Returning 0.");
            return 0; 
        }
    }
}

void wifiinit(){
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    // *** CRITICAL ADDITION: INITIALIZE THE EVENT GROUP ***
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create WiFi event group!");
        system_status = STATUS_WIFI_FAIL;
        return; 
    }
    
    // 检查新的返回值
    if (wifi_connect(WIFI_SSID, WIFI_PASS) == ESP_OK) { // <-- Now this will not crash
        wifi_connected = true;
    } else {
        ESP_LOGE(TAG, "WiFi connection FAILED!");
        system_status = STATUS_WIFI_FAIL;
    }
}

void ledinit(){
     // Configure pin as output
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1); // LED OFF (assuming active low or pulling high)
}

void ledflash(void)
{
    // Simplified flash for success indication
    for (int i=0; i<3; i++){ // Flash 3 times
        gpio_set_level(LED_GPIO, 0); // LED ON
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_GPIO, 1); // LED OFF
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// --- MQTT Event Handler and Init ---
static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch (event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(MQTT_TAG, "MQTT connected");
        mqtt_connected = true;
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(MQTT_TAG, "MQTT message published, msg_id=%d", event->msg_id);
        mqtt_published = true; // <-- Set flag on successful publish
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(MQTT_TAG, "MQTT disconnected");
        mqtt_connected = false;
        system_status = STATUS_MQTT_FAIL;
        break;
    default:
        break;
    }
}

static void mqttinit(void)
{
    if (!wifi_connected) {
        ESP_LOGW(MQTT_TAG, "Skipping MQTT init because WiFi is down.");
        return;
    }
    
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = BROKER_URI,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);

    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);

    esp_mqtt_client_start(client);
}

int get_rssi(void)
{
    if (!wifi_connected) return 0;

    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return ap_info.rssi;
    } else {
        ESP_LOGE("RSSI", "Failed to get AP info");
        return 0;
    }
}
/**
 * @brief Converts the global status code into a human-readable error/status message.
 * @param status_code The integer status code (from system_status_t enum).
 * @return const char* Pointer to the static status string.
 */
const char* get_status_message(int status_code) {
    switch (status_code) {
        case STATUS_OK:
            return "SUCCESS: Data published.";
        case STATUS_WIFI_FAIL:
            return "ERROR: Failed to connect to WiFi.";
        case STATUS_MQTT_FAIL:
            return "ERROR: Failed to connect to MQTT broker.";
        case STATUS_I2C_FAIL:
            return "ERROR: I2C sensor (TMP1075) read failed.";
        case STATUS_NTP_FAIL:
            return "WARNING: Failed to synchronize time (NTP).";
        default:
            return "FATAL ERROR: Unknown status code.";
    }
}

char *build_json(uint64_t epoch, float tempval, int rssi, float batt_vol, int status_code)
{
    // Allocate enough memory for the JSON string
    char *json = malloc(300); 
    if (!json) return NULL;

    const char* status_text = get_status_message(status_code); 

    if(status_code == 0) {}
    else if(status_code == 1 || status_code == 2 || status_code == 4){
        status_code = 1;
    }else{
        status_code = 2;
    }

    float batt_percent = voltage_to_percentage(batt_vol);
    
    // --- 修正后的 snprintf ---
    snprintf(json, 300,
    "{"
        "\"measurements\": ["
            "[%llu, %.2f],[%llu, %d],[%llu, %.2f]" 
        "],"
        "\"board_time\": %llu,"
        "\"heartbeat\": {"
            "\"status\": %d," 
            "\"battery_percent\": %.2f,"
            "\"rssi\": %d,"            
            "\"text\": [\"%s\"]"        
        "}"
    "}",
    (unsigned long long)epoch, tempval, epoch, rssi, epoch, batt_percent,// measurements (3 arguments)
    (unsigned long long)epoch,                 // board_time (1 argument)
    status_code,                               // status (1 argument)
    batt_percent,                              // battery_percent (1 argument)
    rssi,                                      // rssi (1 argument)
    status_text                                // text (1 argument)
    ); // 8 arguments in total (3 + 1 + 1 + 1 + 1 + 1)

    return json;
}

void enterDeepSleep(long long interval){
    ESP_LOGI("SLEEP", "Device is going to sleep for %lld second...", interval);

    const int64_t sleep_time_us = interval * 1000000LL;
    esp_sleep_enable_timer_wakeup(sleep_time_us);
    
    // Cleanup and flush logs before sleep
    esp_log_level_set("*", ESP_LOG_NONE); 
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_deep_sleep_start();
}


// --- 4. Main Application ---

void app_main(void)
{
    // --- Phase 1: Initialization ---
    nvs_flash_init();
    init_adc();
    ledinit(); // Initialize LED pin
    i2cinit(); 
    
    // --- Phase 2: Connection & Data Gathering ---
    wifiinit();
    
    uint64_t epoch = 0;
    if (wifi_connected) {
        epoch = get_epoch_time();
    }
    
    mqttinit();
    
    // Wait for MQTT connection, but limited time
    int retry = 0;
    while (!mqtt_connected && retry < MAX_MQTT_RETRY && wifi_connected) {
        ESP_LOGI(MQTT_TAG, "Waiting for MQTT connection...");
        vTaskDelay(pdMS_TO_TICKS(200)); 
        retry++;
    }
    if (!mqtt_connected && wifi_connected) { // Only set MQTT fail if WiFi was OK
        system_status = STATUS_MQTT_FAIL;
    }

    // --- Phase 3: Sensor Read & Publish ---
    float t_ic = get_temp_ic();
    float batt_vol = readvoltage();
    int rssi = get_rssi();

    char *json = build_json(epoch, t_ic, rssi, batt_vol, system_status);
    
    if (json && mqtt_connected) {
        ESP_LOGI("JSON", "%s", json);
        
        // Publish message
        int msg_id = esp_mqtt_client_publish(client, "teamK/node0/update",
                                              json, 0, 1, 0);
        ESP_LOGI(MQTT_TAG, "Publish msg_id=%d", msg_id);

        // Wait short time to allow MQTT event to be processed
        vTaskDelay(pdMS_TO_TICKS(500)); 
    } else {
        ESP_LOGE(TAG, "Publish skipped. Status: %d", system_status);
    }

    if (json) free(json);
    
    // Stop MQTT client gracefully
    if (client) {
        esp_mqtt_client_stop(client);
    }
    
    // --- Phase 4: Conditional LED Flash and Deep Sleep ---
    
    if (mqtt_published) { // Check the flag set by MQTT_EVENT_PUBLISHED
        ledflash(); // Flash LED on success
    }
    
    enterDeepSleep(SLEEP_INTERVAL_S);
}