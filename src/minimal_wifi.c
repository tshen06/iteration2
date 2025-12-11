/* Simple WiFi connection example for ESP32-C3 / Tufts EE 193-05
 * Based on public domain example from Espressif:
 * https://github.com/espressif/esp-idf/blob/5f4249357372f209fdd57288265741aaba21a2b1/examples/wifi/getting_started/station/main/station_example_main.c
 * Steven Bell <sbell@ece.tufts.edu>
 * February 2024
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define WIFI_MAXIMUM_RETRIES  2  // Maximum number of times to retry if connecting fails

// Minimal acceptable authentication (open, WEP, WPA, WPA2, etc.)
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN


// FreeRTOS event group to signal when we are connected.
// This allows the callback function to communicate back to the main WiFi setup routine
EventGroupHandle_t s_wifi_event_group;
// The event group allows multiple bits for each event, but we only care about two events:
// 0) we are connected to the AP with an IP
// 1) we failed to connect after the maximum amount of retries
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "Iteration 2, Group 1";

static int s_retry_num = 0;


/* Callback for handling all WiFi events.  This could be broken out into
 * separate functions for each of the events, but it doesn't really matter. */
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRIES) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* Initialize the WiFi STA connection.
 * This function configures the connection and then blocks until the connection either
 * succeeds or fails. */

// --- Assumed necessary definitions (MUST be defined globally in your main.c or header) ---
// extern EventGroupHandle_t s_wifi_event_group; 
// #define WIFI_CONNECTED_BIT BIT0
// #define WIFI_FAIL_BIT BIT1
// -----------------------------------------------------------------------------------------


esp_err_t wifi_connect(const char* ssid, const char* pass) // <-- 改变函数签名以返回 esp_err_t
{
    // WARNING: s_wifi_event_group must be initialized globally or outside this function if called repeatedly.
    // For this demonstration, we assume it's correctly managed.
    // s_wifi_event_group = xEventGroupCreate(); 

    // Initialize networking components (usually done once in app_main)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    
    // Register event handlers (assuming event_handler is defined elsewhere)
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
        },
    };
    // Use strlcpy for safer buffer handling if available, otherwise strncpy is used below
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password));
    
    // Configure and start WiFi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "WiFi configured, waiting for callbacks.");

    /* Wait for connection established (WIFI_CONNECTED_BIT) or failed (WIFI_FAIL_BIT) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                pdFALSE,
                pdFALSE,
                portMAX_DELAY); // Wait indefinitely

    /* Process the result of the connection attempt */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to AP SSID: %s", ssid);
        return ESP_OK; // <-- 成功连接时返回 ESP_OK
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID: %s", ssid);
        return ESP_FAIL; // <-- 连接失败时返回 ESP_FAIL
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        return ESP_ERR_NOT_FOUND; // <-- 发生意外事件时返回其他错误
    }
}