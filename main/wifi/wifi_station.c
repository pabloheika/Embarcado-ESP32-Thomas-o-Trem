#include "wifi_station.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "wifi_station";

static EventGroupHandle_t s_wifi_event_group;

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_STATION_CONNECTED_BIT);
        esp_wifi_connect();
        ESP_LOGW(TAG, "Desconectado; tentando reconectar...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *ev = (const ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP obtido: " IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_STATION_CONNECTED_BIT);
    }
}

EventGroupHandle_t wifi_station_event_group(void)
{
    return s_wifi_event_group;
}

esp_err_t wifi_station_init(void)
{
    if (strlen(CONFIG_CARRINHO_WIFI_SSID) == 0) {
        ESP_LOGE(TAG, "CARRINHO_WIFI_SSID vazio. Configure em idf.py menuconfig -> Carrinho WiFi.");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs_flash_erase");
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs_flash_init");

    s_wifi_event_group = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(s_wifi_event_group != NULL, ESP_ERR_NO_MEM, TAG, "EventGroup");

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "esp_event_loop_create_default");
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init");

    ESP_RETURN_ON_ERROR(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL),
                        TAG, "register WIFI_EVENT");
    ESP_RETURN_ON_ERROR(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL),
                        TAG, "register IP_EVENT");

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, CONFIG_CARRINHO_WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, CONFIG_CARRINHO_WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "esp_wifi_set_mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), TAG, "esp_wifi_set_config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start");

    ESP_LOGI(TAG, "Conectando a SSID \"%s\"...", CONFIG_CARRINHO_WIFI_SSID);
    return ESP_OK;
}

esp_err_t wifi_station_wait_connected(TickType_t timeout_ticks)
{
    if (s_wifi_event_group == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_STATION_CONNECTED_BIT, pdFALSE, pdTRUE,
                                           timeout_ticks);
    if ((bits & WIFI_STATION_CONNECTED_BIT) == 0) {
        ESP_LOGE(TAG, "Timeout aguardando IP (STA).");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}
