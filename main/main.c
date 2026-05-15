#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "web_api.h"
#include "wifi.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando carrinho (WiFi + HTTP)...");

    ESP_ERROR_CHECK(wifi_station_init());
    ESP_ERROR_CHECK(wifi_station_wait_connected(pdMS_TO_TICKS(120000)));

    ESP_ERROR_CHECK(web_api_start());

    ESP_LOGI(TAG, "Pronto. Use GET /api/telemetry e POST /api/command no IP acima.");
}
