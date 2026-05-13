#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "contador";

void app_main(void)
{
    while (1) {
        for (int i = 1; i <= 10; i++) {
            ESP_LOGI(TAG, "%d", i);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
