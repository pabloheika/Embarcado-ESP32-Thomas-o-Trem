#include "web_api.h"

#include <string.h>

#include "cJSON.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wifi_station.h"
// #include "mpu6050.h"
// #include "qtr8rc.h"

static const char *TAG = "web_api";

#define TELEMETRY_LOG_INTERVAL_MS 5000

/* GPIO 18 reservado ao emissor IR do QTR8RC — atuador em 4 (subir) e 19 (descer). */
#define PINO_SUBIR  GPIO_NUM_4
#define PINO_DESCER GPIO_NUM_19

static const char *s_estado_atuador = "parado";

#define MAX_JSON_BODY 512

static void add_cors_headers(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    /* Permite fetch de localhost:5173 para 192.168.x.x (Chrome Private Network Access). */
    httpd_resp_set_hdr(req, "Access-Control-Allow-Private-Network", "true");
}

static esp_err_t cors_options_handler(httpd_req_t *req)
{
    add_cors_headers(req);
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t atuador_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PINO_SUBIR) | (1ULL << PINO_DESCER),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "gpio_config atuador");

    gpio_set_level(PINO_SUBIR, 0);
    gpio_set_level(PINO_DESCER, 0);

    ESP_LOGI(TAG, "GPIOs do atuador (SUBIR=%d, DESCER=%d)", PINO_SUBIR, PINO_DESCER);
    return ESP_OK;
}

static esp_err_t dispatch_command(const char *cmd, const cJSON *payload, cJSON **out_body)
{
    (void)payload;

    if (cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (strcmp(cmd, "noop") == 0) {
        ESP_LOGI(TAG, "[COMANDO] noop — teste de conexao");
        *out_body = cJSON_CreateObject();
        if (*out_body == NULL) {
            return ESP_ERR_NO_MEM;
        }
        cJSON_AddStringToObject(*out_body, "status", "ok");
        cJSON_AddStringToObject(*out_body, "cmd", "noop");
        return ESP_OK;
    }

    if (strcmp(cmd, "acionar") == 0) {
        gpio_set_level(PINO_DESCER, 0);
        gpio_set_level(PINO_SUBIR, 1);
        s_estado_atuador = "subindo";
        ESP_LOGI(TAG, "[ACIONAR] Gancho subindo — GPIO%d=HIGH GPIO%d=LOW", PINO_SUBIR, PINO_DESCER);

        *out_body = cJSON_CreateObject();
        if (*out_body == NULL) {
            return ESP_ERR_NO_MEM;
        }
        cJSON_AddStringToObject(*out_body, "status", "acionado");
        cJSON_AddStringToObject(*out_body, "mensagem", "Gancho subindo");
        return ESP_OK;
    }

    if (strcmp(cmd, "recolher") == 0) {
        gpio_set_level(PINO_SUBIR, 0);
        gpio_set_level(PINO_DESCER, 1);
        s_estado_atuador = "descendo";
        ESP_LOGI(TAG, "[RECOLHER] Gancho descendo — GPIO%d=LOW GPIO%d=HIGH", PINO_SUBIR, PINO_DESCER);

        *out_body = cJSON_CreateObject();
        if (*out_body == NULL) {
            return ESP_ERR_NO_MEM;
        }
        cJSON_AddStringToObject(*out_body, "status", "recolhendo");
        cJSON_AddStringToObject(*out_body, "mensagem", "Gancho descendo");
        return ESP_OK;
    }

    if (strcmp(cmd, "parar") == 0) {
        gpio_set_level(PINO_SUBIR, 0);
        gpio_set_level(PINO_DESCER, 0);
        s_estado_atuador = "parado";
        ESP_LOGI(TAG, "[PARAR] Atuador parado — GPIO%d e GPIO%d = LOW", PINO_SUBIR, PINO_DESCER);

        *out_body = cJSON_CreateObject();
        if (*out_body == NULL) {
            return ESP_ERR_NO_MEM;
        }
        cJSON_AddStringToObject(*out_body, "status", "parado");
        cJSON_AddStringToObject(*out_body, "mensagem", "Atuador parado");
        return ESP_OK;
    }

    ESP_LOGW(TAG, "[COMANDO] desconhecido: \"%s\"", cmd);
    return ESP_ERR_NOT_FOUND;
}

static char *build_telemetry_json(void)
{
    int8_t rssi = 0;
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        rssi = ap.rssi;
    }

    const int64_t uptime_ms = esp_timer_get_time() / 1000;

    // mpu6050_sample_t imu = {0};
    // mpu6050_get_last_sample(&imu);
    //
    // qtr8rc_reading_t line = {0};
    // qtr8rc_get_last_reading(&line);

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return NULL;
    }

    cJSON_AddNumberToObject(root, "uptime_ms", (double)uptime_ms);
    cJSON_AddNumberToObject(root, "wifi_rssi", rssi);

    char ip[16] = {0};
    if (wifi_station_get_ip(ip, sizeof(ip)) == ESP_OK) {
        cJSON_AddStringToObject(root, "ip", ip);
    }

    cJSON_AddStringToObject(root, "atuador", s_estado_atuador);

    // cJSON *mpu = cJSON_CreateObject();
    // if (mpu != NULL) {
    //     cJSON_AddNumberToObject(mpu, "accel_x", imu.accel_x);
    //     cJSON_AddNumberToObject(mpu, "accel_y", imu.accel_y);
    //     cJSON_AddNumberToObject(mpu, "accel_z", imu.accel_z);
    //     cJSON_AddNumberToObject(mpu, "gyro_x", imu.gyro_x);
    //     cJSON_AddNumberToObject(mpu, "gyro_y", imu.gyro_y);
    //     cJSON_AddNumberToObject(mpu, "gyro_z", imu.gyro_z);
    //     cJSON_AddNumberToObject(mpu, "temp", imu.temp);
    //     cJSON_AddItemToObject(root, "mpu6050", mpu);
    // }
    //
    // cJSON *qtr = cJSON_CreateObject();
    // if (qtr != NULL) {
    //     cJSON_AddNumberToObject(qtr, "black_mask", line.black_mask);
    //     cJSON_AddBoolToObject(qtr, "line_detected", line.line_detected);
    //     cJSON_AddItemToObject(root, "qtr8rc", qtr);
    // }

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_str;
}

static void telemetry_log_task(void *arg)
{
    (void)arg;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_LOG_INTERVAL_MS));

        char *json_str = build_telemetry_json();
        if (json_str == NULL) {
            ESP_LOGW(TAG, "GET /api/telemetry -> erro ao montar JSON");
            continue;
        }

        ESP_LOGI(TAG, "GET /api/telemetry -> %s", json_str);
        cJSON_free(json_str);
    }
}

static esp_err_t telemetry_get_handler(httpd_req_t *req)
{
    add_cors_headers(req);
    httpd_resp_set_type(req, "application/json");

    char *json_str = build_telemetry_json();
    if (json_str == NULL) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "{\"error\":\"oom\"}");
    }

    esp_err_t send_err = httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    cJSON_free(json_str);
    return send_err;
}

static esp_err_t command_post_handler(httpd_req_t *req)
{
    add_cors_headers(req);
    httpd_resp_set_type(req, "application/json");

    const int total_len = req->content_len;
    if (total_len <= 0 || total_len >= MAX_JSON_BODY) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "{\"error\":\"invalid_content_length\"}");
    }

    char buf[MAX_JSON_BODY];
    memset(buf, 0, sizeof(buf));
    int received = 0;
    while (received < total_len) {
        const int ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0) {
            httpd_resp_set_status(req, "400 Bad Request");
            return httpd_resp_sendstr(req, "{\"error\":\"recv_failed\"}");
        }
        received += ret;
    }
    buf[total_len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (root == NULL) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "{\"error\":\"invalid_json\"}");
    }

    const cJSON *cmd_item = cJSON_GetObjectItemCaseSensitive(root, "cmd");
    if (!cJSON_IsString(cmd_item) || cmd_item->valuestring == NULL) {
        cJSON_Delete(root);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "{\"error\":\"missing_cmd\"}");
    }

    const char *cmd_str = cmd_item->valuestring;
    ESP_LOGI(TAG, "POST /api/command <- \"%s\"", cmd_str);

    const cJSON *payload = cJSON_GetObjectItemCaseSensitive(root, "payload");
    cJSON *out = NULL;
    esp_err_t err = dispatch_command(cmd_str, payload, &out);
    cJSON_Delete(root);

    if (err == ESP_ERR_NOT_FOUND) {
        httpd_resp_set_status(req, "404 Not Found");
        return httpd_resp_sendstr(req, "{\"error\":\"unknown_cmd\"}");
    }
    if (err != ESP_OK || out == NULL) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "{\"error\":\"command_failed\"}");
    }

    char *resp_str = cJSON_PrintUnformatted(out);
    cJSON_Delete(out);
    if (resp_str == NULL) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "{\"error\":\"oom\"}");
    }

    esp_err_t send_err = httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    cJSON_free(resp_str);
    return send_err;
}

esp_err_t web_api_start(void)
{
    ESP_RETURN_ON_ERROR(atuador_gpio_init(), TAG, "atuador_gpio_init");

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.lru_purge_enable = true;
    config.server_port = 80;

    httpd_handle_t server = NULL;
    ESP_RETURN_ON_ERROR(httpd_start(&server, &config), TAG, "httpd_start");

    httpd_uri_t routes[] = {
        { .uri = "/api/telemetry", .method = HTTP_GET, .handler = telemetry_get_handler },
        { .uri = "/api/telemetry", .method = HTTP_OPTIONS, .handler = cors_options_handler },
        { .uri = "/api/command", .method = HTTP_POST, .handler = command_post_handler },
        { .uri = "/api/command", .method = HTTP_OPTIONS, .handler = cors_options_handler },
    };

    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &routes[i]), TAG, "reg route %d", (int)i);
    }

    char ip[16] = {0};
    if (wifi_station_get_ip(ip, sizeof(ip)) == ESP_OK) {
        ESP_LOGI(TAG, "HTTP API em http://%s", ip);
        ESP_LOGI(TAG, "  GET  http://%s/api/telemetry", ip);
        ESP_LOGI(TAG, "  POST http://%s/api/command (acionar|recolher|parar|noop)", ip);
    } else {
        ESP_LOGI(TAG, "HTTP API na porta 80 (IP indisponivel)");
    }

    xTaskCreate(telemetry_log_task, "telemetry_log", 3072, NULL, 5, NULL);
    ESP_LOGI(TAG, "Log de telemetria a cada %d ms", TELEMETRY_LOG_INTERVAL_MS);

    return ESP_OK;
}
