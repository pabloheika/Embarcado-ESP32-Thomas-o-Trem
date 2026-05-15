#include "web_api.h"

#include <string.h>

#include "cJSON.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"

static const char *TAG = "web_api";

#define MAX_JSON_BODY 512

static void add_cors_headers(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

static esp_err_t cors_options_handler(httpd_req_t *req)
{
    add_cors_headers(req);
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t dispatch_command(const char *cmd, const cJSON *payload, cJSON **out_body)
{
    (void)payload;
    if (cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (strcmp(cmd, "noop") == 0) {
        *out_body = cJSON_CreateObject();
        if (*out_body == NULL) {
            return ESP_ERR_NO_MEM;
        }
        cJSON_AddStringToObject(*out_body, "status", "ok");
        cJSON_AddStringToObject(*out_body, "cmd", "noop");
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

static esp_err_t telemetry_get_handler(httpd_req_t *req)
{
    add_cors_headers(req);
    httpd_resp_set_type(req, "application/json");

    int8_t rssi = 0;
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        rssi = ap.rssi;
    }

    const int64_t uptime_ms = esp_timer_get_time() / 1000;

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "{\"error\":\"oom\"}");
    }

    cJSON_AddNumberToObject(root, "uptime_ms", (double)uptime_ms);
    cJSON_AddNumberToObject(root, "wifi_rssi", rssi);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
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

    const cJSON *payload = cJSON_GetObjectItemCaseSensitive(root, "payload");
    cJSON *out = NULL;
    esp_err_t err = dispatch_command(cmd_item->valuestring, payload, &out);
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
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.lru_purge_enable = true;
    config.server_port = 80;

    httpd_handle_t server = NULL;
    ESP_RETURN_ON_ERROR(httpd_start(&server, &config), TAG, "httpd_start");

    httpd_uri_t uri_telemetry_get = {
        .uri = "/api/telemetry",
        .method = HTTP_GET,
        .handler = telemetry_get_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t uri_telemetry_opt = {
        .uri = "/api/telemetry",
        .method = HTTP_OPTIONS,
        .handler = cors_options_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t uri_command_post = {
        .uri = "/api/command",
        .method = HTTP_POST,
        .handler = command_post_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t uri_command_opt = {
        .uri = "/api/command",
        .method = HTTP_OPTIONS,
        .handler = cors_options_handler,
        .user_ctx = NULL,
    };

    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &uri_telemetry_get), TAG, "reg telemetry GET");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &uri_telemetry_opt), TAG, "reg telemetry OPTIONS");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &uri_command_post), TAG, "reg command POST");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &uri_command_opt), TAG, "reg command OPTIONS");

    ESP_LOGI(TAG, "HTTP API em http://<ip>:80 (telemetry, command)");
    return ESP_OK;
}
