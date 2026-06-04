#include "web_api.h"

#include <string.h>

#include "cJSON.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi_station.h"
#include "sys_logger.h"
#include "mode_manager.h"
#include "motor_driver.h"
#include "qtr8rc.h"
#include "mpu6050.h"

static const char *TAG = "web_api";

#define MAX_JSON_BODY 1024

static const char *s_estado_atuador = "parado";

static void add_cors_headers(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Private-Network", "true");
}

static esp_err_t cors_options_handler(httpd_req_t *req)
{
    add_cors_headers(req);
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t dispatch_command(const char *cmd, const cJSON *payload, cJSON **out_body)
{
    *out_body = cJSON_CreateObject();
    if (*out_body == NULL) return ESP_ERR_NO_MEM;

    if (strcmp(cmd, "noop") == 0) {
        cJSON_AddStringToObject(*out_body, "status", "ok");
        return ESP_OK;
    }

    if (strcmp(cmd, "set_mode") == 0) {
        if (!payload) return ESP_ERR_INVALID_ARG;
        cJSON *mode_item = cJSON_GetObjectItem(payload, "mode");
        if (cJSON_IsString(mode_item)) {
            car_mode_t m = MODE_IDLE;
            if (strcmp(mode_item->valuestring, "idle") == 0) m = MODE_IDLE;
            else if (strcmp(mode_item->valuestring, "auto") == 0) m = MODE_AUTONOMOUS;
            else if (strcmp(mode_item->valuestring, "manual") == 0) m = MODE_MANUAL;
            else if (strcmp(mode_item->valuestring, "calibrate") == 0) m = MODE_CALIBRATING;
            
            if (mode_manager_set_mode(m) == ESP_OK) {
                cJSON_AddStringToObject(*out_body, "status", "ok");
                return ESP_OK;
            }
        }
        return ESP_ERR_INVALID_ARG;
    }

    if (strcmp(cmd, "manual_move") == 0) {
        if (mode_manager_get_mode() != MODE_MANUAL) return ESP_ERR_INVALID_STATE;
        if (!payload) return ESP_ERR_INVALID_ARG;
        
        cJSON *left = cJSON_GetObjectItem(payload, "left");
        cJSON *right = cJSON_GetObjectItem(payload, "right");
        
        if (cJSON_IsNumber(left) && cJSON_IsNumber(right)) {
            int l_val = left->valueint;
            int r_val = right->valueint;
            
            if (l_val == 0) motor_left_brake();
            else motor_left_set((uint8_t)(l_val > 0 ? l_val : -l_val), l_val > 0);
            
            if (r_val == 0) motor_right_brake();
            else motor_right_set((uint8_t)(r_val > 0 ? r_val : -r_val), r_val > 0);
            
            cJSON_AddStringToObject(*out_body, "status", "ok");
            return ESP_OK;
        }
        return ESP_ERR_INVALID_ARG;
    }

    if (strcmp(cmd, "set_pid") == 0) {
        if (!payload) return ESP_ERR_INVALID_ARG;
        cJSON *kp = cJSON_GetObjectItem(payload, "kp");
        cJSON *ki = cJSON_GetObjectItem(payload, "ki");
        cJSON *kd = cJSON_GetObjectItem(payload, "kd");
        
        if (cJSON_IsNumber(kp) && cJSON_IsNumber(ki) && cJSON_IsNumber(kd)) {
            car_state_t *state = mode_manager_get_state();
            pid_set_params(&state->pid, kp->valuedouble, ki->valuedouble, kd->valuedouble);
            cJSON_AddStringToObject(*out_body, "status", "ok");
            return ESP_OK;
        }
        return ESP_ERR_INVALID_ARG;
    }

    if (strcmp(cmd, "set_speed") == 0) {
        if (!payload) return ESP_ERR_INVALID_ARG;
        cJSON *speed = cJSON_GetObjectItem(payload, "speed");
        if (cJSON_IsNumber(speed)) {
            car_state_t *state = mode_manager_get_state();
            state->base_speed = speed->valueint;
            cJSON_AddStringToObject(*out_body, "status", "ok");
            return ESP_OK;
        }
        return ESP_ERR_INVALID_ARG;
    }

    if (strcmp(cmd, "acionar") == 0) {
        actuator_set(255, true);
        s_estado_atuador = "subindo";
        cJSON_AddStringToObject(*out_body, "status", "acionado");
        return ESP_OK;
    }

    if (strcmp(cmd, "recolher") == 0) {
        actuator_set(255, false);
        s_estado_atuador = "descendo";
        cJSON_AddStringToObject(*out_body, "status", "recolhendo");
        return ESP_OK;
    }

    if (strcmp(cmd, "parar") == 0) {
        actuator_brake();
        s_estado_atuador = "parado";
        cJSON_AddStringToObject(*out_body, "status", "parado");
        return ESP_OK;
    }

    if (strcmp(cmd, "emergency") == 0) {
        mode_manager_set_mode(MODE_EMERGENCY);
        motor_emergency_stop();
        cJSON_AddStringToObject(*out_body, "status", "emergency");
        return ESP_OK;
    }

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

    mpu6050_sample_t imu = {0};
    mpu6050_get_last_sample(&imu);

    qtr8rc_reading_t line = {0};
    qtr8rc_get_last_reading(&line);

    car_state_t *state = mode_manager_get_state();

    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;

    cJSON_AddNumberToObject(root, "uptime_ms", (double)uptime_ms);
    cJSON_AddNumberToObject(root, "wifi_rssi", rssi);

    char ip[16] = {0};
    if (wifi_station_get_ip(ip, sizeof(ip)) == ESP_OK) {
        cJSON_AddStringToObject(root, "ip", ip);
    }

    const char *mode_str = "idle";
    if (state->mode == MODE_AUTONOMOUS) mode_str = "auto";
    else if (state->mode == MODE_MANUAL) mode_str = "manual";
    else if (state->mode == MODE_CALIBRATING) mode_str = "calibrate";
    else if (state->mode == MODE_EMERGENCY) mode_str = "emergency";

    cJSON_AddStringToObject(root, "mode", mode_str);
    cJSON_AddNumberToObject(root, "base_speed", state->base_speed);
    cJSON_AddBoolToObject(root, "motors_enabled", state->motors_enabled);
    cJSON_AddStringToObject(root, "atuador", s_estado_atuador);

    cJSON *pid_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(pid_json, "kp", state->pid.kp);
    cJSON_AddNumberToObject(pid_json, "ki", state->pid.ki);
    cJSON_AddNumberToObject(pid_json, "kd", state->pid.kd);
    cJSON_AddNumberToObject(pid_json, "error", state->pid.last_error);
    cJSON_AddNumberToObject(pid_json, "correction", state->pid.last_output);
    cJSON_AddItemToObject(root, "pid", pid_json);

    cJSON *qtr_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(qtr_json, "black_mask", line.black_mask);
    cJSON_AddBoolToObject(qtr_json, "line_detected", line.line_detected);
    cJSON *norm_arr = cJSON_CreateIntArray((const int *)line.norm, 8);
    cJSON_AddItemToObject(qtr_json, "norm", norm_arr);
    cJSON_AddItemToObject(root, "qtr8rc", qtr_json);

    cJSON *motors_json = cJSON_CreateObject();
    cJSON_AddNumberToObject(motors_json, "left_duty", motor_get_left_duty());
    cJSON_AddNumberToObject(motors_json, "right_duty", motor_get_right_duty());
    cJSON_AddItemToObject(root, "motors", motors_json);

    cJSON *mpu = cJSON_CreateObject();
    cJSON_AddNumberToObject(mpu, "accel_x", imu.accel_x);
    cJSON_AddNumberToObject(mpu, "accel_y", imu.accel_y);
    cJSON_AddNumberToObject(mpu, "accel_z", imu.accel_z);
    cJSON_AddNumberToObject(mpu, "gyro_x", imu.gyro_x);
    cJSON_AddNumberToObject(mpu, "gyro_y", imu.gyro_y);
    cJSON_AddNumberToObject(mpu, "gyro_z", imu.gyro_z);
    cJSON_AddItemToObject(root, "mpu6050", mpu);

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json_str;
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

static esp_err_t logs_get_handler(httpd_req_t *req)
{
    add_cors_headers(req);
    httpd_resp_set_type(req, "application/json");

    sys_log_entry_t *entries = malloc(sizeof(sys_log_entry_t) * SYS_LOG_MAX_ENTRIES);
    if (!entries) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "{\"error\":\"oom_entries\"}");
    }

    int count = sys_logger_get_recent(entries, SYS_LOG_MAX_ENTRIES);

    cJSON *root = cJSON_CreateObject();
    cJSON *logs_arr = cJSON_CreateArray();

    for (int i = 0; i < count; i++) {
        cJSON *entry = cJSON_CreateObject();
        cJSON_AddNumberToObject(entry, "ts", (double)entries[i].timestamp_ms);
        
        const char *lvl = "DEBUG";
        if (entries[i].level == SYS_LOG_INFO) lvl = "INFO";
        if (entries[i].level == SYS_LOG_WARN) lvl = "WARN";
        if (entries[i].level == SYS_LOG_ERROR) lvl = "ERROR";
        
        cJSON_AddStringToObject(entry, "level", lvl);
        cJSON_AddStringToObject(entry, "tag", entries[i].tag);
        cJSON_AddStringToObject(entry, "msg", entries[i].message);
        cJSON_AddItemToArray(logs_arr, entry);
    }
    
    cJSON_AddItemToObject(root, "logs", logs_arr);
    
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    
    if (json_str == NULL) {
        free(entries);
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, "{\"error\":\"oom_json\"}");
    }

    esp_err_t send_err = httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    cJSON_free(json_str);
    free(entries);
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
        if (out) cJSON_Delete(out);
        httpd_resp_set_status(req, "404 Not Found");
        return httpd_resp_sendstr(req, "{\"error\":\"unknown_cmd\"}");
    }
    if (err != ESP_OK || out == NULL) {
        if (out) cJSON_Delete(out);
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

    httpd_uri_t routes[] = {
        { .uri = "/api/telemetry", .method = HTTP_GET, .handler = telemetry_get_handler },
        { .uri = "/api/telemetry", .method = HTTP_OPTIONS, .handler = cors_options_handler },
        { .uri = "/api/logs", .method = HTTP_GET, .handler = logs_get_handler },
        { .uri = "/api/logs", .method = HTTP_OPTIONS, .handler = cors_options_handler },
        { .uri = "/api/command", .method = HTTP_POST, .handler = command_post_handler },
        { .uri = "/api/command", .method = HTTP_OPTIONS, .handler = cors_options_handler },
    };

    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server, &routes[i]), TAG, "reg route %d", (int)i);
    }

    char ip[16] = {0};
    if (wifi_station_get_ip(ip, sizeof(ip)) == ESP_OK) {
        ESP_LOGI(TAG, "HTTP API em http://%s", ip);
    }

    return ESP_OK;
}
