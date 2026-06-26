#include "sys_logger.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static sys_log_entry_t s_log_buffer[SYS_LOG_MAX_ENTRIES];
static int s_log_head = 0;
static int s_log_count = 0;
static SemaphoreHandle_t s_log_mutex = NULL;
static sys_log_level_t s_min_level = SYS_LOG_INFO;

static const char* level_names[] = {
    "DEBUG", "INFO ", "WARN ", "ERROR"
};

static const char* level_colors[] = {
    "\033[36m", // DEBUG cyan
    "\033[32m", // INFO green
    "\033[33m", // WARN yellow
    "\033[31m"  // ERROR red
};
static const char* color_reset = "\033[0m";

void sys_logger_init(void) {
    if (!s_log_mutex) {
        s_log_mutex = xSemaphoreCreateMutex();
    }
    printf("╔══════════════════════════════════════════════╗\n");
    printf("║    Thomas, o Trem — Sistema de Logs Ativo    ║\n");
    printf("╚══════════════════════════════════════════════╝\n");
}

void sys_log(sys_log_level_t level, const char *tag, const char *fmt, ...) {
    if (level < s_min_level) return;
    
    int64_t ts_ms = esp_timer_get_time() / 1000;
    
    char msg_buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg_buf, sizeof(msg_buf), fmt, args);
    va_end(args);
    
    printf("%s[%7lldms] [%-5s] [%-9s] %s%s\n", 
           level_colors[level], 
           (long long)ts_ms, 
           level_names[level], 
           tag ? tag : "", 
           msg_buf, 
           color_reset);
           
    if (s_log_mutex) {
        if (xSemaphoreTake(s_log_mutex, portMAX_DELAY) == pdTRUE) {
            sys_log_entry_t *entry = &s_log_buffer[s_log_head];
            entry->timestamp_ms = ts_ms;
            entry->level = level;
            strncpy(entry->tag, tag ? tag : "", sizeof(entry->tag) - 1);
            entry->tag[sizeof(entry->tag) - 1] = '\0';
            strncpy(entry->message, msg_buf, sizeof(entry->message) - 1);
            entry->message[sizeof(entry->message) - 1] = '\0';
            
            s_log_head = (s_log_head + 1) % SYS_LOG_MAX_ENTRIES;
            if (s_log_count < SYS_LOG_MAX_ENTRIES) {
                s_log_count++;
            }
            xSemaphoreGive(s_log_mutex);
        }
    }
}

int sys_logger_get_recent(sys_log_entry_t *out, int max_entries) {
    if (!out || max_entries <= 0 || !s_log_mutex) return 0;
    
    int copied = 0;
    if (xSemaphoreTake(s_log_mutex, portMAX_DELAY) == pdTRUE) {
        int to_copy = (s_log_count < max_entries) ? s_log_count : max_entries;
        
        int idx = s_log_head - to_copy;
        if (idx < 0) idx += SYS_LOG_MAX_ENTRIES;
        
        for (int i = 0; i < to_copy; i++) {
            out[i] = s_log_buffer[idx];
            idx = (idx + 1) % SYS_LOG_MAX_ENTRIES;
        }
        copied = to_copy;
        xSemaphoreGive(s_log_mutex);
    }
    return copied;
}

sys_log_level_t sys_logger_get_level(void) {
    return s_min_level;
}

void sys_logger_set_level(sys_log_level_t level) {
    s_min_level = level;
}
