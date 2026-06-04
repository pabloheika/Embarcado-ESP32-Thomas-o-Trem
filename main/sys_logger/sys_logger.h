#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SYS_LOG_DEBUG = 0,
    SYS_LOG_INFO,
    SYS_LOG_WARN,
    SYS_LOG_ERROR,
} sys_log_level_t;

typedef struct {
    int64_t timestamp_ms;
    sys_log_level_t level;
    char tag[16];
    char message[128];
} sys_log_entry_t;

#define SYS_LOG_MAX_ENTRIES 64

void sys_logger_init(void);

void sys_log(sys_log_level_t level, const char *tag, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

int sys_logger_get_recent(sys_log_entry_t *out, int max_entries);

sys_log_level_t sys_logger_get_level(void);

void sys_logger_set_level(sys_log_level_t level);

#ifdef __cplusplus
}
#endif
