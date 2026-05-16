#pragma once

#include "esp_err.h"

/** Inicia servidor HTTP (REST + CORS). Chamar apos WiFi com IP. */
esp_err_t web_api_start(void);
