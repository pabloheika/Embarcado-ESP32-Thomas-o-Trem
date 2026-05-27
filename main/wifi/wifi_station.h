#pragma once

#include <stddef.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/** Bit definido quando STA obtem IP (DHCP). */
#define WIFI_STATION_CONNECTED_BIT ((EventBits_t)0x01u)

/**
 * Inicializa NVS (se necessario), netif, loop de eventos e WiFi em modo STA.
 * Nao bloqueia; chame wifi_station_wait_connected() antes do servidor HTTP.
 */
esp_err_t wifi_station_init(void);

/**
 * Aguarda IP ou timeout. Retorna ESP_OK se conectado, erro caso contrario.
 */
esp_err_t wifi_station_wait_connected(TickType_t timeout_ticks);

/** Copia o IPv4 da STA (ex.: "192.168.0.42"). Requer conexao com IP. */
esp_err_t wifi_station_get_ip(char *buf, size_t buf_len);

/** Event group interno (para testes); preferir wait_connected. */
EventGroupHandle_t wifi_station_event_group(void);
