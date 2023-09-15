/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/*
 * softAP to PPPoS Example (network_dce)
*/

#include <string.h>
#include "esp_netif.h"
#include "esp_modem_api.h"


static esp_modem_dce_t *dce = NULL;


esp_err_t modem_init_network(esp_netif_t *netif)
{
    // setup the DCE
    esp_modem_dte_config_t dte_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    /* setup UART specific configuration based on kconfig options */
    dte_config.uart_config.tx_io_num = 25;
    dte_config.uart_config.rx_io_num = 26;
    // dte_config.uart_config.rts_io_num = 33;
    // dte_config.uart_config.cts_io_num = 27;
    // dte_config.uart_config.flow_control = ESP_MODEM_FLOW_CONTROL_HW;
    dte_config.uart_config.flow_control = ESP_MODEM_FLOW_CONTROL_NONE;
    dte_config.uart_config.rx_buffer_size = 4096;
    dte_config.uart_config.tx_buffer_size = 4096;
    dte_config.uart_config.event_queue_size = 30;
    dte_config.uart_config.baud_rate = 115200;
    dte_config.task_stack_size = 8192;
    dte_config.task_priority = 5;
    dte_config.dte_buffer_size = 4096 / 2;
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(CONFIG_EXAMPLE_MODEM_PPP_APN);
    dce = esp_modem_new_dev(ESP_MODEM_DCE_BG96, &dte_config, &dce_config, netif);
    if (!dce) {
        return ESP_FAIL;
    }

#ifdef CONFIG_EXAMPLE_NEED_SIM_PIN
    // configure the PIN
    bool pin_ok = false;
    if (esp_modem_read_pin(dce, &pin_ok) == ESP_OK && pin_ok == false) {
        if (esp_modem_set_pin(dce, CONFIG_EXAMPLE_SIM_PIN) == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            abort();
        }
    }
#endif // CONFIG_EXAMPLE_NEED_SIM_PIN
    return ESP_OK;
}

void modem_deinit_network(void)
{
    if (dce) {
        esp_modem_destroy(dce);
        dce = NULL;
    }
}

bool modem_start_network()
{
    return esp_modem_set_mode(dce, ESP_MODEM_MODE_DATA) == ESP_OK;
}

bool modem_stop_network()
{
    return esp_modem_set_mode(dce, ESP_MODEM_MODE_COMMAND);
}

bool modem_check_sync()
{
    return esp_modem_sync(dce) == ESP_OK;
}

void modem_reset()
{
    esp_modem_reset(dce);
}

bool modem_check_signal()
{
    int rssi, ber;
    if (esp_modem_get_signal_quality(dce, &rssi, &ber) == ESP_OK) {
        return rssi != 99 && rssi > 5;
    }
    return false;
}

bool modem_set_baud(int baud)
{
    return esp_modem_set_baud(dce, baud) == ESP_OK;
}

bool modem_set_flow_control()
{
    return esp_modem_set_flow_control(dce, 2, 2) == ESP_OK;
}