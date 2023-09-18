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

static const int CONNECT_BIT = BIT0;
static const int GOT_DATA_BIT = BIT2;
static const int USB_DISCONNECTED_BIT = BIT3; // Used only with USB DTE but we define it unconditionally, to avoid too many #ifdefs in the code

#include "esp_modem_usb_c_api.h"
#include "esp_modem_usb_config.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

extern EventGroupHandle_t event_group;
static const char *TAG = "network_dce";

static void usb_terminal_error_handler(esp_modem_terminal_error_t err)
{
    if (err == ESP_MODEM_TERMINAL_DEVICE_GONE)
    {
        ESP_LOGI(TAG, "USB modem disconnected");
        assert(event_group);
        xEventGroupSetBits(event_group, USB_DISCONNECTED_BIT);
    }
}
#define CHECK_USB_DISCONNECTION(event_group)                                              \
    if ((xEventGroupGetBits(event_group) & USB_DISCONNECTED_BIT) == USB_DISCONNECTED_BIT) \
    {                                                                                     \
        esp_modem_destroy(dce);                                                           \
        continue;                                                                         \
    }

static esp_modem_dce_t *dce = NULL;

esp_err_t modem_init_network(esp_netif_t *netif)
{
    // setup the DCE
    esp_modem_dce_config_t dce_config = ESP_MODEM_DCE_DEFAULT_CONFIG(CONFIG_EXAMPLE_MODEM_PPP_APN);

    ESP_LOGI(TAG, "Initializing esp_modem for the BG95 module...");
    struct esp_modem_usb_term_config usb_config = ESP_MODEM_DEFAULT_USB_CONFIG(0x2C7C, 0x0700, 2);
    esp_modem_dce_device_t usb_dev_type = ESP_MODEM_DCE_BG96;

    const esp_modem_dte_config_t dte_usb_config = ESP_MODEM_DTE_DEFAULT_USB_CONFIG(usb_config);
    ESP_LOGI(TAG, "Waiting for USB device connection...");
    dce = esp_modem_new_dev_usb(usb_dev_type, &dte_usb_config, &dce_config, netif);
    assert(dce);
    esp_modem_set_error_cb(dce, usb_terminal_error_handler);
    ESP_LOGI(TAG, "Modem connected, waiting 10 seconds for boot...");
    vTaskDelay(pdMS_TO_TICKS(10000)); // Give DTE some time to boot

    xEventGroupClearBits(event_group, CONNECT_BIT | GOT_DATA_BIT | USB_DISCONNECTED_BIT);

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
        ESP_LOGI(TAG, "Signal quality: rssi=%d, ber=%d", rssi, ber);
        return rssi != 99 && rssi > 5;
    }
    return false;
}
