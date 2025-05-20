#include "wifi.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <cstring>

namespace Wifi {

static const char* TAG = "WifiInit";

static void initWifiBase() {
    static bool initialized = false;
    if (initialized) return;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    initialized = true;
}

void startSoftAP(const std::string& ssid, const std::string& password) {
    initWifiBase();

    wifi_config_t config = {};
    std::strncpy((char*)config.ap.ssid, ssid.c_str(), sizeof(config.ap.ssid));
    std::strncpy((char*)config.ap.password, password.c_str(), sizeof(config.ap.password));
    config.ap.ssid_len = ssid.length();
    config.ap.max_connection = 4;
    config.ap.authmode = password.empty() ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Point d'accès lancé : %s (%s)", ssid.c_str(), password.c_str());
}

void startStation(const std::string& ssid, const std::string& password) {
    initWifiBase();

    wifi_config_t config = {};
    std::strncpy((char*)config.sta.ssid, ssid.c_str(), sizeof(config.sta.ssid));
    std::strncpy((char*)config.sta.password, password.c_str(), sizeof(config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connexion au réseau : %s", ssid.c_str());
}

}  // namespace Wifi
