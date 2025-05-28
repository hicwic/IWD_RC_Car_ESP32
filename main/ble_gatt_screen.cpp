#include "esp_log.h"
#include "host/ble_hs.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string>
#include <cstring>

#include "telemetry_data.h"
#include "settings.h"

#define TAG "BLE_SCREEN"

namespace BLE {

// UUIDs 128 bits déclarés proprement
// static ble_uuid128_t uuid_screen = BLE_UUID128_INIT(
//     0xCD, 0xAB, 0x00, 0x00,
//     0x10, 0x00, 0x80, 0x00,
//     0x00, 0x80, 0x5F, 0x9B,
//     0x34, 0xFB, 0x00, 0x00
// );

// static ble_uuid128_t uuid_command = BLE_UUID128_INIT(
//     0xBA, 0xDC, 0x00, 0x00,
//     0x10, 0x00, 0x80, 0x00,
//     0x00, 0x80, 0x5F, 0x9B,
//     0x34, 0xFB, 0x00, 0x00
// );

static const ble_uuid128_t uuid_screen =
    BLE_UUID128_INIT(0x00, 0x00, 0x00, 0x00, 0x11, 0x11, 0x11, 0x11,
                     0x22, 0x22, 0x22, 0x22, 0x33, 0x33, 0x33, 0x33);

static const ble_uuid128_t uuid_command =
    BLE_UUID128_INIT(0x01, 0x01, 0x01, 0x01, 0x12, 0x12, 0x12, 0x12,
                     0x23, 0x23, 0x23, 0x23, 0x34, 0x34, 0x34, 0x34);


static std::string currentScreen = "unknown";
static uint16_t command_conn_handle = 0;
static uint16_t command_attr_handle = 0;


// static ble_uuid16_t uuid_service = BLE_UUID16_INIT(0x180A);
static const ble_uuid128_t uuid_service =
    BLE_UUID128_INIT(0x2d, 0x71, 0xa2, 0x59, 0xb4, 0x58, 0xc8, 0x12,
                     0x99, 0x99, 0x43, 0x95, 0x12, 0x2f, 0x46, 0x59);




void send_notify_to_app(const char* data) {
    ESP_LOGI(TAG, "tagada");
    if (command_conn_handle != 0) {
        struct os_mbuf* om = ble_hs_mbuf_from_flat(data, strlen(data));
        ble_gatts_notify_custom(command_conn_handle, command_attr_handle, om);
        ESP_LOGI(TAG, "Notified: %s", data);
    }
}

void send_notify_to_app(const uint8_t* data, size_t len) {
    ESP_LOGI(TAG, "youpla");    
    if (command_conn_handle != 0) {
        struct os_mbuf* om = ble_hs_mbuf_from_flat(data, len);
        ble_gatts_notify_custom(command_conn_handle, command_attr_handle, om);
        ESP_LOGI(TAG, "Notified: %s", data);
    }
}

int screen_char_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {

    command_conn_handle = conn_handle;
    command_attr_handle = attr_handle;

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_WRITE_CHR: {
            char buffer[20] = {};
            memcpy(buffer, ctxt->om->om_data, std::min(ctxt->om->om_len, static_cast<uint16_t>(sizeof(buffer) - 1)));
            currentScreen = std::string(buffer);
            ESP_LOGI(TAG, "Écran actif: %s", currentScreen.c_str());

            if (currentScreen == "settings") {
                TelemetryManager::instance().setStreaming(false);
            } else if (currentScreen == "telemetry") {
                TelemetryManager::instance().setStreaming(false);
            } else {
                TelemetryManager::instance().setStreaming(false);
            }
            break;
        }
        case BLE_GATT_ACCESS_OP_READ_CHR:
            os_mbuf_append(ctxt->om, currentScreen.c_str(), currentScreen.length());
            break;
    }
    return 0;
}

int command_char_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg) {

    command_conn_handle = conn_handle;
    command_attr_handle = attr_handle;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        char cmd[50] = {};
        memcpy(cmd, ctxt->om->om_data, std::min(ctxt->om->om_len, static_cast<uint16_t>(sizeof(cmd) - 1)));
        ESP_LOGI(TAG, "[%s] Received: %s", currentScreen.c_str(), cmd);

        if (currentScreen == "telemetry") {
            if (strcmp(cmd, "pausetelemetry") == 0) {
                ESP_LOGI(TAG, "pausetelemetry");
                TelemetryManager::instance().setStreaming(false);
            } else if (strcmp(cmd, "resumetelemetry") == 0) {
                ESP_LOGI(TAG, "resumetelemetry");
                TelemetryManager::instance().setStreaming(true);
            }
        } else if (currentScreen == "settings") {
            if (strcmp(cmd, "loadSettings") == 0) {
                uint8_t out[8];
                SettingsManager::instance().encode(out);
                send_notify_to_app(out, 8);
            }
            else if (reinterpret_cast<const uint8_t*>(cmd)[0] == 0x01) {
                SettingsManager::instance().decode(reinterpret_cast<const uint8_t*>(cmd));
                SettingsManager::instance().save();
            }
        }
    }

    return 0;
}

static const struct ble_gatt_svc_def screen_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &uuid_service.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &uuid_screen.u,
                .access_cb = screen_char_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &uuid_command.u,
                .access_cb = command_char_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }
        }
    },
    { 0 }
};


void register_screen_service() {


    ESP_ERROR_CHECK(ble_gatts_count_cfg(screen_services));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(screen_services));
}

} // namespace BLE
