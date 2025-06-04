#include "esp_log.h"
#include "host/ble_hs.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string>
#include <cstring>

#include "ble_gatt.h"
#include "drive_controller.h"
#include "settings_controller.h"
#include "inputs_controller.h"

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
    ESP_LOG_BUFFER_HEX("DATA", data, len);
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
                DriveController::instance().stopStreamingTelemetry();
            } else if (currentScreen == "telemetry") {
                DriveController::instance().stopStreamingTelemetry();
            } else {
                DriveController::instance().stopStreamingTelemetry();
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
        const uint8_t* data = ctxt->om->om_data;
        size_t len = ctxt->om->om_len;

        if (currentScreen == "telemetry") {
            if (data[0] == MSG_TYPE_COMMAND && data[1] == CMD_TELEMETRY_PAUSE) {
                ESP_LOGI(TAG, "pausetelemetry");
                DriveController::instance().stopStreamingTelemetry();
            } else if (data[0] == MSG_TYPE_COMMAND && data[1] == CMD_TELEMETRY_RESUME) {
                ESP_LOGI(TAG, "resumetelemetry");
                DriveController::instance().startStreamingTelemetry();
            }
        } else if (currentScreen == "settings") {
            if (data[0] == MSG_TYPE_COMMAND && data[1] == CMD_SETTINGS_LOAD) {
                ESP_LOGI(TAG, "Received Command : loadSetting");
                uint8_t buffer[128];
                size_t length;
                SettingsController::instance().encode(buffer, length);
                send_notify_to_app(buffer, length);
            }
            else if (data[0] == MSG_TYPE_COMMAND && data[1] == CMD_SETTINGS_RESET) {
                ESP_LOGI(TAG, "Received Command : reset Settings");                
                SettingsController::instance().clearAllSettings();
            }
            else if (data[0] == MSG_TYPE_DATA && data[1] == DATA_TYPE_SETTINGS) {
                ESP_LOG_BUFFER_HEX("SETTINGS", data, len);
                ESP_LOGI(TAG, "Received Command : save Settings");                      
                SettingsController::instance().decode(data, len);
                ESP_LOGI(TAG, "rcName: %s", SettingsController::instance().get().rcName.c_str());
                SettingsController::instance().save();
            }
        } else if (currentScreen == "control") {
            if (data[0] == MSG_TYPE_COMMAND && data[1] == CMD_CONTROL_OVERRIDE) {
                InputsController::instance().enableOverrideControl();
            }
            else if (data[0] == MSG_TYPE_COMMAND && data[1] == CMD_CONTROL_RELEASE) {
                InputsController::instance().disableOverrideControl();
            }            
            else if (data[0] == MSG_TYPE_DATA && data[1] == DATA_TYPE_CONTROL) {
                InputsController::instance().setControlInputs(data, len);
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
