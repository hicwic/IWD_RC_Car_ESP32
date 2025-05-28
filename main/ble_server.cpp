#include "ble_server.h"
#include "ble_gatt_screen.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nvs_flash.h"



#define GATTS_TAG "BLE_SERVER"

namespace BLE {

static uint8_t ble_addr_type;

static void ble_app_advertise(void);

static int on_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(GATTS_TAG, "Client connected");
            } else {
                ESP_LOGW(GATTS_TAG, "Connection failed; restarting advertising");
                ble_app_advertise();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(GATTS_TAG, "Client disconnected; restarting advertising");
            ble_app_advertise();
            break;

        default:
            break;
    }
    return 0;
}

static void ble_app_on_sync(void) {
    int rc = ble_hs_id_infer_auto(0, &ble_addr_type);
    if (rc != 0) {
        ESP_LOGE(GATTS_TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }
    ble_app_advertise();
}

static void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params = {};
    struct ble_hs_adv_fields fields = {};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const char *name = "RC_CAR_Controller";
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER,
                      &adv_params, on_gap_event, NULL);

    ESP_LOGI(GATTS_TAG, "BLE advertising started (NimBLE C API)");
}

void init_ble_server() {
    // 1. Initialiser NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // 2. Init bluetooth stack
    ESP_ERROR_CHECK(nimble_port_init());

    // 3. Initialiser NimBLE HCI + Host stack
    ESP_ERROR_CHECK(esp_nimble_hci_init());


    // 4. Initialiser services GAP/GATT et assigner callbacks
    ble_svc_gap_init();
    ble_svc_gatt_init();

    BLE::register_screen_service();

    ble_hs_cfg.sync_cb = ble_app_on_sync;

    // 5. Lancer la tâche NimBLE
    nimble_port_freertos_init([](void *param) {
        nimble_port_run();
    });
}

} // namespace BLE
