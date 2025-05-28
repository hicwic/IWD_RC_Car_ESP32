#include "telemetry_data.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble_gatt_screen.h"

#define TAG "TELEMETRY"

TelemetryManager& TelemetryManager::instance() {
    static TelemetryManager inst;
    return inst;
}

void TelemetryManager::update(const telemetry_data_t& newData) {
    std::lock_guard<std::mutex> lock(mutex);
    data = newData;
}

telemetry_data_t TelemetryManager::get() {
    std::lock_guard<std::mutex> lock(mutex);
    return data;
}

void TelemetryManager::setStreaming(bool enabled) {
    streaming = enabled;
}

void TelemetryManager::start() {
    xTaskCreatePinnedToCore([](void *param) {
        TelemetryManager::instance().telemetryTaskLoop();
    }, "telemetry_task", 4096, nullptr, 1, &this->telemetryTaskHandle, 1);
}

void TelemetryManager::telemetryTaskLoop() {
    while (true) {
        if (streaming) {
            auto snapshot = get(); // thread-safe

            char buf[128];
            snprintf(buf, sizeof(buf),
                     "ts:%.2f,cs:%.2f,rl:%.2f,rr:%.2f,rev:%d,revOK:%d,ramp:%d,coast:%d",
                     snapshot.target_speed,
                     snapshot.current_speed,
                     snapshot.left_motor,
                     snapshot.right_motor,
                     snapshot.reverse,
                     snapshot.ready_for_reverse,
                     snapshot.ramping,
                     snapshot.coasting);

            BLE::send_notify_to_app(buf);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
