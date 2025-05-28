#pragma once

#include <mutex>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    float target_speed;
    float current_speed;
    float left_motor;
    float right_motor;

    uint8_t reverse;
    uint8_t ready_for_reverse;
    uint8_t ramping;
    uint8_t coasting;
} telemetry_data_t;

class TelemetryManager {
public:
    static TelemetryManager& instance();

    void update(const telemetry_data_t& newData);
    telemetry_data_t get();

    void start();
    void setStreaming(bool enabled);

private:
    TelemetryManager() = default;

    void telemetryTaskLoop();

    telemetry_data_t data{};
    std::mutex mutex;

    bool streaming = false;
    TaskHandle_t telemetryTaskHandle = nullptr;
};
