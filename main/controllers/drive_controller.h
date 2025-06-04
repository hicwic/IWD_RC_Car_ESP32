#pragma once

#include <mutex>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;

    float target_speed;
    float current_speed;

    float central_motor;
    float front_left_motor;
    float front_right_motor;
    float rear_left_motor;
    float rear_right_motor;

    uint8_t reverse;
    uint8_t ready_for_reverse;
    uint8_t ramping;
    uint8_t coasting;
} motion_data_t;

class DriveController {
public:
    static DriveController& instance();

    void update(const motion_data_t& newData);
    motion_data_t get();

    void start();
    void startStreamingTelemetry();
    void stopStreamingTelemetry();
       

private:
    DriveController() = default;

    void telemetryTaskLoop();
    void controllerTaskLoop();

    motion_data_t data{};
    std::mutex mutex;

    bool streamingTelemetry = false;
    TaskHandle_t telemetryTaskHandle = nullptr;
    TaskHandle_t controllerTaskHandle = nullptr;
};
