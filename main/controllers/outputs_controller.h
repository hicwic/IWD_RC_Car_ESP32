#pragma once

#include "Arduino.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/ledc.h"

#include <string.h>

#include "DShotRMT.h"

#include "settings_controller.h"
#include "ble_gatt.h"
#include "tlv_utils.h"


#define ESC_CENTRAL_GPIO GPIO_NUM_8

#define ESC_REAR_RIGHT_GPIO GPIO_NUM_13
#define ESC_REAR_LEFT_GPIO GPIO_NUM_12

#define ESC_FRONT_RIGHT_GPIO GPIO_NUM_11
#define ESC_FRONT_LEFT_GPIO GPIO_NUM_10

#define SERVO_GPIO GPIO_NUM_9


constexpr uint32_t PWM_RESOLUTION_BITS = 13;
constexpr uint32_t PWM_DUTY_MAX = (1 << PWM_RESOLUTION_BITS) - 1;  // 8191
constexpr uint32_t PWM_PERIOD_US = 20000; // 50 Hz

struct MotorConfig {
    uint8_t mode;
    ledc_channel_t pwmChannel;
    DShotRMT* dshot;
};

class OutputsController {
public:
    static OutputsController& instance();

    volatile float valueCentral = 0;
    volatile float valueRearLeft = 0;
    volatile float valueRearRight = 0;
    volatile float valueFrontLeft = 0;
    volatile float valueFrontRight = 0;

    volatile int16_t valueServoRaw = 0;

    void start();
    void restart();
    void stop();


    void startStreamingTelemetry();
    void stopStreamingTelemetry();


private:
    OutputsController() = default;

    DShotRMT* centralESC = nullptr;
    DShotRMT* leftFrontESC = nullptr;
    DShotRMT* rightFrontESC = nullptr;
    DShotRMT* leftRearESC = nullptr;
    DShotRMT* rightRearESC = nullptr;

    int normalizedToDShotCommand(float normalized);
    int normalizedToPwmUs(float normalized);

    void initMotor(uint8_t mode, ledc_channel_t channel, gpio_num_t gpio, DShotRMT*& esc);
    void setMotorOutput(const MotorConfig& cfg, float normalized);
    void pwmInitChannel(ledc_channel_t channel, gpio_num_t gpio);
    void setPWMDutyUs(ledc_channel_t channel, int us);

    void updateTaskLoop();
    TaskHandle_t updateTaskHandle = nullptr;

    void telemetryTaskLoop();
    static void telemetryTaskEntry(void*);
    TaskHandle_t telemetryTaskHandle = nullptr;
    bool streamingTelemetry = false;

};



