#pragma once

#include "Arduino.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/ledc.h"

#include <string.h>

#include "DShotRMT.h"

#include "inputs_controller.h"
#include "settings_controller.h"
#include "ble_gatt.h"
#include "tlv_utils.h"


#define ESC_CENTRAL_GPIO GPIO_NUM_13

#define ESC_REAR_RIGHT_GPIO GPIO_NUM_13
#define ESC_REAR_LEFT_GPIO GPIO_NUM_12

#define ESC_FRONT_RIGHT_GPIO GPIO_NUM_11
#define ESC_FRONT_LEFT_GPIO GPIO_NUM_10

#define SERVO_GPIO GPIO_NUM_7


constexpr uint32_t PWM_RESOLUTION_BITS = 13;
constexpr uint32_t PWM_DUTY_MAX = (1 << PWM_RESOLUTION_BITS) - 1;  // 8191
constexpr uint32_t PWM_PERIOD_US = 20000; // 50 Hz

struct MotorConfig {
    uint8_t mode;
    ledc_channel_t pwmChannel;
    DShotRMT* dshot;
};

enum MotorIndex {
    MOTOR_CENTRAL = 0,
    MOTOR_FRONT_LEFT,
    MOTOR_FRONT_RIGHT,
    MOTOR_REAR_LEFT,
    MOTOR_REAR_RIGHT,
    MOTOR_COUNT
};

class OutputsController {
public:
    static OutputsController& instance();

    uint32_t currentRPM[MOTOR_COUNT];

    volatile float valueCentral = 0;
    volatile float valueRearLeft = 0;
    volatile float valueRearRight = 0;
    volatile float valueFrontLeft = 0;
    volatile float valueFrontRight = 0;

    void start();
    void restart();
    void stop();


    void startStreamingTelemetry();
    void stopStreamingTelemetry();

    void dshotTick();

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
    void pwmInitChannel(ledc_channel_t channel, gpio_num_t gpio);
    void setPWMDutyUs(ledc_channel_t channel, int us);

    void setMotorDShotOutput(MotorIndex index, DShotRMT* esc, float normalized, uint8_t mode);
    void setMotorPWMOutput(ledc_channel_t channel, float normalized);

    void dshotTaskLoop();
    void pwmTaskLoop();

    TaskHandle_t dshotTaskHandle = nullptr;
    TaskHandle_t pwmTaskHandle = nullptr;    

    void telemetryTaskLoop();
    static void telemetryTaskEntry(void*);
    TaskHandle_t telemetryTaskHandle = nullptr;
    bool streamingTelemetry = false;

    void startDshotTimer();

};



