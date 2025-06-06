
#pragma once

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <string.h>

#include "ble_gatt.h"
#include "tlv_utils.h"

#define NUM_CHANNELS 4

#define PWM_MIN 1000
#define PWM_MID 1500
#define PWM_MAX 2000

class InputsController {
public:
    static InputsController& instance();

    gpio_num_t chPins[NUM_CHANNELS] = {GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4};
    int16_t pulseWidth[NUM_CHANNELS] = {PWM_MID, PWM_MID, PWM_MID, PWM_MID};
    int pulseStart[NUM_CHANNELS] = {0};
    bool pulseStarted[NUM_CHANNELS] = {false};

    int8_t getValueForChan(uint8_t ch);
    int16_t getRawValueForChan(uint8_t ch);
    
    void start();

    void enableOverrideControl();
    void disableOverrideControl();    

    void setControlInputs(const uint8_t* in, size_t len);

    void startStreamingTelemetry();
    void stopStreamingTelemetry();

private:
    InputsController() = default;

    int8_t pwmToPercent(int16_t pwm);
    int16_t percentToPwm(int8_t percent);

    bool overrideControl = false;
    int16_t overridedChan1 = 1500;
    int16_t overridedChan2 = 1500;

    void telemetryTaskLoop();
    static void telemetryTaskEntry(void*);
    TaskHandle_t telemetryTaskHandle = nullptr;
    bool streamingTelemetry = false;    
};



