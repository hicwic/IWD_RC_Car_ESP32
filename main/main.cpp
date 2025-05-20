#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <cstring>

#include "Arduino.h"
#include "Adafruit_NeoPixel.h"
#include "DShotRMT.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_pm.h"

// === CONFIG ===
#define TAG "RC_ESC"
#define LOG_LEVEL ESP_LOG_INFO

#define NUM_CHANNELS 4
const gpio_num_t chPins[NUM_CHANNELS] = {GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3, GPIO_NUM_4};

#define ESC_LEFT_GPIO GPIO_NUM_12
#define ESC_RIGHT_GPIO GPIO_NUM_13

#define PWM_MIN 1000
#define PWM_MID 1500
#define PWM_MAX 2000

#define DEADZONE 10
#define MAX_MIX 50
#define MIN_THROTTLE_FWD 10
#define NEUTRAL_DELAY_MS 500
#define INVERT_STEERING_LOGIC false //used to invert differential steering logic (used if steering channel need to be reversed on radio)

#define LOOP_DELAY_MS 10

#define RAMP_STEPS 6
#define RAMP_INTERVAL_MS 15

#define LED_PIN    21      // Embedded RGB Led (ESP32-s3 Zero)
#define LED_COUNT  1

// Var for using led as indicator
Adafruit_NeoPixel pixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Vars for PWM input
volatile int pulseWidth[NUM_CHANNELS] = {PWM_MID, PWM_MID, PWM_MID, PWM_MID};
volatile int pulseStart[NUM_CHANNELS] = {0};
volatile bool pulseStarted[NUM_CHANNELS] = {false};

// ESCs variables
DShotRMT leftESC(ESC_LEFT_GPIO);
DShotRMT rightESC(ESC_RIGHT_GPIO);

volatile uint16_t dshotValueLeft = 0;
volatile uint16_t dshotValueRight = 0;

// power management
esp_pm_lock_handle_t power_lock;


// === Utils ===
int pwmToPercent(int pwm) {
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    return ((pwm - PWM_MID) * 100) / (PWM_MAX - PWM_MID);
}

int velocityToDShotCommand(int velocity) {
    // Mode 3D : 0 = stop, 48..1047 = reverse, 1049..2047 = forward
    if (velocity == 0) return 0;

    if (velocity > 0) {
        int scaled = 1049 + (velocity * (2047 - 1049)) / 100;
        if (scaled > 2047) scaled = 2047;
        return scaled;
    } else {
        int scaled = 48 + (abs(velocity) * (1047 - 48)) / 100;
        if (scaled > 1047) scaled = 1047;
        return scaled;
    }
}

// === ISR read PWM ===
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    int ch = (int)arg;
    int level = gpio_get_level(chPins[ch]);
    int64_t now = esp_timer_get_time();

    if(level == 1) {
        pulseStart[ch] = now;
        pulseStarted[ch] = true;
    } else {
        if(pulseStarted[ch]) {
            int width = now - pulseStart[ch];
            if (width >= 500 && width <= 2500) {
                pulseWidth[ch] = width;
            } else {
                pulseWidth[ch] = PWM_MID;
            }
            pulseStarted[ch] = false;
        }
    }
}

void dshotTask(void *pvParameters) {
    while (true) {
        leftESC.send_dshot_value(dshotValueLeft);
        rightESC.send_dshot_value(dshotValueRight);
        vTaskDelay(pdMS_TO_TICKS(5));  // 5 ms = 200 Hz
    }
}

void control_task(void* arg) {
    float baseVelocity = 0;
    float targetVelocity = 0;
    float rampTargetVelocity = 0;
    float rampStep = 0;
    int rampStepCount = 0;
    bool ramping = false;

    // LED status blinking
    unsigned long lastBlinkTime = 0;
    bool ledState = false;

    int velocityReductionRate = 20;
    bool reverseMode = false;
    bool readyForReverse = false;

    uint64_t neutralStartTime = 0;
    uint64_t lastRampTime = 0;
    uint64_t lastLoopTime = esp_timer_get_time();
    
    while (1) {
        uint64_t now = esp_timer_get_time();
        float deltaTimeS = (now - lastLoopTime) / 1000.0 / 1000.0;
        lastLoopTime = now;

        // Lecture RC
        int dirPWM = pulseWidth[0];
        int throttlePWM = pulseWidth[1];
        int mixPWM = pulseWidth[2];
        int decayPWM = pulseWidth[3];

        int dirPercent = pwmToPercent(dirPWM);
        int throttlePercent = pwmToPercent(throttlePWM);
        int mixPercent = map(mixPWM, PWM_MIN, PWM_MAX, 0, MAX_MIX);
        velocityReductionRate = map(decayPWM, PWM_MIN, PWM_MAX, 20, 100);

        const int mixSign = INVERT_STEERING_LOGIC ? -1 : 1;

        // === THROTTLE LOGIC ===
        //Moving foward, we set target velocity to throttle. Reverse is reinit
        if (throttlePercent > DEADZONE) {
            reverseMode = false;
            readyForReverse = false;
            neutralStartTime = 0;
            targetVelocity = throttlePercent;
        } 
        //Deadzone, targetVelocity is 0. We start the neutral timer (time before we can reverse)
        else if (throttlePercent >= -DEADZONE && throttlePercent <= DEADZONE) {
            reverseMode = false;
            ramping = false;
            targetVelocity = 0;

            bool isNeutral = fabs(baseVelocity) < 1.0;
            if (isNeutral) {
                if (neutralStartTime == 0) neutralStartTime = now;
                else if (now - neutralStartTime >= NEUTRAL_DELAY_MS) readyForReverse = true;
            } else {
                neutralStartTime = 0;
                readyForReverse = false;
            }
        } 
        else {
            // we brake if pushing trigger, and not ready for reverse. TargetVelocity to 0, cancel ramping.
            if (!readyForReverse) {
                reverseMode = false;
                ramping = false;
                baseVelocity = 0;
                targetVelocity = 0;
            } 
            // if ready for reverse, we set targetVelocity to throttle (must be < 0)
            else {
                reverseMode = true;
                targetVelocity = throttlePercent;
            }
        }

        // === COASTING LOGIC ===
        if (abs(baseVelocity) > abs(targetVelocity) && !ramping) {
            int direction = baseVelocity > 0 ? 1 : -1;
            baseVelocity = direction * std::max(abs(baseVelocity)-velocityReductionRate*deltaTimeS, abs(targetVelocity));  
        }

        // === ACCELERATION LOGIC ===
        if (fabs(targetVelocity) > 0 && !ramping) {
            //if the base velocity is 0 and we accelerate start ramping
            if (baseVelocity == 0) {
                ramping = true;
                rampTargetVelocity = targetVelocity;
                rampStepCount = 0;
                rampStep = rampTargetVelocity / RAMP_STEPS;
                baseVelocity = 0;
                lastRampTime = now;
            } 
            //standard acceleration from non zero base velocity to target velocity
            else if (abs(baseVelocity) < abs(targetVelocity)) {
                baseVelocity = targetVelocity;
            }
        }

        // === RAMPING LOGIC ===
        if (ramping) {
            //cancel ramping if deccelerate
            if (fabs(targetVelocity) < fabs(rampTargetVelocity)) {
                ramping = false;
            } else if (now - lastRampTime >= RAMP_INTERVAL_MS) {
                //if targetVelocity is higher and ramping, then we recalculate steps and update rampTargetVelocity (occur while multiple read while pulling throttle trigger)
                if (fabs(targetVelocity) > fabs(rampTargetVelocity)) {
                    int direction = targetVelocity > 0 ? 1 : -1;
                    rampStep = direction * (fabs(targetVelocity) - fabs(baseVelocity)) / (RAMP_STEPS - rampStepCount);
                    rampTargetVelocity = targetVelocity;
                }

                baseVelocity += rampStep;
                rampStepCount++;
                lastRampTime = now;

                //once reached the num au ramping steps we conclude ramping
                if (rampStepCount >= RAMP_STEPS) {
                    baseVelocity = rampTargetVelocity;
                    ramping = false;
                }
            }
        }

        // === DIFFERENTIAL ===
        float leftVelocity = baseVelocity;
        float rightVelocity = baseVelocity;

        if (!ramping) {
            float diff = dirPercent * mixPercent / 100.0;
            leftVelocity  = baseVelocity - mixSign * diff;
            rightVelocity = baseVelocity + mixSign * diff;
        }

        // Min. throttle fwd/rev
        if (baseVelocity > 0) {
            leftVelocity  = fmax(leftVelocity,  MIN_THROTTLE_FWD);
            rightVelocity = fmax(rightVelocity, MIN_THROTTLE_FWD);
        } else if (baseVelocity < 0) {
            leftVelocity  = fmin(leftVelocity,  -MIN_THROTTLE_FWD);
            rightVelocity = fmin(rightVelocity, -MIN_THROTTLE_FWD);
        } else {
            leftVelocity = 0;
            rightVelocity = 0;
        }

        // DShot
        dshotValueLeft = velocityToDShotCommand(leftVelocity);
        dshotValueRight = velocityToDShotCommand(rightVelocity);


        float loopTime = (esp_timer_get_time() - now) / 1000.0;


        // === LED BLINKING FEEDBACK ===
        if (baseVelocity != 0.0) {
            int direction = baseVelocity > 0 ? 1 : -1;
            float frequencyHz = abs(baseVelocity) / 10.0; // between 0 et 10 Hz
            frequencyHz = constrain(frequencyHz, 0.5, 10.0); // avoid 0 Hz (divide by 0)
            float period = 1000.0 / frequencyHz; // convert to ms
            float halfPeriod = period / 2.0;

            if (millis() - lastBlinkTime >= halfPeriod) {
                ledState = !ledState;
                lastBlinkTime = millis();
            }

            if (ledState) {
                if (direction > 0) {
                    pixel.setPixelColor(0, pixel.Color(0, 255, 0)); // move forward = blinking green
                } else {
                    pixel.setPixelColor(0, pixel.Color(255, 255, 255)); // move backward = blinking white
                }
            } else {
                pixel.setPixelColor(0, pixel.Color(0, 0, 0)); // LED off
            }
            pixel.show();

        } else {
            ledState = false;
            pixel.setPixelColor(0, pixel.Color(255, 0, 0)); // stopped = fix red
            pixel.show();
        }


        // DEBUG
        ESP_LOGI(TAG, "Throttle:%d | Dir:%d | Mix:%d | VelReduc:%d | TargetVel:%.1f | BaseVel:%.1f | LVel:%.1f | RVel:%.1f | LDShot:%d | RDShot:%d | Rev:%d | RdyRev:%d | Ramping:%d | DelTime:%.3f | LoopTime:%.3f",
            throttlePercent, dirPercent, mixPercent, velocityReductionRate, targetVelocity,
            baseVelocity, leftVelocity, rightVelocity, dshotValueLeft, dshotValueRight,
            reverseMode, readyForReverse, ramping, deltaTimeS, loopTime);


        vTaskDelay(pdMS_TO_TICKS(std::max<long>((LOOP_DELAY_MS-loopTime), 1)));
    }
}


// === Setup ===
extern "C" void app_main(void) {
    initArduino();

    esp_log_level_set(TAG, LOG_LEVEL);

    //create power lock to prevent ESP to go in light sleep (cannot handle leds in light sleep)
    esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "no_light_sleep", &power_lock);
    esp_pm_lock_acquire(power_lock);

    vTaskDelay(pdMS_TO_TICKS(2000));

    pixel.begin(); // Init Led
    pixel.setBrightness(50); 
    pixel.setPixelColor(0, pixel.Color(0, 0, 255));
    pixel.show();

    vTaskDelay(pdMS_TO_TICKS(1000));

    esp_log_level_set(TAG, LOG_LEVEL);

    // Config GPIO entrée + interruption
    for (int i = 0; i < NUM_CHANNELS; i++) {
        gpio_config_t io_conf;
        memset(&io_conf, 0, sizeof(io_conf));
        io_conf.intr_type = GPIO_INTR_ANYEDGE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = 1ULL << chPins[i];
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_config(&io_conf);
    }
    gpio_install_isr_service(0);
    for (int i = 0; i < NUM_CHANNELS; i++) {
        gpio_isr_handler_add(chPins[i], gpio_isr_handler, (void*)i);
    }


    // Init Dshot ESCs
    leftESC.begin(DSHOT300, NO_BIDIRECTION , 12);
    rightESC.begin(DSHOT300, NO_BIDIRECTION, 12);


    // Init with 0 value to arm ESCs
    dshotValueLeft = DSHOT_CMD_MOTOR_STOP;
    dshotValueRight = DSHOT_CMD_MOTOR_STOP;

    // Start Dshot update loop
    xTaskCreate(dshotTask, "dShot_task", 2048, NULL, 5, NULL);

    vTaskDelay(pdMS_TO_TICKS(5000));

    // Start control loop
    xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);

}