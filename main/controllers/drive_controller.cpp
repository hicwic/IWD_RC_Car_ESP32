#include <algorithm>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_pm.h"


#include "ble_gatt.h"
#include "tlv_utils.h"

#include "drive_controller.h"
#include "settings_controller.h"
#include "inputs_controller.h"
#include "outputs_controller.h"

#define TAG "DRIVECONTROLLER"


#define DEADZONE 10
#define MAX_MIX 50
#define MIN_THROTTLE_FWD 10
#define NEUTRAL_DELAY_MS 500
#define INVERT_STEERING_LOGIC false //used to invert differential steering logic (used if steering channel need to be reversed on radio)

#define LOOP_DELAY_MS 10

#define RAMP_STEPS 6
#define RAMP_INTERVAL_MS 15


DriveController& DriveController::instance() {
    static DriveController inst;
    return inst;
}

void DriveController::update(const motion_data_t& newData) {
    std::lock_guard<std::mutex> lock(mutex);
    data = newData;
}

motion_data_t DriveController::get() {
    std::lock_guard<std::mutex> lock(mutex);
    return data;
}

void DriveController::startStreamingTelemetry() {
    streamingTelemetry = true;
}

void DriveController::stopStreamingTelemetry() {
    streamingTelemetry = false;
}

void DriveController::start() {
    xTaskCreatePinnedToCore([](void *param) {
        DriveController::instance().controllerTaskLoop();
    }, "controller_task", 4096, nullptr, 1, &this->controllerTaskHandle, 1);

    xTaskCreatePinnedToCore([](void *param) {
        DriveController::instance().telemetryTaskLoop();
    }, "telemetry_task", 4096, nullptr, 1, &this->telemetryTaskHandle, 1);
}

void DriveController::telemetryTaskLoop() {
    while (true) {
        if (streamingTelemetry) {
            auto snapshot = get(); // thread-safe

            uint8_t buffer[64];
            TLVWriter writer(buffer);
            writer.begin(MSG_TYPE_DATA, DATA_TYPE_TELEMETRY_MOTION);

            writer.addFloat(0x01, snapshot.target_speed);      // ts
            writer.addFloat(0x02, snapshot.current_speed);     // cs

            const auto& settings = SettingsController::instance().get();
            if (settings.driveMode == DRIVE_MODE_XCWD) {
                writer.addFloat(0x0B, snapshot.central_motor);  // fr   
            } else {
                if (settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_FIWD) {
                    writer.addFloat(0x09, snapshot.front_left_motor);   // fl
                    writer.addFloat(0x0A, snapshot.front_right_motor);  // fr
                }

                if (settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_RIWD) {
                    writer.addFloat(0x03, snapshot.rear_left_motor);   // rl
                    writer.addFloat(0x04, snapshot.rear_right_motor);  // rr                    
                }
            }

            writer.addBool(0x05, snapshot.reverse);             // rev
            writer.addBool(0x06, snapshot.ready_for_reverse);   // revOK
            writer.addBool(0x07, snapshot.ramping);             // ramp
            writer.addBool(0x08, snapshot.coasting);            // coast

            BLE::send_notify_to_app(buffer, writer.length());
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void DriveController::controllerTaskLoop() {
//     float baseVelocity = 0;
//     float targetVelocity = 0;
    float rampTargetVelocity = 0;
    float rampStep = 0;
    int rampStepCount = 0;
    bool ramping = false;

//     int velocityReductionRate = 20;
//     bool reverseMode = false;
//     bool readyForReverse = false;
//     bool coasting = false;

    uint64_t neutralStartTime = 0;
    uint64_t lastRampTime = 0;
    uint64_t lastLoopTime = esp_timer_get_time();
    
    while (1) {
        uint64_t now = esp_timer_get_time();
        float deltaTimeS = (now - lastLoopTime) / 1000.0 / 1000.0;
        lastLoopTime = now;

        // Lecture RC
        int dirPercent = InputsController::instance().getValueForChan(1);
        int throttlePercent = InputsController::instance().getValueForChan(2);;

        const int mixSign = INVERT_STEERING_LOGIC ? -1 : 1;

        // === THROTTLE LOGIC ===
        //Moving foward, we set target velocity to throttle. Reverse is reinit
        if (throttlePercent > DEADZONE) {
            data.reverse = false;
            data.ready_for_reverse = false;
            neutralStartTime = 0;
            data.target_speed = throttlePercent;
        } 
        //Deadzone, targetVelocity is 0. We start the neutral timer (time before we can reverse)
        else if (throttlePercent >= -DEADZONE && throttlePercent <= DEADZONE) {
            data.reverse = false;
            data.ramping = false;
            data.target_speed = 0;

            bool isNeutral = fabs(data.current_speed) < 1.0;
            if (isNeutral) {
                if (neutralStartTime == 0) neutralStartTime = now;
                else if (now - neutralStartTime >= NEUTRAL_DELAY_MS) data.ready_for_reverse = true;
            } else {
                neutralStartTime = 0;
                data.ready_for_reverse = false;
            }
        } 
        else {
            // we brake if pushing trigger, and not ready for reverse. TargetVelocity to 0, cancel ramping.
            if (!data.ready_for_reverse) {
                data.reverse = false;
                data.ramping = false;
                data.current_speed = 0;
                data.target_speed = 0;
            } 
            // if ready for reverse, we set targetVelocity to throttle (must be < 0)
            else {
                data.reverse = true;
                data.target_speed = throttlePercent;
            }
        }

        // === COASTING LOGIC ===
        data.coasting = false;
        if (abs(data.current_speed) > abs(data.target_speed) && !ramping) {
            int direction = data.current_speed > 0 ? 1 : -1;
            data.current_speed = direction * std::max(abs(data.current_speed)-SettingsController::instance().get().coastingFactor*deltaTimeS, abs(data.target_speed));  
            data.coasting = true;
        }

        // === ACCELERATION LOGIC ===
        if (fabs(data.target_speed) > 0 && !ramping) {
            //if the base velocity is 0 and we accelerate start ramping
            if (data.current_speed == 0) {
                ramping = true;
                rampTargetVelocity = data.target_speed;
                rampStepCount = 0;
                rampStep = rampTargetVelocity / RAMP_STEPS;
                data.current_speed = 0;
                lastRampTime = now;
            } 
            //standard acceleration from non zero base velocity to target velocity
            else if (abs(data.current_speed) < abs(data.target_speed)) {
                data.current_speed = data.target_speed;
            }
        }

        // === RAMPING LOGIC ===
        if (ramping) {
            //cancel ramping if deccelerate
            if (fabs(data.target_speed) < fabs(rampTargetVelocity)) {
                ramping = false;
            } else if (now - lastRampTime >= RAMP_INTERVAL_MS) {
                //if data.target_speed is higher and ramping, then we recalculate steps and update rampTargetVelocity (occur while multiple read while pulling throttle trigger)
                if (fabs(data.target_speed) > fabs(rampTargetVelocity)) {
                    int direction = data.target_speed > 0 ? 1 : -1;
                    rampStep = direction * (fabs(data.target_speed) - fabs(data.current_speed)) / (RAMP_STEPS - rampStepCount);
                    rampTargetVelocity = data.target_speed;
                }

                data.current_speed += rampStep;
                rampStepCount++;
                lastRampTime = now;

                //once reached the num au ramping steps we conclude ramping
                if (rampStepCount >= RAMP_STEPS) {
                    data.current_speed = rampTargetVelocity;
                    ramping = false;
                }
            }
        }

        // === DIFFERENTIAL ===
        float leftVelocity = data.current_speed;
        float rightVelocity = data.current_speed;

        if (!ramping) {
            float diff = dirPercent * SettingsController::instance().get().rearDiffFactor / 100.0;
            leftVelocity  = data.current_speed - mixSign * diff;
            rightVelocity = data.current_speed + mixSign * diff;
        }

        // Min. throttle fwd/rev
        if (data.current_speed > 0) {
            leftVelocity  = fmax(leftVelocity,  MIN_THROTTLE_FWD);
            rightVelocity = fmax(rightVelocity, MIN_THROTTLE_FWD);
        } else if (data.current_speed < 0) {
            leftVelocity  = fmin(leftVelocity,  -MIN_THROTTLE_FWD);
            rightVelocity = fmin(rightVelocity, -MIN_THROTTLE_FWD);
        } else {
            leftVelocity = 0;
            rightVelocity = 0;
        }

        // // DShot
        OutputsController::instance().valueRearLeft = leftVelocity;
        OutputsController::instance().valueRearRight = rightVelocity;


        float loopTime = (esp_timer_get_time() - now) / 1000.0;


        // // === LED BLINKING FEEDBACK ===
        // if (baseVelocity != 0.0) {
        //     int direction = baseVelocity > 0 ? 1 : -1;
        //     float frequencyHz = abs(baseVelocity) / 10.0; // between 0 et 10 Hz
        //     frequencyHz = constrain(frequencyHz, 0.5, 10.0); // avoid 0 Hz (divide by 0)
        //     float period = 1000.0 / frequencyHz; // convert to ms
        //     float halfPeriod = period / 2.0;

        //     if (millis() - lastBlinkTime >= halfPeriod) {
        //         ledState = !ledState;
        //         lastBlinkTime = millis();
        //     }

        //     if (ledState) {
        //         if (direction > 0) {
        //             pixel.setPixelColor(0, pixel.Color(0, 255, 0)); // move forward = blinking green
        //         } else {
        //             pixel.setPixelColor(0, pixel.Color(255, 255, 255)); // move backward = blinking white
        //         }
        //     } else {
        //         pixel.setPixelColor(0, pixel.Color(0, 0, 0)); // LED off
        //     }
        //     pixel.show();

        // } else {
        //     ledState = false;
        //     pixel.setPixelColor(0, pixel.Color(255, 0, 0)); // stopped = fix red
        //     pixel.show();
        // }


// #if DEBUG
        
        // ESP_LOGI(TAG, "Throttle:%d | Dir:%d | Mix:%d | VelReduc:%d | TargetVel:%.1f | BaseVel:%.1f | LVel:%.1f | RVel:%.1f | LDShot:%d | RDShot:%d | Rev:%d | RdyRev:%d | Ramping:%d | DelTime:%.3f | LoopTime:%.3f",
        //     throttlePercent, dirPercent, SettingsController::instance().get().rearDiffFactor, SettingsController::instance().get().coastingFactor, data.target_speed,
        //     data.current_speed, leftVelocity, rightVelocity, OutputsController::instance().dshotValueLeft, OutputsController::instance().dshotValueRight,
        //     data.reverse, data.ready_for_reverse, data.ramping, deltaTimeS, loopTime);
// #endif

        // // Update telemetry
        // telemetry_data_t t;
        // t.timestamp_ms = esp_timer_get_time() / 1000;
        // t.target_speed = targetVelocity;
        // t.current_speed = baseVelocity;
        // t.left_motor = leftVelocity;
        // t.right_motor = rightVelocity;
        // t.reverse = reverseMode?1:0;
        // t.ready_for_reverse = readyForReverse?1:0;
        // t.ramping = ramping?1:0;
        // t.coasting = coasting?1:0;

        // TelemetryManager::instance().update(t);


        vTaskDelay(pdMS_TO_TICKS(std::max<long>((LOOP_DELAY_MS-loopTime), 1)));
    }
}
