#include "outputs_controller.h"

OutputsController& OutputsController::instance() {
    static OutputsController inst;
    return inst;
}

int OutputsController::normalizedToDShotCommand(float normalized) {
    // Mode 3D : 0 = stop, 48..1047 = reverse, 1049..2047 = forward
    if (normalized == 0) return 0;

    if (normalized > 0) {
        int scaled = roundf(1049 + (normalized * (2047 - 1049)) / 100.0f);
        if (scaled > 2047) scaled = 2047;
        return scaled;
    } else {
        int scaled = roundf(48 + (abs(normalized) * (1047 - 48)) / 100.0f);
        if (scaled > 1047) scaled = 1047;
        return scaled;
    }
}

int OutputsController::normalizedToPwmUs(float normalized) {
    // PWM standard : 1000 µs (full reverse) → 1500 µs (stop) → 2000 µs (full forward)

    if (normalized > 100) normalized = 100;
    if (normalized < -100) normalized = -100;

    return roundf(1500 + (normalized * 500.0f) / 100.0f);
}


void OutputsController::initMotor(uint8_t mode, ledc_channel_t channel, gpio_num_t gpio, DShotRMT*& esc) {
    switch (mode) {
        case MOTOR_MODE_PWM:
            pwmInitChannel(channel, gpio);
            setPWMDutyUs(channel, 1500); // neutre
            break;

        case MOTOR_MODE_DSHOT:
        ESP_LOGI("ESC_INIT", "Init motor mode=%d on gpio=%d", mode, gpio);
            esc = new DShotRMT(gpio);
            esc->begin(DSHOT300, NO_BIDIRECTION, 12);
            break;

        case MOTOR_MODE_BDSHOT:
        ESP_LOGI("ESC_INIT", "Init motor mode=%d on gpio=%d", mode, gpio);
            esc = new DShotRMT(gpio);
            esc->begin(DSHOT300, ENABLE_BIDIRECTION, 12);
            break;
    }
}

void OutputsController::setMotorOutput(const MotorConfig& cfg, float normalized) {
    switch (cfg.mode) {
        case MOTOR_MODE_PWM:
            setPWMDutyUs(cfg.pwmChannel, normalizedToPwmUs(normalized));
            break;

        case MOTOR_MODE_DSHOT:
        case MOTOR_MODE_BDSHOT:
            if (cfg.dshot) {
                cfg.dshot->send_dshot_value(normalizedToDShotCommand(normalized));
            }
            break;
    }
}

void OutputsController::updateTaskLoop() {
    while (true) {
        const auto& settings = SettingsController::instance().get();

        // Update motors
        if (settings.driveMode == DRIVE_MODE_XCWD) {
            MotorConfig centralMotor {
                .mode = settings.centralDriveTrainType,
                .pwmChannel = LEDC_CHANNEL_0,
                .dshot = centralESC
            };
            setMotorOutput(centralMotor, valueCentral);

        } else {
            if (settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_FIWD) {
                MotorConfig frontLeftCfg  = { settings.frontDriveTrainType, LEDC_CHANNEL_0, leftFrontESC };
                MotorConfig frontRightCfg = { settings.frontDriveTrainType, LEDC_CHANNEL_1, rightFrontESC };
                
                setMotorOutput(frontLeftCfg, valueFrontLeft);
                setMotorOutput(frontRightCfg, valueFrontRight);
            }

            if (settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_RIWD) {
                MotorConfig rearLeftCfg  = { settings.rearDriveTrainType, LEDC_CHANNEL_2, leftRearESC };
                MotorConfig rearRightCfg = { settings.rearDriveTrainType, LEDC_CHANNEL_3, rightRearESC };
                
                setMotorOutput(rearLeftCfg, valueRearLeft);
                setMotorOutput(rearRightCfg, valueRearRight);
            }
        }

        // Update Servo from input
        setPWMDutyUs(LEDC_CHANNEL_4, InputsController::instance().getRawValueForChan(1) + settings.steeringTrim * 5);

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


// void OutputsController::pwmInitChannel(ledc_channel_t channel, gpio_num_t gpio) {
//     static bool timerInitialized = false;

//     if (!timerInitialized) {
//         ledc_timer_config_t timer_conf;
//         timer_conf.speed_mode       = LEDC_LOW_SPEED_MODE;
//         timer_conf.timer_num        = LEDC_TIMER_0;
//         timer_conf.duty_resolution  = LEDC_TIMER_13_BIT;
//         timer_conf.freq_hz          = 50;
//         timer_conf.clk_cfg          = LEDC_AUTO_CLK;
// #if ESP_IDF_VERSION_MAJOR >= 5
//         timer_conf.deconfigure      = false;
// #endif
//         ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));
//         timerInitialized = true;
//     }

//     ledc_channel_config_t channel_conf;
//     channel_conf.speed_mode = LEDC_LOW_SPEED_MODE;
//     channel_conf.channel    = channel;
//     channel_conf.timer_sel  = LEDC_TIMER_0;
//     channel_conf.intr_type  = LEDC_INTR_DISABLE;
//     channel_conf.gpio_num   = gpio;
//     channel_conf.duty       = 0;
//     channel_conf.hpoint     = 0;

//     ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
// }

void OutputsController::pwmInitChannel(ledc_channel_t channel, gpio_num_t gpio) {
    static bool timerInitialized = false;

    if (!timerInitialized) {
        ledc_timer_config_t timer_conf = {};
        timer_conf.speed_mode       = LEDC_LOW_SPEED_MODE;
        timer_conf.timer_num        = LEDC_TIMER_0;
        timer_conf.duty_resolution  = LEDC_TIMER_13_BIT;
        timer_conf.freq_hz          = 50;
        timer_conf.clk_cfg          = LEDC_AUTO_CLK;
#if ESP_IDF_VERSION_MAJOR >= 5
        timer_conf.deconfigure      = false;
#endif
        ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));
        timerInitialized = true;
    }

    ledc_channel_config_t channel_conf = {};
    channel_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_conf.channel    = channel;
    channel_conf.timer_sel  = LEDC_TIMER_0;
    channel_conf.intr_type  = LEDC_INTR_DISABLE;
    channel_conf.gpio_num   = gpio;
    channel_conf.duty       = 0;
    channel_conf.hpoint     = 0;

#if ESP_IDF_VERSION_MAJOR >= 5
    channel_conf.flags.output_invert = 0;  // ce champ est requis !
#endif

    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
}

void OutputsController::setPWMDutyUs(ledc_channel_t channel, int us) {
    // Clamp la valeur entre 1000 et 2000 µs
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;

    uint32_t duty = (us * PWM_DUTY_MAX) / PWM_PERIOD_US;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void OutputsController::start() {

    // Init Servo output (pwm)
    pwmInitChannel(LEDC_CHANNEL_4, SERVO_GPIO);

    // Init with 0 value to arm ESCs
    valueCentral = 0;
    valueRearLeft = 0;
    valueRearRight = 0;
    valueFrontLeft = 0;
    valueFrontRight = 0;

    // Init ESCs
    const auto& settings = SettingsController::instance().get();

    if (settings.driveMode == DRIVE_MODE_XCWD) {
        initMotor(settings.centralDriveTrainType, LEDC_CHANNEL_0, ESC_CENTRAL_GPIO, centralESC);

    } else {
        if (settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_FIWD) {
            initMotor(settings.frontDriveTrainType, LEDC_CHANNEL_0, ESC_FRONT_LEFT_GPIO, leftFrontESC);
            initMotor(settings.frontDriveTrainType, LEDC_CHANNEL_1, ESC_FRONT_RIGHT_GPIO, rightFrontESC);
        }

        if (settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_RIWD) {
            initMotor(settings.rearDriveTrainType, LEDC_CHANNEL_2, ESC_REAR_LEFT_GPIO, leftRearESC);
            initMotor(settings.rearDriveTrainType, LEDC_CHANNEL_3, ESC_REAR_RIGHT_GPIO, rightRearESC);
        }
    }

    xTaskCreatePinnedToCore([](void *param) {
        OutputsController::instance().updateTaskLoop();
    }, "dShot_task", 4096, nullptr, 1, &this->updateTaskHandle, 1);

    //delay to arm ESCs
    vTaskDelay(pdMS_TO_TICKS(5000));


    xTaskCreatePinnedToCore(telemetryTaskEntry, "outputs_telemetry_task", 4096, nullptr, 1, &telemetryTaskHandle, 1);
}

void OutputsController::startStreamingTelemetry() {
    streamingTelemetry = true;
}

void OutputsController::stopStreamingTelemetry() {
    streamingTelemetry = false;
}

void OutputsController::telemetryTaskEntry(void* param) {
    OutputsController::instance().telemetryTaskLoop();
}

void OutputsController::telemetryTaskLoop() {

    while (true) {
        if (streamingTelemetry) {
            uint8_t buffer[64];
            TLVWriter writer(buffer);
            writer.begin(MSG_TYPE_DATA, DATA_TYPE_TELEMETRY_OUTPUTS);

            auto writeOutput = [&](uint8_t id, float value, int mode) {
                int16_t encoded = (mode == MOTOR_MODE_PWM)
                    ? normalizedToPwmUs(value)
                    : normalizedToDShotCommand(value);
                writer.addUint16(id, encoded);
            };

            Settings settings = SettingsController::instance().get();

            // Write motor output
            if (settings.driveMode == DRIVE_MODE_XCWD) {
                writeOutput(0x05, valueCentral, settings.centralDriveTrainType);
            } else {
                if (settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_FIWD) {
                    writeOutput(0x03, valueFrontLeft, settings.frontDriveTrainType);
                    writeOutput(0x04, valueFrontRight, settings.frontDriveTrainType);
                }
                if (settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_RIWD) {
                    writeOutput(0x01, valueRearLeft, settings.rearDriveTrainType);
                    writeOutput(0x02, valueRearRight, settings.rearDriveTrainType);
                }
            }

            //Write servo output
            writer.addUint16(0x06, InputsController::instance().getRawValueForChan(1) + settings.steeringTrim * 5);

            BLE::send_notify_to_app(buffer, writer.length());
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void OutputsController::stop() {
    if (updateTaskHandle) {
        vTaskDelete(updateTaskHandle);
        updateTaskHandle = nullptr;
    }

    if (telemetryTaskHandle) {
        vTaskDelete(telemetryTaskHandle);
        telemetryTaskHandle = nullptr;
    }

    // Reset PWM outputs to neutral
    for (int ch = 0; ch < LEDC_CHANNEL_MAX; ++ch) {
        ledc_stop(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(ch), 1); // signal à 0
    }

    // Delete DShot objects if they exist
    auto deleteIfExists = [](DShotRMT*& esc) {
        if (esc) {
            delete esc;
            esc = nullptr;
        }
    };

    deleteIfExists(centralESC);
    deleteIfExists(leftFrontESC);
    deleteIfExists(rightFrontESC);
    deleteIfExists(leftRearESC);
    deleteIfExists(rightRearESC);
}

void OutputsController::restart() {
    stop();
    start();
}
