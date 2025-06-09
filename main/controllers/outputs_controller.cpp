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
            esc->begin(DSHOT300, NO_BIDIRECTION, 14);
            break;

        case MOTOR_MODE_BDSHOT:
        ESP_LOGI("ESC_INIT", "Init motor mode=%d on gpio=%d", mode, gpio);
            esc = new DShotRMT(gpio);
            esc->begin(DSHOT300, ENABLE_BIDIRECTION, 14);
            break;
    }
}

void OutputsController::setMotorDShotOutput(MotorIndex index, DShotRMT* esc, float normalized, uint8_t mode) {
    static uint32_t lastSendMicros[MOTOR_COUNT] = {0};

    if (!esc) return;

    uint32_t now = micros();

    if (mode == MOTOR_MODE_BDSHOT) {
        uint32_t rpm = 0;
        auto result = esc->get_dshot_packet(&rpm);

        if (result == DECODE_SUCCESS) {
            currentRPM[index] = rpm;
        }
    }

    if (now - lastSendMicros[index] > 300) {
        esc->send_dshot_value(normalizedToDShotCommand(normalized));
        lastSendMicros[index] = now;
    }
}

void OutputsController::dshotTaskLoop() {
    while (true) {
        const auto& settings = SettingsController::instance().get();

        if (settings.driveMode == DRIVE_MODE_XCWD &&
            settings.centralDriveTrainType != MOTOR_MODE_PWM) {
            setMotorDShotOutput(MOTOR_CENTRAL, centralESC, valueCentral, settings.centralDriveTrainType);
        }

        if ((settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_FIWD) &&
            settings.frontDriveTrainType != MOTOR_MODE_PWM) {
            setMotorDShotOutput(MOTOR_FRONT_LEFT, leftFrontESC, valueFrontLeft, settings.frontDriveTrainType);
            setMotorDShotOutput(MOTOR_FRONT_RIGHT, rightFrontESC, valueFrontRight, settings.frontDriveTrainType);
        }

        if ((settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_RIWD) &&
            settings.rearDriveTrainType != MOTOR_MODE_PWM) {
            setMotorDShotOutput(MOTOR_REAR_LEFT, leftRearESC, valueRearLeft, settings.rearDriveTrainType);
            setMotorDShotOutput(MOTOR_REAR_RIGHT, rightRearESC, valueRearRight, settings.rearDriveTrainType);
        }

        taskYIELD();
    }
}

void OutputsController::setMotorPWMOutput(ledc_channel_t channel, float normalized) {
    setPWMDutyUs(channel, normalizedToPwmUs(normalized));
}

void OutputsController::pwmTaskLoop() {
    while (true) {
        const auto& settings = SettingsController::instance().get();

        if (settings.driveMode == DRIVE_MODE_XCWD &&
            settings.centralDriveTrainType == MOTOR_MODE_PWM) {
            setMotorPWMOutput(LEDC_CHANNEL_0, valueCentral);
        }

        if ((settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_FIWD) &&
            settings.frontDriveTrainType == MOTOR_MODE_PWM) {
            setMotorPWMOutput(LEDC_CHANNEL_0, valueFrontLeft);
            setMotorPWMOutput(LEDC_CHANNEL_1, valueFrontRight);
        }

        if ((settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_RIWD) &&
            settings.rearDriveTrainType == MOTOR_MODE_PWM) {
            setMotorPWMOutput(LEDC_CHANNEL_2, valueRearLeft);
            setMotorPWMOutput(LEDC_CHANNEL_3, valueRearRight);
        }

        // Servo
        setPWMDutyUs(
            LEDC_CHANNEL_4,
            InputsController::instance().getRawValueForChan(1) + settings.steeringTrim * 5
        );

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


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

    xTaskCreatePinnedToCore([](void*) {
        OutputsController::instance().pwmTaskLoop();
        vTaskDelete(nullptr);
    }, "PWMTask", 4096, nullptr, 1, &this->pwmTaskHandle, 1);

    // xTaskCreatePinnedToCore([](void*) {
    //     OutputsController::instance().dshotTaskLoop();
    //     vTaskDelete(nullptr);
    // }, "DShotTask", 4096, nullptr, 0, &this->dshotTaskHandle, 1);

    startDshotTimer();

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
                if (settings.centralDriveTrainType == MOTOR_MODE_BDSHOT) {
                    writer.addUInt32(0x15, currentRPM[MOTOR_CENTRAL]);
                }                
            } else {
                if (settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_FIWD) {
                    writeOutput(0x03, valueFrontLeft, settings.frontDriveTrainType);
                    writeOutput(0x04, valueFrontRight, settings.frontDriveTrainType);
                    if (settings.frontDriveTrainType == MOTOR_MODE_BDSHOT) {
                        writer.addUInt32(0x13, currentRPM[MOTOR_FRONT_LEFT]);
                        writer.addUInt32(0x14, currentRPM[MOTOR_FRONT_RIGHT]);
                    }
                }
                if (settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_RIWD) {
                    writeOutput(0x01, valueRearLeft, settings.rearDriveTrainType);
                    writeOutput(0x02, valueRearRight, settings.rearDriveTrainType);
                    if (settings.rearDriveTrainType == MOTOR_MODE_BDSHOT) {
                        writer.addUInt32(0x11, currentRPM[MOTOR_REAR_LEFT]);
                        writer.addUInt32(0x12, currentRPM[MOTOR_REAR_RIGHT]);
                    }                    
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
    if (dshotTaskHandle) {
        vTaskDelete(dshotTaskHandle);
        dshotTaskHandle = nullptr;
    }

    if (pwmTaskHandle) {
        vTaskDelete(pwmTaskHandle);
        pwmTaskHandle = nullptr;
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


void IRAM_ATTR dshot_timer_callback(void* arg) {
    OutputsController::instance().dshotTick();
}

void OutputsController::startDshotTimer() {
    const esp_timer_create_args_t timer_args = {
        .callback = &dshot_timer_callback,
        .arg = nullptr,
        .name = "dshot_timer"
    };

    esp_timer_handle_t timer;
    esp_timer_create(&timer_args, &timer);

    // 100 µs
    esp_timer_start_periodic(timer, 100);
}

void OutputsController::dshotTick() {
    const auto& settings = SettingsController::instance().get();

    if (settings.driveMode == DRIVE_MODE_XCWD &&
        settings.centralDriveTrainType != MOTOR_MODE_PWM) {
        setMotorDShotOutput(MOTOR_CENTRAL, centralESC, valueCentral, settings.centralDriveTrainType);
    }

    if ((settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_FIWD) &&
        settings.frontDriveTrainType != MOTOR_MODE_PWM) {
        setMotorDShotOutput(MOTOR_FRONT_LEFT, leftFrontESC, valueFrontLeft, settings.frontDriveTrainType);
        setMotorDShotOutput(MOTOR_FRONT_RIGHT, rightFrontESC, valueFrontRight, settings.frontDriveTrainType);
    }

    if ((settings.driveMode == DRIVE_MODE_AIWD || settings.driveMode == DRIVE_MODE_RIWD) &&
        settings.rearDriveTrainType != MOTOR_MODE_PWM) {
        setMotorDShotOutput(MOTOR_REAR_LEFT, leftRearESC, valueRearLeft, settings.rearDriveTrainType);
        setMotorDShotOutput(MOTOR_REAR_RIGHT, rightRearESC, valueRearRight, settings.rearDriveTrainType);
    }
}