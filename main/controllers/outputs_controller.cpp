#include "outputs_controller.h"

OutputsController& OutputsController::instance() {
    static OutputsController inst;
    return inst;
}

int OutputsController::velocityToDShotCommand(int velocity) {
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

int OutputsController::velocityToPwmUs(int velocity) {
    // PWM standard : 1000 µs (full reverse) → 1500 µs (stop) → 2000 µs (full forward)

    if (velocity > 100) velocity = 100;
    if (velocity < -100) velocity = -100;

    return 1500 + (velocity * 500) / 100;
}


void OutputsController::initMotor(uint8_t mode, ledc_channel_t channel, gpio_num_t gpio, DShotRMT*& esc) {
    switch (mode) {
        case MOTOR_MODE_PWM:
            pwmInitChannel(channel, gpio);
            setPWMDutyUs(channel, 1500); // neutre
            break;

        case MOTOR_MODE_DSHOT:
            esc = new DShotRMT(gpio);
            esc->begin(DSHOT300, NO_BIDIRECTION, 12);
            break;

        case MOTOR_MODE_BDSHOT:
            esc = new DShotRMT(gpio);
            esc->begin(DSHOT300, ENABLE_BIDIRECTION, 12);
            break;
    }
}

void OutputsController::setMotorOutput(const MotorConfig& cfg, int velocity) {
    switch (cfg.mode) {
        case MOTOR_MODE_PWM:
            setPWMDutyUs(cfg.pwmChannel, velocityToPwmUs(velocity));
            break;

        case MOTOR_MODE_DSHOT:
        case MOTOR_MODE_BDSHOT:
            if (cfg.dshot) {
                cfg.dshot->send_dshot_value(velocityToDShotCommand(velocity));
            }
            break;
    }
}

void OutputsController::updateTaskLoop() {
    const auto& settings = SettingsController::instance().get();

    while (true) {
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

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


void OutputsController::pwmInitChannel(ledc_channel_t channel, gpio_num_t gpio) {
    static bool timerInitialized = false;

    if (!timerInitialized) {
        ledc_timer_config_t timer_conf;
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

    ledc_channel_config_t channel_conf;
    channel_conf.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_conf.channel    = channel;
    channel_conf.timer_sel  = LEDC_TIMER_0;
    channel_conf.intr_type  = LEDC_INTR_DISABLE;
    channel_conf.gpio_num   = gpio;
    channel_conf.duty       = 0;
    channel_conf.hpoint     = 0;

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

}
