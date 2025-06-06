
#include "inputs_controller.h"

InputsController& InputsController::instance() {
    static InputsController inst;
    return inst;
}


void InputsController::enableOverrideControl() {
    overrideControl = true;
}

void InputsController::disableOverrideControl() {
    overrideControl = false;
}


void InputsController::setControlInputs(const uint8_t* in, size_t len) {
    TLVReader reader(in, len);
    if (!reader.checkHeader(MSG_TYPE_DATA, DATA_TYPE_CONTROL)) return;

    uint8_t id, size;
    const uint8_t* data;

    while (reader.next(id, size, data)) {
        if (size < 2) continue;

        uint16_t value = data[0] | (data[1] << 8);

        switch (id) {
            case 0x01:
                overridedChan1 = value;
                break;
            case 0x02:
                overridedChan2 = value;
                break;
            default:
                break;
        }
    }
}

int8_t InputsController::pwmToPercent(int16_t pwm) {
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    return ((pwm - PWM_MID) * 100) / (PWM_MAX - PWM_MID);
}

int16_t InputsController::percentToPwm(int8_t percent) {
    if (percent < -100) percent = -100;
    if (percent > 100)  percent = 100;
    return PWM_MID + ((PWM_MAX - PWM_MID) * percent) / 100;
}

int8_t InputsController::getValueForChan(uint8_t ch) {
    if (overrideControl && ch == 1)
        return pwmToPercent(overridedChan1);
    if (overrideControl && ch == 2)
        return pwmToPercent(overridedChan2);

    return pwmToPercent(pulseWidth[ch-1]);
}

int16_t InputsController::getRawValueForChan(uint8_t ch) {
    if (overrideControl && ch == 1)
        return overridedChan1;
    if (overrideControl && ch == 2)
        return overridedChan2;

    return pulseWidth[ch-1];
}

// === ISR read PWM ===
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    int ch = (int)arg;
    int level = gpio_get_level(InputsController::instance().chPins[ch]);
    int64_t now = esp_timer_get_time();

    if(level == 1) {
        InputsController::instance().pulseStart[ch] = now;
        InputsController::instance().pulseStarted[ch] = true;
    } else {
        if(InputsController::instance().pulseStarted[ch]) {
            int width = now - InputsController::instance().pulseStart[ch];
            if (width >= 500 && width <= 2500) {
                InputsController::instance().pulseWidth[ch] = width;
            } else {
                InputsController::instance().pulseWidth[ch] = PWM_MID;
            }
            InputsController::instance().pulseStarted[ch] = false;
        }
    }
}

void InputsController::start() {
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

    xTaskCreatePinnedToCore(telemetryTaskEntry, "inputs_telemetry_task", 4096, nullptr, 1, &telemetryTaskHandle, 1);
}

void InputsController::startStreamingTelemetry() {
    streamingTelemetry = true;
}

void InputsController::stopStreamingTelemetry() {
    streamingTelemetry = false;
}

void InputsController::telemetryTaskEntry(void* param) {
    InputsController::instance().telemetryTaskLoop();
}

void InputsController::telemetryTaskLoop() {
    while (true) {
        if (streamingTelemetry) {
            uint8_t buffer[32];
            TLVWriter writer(buffer);
            writer.begin(MSG_TYPE_DATA, DATA_TYPE_TELEMETRY_INPUTS);

            for (int i = 0; i < NUM_CHANNELS; ++i) {
                writer.addUint16(0x10 + i, getRawValueForChan(i+1));
            }

            BLE::send_notify_to_app(buffer, writer.length());
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}