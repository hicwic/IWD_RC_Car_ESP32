
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
    if (len < 4) return;
    if (in[0] != MSG_TYPE_DATA || in[1] != DATA_TYPE_CONTROL) return;

    overridedChan1     = in[2];
    overridedChan2     = in[3];
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
        return overridedChan1;
    if (overrideControl && ch == 2)
        return overridedChan2;

    return pwmToPercent(pulseWidth[ch-1]);
}

int8_t InputsController::getRawValueForChan(uint8_t ch) {
    if (overrideControl && ch == 1)
        return percentToPwm(overridedChan1);
    if (overrideControl && ch == 2)
        return percentToPwm(overridedChan2);

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
}