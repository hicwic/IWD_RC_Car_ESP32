//include <stdio.h>
//#include <stdbool.h>
//#include <math.h>
//#include <cstring>

#include "Arduino.h"
//#include "Adafruit_NeoPixel.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_pm.h"

#include "ble_server.h"
#include "drive_controller.h"
#include "settings_controller.h"
#include "inputs_controller.h"
#include "outputs_controller.h"


// === CONFIG ===
#define TAG "RC_ESC"
#define DEBUG true
#define LOG_LEVEL ESP_LOG_INFO


#define LED_PIN    21      // Embedded RGB Led (ESP32-s3 Zero)
#define LED_COUNT  1

// Var for using led as indicator
//Adafruit_NeoPixel pixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// power management
// esp_pm_lock_handle_t power_lock;


// === Setup ===
extern "C" void app_main(void) {
    initArduino();

#if DEBUG
    esp_log_level_set(TAG, LOG_LEVEL);
#endif

    //create power lock to prevent ESP to go in light sleep (cannot handle leds in light sleep)
    // esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "no_light_sleep", &power_lock);
    // esp_pm_lock_acquire(power_lock);

    //vTaskDelay(pdMS_TO_TICKS(2000));

    //load settings from nvs
    SettingsController::instance().load();

    //Bluetooth Init
    BLE::init_ble_server();

    //Inputs Controller Start
    InputsController::instance().start();

    //Ouputs Controller Start
    OutputsController::instance().start();

    //Drive Controller Start
    DriveController::instance().start();

    // pixel.begin(); // Init Led
    // pixel.setBrightness(50); 
    // pixel.setPixelColor(0, pixel.Color(0, 0, 255));
    // pixel.show();

}