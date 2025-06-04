#pragma once

#include <cstdint>
#include <string>
#include <cstring>

/*

enableRamping       bool
enableCoasting      bool

enableFrontMotors   bool
enableRearMotors    bool

frontDiffFactor        0 -> 100
rearDiffFactor         0 -> 100
middleDiffRatio     0 -> 100

throttleDeadzone    0->20
steeringDeadzone    0->20

steeringTrim        -50->50

*/
#define DRIVE_MODE_XCWD     0
#define DRIVE_MODE_FIWD     1
#define DRIVE_MODE_RIWD     2
#define DRIVE_MODE_AIWD     3

#define MOTOR_MODE_PWM      0
#define MOTOR_MODE_DSHOT    1
#define MOTOR_MODE_BDSHOT   2   //bidirectionnal DSHOT


struct Settings {
    std::string rcName = "MyRC";

    bool enableRamping = true;
    bool enableCoasting = true;

    uint8_t driveMode = DRIVE_MODE_XCWD; 

    uint8_t centralDriveTrainType = MOTOR_MODE_PWM;
    uint8_t frontDriveTrainType = MOTOR_MODE_PWM;    
    uint8_t rearDriveTrainType = MOTOR_MODE_PWM;

    uint8_t frontDiffFactor = 50;
    uint8_t rearDiffFactor = 50;
    uint8_t middleDiffRatio = 50;

    uint8_t coastingFactor = 50;

    uint8_t throttleDeadzone = 10;
    uint8_t steeringDeadzone = 0;

    int8_t steeringTrim = 0;
    int8_t throttleTrim = 0;

    bool steeringInverted = false;
    bool throttleInverted = false;
};

class SettingsController {
public:
    static SettingsController& instance();

    void load();
    void save();
    void clearAllSettings();

    void set(const Settings& s);
    Settings get() const;

    void encode(uint8_t* out, size_t& len);
    void decode(const uint8_t* in, size_t len);

private:
    SettingsController() = default;
    Settings settings;
};