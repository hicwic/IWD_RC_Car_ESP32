#pragma once

#include <cstdint>

/*

enableRamping       bool
enableCoasting      bool

enableFrontMotors   bool
enableRearMotors    bool

frontDiffMix        0 -> 100
rearDiffMix         0 -> 100
middleDiffRatio     0 -> 100

deadzoneThrottle    0->20
deadzoneSteering    0->20

steeringTrim        -50->50

*/

struct Settings {
    bool enableRamping = true;
    bool enableCoasting = true;

    bool enableFrontMotors = false;
    bool enableRearMotors = false;

    uint8_t frontDiffMix = 50;       // 0 - 100
    uint8_t rearDiffMix = 50;        // 0 - 100
    uint8_t middleDiffRatio = 50;    // 0 - 100

    uint8_t deadzoneThrottle = 10;   // 0 - 20
    uint8_t deadzoneSteering = 0;   // 0 - 20

    int8_t steeringTrim = 0;        // -50 - 50
};


class SettingsManager {
public:
    static SettingsManager& instance();

    void load();
    void save();

    void set(const Settings& s);
    Settings get() const;

    void encode(uint8_t* out) const;
    void decode(const uint8_t* in);

private:
    SettingsManager() = default;
    Settings settings;
};