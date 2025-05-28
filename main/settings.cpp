#include "settings.h"
#include "nvs_flash.h"
#include "nvs.h"

SettingsManager& SettingsManager::instance() {
    static SettingsManager instance;
    return instance;
}

void SettingsManager::load() {
    nvs_handle_t handle;
    if (nvs_open("settings", NVS_READONLY, &handle) != ESP_OK) return;

    uint8_t flags;
    nvs_get_u8(handle, "flags", &flags);
    settings.enableRamping     = flags & 0x01;
    settings.enableCoasting    = flags & 0x02;
    settings.enableFrontMotors = flags & 0x04;
    settings.enableRearMotors  = flags & 0x08;

    nvs_get_u8(handle, "front", &settings.frontDiffMix);
    nvs_get_u8(handle, "rear", &settings.rearDiffMix);
    nvs_get_u8(handle, "middle", &settings.middleDiffRatio);
    nvs_get_u8(handle, "dzT", &settings.deadzoneThrottle);
    nvs_get_u8(handle, "dzS", &settings.deadzoneSteering);

    int8_t trim;
    nvs_get_i8(handle, "trim", &trim);
    settings.steeringTrim = trim;

    nvs_close(handle);
}

void SettingsManager::save() {
    nvs_handle_t handle;
    if (nvs_open("settings", NVS_READWRITE, &handle) != ESP_OK) return;

    uint8_t flags =
        (settings.enableRamping     ? 0x01 : 0) |
        (settings.enableCoasting    ? 0x02 : 0) |
        (settings.enableFrontMotors ? 0x04 : 0) |
        (settings.enableRearMotors  ? 0x08 : 0);

    nvs_set_u8(handle, "flags", flags);
    nvs_set_u8(handle, "front", settings.frontDiffMix);
    nvs_set_u8(handle, "rear", settings.rearDiffMix);
    nvs_set_u8(handle, "middle", settings.middleDiffRatio);
    nvs_set_u8(handle, "dzT", settings.deadzoneThrottle);
    nvs_set_u8(handle, "dzS", settings.deadzoneSteering);
    nvs_set_i8(handle, "trim", settings.steeringTrim);

    nvs_commit(handle);
    nvs_close(handle);
}

void SettingsManager::set(const Settings& s) {
    settings = s;
}

Settings SettingsManager::get() const {
    return settings;
}

void SettingsManager::encode(uint8_t* out) const {
    out[0] = 0x01; // type
    out[1] =
        (settings.enableRamping     ? 0x01 : 0) |
        (settings.enableCoasting    ? 0x02 : 0) |
        (settings.enableFrontMotors ? 0x04 : 0) |
        (settings.enableRearMotors  ? 0x08 : 0);

    out[2] = settings.frontDiffMix;
    out[3] = settings.rearDiffMix;
    out[4] = settings.middleDiffRatio;
    out[5] = settings.deadzoneThrottle;
    out[6] = settings.deadzoneSteering;
    out[7] = static_cast<uint8_t>(settings.steeringTrim + 50);
}

void SettingsManager::decode(const uint8_t* in) {
    if (in[0] != 0x01) return;

    settings.enableRamping     = in[1] & 0x01;
    settings.enableCoasting    = in[1] & 0x02;
    settings.enableFrontMotors = in[1] & 0x04;
    settings.enableRearMotors  = in[1] & 0x08;

    settings.frontDiffMix      = in[2];
    settings.rearDiffMix       = in[3];
    settings.middleDiffRatio   = in[4];
    settings.deadzoneThrottle  = in[5];
    settings.deadzoneSteering  = in[6];
    settings.steeringTrim      = static_cast<int8_t>(in[7]) - 50;
}
