#include "settings_controller.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "ble_gatt.h"
#include "tlv_utils.h"

SettingsController& SettingsController::instance() {
    static SettingsController instance;
    return instance;
}

void SettingsController::load() {
    nvs_handle_t handle;
    if (nvs_open("settings", NVS_READONLY, &handle) != ESP_OK) return;

    uint8_t flags;
    if (nvs_get_u8(handle, "flags", &flags) == ESP_OK) {
        settings.enableRamping     = flags & 0x01;
        settings.enableCoasting    = flags & 0x02;
    }

    nvs_get_u8(handle, "drive", &settings.driveMode);
    nvs_get_u8(handle, "cdtt", &settings.centralDriveTrainType);
    nvs_get_u8(handle, "fdtt", &settings.frontDriveTrainType);
    nvs_get_u8(handle, "rdtt", &settings.rearDriveTrainType);

    char nameBuf[32];
    size_t len = sizeof(nameBuf);
    if (nvs_get_str(handle, "rcName", nameBuf, &len) == ESP_OK) {
        settings.rcName = std::string(nameBuf);
    }

    nvs_get_u8(handle, "front", &settings.frontDiffFactor);
    nvs_get_u8(handle, "rear", &settings.rearDiffFactor);
    nvs_get_u8(handle, "middle", &settings.middleDiffRatio);
    nvs_get_u8(handle, "coast", &settings.coastingFactor);
    nvs_get_u8(handle, "dzT", &settings.throttleDeadzone);
    nvs_get_u8(handle, "dzS", &settings.steeringDeadzone);
    nvs_get_i8(handle, "trimS", &settings.steeringTrim);
    nvs_get_i8(handle, "trimT", &settings.throttleTrim);

    uint8_t inv;
    if (nvs_get_u8(handle, "invS", &inv) == ESP_OK) settings.steeringInverted = inv;
    if (nvs_get_u8(handle, "invT", &inv) == ESP_OK) settings.throttleInverted = inv;

    nvs_close(handle);
}


void SettingsController::save() {
    nvs_handle_t handle;
    if (nvs_open("settings", NVS_READWRITE, &handle) != ESP_OK) return;

    // Flags combinés
    uint8_t flags =
        (settings.enableRamping     ? 0x01 : 0) |
        (settings.enableCoasting    ? 0x02 : 0);
    nvs_set_u8(handle, "flags", flags);

    // Autres champs
    nvs_set_u8(handle, "drive", settings.driveMode);
    nvs_set_u8(handle, "cdtt", settings.centralDriveTrainType);
    nvs_set_u8(handle, "fdtt", settings.frontDriveTrainType);
    nvs_set_u8(handle, "rdtt", settings.rearDriveTrainType);            
    nvs_set_str(handle, "rcName", settings.rcName.c_str());
    nvs_set_u8(handle, "front", settings.frontDiffFactor);
    nvs_set_u8(handle, "rear", settings.rearDiffFactor);
    nvs_set_u8(handle, "middle", settings.middleDiffRatio);
    nvs_set_u8(handle, "coast", settings.coastingFactor);
    nvs_set_u8(handle, "dzT", settings.throttleDeadzone);
    nvs_set_u8(handle, "dzS", settings.steeringDeadzone);
    nvs_set_i8(handle, "trimS", settings.steeringTrim);
    nvs_set_i8(handle, "trimT", settings.throttleTrim);
    nvs_set_u8(handle, "invS", settings.steeringInverted ? 1 : 0);
    nvs_set_u8(handle, "invT", settings.throttleInverted ? 1 : 0);

    nvs_commit(handle);
    nvs_close(handle);
}

void SettingsController::clearAllSettings() {
    nvs_handle_t handle;
    if (nvs_open("settings", NVS_READWRITE, &handle) != ESP_OK) return;

    nvs_erase_all(handle); // wipe everything in "settings"
    nvs_commit(handle);
    nvs_close(handle);

    settings = Settings();
}

void SettingsController::set(const Settings& s) {
    settings = s;
}

Settings SettingsController::get() const {
    return settings;
}

void SettingsController::encode(uint8_t* out, size_t& len) {
    TLVWriter writer(out);
    writer.begin(MSG_TYPE_DATA, DATA_TYPE_SETTINGS);

    writer.addString(0x01, settings.rcName);
    writer.addUint8(0x02, settings.driveMode);
    writer.addBool(0x03, settings.enableRamping);
    writer.addBool(0x04, settings.enableCoasting);
    writer.addBool(0x05, settings.centralDriveTrainType);
    writer.addBool(0x07, settings.frontDriveTrainType);
    writer.addBool(0x09, settings.rearDriveTrainType);
    writer.addUint8(0x06, settings.frontDiffFactor);
    writer.addUint8(0x08, settings.rearDiffFactor);
    writer.addUint8(0x0A, settings.middleDiffRatio);
    writer.addUint8(0x0B, settings.throttleDeadzone);
    writer.addUint8(0x0C, settings.steeringDeadzone);
    writer.addInt8(0x0D, settings.steeringTrim);
    writer.addInt8(0x0E, settings.throttleTrim);
    writer.addBool(0x0F, settings.steeringInverted);
    writer.addBool(0x10, settings.throttleInverted);
    writer.addUint8(0x11, settings.coastingFactor);

    len = writer.length();
}

void SettingsController::decode(const uint8_t* in, size_t len) {
    TLVReader reader(in, len);
    if (!reader.checkHeader(MSG_TYPE_DATA, DATA_TYPE_SETTINGS)) return;

    uint8_t id, size;
    const uint8_t* data;

    while (reader.next(id, size, data)) {
        switch (id) {
            case 0x01: settings.rcName = std::string((const char*)data, size); break;
            case 0x02: settings.driveMode = data[0]; break;
            case 0x03: settings.enableRamping = data[0]; break;
            case 0x04: settings.enableCoasting = data[0]; break;
            case 0x05: settings.centralDriveTrainType = data[0]; break;
            case 0x07: settings.frontDriveTrainType = data[0]; break;
            case 0x09: settings.rearDriveTrainType = data[0]; break;
            case 0x06: settings.frontDiffFactor = data[0]; break;
            case 0x08: settings.rearDiffFactor = data[0]; break;
            case 0x0A: settings.middleDiffRatio = data[0]; break;
            case 0x0B: settings.throttleDeadzone = data[0]; break;
            case 0x0C: settings.steeringDeadzone = data[0]; break;
            case 0x0D: settings.steeringTrim = static_cast<int8_t>(data[0]); break;
            case 0x0E: settings.throttleTrim = static_cast<int8_t>(data[0]); break;
            case 0x0F: settings.steeringInverted = data[0]; break;
            case 0x10: settings.throttleInverted = data[0]; break;
            case 0x11: settings.coastingFactor = data[0]; break;
        }
    }
}