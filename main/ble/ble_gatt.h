#pragma once

// == MSG TYPE ==
#define MSG_TYPE_COMMAND        0xF0
#define MSG_TYPE_DATA           0xFA

// == DATA TYPE ==
#define DATA_TYPE_SETTINGS              0x01
#define DATA_TYPE_CONTROL               0x02
#define DATA_TYPE_TELEMETRY_MOTION      0x03
#define DATA_TYPE_TELEMETRY_INPUTS      0x04
#define DATA_TYPE_TELEMETRY_OUTPUTS     0x05
#define DATA_TYPE_FEEDBACKS             0x06

// == CMD TYPE ==
#define CMD_TELEMETRY_PAUSE     0x02
#define CMD_TELEMETRY_RESUME    0x03

#define CMD_TELEMETRY_SEND_MOTION     0x08    
#define CMD_TELEMETRY_SEND_INPUTS     0x09
#define CMD_TELEMETRY_SEND_OUTPUTS    0x0F
#define CMD_TELEMETRY_SEND_ALL        0x10


#define CMD_SETTINGS_LOAD       0x04
#define CMD_SETTINGS_RESET      0x05

#define CMD_CONTROL_OVERRIDE    0x06
#define CMD_CONTROL_RELEASE     0x07

#define SCREEN_HOME             0x01
#define SCREEN_TELEMETRY        0x02
#define SCREEN_SETTING          0x03
#define SCREEN_CONTROL          0x04


namespace BLE {
    void register_screen_service();
    void send_notify_to_app(const char* data);
    void send_notify_to_app(const uint8_t* data, size_t len);
}
