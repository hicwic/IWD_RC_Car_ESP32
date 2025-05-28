#pragma once

const int MSG_TYPE_SETTINGS = 0x01;

namespace BLE {
    void register_screen_service();
    void send_notify_to_app(const char* data);
    void send_notify_to_app(const uint8_t* data, size_t len);
}
