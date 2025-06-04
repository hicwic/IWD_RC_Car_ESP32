idf_component_register(
    SRCS "controllers/outputs_controller.cpp" "controllers/inputs_controller.cpp" "controllers/settings_controller.cpp" "controllers/drive_controller.cpp" "ble/ble_gatt.cpp" "ble/ble_server.cpp" "main.cpp" 
    INCLUDE_DIRS "" "ble" "controllers" "utils"
)
