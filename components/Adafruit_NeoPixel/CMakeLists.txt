idf_component_register(
    SRCS "src/Adafruit_NeoPixel.cpp" "src/esp.c"
    INCLUDE_DIRS "./src"
    REQUIRES arduino
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=parentheses)