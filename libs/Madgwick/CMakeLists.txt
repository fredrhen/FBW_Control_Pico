add_library(Madgwick STATIC MadgwickAHRS.cpp)
target_link_libraries(Madgwick pico_stdlib Quaternion)

pico_enable_stdio_usb(Madgwick 1)
pico_enable_stdio_uart(Madgwick 0)