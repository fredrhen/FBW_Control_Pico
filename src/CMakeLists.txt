add_executable(main main.cpp)
target_link_libraries(main pico_stdlib MPU9250 Attitude)

# enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(main)

add_executable(Calibration Calibration.cpp)
target_link_libraries(Calibration pico_stdlib MPU9250 Attitude)

# enable usb output, disable uart output
pico_enable_stdio_usb(Calibration 1)
pico_enable_stdio_uart(Calibration 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(Calibration)