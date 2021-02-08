add_executable(MStream Magnetometer_Stream.cpp)
target_link_libraries(MStream pico_stdlib MPU9250)

# enable usb output, disable uart output
pico_enable_stdio_usb(MStream 1)
pico_enable_stdio_uart(MStream 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(MStream)

add_executable(AStream Accel_Stream.cpp)
target_link_libraries(AStream pico_stdlib MPU9250)

# enable usb output, disable uart output
pico_enable_stdio_usb(AStream 1)
pico_enable_stdio_uart(AStream 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(AStream)