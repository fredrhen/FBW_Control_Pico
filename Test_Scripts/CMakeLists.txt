add_executable(MadTest Madgwick_Test.cpp)
target_link_libraries(MadTest pico_stdlib)

# enable usb output, disable uart output
pico_enable_stdio_usb(MadTest 1)
pico_enable_stdio_uart(MadTest 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(MadTest)
