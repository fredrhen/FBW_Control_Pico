
add_library(MPU9250 STATIC MPU9250.cpp)
target_link_libraries(MPU9250 pico_stdlib hardware_spi pico_mem_ops)
