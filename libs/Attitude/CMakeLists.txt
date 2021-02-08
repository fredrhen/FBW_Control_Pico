add_library(Attitude STATIC Attitude.cpp)
target_link_libraries(Attitude pico_stdlib Quaternion Madgwick)