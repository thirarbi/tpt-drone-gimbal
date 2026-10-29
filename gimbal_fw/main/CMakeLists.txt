idf_component_register(
    SRCS "app_main.c" "ctrl_task.c" "servo_task.c" "comms_task.c" "gimbal_state.c" "bench_tests.c"
    INCLUDE_DIRS "."
    REQUIRES gimbal_math calib mpu6050 esp_driver_gpio esp_driver_uart esp_timer xl320 proto
)
# §4.4: the ESP32 FPU is single precision; catch accidental double maths
target_compile_options(${COMPONENT_LIB} PRIVATE -Wdouble-promotion)
