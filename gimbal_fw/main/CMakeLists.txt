idf_component_register(
    SRCS "app_main.c" "ctrl_task.c"
    INCLUDE_DIRS "."
    REQUIRES gimbal_math calib mpu6050 esp_driver_gpio esp_timer
)