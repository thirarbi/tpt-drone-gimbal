idf_component_register(
    SRCS "calib.c"
    INCLUDE_DIRS "."
    REQUIRES gimbal_math nvs_flash
)