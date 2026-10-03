idf_component_register(SRCS "main.c" "motor_control.c"
                       PRIV_REQUIRES driver esp_timer micro_ros_espidf_component espressif__led_strip
                       INCLUDE_DIRS ".")
