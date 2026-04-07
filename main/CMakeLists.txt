idf_component_register(SRCS "main.c" "can_master/can_master.c" "cmd_control/cmd_control.c" "status_led/status_led.c"
                       INCLUDE_DIRS "." "can_master" "cmd_control" "status_led"
                       REQUIRES driver console esp_driver_rmt spiffs nvs_flash)
