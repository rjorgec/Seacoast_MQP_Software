idf_component_register(
    SRCS
        "app_main.c"
        "display.c"
        "ui.c"
        "ui_screens.c"
        "touch_ns2009.c"
        "touch_cal.c"
        "control.c"
        "dosing.c"
        "loadcell.c"
        "flap.c"
        "sys_sequence.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer
    PRIV_REQUIRES nvs_flash
    PRIV_REQUIRES motor_hal
)
