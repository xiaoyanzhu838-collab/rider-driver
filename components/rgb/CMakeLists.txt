idf_component_register(SRCS "ws2812.c" "rgb_service.c" "rgb_app.c" "rgb_effects.c" "rgb_control.c"
                    INCLUDE_DIRS "include"
                    REQUIRES led_strip board driver)

