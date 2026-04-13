idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES rgb fan led button imu uart board motor)
