idf_component_register(SRCS "ahrs.c" "imu.c" "imu_app.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver board icm42670)

