idf_component_register(SRCS "ahrs.cpp" "imu.c" "imu_app.c" "third_party/vqf/vqf.cpp"
                    INCLUDE_DIRS "include" "."
                    REQUIRES driver board icm42670)

