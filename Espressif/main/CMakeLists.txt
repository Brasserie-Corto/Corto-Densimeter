idf_component_register(
    SRCS    "densimeter_main.c"
            "Hardware/I2C/i2c_functions.c"
            "Hardware/MPU6050/mpu6050_functions.c"
            "Hardware/LED/led_functions.c"
            "Software/DENSIMETER/densimeter_functions.c"
    INCLUDE_DIRS    "."
                    "Hardware/I2C"
                    "Hardware/MPU6050"
                    "Hardware/LED"
                    "Software/DENSIMETER"
    REQUIRES driver esp_driver_i2c esp_driver_gpio  )
