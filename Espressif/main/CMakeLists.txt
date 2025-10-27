idf_component_register(
    SRCS    "densimeter_main.c"        # Fichiers sources
            "Hardware/I2C/i2c_functions.c"
            "Hardware/MPU6050/mpu6050_functions.c"
            "Hardware/LED/led_functions.c"
    INCLUDE_DIRS    "."                # Répertoire d'inclusion
                    "Hardware/I2C"
                    "Hardware/MPU6050"
                    "Hardware/LED"
    REQUIRES driver                 # Déclare la dépendance au composant "driver" (pour I2C)
)
