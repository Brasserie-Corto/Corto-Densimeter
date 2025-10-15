idf_component_register(
    SRCS "densimeter_main.c"        # Fichiers sources
    INCLUDE_DIRS "."                # Répertoire d'inclusion
    REQUIRES driver                 # Déclare la dépendance au composant "driver" (pour I2C)
)
