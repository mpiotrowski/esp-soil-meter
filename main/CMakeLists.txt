idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES spi_flash
                    PRIV_REQUIRES nvs_flash
                    PRIV_REQUIRES bt
                    PRIV_REQUIRES esp_timer
                    INCLUDE_DIRS "")
