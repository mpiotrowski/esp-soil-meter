idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES spi_flash
                    PRIV_REQUIRES nvs_flash
                    REQUIRES bt
                    INCLUDE_DIRS "")
