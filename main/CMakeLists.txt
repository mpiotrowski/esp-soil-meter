idf_component_register(SRCS "main.c" "pb_encode.c" "pb_decode.c" "pb_common.c" "soil_meter.pb.c"
                    PRIV_REQUIRES spi_flash
                    PRIV_REQUIRES nvs_flash
                    PRIV_REQUIRES bt
                    PRIV_REQUIRES esp_timer
                    PRIV_REQUIRES stemma-sensor
                    INCLUDE_DIRS ".")
