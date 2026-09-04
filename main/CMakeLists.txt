idf_component_register(SRCS "main.c" "veml7700.c" "integrator.c" "metering.c" "console.c" "exposure.c"
                        INCLUDE_DIRS "."
                        PRIV_REQUIRES driver esp_adc esp_timer)
