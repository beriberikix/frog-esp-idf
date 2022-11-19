set(includes
    "${idf_path}/components/freertos/include/freertos"
)

set(srcs
    "main.c"
    "nvs.c"
    "wifi.c"
)

set(deps
    "golioth_sdk"
    "console"
    "spi_flash"
    "nvs_flash"
    "json"
    "driver"
    "esp_hw_support"
    "lc709203f"
    "scd30"
    "dps310"
)

list(APPEND EXTRA_C_FLAGS_LIST
    -Werror
)

idf_component_register(
    INCLUDE_DIRS "${includes}"
    SRCS "${srcs}"
    PRIV_REQUIRES "${deps}"
)

component_compile_options(${EXTRA_C_FLAGS_LIST})