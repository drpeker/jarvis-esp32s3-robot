file(GLOB SRCS "*.cc")
# Caption font (fonts/README.txt), embedded 16-byte aligned through an assembler file.
set(caption_font_s ${CMAKE_CURRENT_BINARY_DIR}/caption_font.S)
set(FONT_FILE ${CMAKE_CURRENT_LIST_DIR}/fonts/font_noto_sans_common_14_1.bin)
configure_file(${CMAKE_CURRENT_LIST_DIR}/fonts/caption_font.S.in ${caption_font_s} @ONLY)
set_property(SOURCE ${caption_font_s} APPEND PROPERTY OBJECT_DEPENDS ${FONT_FILE})
idf_component_register(SRCS ${SRCS} ${caption_font_s}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES nvs_flash esp_driver_i2c esp_driver_ledc esp_driver_gpio
                                     esp_driver_i2s esp_driver_spi esp_lcd esp_wifi esp_netif
                                     esp_event esp_http_client esp_https_ota app_update
                                     mbedtls json esp_timer mqtt esp-sr esp_http_server)
# Crash recorder (crash.cc) wraps the panic handler.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_panic_handler")

# Wake word models, built from source on every build: esp-sr's own tool copies the models
# selected in sdkconfig (CONFIG_SR_WN_*) out of the esp-sr component and packs them into
# srmodels.bin, which is embedded in the app (wake_word.cc).
idf_component_get_property(esp_sr_dir espressif__esp-sr COMPONENT_DIR)
set(wake_dir ${CMAKE_BINARY_DIR}/wake_models)
set(wake_bin ${wake_dir}/srmodels/srmodels.bin)
add_custom_command(
    OUTPUT ${wake_bin}
    COMMAND ${python} ${esp_sr_dir}/model/movemodel.py -d1 ${SDKCONFIG} -d2 ${esp_sr_dir} -d3 ${wake_dir}
    DEPENDS ${SDKCONFIG} ${esp_sr_dir}/model/movemodel.py ${esp_sr_dir}/model/pack_model.py
    COMMENT "Packing wake word models (esp-sr)"
    VERBATIM)
add_custom_target(wake_models DEPENDS ${wake_bin})
target_add_binary_data(${COMPONENT_LIB} ${wake_bin} BINARY DEPENDS wake_models)
