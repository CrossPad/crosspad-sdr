# SDR -- ESP-IDF component.
#
# The one app of the sdr firmware kit (crosspad-app.json "kits"): built into
# that image only. In any other kit it is a component with no sources, so the
# full firmware carries no SDR code, no reserved SRAM bank and no PHY wraps.
# The requirements stay the same either way: ESP-IDF reads them in an early
# pass that does not know the kit.
#
# esp-sdr (submodule) is the receiver: its calibration and tuning helpers are
# compiled from main/common/; port/idf/sdr_capture_s3.c is its S3 snapshot
# capture without the serial protocol.

set(_sdr_srcs "")
set(_sdr_kit "full")
if(DEFINED CROSSPAD_FW_KIT)
    set(_sdr_kit "${CROSSPAD_FW_KIT}")
endif()
file(READ "${CMAKE_CURRENT_LIST_DIR}/crosspad-app.json" _sdr_app_json)
string(JSON _sdr_kit_count LENGTH "${_sdr_app_json}" kits)
math(EXPR _sdr_last "${_sdr_kit_count} - 1")
foreach(_i RANGE ${_sdr_last})
    string(JSON _k GET "${_sdr_app_json}" kits ${_i})
    if(_k STREQUAL _sdr_kit AND IDF_TARGET STREQUAL "esp32s3")
        set(_sdr_common "${CMAKE_CURRENT_LIST_DIR}/esp-sdr/main/common")
        set(_sdr_srcs
            "src/SdrApp.cpp"
            "port/idf/sdr_worker.cpp"
            "port/idf/sdr_verbs.cpp"
            "port/idf/sdr_capture_s3.c"
            "${_sdr_common}/rx_recalibration.c"
            "${_sdr_common}/rx_calibration_state.c")
    endif()
endforeach()

idf_component_register(
    SRCS ${_sdr_srcs}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "port/idf" "esp-sdr/main/common"
    REQUIRES
        crosspad-core
        crosspad-gui
        lvgl
    PRIV_REQUIRES
        esp_wifi
        esp_phy
        esp_event
        esp_timer
        heap
)

if(_sdr_srcs)
    # WHOLE_ARCHIVE keeps the REGISTER_APP-generated _register_Sdr_app()
    # alive through linking -- main calls it via AppRegistry_InitAll().
    set_property(TARGET ${COMPONENT_LIB} PROPERTY WHOLE_ARCHIVE ON)
    # esp-sdr's own build hooks for the S3 (its main/CMakeLists.txt and
    # common/rx_calibration_link.cmake): retunes measure DC at the requested
    # LO, and every DC measurement is counted. libphy, linked after this
    # archive, is what calls the wraps: -u pulls their objects in up front.
    target_link_options(${COMPONENT_LIB} INTERFACE
        "-Wl,--wrap=chip_v7_set_chan_ana"
        "-Wl,--wrap=set_rx_gain_cal_dc"
        "-Wl,-u,__wrap_chip_v7_set_chan_ana"
        "-Wl,-u,__wrap_set_rx_gain_cal_dc"
        "-Wl,-u,sdr_verbs_handle"
        "-T${CMAKE_CURRENT_LIST_DIR}/port/idf/sram_guard.ld")
    # force_rx_gain() lives in the RF test archive, which esp_phy links only
    # with CONFIG_ESP_PHY_ENABLE_CERT_TEST; this kit takes it on its own.
    set(_phy_lib_dir "${IDF_PATH}/components/esp_phy/lib/${IDF_TARGET}")
    target_link_libraries(${COMPONENT_LIB} INTERFACE "${_phy_lib_dir}/librftest.a" "${_phy_lib_dir}/libphy.a")
endif()
