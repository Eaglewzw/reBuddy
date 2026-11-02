set(DAILY_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
file(READ "${DAILY_ROOT}/assets/verification/daily_c5_mapping.json" DAILY_REPORT)
foreach(PAIR "record/daily_ready/manifest.json|source_manifest_sha256"
             "components/rebuddy_core/src/rb_daily_data.c|data_sha256"
             "tools/export_daily_c5.py|generator_sha256")
    string(REPLACE "|" ";" PARTS "${PAIR}")
    list(GET PARTS 0 RELATIVE_PATH)
    list(GET PARTS 1 HASH_KEY)
    file(SHA256 "${DAILY_ROOT}/${RELATIVE_PATH}" ACTUAL_HASH)
    string(JSON EXPECTED_HASH GET "${DAILY_REPORT}" "${HASH_KEY}")
    if(NOT ACTUAL_HASH STREQUAL EXPECTED_HASH)
        message(FATAL_ERROR "Stale C5 daily mapping. Run tools/export_daily_c5.py with the RS Python environment.")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${DAILY_ROOT}/${RELATIVE_PATH}")
endforeach()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${DAILY_ROOT}/assets/verification/daily_c5_mapping.json")
idf_component_register(SRCS "src/rb_pet.c" "src/rb_choreo.c" "src/rb_motion.c"
    "src/rb_daily.c" "src/rb_daily_data.c" INCLUDE_DIRS "include")
