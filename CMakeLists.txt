cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
set(PROJECT_VER "0.2.0")
project(reBuddy)
idf_build_get_property(rb_python PYTHON)
add_custom_target(rebuddy_flash_budget ALL
    COMMAND "${rb_python}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/check_flash_budget.py"
        --app "${CMAKE_BINARY_DIR}/reBuddy.bin"
    DEPENDS gen_project_binary
    VERBATIM)
add_dependencies(app rebuddy_flash_budget)
add_dependencies(flash rebuddy_flash_budget)
idf_component_get_property(rb_lvgl lvgl__lvgl COMPONENT_LIB)
target_include_directories(${rb_lvgl} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/components/rebuddy_board/include")
target_compile_definitions(${rb_lvgl} PRIVATE LV_MEM_POOL_INCLUDE="rb_memory.h" LV_MEM_POOL_ALLOC=rb_memory_lvgl_pool)
