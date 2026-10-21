cmake_minimum_required(VERSION 3.16)
set(REBUDDY_CONSOLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.references/esp32c5_ui_console"
    CACHE PATH "External private board support checkout (kept outside reBuddy)")
get_filename_component(REBUDDY_CONSOLE_DIR "${REBUDDY_CONSOLE_DIR}" REALPATH)
if(NOT EXISTS "${REBUDDY_CONSOLE_DIR}/components/ui_console_display/CMakeLists.txt")
    message(FATAL_ERROR "Clone the private board repository outside reBuddy and set -DREBUDDY_CONSOLE_DIR=/absolute/path/to/esp32c5_ui_console. Host tests do not require this dependency.")
endif()
# Absolute Kconfig sources also work in Ninja/ldgen without shell environment variables.
file(WRITE "${CMAKE_BINARY_DIR}/rebuddy.Kconfig.projbuild"
    "source \"${REBUDDY_CONSOLE_DIR}/main/Kconfig.projbuild\"\n"
    "source \"${CMAKE_CURRENT_SOURCE_DIR}/main/Kconfig.projbuild\"\n")
set(EXTRA_COMPONENT_DIRS "${REBUDDY_CONSOLE_DIR}/components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
set(PROJECT_VER "0.1.0")
project(reBuddy)
idf_component_get_property(rb_lvgl lvgl__lvgl COMPONENT_LIB)
target_include_directories(${rb_lvgl} PRIVATE "${REBUDDY_CONSOLE_DIR}/main/platform")
target_compile_definitions(${rb_lvgl} PRIVATE LV_MEM_POOL_INCLUDE="console_memory.h" LV_MEM_POOL_ALLOC=console_memory_lvgl_pool)
