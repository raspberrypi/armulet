cmake_minimum_required(VERSION 3.15)
include(pico_sdk_import.cmake)
project(armulet C CXX)
pico_sdk_init()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(armulet INTERFACE)
target_sources(armulet INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/armulet.c
        )
target_include_directories(armulet INTERFACE ${CMAKE_CURRENT_LIST_DIR})

if (PICO_RISCV)
    add_subdirectory(varmulet)
    target_link_libraries(armulet INTERFACE varmulet)
else()
    add_subdirectory(carmulet)
    target_link_libraries(armulet INTERFACE carmulet)
    target_link_libraries(armulet INTERFACE pico_stdlib)
endif()

pico_is_top_level_project(ARMULET_TOP_LEVEL_PROJECT)

if (ARMULET_TOP_LEVEL_PROJECT)
    add_subdirectory(test)
endif()
