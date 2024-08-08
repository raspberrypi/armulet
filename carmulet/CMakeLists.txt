add_library(carmulet INTERFACE)
target_sources(carmulet INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/carmulet.c
        )
target_include_directories(carmulet INTERFACE ${CMAKE_CURRENT_LIST_DIR})
