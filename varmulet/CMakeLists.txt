
add_library(varmulet INTERFACE)
target_sources(armulet INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/varmulet.S
        ${CMAKE_CURRENT_LIST_DIR}/varmulet_armv6m_core.S
        ${CMAKE_CURRENT_LIST_DIR}/varmulet_hooks_default.S
        )
target_include_directories(varmulet INTERFACE ${CMAKE_CURRENT_LIST_DIR})
