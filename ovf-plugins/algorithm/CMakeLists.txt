# ovf-plugins/algorithm - 算法插件（扩展）

file(GLOB_RECURSE ALGO_PLUGIN_SOURCES "src/*.cpp")
file(GLOB_RECURSE ALGO_PLUGIN_HEADERS "include/*.h")

add_library(ovf-plugin-algorithm-ext SHARED ${ALGO_PLUGIN_SOURCES} ${ALGO_PLUGIN_HEADERS})

target_include_directories(ovf-plugin-algorithm-ext
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(ovf-plugin-algorithm-ext
    PUBLIC
        ovf-core
        ovf-algorithm
)

install(TARGETS ovf-plugin-algorithm-ext
    RUNTIME DESTINATION bin/plugins
    LIBRARY DESTINATION lib/plugins
)