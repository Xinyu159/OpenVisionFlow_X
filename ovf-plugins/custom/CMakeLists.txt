# ovf-plugins/custom - 自定义算子插件示例

file(GLOB_RECURSE CUSTOM_SOURCES "src/*.cpp")
file(GLOB_RECURSE CUSTOM_HEADERS "include/*.h")

add_library(ovf-plugin-custom SHARED ${CUSTOM_SOURCES} ${CUSTOM_HEADERS})

target_include_directories(ovf-plugin-custom
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(ovf-plugin-custom
    PUBLIC
        ovf-core
)

install(TARGETS ovf-plugin-custom
    RUNTIME DESTINATION bin/plugins
    LIBRARY DESTINATION lib/plugins
)