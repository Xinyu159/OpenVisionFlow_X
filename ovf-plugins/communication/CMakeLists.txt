# ovf-plugins/communication - 通信插件

file(GLOB_RECURSE COMM_SOURCES "src/*.cpp")
file(GLOB_RECURSE COMM_HEADERS "include/*.h")

add_library(ovf-plugin-communication SHARED ${COMM_SOURCES} ${COMM_HEADERS})

target_include_directories(ovf-plugin-communication
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(ovf-plugin-communication
    PUBLIC
        ovf-core
)

install(TARGETS ovf-plugin-communication
    RUNTIME DESTINATION bin/plugins
    LIBRARY DESTINATION lib/plugins
)