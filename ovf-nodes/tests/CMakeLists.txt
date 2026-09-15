# ovf-nodes 的测试
#
# ⚠️ 这里链的是 **ovf-nodes-link**，不是 ovf-nodes。
# 这个测试文件里**从头到尾没有调用 initialize_user_nodes()** ——
# 那就是重点：它验证的是第 1 层防御（--no-as-needed）单独也能让算子注册上。
# 如果哪天有人把 ovf-nodes-link 改回直接链 ovf-nodes，这个测试会挂，
# 而且挂的方式就是生产环境会发生的那种：节点全没了。
cmake_minimum_required(VERSION 3.16)

add_executable(test_user_nodes
    test_user_nodes.cpp
)

target_include_directories(test_user_nodes PRIVATE
    ${CMAKE_SOURCE_DIR}/tests            # test_framework.h
)

target_link_libraries(test_user_nodes
    ovf-core
    ovf-nodes-link
)

if(WIN32)
    set_target_properties(test_user_nodes PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/bin/Release"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CMAKE_BINARY_DIR}/bin/Debug"
    )
else()
    set_target_properties(test_user_nodes PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
endif()

enable_testing()
add_test(NAME UserNodesTest COMMAND test_user_nodes)

install(TARGETS test_user_nodes RUNTIME DESTINATION bin)
