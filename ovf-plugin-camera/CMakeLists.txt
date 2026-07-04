# ovf-plugin-camera - 相机驱动插件
# 基于GenICam标准的通用相机驱动

file(GLOB_RECURSE CAMERA_SOURCES "src/*.cpp")
file(GLOB_RECURSE CAMERA_HEADERS "include/*.h")

add_library(ovf-plugin-camera SHARED ${CAMERA_SOURCES} ${CAMERA_HEADERS})

target_compile_definitions(ovf-plugin-camera PRIVATE OVF_PLUGIN_CAMERA_EXPORTS)

target_include_directories(ovf-plugin-camera
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(ovf-plugin-camera
    PUBLIC
        ovf-core
)

if(WIN32)
    set_target_properties(ovf-plugin-camera PROPERTIES 
        WINDOWS_EXPORT_ALL_SYMBOLS ON
        ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/Release"
        ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/Debug"
    )
endif()

install(TARGETS ovf-plugin-camera
    RUNTIME DESTINATION bin/plugins
    LIBRARY DESTINATION lib/plugins
)