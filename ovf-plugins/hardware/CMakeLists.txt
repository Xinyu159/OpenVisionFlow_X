# ovf-plugins/hardware - 硬件插件（不依赖OpenCV）

file(GLOB_RECURSE HARDWARE_SOURCES "src/*.cpp")
file(GLOB_RECURSE HARDWARE_HEADERS "include/*.h")

add_library(ovf-plugin-hardware SHARED ${HARDWARE_SOURCES} ${HARDWARE_HEADERS})

target_include_directories(ovf-plugin-hardware
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(ovf-plugin-hardware
    PUBLIC
        ovf-core
)

if(WIN32)
    set_target_properties(ovf-plugin-hardware PROPERTIES 
        WINDOWS_EXPORT_ALL_SYMBOLS ON
        ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/lib/Release"
        ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/lib/Debug"
    )
endif()

install(TARGETS ovf-plugin-hardware
    RUNTIME DESTINATION bin/plugins
    LIBRARY DESTINATION lib/plugins
)