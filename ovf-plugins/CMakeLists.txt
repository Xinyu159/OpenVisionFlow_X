# ovf-plugins - 插件集合
cmake_minimum_required(VERSION 3.16)

project(ovf-plugins VERSION 0.1.0 LANGUAGES CXX)

# 硬件插件子目录
add_subdirectory(hardware)
add_subdirectory(communication)
add_subdirectory(algorithm)
add_subdirectory(custom)