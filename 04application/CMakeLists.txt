cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(test)
set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})#包含头文件目录
link_directories(${PCL_LIBRARY_DIRS})#设置依赖库链接目标
add_definitions(${PCL_DEFINITIONS})#添加预处理器和编译器标志

add_executable (main
#01_点云分割/03.cpp
#        02_点云曲面重建/01.cpp
#        03_3D包容盒子/01.cpp
        04_点云模板匹配/01.cpp
)

target_link_libraries (main ${PCL_LIBRARIES})
