cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(test)
set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})#包含头文件目录
link_directories(${PCL_LIBRARY_DIRS})#设置依赖库链接目标
add_definitions(${PCL_DEFINITIONS})#添加预处理器和编译器标志

add_executable (main
        01.cpp
)
target_link_libraries (main ${PCL_LIBRARIES})
