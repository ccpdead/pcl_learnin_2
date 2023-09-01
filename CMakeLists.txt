cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(test)
set(CMAKE_CXX_STANDARD 14)

find_package(PCL REQUIRED)

#包含头文件目录
include_directories(${PCL_INCLUDE_DIRS})
#设置依赖库链接目标
link_directories(${PCL_LIBRARY_DIRS})
#添加预处理器和编译器标志
add_definitions(${PCL_DEFINITIONS})

add_executable (main
        01basis/01display/03.cpp
        jhr/view.cpp


)
target_link_libraries (main ${PCL_LIBRARIES})
