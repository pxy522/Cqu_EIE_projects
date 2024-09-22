cmake_minimum_required(VERSION 3.10)

project( ObjectOriented VERSION 0.01 LANGUAGES CXX)

#设置源代码目录
set(SOURCE_DIR ${CMAKE_SOURCE_DIR}/source)
#设置头文件目录
set(INCLUDE_DIR 
    ${CMAKE_SOURCE_DIR}/include
    C:/m_path/3rdparty/json/include
)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable( ObjectOriented ${SOURCE_DIR}/main.cpp)

target_include_directories(ObjectOriented PRIVATE ${INCLUDE_DIR})

# 设置可执行文件的安装路径（安装到 bin 目录下）
install(TARGETS ObjectOriented DESTINATION ${CMAKE_SOURCE_DIR}/bin)