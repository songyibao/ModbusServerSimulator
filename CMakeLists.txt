cmake_minimum_required(VERSION 3.28)
project(ModbusServer)

set(CMAKE_CXX_STANDARD 11)
include_directories(/usr/include/modbus)
add_executable(ModbusServer main.cpp)
target_link_libraries(ModbusServer PRIVATE modbus)