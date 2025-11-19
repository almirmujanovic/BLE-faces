cmake_minimum_required(VERSION 3.16)

set(PARTITION_TABLE_CSV_PATH ${CMAKE_CURRENT_SOURCE_DIR}/partitions.csv)

# Enable C++ standard library
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(BLE_faces)