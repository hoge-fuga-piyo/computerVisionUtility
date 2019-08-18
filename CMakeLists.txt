###############################
# Compiler setting
###############################
cmake_minimum_required(VERSION 3.8)

###############################
# External libraries
###############################
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Exiv2
find_path(Exiv2_INCLUDE_DIR NAMES exiv2.hpp)
find_library(Exiv2_LIBRARY NAMES exiv2.lib)
include_directories(${Exiv2_INCLUDE_DIR})

###############################
# For unit test
###############################
# Google Test
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

###############################
# Sources
###############################
set(PROJ_SHARED_DIR ${CMAKE_SOURCE_DIR}/src)
file(GLOB PROJ_SHARED_SOURCE ${PROJ_SHARED_DIR}/*.cpp)
file(GLOB PROJ_SHARED_INCLUDE ${PROJ_SHARED_DIR}/*.h ${PROJ_SHARED_DIR}/*.hpp)

###############################
# Test
###############################
set(PROJ_TEST_DIR ${CMAKE_SOURCE_DIR}/test)
add_subdirectory(${PROJ_TEST_DIR})
