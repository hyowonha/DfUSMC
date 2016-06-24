cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE Release)
set(DCMAKE_BUILD_TYPE Release)

project(DfUSMC)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(DfUSMC main.cpp DfUSMC.hpp DfUSMC.cpp)
target_link_libraries(DfUSMC ${OpenCV_LIBS} ${CERES_LIBRARIES})
