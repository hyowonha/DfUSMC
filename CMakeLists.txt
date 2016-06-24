cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE Release)
set(DCMAKE_BUILD_TYPE Release)

project(DfUSMC)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/Yang_cvpr12)

add_executable(DfUSMC main.cpp DfUSMC.cpp ${PROJECT_SOURCE_DIR}/Yang_cvpr12/qx_tree_upsampling.cpp ${PROJECT_SOURCE_DIR}/Yang_cvpr12/qx_tree_filter.cpp ${PROJECT_SOURCE_DIR}/Yang_cvpr12/qx_mst_kruskals_image.cpp ${PROJECT_SOURCE_DIR}/Yang_cvpr12/qx_basic.cpp ${PROJECT_SOURCE_DIR}/Yang_cvpr12/ctmf.c)

target_link_libraries(DfUSMC ${OpenCV_LIBS} ${CERES_LIBRARIES})
