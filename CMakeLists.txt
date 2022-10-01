cmake_minimum_required(VERSION 2.8)
project(simple_sfm)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

find_package(PCL 1.7 REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(... ${PCL_INCLUDE_DIRS})
link_directories(... ${PCL_LIBRARY_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

add_definitions(
    -DBOOST_COROUTINE_NO_DEPRECATION_WARNING=1
    -DBOOST_ALL_DYN_LINK=1
    -DBOOST_ERROR_CODE_HEADER_ONLY=1
)

add_executable(simple_sfm
    src/sfm/main.cpp
    src/sfm/sfm.cpp
    src/sfm/io.cpp
    src/sfm/frame.cpp
    src/sfm/sfm_util.cpp
    src/sfm/vis.cpp
    src/sfm/view.cpp
)

target_link_libraries(simple_sfm
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    Eigen3::Eigen
)