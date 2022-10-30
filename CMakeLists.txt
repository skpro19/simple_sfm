cmake_minimum_required(VERSION 2.8)
project(simple_sfm)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# find_package(Eigen3 REQUIRED)
# include_directories(EIGEN3_INCLUDE_DIR)

# find_package(Ceres REQUIRED)
add_definitions(
    -DBOOST_COROUTINE_NO_DEPRECATION_WARNING=1
    -DBOOST_ALL_DYN_LINK=1
    -DBOOST_ERROR_CODE_HEADER_ONLY=1
)

add_executable(simple_sfm
    src/sfm/main.cpp
    src/sfm/core_sfm.cpp
    src/sfm/sfm_helpers.cpp
    src/sfm/io.cpp
    src/sfm/frame.cpp
    src/sfm/vis.cpp
    src/sfm/sfm_util.cpp

    # src/sfm/vis.cpp
)

target_link_libraries(simple_sfm
    ${OpenCV_LIBS}

    # Eigen3::Eigen
    ceres
)