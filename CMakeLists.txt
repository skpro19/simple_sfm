cmake_minimum_required(VERSION 2.8)
project(simple_visual_odom)

add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# SET(CMAKE_CXX_COMPILER "/usr/bin/g++")
add_definitions(
    -DBOOST_COROUTINE_NO_DEPRECATION_WARNING=1
    -DBOOST_ALL_DYN_LINK=1
    -DBOOST_ERROR_CODE_HEADER_ONLY=1

    # -D_GLIBCXX_USE_CXX11_ABI=0
)

# set(-D_GLIBCXX_USE_CXX11_ABI=0)
add_executable(simple_visual_odom
    src/vo/visual_odom.cpp
    src/vo/io.cpp
    src/sfm/main.cpp
    src/sfm/sfm.cpp
    src/vo/vo_pipeline.cpp
    src/vo/visualization_helpers.cpp)

target_link_libraries(simple_visual_odom
    ${OpenCV_LIBS}
    boost_system
    boost_filesystem

    # lstdc++
)