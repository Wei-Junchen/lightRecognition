cmake_minimum_required(VERSION 3.10)
project(demo)

include_directories(${CMAKE_SOURCE_DIR}/inc)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(demo src/main.cpp)
target_link_libraries(demo ${OpenCV_LIBS} Eigen3::Eigen)
