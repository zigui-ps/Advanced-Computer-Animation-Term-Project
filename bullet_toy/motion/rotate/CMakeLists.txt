cmake_minimum_required(VERSION 3.5.1)
project (AdvancedAnmation)

add_compile_options(-std=gnu++17 -Ofast -g)
include_directories("${CMAKE_HOME_DIRECTORY}/include")

find_package (Eigen3 REQUIRED NO_MODULE)
file(GLOB srcs A.cpp)

add_executable (HW4 ${srcs})
target_link_libraries (HW4 Eigen3::Eigen GL GLU GLEW glut tinyxml)
