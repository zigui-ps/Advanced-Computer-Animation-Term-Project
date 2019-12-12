cmake_minimum_required(VERSION 3.8)
project(sandbox2)

add_definitions(-DBT_USE_DOUBLE_PRECISION)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g")
find_package(Bullet REQUIRED)

message(STATUS "${BULLET_INCLUDE_DIR}")
message(STATUS "${BULLET_LIBRARIES}")
find_package(OpenGL REQUIRED)
include_directories("${BULLET_INCLUDE_DIR}")
include_directories("${OPENGL_INCLUDE_DIR}")
include_directories("./")

file(GLOB src "*.cpp")
add_executable(sandbox ${src})
target_link_libraries(sandbox ${BULLET_LIBRARIES} ${OPENGL_LIBRARIES} -lGL -lGLU -lglut)
