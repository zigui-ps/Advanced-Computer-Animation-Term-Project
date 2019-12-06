cmake_minimum_required(VERSION 3.5)
project(animation)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)

add_subdirectory (external)

set(ALL_LIBS
        ${OPENGL_LIBRARY}
        glfw
        GLEW_1130
        )

include_directories(
        common/glad
        common
        external/glew-1.13.0/include
        external/glm-0.9.7.1
        external/glfw-3.1.2/include
        external/glad
	external/assimp-5.0.0/include
        .
)
add_executable(IK
        IK/main.cpp
	common/bvh.cpp
	common/glad/glad.c
)
add_executable(final
        finalproject/main.cpp
	common/bvh.cpp
	common/glad/glad.c
	common/objloader.cpp
)
add_executable(loader
        modelloader/main.cpp
        common/glad/glad.c
        common/bvh.cpp
)
target_link_libraries(IK
        ${ALL_LIBS}
)
target_link_libraries(final
        ${ALL_LIBS}
)
target_link_libraries(loader
        ${ALL_LIBS}
	assimp
)
