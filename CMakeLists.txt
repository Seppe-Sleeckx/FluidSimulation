cmake_minimum_required(VERSION 3.22)
project(Fluid_Solver VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE SOURCES 
    "src/*.cpp"
    "src/*.h"
)

include(FetchContent)

FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        5.0.0
)

FetchContent_MakeAvailable(eigen)

add_executable(Fluid_Solver ${SOURCES} "src/renderer/renderer_utils.h")

target_link_libraries(Fluid_Solver PRIVATE Eigen3::Eigen)