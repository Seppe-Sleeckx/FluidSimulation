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

add_library(Fluid_Solver ${SOURCES} "src/empty.cpp")

target_link_libraries(Fluid_Solver PRIVATE Eigen3::Eigen)


#Emscriptem build stuff
if(EMSCRIPTEN)

    target_compile_options(Fluid_Solver PRIVATE -pthread)

    add_executable(fluid_wasm src/empty.cpp) #needed because emscritpem can only generate WASM from executables, not libraries so we add an empty file where all libraries get pulled in as final link target

    target_link_libraries(fluid_wasm
        PRIVATE Fluid_Solver
    )

    target_compile_options(fluid_wasm PRIVATE -pthread)

    set_target_properties(fluid_wasm PROPERTIES
        OUTPUT_NAME "fluid_solver"
        SUFFIX ".js"
    )

    target_link_options(fluid_wasm PRIVATE
        "-sWASM=1"
        "-sMODULARIZE=1"
        "-sEXPORT_ES6=1"
        "-sENVIRONMENT=web,worker"
        "-sALLOW_MEMORY_GROWTH=1"
        "-pthread"
        "-sUSE_PTHREADS=1"
        "-sPTHREAD_POOL_SIZE=navigator.hardwareConcurrency"
        "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap', 'FS', 'HEAP32', 'HEAPF32']"
        "-sEXPORTED_FUNCTIONS=['_Create','_Destroy','_Initialize','_Simulate','_GetParticleCount','_GetParticlePositions', '_GetGridXDimension', '_GetGridYDimension', '_GetGridZDimension', '_WriteMeasurementsToFile', '_malloc', '_free']"
    )

endif()