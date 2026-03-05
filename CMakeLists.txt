cmake_minimum_required(VERSION 3.28)
project(Taiyi)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(BUILD_LEGACY_TAIYI "Build legacy Taiyi renderer-based executable" OFF)

set(EXTERN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty")
set(EIGEN_DIR "${EXTERN_DIR}/eigen-3.4.0")
if(NOT EXISTS "${EIGEN_DIR}/Eigen/Core")
    message(FATAL_ERROR
            "Eigen not found in ${EIGEN_DIR}\n"
            "Expected layout:\n"
            "  ${EIGEN_DIR}/Eigen/\n"
            "  ${EIGEN_DIR}/unsupported/\n")
endif()

add_library(eigen_headers INTERFACE)
target_include_directories(eigen_headers INTERFACE "${EIGEN_DIR}")
add_library(Eigen3::Eigen ALIAS eigen_headers)

add_executable(tiny_vbd_demo
        tiny_vbd/main.cpp
        tiny_vbd/builder/Builder.cpp
        tiny_vbd/collision/Collision.cpp
        tiny_vbd/contact/Contact.cpp
        tiny_vbd/solver/Solver.cpp)

target_include_directories(tiny_vbd_demo
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(tiny_vbd_demo PRIVATE Eigen3::Eigen)

if(MSVC)
    target_compile_options(tiny_vbd_demo PRIVATE /W4 /permissive- /utf-8 /Zc:__cplusplus)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(tiny_vbd_demo PRIVATE -Wall -Wextra -Wpedantic)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(tiny_vbd_demo PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(BUILD_LEGACY_TAIYI)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    add_subdirectory("${EXTERN_DIR}/raylib-master")

    set(IMGUI_DIR "${EXTERN_DIR}/imgui-master")
    add_library(imgui STATIC
            "${IMGUI_DIR}/imgui.cpp"
            "${IMGUI_DIR}/imgui_demo.cpp"
            "${IMGUI_DIR}/imgui_draw.cpp"
            "${IMGUI_DIR}/imgui_tables.cpp"
            "${IMGUI_DIR}/imgui_widgets.cpp")
    target_include_directories(imgui PUBLIC "${IMGUI_DIR}")

    set(RLIMGUI_DIR "${EXTERN_DIR}/rlImGui-main")
    add_library(rlImGui STATIC "${RLIMGUI_DIR}/rlImGui.cpp")
    target_include_directories(rlImGui PUBLIC "${RLIMGUI_DIR}" "${IMGUI_DIR}")
    target_link_libraries(rlImGui PUBLIC raylib imgui)

    add_executable(Taiyi sample/main.cpp
            sim/Model.cpp sim/Scene.cpp sim/Builder.cpp
            raylib_plugin/CameraController.cpp raylib_plugin/ShaderManager.cpp
            sample/Sample.cpp sample/SampleRegistry.cpp
            common/Application.cpp collision/TriMeshCollision.cpp
            solver/VBD/VBDSolver.cpp raylib_plugin/RenderHelper.cpp
            collision/Collide.cpp geometry/Geometry.cpp
            solver/VBD/VBDParticle_Kernel.cpp solver/VBD/VBDRigid_Kernel.cpp
            solver/Order/OrderSolver.cpp)

    target_include_directories(Taiyi PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/collision
            ${CMAKE_CURRENT_SOURCE_DIR}/common
            ${CMAKE_CURRENT_SOURCE_DIR}/sim
            ${CMAKE_CURRENT_SOURCE_DIR}/raylib_plugin
            ${CMAKE_CURRENT_SOURCE_DIR}/sample
            ${CMAKE_CURRENT_SOURCE_DIR}/solver
            ${CMAKE_CURRENT_SOURCE_DIR}/solver/VBD
            ${CMAKE_CURRENT_SOURCE_DIR}/imgui_plugin
            ${CMAKE_CURRENT_SOURCE_DIR}/geometry)

    target_link_libraries(Taiyi PRIVATE Eigen3::Eigen rlImGui imgui raylib)
endif()
