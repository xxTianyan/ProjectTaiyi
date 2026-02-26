cmake_minimum_required(VERSION 3.28)
project(Taiyi)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# start Address Sanitizer when debugging
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address -g")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fsanitize=address -g")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fsanitize=address")

set(EXTERN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty")

message(STATUS "EXTERN_DIR = ${EXTERN_DIR}")

# -----------  Eigen --------------
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

# -----------  Raylib --------------
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory("${EXTERN_DIR}/raylib-master")

# -----------  ImGui --------------
set(IMGUI_DIR "${EXTERN_DIR}/imgui-master")
foreach(_f IN ITEMS imgui.cpp imgui_demo.cpp imgui_draw.cpp imgui_tables.cpp imgui_widgets.cpp)
    if(NOT EXISTS "${IMGUI_DIR}/${_f}")
        message(FATAL_ERROR "Missing ${IMGUI_DIR}/${_f}. Check extern/imgui is a full clone of Dear ImGui.")
    endif()
endforeach()

add_library(imgui STATIC
        "${IMGUI_DIR}/imgui.cpp"
        "${IMGUI_DIR}/imgui_demo.cpp"
        "${IMGUI_DIR}/imgui_draw.cpp"
        "${IMGUI_DIR}/imgui_tables.cpp"
        "${IMGUI_DIR}/imgui_widgets.cpp"
)
target_include_directories(imgui PUBLIC "${IMGUI_DIR}")

# ---------- rlImGui（Raylib + ImGui 的桥接库） ----------
set(RLIMGUI_DIR "${EXTERN_DIR}/rlImGui-main")
if(NOT EXISTS "${RLIMGUI_DIR}/rlImGui.cpp")
    message(FATAL_ERROR "Missing ${RLIMGUI_DIR}/rlImGui.cpp. Put rlImGui sources under extern/rlImGui")
endif()

add_library(rlImGui STATIC
        "${RLIMGUI_DIR}/rlImGui.cpp"
)
target_include_directories(rlImGui PUBLIC "${RLIMGUI_DIR}" "${IMGUI_DIR}")
target_link_libraries(rlImGui PUBLIC raylib imgui)

# ---------- My executable ----------
add_executable(Taiyi sample/main.cpp
        sim/Model.cpp
        sim/Scene.cpp
        sim/Builder.cpp
        raylib_plugin/CameraController.cpp
        raylib_plugin/ShaderManager.cpp
        sample/Sample.cpp
        sample/SampleRegistry.cpp
        common/Application.cpp
        collision/TriMeshCollision.cpp
        solver/VBD/VBDSolver.cpp
        raylib_plugin/RenderHelper.cpp
        collision/Collide.cpp
        geometry/Geometry.cpp
        solver/VBD/VBDParticle_Kernel.cpp
        solver/VBD/VBDRigid_Kernel.cpp
        sim/Control.h
        sample/basic_joints.hpp
        solver/mujoco/MujocoSolver.cpp)

target_include_directories(Taiyi
        PRIVATE
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

if(APPLE)
    target_compile_definitions(Taiyi PRIVATE PLATFORM_MACOS)
elseif(WIN32)
    target_compile_definitions(Taiyi PRIVATE PLATFORM_WINDOWS)
elseif(UNIX)
    target_compile_definitions(Taiyi PRIVATE PLATFORM_LINUX)
endif()

if(MSVC)
    target_compile_options(Taiyi PRIVATE /W4 /permissive- /utf-8 /Zc:__cplusplus)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(Taiyi PRIVATE -Wall -Wextra -Wpedantic)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(Taiyi PRIVATE -Wall -Wextra -Wpedantic)
endif()

