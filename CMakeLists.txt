cmake_minimum_required(VERSION 3.25)

option(USE_SAMPLES "Build the PC sample host (SDL3 + ImGui)" OFF)
if(USE_SAMPLES)
    list(APPEND VCPKG_MANIFEST_FEATURES "samples")
endif()
option(USE_TESTING "Build the unit tests (GoogleTest)" OFF)
if(USE_TESTING)
    list(APPEND VCPKG_MANIFEST_FEATURES "tests")
endif()

project(Gpr925Physics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CompilerWarnings)

# devkitPro's NDS.cmake sets CMAKE_SYSTEM_NAME to NintendoDS
if(CMAKE_SYSTEM_NAME STREQUAL "NintendoDS")
    set(GPR925_NDS ON)
    set(GPR925_DEFAULT_SCALAR "FIXED")
else()
    set(GPR925_NDS OFF)
    set(GPR925_DEFAULT_SCALAR "FLOAT")
endif()

set(GPR925_SCALAR "${GPR925_DEFAULT_SCALAR}" CACHE STRING
    "Type of gpr925::Scalar: FLOAT (PC default) or FIXED (20.12 fixed point, DS default)")
set_property(CACHE GPR925_SCALAR PROPERTY STRINGS FLOAT FIXED)
if(NOT GPR925_SCALAR MATCHES "^(FLOAT|FIXED)$")
    message(FATAL_ERROR "GPR925_SCALAR must be FLOAT or FIXED (got '${GPR925_SCALAR}')")
endif()
message(STATUS "Gpr925Physics: gpr925::Scalar is ${GPR925_SCALAR}")

# No exceptions or RTTI in the engine and the samples (the DS has no room for them); the tests keep them for
# GoogleTest. On the DS the code is built in ARM mode: in Thumb mode a 64-bit multiply (every Fixed multiply) is a
# call to __aeabi_lmul, in ARM mode it is one SMULL instruction.
function(gpr925_set_engine_options target)
    gpr925_set_project_warnings(${target})
    if(NOT MSVC)
        target_compile_options(${target} PRIVATE -fno-exceptions -fno-rtti)
    endif()
    if(GPR925_NDS)
        target_compile_options(${target} PRIVATE -marm)
    endif()
endfunction()

# EngineLib (the shared maths library, a git submodule) follows our warnings-as-errors setting. Its warning flags are
# INTERFACE: they apply to every target that links it.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/externals/EngineLib/CMakeLists.txt")
    message(FATAL_ERROR "externals/EngineLib is empty: run 'git submodule update --init --recursive'")
endif()
set(WARNINGS_AS_ERRORS ${GPR925_WARNINGS_AS_ERRORS} CACHE BOOL "Treat compiler warnings as errors" FORCE)
add_subdirectory(externals/EngineLib)

# The engine: your physics code goes in include/gpr925 and src
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS src/*.cc include/*.h)
add_library(Gpr925Physics STATIC ${SRC_FILES})
target_include_directories(Gpr925Physics PUBLIC include/)
target_link_libraries(Gpr925Physics PUBLIC EngineLib)
target_compile_definitions(Gpr925Physics PUBLIC GPR925_SCALAR_${GPR925_SCALAR})
if(GPR925_NDS)
    target_compile_definitions(Gpr925Physics PUBLIC GPR925_PLATFORM_NDS)
endif()
if(MSVC)
    target_compile_definitions(Gpr925Physics PRIVATE _USE_MATH_DEFINES=1)
endif()
gpr925_set_engine_options(Gpr925Physics)

# The DS builds the samples library without USE_SAMPLES: the nds preset keeps it OFF to avoid vcpkg's SDL.
# tests/samples_test.cc links it too.
if(USE_SAMPLES OR USE_TESTING OR GPR925_NDS)
    add_subdirectory(main/samples)
endif()

if(USE_SAMPLES)
    find_package(SDL3 CONFIG REQUIRED)
    find_package(imgui CONFIG REQUIRED)
    # PC sample host: SDL window, ImGui menu, SdlSampleDrawer
    file(GLOB SDL_HOST_FILES CONFIGURE_DEPENDS main/sdl/*.cc main/sdl/*.h)
    add_executable(Gpr925Sample ${SDL_HOST_FILES})
    target_link_libraries(Gpr925Sample PRIVATE Gpr925Samples SDL3::SDL3 imgui::imgui)
    gpr925_set_project_warnings(Gpr925Sample)
endif()

if(USE_TESTING)
    find_package(GTest CONFIG REQUIRED)
    file(GLOB TEST_FILES CONFIGURE_DEPENDS tests/*.cc)
    enable_testing()
    include(GoogleTest)
    # One executable per test file
    foreach(TEST_FILE ${TEST_FILES})
        get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
        add_executable(${TEST_NAME} ${TEST_FILE})
        target_link_libraries(${TEST_NAME} PRIVATE Gpr925Samples GTest::gtest GTest::gtest_main)
        gpr925_set_project_warnings(${TEST_NAME})
        set_target_properties(${TEST_NAME} PROPERTIES FOLDER Tests/)
        gtest_discover_tests(${TEST_NAME})
    endforeach()
endif()

if(GPR925_NDS)
    add_subdirectory(nds)
endif()
