cmake_minimum_required(VERSION 3.16)
project(
  orc
  LANGUAGES CXX
  VERSION 0.2.0)
include(GNUInstallDirs)

# ---------------------------------------------------------------------------
# Global compile options
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# Default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      Debug
      CACHE STRING "" FORCE)
endif()

if(NOT DEFINED CMAKE_CXX_COMPILER)
  find_program(LATEST_GCC NAMES g++-12)
  if(LATEST_GCC)
    set(CMAKE_CXX_COMPILER
        "${LATEST_GCC}"
        CACHE STRING "" FORCE)
  endif()
endif()

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wno-error=pedantic) # added as Matplot++ would not compile due to a warning
                                           # otherwise
endif()

if(MSVC)
  add_compile_options(/bigobj)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Boost 1.71...1.83 REQUIRED)
find_package(mujoco 3.3.2 EXACT REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED NO_MODULE)

# ---------------------------------------------------------------------------
# FlatBuffers (required for serialization)
# ---------------------------------------------------------------------------
option(BUILD_FLATBUFFERS "Compile Flatbuffers header files" OFF)

# FlatBuffers CMake integration for ORC This module builds FlatBuffers from the vendored source
# tree. The flatbuffers target provides the flatc compiler.

set(FLATBUFFERS_SRC_DIR
    "${CMAKE_SOURCE_DIR}/third_party/flatbuffers"
    CACHE PATH "Path to FlatBuffers source directory")

# Configure FlatBuffers build options
set(FLATBUFFERS_BUILD_TESTS
    OFF
    CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATC
    ON
    CACHE BOOL "" FORCE)
set(FLATBUFFERS_BUILD_FLATHASH
    OFF
    CACHE BOOL "" FORCE)

# Build FlatBuffers as a subdirectory The 'flatbuffers' target will be available for building flatc
# compiler
add_subdirectory(${FLATBUFFERS_SRC_DIR} ${CMAKE_CURRENT_BINARY_DIR}/flatbuffers-build
                 EXCLUDE_FROM_ALL)
# ---------------------------------------------------------------------------
# Build the shared library
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED src/dummy.cpp src/Logger.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(BUILD_FLATBUFFERS)
  set(ORC_FBS_SCHEMA "${CMAKE_SOURCE_DIR}/proto/orc_messages.fbs")
  set(ORC_FBS_OUT_DIR "${CMAKE_SOURCE_DIR}/include/orc/com/flatbuffers")
  set(ORC_FBS_GENERATED_HEADER "${ORC_FBS_OUT_DIR}/orc_messages_generated.h")

  add_custom_command(
    OUTPUT ${ORC_FBS_GENERATED_HEADER}
    COMMAND $<TARGET_FILE:flatc> --cpp --gen-mutable --gen-object-api --scoped-enums -o
            ${ORC_FBS_OUT_DIR} ${ORC_FBS_SCHEMA}
    DEPENDS flatc ${ORC_FBS_SCHEMA}
    COMMENT "Generating FlatBuffers C++ headers from ${ORC_FBS_SCHEMA}"
    VERBATIM)

  add_custom_target(generate_orc_flatbuffers_headers DEPENDS ${ORC_FBS_GENERATED_HEADER})
  add_dependencies(${PROJECT_NAME} generate_orc_flatbuffers_headers)
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC Boost::boost mujoco::mujoco Eigen3::Eigen)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
         $<BUILD_INTERFACE:${FLATBUFFERS_SRC_DIR}/include>)

# -------------------------------------------------------------------------------
# Install the library and headers
# -------------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Optionally compile the examples in the sub-directory
# ---------------------------------------------------------------------------
option(BUILD_EXAMPLES "Build example executables" OFF)

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# ---------------------------------------------------------------------------
# Optionally compile the tests in the sub-directory
# ---------------------------------------------------------------------------
option(BUILD_TESTS "Build tests" OFF)

if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# -------------------------------------------------------------------------------
# Code coverage
# -------------------------------------------------------------------------------
option(CODE_COVERAGE "Enable coverage reporting" OFF)

if(CODE_COVERAGE)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(STATUS "Building with code coverage flags")
    add_compile_options(--coverage -O0 -g)
    add_link_options(--coverage)
  else()
    message(WARNING "Code coverage not supported by this compiler")
  endif()
endif()

# ---------------------------------------------------------------------------
# Optionally build Doxygen documentation
# ---------------------------------------------------------------------------
option(BUILD_DOXYGEN "Build Doxygen documentation" OFF)

if(BUILD_DOXYGEN)
  add_subdirectory(doc)
endif()

# ---------------------------------------------------------------------------
# Optionally build python bindings
# ---------------------------------------------------------------------------
if(SKBUILD)
  add_subdirectory(python)
endif()
