cmake_minimum_required(VERSION 3.15...3.27)
project(orcpy LANGUAGES CXX)

# nanobind preparation, as described in https://nanobind.readthedocs.io/en/latest/building.html
if(CMAKE_VERSION VERSION_LESS 3.18)
  set(DEV_MODULE Development)
else()
  set(DEV_MODULE Development.Module)
endif()

find_package(
  Python 3.8
  COMPONENTS Interpreter ${DEV_MODULE}
  REQUIRED)
find_package(nanobind CONFIG REQUIRED)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel"
                                               "RelWithDebInfo")
endif()

file(GLOB BINDING_SOURCES CONFIGURE_DEPENDS core/src/*.cpp)
nanobind_add_module(_core ${BINDING_SOURCES})

target_link_libraries(_core PUBLIC orc)

target_include_directories(_core PRIVATE core/src/)

target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})
target_link_options(_core PRIVATE -Wl,--no-as-needed)

# TODO RPATH for mujoco: is this the proper way (portable, ...)?
set_target_properties(_core PROPERTIES BUILD_RPATH "$ORIGIN" INSTALL_RPATH
                                                             "$ORIGIN:$ORIGIN/../../mujoco")

install(TARGETS _core LIBRARY DESTINATION orcpy/core)

install(
  DIRECTORY core/
  DESTINATION orcpy/core
  FILES_MATCHING
  PATTERN "*.py"
  PATTERN "*.pyi")

install(
  DIRECTORY robots/
  DESTINATION orcpy/robots
  FILES_MATCHING
  PATTERN "*.py"
  PATTERN "py.typed")

# ---------------------------------------------------------------------------
# Bundling .so / .dll files
# ---------------------------------------------------------------------------
add_dependencies(_core orc)

add_custom_command(
  TARGET _core
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:orc> $<TARGET_FILE_DIR:_core>)
install(FILES $<TARGET_FILE:orc> DESTINATION orcpy/core)

# nanobind stub generation
nanobind_add_stub(
  _core_stub
  MODULE
  _core
  PYTHON_PATH
  $<TARGET_FILE_DIR:_core>
  DEPENDS
  _core
  RECURSIVE
  OUTPUT_PATH
  "${CMAKE_CURRENT_BINARY_DIR}/orcpy/core"
  MARKER_FILE
  "${CMAKE_CURRENT_BINARY_DIR}/orcpy/core/py.typed")
install(TARGETS _core DESTINATION orcpy/core)

install(
  DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/orcpy/core/"
  DESTINATION "orcpy/core"
  FILES_MATCHING
  PATTERN "*.pyi"
  PATTERN "py.typed")

# Windows specific
if(WIN32)
  add_custom_command(
    TARGET _core
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:mujoco::mujoco>
            $<TARGET_FILE_DIR:_core>)
  install(FILES $<TARGET_FILE:mujoco::mujoco> DESTINATION orcpy/core)
endif()
