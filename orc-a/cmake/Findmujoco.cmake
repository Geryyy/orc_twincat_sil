# Findmujoco.cmake - Find MuJoCo when installed from a pip wheel or system prefix
# Usage: put this file in your project's cmake/ directory and add that
# directory to CMAKE_MODULE_PATH before calling `find_package(mujoco REQUIRED)`.
# The module prefers a CMake config package (mujocoConfig.cmake) if present, and
# otherwise locates headers/libraries under common layouts including Python venv
# site-packages where pip installs mujoco (e.g. <venv>/lib/python3.10/site-packages/mujoco).
if(mujoco_FOUND)
  return()
endif()

# Prefer a config-style package if one exists (won't call this module recursively). Using QUIET
# avoids noisy output when not present.
find_package(mujoco CONFIG QUIET)
if(mujoco_FOUND)
  # If the config package provided targets/variables, use them as-is.
  return()
endif()

find_package(
  Python
  COMPONENTS Interpreter
  REQUIRED)
# Provide a variable MUJOCO_ROOT (cache) so callers can override with -D MUJOCO_ROOT=...
if(NOT DEFINED MUJOCO_ROOT)
  set(MUJOCO_ROOT
      ""
      CACHE PATH "Root directory containing mujoco install (site-packages/mujoco or prefix)")
endif()

execute_process(
  COMMAND "${Python_EXECUTABLE}" -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))"
  OUTPUT_VARIABLE MUJOCO_ROOT
  OUTPUT_STRIP_TRAILING_WHITESPACE)

# Try to locate the headers by searching for mujoco/mujoco.h. We include some useful PATH_SUFFIXES
# that match the pip wheel layout.
find_path(
  mujoco_INCLUDE_DIR
  NAMES "mujoco/mujoco.h"
  HINTS ${MUJOCO_ROOT} ${CMAKE_PREFIX_PATH} $ENV{PYTHONPATH} $ENV{VIRTUAL_ENV}
  PATH_SUFFIXES
    "mujoco/include" "include" "include/mujoco"
    "lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/mujoco/include")

# Try find_library with several candidate names and useful PATH_SUFFIXES that include the
# site-packages layout used by pip wheels. list common names first (unversioned).
find_library(
  mujoco_LIBRARY
  NAMES mujoco libmujoco mujoco.so libmujoco.so
  HINTS ${MUJOCO_ROOT} ${CMAKE_PREFIX_PATH} $ENV{VIRTUAL_ENV}
  PATH_SUFFIXES "lib" "." "lib64"
                "lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages/mujoco")

# If find_library failed (common when the wheel only ships a versioned .so like libmujoco.so.3.3.2),
# fall back to scanning for libmujoco*.so* and pick the first hit.
if(NOT mujoco_LIBRARY)
  set(_mujo_search_paths "${MUJOCO_ROOT}" "${MUJOCO_ROOT}/lib" "${CMAKE_PREFIX_PATH}")
  # Expand CMAKE_PREFIX_PATH entries to useful site-packages patterns too
  foreach(_p IN LISTS _mujo_search_paths)
    if(IS_DIRECTORY "${_p}/site-packages/mujoco")
      file(GLOB _mujo_candidates "${_p}/site-packages/mujoco/libmujoco*.so*"
           "${_p}/site-packages/mujoco/lib/libmujoco*.so*")
      list(APPEND _mujo_glob_results ${_mujo_candidates})
    endif()
    # also check direct places relative to MUJOCO_ROOT
    if(IS_DIRECTORY "${MUJOCO_ROOT}")
      file(GLOB _mujo_candidates2 "${MUJOCO_ROOT}/libmujoco*.so*"
           "${MUJOCO_ROOT}/lib/libmujoco*.so*")
      list(APPEND _mujo_glob_results ${_mujo_candidates2})
    endif()
  endforeach()

  # Also try a broad recursive search under MUJOCO_ROOT (fast for small venv dirs)
  if(IS_DIRECTORY "${MUJOCO_ROOT}")
    file(GLOB_RECURSE _mujo_recursive "${MUJOCO_ROOT}/*libmujoco*.so*")
    list(APPEND _mujo_glob_results ${_mujo_recursive})
  endif()

  list(LENGTH _mujo_glob_results _mujo_len)
  if(_mujo_len GREATER 0)
    list(GET _mujo_glob_results 0 mujoco_LIBRARY)
  endif()
endif()

# If both header and library were found, create an imported target and set variables expected by
# consumers of find_package() style modules.
include(FindPackageHandleStandardArgs)
set(_REQUIRED_VARS "mujoco_INCLUDE_DIR;mujoco_LIBRARY")
# set variables for find_package_handle_standard_args (needs exact var names)
set(mujoco_INCLUDE_DIR
    ${mujoco_INCLUDE_DIR}
    CACHE PATH "Path to mujoco include directory")
set(mujoco_LIBRARY
    ${mujoco_LIBRARY}
    CACHE FILEPATH "Path to mujoco library")

find_package_handle_standard_args(mujoco "MuJoCo library and headers" mujoco_INCLUDE_DIR
                                  mujoco_LIBRARY)

if(mujoco_FOUND)
  # Provide common variable names used in projects
  set(MUJOCO_INCLUDE_DIR
      ${mujoco_INCLUDE_DIR}
      CACHE PATH "MuJoCo include dir" FORCE)
  set(MUJOCO_LIBRARY
      ${mujoco_LIBRARY}
      CACHE FILEPATH "MuJoCo library" FORCE)

  # Create an imported target for nicer use by callers
  if(NOT TARGET mujoco::mujoco)
    add_library(mujoco::mujoco UNKNOWN IMPORTED)
    set_target_properties(
      mujoco::mujoco PROPERTIES IMPORTED_LOCATION "${mujoco_LIBRARY}" INTERFACE_INCLUDE_DIRECTORIES
                                                                      "${mujoco_INCLUDE_DIR}")
  endif()
else()
  # Helpful guidance for the user
  message(STATUS "Findmujoco: MUJOCO_ROOT='${MUJOCO_ROOT}'")
  message(STATUS "Findmujoco: tried common venv/site-packages locations.")
  message(STATUS "Pass -D MUJOCO_ROOT=/abs/path/to/.venv/lib/pythonX.Y/site-packages/mujoco")
  message(STATUS "or set env MUJOCO_PY_MUJOCO_PATH to help the search.")
endif()
