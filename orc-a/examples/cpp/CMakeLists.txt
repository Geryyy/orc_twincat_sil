# Matplot++ is a heavy FetchContent dependency (compile-time cost, pedantic
# warning suppression). It is only needed by the plotting examples, so gate
# it behind an opt-in flag. Users who just want the simpler examples can
# configure with -DBUILD_EXAMPLES=ON -DBUILD_EXAMPLES_WITH_PLOT=OFF (default).
option(BUILD_EXAMPLES_WITH_PLOT "Fetch matplot++ and build plotting examples" OFF)

if(BUILD_EXAMPLES_WITH_PLOT)
  include(FetchContent)
  FetchContent_Declare(
    matplotplusplus
    GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
    GIT_TAG origin/master)
  FetchContent_GetProperties(matplotplusplus)
  if(NOT matplotplusplus_POPULATED)
    FetchContent_Populate(matplotplusplus)
    add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
  endif()

  add_executable(interpolator_example interpolator_example.cpp)

  set(executables interpolator_example)

  foreach(exe IN LISTS executables)
    target_link_libraries(${exe} PRIVATE matplot orc)
    target_compile_features(${exe} PRIVATE cxx_std_20)
  endforeach()
else()
  message(STATUS "examples/cpp: BUILD_EXAMPLES_WITH_PLOT=OFF, skipping matplot++-based examples")
endif()
