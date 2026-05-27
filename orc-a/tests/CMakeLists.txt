include(FetchContent)
# Googletest pinned to a specific commit (post-v1.14.0, pre-v1.15) so test behaviour is reproducible
# across CI runs. Bump deliberately when upstream ships a tagged release that we have validated
# locally.
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip)

if(WIN32)
  set(gtest_force_shared_crt
      ON
      CACHE BOOL "" FORCE)
endif()

FetchContent_MakeAvailable(googletest)

# Set the path to the GTest library and headers
message(STATUS "Build Tests")

file(GLOB TEST_SOURCES "*_test.cpp")

add_executable(runTests ${TEST_SOURCES})

target_link_libraries(runTests GTest::gtest_main ${PROJECT_NAME})

include(GoogleTest)
gtest_discover_tests(runTests WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set_target_properties(${PROJECT_NAME} runTests PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                                          ${CMAKE_BINARY_DIR}/bin)
