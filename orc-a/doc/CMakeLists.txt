# Doxygen -> XML pipeline feeding Breathe (see doc/conf.py).
#
# The XML is produced in ${CMAKE_CURRENT_BINARY_DIR}/doxygen/xml, which for a
# default top-level build resolves to build/doc/doxygen/xml — exactly the path
# the Breathe `breathe_projects` entry in doc/conf.py points at. Keep those
# two in sync when editing either file.
#
# Enable with -DBUILD_DOXYGEN=ON (see top-level CMakeLists.txt). The Sphinx
# docs CI step in .github/workflows/docs.yml is responsible for invoking
# CMake with BUILD_DOXYGEN=ON before calling sphinx-build, so that Breathe
# sees a populated XML tree.

find_package(Doxygen)

if(NOT DOXYGEN_FOUND)
  message(WARNING "BUILD_DOXYGEN=ON but Doxygen was not found. "
                  "Breathe-based Sphinx pages will render empty API references.")
  return()
endif()

message(STATUS "Doxygen found: ${DOXYGEN_EXECUTABLE} -- ${DOXYGEN_VERSION}")

set(DOXYGEN_INPUT_DIR ${PROJECT_SOURCE_DIR}/include)
set(DOXYGEN_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/doxygen)
set(DOXYGEN_XML_DIR ${DOXYGEN_OUTPUT_DIR}/xml)
set(DOXYGEN_INDEX_FILE ${DOXYGEN_XML_DIR}/index.xml)
set(DOXYFILE_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
set(DOXYFILE_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

configure_file(${DOXYFILE_IN} ${DOXYFILE_OUT} @ONLY)

file(MAKE_DIRECTORY ${DOXYGEN_OUTPUT_DIR})

add_custom_command(
  OUTPUT ${DOXYGEN_INDEX_FILE}
  COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_OUT}
  MAIN_DEPENDENCY ${DOXYFILE_OUT}
  ${DOXYFILE_IN}
  COMMENT "Generating Doxygen XML + HTML for Breathe (${DOXYGEN_XML_DIR})"
  VERBATIM)

add_custom_target(docs ALL DEPENDS ${DOXYGEN_INDEX_FILE})

# Expose the XML dir so downstream docs tooling can pick it up if needed.
set(ORC_DOXYGEN_XML_DIR
    ${DOXYGEN_XML_DIR}
    PARENT_SCOPE)
