## Global settings ##

cmake_minimum_required(VERSION 3.1)

set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake
)

if(NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX
      "${CMAKE_SOURCE_DIR}/../dist"
      CACHE STRING "Final install location." FORCE)
endif()

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## ======================================================================== ##
## Project setup.
## ======================================================================== ##

project(mitsuba_openPGL_superbuild)

include(ExternalProject)
include(GNUInstallDirs)
include(ProcessorCount)

## Setup top-level variables ##

set(installDir ${CMAKE_INSTALL_PREFIX})

get_filename_component(INSTALL_DIR_ABSOLUTE
  ${installDir} ABSOLUTE BASE_DIR ${CMAKE_CURRENT_BINARY_DIR})

ProcessorCount(PROCESSOR_COUNT)

if(NOT PROCESSOR_COUNT EQUAL 0)
  set(BUILD_JOBS ${PROCESSOR_COUNT} CACHE STRING "Number of build jobs '-j <n>'")
else()
  set(BUILD_JOBS 4 CACHE STRING "Number of build jobs '-j <n>'")
endif()

if (${CMAKE_VERSION} VERSION_GREATER 3.11.4)
  set(PARALLEL_JOBS_OPTS -j ${BUILD_JOBS})
endif()

set(DEFAULT_BUILD_COMMAND ${CMAKE_COMMAND} --build . --config Release ${PARALLEL_JOBS_OPTS})

# We must replace separators so that ExternalProject_Add can work with the
# paths we generate.
macro(add_to_prefix_path PATH)
  list(APPEND CMAKE_PREFIX_PATH ${PATH})
  string(REPLACE ";" "|" CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}")
endmacro()

## ======================================================================== ##
## Build OpenEXR
## ======================================================================== ##

option(BUILD_OPENEXR "Build OpenEXR as a part of the superbuild." ON)
if (BUILD_OPENEXR)
    include(dep_openexr)
endif()


## ======================================================================== ##
## Build Dependencies
## ======================================================================== ##

option(BUILD_DEPENDENCIES "Build Mitsuba dependecies?" ON)
if (BUILD_DEPENDENCIES)
  include(dep_eigen)
  include(dep_jpeg)
  include(dep_png)
  include(dep_xerces)
  #include(dep_freeglut)
  include(dep_glew)
endif()


## ======================================================================== ##
## Build Mitsuba
## ======================================================================== ##

option(BUILD_DEPENDENCIES_ONLY "Build only the dependency libraries (e.g., Eigen, Glew, libJpeg, libPNG, ...)." OFF)

if (NOT BUILD_DEPENDENCIES_ONLY)

  set(MITSUBA_ARGS
    -DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}
    -DBUILD_PYTHON=ON
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_INSTALL_INCLUDEDIR=${CMAKE_INSTALL_INCLUDEDIR}
    -DCMAKE_INSTALL_LIBDIR=${CMAKE_INSTALL_LIBDIR}
    -DCMAKE_INSTALL_DOCDIR=${CMAKE_INSTALL_DOCDIR}
    -DCMAKE_INSTALL_BINDIR=${CMAKE_INSTALL_BINDIR}
    $<$<BOOL:${BUILD_TBB}>:-DTBB_ROOT=${CMAKE_INSTALL_PREFIX}>
    -DOPENEXR_HOME=${CMAKE_INSTALL_PREFIX}
    -DILMBASE_HOME=${CMAKE_INSTALL_PREFIX}
  )

  ExternalProject_Add(mitsuba
    PREFIX mitsuba
    DOWNLOAD_COMMAND ""
    STAMP_DIR mitsuba/stamp
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..
    BINARY_DIR mitsuba/build
    LIST_SEPARATOR | # CMAKE_PREFIX_PATH uses alternate separator.
    CMAKE_ARGS
      ${MITSUBA_ARGS}
    BUILD_COMMAND ${DEFAULT_BUILD_COMMAND}
    BUILD_ALWAYS ON
  )

  ExternalProject_Add_StepDependencies(mitsuba
  configure
    $<$<BOOL:${BUILD_DEPENDENCIES}>:eigen>
    $<$<BOOL:${BUILD_DEPENDENCIES}>:jpeg>
    $<$<BOOL:${BUILD_DEPENDENCIES}>:png>
    $<$<BOOL:${BUILD_DEPENDENCIES}>:xerces>
    #$<$<BOOL:${BUILD_DEPENDENCIES}>:freeglut>
    $<$<BOOL:${BUILD_DEPENDENCIES}>:glew>
    $<$<BOOL:${BUILD_OPENEXR}>:openexr>
  )
endif()