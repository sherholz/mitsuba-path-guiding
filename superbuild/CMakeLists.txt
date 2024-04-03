## Global settings ##

cmake_minimum_required(VERSION 3.12)

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
## Build TBB
## ======================================================================== ##

option(BUILD_TBB "Build Intel Threading Building Blocks or search in environment?" ON)
option(BUILD_TBB_FROM_SOURCE "Build Intel Threading Building Blocks from source?" OFF)
if (BUILD_TBB)
  set(TBB_VERSION "2021.11.0" CACHE STRING "TBB version to download")
  if (BUILD_TBB_FROM_SOURCE)
    if (TBB_VERSION VERSION_LESS 2021)
      message(FATAL_ERROR "Only TBB 2021 and later are supported when building TBB from source")
    endif()

    string(REGEX REPLACE "(^[0-9]+\.[0-9]+\.[0-9]+$)" "v\\1" TBB_ARCHIVE ${TBB_VERSION})
    set(_TBB_URL "https://github.com/oneapi-src/oneTBB/archive/refs/tags/${TBB_ARCHIVE}.zip")
    set(_TBB_HASH "2f0bfce641d238e80798fc48397d43821bd977d49c4e03bc785be363b7ab4742")
  else()
    if (TBB_VERSION VERSION_LESS 2021)
      set(TBB_BASE_URL "https://github.com/oneapi-src/oneTBB/releases/download/v${TBB_VERSION}/tbb-${TBB_VERSION}")
    else()
      set(TBB_BASE_URL "https://github.com/oneapi-src/oneTBB/releases/download/v${TBB_VERSION}/oneapi-tbb-${TBB_VERSION}")
    endif()
    if (APPLE)
      set(_TBB_URL "${TBB_BASE_URL}-mac.tgz")
      set(_TBB_HASH "360bcb20bcdcd01e8492c32bba6d5d5baf4bc83f77fb9dbf1ff701ac816e3b44")
      set(TBB_LIB_SUBDIR "")
    elseif(WIN32)
      set(_TBB_URL "${TBB_BASE_URL}-win.zip")
      set(_TBB_HASH "02f0e93600fba69bb1c00e5dd3f66ae58f56e5410342f6155455a95ba373b1b6")
      set(TBB_LIB_SUBDIR "intel64/vc14")
    else()
      set(_TBB_URL "${TBB_BASE_URL}-lin.tgz")
      set(_TBB_HASH "95659f4d7b1711c41ffa190561d4e5b6841efc8091549661c7a2e6207e0fa79b")
      set(TBB_LIB_SUBDIR "intel64/gcc4.8")
    endif()
  endif()
  set(TBB_URL "${_TBB_URL}" CACHE STRING "URL of the TBB archive.")
  set(TBB_HASH "${_TBB_HASH}" CACHE STRING "SHA256 hash of the TBB archive.")
  include(dep_tbb)
endif()

## ======================================================================== ##
## Build Dependencies
## ======================================================================== ##

option(BUILD_DEPENDENCIES "Build Mitsuba dependecies?" ON)
if (DEPS_DIR)
  set(BUILD_DEPENDENCIES OFF)
endif()
if (BUILD_DEPENDENCIES)
  include(dep_boost)
  include(dep_eigen)
  include(dep_jpeg)
  include(dep_png)
  include(dep_xerces)
  #include(dep_freeglut)
  include(dep_openexr)
  include(dep_glew)
  include(dep_blosc)
  include(dep_vdb)
endif()

## ======================================================================== ##
## Build OpenImageDenoise
## ======================================================================== ##

option(BUILD_OIDN "Build OpenImageDenoise as a part of the superbuild." ON)
option(BUILD_OIDN_FROM_SOURCE "Build OpenImageDenoise or use pre-built version?" ON)
set(OIDN_VERSION "2.2.0" CACHE STRING "Which version of OpenImageDenoise to build?")

if(BUILD_OIDN_FROM_SOURCE)
  include(dep_ispc)
endif()
if (BUILD_OIDN)
  include(dep_oidn)
endif()



## ======================================================================== ##
## Build Mitsuba
## ======================================================================== ##

option(BUILD_DEPENDENCIES_ONLY "Build only the dependency libraries (e.g., Eigen, Glew, libJpeg, libPNG, ...)." OFF)

if (NOT BUILD_DEPENDENCIES_ONLY)

  if(NOT DEPS_DIR)
    set(DEPS_DIR ${CMAKE_INSTALL_PREFIX})
  endif()

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
    $<$<BOOL:${BUILD_TBB}>:-DTBB_ROOT=${DEPS_DIR}>
    -Dopenpgl_DIR=${openpgl_DIR}
    -DOpenImageDenoise_DIR=${DEPS_DIR}"/lib/cmake/OpenImageDenoise-"${OIDN_VERSION}
    -DOPENEXR_HOME=${DEPS_DIR}
    -DILMBASE_HOME=${DEPS_DIR}
    -DGLEW_ROOT_DIR=${DEPS_DIR}
    -DMITSUBA_DEPS=${DEPS_DIR}
  )
if(APPLE)
  if(OpenMP_ROOT)
    set(MITSUBA_ARGS ${MITSUBA_ARGS}
      -DOpenMP_ROOT:PATH=${OpenMP_ROOT}
    )
  else()
    set(MITSUBA_ARGS ${MITSUBA_ARGS}
      -DOpenMP_ROOT:PATH=/opt/homebrew/opt/libomp
    )
  endif()
endif()
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
    $<$<BOOL:${BUILD_DEPENDENCIES}>:boost>
    $<$<BOOL:${BUILD_DEPENDENCIES}>:eigen>
    $<$<BOOL:${BUILD_DEPENDENCIES}>:jpeg>
    $<$<BOOL:${BUILD_DEPENDENCIES}>:png>
    $<$<BOOL:${BUILD_DEPENDENCIES}>:xerces>
    #$<$<BOOL:${BUILD_DEPENDENCIES}>:freeglut>
    $<$<BOOL:${BUILD_DEPENDENCIES}>:glew>
    $<$<BOOL:${BUILD_DEPENDENCIES}>:blosc>
    $<$<BOOL:${BUILD_DEPENDENCIES}>:openvdb>
    $<$<BOOL:${BUILD_OPENEXR}>:openexr>
    $<$<BOOL:${BUILD_TBB}>:tbb>
    $<$<BOOL:${BUILD_OIDN}>:oidn>
  )
endif()
