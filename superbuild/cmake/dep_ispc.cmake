## Copyright 2019 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

set(COMPONENT_NAME ispc)

set(COMPONENT_PATH ${INSTALL_DIR_ABSOLUTE})
if (INSTALL_IN_SEPARATE_DIRECTORIES)
  set(COMPONENT_PATH ${INSTALL_DIR_ABSOLUTE}/${COMPONENT_NAME})
endif()

set(ISPC_VERSION "1.23.0" CACHE STRING "Which version of ISPC to download?")
if (ISPC_VERSION STREQUAL "1.23.0")
  set(ISPC_SOURCE_HASH "d665f0d4160a8777cf478e5f806478bb817daa40b3111f366e07dc6615b0ebe7")
  if (APPLE)
    if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm64|aarch64")
      set(ISPC_HASH "2cf144aaa6d8117e3a9e0782984fa4cb45127387dd4fb385b187115d6c1a0d68")
    else()
      set(ISPC_HASH "b9e6dcc045f5a2e29a6c43354b6a747c4486a341608d785f5f99eca8ac207a72")
    endif()
  elseif (WIN32)
    set(ISPC_HASH "709350902381968ee58fd67e9aed63df99b1313bc55a94195977bcc8d90bdced")
  else()
    if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm64|aarch64")
      set(ISPC_HASH "f82a0464b8d172c2c015f4252154651c6a25061f7a47ffb7f7f5b785eddfc9cc")
    else()
      set(ISPC_HASH "9183b2dd128fa5ca30ab24ca2cf6730a11572e4efd82fd284d167109f9c9d077")
    endif()
  endif()
endif()

set(ISPCRT_LIBDIR "lib")
if (APPLE)
  set(ISPC_OSSUFFIX "macOS.${CMAKE_SYSTEM_PROCESSOR}.tar.gz")
elseif(WIN32)
  set(ISPC_OSSUFFIX "windows.zip")
else()
  if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm64|aarch64")
    set(ISPC_OSSUFFIX "linux.aarch64.tar.gz")
  else()
    set(ISPC_OSSUFFIX "linux-oneapi.tar.gz")
    set(ISPCRT_LIBDIR "lib64")
  endif()
endif()

set(ISPC_URL "https://github.com/ispc/ispc/releases/download/v${ISPC_VERSION}/ispc-v${ISPC_VERSION}-${ISPC_OSSUFFIX}")

if (ISPC_HASH)
  set(ISPC_URL_HASH URL_HASH SHA256=${ISPC_HASH})
endif()

set(ISPC_INSTALL copy_directory <SOURCE_DIR>/ ${COMPONENT_PATH})


ExternalProject_Add(${COMPONENT_NAME}
  PREFIX ${COMPONENT_NAME}
  STAMP_DIR ${COMPONENT_NAME}/stamp
  SOURCE_DIR ${COMPONENT_NAME}/src
  BINARY_DIR ${COMPONENT_NAME}
  URL ${ISPC_URL}
  ${ISPC_URL_HASH}
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND "${CMAKE_COMMAND}" -E ${ISPC_INSTALL}
  BUILD_ALWAYS OFF
)

set(ISPC_PATH "${CMAKE_INSTALL_PREFIX}/bin/ispc${CMAKE_EXECUTABLE_SUFFIX}")
