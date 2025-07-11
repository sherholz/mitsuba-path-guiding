option(USE_PYTHON "enable python support" FALSE)
option(USE_PYTHON2 "if USE_PYTHON enabled, use python2 instead of python3" FALSE)

set (EP_BOOST "boost")

set(BOOST_BUILD_COMMAND ./b2 install --layout=tagged --prefix=${CMAKE_INSTALL_PREFIX} --build-dir=${CMAKE_CURRENT_BINARY_DIR}/build -j${BUILD_JOBS} address-model=64 link=shared runtime-link=shared threading=multi variant=release --with-atomic --with-program_options --with-regex --with-date_time --with-system --with-thread --with-iostreams --with-filesystem --with-serialization --with-wave --with-chrono)

set(BOOST_PYTHON_VERSIONS)

if(APPLE)
  set(BOOST_URL https://boostorg.jfrog.io/artifactory/main/release/1.78.0/source/boost_1_78_0.tar.gz)
else()
  set(BOOST_URL https://boostorg.jfrog.io/artifactory/main/release/1.74.0/source/boost_1_74_0.tar.gz)
endif()

if (USE_PYTHON)
  message("writing file: " ${CMAKE_CURRENT_BINARY_DIR}/source/boost/python-config.jam)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/source/boost/python-config.jam)
  if (USE_PYTHON2)
    find_package(PythonInterp 2.7 REQUIRED)
  else()
    find_package(PythonInterp 3.0 REQUIRED)
  endif()
  message("python: " ${PYTHON_EXECUTABLE})
  file(
    APPEND ${CMAKE_CURRENT_BINARY_DIR}/source/boost/python-config.jam
    "using python : ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR} : \"${PYTHON_EXECUTABLE}\" ; \n"
  )
  list(APPEND BOOST_PYTHON_VERSIONS "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
  list(JOIN BOOST_PYTHON_VERSIONS "," BOOST_PYTHON_VERSIONS)

  set(BOOST_BUILD_COMMAND ${BOOST_BUILD_COMMAND}  --user-config=${CMAKE_CURRENT_BINARY_DIR}/source/boost/python-config.jam python=${BOOST_PYTHON_VERSIONS} --with-python)

endif()

ExternalProject_Add (
  ${EP_BOOST}

  PREFIX         ${EP_BOOST}/source/boost
  BUILD_IN_SOURCE 1
  URL ${BOOST_URL}
  BUILD_ALWAYS   OFF

  LIST_SEPARATOR | # Use the alternate list separator

  CONFIGURE_COMMAND ./bootstrap.sh --prefix=${CMAKE_INSTALL_PREFIX}
  BUILD_COMMAND ${BOOST_BUILD_COMMAND}
  INSTALL_COMMAND ""
  INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
)

