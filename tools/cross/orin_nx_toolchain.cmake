set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(ORIN_NX_SYSROOT "/home/user/jetson/orin-nx/sysroot" CACHE PATH "Orin NX sysroot")

set(CMAKE_SYSROOT "${ORIN_NX_SYSROOT}")
get_filename_component(_ORIN_NX_TOOLCHAIN_DIR "${CMAKE_CURRENT_LIST_FILE}" DIRECTORY)

set(CMAKE_C_COMPILER "${_ORIN_NX_TOOLCHAIN_DIR}/orin_nx_sysroot_gcc")
set(CMAKE_CXX_COMPILER "${_ORIN_NX_TOOLCHAIN_DIR}/orin_nx_sysroot_g++")
set(CMAKE_AR "${_ORIN_NX_TOOLCHAIN_DIR}/orin_nx_sysroot_ar" CACHE FILEPATH "Target ar wrapper")
set(CMAKE_RANLIB "${_ORIN_NX_TOOLCHAIN_DIR}/orin_nx_sysroot_ranlib" CACHE FILEPATH "Target ranlib wrapper")
set(CMAKE_MAKE_PROGRAM /usr/bin/gmake CACHE FILEPATH "Host make program")

set(_ORIN_NX_FIND_ROOT_PATH
  "${ORIN_NX_SYSROOT}"
  "${ORIN_NX_SYSROOT}/opt/ros/humble")
if(CMAKE_INSTALL_PREFIX)
  list(APPEND _ORIN_NX_FIND_ROOT_PATH "${CMAKE_INSTALL_PREFIX}")
endif()
set(CMAKE_FIND_ROOT_PATH ${_ORIN_NX_FIND_ROOT_PATH})

set(CMAKE_PREFIX_PATH
  "${ORIN_NX_SYSROOT}/opt/ros/humble"
  "${ORIN_NX_SYSROOT}/usr"
  "${ORIN_NX_SYSROOT}/usr/lib/aarch64-linux-gnu/cmake"
  "${CMAKE_PREFIX_PATH}")

set(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)
set(CMAKE_LINK_DEPENDS_NO_SHARED TRUE)

set(_ORIN_NX_RPATH_LINK_FLAGS
  "-Wl,-rpath-link,${ORIN_NX_SYSROOT}/lib"
  "-Wl,-rpath-link,${ORIN_NX_SYSROOT}/lib/aarch64-linux-gnu"
  "-Wl,-rpath-link,${ORIN_NX_SYSROOT}/usr/lib"
  "-Wl,-rpath-link,${ORIN_NX_SYSROOT}/usr/lib/aarch64-linux-gnu"
  "-Wl,-rpath-link,${ORIN_NX_SYSROOT}/usr/lib/aarch64-linux-gnu/blas"
  "-Wl,-rpath-link,${ORIN_NX_SYSROOT}/usr/lib/aarch64-linux-gnu/lapack"
  "-Wl,-rpath-link,${ORIN_NX_SYSROOT}/usr/lib/aarch64-linux-gnu/openblas-pthread")
string(JOIN " " _ORIN_NX_RPATH_LINK_FLAGS ${_ORIN_NX_RPATH_LINK_FLAGS})
set(CMAKE_EXE_LINKER_FLAGS_INIT "${_ORIN_NX_RPATH_LINK_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${_ORIN_NX_RPATH_LINK_FLAGS}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "${_ORIN_NX_RPATH_LINK_FLAGS}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(PKG_CONFIG_EXECUTABLE /usr/bin/pkg-config)
set(ENV{PKG_CONFIG_SYSROOT_DIR} "${ORIN_NX_SYSROOT}")
set(ENV{PKG_CONFIG_LIBDIR}
  "${ORIN_NX_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig:${ORIN_NX_SYSROOT}/usr/share/pkgconfig")

set(Python3_EXECUTABLE /usr/bin/python3 CACHE FILEPATH "Host Python for ROS 2 generators")
set(PYTHON_EXECUTABLE /usr/bin/python3 CACHE FILEPATH "Host Python for ROS 2 generators")
set(PYTHON_INCLUDE_DIRS "${ORIN_NX_SYSROOT}/usr/include/python3.10" CACHE PATH "Target Python include directory")
set(PYTHON_LIBRARIES "${ORIN_NX_SYSROOT}/usr/lib/aarch64-linux-gnu/libpython3.10.so.1.0" CACHE FILEPATH "Target Python library")
set(PYTHON_LIBRARY "${ORIN_NX_SYSROOT}/usr/lib/aarch64-linux-gnu/libpython3.10.so.1.0" CACHE FILEPATH "Target Python library")
set(PythonExtra_INCLUDE_DIRS "${ORIN_NX_SYSROOT}/usr/include/python3.10" CACHE PATH "Target Python include directory")
set(PythonExtra_LIBRARIES "${ORIN_NX_SYSROOT}/usr/lib/aarch64-linux-gnu/libpython3.10.so.1.0" CACHE FILEPATH "Target Python library")
set(PYTHON_SOABI "cpython-310-aarch64-linux-gnu" CACHE INTERNAL "Target Python SOABI")
