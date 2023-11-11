cmake_minimum_required(VERSION 3.5)

# set up gcc and g++ Compiler
set(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_COMPILER /usr/bin/g++)

# project name
project(safety_filter)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)

# <<<< begin to set up this project >>>>
# Set the default build type.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE Release CACHE STRING
			"Choose the type of build; options are Debug Release RelWithDebInfo MinSizeRel."
			FORCE)
	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
			Debug Release RelWithDebInfo MinSizeRel)
endif()

# Set the default installation prefix.
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING
			"My default install prefix" FORCE)
endif()

include(CMakePackageConfigHelpers)
include(CTest)
include(GNUInstallDirs)

# set the python compiler
set(Python3_EXECUTABLE "/home/kaijun/anaconda3/envs/mr1000/bin/python")
find_package (Python3 COMPONENTS Interpreter Development NumPy)
# set path to FindXXX.cmake files, for finding package
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
# find PyBind11 NumPy Eigen3
find_package(pybind11 CONFIG REQUIRED)
find_package(NumPy MODULE)
find_package(Eigen3 MODULE REQUIRED)
find_package(CDDlib MODULE REQUIRED)
find_package(OsqpEigen REQUIRED)
# Set RPATH for installed binaries.
set(CMAKE_INSTALL_RPATH
		"${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
		"${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)

list(REMOVE_DUPLICATES CMAKE_INSTALL_RPATH)

set(SAFETY_FILTER_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
set(SAFETY_FILTER_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(SAFETY_FILTER_LIBRARY_DIR ${CMAKE_INSTALL_LIBDIR})
set(SAFETY_FILTER_PYTHON_DIR
		${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages)

add_subdirectory(src/cxx)
add_subdirectory(src/python)
