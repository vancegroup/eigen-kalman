# CMake cross-platform build system
# 2009-2010 Ryan Pavlik <rpavlik@iastate.edu>
# http://academic.cleardefinition.com/
# Iowa State University HCI Graduate Program/VRAC

cmake_minimum_required(VERSION 2.6.2)

# Set package properties
project(eigenKalman)

set(CPACK_PACKAGE_VENDOR "Iowa State University")
set(CPACK_PACKAGE_CONTACT "Ryan Pavlik <rpavlik@iastate.edu>")
set(CPACK_PACKAGE_VERSION_MAJOR "1")
set(CPACK_PACKAGE_VERSION_MINOR "0")
set(CPACK_PACKAGE_VERSION_PATCH "0")
set(CPACK_PACKAGE_VERSION
	"${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${CPACK_PACKAGE_VERSION}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${CPACK_PACKAGE_VERSION}-src")

###
# Set up options
###

# Define Simple Options
option(BUILD_WITH_EIGEN3 "Build using Eigen3 pre-release, instead of Eigen2" off)

###
# Perform build configuration of dependencies
###

if(BUILD_WITH_EIGEN3)
	include_directories("${CMAKE_CURRENT_SOURCE_DIR}/third-party/eigen")
else()
	include_directories("${CMAKE_CURRENT_SOURCE_DIR}/third-party/eigen-20")
endif()

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

# Locally-developed modules dist'ed with this app - always have this first.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(UseBackportedModules)
include(CppcheckTargets)
include(DoxygenTargets)
include(EnableExtraCompilerWarnings)
include(MSVCMultipleProcessCompile)

include(CTest)
include(CreateDashboardScripts)

include(SetDefaultBuildType)
set_default_build_type(RelWithDebInfo)

###
# Build the project
###

if(WIN32)
	# Fix build: windows.h has annoying defines that otherwise break Eigen
	add_definitions(-DNOMINMAX)
endif()

add_executable(kftest eigenkf/KalmanFilter.h kftest.cpp)