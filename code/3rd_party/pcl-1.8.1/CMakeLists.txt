cmake_minimum_required(VERSION 3.12)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "possible configurations" FORCE)

# In case the user does not setup CMAKE_BUILD_TYPE, assume it's RelWithDebInfo
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "build type default to RelWithDebInfo, set to Release to improve performance" FORCE)
endif("${CMAKE_BUILD_TYPE}" STREQUAL "")

project(PCL)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

option(PCL_SHARED_LIBS  "Build PCL as shared libraries"  OFF)

### ---[ Find universal dependencies
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

# ---[ Include pkgconfig
#include (FindPkgConfig)

# ---[ Release/Debug specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  add_definitions("-DBOOST_DISABLE_ASSERTS -DEIGEN_NO_DEBUG")
endif()
if(WIN32 AND NOT MINGW)
  if(NOT DEFINED CMAKE_DEBUG_POSTFIX)
    set(CMAKE_DEBUG_POSTFIX "_debug")
  endif()
  if(NOT DEFINED CMAKE_RELEASE_POSTFIX)
    set(CMAKE_RELEASE_POSTFIX "_release")
  endif()
  if(NOT DEFINED CMAKE_RELWITHDEBINFO_POSTFIX)
    set(CMAKE_RELWITHDEBINFO_POSTFIX "_release")
  endif()
  if(NOT DEFINED CMAKE_MINSIZEREL_POSTFIX)
    set(CMAKE_MINSIZEREL_POSTFIX "_release")
  endif()
endif()

# ---[ special maintainer mode
SET(CMAKE_CXX_FLAGS_MAINTAINER "-pedantic -Wno-variadic-macros -Weffc++ -Wno-long-long" CACHE STRING
    "Flags used by the C++ compiler during maintainer builds."
    FORCE)
SET(CMAKE_C_FLAGS_MAINTAINER "-pedantic -Wno-variadic-macros -Weffc++ -Wno-long-long" CACHE STRING
    "Flags used by the C compiler during maintainer builds."
    FORCE)
SET(CMAKE_EXE_LINKER_FLAGS_MAINTAINER
    "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
    "Flags used for linking binaries during maintainer builds."
    FORCE)
SET(CMAKE_SHARED_LINKER_FLAGS_MAINTAINER
    "-Wl,--warn-unresolved-symbols,--warn-once" CACHE STRING
    "Flags used by the shared libraries linker during maintainer builds."
    FORCE)
MARK_AS_ADVANCED(
    CMAKE_CXX_FLAGS_MAINTAINER
    CMAKE_C_FLAGS_MAINTAINER
    CMAKE_EXE_LINKER_FLAGS_MAINTAINER
    CMAKE_SHARED_LINKER_FLAGS_MAINTAINER )
# Update the documentation string of CMAKE_BUILD_TYPE for GUIs
SET(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel Maintainer."
    FORCE)

# Compiler identification
# Define a variable CMAKE_COMPILER_IS_X where X is the compiler short name.
# Note: CMake automatically defines one for GNUCXX, nothing to do in this case.
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(CMAKE_COMPILER_IS_CLANG 1)
elseif(__COMPILER_PATHSCALE)
  set(CMAKE_COMPILER_IS_PATHSCALE 1)
elseif(MSVC)
  set(CMAKE_COMPILER_IS_MSVC 1)
endif()

include("${PCL_SOURCE_DIR}/cmake/pcl_verbosity.cmake")
include("${PCL_SOURCE_DIR}/cmake/pcl_targets.cmake")
include("${PCL_SOURCE_DIR}/cmake/pcl_options.cmake")

# Enable verbose timing display?
if(CMAKE_TIMING_VERBOSE AND UNIX)
  set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_SOURCE_DIR}/cmake/custom_output.sh")
endif(CMAKE_TIMING_VERBOSE AND UNIX)

# check for SSE flags
include("${PCL_SOURCE_DIR}/cmake/pcl_find_sse.cmake")
if (PCL_ENABLE_SSE)
  PCL_CHECK_FOR_SSE()
endif (PCL_ENABLE_SSE)

# ---[ Unix/Darwin/Windows specific flags
if(CMAKE_COMPILER_IS_GNUCXX)
  if("${CMAKE_CXX_FLAGS}" STREQUAL "")
    SET(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unknown-pragmas -fno-strict-aliasing -Wno-format-extra-args -Wno-sign-compare -Wno-invalid-offsetof -Wno-conversion ${SSE_FLAGS}")

    # Enable -Wabi for GCC > 4.3, and -Wno-deprecated for GCC < 4.3
    # to disable a lot of warnings which are not fixable
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if (GCC_VERSION VERSION_GREATER 4.3)
      message(STATUS "-- GCC > 4.3 found, enabling -Wabi")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wabi")
    else()
      message(STATUS "-- GCC < 4.3 found, enabling -Wno-deprecated")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")
    endif ()
  endif()

  if(NOT ANDROID)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
  endif(NOT ANDROID)

  if("${CMAKE_SHARED_LINKER_FLAGS}" STREQUAL "" AND NOT CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--as-needed")
  endif()

  if(WIN32)
    if(PCL_SHARED_LIBS)
      SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--export-all-symbols -Wl,--enable-auto-import")
      if (MINGW)
        add_definitions("-DBOOST_THREAD_USE_LIB")
        SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--allow-multiple-definition")
      endif()
    else(PCL_SHARED_LIBS)
      add_definitions("-DBOOST_LIB_DIAGNOSTIC -DBOOST_THREAD_USE_LIB")
    endif(PCL_SHARED_LIBS)
  endif()
endif()

if(CMAKE_COMPILER_IS_MSVC)
  add_definitions("-DBOOST_ALL_NO_LIB -D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -DNOMINMAX -DPCL_ONLY_CORE_POINT_TYPES /bigobj ${SSE_DEFINITIONS}")
  if("${CMAKE_CXX_FLAGS}" STREQUAL " /DWIN32 /D_WINDOWS /W3 /GR /EHsc")	# Check against default flags
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj /EHsc /fp:precise /wd4800 /wd4521 /wd4251 /wd4275 /wd4305 /wd4355 ${SSE_FLAGS}")

    # Add extra code generation/link optimizations
    if(CMAKE_MSVC_CODE_LINK_OPTIMIZATION)
      SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /GL")
      SET(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /LTCG /OPT:REF")
      SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /LTCG")
    endif(CMAKE_MSVC_CODE_LINK_OPTIMIZATION)
    # /MANIFEST:NO") # please, don't disable manifest generation, otherwise crash at start for vs2008

    if( MSVC_VERSION GREATER 1500 AND ${CMAKE_VERSION} VERSION_GREATER "2.8.6")
      include(ProcessorCount)
      ProcessorCount(N)
      if(NOT N EQUAL 0)
        SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   /MP${N} ")
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP${N} ")
      endif()
    endif()
  endif()
endif()

if(CMAKE_COMPILER_IS_PATHSCALE)
  if("${CMAKE_CXX_FLAGS}" STREQUAL "")
    SET(CMAKE_CXX_FLAGS "-Wno-uninitialized -zerouv -pthread -mp")
  endif()
  if("${CMAKE_SHARED_LINKER_FLAGS}" STREQUAL "")
    SET(CMAKE_SHARED_LINKER_FLAGS "-mp")
  endif()
endif()

if(CMAKE_COMPILER_IS_CLANG)
  if("${CMAKE_C_FLAGS}" STREQUAL "")
    SET(CMAKE_C_FLAGS "-Qunused-arguments")
  endif()
  if("${CMAKE_CXX_FLAGS}" STREQUAL "")
    SET(CMAKE_CXX_FLAGS "-ftemplate-depth=1024 -Qunused-arguments -Wno-invalid-offsetof ${SSE_FLAGS}") # Unfortunately older Clang versions do not have this: -Wno-unnamed-type-template-args
    if(APPLE AND WITH_CUDA AND CUDA_FOUND)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")
    endif()
  endif()
  SET(CLANG_LIBRARIES "stdc++")
endif()

include("${PCL_SOURCE_DIR}/cmake/pcl_utils.cmake")
set(PCL_VERSION "1.8.1" CACHE STRING "PCL version")
DISSECT_VERSION()
GET_OS_INFO()
SET_INSTALL_DIRS()

if(WIN32)
  set(PCL_RESOURCES_DIR "${PCL_SOURCE_DIR}/resources")
  set(PCL_POINTCLOUDS_DIR "${PCL_RESOURCES_DIR}/pointclouds")
endif(WIN32)

set(PCL_OUTPUT_LIB_DIR "${PCL_BINARY_DIR}/${LIB_INSTALL_DIR}")
set(PCL_OUTPUT_BIN_DIR "${PCL_BINARY_DIR}/${BIN_INSTALL_DIR}")
make_directory("${PCL_OUTPUT_LIB_DIR}")
make_directory("${PCL_OUTPUT_BIN_DIR}")
if(WIN32)
  foreach(config ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${config} CONFIG)
  endforeach(config)
else(WIN32)
endif(WIN32)


###
# this is copy paste form http://www.itk.org/Wiki/CMake_RPATH_handling
# in order to always make a full statement RPATH
###
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}" is_system_dir)
IF("${is_system_dir}" STREQUAL "-1")
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
ENDIF("${is_system_dir}" STREQUAL "-1")

include_directories(SYSTEM ${EIGEN_DIR})

# FLANN (required)
if(NOT PCL_SHARED_LIBS OR ((WIN32 AND NOT MINGW) AND NOT PCL_BUILD_WITH_FLANN_DYNAMIC_LINKING_WIN32))
  set(FLANN_USE_STATIC ON)
endif(NOT PCL_SHARED_LIBS OR ((WIN32 AND NOT MINGW) AND NOT PCL_BUILD_WITH_FLANN_DYNAMIC_LINKING_WIN32))

# Boost (required)
find_package(Boost REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

### ---[ Create the config.h file
set(pcl_config_h_in "${CMAKE_CURRENT_SOURCE_DIR}/pcl_config.h.in")
set(pcl_config_h "${CMAKE_BINARY_DIR}/include/pcl/pcl_config.h")
configure_file("${pcl_config_h_in}" "${pcl_config_h}")
PCL_ADD_INCLUDES(common "" "${pcl_config_h}")
include_directories("${CMAKE_BINARY_DIR}/include")

### ---[ Add the libraries subdirectories
include("${PCL_SOURCE_DIR}/cmake/pcl_targets.cmake")

collect_subproject_directory_names("${PCL_SOURCE_DIR}" "CMakeLists.txt" PCL_MODULES_NAMES PCL_MODULES_DIRS)
set(PCL_MODULES_NAMES_UNSORTED ${PCL_MODULES_NAMES})
topological_sort(PCL_MODULES_NAMES PCL_ _DEPENDS)
sort_relative(PCL_MODULES_NAMES_UNSORTED PCL_MODULES_NAMES PCL_MODULES_DIRS)
foreach(subdir ${PCL_MODULES_DIRS})
  add_subdirectory("${PCL_SOURCE_DIR}/${subdir}")
endforeach(subdir)

### ---[ Documentation

### ---[ Configure PCLConfig.cmake
include("${PCL_SOURCE_DIR}/cmake/pcl_pclconfig.cmake")

### ---[ Package creation
include("${PCL_SOURCE_DIR}/cmake/pcl_all_in_one_installer.cmake")
include("${PCL_SOURCE_DIR}/cmake/pcl_cpack.cmake")

if(CPACK_GENERATOR)
  message(STATUS "Found CPack generators: ${CPACK_GENERATOR}")
  PCL_MAKE_CPACK_INPUT()
  set(CPACK_PROJECT_CONFIG_FILE "${PCL_CPACK_CFG_FILE}")
  include(CPack)
endif(CPACK_GENERATOR)
### ---[ Make a pretty picture of the dependency graph
include("${PCL_SOURCE_DIR}/cmake/dep_graph.cmake")
MAKE_DEP_GRAPH()

### ---[ Finish up
#PCL_WRITE_STATUS_REPORT()
PCL_RESET_MAPS()
