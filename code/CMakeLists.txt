cmake_minimum_required(VERSION 3.12)

################################################################################

project(PLADE)

################################################################################

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

################################################################################

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

################################################################################

# Boost (required)
find_package(Boost REQUIRED)

################################################################################

### Configuration
set(PLADE_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(PLADE_THIRD_PARTY ${PLADE_ROOT}/3rd_party)

set(EIGEN_DIR ${PLADE_THIRD_PARTY}/eigen-3.4.0)
set(PCL_DIR ${PLADE_THIRD_PARTY}/pcl-1.8.1)


################################################################################

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

################################################################################

add_subdirectory(3rd_party)
add_subdirectory(PLADE)