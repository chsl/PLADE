cmake_minimum_required(VERSION 3.12)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

find_package(Boost REQUIRED)
message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")

add_executable(${PROJECT_NAME}
		plade.h
		plade.cpp
        util.h
		util.cpp
		plane_extraction.h
		plane_extraction.cpp
		main.cpp
		)

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Executables")

target_include_directories(${PROJECT_NAME} PRIVATE
		${Boost_INCLUDE_DIRS}
		${EIGEN_DIR}
		${PLADE_THIRD_PARTY}
		${PCL_DIR}/common/include
		${PCL_DIR}/io/include
		${PCL_DIR}/features/include
		${PCL_DIR}/search/include
		${PCL_DIR}/kdtree/include
		${PCL_DIR}/filters/include
		${PCL_DIR}/registration/include
		${PCL_DIR}/segmentation/include
		${PLADE_THIRD_PARTY}/ann_1.1.2/include
		${PLADE_THIRD_PARTY}/ransac
		${PLADE_THIRD_PARTY}/opencv/include
		${PLADE_THIRD_PARTY}/opencv/modules/core/include
		${PLADE_THIRD_PARTY}/opencv/modules/imgproc/include
		${CMAKE_BINARY_DIR}/include
	)


# The resources directory
target_compile_definitions(
		${PROJECT_NAME}
		PRIVATE
		"PLADE_CODE_DIR=\"${PLADE_ROOT}\""
)

target_link_libraries( ${PROJECT_NAME} ann ransac opencv_core zlib opencv_imgproc pcl_common pcl_io pcl_kdtree pcl_filters)

# request Easy3D
find_package(Easy3D QUIET)
if (Easy3D_FOUND)
	message(STATUS "Found Easy3D v${Easy3D_VERSION}")
	target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_EASY3D)
	target_link_libraries( ${PROJECT_NAME} easy3d::viewer)
endif ()