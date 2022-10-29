cmake_minimum_required(VERSION 3.12)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

find_package(Boost COMPONENTS thread REQUIRED)
message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")
#message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")    
link_directories(${Boost_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}
		plade.h
		plade.cpp
        util.h
		util.cpp
		plane_extraction.h
		plane_extraction.cpp
		ply_reader.h
		ply_reader.cpp
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
		${PLADE_THIRD_PARTY}/rply
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


target_link_libraries( ${PROJECT_NAME} ann ransac rply opencv_core opencv_imgproc pcl_common pcl_kdtree pcl_filters)
target_link_libraries( ${PROJECT_NAME} Boost::thread)