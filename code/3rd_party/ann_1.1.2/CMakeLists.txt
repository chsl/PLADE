cmake_minimum_required(VERSION 3.12)

project(ann)

set(kdtree_HEADERS
    include/ANN/ANN.h
    include/ANN/ANNperf.h
    include/ANN/ANNx.h
    )

set(kdtree_SOURCES
    src/ANN.cpp
    src/bd_fix_rad_search.cpp
    src/bd_pr_search.cpp
    src/bd_search.cpp
    src/bd_tree.cpp
    src/brute.cpp
    src/kd_dump.cpp
    src/kd_fix_rad_search.cpp
    src/kd_pr_search.cpp
    src/kd_search.cpp
    src/kd_split.cpp
    src/kd_tree.cpp
    src/kd_util.cpp
    src/perf.cpp
    )


add_library(ann STATIC ${kdtree_SOURCES} ${kdtree_HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE
        ${PLADE_THIRD_PARTY}/ann_1.1.2/include
        ${PLADE_THIRD_PARTY}/ann_1.1.2/src
        )

set_target_properties(ann PROPERTIES FOLDER "3rd_party")

if (MSVC)
    target_compile_definitions(ann PRIVATE
        _CRT_SECURE_NO_WARNINGS
        _CRT_SECURE_NO_DEPRECATE
        )
endif()