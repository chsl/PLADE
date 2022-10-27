set(SUBSYS_NAME io)
set(SUBSYS_DESC "Point cloud IO library")
set(SUBSYS_DEPS common)

set(build TRUE)
PCL_SUBSYS_OPTION(build "${SUBSYS_NAME}" "${SUBSYS_DESC}" ON)
if(WIN32)
    PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} OPT_DEPS openni openni2 ensenso davidSDK dssdk rssdk pcap png vtk)
else(WIN32)
    PCL_SUBSYS_DEPEND(build "${SUBSYS_NAME}" DEPS ${SUBSYS_DEPS} OPT_DEPS openni openni2 ensenso davidSDK dssdk pcap png vtk libusb-1.0)
endif(WIN32)

PCL_ADD_DOC("${SUBSYS_NAME}")

if(build)
     set(PLY_SOURCES src/ply/ply_parser.cpp)
    set(PLY_INCLUDES
        "include/pcl/${SUBSYS_NAME}/ply/byte_order.h"
        "include/pcl/${SUBSYS_NAME}/ply/io_operators.h"
        "include/pcl/${SUBSYS_NAME}/ply/ply.h"
        "include/pcl/${SUBSYS_NAME}/ply/ply_parser.h"
       )
    PCL_ADD_LIBRARY(pcl_io_ply "${SUBSYS_NAME}" ${PLY_SOURCES} ${PLY_INCLUDES})
	set_target_properties(pcl_io_ply PROPERTIES FOLDER "3rd_party")
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/ply" ${PLY_INCLUDES})
    target_include_directories(pcl_io_ply PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

    set(srcs
        src/pcd_io.cpp
        src/ply_io.cpp
        src/ascii_io.cpp
        src/file_io.cpp
        src/auto_io.cpp
        src/io_exception.cpp
        )

    set(incs
        "include/pcl/${SUBSYS_NAME}/boost.h"
        "include/pcl/${SUBSYS_NAME}/eigen.h"
        "include/pcl/${SUBSYS_NAME}/debayer.h"
        "include/pcl/${SUBSYS_NAME}/file_io.h"
        "include/pcl/${SUBSYS_NAME}/auto_io.h"
        "include/pcl/${SUBSYS_NAME}/lzf.h"
        "include/pcl/${SUBSYS_NAME}/lzf_image_io.h"
        "include/pcl/${SUBSYS_NAME}/io.h"
        "include/pcl/${SUBSYS_NAME}/grabber.h"
        "include/pcl/${SUBSYS_NAME}/file_grabber.h"
        "include/pcl/${SUBSYS_NAME}/pcd_grabber.h"
        "include/pcl/${SUBSYS_NAME}/pcd_io.h"
        "include/pcl/${SUBSYS_NAME}/vtk_io.h"
        "include/pcl/${SUBSYS_NAME}/ply_io.h"
        "include/pcl/${SUBSYS_NAME}/tar.h"
        "include/pcl/${SUBSYS_NAME}/obj_io.h"
        "include/pcl/${SUBSYS_NAME}/ascii_io.h"
        "include/pcl/${SUBSYS_NAME}/ifs_io.h"
        "include/pcl/${SUBSYS_NAME}/image_grabber.h"
        "include/pcl/${SUBSYS_NAME}/hdl_grabber.h"
        "include/pcl/${SUBSYS_NAME}/vlp_grabber.h"
        "include/pcl/${SUBSYS_NAME}/robot_eye_grabber.h"
        "include/pcl/${SUBSYS_NAME}/point_cloud_image_extractors.h"
        "include/pcl/${SUBSYS_NAME}/io_exception.h"
        "include/pcl/${SUBSYS_NAME}/pxc_grabber.h"  # contains only depreciation note
        )

    set(compression_incs
        include/pcl/compression/color_coding.h
        include/pcl/compression/compression_profiles.h
        include/pcl/compression/entropy_range_coder.h
        include/pcl/compression/point_coding.h
       )
    if(PNG_FOUND)
      list(APPEND compression_incs
          include/pcl/compression/organized_pointcloud_conversion.h
          include/pcl/compression/libpng_wrapper.h
          )
      if(WITH_OPENNI OR WITH_OPENNI2)
        list(APPEND compression_incs
          include/pcl/compression/organized_pointcloud_compression.h
          )
      endif()
    endif(PNG_FOUND)

    set(impl_incs
        "include/pcl/${SUBSYS_NAME}/impl/ascii_io.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/pcd_io.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/auto_io.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/lzf_image_io.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/synchronized_queue.hpp"
        "include/pcl/${SUBSYS_NAME}/impl/point_cloud_image_extractors.hpp"
        include/pcl/compression/impl/entropy_range_coder.hpp
        ${VTK_IO_INCLUDES_IMPL}
       )
    if(PNG_FOUND AND (WITH_OPENNI OR WITH_OPENNI2))
     list(APPEND impl_incs
         include/pcl/compression/impl/organized_pointcloud_compression.hpp
        )
    endif()

    set(LIB_NAME "pcl_${SUBSYS_NAME}")

    add_definitions(${VTK_DEFINES})
    PCL_ADD_LIBRARY("${LIB_NAME}" "${SUBSYS_NAME}" ${srcs} ${incs} ${compression_incs} ${impl_incs} ${OPENNI_INCLUDES} ${OPENNI2_INCLUDES})
	set_target_properties(${LIB_NAME} PROPERTIES FOLDER "3rd_party")
    target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" ${VTK_INCLUDE_DIRECTORIES})
    target_link_libraries("${LIB_NAME}" pcl_common pcl_io_ply ${VTK_LIBRARIES} )

    set(EXT_DEPS eigen3)

    PCL_MAKE_PKGCONFIG("${LIB_NAME}" "${SUBSYS_NAME}" "${SUBSYS_DESC}"
      "${SUBSYS_DEPS}" "${EXT_DEPS}" "" "" "")

    # Install include files
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}" ${incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" compression ${compression_incs})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/openni_camera" ${OPENNI_INCLUDES})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/openni2" ${OPENNI2_INCLUDES})
    PCL_ADD_INCLUDES("${SUBSYS_NAME}" "${SUBSYS_NAME}/impl" ${impl_incs})

	set_target_properties(${LIB_NAME} PROPERTIES FOLDER "3rd_party")
endif(build)
