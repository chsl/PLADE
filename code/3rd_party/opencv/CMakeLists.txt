# ----------------------------------------------------------------------------
#  Root CMake file for OpenCV
#
#    From the off-tree build directory, invoke:
#      $ cmake <PATH_TO_OPENCV_ROOT>
#
# ----------------------------------------------------------------------------

# Disable in-source builds to prevent source tree corruption.
if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
")
endif()

set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS true)

# Following block can broke build in case of cross-compilng
# but CMAKE_CROSSCOMPILING variable will be set only on project(OpenCV) command
# so we will try to detect crosscompiling by presense of CMAKE_TOOLCHAIN_FILE
if(NOT CMAKE_TOOLCHAIN_FILE)
  # it _must_ go before project(OpenCV) in order to work
  if(WIN32)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")
  else()
    set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Installation Directory")
  endif()
else(NOT CMAKE_TOOLCHAIN_FILE)
  #Android: set output folder to ${CMAKE_BINARY_DIR}
  set( LIBRARY_OUTPUT_PATH_ROOT ${CMAKE_BINARY_DIR} CACHE PATH "root for library output, set this to change where android libs are compiled to" )
  # any crosscompiling
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")
endif(NOT CMAKE_TOOLCHAIN_FILE)

# --------------------------------------------------------------
# Top level OpenCV project
# --------------------------------------------------------------
if(CMAKE_GENERATOR MATCHES Xcode AND XCODE_VERSION VERSION_GREATER 4.3)
  cmake_minimum_required(VERSION 3.0)
elseif(IOS)
  cmake_minimum_required(VERSION 3.0)
else()
  cmake_minimum_required(VERSION 2.8.12.2)
endif()


# must go before the project command
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)
if(DEFINED CMAKE_BUILD_TYPE AND CMAKE_VERSION VERSION_GREATER "2.8")
  set_property( CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES} )
endif()

enable_testing()

project(OpenCV CXX C)

if(MSVC)
  set(CMAKE_USE_RELATIVE_PATHS ON CACHE INTERNAL "" FORCE)
endif()

include(cmake/OpenCVUtils.cmake)

ocv_cmake_eval(DEBUG_PRE ONCE)

ocv_clear_vars(OpenCVModules_TARGETS)

# ----------------------------------------------------------------------------
# Break in case of popular CMake configuration mistakes
# ----------------------------------------------------------------------------
if(NOT CMAKE_SIZEOF_VOID_P GREATER 0)
  message(FATAL_ERROR "CMake fails to determine the bitness of the target platform.
  Please check your CMake and compiler installation. If you are cross-compiling then ensure that your CMake toolchain file correctly sets the compiler details.")
endif()

# ----------------------------------------------------------------------------
# Detect compiler and target platform architecture
# ----------------------------------------------------------------------------
include(cmake/OpenCVDetectCXXCompiler.cmake)

# Add these standard paths to the search paths for FIND_LIBRARY
# to find libraries from these locations first
if(UNIX AND NOT ANDROID)
  if(X86_64 OR CMAKE_SIZEOF_VOID_P EQUAL 8)
    if(EXISTS /lib64)
      list(APPEND CMAKE_LIBRARY_PATH /lib64)
    else()
      list(APPEND CMAKE_LIBRARY_PATH /lib)
    endif()
    if(EXISTS /usr/lib64)
      list(APPEND CMAKE_LIBRARY_PATH /usr/lib64)
    else()
      list(APPEND CMAKE_LIBRARY_PATH /usr/lib)
    endif()
  elseif(X86 OR CMAKE_SIZEOF_VOID_P EQUAL 4)
    if(EXISTS /lib32)
      list(APPEND CMAKE_LIBRARY_PATH /lib32)
    else()
      list(APPEND CMAKE_LIBRARY_PATH /lib)
    endif()
    if(EXISTS /usr/lib32)
      list(APPEND CMAKE_LIBRARY_PATH /usr/lib32)
    else()
      list(APPEND CMAKE_LIBRARY_PATH /usr/lib)
    endif()
  endif()
endif()

# Add these standard paths to the search paths for FIND_PATH
# to find include files from these locations first
if(MINGW)
  if(EXISTS /mingw)
      list(APPEND CMAKE_INCLUDE_PATH /mingw)
  endif()
  if(EXISTS /mingw32)
      list(APPEND CMAKE_INCLUDE_PATH /mingw32)
  endif()
  if(EXISTS /mingw64)
      list(APPEND CMAKE_INCLUDE_PATH /mingw64)
  endif()
endif()

# ----------------------------------------------------------------------------
# OpenCV cmake options
# ----------------------------------------------------------------------------

# Optional 3rd party components
# ===================================================
OCV_OPTION(WITH_1394           "Include IEEE1394 support"                    ON   IF (NOT ANDROID AND NOT IOS) )
OCV_OPTION(WITH_AVFOUNDATION   "Use AVFoundation for Video I/O (iOS/Mac)"    OFF   IF APPLE)
OCV_OPTION(WITH_CARBON         "Use Carbon for UI instead of Cocoa"          OFF  IF APPLE )
OCV_OPTION(WITH_CUDA           "Include NVidia Cuda Runtime support"         OFF   IF (CMAKE_VERSION VERSION_GREATER "2.8" AND NOT IOS) )
OCV_OPTION(WITH_VTK            "Include VTK library support (and build opencv_viz module eiher)"             OFF IF (NOT ANDROID AND NOT IOS AND NOT CMAKE_CROSSCOMPILING) )
OCV_OPTION(WITH_CUFFT          "Include NVidia Cuda Fast Fourier Transform (FFT) library support"            OFF  IF (CMAKE_VERSION VERSION_GREATER "2.8" AND NOT IOS) )
OCV_OPTION(WITH_CUBLAS         "Include NVidia Cuda Basic Linear Algebra Subprograms (BLAS) library support" OFF IF (CMAKE_VERSION VERSION_GREATER "2.8" AND NOT IOS) )
OCV_OPTION(WITH_NVCUVID        "Include NVidia Video Decoding library support"                               OFF IF (CMAKE_VERSION VERSION_GREATER "2.8" AND NOT ANDROID AND NOT IOS AND NOT APPLE) )
OCV_OPTION(WITH_EIGEN          "Include Eigen2/Eigen3 support"               ON)
OCV_OPTION(WITH_VFW            "Include Video for Windows support"           OFF   IF WIN32 )
OCV_OPTION(WITH_FFMPEG         "Include FFMPEG support"                      OFF   IF (NOT ANDROID AND NOT IOS))
OCV_OPTION(WITH_GSTREAMER      "Include Gstreamer support"                   OFF   IF (UNIX AND NOT ANDROID) )
OCV_OPTION(WITH_GSTREAMER_0_10 "Enable Gstreamer 0.10 support (instead of 1.x)"   OFF )
OCV_OPTION(WITH_GTK            "Include GTK support"                         OFF   IF (UNIX AND NOT APPLE AND NOT ANDROID) )
OCV_OPTION(WITH_IMAGEIO        "ImageIO support for OS X"                    OFF  IF APPLE )
OCV_OPTION(WITH_IPP            "Include Intel IPP support"                   OFF  IF (MSVC OR X86 OR X86_64) )
OCV_OPTION(WITH_JASPER         "Include JPEG2K support"                      OFF   IF (NOT IOS) )
OCV_OPTION(WITH_JPEG           "Include JPEG support"                        OFF)
OCV_OPTION(WITH_OPENEXR        "Include ILM support via OpenEXR"             OFF   IF (NOT IOS) )
OCV_OPTION(WITH_OPENGL         "Include OpenGL support"                      OFF  IF (NOT ANDROID) )
OCV_OPTION(WITH_OPENNI         "Include OpenNI support"                      OFF  IF (NOT ANDROID AND NOT IOS) )
OCV_OPTION(WITH_PNG            "Include PNG support"                         OFF)
OCV_OPTION(WITH_PVAPI          "Include Prosilica GigE support"              OFF   IF (NOT ANDROID AND NOT IOS) )
OCV_OPTION(WITH_GIGEAPI        "Include Smartek GigE support"                OFF   IF (NOT ANDROID AND NOT IOS) )
OCV_OPTION(WITH_QT             "Build with Qt Backend support"               OFF  IF (NOT ANDROID AND NOT IOS) )
OCV_OPTION(WITH_WIN32UI        "Build with Win32 UI Backend support"         OFF   IF WIN32 )
OCV_OPTION(WITH_QUICKTIME      "Use QuickTime for Video I/O"                 OFF  IF APPLE )
OCV_OPTION(WITH_QTKIT          "Use QTKit Video I/O backend"                 OFF  IF APPLE )
OCV_OPTION(WITH_TBB            "Include Intel TBB support"                   OFF  IF (NOT IOS) )
OCV_OPTION(WITH_OPENMP         "Include OpenMP support"                      OFF)
OCV_OPTION(WITH_CSTRIPES       "Include C= support"                          OFF  IF WIN32 )
OCV_OPTION(WITH_TIFF           "Include TIFF support"                        OFF   IF (NOT IOS) )
OCV_OPTION(WITH_UNICAP         "Include Unicap support (GPL)"                OFF  IF (UNIX AND NOT APPLE AND NOT ANDROID) )
OCV_OPTION(WITH_V4L            "Include Video 4 Linux support"               OFF   IF (UNIX AND NOT ANDROID) )
OCV_OPTION(WITH_LIBV4L         "Use libv4l for Video 4 Linux support"        OFF   IF (UNIX AND NOT ANDROID) )
OCV_OPTION(WITH_DSHOW          "Build HighGUI with DirectShow support"       OFF   IF (WIN32 AND NOT ARM) )
OCV_OPTION(WITH_MSMF           "Build HighGUI with Media Foundation support" OFF  IF WIN32 )
OCV_OPTION(WITH_XIMEA          "Include XIMEA cameras support"               OFF  IF (NOT ANDROID) )
OCV_OPTION(WITH_XINE           "Include Xine support (GPL)"                  OFF  IF (UNIX AND NOT APPLE AND NOT ANDROID) )
OCV_OPTION(WITH_OPENCL         "Include OpenCL Runtime support"              OFF   IF (NOT IOS) )
OCV_OPTION(WITH_OPENCLAMDFFT   "Include AMD OpenCL FFT library support"      OFF   IF (NOT ANDROID AND NOT IOS) )
OCV_OPTION(WITH_OPENCLAMDBLAS  "Include AMD OpenCL BLAS library support"     OFF   IF (NOT ANDROID AND NOT IOS) )
OCV_OPTION(WITH_INTELPERC      "Include Intel Perceptual Computing support"  OFF  IF WIN32 )

# OpenCV build components
# ===================================================
OCV_OPTION(BUILD_SHARED_LIBS        "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)" OFF )
OCV_OPTION(BUILD_opencv_apps        "Build utility applications (used for example to train classifiers)" OFF )
OCV_OPTION(BUILD_ANDROID_EXAMPLES   "Build examples for Android platform"         OFF  IF ANDROID )
OCV_OPTION(BUILD_DOCS               "Create build rules for OpenCV Documentation" OFF )
OCV_OPTION(BUILD_EXAMPLES           "Build all examples"                          OFF )
OCV_OPTION(BUILD_PACKAGE            "Enables 'make package_source' command"       OFF )
OCV_OPTION(BUILD_PERF_TESTS         "Build performance tests"                     OFF  IF (NOT IOS) )
OCV_OPTION(BUILD_TESTS              "Build accuracy & regression tests"           OFF  IF (NOT IOS) )
OCV_OPTION(BUILD_WITH_DEBUG_INFO    "Include debug info into debug libs (not MSCV only)" OFF )
OCV_OPTION(BUILD_WITH_STATIC_CRT    "Enables use of statically linked CRT for staticaly linked OpenCV" ON IF MSVC )
OCV_OPTION(BUILD_FAT_JAVA_LIB       "Create fat java wrapper containing the whole OpenCV library" OFF IF NOT BUILD_SHARED_LIBS AND CMAKE_COMPILER_IS_GNUCXX )
OCV_OPTION(BUILD_ANDROID_SERVICE    "Build OpenCV Manager for Google Play" OFF )
OCV_OPTION(BUILD_ANDROID_PACKAGE    "Build platform-specific package for Google Play" OFF IF ANDROID )
OCV_OPTION(BUILD_TINY_GPU_MODULE    "Build tiny gpu module with limited image format support" OFF )

# 3rd party libs
OCV_OPTION(BUILD_ZLIB               "Build zlib from source"            OFF )
OCV_OPTION(BUILD_TIFF               "Build libtiff from source"         OFF )
OCV_OPTION(BUILD_JASPER             "Build libjasper from source"       OFF )
OCV_OPTION(BUILD_JPEG               "Build libjpeg from source"         OFF )
OCV_OPTION(BUILD_PNG                "Build libpng from source"          OFF )
OCV_OPTION(BUILD_OPENEXR            "Build openexr from source"         OFF )
OCV_OPTION(BUILD_TBB                "Download and build TBB from source" OFF )

# OpenCV installation options
# ===================================================
OCV_OPTION(INSTALL_CREATE_DISTRIB   "Change install rules to build the distribution package" OFF )
OCV_OPTION(INSTALL_C_EXAMPLES       "Install C examples"        OFF )
OCV_OPTION(INSTALL_PYTHON_EXAMPLES  "Install Python examples"   OFF )
OCV_OPTION(INSTALL_ANDROID_EXAMPLES "Install Android examples"  OFF IF ANDROID )
OCV_OPTION(INSTALL_TO_MANGLED_PATHS "Enables mangled install paths, that help with side by side installs." OFF IF (UNIX AND NOT ANDROID AND NOT IOS AND BUILD_SHARED_LIBS) )
OCV_OPTION(INSTALL_TESTS            "Install accuracy and performance test binaries and test data" OFF)

# OpenCV build options
# ===================================================
OCV_OPTION(ENABLE_CCACHE              "Use ccache"                                               (UNIX AND NOT IOS AND (CMAKE_GENERATOR MATCHES "Makefile" OR CMAKE_GENERATOR MATCHES "Ninja")) )
OCV_OPTION(ENABLE_DYNAMIC_CUDA        "Enabled dynamic CUDA linkage"                             OFF  )
OCV_OPTION(ENABLE_PRECOMPILED_HEADERS "Use precompiled headers"                                  OFF   )
OCV_OPTION(ENABLE_SOLUTION_FOLDERS    "Solution folder in Visual Studio or in other IDEs"        (MSVC_IDE OR CMAKE_GENERATOR MATCHES Xcode) IF (CMAKE_VERSION VERSION_GREATER "2.8.0") )
OCV_OPTION(ENABLE_PROFILING           "Enable profiling in the GCC compiler (Add flags: -g -pg)" OFF  IF CMAKE_COMPILER_IS_GNUCXX )
OCV_OPTION(ENABLE_COVERAGE            "Enable coverage collection with  GCov"                    OFF  IF CMAKE_COMPILER_IS_GNUCXX )
OCV_OPTION(ENABLE_OMIT_FRAME_POINTER  "Enable -fomit-frame-pointer for GCC"                      ON   IF CMAKE_COMPILER_IS_GNUCXX AND NOT (APPLE AND CMAKE_COMPILER_IS_CLANGCXX) )
OCV_OPTION(ENABLE_POWERPC             "Enable PowerPC for GCC"                                   ON   IF (CMAKE_COMPILER_IS_GNUCXX AND CMAKE_SYSTEM_PROCESSOR MATCHES powerpc.*) )
OCV_OPTION(ENABLE_FAST_MATH           "Enable -ffast-math (not recommended for GCC 4.6.x)"       OFF  IF (CMAKE_COMPILER_IS_GNUCXX AND (X86 OR X86_64)) )
OCV_OPTION(ENABLE_SSE                 "Enable SSE instructions"                                  ON   IF ((MSVC OR CMAKE_COMPILER_IS_GNUCXX) AND (X86 OR X86_64)) )
OCV_OPTION(ENABLE_SSE2                "Enable SSE2 instructions"                                 ON   IF ((MSVC OR CMAKE_COMPILER_IS_GNUCXX) AND (X86 OR X86_64)) )
OCV_OPTION(ENABLE_SSE3                "Enable SSE3 instructions"                                 ON   IF ((CV_ICC OR CMAKE_COMPILER_IS_GNUCXX) AND (X86 OR X86_64)) )
OCV_OPTION(ENABLE_SSSE3               "Enable SSSE3 instructions"                                OFF  IF (CMAKE_COMPILER_IS_GNUCXX AND (X86 OR X86_64)) )
OCV_OPTION(ENABLE_SSE41               "Enable SSE4.1 instructions"                               OFF  IF ((CV_ICC OR CMAKE_COMPILER_IS_GNUCXX) AND (X86 OR X86_64)) )
OCV_OPTION(ENABLE_SSE42               "Enable SSE4.2 instructions"                               OFF  IF (CMAKE_COMPILER_IS_GNUCXX AND (X86 OR X86_64)) )
OCV_OPTION(ENABLE_AVX                 "Enable AVX instructions"                                  OFF  IF ((MSVC OR CMAKE_COMPILER_IS_GNUCXX) AND (X86 OR X86_64)) )
OCV_OPTION(ENABLE_AVX2                "Enable AVX2 instructions"                                 OFF  IF ((MSVC OR CMAKE_COMPILER_IS_GNUCXX) AND (X86 OR X86_64)) )
OCV_OPTION(ENABLE_NEON                "Enable NEON instructions"                                 AARCH64 IF CMAKE_COMPILER_IS_GNUCXX AND (ARM OR AARCH64))
OCV_OPTION(ENABLE_VFPV3               "Enable VFPv3-D32 instructions"                            AARCH64 IF CMAKE_COMPILER_IS_GNUCXX AND (ARM OR AARCH64))
OCV_OPTION(ENABLE_NOISY_WARNINGS      "Show all warnings even if they are too noisy"             OFF )
OCV_OPTION(OPENCV_WARNINGS_ARE_ERRORS "Treat warnings as errors"                                 OFF )
OCV_OPTION(ENABLE_WINRT_MODE          "Build with Windows Runtime support"                       OFF  IF WIN32 )
OCV_OPTION(ENABLE_WINRT_MODE_NATIVE   "Build with Windows Runtime native C++ support"            OFF  IF WIN32 )
OCV_OPTION(ENABLE_LIBVS2013           "Build VS2013 with Visual Studio 2013 libraries"           OFF  IF WIN32 AND (MSVC_VERSION EQUAL 1800) )
OCV_OPTION(ENABLE_WINSDK81            "Build VS2013 with Windows 8.1 SDK"                        OFF  IF WIN32 AND (MSVC_VERSION EQUAL 1800) )
OCV_OPTION(ENABLE_WINPHONESDK80       "Build with Windows Phone 8.0 SDK"                         OFF  IF WIN32 AND (MSVC_VERSION EQUAL 1700) )
OCV_OPTION(ENABLE_WINPHONESDK81       "Build VS2013 with Windows Phone 8.1 SDK"                  OFF  IF WIN32 AND (MSVC_VERSION EQUAL 1800) )

# uncategorized options
# ===================================================
OCV_OPTION(CMAKE_VERBOSE "Verbose mode" OFF )

# backward compatibility
# ===================================================
include(cmake/OpenCVLegacyOptions.cmake OPTIONAL)


# ----------------------------------------------------------------------------
#  Get actual OpenCV version number from sources
# ----------------------------------------------------------------------------
include(cmake/OpenCVVersion.cmake)


# ----------------------------------------------------------------------------
#  Build & install layouts
# ----------------------------------------------------------------------------

# Save libs and executables in the same place
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin" CACHE PATH "Output directory for applications" )

if (ANDROID)
  if (ANDROID_ABI MATCHES "NEON")
    set(ENABLE_NEON ON)
  endif()
  if (ANDROID_ABI MATCHES "VFPV3")
    set(ENABLE_VFPV3 ON)
  endif()
endif()

if(NOT DEFINED OPENCV_DOC_INSTALL_PATH)
  if(ANDROID OR WIN32)
    set(OPENCV_DOC_INSTALL_PATH doc)
  elseif(INSTALL_TO_MANGLED_PATHS)
    set(OPENCV_DOC_INSTALL_PATH share/OpenCV-${OPENCV_VERSION}/doc)
  else()
    set(OPENCV_DOC_INSTALL_PATH share/OpenCV/doc)
  endif()
endif()

if(WIN32)
  if(DEFINED OpenCV_RUNTIME AND DEFINED OpenCV_ARCH)
    set(OpenCV_INSTALL_BINARIES_PREFIX "${OpenCV_ARCH}/${OpenCV_RUNTIME}/")
  else()
    message(STATUS "Can't detect runtime and/or arch")
    set(OpenCV_INSTALL_BINARIES_PREFIX "")
  endif()
elseif(ANDROID)
  set(OpenCV_INSTALL_BINARIES_PREFIX "sdk/native/")
else()
  set(OpenCV_INSTALL_BINARIES_PREFIX "")
endif()

if(ANDROID)
  set(OPENCV_SAMPLES_BIN_INSTALL_PATH "${OpenCV_INSTALL_BINARIES_PREFIX}samples/${ANDROID_NDK_ABI_NAME}")
else()
  set(OPENCV_SAMPLES_BIN_INSTALL_PATH "${OpenCV_INSTALL_BINARIES_PREFIX}samples")
endif()

if(ANDROID)
  set(OPENCV_BIN_INSTALL_PATH "${OpenCV_INSTALL_BINARIES_PREFIX}bin/${ANDROID_NDK_ABI_NAME}")
else()
  set(OPENCV_BIN_INSTALL_PATH "${OpenCV_INSTALL_BINARIES_PREFIX}bin")
endif()

if(NOT OPENCV_TEST_INSTALL_PATH)
  set(OPENCV_TEST_INSTALL_PATH "${OPENCV_BIN_INSTALL_PATH}")
endif()

if(ANDROID)
  set(LIBRARY_OUTPUT_PATH         "${OpenCV_BINARY_DIR}/lib/${ANDROID_NDK_ABI_NAME}")
  set(3P_LIBRARY_OUTPUT_PATH      "${OpenCV_BINARY_DIR}/3rdparty/lib/${ANDROID_NDK_ABI_NAME}")
  set(OPENCV_LIB_INSTALL_PATH     sdk/native/libs/${ANDROID_NDK_ABI_NAME})
  set(OPENCV_3P_LIB_INSTALL_PATH  sdk/native/3rdparty/libs/${ANDROID_NDK_ABI_NAME})
  set(OPENCV_CONFIG_INSTALL_PATH  sdk/native/jni)
  set(OPENCV_INCLUDE_INSTALL_PATH sdk/native/jni/include)
  set(OPENCV_SAMPLES_SRC_INSTALL_PATH samples/native)
else()
  set(LIBRARY_OUTPUT_PATH         "${OpenCV_BINARY_DIR}/lib")
  set(3P_LIBRARY_OUTPUT_PATH      "${OpenCV_BINARY_DIR}/3rdparty/lib${LIB_SUFFIX}")
  if(WIN32)
    if(OpenCV_STATIC)
      set(OPENCV_LIB_INSTALL_PATH   "${OpenCV_INSTALL_BINARIES_PREFIX}staticlib${LIB_SUFFIX}")
    else()
      set(OPENCV_LIB_INSTALL_PATH   "${OpenCV_INSTALL_BINARIES_PREFIX}lib${LIB_SUFFIX}")
    endif()
    set(OPENCV_3P_LIB_INSTALL_PATH  "${OpenCV_INSTALL_BINARIES_PREFIX}staticlib${LIB_SUFFIX}")
    set(OPENCV_SAMPLES_SRC_INSTALL_PATH    samples/native)
  else()
    set(OPENCV_LIB_INSTALL_PATH     lib${LIB_SUFFIX})
    set(OPENCV_3P_LIB_INSTALL_PATH  share/OpenCV/3rdparty/${OPENCV_LIB_INSTALL_PATH})
    set(OPENCV_SAMPLES_SRC_INSTALL_PATH    share/OpenCV/samples)
  endif()
  set(OPENCV_INCLUDE_INSTALL_PATH "include")

  math(EXPR SIZEOF_VOID_P_BITS "8 * ${CMAKE_SIZEOF_VOID_P}")
  if(LIB_SUFFIX AND NOT SIZEOF_VOID_P_BITS EQUAL LIB_SUFFIX)
    set(OPENCV_CONFIG_INSTALL_PATH lib${LIB_SUFFIX}/cmake/opencv)
  else()
    set(OPENCV_CONFIG_INSTALL_PATH share/OpenCV)
  endif()
endif()

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${OPENCV_LIB_INSTALL_PATH}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(INSTALL_TO_MANGLED_PATHS)
  set(OPENCV_INCLUDE_INSTALL_PATH ${OPENCV_INCLUDE_INSTALL_PATH}/opencv-${OPENCV_VERSION})
endif()

if(WIN32)
  # Postfix of DLLs:
  set(OPENCV_DLLVERSION "${OPENCV_VERSION_MAJOR}${OPENCV_VERSION_MINOR}${OPENCV_VERSION_PATCH}")
  set(OPENCV_DEBUG_POSTFIX d)
else()
  # Postfix of so's:
  set(OPENCV_DLLVERSION "")
  set(OPENCV_DEBUG_POSTFIX "")
endif()

if(DEFINED CMAKE_DEBUG_POSTFIX)
  set(OPENCV_DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}")
endif()

if(CMAKE_VERBOSE)
  set(CMAKE_VERBOSE_MAKEFILE 1)
endif()


# ----------------------------------------------------------------------------
#  Path for build/platform -specific headers
# ----------------------------------------------------------------------------
set(OPENCV_CONFIG_FILE_INCLUDE_DIR "${CMAKE_BINARY_DIR}/" CACHE PATH "Where to create the platform-dependant cvconfig.h")
ocv_include_directories(${OPENCV_CONFIG_FILE_INCLUDE_DIR})

# ----------------------------------------------------------------------------
#  Path for additional modules
# ----------------------------------------------------------------------------
#set(OPENCV_EXTRA_MODULES_PATH "" CACHE PATH "Where to look for additional OpenCV modules")

# ----------------------------------------------------------------------------
#  Autodetect if we are in a GIT repository
# ----------------------------------------------------------------------------

# don't use FindGit because it requires CMake 2.8.2
set(git_names git eg) # eg = easy git
# Prefer .cmd variants on Windows unless running in a Makefile in the MSYS shell
if(CMAKE_HOST_WIN32)
  if(NOT CMAKE_GENERATOR MATCHES "MSYS")
    set(git_names git.cmd git eg.cmd eg)
  endif()
endif()

find_host_program(GIT_EXECUTABLE NAMES ${git_names} PATH_SUFFIXES Git/cmd Git/bin DOC "git command line client")
mark_as_advanced(GIT_EXECUTABLE)

if(GIT_EXECUTABLE)
  execute_process(COMMAND ${GIT_EXECUTABLE} describe --tags --always --dirty --match "2.[0-9].[0-9]*"
    WORKING_DIRECTORY "${OpenCV_SOURCE_DIR}"
    OUTPUT_VARIABLE OPENCV_VCSVERSION
    RESULT_VARIABLE GIT_RESULT
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT GIT_RESULT EQUAL 0)
    set(OPENCV_VCSVERSION "unknown")
  endif()
else()
  # We don't have git:
  set(OPENCV_VCSVERSION "unknown")
endif()


# ----------------------------------------------------------------------------
# OpenCV compiler and linker options
# ----------------------------------------------------------------------------
# In case of Makefiles if the user does not setup CMAKE_BUILD_TYPE, assume it's Release:
if(CMAKE_GENERATOR MATCHES "Makefiles|Ninja" AND "${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

include(cmake/OpenCVCompilerOptions.cmake)


# ----------------------------------------------------------------------------
# Use statically or dynamically linked CRT?
# Default: dynamic
# ----------------------------------------------------------------------------
if(MSVC)
  include(cmake/OpenCVCRTLinkage.cmake)
endif(MSVC)

if(WIN32 AND NOT MINGW)
  add_definitions(-D_VARIADIC_MAX=10)
endif(WIN32 AND NOT MINGW)


# ----------------------------------------------------------------------------
#       CHECK FOR SYSTEM LIBRARIES, OPTIONS, ETC..
# ----------------------------------------------------------------------------
if(UNIX)
  if(NOT APPLE)
    CHECK_INCLUDE_FILE(pthread.h HAVE_LIBPTHREAD)
    if(ANDROID)
      set(OPENCV_LINKER_LIBS ${OPENCV_LINKER_LIBS} dl m log)
    elseif(${CMAKE_SYSTEM_NAME} MATCHES "FreeBSD|NetBSD|DragonFly")
      set(OPENCV_LINKER_LIBS ${OPENCV_LINKER_LIBS} m pthread)
    elseif(${CMAKE_SYSTEM_NAME} MATCHES "QNX")
      set(OPENCV_LINKER_LIBS ${OPENCV_LINKER_LIBS} m)
    else()
      set(OPENCV_LINKER_LIBS ${OPENCV_LINKER_LIBS} dl m pthread rt)
    endif()
  else()
    set(HAVE_LIBPTHREAD YES)
  endif()
endif()

include(cmake/OpenCVPCHSupport.cmake)
include(cmake/OpenCVModule.cmake)

# ----------------------------------------------------------------------------
#  Detect endianness of build platform
# ----------------------------------------------------------------------------

if(CMAKE_SYSTEM_NAME STREQUAL iOS)
  # test_big_endian needs try_compile, which doesn't work for iOS
  # http://public.kitware.com/Bug/view.php?id=12288
  set(WORDS_BIGENDIAN 0)
else()
  include(TestBigEndian)
  test_big_endian(WORDS_BIGENDIAN)
endif()

# ----------------------------------------------------------------------------
#  Detect other 3rd-party libraries/tools
# ----------------------------------------------------------------------------

# --- LATEX for pdf documentation ---
unset(HAVE_DOXYGEN CACHE)
if(BUILD_DOCS)
  include(cmake/OpenCVFindLATEX.cmake)
  find_host_program(DOXYGEN_BUILD doxygen)
  if (DOXYGEN_BUILD)
    set(HAVE_DOXYGEN 1)
  endif (DOXYGEN_BUILD)
endif(BUILD_DOCS)

# --- Python Support ---
include(cmake/OpenCVDetectPython.cmake)

# --- Java Support ---
include(cmake/OpenCVDetectApacheAnt.cmake)
if(ANDROID)
  include(cmake/OpenCVDetectAndroidSDK.cmake)

  if(NOT ANDROID_TOOLS_Pkg_Revision GREATER 13)
    message(WARNING "OpenCV requires Android SDK tools revision 14 or newer. Otherwise tests and samples will no be compiled.")
  endif()
else()
 # find_package(JNI)
endif()

if(ANDROID AND ANDROID_EXECUTABLE AND ANT_EXECUTABLE AND (ANT_VERSION VERSION_GREATER 1.7) AND (ANDROID_TOOLS_Pkg_Revision GREATER 13))
  SET(CAN_BUILD_ANDROID_PROJECTS TRUE)
else()
  SET(CAN_BUILD_ANDROID_PROJECTS FALSE)
endif()

# --- OpenCL ---
if(WITH_OPENCL)
  include(cmake/OpenCVDetectOpenCL.cmake)
endif()

# --- VTK support ---
include(cmake/OpenCVDetectVTK.cmake)

# ----------------------------------------------------------------------------
# Add CUDA libraries (needed for apps/tools, samples)
# ----------------------------------------------------------------------------
if(NOT HAVE_CUDA)
  set(ENABLE_DYNAMIC_CUDA OFF)
endif()

if(HAVE_CUDA AND NOT ENABLE_DYNAMIC_CUDA)
  set(OPENCV_LINKER_LIBS ${OPENCV_LINKER_LIBS} ${CUDA_LIBRARIES} ${CUDA_npp_LIBRARY})
  if(HAVE_CUBLAS)
    set(OPENCV_LINKER_LIBS ${OPENCV_LINKER_LIBS} ${CUDA_cublas_LIBRARY})
  endif()
  if(HAVE_CUFFT)
    set(OPENCV_LINKER_LIBS ${OPENCV_LINKER_LIBS} ${CUDA_cufft_LIBRARY})
  endif()
endif()

# ----------------------------------------------------------------------------
# Solution folders:
# ----------------------------------------------------------------------------
if(ENABLE_SOLUTION_FOLDERS)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
  set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")
endif()

# Extra OpenCV targets: uninstall, package_source, perf, etc.
#include(cmake/OpenCVExtraTargets.cmake)


# ----------------------------------------------------------------------------
# Process subdirectories
# ----------------------------------------------------------------------------

# opencv.hpp and legacy headers
add_subdirectory(include)

# OpenCV modules
add_subdirectory(modules)


# ----------------------------------------------------------------------------
# Finalization: generate configuration-based files
# ----------------------------------------------------------------------------
ocv_track_build_dependencies()

# Generate platform-dependent and configuration-dependent headers
include(cmake/OpenCVGenHeaders.cmake)

# Generate opencv.pc for pkg-config command
#include(cmake/OpenCVGenPkgconfig.cmake)

# Generate OpenCV.mk for ndk-build (Android build tool)
#include(cmake/OpenCVGenAndroidMK.cmake)

# Generate OpenCVСonfig.cmake and OpenCVConfig-version.cmake for cmake projects
#include(cmake/OpenCVGenConfig.cmake)

# Generate Info.plist for the IOS framework
#include(cmake/OpenCVGenInfoPlist.cmake)

# Generate environment setup file
if(INSTALL_TESTS AND OPENCV_TEST_DATA_PATH)
  if(ANDROID)
    get_filename_component(TEST_PATH ${OPENCV_TEST_INSTALL_PATH} DIRECTORY)
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/opencv_run_all_tests_android.sh.in"
                   "${CMAKE_BINARY_DIR}/unix-install/opencv_run_all_tests.sh" @ONLY)
    install(PROGRAMS "${CMAKE_BINARY_DIR}/unix-install/opencv_run_all_tests.sh"
            DESTINATION . COMPONENT tests)
  elseif(WIN32 OR UNIX)
    set(OPENCV_PYTHON_TESTS_LIST "")
    if(BUILD_opencv_python)
      file(GLOB py_tests modules/python/test/*.py)
      install(PROGRAMS ${py_tests} DESTINATION ${OPENCV_TEST_INSTALL_PATH} COMPONENT tests)
      if(BUILD_opencv_nonfree)
        file(GLOB py_nonfree_tests modules/python/test/nonfree_tests/*.py)
        install(PROGRAMS ${py_nonfree_tests} DESTINATION ${OPENCV_TEST_INSTALL_PATH}/nonfree_tests COMPONENT tests)
      endif()
      set(OPENCV_PYTHON_TESTS_LIST "test.py")
    endif()
    if(WIN32)
        configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/opencv_run_all_tests_windows.cmd.in"
                       "${CMAKE_BINARY_DIR}/win-install/opencv_run_all_tests.cmd" @ONLY)
        install(PROGRAMS "${CMAKE_BINARY_DIR}/win-install/opencv_run_all_tests.cmd"
                DESTINATION ${OPENCV_TEST_INSTALL_PATH} COMPONENT tests)
    else()
        configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/opencv_run_all_tests_unix.sh.in"
                       "${CMAKE_BINARY_DIR}/unix-install/opencv_run_all_tests.sh" @ONLY)
        install(PROGRAMS "${CMAKE_BINARY_DIR}/unix-install/opencv_run_all_tests.sh"
                DESTINATION ${OPENCV_TEST_INSTALL_PATH} COMPONENT tests)
    endif()
  endif()
endif()

if(NOT OPENCV_README_FILE)
  if(ANDROID)
    set(OPENCV_README_FILE ${CMAKE_CURRENT_SOURCE_DIR}/platforms/android/README.android)
  endif()
endif()

if(NOT OPENCV_LICENSE_FILE)
  set(OPENCV_LICENSE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE)
endif()

# for UNIX it does not make sense as LICENSE and readme will be part of the package automatically
if(ANDROID OR NOT UNIX)
  install(FILES ${OPENCV_LICENSE_FILE}
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
        DESTINATION . COMPONENT libs)
  if(OPENCV_README_FILE)
    install(FILES ${OPENCV_README_FILE}
            PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
            DESTINATION . COMPONENT libs)
  endif()
endif()
