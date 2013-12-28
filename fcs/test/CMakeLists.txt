CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7 FATAL_ERROR)
PROJECT(unittest)

# Enable ExternalProject CMake module
INCLUDE(ExternalProject)

set(CMAKE_CXX_FLAGS "-O3 -mssse3 -oaddmul")
set(CMAKE_C_FLAGS "-O3 -mssse3 -oaddmul-Weverything -Werror -Wno-documentation -Wno-padded -Wno-unknown-pragmas")

# Set default ExternalProject root directory
SET_DIRECTORY_PROPERTIES(PROPERTIES EP_PREFIX .)

# Add googletest
ExternalProject_Add(googletest
SVN_REPOSITORY http://googletest.googlecode.com/svn/trunk/
TIMEOUT 30
# Disable install step
INSTALL_COMMAND ""
# Wrap download, configure and build steps in a script to log output
LOG_DOWNLOAD ON
LOG_CONFIGURE ON
LOG_BUILD ON)

# Specify googletest include dir
ExternalProject_Get_Property(googletest source_dir)
SET(googletest_dir ${source_dir})

# Add UKF library
ExternalProject_Add(ukf
  GIT_REPOSITORY https://github.com/sfwa/ukf.git
  BUILD_COMMAND $(MAKE) fcsukf
  INSTALL_COMMAND ""
)

# Add TRICAL library
ExternalProject_Add(TRICAL
  GIT_REPOSITORY https://github.com/sfwa/TRICAL.git
  INSTALL_COMMAND ""
)

INCLUDE_DIRECTORIES(${googletest_dir}/include ../)

# Add test executable target
ADD_EXECUTABLE(unittest
    ../stats/stats.c
    ../piksi/piksi.c
    ../util/cobsr.c
    ../util/base64.c
    ../util/crc8.c
    ../util/crc16sbp.c
    ../util/crc32.c
    ../util/util.c
    test_util.cpp
    ../comms/comms.c
    ../comms/config.c
    ../comms/gcs.c
    ../comms/state.c
    ../comms/status.c
    ../comms/waypoint.c
    ../comms/calibration.c
    test_comms.cpp
    ../drivers/stream.c
    ../drivers/peripheral.c
    test_stream.cpp
    ../ahrs/wmm.c
    test_wmm.cpp
    ../ahrs/measurement.c
    ../ahrs/ahrs.c
    test_ahrs.cpp
    ../hardware/platform/cpuv1-ioboardv1.c
    test_hardware.cpp)

# Create dependency of test on googletest, ukf and TRICAL
ADD_DEPENDENCIES(unittest googletest)
ADD_DEPENDENCIES(unittest ukf)
ADD_DEPENDENCIES(unittest TRICAL)

# Specify test's link libraries
ExternalProject_Get_Property(googletest binary_dir)
TARGET_LINK_LIBRARIES(unittest
    ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}gtest.a
    ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main.a
    pthread)

# Find UKF library build location
ExternalProject_Get_Property(ukf binary_dir)
TARGET_LINK_LIBRARIES(unittest
    ${binary_dir}/ccs-c66x/${CMAKE_FIND_LIBRARY_PREFIXES}fcsukf.a)

# Find TRICAL library build location
ExternalProject_Get_Property(TRICAL binary_dir)
TARGET_LINK_LIBRARIES(unittest
    ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}TRICALstatic.a)

ADD_TEST(unittest unittest)
ADD_CUSTOM_TARGET(check COMMAND ${CMAKE_CTEST_COMMAND} DEPENDS unittest)

# Add a dynamic library target for the test driver
ADD_LIBRARY(fcs SHARED
    ../stats/stats.c
    ../util/cobsr.c
    ../util/base64.c
    ../util/crc8.c
    ../util/crc16sbp.c
    ../util/crc32.c
    ../util/util.c
    ../nmpc/nmpc.c
    ../piksi/piksi.c
    ../drivers/stream.c
    ../drivers/peripheral.c
    ../comms/comms.c
    ../comms/config.c
    ../comms/gcs.c
    ../comms/state.c
    ../comms/status.c
    ../comms/waypoint.c
    ../comms/calibration.c
    ../hardware/platform/cpuv1-ioboardv1.c
    ../ahrs/measurement.c
    ../ahrs/wmm.c
    ../ahrs/ahrs.c)

ADD_DEPENDENCIES(fcs ukf)
ADD_DEPENDENCIES(fcs TRICAL)

ExternalProject_Get_Property(ukf binary_dir)
TARGET_LINK_LIBRARIES(fcs
    ${binary_dir}/ccs-c66x/${CMAKE_FIND_LIBRARY_PREFIXES}fcsukf.a)

ExternalProject_Get_Property(TRICAL binary_dir)
TARGET_LINK_LIBRARIES(fcs
    ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}TRICALstatic.a)
