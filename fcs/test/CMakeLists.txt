CMAKE_MINIMUM_REQUIRED(VERSION 2.8.7 FATAL_ERROR)
PROJECT(unittest)

# Enable ExternalProject CMake module
INCLUDE(ExternalProject)


set(CMAKE_CXX_FLAGS "-O3 -mssse3 -oaddmul")
set(CMAKE_C_FLAGS "-O3 -mssse3 -oaddmul -Weverything -Wno-documentation -Wno-padded -Wno-packed -Wno-float-equal -Wno-unknown-pragmas -Werror")

# For ASAN/UBSAN
# set(CMAKE_C_COMPILER "/usr/local/bin/clang")
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address -fsanitize=undefined -O0 -mssse3 -oaddmul -Weverything -Wno-documentation -Wno-padded -Wno-packed -Wno-float-equal -Wno-unknown-pragmas -Werror")
# set(CMAKE_LDFLAGS "${LDFLAGS} -L/usr/local/lib/llvm-3.4/usr/lib")


# Set default ExternalProject root directory
SET_DIRECTORY_PROPERTIES(PROPERTIES EP_PREFIX .)

# Add googletest
ExternalProject_Add(googletest
SVN_REPOSITORY http://googletest.googlecode.com/svn/trunk/
TIMEOUT 30
# Disable install step
INSTALL_COMMAND ""
UPDATE_COMMAND ""
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
  UPDATE_COMMAND ""
)

# Add TRICAL library
ExternalProject_Add(TRICAL
  GIT_REPOSITORY https://github.com/sfwa/TRICAL.git
  INSTALL_COMMAND ""
  UPDATE_COMMAND ""
)

# Add NMPC library
#ExternalProject_Add(nmpc
#  GIT_REPOSITORY git+ssh://git@github.com/sfwa/nmpc.git
#  INSTALL_COMMAND ""
#  UPDATE_COMMAND ""
#)

INCLUDE_DIRECTORIES(${googletest_dir}/include ../)

# Add test executable target
ADD_EXECUTABLE(unittest
    ../stats/stats.c
    ../util/cobsr.c
    ../util/crc32.c
    test_util.cpp
    ../drivers/stream.c
    test_stream.cpp
    ../ahrs/wmm.c
    test_wmm.cpp
    ../ahrs/ahrs.c
    test_ahrs.cpp
    ../exports/log.c
    ../exports/calibration.c
    ../exports/exports.c
    ../exports/parameter.c
    test_exports.cpp
    test_log.cpp
    test_parameter.cpp
    ../control/control.c
    ../control/dubins.c
    ../control/figure-eight.c
    ../control/linear.c
    ../control/trajectory.c
    test_control.cpp
    ../hardware/platform/cpuv1-ioboardv1.c
    test_hardware.cpp)

# Create dependency of test on googletest, ukf, TRICAL, and nmpc
ADD_DEPENDENCIES(unittest googletest)
ADD_DEPENDENCIES(unittest ukf)
ADD_DEPENDENCIES(unittest TRICAL)
#ADD_DEPENDENCIES(unittest nmpc)

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

# Find NMPC library build location
#ExternalProject_Get_Property(nmpc binary_dir)
#TARGET_LINK_LIBRARIES(unittest
#    ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}fcsnmpc.a)

TARGET_LINK_LIBRARIES(unittest
    ${binary_dir}/../../../nmpc_build/ccs-c66x/libfcsnmpc.a)

ADD_TEST(unittest unittest)
ADD_CUSTOM_TARGET(check COMMAND ${CMAKE_CTEST_COMMAND} DEPENDS unittest)

# Add a dynamic library target for the test driver
ADD_LIBRARY(fcs SHARED
    ../stats/stats.c
    ../util/cobsr.c
    ../util/crc32.c
    ../control/control.c
    ../drivers/stream.c
    ../hardware/platform/cpuv1-ioboardv1.c
    ../ahrs/wmm.c
    ../ahrs/ahrs.c
    ../control/control.c
    ../control/trajectory.c
    ../control/dubins.c
    ../control/figure-eight.c
    ../control/linear.c
    ../exports/log.c
    ../exports/calibration.c
    ../exports/exports.c
    ../exports/parameter.c)

ADD_DEPENDENCIES(fcs ukf)
ADD_DEPENDENCIES(fcs TRICAL)
#ADD_DEPENDENCIES(fcs nmpc)

ExternalProject_Get_Property(ukf binary_dir)
TARGET_LINK_LIBRARIES(fcs
    ${binary_dir}/ccs-c66x/${CMAKE_FIND_LIBRARY_PREFIXES}fcsukf.a)

ExternalProject_Get_Property(TRICAL binary_dir)
TARGET_LINK_LIBRARIES(fcs
    ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}TRICALstatic.a)

#ExternalProject_Get_Property(nmpc binary_dir)
#TARGET_LINK_LIBRARIES(fcs
#    ${binary_dir}/ccs-c66x/${CMAKE_FIND_LIBRARY_PREFIXES}fcsnmpc.a)

TARGET_LINK_LIBRARIES(fcs
    ${binary_dir}/../../../nmpc_build/ccs-c66x/libfcsnmpc.a)

TARGET_LINK_LIBRARIES(fcs /usr/local/lib/clang/3.3/lib/darwin/libclang_rt.asan_osx.a)
TARGET_LINK_LIBRARIES(fcs /usr/local/lib/clang/3.3/lib/darwin/libclang_rt.ubsan_osx.a)
