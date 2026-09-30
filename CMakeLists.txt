# Copyright 2026 Jeff Long
# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.28)

project(
  gr4-uhd
  VERSION 0.1.0
  DESCRIPTION "USRP source and sink blocks for GNU Radio 4, loaded as a block library"
  LANGUAGES CXX)

# A single-config generator given no build type compiles without optimization and without
# NDEBUG. The package builds Release there unless the caller names a type.
get_property(GR4UHD_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT CMAKE_BUILD_TYPE AND NOT GR4UHD_MULTI_CONFIG)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  message(STATUS "gr4-uhd: no CMAKE_BUILD_TYPE given; building Release")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# GNU Radio 4 core, its block-library macros, and UHD 4.0 or later. The blocks are written
# against UHD 4's streamer interface, property-tree paths and metadata error codes.
find_package(Threads REQUIRED)
find_package(gnuradio4 4.0 CONFIG REQUIRED)
find_package(GnuRadioBlockLib CONFIG REQUIRED)
find_package(UHD 4.0 CONFIG REQUIRED)
# uhd/CMakeLists.txt takes the library and the headers from these two entries, and a lookup of
# its own keeps an entry already set. They hold what find_package(UHD) found, so the blocks
# build against the UHD this configure reports.
set(CAPTURE_UHD_LIB ${UHD_LIBRARIES} CACHE FILEPATH "libuhd, as find_package(UHD) found it" FORCE)
set(CAPTURE_UHD_INCLUDE ${UHD_INCLUDE_DIRS} CACHE PATH "UHD's include directory, as find_package(UHD) found it" FORCE)

# The cases: one binary per subject under uhd/test/ and one ctest entry per case, each named
# on the binary's command line. The offline cases carry the label offline and ask no device.
# The hardware cases open a USRP and carry the label hardware; CAPTURE_HW_UHD_ARGS in the
# environment selects the unit. Boost.UT is one header: GR4UHD_UT_INCLUDE_DIR names the
# directory holding boost/ut.hpp.
option(GR4UHD_ENABLE_TESTING "Build the cases" ${PROJECT_IS_TOP_LEVEL})
option(GR4UHD_ENABLE_HARDWARE_TESTS "Build the cases that open a USRP (label hardware)" OFF)
set(GR4UHD_UT_INCLUDE_DIR "" CACHE PATH "Directory holding boost/ut.hpp")

# The settings common/cmake/capture.cmake and uhd/CMakeLists.txt read: the USRP directory,
# the package's names, the label of the offline cases, and the options above. The block
# library compiles both device headers, so the separate header parse is off. The package
# installs its discovery library under any top.
set(CAPTURE_DEVICES uhd)
set(CAPTURE_PACKAGE gr4-uhd)
set(CAPTURE_PACKAGE_BLOCKS GrUhdBlocks)
set(CAPTURE_TEST_LABEL offline)
set(CAPTURE_UT_VARIABLE GR4UHD_UT_INCLUDE_DIR)
set(CAPTURE_ENABLE_TESTING ${GR4UHD_ENABLE_TESTING})
set(CAPTURE_ENABLE_HEADER_PARSE OFF)
set(CAPTURE_ENABLE_DISCOVERY_LIBRARY ON)
if(GR4UHD_UT_INCLUDE_DIR)
  set(CAPTURE_UT_INCLUDE_DIR ${GR4UHD_UT_INCLUDE_DIR})
endif()
if(GR4UHD_ENABLE_HARDWARE_TESTS)
  set(CAPTURE_HW_UHD ON)
endif()
include(common/cmake/capture.cmake)
add_library(gr4-uhd::headers ALIAS capture-interface)

add_subdirectory(uhd)

# The plain discovery functions of capture/common/DiscoveryLibrary.hpp over the USRP family.
# capture_finish() also prints the capture: lines naming the block library built or the
# reason it was not.
capture_finish(gr4-uhd-discovery gr4-uhd::discovery)
if(NOT TARGET gr4-uhd-blocks)
  message(FATAL_ERROR "gr4-uhd: the block library was not built (see the capture: lines above)")
endif()
if(TARGET gr4-uhd-discovery)
  set_target_properties(gr4-uhd-discovery PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
  # The discovery library a program links, asked for the families it carries.
  if(CAPTURE_TESTS)
    capture_test(
      qa_discovery_library_uhd
      DIR uhd/test
      CASES discovery.uhd-the-linked-library-carries-the-usrp-family
      LIBS gr4-uhd-discovery)
  endif()
endif()

# ---- install ---------------------------------------------------------------------------
#
# The block library goes to the plugin directory of GNU Radio 4 under the same prefix, the
# discovery library to the library directory, and the headers under include/gr4-uhd/.
# find_package(gr4-uhd) gives gr4-uhd::blocks, gr4-uhd::discovery and gr4-uhd::headers.
install(
  TARGETS gr4-uhd-blocks
  EXPORT gr4-uhdTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/gnuradio-4/plugins)
install(TARGETS capture-interface EXPORT gr4-uhdTargets)
if(TARGET gr4-uhd-discovery)
  install(
    TARGETS gr4-uhd-discovery
    EXPORT gr4-uhdTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
install(
  DIRECTORY common/include/capture uhd/include/capture
  DESTINATION ${CAPTURE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
  PATTERN "*.hpp")
install(
  EXPORT gr4-uhdTargets
  NAMESPACE gr4-uhd::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gr4-uhd)
file(
  CONFIGURE
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/gr4-uhdConfig.cmake
  CONTENT
    [[
include(CMakeFindDependencyMacro)
find_dependency(Threads)
find_dependency(gnuradio4 CONFIG)
include("${CMAKE_CURRENT_LIST_DIR}/gr4-uhdTargets.cmake")
]]
  @ONLY)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/gr4-uhdConfigVersion.cmake
                                 COMPATIBILITY SameMinorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/gr4-uhdConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/gr4-uhdConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gr4-uhd)
