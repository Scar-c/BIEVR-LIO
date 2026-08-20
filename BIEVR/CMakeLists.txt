cmake_minimum_required(VERSION 3.16)
project(bievr_lio VERSION 0.0.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Build configuration
# ---------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Build a shared library by default (matches the previous catkin behaviour).
option(BUILD_SHARED_LIBS "Build bievr_lio as a shared library" ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Dependencies
#
# Ceres / glog / Eigen are currently provided by the *_catkin source packages
# in the workspace devel/install space, so they are found via CMAKE_PREFIX_PATH
# when the workspace is sourced. TBB is found system-wide.
# ---------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(TBB REQUIRED)
find_package(glog QUIET)  # usually re-exported transitively by Ceres

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME}
  src/bievr_map.cpp
  src/common.cpp
  src/pipeline.cpp
  src/imu_integrator.cpp
  src/inertial_factor.cpp
  src/intensity_processor.cpp
  src/intensity_sampling.cpp
  src/ls_optimizer.cpp
  src/preprocess.cpp
  src/prior_factor.cpp
  src/synchronizer.cpp
  src/timing.cpp
  src/undistort.cpp
  src/utils.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# C++20 with concepts. GCC < 10 spells C++20 as c++2a and needs -fconcepts.
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)
target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<CXX_COMPILER_ID:GNU>:-fconcepts>
  $<$<CONFIG:Release>:-O3>
)

# Route all logging through log++ (include/bievr_lio/log++.h) in LPP mode. PUBLIC
# so consumers that include our headers compile log++ in the same mode.
target_compile_definitions(${PROJECT_NAME} PUBLIC MODE_LPP)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  Ceres::ceres
  TBB::tbb
)
if(glog_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC glog::glog)
endif()

# ---------------------------------------------------------------------------
# Install & export
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

set(BIEVR_LIO_HAS_GLOG ${glog_FOUND})
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Unit tests (COIN-BIEVR reproduction plan Sections 43-48)
# ---------------------------------------------------------------------------
option(BIEVR_BUILD_TESTS "Build BIEVR unit tests" OFF)
if(BIEVR_BUILD_TESTS)
  enable_testing()
  file(GLOB BIEVR_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)
  foreach(src ${BIEVR_TEST_SOURCES})
    get_filename_component(test_name ${src} NAME_WE)
    add_executable(${test_name} ${src})
    target_link_libraries(${test_name} PRIVATE ${PROJECT_NAME}::${PROJECT_NAME})
    set_target_properties(${test_name} PROPERTIES
      CXX_STANDARD 20
      CXX_STANDARD_REQUIRED ON
      CXX_EXTENSIONS OFF
    )
    target_compile_options(${test_name} PRIVATE $<$<CXX_COMPILER_ID:GNU>:-fconcepts>)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
