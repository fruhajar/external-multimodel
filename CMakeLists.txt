cmake_minimum_required(VERSION 3.20)

project(BallisticsMultiModel VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

# Shared by default to match how this is deployed; -DBUILD_SHARED_LIBS=OFF gives a static lib.
option(BUILD_SHARED_LIBS "Build as a shared library" ON)
option(BALLISTICS_MM_BUILD_TESTS "Build tests" OFF)
option(BALLISTICS_MM_BUILD_EXAMPLES "Build examples" OFF)

find_package(glm CONFIG REQUIRED)

configure_file(cmake/version.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/ballistics/mm/version.h @ONLY)

add_library(ballistics-multimodel
  src/atmosphere.cpp
  src/catalogue.cpp
  src/config.cpp
  src/drag.cpp
  src/driver.cpp
  src/environment.cpp
  src/forces.cpp
  src/launch.cpp
  src/model_mpmm.cpp
  src/model_pointmass.cpp
  src/model_rigidbody.cpp
  src/model_state.cpp
  src/predict.cpp
  src/projectile.cpp
  src/solve.cpp
  src/types.cpp
  src/wind.cpp
)
add_library(Ballistics::MultiModel ALIAS ballistics-multimodel)

target_include_directories(ballistics-multimodel
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# glm types appear in the public headers, so consumers need its include directories too.
target_link_libraries(ballistics-multimodel PUBLIC glm::glm)

target_compile_definitions(ballistics-multimodel PRIVATE
  GLM_FORCE_INLINE
  GLM_FORCE_LEFT_HANDED
  GLM_FORCE_SILENT_WARNINGS
)

target_compile_options(ballistics-multimodel PRIVATE
  -Wall -Wextra -Wswitch-enum -Wshadow
)

# Default visibility: the whole API is public and there are no export macros to gate it.
set_target_properties(ballistics-multimodel PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  EXPORT_NAME MultiModel
)

if(BALLISTICS_MM_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(BALLISTICS_MM_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS ballistics-multimodel
  EXPORT BallisticsMultiModelTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/ballistics/mm/version.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ballistics/mm)

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/BallisticsMultiModel)

install(EXPORT BallisticsMultiModelTargets
  FILE BallisticsMultiModelTargets.cmake
  NAMESPACE Ballistics::
  DESTINATION ${INSTALL_CONFIGDIR}
)

configure_package_config_file(cmake/BallisticsMultiModelConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/BallisticsMultiModelConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/BallisticsMultiModelConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/BallisticsMultiModelConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/BallisticsMultiModelConfigVersion.cmake
  DESTINATION ${INSTALL_CONFIGDIR}
)

# Packaging, only when this library is built on its own rather than from the combined tree.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(CPACK_PACKAGE_NAME "ballistics-multimodel")
  set(CPACK_PACKAGE_VENDOR "fruhajar")
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Trajectory prediction with interchangeable ballistic models")
  set(CPACK_PACKAGE_HOMEPAGE_URL "https://github.com/fruhajar")
  set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
  set(CPACK_RESOURCE_FILE_LICENSE ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE)

  set(CPACK_GENERATOR "DEB")
  set(CPACK_DEBIAN_PACKAGE_MAINTAINER "fruhajar <fruhajar@protonmail.com>")
  set(CPACK_DEBIAN_PACKAGE_SECTION "libs")

  # Dependency scanning needs file(1), which minimal images often lack.
  find_program(BALLISTICS_FILE_UTILITY file)
  if(BALLISTICS_FILE_UTILITY)
    set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
  endif()

  include(CPack)
endif()
