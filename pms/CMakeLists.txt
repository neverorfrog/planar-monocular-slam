cmake_minimum_required(VERSION 3.20...3.27)

# ============================= PROJECT SETUP ===============================================
project(pms)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PMS_CPP_LIB pms_cpp)
set(PMS_BINDING_LIB ${PROJECT_NAME})

add_library(cxx_setup INTERFACE)
target_compile_options(cxx_setup INTERFACE -Wall -Wpedantic -Wextra -fpermissive)
target_compile_features(cxx_setup INTERFACE cxx_std_20)

if(CMAKE_CXX_COMPILER_LOADED)
  message(STATUS "Path: ${CMAKE_CXX_COMPILER}")
  message(STATUS "Compiler ID: ${CMAKE_CXX_COMPILER_ID}")
  message(STATUS "Compiler Version: ${CMAKE_CXX_COMPILER_VERSION}")
endif()

# =============================================================================================

# ============================ BUILD OPTIONS ==================================================
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions("DEBUG")
endif()

message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# =============================================================================================

# ============================ EXTERNAL LIBRARIES =============================================
include(FetchContent)

# EIGEN3
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
FetchContent_MakeAvailable(eigen)

# NANOBIND
FetchContent_Declare(
  nanobind
  GIT_REPOSITORY https://github.com/wjakob/nanobind.git
  GIT_TAG v2.7.0
)
FetchContent_MakeAvailable(nanobind)

# GOOGLE TEST (only if tests are enabled)
option(BUILD_TESTS "Build the tests" ON)
if(BUILD_TESTS)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

# =============================================================================================

# ============================ PLANAR-MONOCULAR-SLAM LIBRARY ==================================
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
file(GLOB_RECURSE PMS_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(REMOVE_ITEM PMS_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/bindings.cpp)
add_library(${PMS_CPP_LIB} STATIC ${PMS_SRCS})

target_include_directories(${PMS_CPP_LIB}
  PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${eigen_SOURCE_DIR}>
)

set(PMS_CPP_EXTRA_LIBRARIES
  $<BUILD_INTERFACE:cxx_setup>
  $<BUILD_INTERFACE:Eigen3::Eigen>
)

target_link_libraries(${PMS_CPP_LIB} PUBLIC ${PMS_CPP_EXTRA_LIBRARIES})

# =============================================================================================

# ============================ PYTHON BINDINGS ===================================================
find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)
message(STATUS "Python_EXECUTABLE: ${Python_EXECUTABLE}")
message(STATUS "Python_VERSION: ${Python_VERSION}")
message(STATUS "PYTHON INCLUDE DIRECTORIES" ${Python_INCLUDE_DIRS})

execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT
)

execute_process(
  COMMAND ${Python_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_path('purelib'))"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

nanobind_add_module(${PMS_BINDING_LIB} src/bindings.cpp)
target_include_directories(${PMS_BINDING_LIB} PUBLIC ${eigen_SOURCE_DIR})
target_link_libraries(${PMS_BINDING_LIB} PUBLIC ${PMS_CPP_LIB})

# find_package(nanoeigenpy CONFIG REQUIRED)
install(
  TARGETS ${PMS_BINDING_LIB}
  EXPORT ${PMS_BINDING_LIB}Targets
  LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${BINDIR}
)

# =============================================================================================

# ================================= TEST =====================================================
if(BUILD_TESTS)
  add_subdirectory(test)
endif()

# =============================================================================================
