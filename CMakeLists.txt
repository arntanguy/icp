cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# Options. Turn on with 'cmake -Dmyvarname=ON'.
option(ENABLE_TESTS "Build all unit tests." OFF) 
option(ENABLE_EXAMPLES "Build all example binaries." OFF) 
option(ENABLE_GLOG "Build all example binaries." ON) 

# Make PROJECT_SOURCE_DIR, PROJECT_BINARY_DIR, and PROJECT_NAME available.
set(PROJECT_NAME icp)
project(${PROJECT_NAME})

set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )

set(ICP_MAJOR_VERSION 0)
set(ICP_MINOR_VERSION 1)
set(ICP_PATCH_VERSION 0)
set(ICP_VERSION
  ${ICP_MAJOR_VERSION}.${ICP_MINOR_VERSION}.${ICP_PATCH_VERSION})

set(PROJECT_BINARY_DIR bin)
set(PROJECT_SOURCE_DIR src)
set(PROJECT_INCLUDE_DIR include/icp)
set(SOPHUS_INCLUDE_DIR include/Sophus/sophus)

set(ICP_LIB_NAME icp)
set(ICP_EXAMPLE_BIN icp_example)

set(INSTALL_CMAKE_DIR /usr/local/cmake)


# For YouCompleteMe and Clang compilation
set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )


# If you want your own include/ directory, set this, and then you can do
# include_directories(${COMMON_INCLUDES}) in other CMakeLists.txt files.
# set(COMMON_INCLUDES ${PROJECT_SOURCE_DIR}/include)
find_package(Eigen3 REQUIRED)

find_package(PCL 1.7.2 REQUIRED COMPONENTS common io features visualization)
add_definitions(${PCL_DEFINITIONS})

# Look for c++11 support
if(CMAKE_COMPILER_IS_GNUCXX)
   execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
   if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
        message(STATUS "C++11 activated.")
        add_definitions("-std=c++11")
   elseif(GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
        message(WARNING "C++0x activated. If you get any errors update to a compiler which fully supports C++11")
        add_definitions("-std=gnu++0x")
   else ()
        message(FATAL_ERROR "C++11 needed. Therefore a gcc compiler with a version higher than 4.3 is needed.")   
   endif()
else(CMAKE_COMPILER_IS_GNUCXX)
   add_definitions("-std=c++0x") 
endif(CMAKE_COMPILER_IS_GNUCXX)

add_definitions(-g -Wall -Wno-comment -Wno-unused-local-typedefs)


set(LIBRARIES
  ${EIGEN3_LIBRARY}
  ${PCL_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_FEATURES_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
  )

if(ENABLE_GLOG)
  add_definitions(-DGLOG_ENABLED)
  set(LIBRARIES ${LIBRARIES} glog)
endif()


include_directories (
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
  )

link_directories (
  ${PCL_LIBRARY_DIRS}
  )

add_subdirectory(src)

################################
# Testing
################################
if (ENABLE_TESTS)
  enable_testing()
  # This adds another subdirectory, which has 'project(gtest)'.
  add_subdirectory(lib/gtest-1.6.0)
  add_subdirectory(test)
endif()

#################################
# INSTALL
################################
configure_file(cmake/FindICP.cmake.in cmake/FindICP.cmake @ONLY)
install (FILES ${CMAKE_BINARY_DIR}/cmake/FindICP.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/cmake)
install (DIRECTORY "${PROJECT_INCLUDE_DIR}" DESTINATION include)

#################################
# Doxygen
################################
# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  add_custom_target(doc
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif(DOXYGEN_FOUND)
