cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# Options. Turn on with 'cmake -Dmyvarname=ON'.
option(ENABLE_TESTS "Build all unit tests." OFF) 
option(ENABLE_EXAMPLES "Build all example binaries." OFF) 
option(ENABLE_GLOG "Build all example binaries." OFF) 

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

set(INSTALL_LIB_DIR /usr/local/lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR /usr/local/bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR /usr/local/include CACHE PATH "Installation directory for includes")
set(INSTALL_CMAKE_DIR /usr/local/cmake)


# For YouCompleteMe and Clang compilation
set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )


# If you want your own include/ directory, set this, and then you can do
# include_directories(${COMMON_INCLUDES}) in other CMakeLists.txt files.
# set(COMMON_INCLUDES ${PROJECT_SOURCE_DIR}/include)
find_package(Eigen3 REQUIRED)

find_package(PCL 1.7.2)# REQUIRED COMPONENTS common io features visualization)
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

set(LIBRARIES_EXAMPLES
  ${ICP_LIB_NAME}
)

if(ENABLE_GLOG)
  add_definitions(-DGLOG_ENABLED)
  set(LIBRARIES ${LIBRARIES} glog)
  set(LIBRARIES_EXAMPLES ${LIBRARIES_EXAMPLES} glog)
endif()


include_directories (
  ${PROJECT_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include/Sophus
  )

link_directories (
  ${PCL_LIBRARY_DIRS}
  )


################################
# Normal Libraries & Executables
################################
# Add all cpp files but main.cpp to the library
#file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/icp/*.cpp)
set(SOURCES
src/icp/error.cpp
src/icp/error_point_to_plane.cpp
src/icp/error_point_to_plane_sim3.cpp
src/icp/error_point_to_point_sim3.cpp
src/icp/error_point_to_point.cpp
src/icp/constraints.cpp
src/icp/icp.cpp
src/icp/maximum_absolute_deviation.cpp
#src/icp/mestimator.cpp
#src/icp/mestimator_hubert.cpp
)

set(TEST_SOURCES
src/test/test_main.cpp
src/test/test_eigentools.cpp
src/test/test_error.cpp
src/test/test_icp_common.cpp
src/test/test_mestimator.cpp
src/test/test_pcltools.cpp
#src/test/test_constraints.cpp
#src/test/test_la.cpp
)

MESSAGE(STATUS "Compiling icp library from the following sources:\n\t ${SOURCES}")
if(SOURCES)
  # Create and link library
  add_library("${ICP_LIB_NAME}" SHARED ${SOURCES})
  link_directories("/usr/lib")
  target_link_libraries("${ICP_LIB_NAME}" ${LIBRARIES})
endif()

if(ENABLE_EXAMPLES)
  add_executable(${ICP_EXAMPLE_BIN} src/main.cpp)
  target_link_libraries(${ICP_EXAMPLE_BIN} ${LIBRARIES_EXAMPLES})
endif()

################################
# Testing
################################
if (ENABLE_TESTS)
  # This adds another subdirectory, which has 'project(gtest)'.
  add_subdirectory(lib/gtest-1.6.0)

  enable_testing()

  # Include the gtest library. gtest_SOURCE_DIR is available due to
  # 'project(gtest)' above.
  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

  ##############
  # Unit Tests
  ##############
  # file(GLOB TEST_SOURCES src/test/*.cpp)
  add_executable("${PROJECT_NAME}_test" ${TEST_SOURCES})

  # Standard linking to gtest stuff.
  target_link_libraries("${PROJECT_NAME}_test" gtest)

  # Extra linking for the project.
  if(SOURCES)
    MESSAGE(STATUS "Linking tests with ICP library")
    target_link_libraries("${PROJECT_NAME}_test" "${ICP_LIB_NAME}")
  else()
    target_link_libraries("${PROJECT_NAME}_test" ${LIBRARIES})
  endif()

  # You can also omit NAME and COMMAND. The second argument could be some other
  # test executable.
  add_test(test-all "${PROJECT_NAME}_test")
endif()


#################################
# INSTALL
################################

install (TARGETS ${ICP_LIB_NAME} DESTINATION ${INSTALL_LIB_DIR})
install (DIRECTORY "${PROJECT_INCLUDE_DIR}" DESTINATION ${INSTALL_INCLUDE_DIR})
install (DIRECTORY "${SOPHUS_INCLUDE_DIR}" DESTINATION ${INSTALL_INCLUDE_DIR})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindICP.cmake DESTINATION ${INSTALL_CMAKE_DIR})
 
# Add all targets to the build-tree export set
#export(TARGETS ${ICP_LIB_NAME} 
#  FILE "${PROJECT_BINARY_DIR}/IcpTargets.cmake")

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
