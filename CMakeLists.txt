cmake_minimum_required(VERSION 2.8)

# Options. Turn on with 'cmake -Dmyvarname=ON'.
option(test "Build all tests." OFF) # Makes boolean 'test' available.

# Make PROJECT_SOURCE_DIR, PROJECT_BINARY_DIR, and PROJECT_NAME available.
set(PROJECT_NAME icp)
project(${PROJECT_NAME})

set(ICP_MAJOR_VERSION 0)
set(ICP_MINOR_VERSION 1)
set(ICP_PATCH_VERSION 0)
set(ICP_VERSION
  ${ICP_MAJOR_VERSION}.${ICP_MINOR_VERSION}.${ICP_PATCH_VERSION})

set(PROJECT_BINARY_DIR bin)
set(PROJECT_SOURCE_DIR src)
set(PROJECT_INCLUDE_DIR include/icp)

set(ICP_LIB_NAME icp)
set(ICP_EXAMPLE_BIN icp_example)

set(INSTALL_LIB_DIR /usr/local/lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR /usr/local/bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR /usr/local/include CACHE PATH "Installation directory for includes")
set(INSTALL_CMAKE_DIR /usr/local/cmake)


set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

# For YouCompleteMe and Clang compilation
set( CMAKE_EXPORT_COMPILE_COMMANDS 1 )


# If you want your own include/ directory, set this, and then you can do
# include_directories(${COMMON_INCLUDES}) in other CMakeLists.txt files.
# set(COMMON_INCLUDES ${PROJECT_SOURCE_DIR}/include)
find_package(Eigen3 REQUIRED)

find_package(PCL 1.7.2 REQUIRED COMPONENTS common io visualization)
add_definitions(${PCL_DEFINITIONS})


set(LIBRARIES
  ${EIGEN3_LIBRARY}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
  glog
  )


include_directories (
  ${PROJECT_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  )

link_directories (
  ${PCL_LIBRARY_DIRS}
  )


################################
# Normal Libraries & Executables
################################
# Add all cpp files but main.cpp to the library
file(GLOB SOURCES src/icp/*.cpp)
MESSAGE(STATUS "Compiling icp library from the following sources:\n\t ${SOURCES}")
if(SOURCES)
  # Create and link library
  add_library("${ICP_LIB_NAME}" SHARED ${SOURCES})
  target_link_libraries("${ICP_LIB_NAME}" ${LIBRARIES})
endif()

add_executable(${ICP_EXAMPLE_BIN} src/main.cpp)
if(SOURCES)
  # Key idea: SEPARATE OUT your main() function into its own file so it can be its
  # own executable. Separating out main() means you can add this library to be
  # used elsewhere.
  MESSAGE(STATUS "Linking main program with ICP library")
  target_link_libraries(${ICP_EXAMPLE_BIN} "${ICP_LIB_NAME}" glog)
else()
  target_link_libraries(${ICP_EXAMPLE_BIN} ${LIBRARIES} glog)
endif()

################################
# Testing
################################
if (test)
  # This adds another subdirectory, which has 'project(gtest)'.
  add_subdirectory(lib/gtest-1.6.0)

  enable_testing()

  # Include the gtest library. gtest_SOURCE_DIR is available due to
  # 'project(gtest)' above.
  include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

  ##############
  # Unit Tests
  ##############
  file(GLOB TEST_SOURCES src/test/*.cpp)
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
