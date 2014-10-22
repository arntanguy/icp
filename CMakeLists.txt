cmake_minimum_required(VERSION 2.8)

# Options. Turn on with 'cmake -Dmyvarname=ON'.
option(test "Build all tests." OFF) # Makes boolean 'test' available.

# Make PROJECT_SOURCE_DIR, PROJECT_BINARY_DIR, and PROJECT_NAME available.
set(PROJECT_NAME icp)
project(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

# If you want your own include/ directory, set this, and then you can do
# include_directories(${COMMON_INCLUDES}) in other CMakeLists.txt files.
# set(COMMON_INCLUDES ${PROJECT_SOURCE_DIR}/include)
find_package(Eigen3 REQUIRED)

add_subdirectory("lib/sophus-0.9a")
# Sophus only has header files
find_package(Sophus REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io visualization)
add_definitions(${PCL_DEFINITIONS})


set(LIBRARIES
  ${EIGEN3_LIBRARY}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
  glog
)


include_directories (
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Sophus_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories (
  ${PCL_LIBRARY_DIRS}
)


################################
# Normal Libraries & Executables
################################
# Add all cpp files but main.cpp to the library
file(GLOB SOURCES src/*.cpp)
list(REMOVE_ITEM SOURCES "src/main.cpp")
MESSAGE(STATUS "Compiling icp library from the following sources:\n\t ${SOURCES}")
# Create and link library
add_library("${PROJECT_NAME}_lib" ${SOURCES})
target_link_libraries("${PROJECT_NAME}_lib" ${LIBRARIES})

add_executable(${PROJECT_NAME} src/main.cpp)
# Key idea: SEPARATE OUT your main() function into its own file so it can be its
# own executable. Separating out main() means you can add this library to be
# used elsewhere.
target_link_libraries(${PROJECT_NAME} "${PROJECT_NAME}_lib" glog)

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
  target_link_libraries("${PROJECT_NAME}_test" "${PROJECT_NAME}_lib")

  # You can also omit NAME and COMMAND. The second argument could be some other
  # test executable.
  add_test(test-all "${PROJECT_NAME}_test")
endif()




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