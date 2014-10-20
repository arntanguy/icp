cmake_minimum_required(VERSION 2.8)

# Options. Turn on with 'cmake -Dmyvarname=ON'.
option(test "Build all tests." OFF) # Makes boolean 'test' available.

# Make PROJECT_SOURCE_DIR, PROJECT_BINARY_DIR, and PROJECT_NAME available.
set(PROJECT_NAME MyProject)
project(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-g -Wall")

# If you want your own include/ directory, set this, and then you can do
# include_directories(${COMMON_INCLUDES}) in other CMakeLists.txt files.
# set(COMMON_INCLUDES ${PROJECT_SOURCE_DIR}/include)

################################
# Normal Libraries & Executables
################################
add_library("${PROJECT_NAME}_lib" "src/project1.cpp")
file(GLOB SOURCES src/*.cpp)
list(REMOVE_ITEM SOURCES "src/main.cpp")

include_directories ("include")
add_executable(${PROJECT_NAME} src/main.cpp)
# Key idea: SEPARATE OUT your main() function into its own file so it can be its
# own executable. Separating out main() means you can add this library to be
# used elsewhere.

target_link_libraries(${PROJECT_NAME} "${PROJECT_NAME}_lib")

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
  target_link_libraries("${PROJECT_NAME}_test" gtest gtest_main)

  # Extra linking for the project.
  target_link_libraries("${PROJECT_NAME}_test" "${PROJECT_NAME}_lib")

  # This is so you can do 'make test' to see all your tests run, instead of
  # manually running the executable "${PROJECT_NAME}_test" to see those specific tests.
  add_test(NAME that-test-I-made COMMAND "${PROJECT_NAME}_test")

  # You can also omit NAME and COMMAND. The second argument could be some other
  # test executable.
  add_test(that-other-test-I-made "${PROJECT_NAME}_test")
endif()
