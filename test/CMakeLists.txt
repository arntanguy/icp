set(TEST_SOURCES
test_main.cpp
test_eigentools.cpp
test_error.cpp
test_icp_common.cpp
test_maximum_absolute_deviation.cpp
test_pcltools.cpp
)

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
target_link_libraries("${PROJECT_NAME}_test" ${LIBRARIES})
target_link_libraries("${PROJECT_NAME}_test" ${ICP_LIB_NAME})

# You can also omit NAME and COMMAND. The second argument could be some other
# test executable.
add_test(NAME test-all
              WORKING_DIRECTORY ${CMAKE_CURRENT_DIRECTORY}
              COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_test)
