add_executable(icp_methods icp_methods.cpp)
target_link_libraries(icp_methods ${ICP_LIB_NAME})

# add_executable(visualize_mestimators src/examples/visualize_mestimators.cpp)
# target_link_libraries(visualize_mestimators ${LIBRARIES_EXAMPLES})

add_executable(icp_step_by_step step_by_step.cpp)
target_link_libraries(icp_step_by_step ${ICP_LIB_NAME})
