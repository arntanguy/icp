################################
# Normal Libraries & Executables
################################
# Add all cpp files but main.cpp to the library
#file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/icp/*.cpp)
set(SOURCES
error.cpp
error_point_to_plane.cpp
error_point_to_plane_sim3.cpp
error_point_to_point_sim3.cpp
error_point_to_point.cpp
error_point_to_point_so3.cpp
error_point_to_plane_so3.cpp
constraints.cpp
icp.cpp
mestimator.cpp
)

MESSAGE(STATUS "Compiling icp library from the following sources:\n\t ${SOURCES}")
if(SOURCES)
  # Create and link library
  add_library("${ICP_LIB_NAME}" SHARED ${SOURCES})
  link_directories("/usr/lib")
  target_link_libraries("${ICP_LIB_NAME}" ${LIBRARIES})
endif()

#################################
# INSTALL
################################
install (TARGETS ${ICP_LIB_NAME} DESTINATION lib)
 
