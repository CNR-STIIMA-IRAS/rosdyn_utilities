cmake_minimum_required(VERSION 3.0.2)
project(rosdyn_chain_state)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

cnr_set_flags()

find_package(catkin REQUIRED COMPONENTS
  state_space_filters
  eigen_matrix_utils
  roscpp
  rosdyn_core
  realtime_tools
)

find_package(Eigen3 3.3 REQUIRED)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS state_space_filters roscpp rosdyn_core realtime_tools
)

###########
## Build ##
###########
include_directories(include  ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/chain_state.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen ${EIGEN3_LIBRARIES})
cnr_target_compile_options(${PROJECT_NAME})

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


#############
## Testing ##
#############
message(STATUS "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Test enabled!")
## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    message(STATUS "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ Test enabled!")
    add_rostest_gtest(${PROJECT_NAME}_test test/test.launch test/test.cpp)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${rostest_LIBRARIES} )
    cnr_target_compile_options(${PROJECT_NAME}_test)
    if(ENABLE_COVERAGE_TESTING)
      set(COVERAGE_EXCLUDES "*/test*")
      add_code_coverage(
        NAME coverage_report
        DEPENDENCIES ${PROJECT_NAME}_test
      )
    endif()
    
    # install(DIRECTORY test/
    #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    #   PATTERN ".svn" EXCLUDE)

    # install(TARGETS ${PROJECT_NAME}_test
    #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    # )
endif()

