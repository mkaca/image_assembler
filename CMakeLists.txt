cmake_minimum_required(VERSION 2.8.3)
project(image_assembler)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(
-std=c++14
-Wall
-Wextra
-Werror
-Wno-unused-parameter
)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    pcl_conversions
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2_ros
)
find_package(PCL REQUIRED)


###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS
    include
 CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2_ros
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# image assembler executable
add_executable(assemble_image
  nodes/assemble_image_node.cpp
  src/assemble_image.cpp
)
target_link_libraries(assemble_image
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(assemble_image
  ${catkin_EXPORTED_TARGETS}
)


# ############
# # Install ##
# ############

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS
    assemble_image
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# ############
# # Testing ##
# ############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # Todo: enable once unit tests are added
  # add_rostest(test/test_image_assembler.test ARGS coverage:=ENABLE_COVERAGE_TESTING)

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*") # applies to c++ coverage report only
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES tests
    )
  endif()
endif()
