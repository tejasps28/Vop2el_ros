cmake_minimum_required(VERSION 3.8)
project(vop2el_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Keep runtime performance predictable when users run plain `colcon build`.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)

set(VOP2EL_CORE_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/../vop2el_src/vop2el_lib/src/Vop2elAlgorithm.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../vop2el_src/vop2el_lib/src/StereoImagesHandler.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../vop2el_src/vop2el_lib/src/Vop2elMatcher.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../vop2el_src/vop2el_lib/src/PatchCorrector.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../vop2el_src/vop2el_lib/src/Common.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../vop2el_src/test/Utils.cpp
)

add_library(vop2el_core ${VOP2EL_CORE_SOURCES})
target_include_directories(vop2el_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../vop2el_src/vop2el_lib/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../vop2el_src/test
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(vop2el_core
  ${OpenCV_LIBS}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)
if(TARGET Ceres::ceres)
  target_link_libraries(vop2el_core Ceres::ceres)
else()
  target_include_directories(vop2el_core PUBLIC ${CERES_INCLUDE_DIRS})
  target_link_libraries(vop2el_core ${CERES_LIBRARIES})
endif()

add_executable(vop2el_node src/vop2el_node.cpp)
target_include_directories(vop2el_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../vop2el_src/vop2el_lib/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../vop2el_src/test
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(vop2el_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_transport
  cv_bridge
  message_filters
)
target_link_libraries(vop2el_node vop2el_core)

install(TARGETS
  vop2el_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/odom_to_tum.py
  scripts/record_debug_stats.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  my_files
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
