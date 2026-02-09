cmake_minimum_required(VERSION 3.5)
project(vop2el_ros1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  diagnostic_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  image_transport
  cv_bridge
  message_filters
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs diagnostic_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs image_transport cv_bridge message_filters
)

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
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

target_link_libraries(vop2el_core
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${CERES_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_executable(vop2el_node src/vop2el_node.cpp)

target_include_directories(vop2el_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../vop2el_src/vop2el_lib/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../vop2el_src/test
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

target_link_libraries(vop2el_node
  vop2el_core
  ${catkin_LIBRARIES}
)

install(TARGETS vop2el_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
