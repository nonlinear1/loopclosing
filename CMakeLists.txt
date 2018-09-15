cmake_minimum_required(VERSION 2.8.3)
project(loam_velodyne)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  ndt_omp
  rviz)
add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})

find_package(Qt5 REQUIRED Core Widgets)#added
set(QT_LIBRARIES Qt5::Widgets)#added

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
        #${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS})
add_message_files(
  FILES
  VectorOdometry.msg
  FloorCoeffs.msg
  # Message2.msg
)
add_service_files(
  FILES
  SaveMap.srv
  GlobalMap.srv
)
generate_messages(
 DEPENDENCIES
 nav_msgs
)
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime ndt_omp
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
  LIBRARIES loam
)

#add_definitions( -march=native )

add_subdirectory(src/lib)

add_executable(multiScanRegistration src/multi_scan_registration_node.cpp)
target_link_libraries(multiScanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(laserOdometry src/laser_odometry_node.cpp)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(laserMapping src/laser_mapping_node.cpp)
target_link_libraries(laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(transformMaintenance src/transform_maintenance_node.cpp)
target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(floorDetection src/floor_detection_node.cpp)
target_link_libraries(floorDetection ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(laserSubMapping src/laser_submapping_node.cpp)
target_link_libraries(laserSubMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(backendOptim src/backend_optimization_node.cpp)
target_link_libraries(backendOptim
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    loam)

#if (CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  # TODO: Download test data
#  catkin_download_test_data(${PROJECT_NAME}_test_data.tar.gz
#    https://dl.dropboxusercontent.com/s/y4hn486461tfmpm/velodyne_loam_test_data.tar.gz
#    MD5 3d5194e6981975588b7a93caebf79ba4)
#  add_custom_target(${PROJECT_NAME}_test_data
#    COMMAND ${CMAKE_COMMAND} -E tar -xzf velodyne_loam_test_data.tar.gz
#    DEPENDS ${PROJECT_NAME}_test_data.tar.gz)
#  configure_file(tests/loam.test.in
#                 ${PROJECT_BINARY_DIR}/test/loam.test)
#  add_rostest(${PROJECT_BINARY_DIR}/test/loam.test
#    DEPENDENCIES
#      ${PROJECT_NAME}_test_data
#      multiScanRegistration
#      laserOdometry
#      laserMapping
#      transformMaintenance)
#endif()


