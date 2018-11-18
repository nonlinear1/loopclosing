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
  rviz
  cv_bridge
  cloud_msgs)
#add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Draco REQUIRED dracodec dracoenc draco)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${draco_INCLUDE_DIR})

find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})

find_package(Qt5 REQUIRED Core Widgets)#added
set(QT_LIBRARIES Qt5::Widgets)#added
add_definitions(-DQT_NO_KEYWORDS)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
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
  FloorCoeffs.msg
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
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime ndt_omp cv_bridge
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
  LIBRARIES loam
)
qt5_wrap_cpp(MOC_FILES
  include/rviz_plugion/mapinfoupdatesrv.h
)
#add_definitions( -march=native )
add_library(mapinfo
            src/rviz_plugion/mapinfoupdatesrv.cpp
            ${MOC_FILES})
target_link_libraries(mapinfo
    ${catkin_LIBRARIES}
    ${QT_LIBRARIES})

add_subdirectory(src/lib)


#add_executable(multiScanRegistration src/multi_scan_registration_node.cpp)
#target_link_libraries(multiScanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

#add_executable(laserOdometry src/laser_odometry_node.cpp)
#target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

#add_executable(laserMapping src/laser_mapping_node.cpp)
#target_link_libraries(laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

#add_executable(transformMaintenance src/transform_maintenance_node.cpp)
#target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(laserImage_node src/laser_image_node.cpp)
target_link_libraries(laserImage_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(imageProjection src/imageProjection.cpp)
add_dependencies(imageProjection ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(featureAssociation src/featureAssociation.cpp)
add_dependencies(featureAssociation ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(featureAssociation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(mapOptmization src/mapOptmization.cpp)
target_link_libraries(mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(transformFusion src/transformFusion.cpp)
target_link_libraries(transformFusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(floorDetection src/floor_detection_node.cpp)
#target_link_libraries(floorDetection ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

#add_executable(laserSubMapping src/laser_submapping_node.cpp)
#target_link_libraries(laserSubMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )
#add_executable(windowoptim_node src/windowoptim_node.cpp)
#target_link_libraries(windowoptim_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )
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


