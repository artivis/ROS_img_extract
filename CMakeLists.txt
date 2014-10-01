cmake_minimum_required(VERSION 2.8.3)
project(ros_img_extractor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  tf
)

find_package( OpenCV REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs image_transport cv_bridge tf
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

## Declare a cpp library
add_library(${PROJECT_NAME}
   src/image_extractor.cpp
)
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES} 
)

## Declare a cpp executable

add_executable(img_extractor src/bag_img_extractor.cpp)
target_link_libraries(img_extractor
   ${catkin_LIBRARIES} ${PROJECT_NAME}
)

add_executable(img_extractor_pose src/bag_img_pose_extractor.cpp)
target_link_libraries(img_extractor_pose
   ${catkin_LIBRARIES} ${PROJECT_NAME}
)

add_executable(img_sync_extractor src/bag_img_sync_extractor.cpp)
target_link_libraries(img_sync_extractor
   ${catkin_LIBRARIES} ${PROJECT_NAME}
)

add_executable(img_sync_extractor_pose src/bag_img_sync_pose_extractor.cpp)
target_link_libraries(img_sync_extractor_pose
   ${catkin_LIBRARIES} ${PROJECT_NAME}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS img_extractor img_extractor_pose img_sync_extractor img_sync_extractor_pose ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
