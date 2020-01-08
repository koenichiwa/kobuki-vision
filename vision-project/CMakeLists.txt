cmake_minimum_required(VERSION 2.8.3)
project(vision)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(OpenCV_INCLUDE_DIRS $ENV{HOME}/installation/OpenCV-4.1.1/lib/cmake/opencv4)
find_package(catkin REQUIRED COMPONENTS darknet_ros roslaunch roscpp rospy std_msgs cv_bridge image_transport pcl_msgs message_generation genmsg)


#Dit moet in aparte calls for some reason
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS)
find_package(darknet_ros REQUIRED COMPONENTS)


set(PCL_CONV_NODE_SRC src/pcl_conv_node/pcl_conv_node.cpp)
set(MIDDLEWARE_NODE_SRC src/middleware_node/middleware_node.cpp)
set(PCL_UTIL_SERVICE_SRC src/pcl_util_service/pcl_util_service.cpp)


#add_message_files(FILES ObjectPosition.msg)
add_service_files(FILES PclDistance.srv PclContainsNaN.srv ObjectPosition.srv)
generate_messages(
        DEPENDENCIES
        std_msgs
)
catkin_package(CATKIN_DEPENDS sensor_msgs std_msgs message_runtime)

#Exectable aanmaken
add_executable(pcl_conv_node pcl_conv_node/ ${PCL_CONV_NODE_SRC})
add_executable(middleware_node middleware_node/ ${MIDDLEWARE_NODE_SRC})
add_executable(pcl_util_service pcl_util_service/ ${PCL_UTIL_SERVICE_SRC})


include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${darknet_ros_INCLUDE_DIRS})

#dit moet ook..
target_link_libraries(pcl_conv_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(middleware_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_util_service ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})


add_dependencies(pcl_conv_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(middleware_node ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pcl_util_service ${PROJECT_NAME}_generate_messages_cpp)




