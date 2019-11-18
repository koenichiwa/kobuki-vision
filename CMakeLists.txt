cmake_minimum_required(VERSION 2.8.3)
project(vision)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(OpenCV_INCLUDE_DIRS $ENV{HOME}/installation/OpenCV-4.1.1/lib/cmake/opencv4)
find_package(catkin REQUIRED COMPONENTS darknet_ros roslaunch roscpp std_msgs cv_bridge image_transport pcl_msgs message_generation)


#Dit moet in aparte calls for some reason
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS)
find_package(darknet_ros REQUIRED COMPONENTS)


set(DTO_NODE_SRC src/dto_node/dto_node.cpp)
set(PCL_CONV_NODE_SRC src/pcl_conv_node/pcl_conv_node.cpp)
set(MIDDLEWARE_NODE_SRC src/middleware_node/middleware_node.cpp)


add_message_files(FILES ObjectPosition.msg)
generate_messages(
        DEPENDENCIES
        std_msgs
)
catkin_package(CATKIN_DEPENDS sensor_msgs std_msgs message_runtime)


#Exectable aanmaken
add_executable(dto_node dto_node/ ${DTO_NODE_SRC})
add_executable(pcl_conv_node pcl_conv_node/ ${PCL_CONV_NODE_SRC})
add_executable(middleware_node middleware_node/ ${MIDDLEWARE_NODE_SRC})



include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${darknet_ros_INCLUDE_DIRS})


#dit moet ook..
target_link_libraries(dto_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcl_conv_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(middleware_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})




