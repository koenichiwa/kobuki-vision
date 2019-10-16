cmake_minimum_required(VERSION 2.8.3)
project(vision)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(OpenCV_INCLUDE_DIRS $ENV{HOME}/installation/OpenCV-4.1.1/lib/cmake/opencv4)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs cv_bridge image_transport pcl_msgs)

#Dit moet in aparte calls for some reason
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS)


set(CONV_NODE_SRC src/img_conv_node/img_conv_node.cpp)
set(CAMERA_NODE_SRC src/camera_node/camera_node.cpp)
set(DTO_NODE_SRC src/dto_node/dto_node.cpp)

#Exectable aanmaken
add_executable(camera_node camera__node/ ${CAMERA_NODE_SRC})
add_executable(img_conv_node img_conv_node/ ${CONV_NODE_SRC})
add_executable(dto_node dto_node/ ${DTO_NODE_SRC})


include_directories(${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS})


#dit moet ook..
target_link_libraries(img_conv_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
