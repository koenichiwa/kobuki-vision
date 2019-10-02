cmake_minimum_required(VERSION 2.8.3)
project(vision)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(OpenCV_DIR $ENV{HOME}/installation/OpenCV-4.1.1/lib/cmake/opencv4)
set(WS_DIR workspace/)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(OpenCV REQUIRED)
catkin_package(
        sensor_msgs
        cv_bridge
        roscpp
        std_msgs
        image_transport
)
set(SOURCE_FILES ${WS_DIR}/src/)
add_executable(${PROJECT_NAME}_node ${SOURCE_FILES})
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

#project(OpenCVTest)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(OpenCV_DIR $ENV{HOME}/installation/OpenCV-4.1.1/lib/cmake/opencv4)
#
#find_package(OpenCV REQUIRED)
#set(SOURCE_FILES main.cpp)
#add_executable(${PROJECT_NAME} ${SOURCE_FILES})
#include_directories(${OpenCV_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
