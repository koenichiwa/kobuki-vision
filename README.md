# Kobuki Vision

## Description
Kobuki Vision is a submodule of the [Kobuki Project](https://github.com/HvA-Robotics/kobuki-central),
made by 4th year Techinal Computing students at the [AUAS](https://www.amsterdamuas.com/). The project is 
focused on object detection using a [stereo vision camera](https://orbbec3d.com/product-astra-pro/). 

## Getting it running
Before you can run this project, there are a few steps to be taken. 

### Step 1
Install all required ROS dependencies for this project. 
 - [Orbbec Astra ROS](http://wiki.ros.org/astra_camera)
 - [Darknet ROS](https://github.com/leggedrobotics/darknet_ros)
 
 ### Step 2
 Configure **Darknet ROS**
 
 1. Change all thresholds to from 0.3 (default) to 0.7. This will reduce the number of false positives.
 
    ```
    cd darknet_ros/darknet_ros/config/
    ```
    ``` 
    nano y*.yaml
    ```
    change: *threshold: value: 0.3* to *threshold: value: 0.7*
    
    A threshold of 0.7 will force Yolo to filter out all detections which have a certainty below 70%.
    
 2. Auto resize the images in darknet.
    ```Bash
    cd darknet_ros/darknet_ros/yolo_network_config/cfg/
    ```
    ```Bash
    nano *.cfg
    ```
    Under *[net]*, change both width and height to **208** (half the default width and heigth). This will 
    lead to slightly less accuracy, but will lead to ten times the normal fps (when running on a processor).

## Testing it
Starting Kobuki Vision is very easy. There is a launch file which automatically launches
the required dependencies. Running the following single launch file is all it takes to run this project.
```Bash
roscore
```
```Bash
roslaunch vision vision.launch
```
This will start Darknet ROS, Astra ROS and Kobuki Vision. 

### Setting the object to detect
To decide what Kobuki Vision needs to detect you can run the following command
```Bash
rostopic pub /speech/detect std_msgs/String "data: '${object}'" 
```
where **${object}** is the object you'd like to detect. See all available objects [here](https://github.com/leggedrobotics/darknet_ros/blob/master/darknet_ros/config/yolov2.yaml).

### Listening to detections
When Kobuki Vision detects a given object to detect, it will publish it on a specific topic.
To listen to this topic, run:
```Bash
rostopic echo /vision/object_position
```
Note: when you have **not** set any object to detect, Kobuki Vision won't publish anything. 

