# Kobuki Vision

## Description
Kobuki Vision is a bla bla bla...

## Getting it running

Install all required ROS dependencies for this project. 
 - [Orbbec Astra ROS](http://wiki.ros.org/astra_camera)
 - [Darknet ROS](https://github.com/leggedrobotics/darknet_ros)
 
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

### Changing params

In the launch file it is possible to adjust three darknet params. `threshold`, `opencv_view` and `config file` can be 
adjusted in the `launch/vision.launch` file. 

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

