# Kobuki Vision

## Description
Kobuki Vision is a submodule of the [Kobuki Project](https://github.com/HvA-Robotics/kobuki-central),
made by 4th year Techinal Computing students at the [AUAS](https://www.amsterdamuas.com/). The project is 
focused on object detection using a [stereo vision camera](https://orbbec3d.com/product-astra-pro/). 

## Getting it running

Use the init script from the root directory!
```Bash
./init.sh
```
That should be it.

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

