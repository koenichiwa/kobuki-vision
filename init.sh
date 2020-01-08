#!/bin/bash
thresHold=0.7
imSize=208
yoloV2TinyYaml=submodules/darknet_ros/darknet_ros/config/yolov2-tiny.yaml
yoloV2TinyCFG=submodules/darknet_ros/darknet_ros/yolo_network_config/cfg/yolov2-tiny.cfg
sourceFiles() {
  source /opt/ros/kinetic/setup.bash
  source ~/catkin_ws/devel/setup.bash
}

installSubModules() {
  git submodule update --init --recursive
}

installAstraCamera() {
  sudo apt install ros-*-rgbd-launch ros-*-libuvc ros-*-libuvc-camera ros-*-libuvc-ros
  roscd astra_camera
  ./scripts/create_udev_rules
}

adaptConfiguration() {
  roscd vision
  cd ..
  if [[ -f ${yoloV2TinyYaml} ]]; then
    sed -i "s/value: 0.*/value:0.7/" ${yoloV2TinyYaml}
    echo "Yolo V2 tiny threshold altered to ${thresHold}!"
  else
    echo "${yoloV2TinyYaml} not found!"
    exit 1
  fi
  if [[ -f ${yoloV2TinyCFG} ]]; then
    sed -i "s/width=[0-9]*/width=208/" ${yoloV2TinyCFG}
    sed -i "s/height=[0-9]*/height=208/" ${yoloV2TinyCFG}
    echo "Yolo V2 tiny cfg width & height altered to ${imSize}!"
  else
    echo "${yoloV2TinyCFG} not found!"
    exit 1
  fi
}

echo "Welcome to Kobuki-vision!"
sourceFiles
echo "Going to install the submodules..."
installSubModules
echo "Going to install the astra_camera"
installAstraCamera
echo "Going to adapt configurations"
adaptConfiguration
echo "Done!"
echo "If catkin_make does not build due to darknet_ros, please use catkin_make -DCATKIN_BLACKLIST_PACKAGES='darknet_ros' instead."
