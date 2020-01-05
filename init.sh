#!/bin/bash
thresHold=0.7
imSize=208
yoloV2TinyYaml=submodules/darknet_ros/darknet_ros/config/yolov2-tiny.yaml
yoloV2TinyCFG=submodules/darknet_ros/darknet_ros/yolo_network_config/cfg/yolov2-tiny.cfg
echo "Welcome to Kobuki-vision!"
if [[ -d submodules ]];then
	echo "Submodules found."
	if [[ -f ${yoloV2TinyYaml} ]];then
		sed -i "s/value: 0.*/value:0.7/" ${yoloV2TinyYaml}
		echo "Yolo V2 tiny threshold altered to ${thresHold}!"
	else
		echo "${yoloV2TinyYaml} not found!"
		exit -1;
	fi
	if [[ -f ${yoloV2TinyCFG} ]];then
		sed -i "s/width=[0-9]*/width=208/" ${yoloV2TinyCFG}
		sed -i "s/height=[0-9]*/height=208/" ${yoloV2TinyCFG}
		echo "Yolo V2 tiny cfg width & height altered to ${imSize}!"
	else
		echo "${yoloV2TinyCFG} not found!"
		exit -1;
	fi
	echo "Set-up done!";
else
	echo "Submodules directory does not exist! Exiting..."
	exit -1;
fi
