# RATE: Real-time Asynchronous Feature Tracking with Event Cameras

This repo is for event-based continuous corner detection and tracking described in the following paper:

Mikihiro Ikura, Cedric Le Gentil, Marcus G. Müller, Florian Schuler, Atsushi Yamashita and Wolfgang Stürzl: "RATE: Real-time Asynchronous Feature Tracking with Event Cameras", Proceedings of the 2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS2024), Abu Dhabi (UAE), October 2024.

## ros packages lists
- event_array_msgs
  - Definitions for ROS messages created by SilkyEvCam
- gen_seeds
  - Publish seeds from text file to haste
- haste_ros
  - Subscribe seeds and event_msgs
  - Publish tracking results
- my_events
  - Subscribe event msgs
  - Publish detection results by Harris corner detector and Time slice of events
- rpg_dvs_ros
  - ros packages from scaramuzza's lab for dvs sensors
  - Use dvs_renderer to visualize events as images
## Requirements
- Ceres
- libcaer

## Before starting to run
### Update submodule
```
git submodule update --init --recursive
```
### Build docker image
```
docker build -t rate:latest .
```

### Run docker container
```
docker run -it --privileged --net=host --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /path_to_rosbag_data_in_host/:/app/rosbag --name rate rate:latest
```
or
```
docker compose up -d
docker exec -it rate /bin/bash
```
- Mount a directory where rosbag files are saved 
### Build catkin_ws (in docker terminal)
```
cd ~/catkin_ws
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```
### Open X server (host in new terminal)
```
xhost local:docker
```
## How to run RATE, continuous feature detection and tracking with event rosbag data
1st terminal (in docker container)
```
roslaunch haste_ros haste_fd_timeslice_sae.launch event_topic:=/dvs/events camera_size:=240x180 camera_calib:=/root/catkin_ws/src/haste_ros/haste/dataset/calib.txt
```
2nd terminal (in host if you have ros environment locally / otherwise in docker container)
```
rosbag play boxes_6dof.bag
```
- Use rosbag file from scaramuzza's dataset (https://rpg.ifi.uzh.ch/davis_data.html)
- Resolution of event camera in this dataset is 240x180.
