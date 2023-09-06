# Harris corner detector with Time slice of events + Haste tracker

This repo is for event-based continuous corner detection and tracking.

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


## Before starting to run
Download docker image
```
```
Run docker container
```
docker run -it --privileged --net=host --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /path_to_rosbag_data_in_host/:/app/rosbag --name haste-run osrf/ros:melodic-desktop-full-arcstar-haste-metavisiondriver
```
Open X server (host)
```
xhost local:
```
## How to run continuous detection and tracking
1st terminal (in docker container)
```
roslaunch haste_ros haste_fd_timeslice_sae.launch event_topic:=/dvs/events camera_size:=240x180 camera_calib:=/path_to_haste_ros/haste/dataset/calib.txt
```
2nd terminal (in host)
```
rosbag play boxes_6dof.bag
```
- Use rosbag file from scaramuzza's dataset (https://rpg.ifi.uzh.ch/davis_data.html)
- Resolution of event camera in this dataset is 240x180.

## How to run haste with seed.txt
1st terminal (in docker container)
```
roslaunch haste_ros haste_with_gen_seeds.launch camera_calib:=/path_to_haste_ros/haste/dataset/calib.txt camera_size:=240x180 event_topic:=/dvs/events seed_file:=/path_to_gen_seed/seed/shapes_rotation_corner_classification.txt
```
2nd terminal (in host)
```
rosbag play shapes_rotation.bag 
```
