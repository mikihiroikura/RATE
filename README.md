# Harris corner detector with Time slice of events + Haste tracker

This repo is for event-based continuous corner detection and tracking.

### Before starting to run
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
## How to run
1st terminal (in docker container)
```
roslaunch haste_ros haste_fd_timeslice_sae.launch event_topic:=/dvs/events camera_size:=240x180
```
2nd terminal (in host)
```
rosbag play boxes_6dof.bag
```
- Use rosbag file from scaramuzza's dataset (https://rpg.ifi.uzh.ch/davis_data.html)
- Resolution of event camera in this dataset is 240x180.
