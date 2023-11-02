FROM osrf/ros:melodic-desktop-full
RUN apt update && apt install -y git wget
ENV DISPLAY host.docker.internal:0.0
## python-catkin-tools
#### ROS setup ####
RUN mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src && \
    # /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_init_workspace" && \
    cd ~/catkin_ws
    # && \
    # catkin build
    # && \
    # source devel/setup.bash && \
    # # /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make" && \
    # echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
	# echo "source /home/root/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
	# echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/home/root/catkin_ws" >> ~/.bashrc && \
	# echo "export ROS_WORKSPACE=/home/root/catkin_ws" >> ~/.bashrc

#### Install dependencies ####
RUN apt-get install -y python-catkin-tools
# catkin_simple
RUN cd ~/catkin_ws/src && \
    git clone https://github.com/catkin/catkin_simple.git
# build libcaer library
RUN mkdir ~/src && cd ~/src && \
    git clone https://gitlab.com/inivation/dv/libcaer.git && cd libcaer && \
    apt-get install -y build-essential cmake pkg-config libusb-1.0-0-dev && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr . && \
    make && make install
# build ceres
RUN apt-get install -y libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libeigen3-dev && \
    cd ~/src && wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz && \
    tar zxf ceres-solver-2.1.0.tar.gz && \
    mkdir -p ~/src/ceres-solver-2.1.0/build && cd ~/src/ceres-solver-2.1.0/build && \
    cmake .. && make && make install

#### Copy source code ####
COPY ./ /root/catkin_ws/src